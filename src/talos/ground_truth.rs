use crate::capture::CaptureSource;
use crate::components::{Controlled, Infantry};
use crate::robomaster::prelude::{
    Activation, Armor, ArmorLabel, ArmorRoot, MechanismState, Outpost, OutpostRotationMode,
    OutpostRotator, PowerRune, PowerRuneMechanism, PowerRuneRotation, RotationMode, RuneMode, Team,
};
use crate::talos::capture::{TalosCaptureContext, TalosFrameStamp, TalosGroundTruthFrame};
use crate::talos::plugin::M_ALIGN_MAT3;
use crate::util::entity_query::HierarchyQuery;
use avian3d::prelude::AngularVelocity;
use bevy::prelude::*;
use std::collections::HashMap;
use talos_ipc::*;

fn to_ros_vec3(v: Vec3) -> Vec3 {
    M_ALIGN_MAT3 * v
}

fn team_to_u8(team: &Team) -> u8 {
    match team {
        Team::Red => 0,
        Team::Blue => 1,
    }
}

fn activation_to_u8(a: &Activation) -> u8 {
    match a {
        Activation::Deactivated => 0,
        Activation::Activating => 1,
        Activation::Activated => 2,
        Activation::Completed => 3,
    }
}

fn mechanism_state_to_u8(s: &MechanismState) -> u8 {
    match s {
        MechanismState::Inactive { .. } => 0,
        MechanismState::Activating(_) => 1,
        MechanismState::Activated { .. } => 2,
        MechanismState::Failed { .. } => 3,
    }
}

fn rune_mode_to_u8(m: &RuneMode) -> u8 {
    match m {
        RuneMode::Small => 0,
        RuneMode::Large => 1,
    }
}

/// Exact POWER.glb bullseye node statuses. LEGGING/PADDING share the TARGET_ prefix
/// but are not the 0.7 m aim point and must never enter blade geometry.
fn parse_rune_bullseye_node(face_prefix: &str, node_name: &str) -> Option<usize> {
    let rest = node_name.strip_prefix(face_prefix)?;
    let (target_id, status) = rest.split_once('_')?;
    if !matches!(status, "ACTIVATED" | "ACTIVE" | "COMPLETED" | "DISABLED") {
        return None;
    }
    let target_id = target_id.parse::<usize>().ok()?;
    (1..=5).contains(&target_id).then_some(target_id - 1)
}

fn rune_face_index(name: &str) -> Option<usize> {
    let rest = name.strip_prefix("FACE_")?;
    if rest.contains('_') {
        return None;
    }
    rest.parse().ok()
}

fn rune_blade_identity(face_index: usize, blade_id: usize) -> u16 {
    ((face_index as u16) << 8) | (blade_id as u16)
}

/// Return the current blade bullseye from POWER.glb node names.
///
/// Logical blade id is `FACE_<face>_TARGET_<1..5>` and is independent of ECS order.
/// Only ACTIVATED/ACTIVE/COMPLETED/DISABLED nodes are accepted. When `preferred_blade`
/// is set (mechanism Activating/Activated/Completed), that blade is published; otherwise
/// the lowest id with a valid bullseye is used as a diagnostic fallback.
fn rune_blade_geometry(
    rune: Entity,
    center: Vec3,
    preferred_blade: Option<usize>,
    qq: &HierarchyQuery,
    named_transforms: &Query<(Entity, &Name, &GlobalTransform)>,
) -> Option<(f32, f32, usize, Vec3, u16)> {
    let face_name = qq.name.get(rune).ok()?;
    let face_index = rune_face_index(face_name.as_str()).unwrap_or(0);
    let prefix = format!("{}_TARGET_", face_name.as_str());
    let mut by_blade: HashMap<usize, Vec3> = HashMap::new();
    for (_entity, name, tf) in named_transforms.iter() {
        let Some(blade_id) = parse_rune_bullseye_node(&prefix, name.as_str()) else {
            continue;
        };
        by_blade.entry(blade_id).or_insert(tf.translation());
    }
    if by_blade.is_empty() {
        return None;
    }
    let blade_id = preferred_blade
        .filter(|id| by_blade.contains_key(id))
        .or_else(|| by_blade.keys().copied().min())?;
    let blade = *by_blade.get(&blade_id)?;
    let radius = blade.distance(center);
    if !radius.is_finite() || radius <= 1e-4 {
        return None;
    }
    let v = blade - center;
    let current_angle = v.z.atan2(v.x);
    Some((
        radius,
        current_angle,
        blade_id,
        blade,
        rune_blade_identity(face_index, blade_id),
    ))
}

/// Compute yaw in the ROS reference frame from a Bevy GlobalTransform.
///
/// The alignment matrix maps Bevy (Y-up) → ROS (Z-up).
/// We convert the rotation quaternion through the alignment to extract the Z-up yaw.
fn ros_yaw(global_tf: &GlobalTransform) -> f32 {
    let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);
    let ros_rot = align_quat * global_tf.rotation() * align_quat.inverse();
    // `EulerRot::ZYX` 按指定轴序返回 `(z, y, x)` 三个分量。ROS yaw 是绕 Z，
    // 所以必须取第一个；第三个是绕 X 的 roll。取错时纯 yaw 转动会一直报 0，
    // 车辆与前哨站的 yaw/vyaw 评估都会静默失真。
    let (yaw, _, _) = ros_rot.to_euler(EulerRot::ZYX);
    yaw
}

/// 板位参考点的取法。两类资产的节点结构不同，不能用同一条路径。
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum PlateReference {
    /// 必须有 `*CENTER` 后代节点，取不到就跳过这块板。
    ///
    /// 步兵/英雄走这条：`assets/vehicle.glb` 里每块板都有 `<n>_ARMOR_CENTER`。
    /// 缺一个说明资产换了节点名或子树没加载完，此时"退回板根"会静默给出一个不是
    /// 板心的点，宁可少发这一块板，让消费端从 `armor_position_valid` 看出来。
    CenterNode,
    /// 没有 `*CENTER` 后代时退回板根节点自己。
    ///
    /// 前哨站走这条：`assets/OUTPOST.glb` 里 `*CENTER` 节点数是 **0**（实测遍历
    /// 全部 glTF 节点），板根 `A/B/C_ARMOR_ROOT` 直接落在板位上——相对旋转节点
    /// `OUTPOST_A_ROTATE` 的水平半径实测 0.2757 / 0.2753 / 0.2748 m，互差 <1 mm，
    /// 三块板 120° 分布。对照组：`vehicle.glb` 的 `1_ARMOR_CENTER` 局部平移是恒等，
    /// 也就是说步兵的板根与板心本来就重合，这个退化没有引入新的口径。
    CenterNodeOrPlateRoot,
}

/// A candidate plate reference in Bevy world coordinates.
#[derive(Copy, Clone)]
struct PlateCandidate {
    distance_squared: f32,
    world: Vec3,
    outpost_radius_m: f32,
}

/// The outpost has a three-plate ring: valid plate roots share one horizontal radius in the
/// current `OutpostRotator` local frame.  An asset node can be named like an armor root without
/// being one of those three plates, so names are deliberately not part of this decision.
///
/// Two agreeing roots are enough to retain a useful degraded sample when an asset constructor
/// cannot build one normal plate.  A singleton is not evidence of a ring model, so it is safer to
/// leave `armor_position_valid=0` than publish an arbitrary root as a precision reference.
const OUTPOST_RADIUS_MODEL_MIN_INLIERS: usize = 2;
const OUTPOST_RADIUS_MODEL_REL_TOLERANCE: f32 = 0.20;
const OUTPOST_RADIUS_MODEL_ABS_TOLERANCE_M: f32 = 0.02;
const OUTPOST_RADIUS_MODEL_MIN_RADIUS_M: f32 = 0.05;

fn select_outpost_plate_reference_with_support(
    candidates: &[PlateCandidate],
) -> Option<(Vec3, usize)> {
    // Fit the densest equal-radius cluster instead of trusting an asset label or an absolute
    // radius.  The intended model has three members; a two-member cluster remains admissible for
    // the known constructor-degraded case, while an isolated far-away node is rejected.
    let mut best_model: Option<(usize, f32)> = None;
    for candidate in candidates {
        let radius = candidate.outpost_radius_m;
        if !radius.is_finite() || radius < OUTPOST_RADIUS_MODEL_MIN_RADIUS_M {
            continue;
        }
        let tolerance =
            (radius * OUTPOST_RADIUS_MODEL_REL_TOLERANCE).max(OUTPOST_RADIUS_MODEL_ABS_TOLERANCE_M);
        let support = candidates
            .iter()
            .filter(|other| (other.outpost_radius_m - radius).abs() <= tolerance)
            .count();
        if best_model.is_none_or(|(best_support, best_radius)| {
            support > best_support || (support == best_support && radius < best_radius)
        }) {
            best_model = Some((support, radius));
        }
    }

    let (support, radius) = best_model?;
    if support < OUTPOST_RADIUS_MODEL_MIN_INLIERS {
        return None;
    }
    let tolerance =
        (radius * OUTPOST_RADIUS_MODEL_REL_TOLERANCE).max(OUTPOST_RADIUS_MODEL_ABS_TOLERANCE_M);
    candidates
        .iter()
        .filter(|candidate| (candidate.outpost_radius_m - radius).abs() <= tolerance)
        .min_by(|a, b| a.distance_squared.total_cmp(&b.distance_squared))
        .map(|candidate| (candidate.world, support))
}

/// 找到"自瞄真正会瞄的那块装甲板"的板位世界位置（ROS 系）。
///
/// 为什么需要它：真值里的 `position` 是整车中心，而自瞄解算出来、planner 追的是
/// 装甲板板心。步兵四块板呈盒状分布，板心偏心半径约 0.2m、比车心高约 0.06m，
/// 1.5m 距离上折算成 2 度量级的固定几何差。消费端拿整车中心算瞄准误差，会把
/// 这段几何差整个算进"闭环残差"，得出的数字既不是估计误差也不是控制误差。
///
/// 选板依据：板位距相机最近。四块板在凸壳上，朝向观察者的那块必然是最近的一块，
/// 这与自瞄"只能看见朝向自己的板"一致。这是几何代理而非复现自瞄的选板逻辑
/// （自瞄按图像里的检测结果和 Tracker 的 armor id 选板），所以两者在换板瞬间
/// 可能不一致——评估侧必须把这一点当作已知误差来源，不能当成闭环精度。
fn select_plate_reference(
    root: Entity,
    camera_pos: Vec3,
    reference: PlateReference,
    qq: &HierarchyQuery,
    armor_roots: &Query<(Entity, &Armor), With<ArmorRoot>>,
    transforms: &Query<&GlobalTransform>,
) -> Option<Vec3> {
    select_plate_reference_with_support(root, camera_pos, reference, qq, armor_roots, transforms)
        .map(|(world, _)| world)
}

fn select_plate_reference_with_support(
    root: Entity,
    camera_pos: Vec3,
    reference: PlateReference,
    qq: &HierarchyQuery,
    armor_roots: &Query<(Entity, &Armor), With<ArmorRoot>>,
    transforms: &Query<&GlobalTransform>,
) -> Option<(Vec3, usize)> {
    let rotor_tf = (reference == PlateReference::CenterNodeOrPlateRoot)
        .then(|| transforms.get(root).ok())
        .flatten();
    let mut candidates = Vec::new();

    for (plate, _armor) in armor_roots.iter() {
        // 这块板属于哪个目标：沿父链找，与 armor/collision.rs 里判定命中的写法一致。
        // 前哨站的板挂在旋转节点下，旋转节点又挂在前哨站根下，所以同一条祖先链判定
        // 对两类资产都成立。
        if !qq
            .child_of
            .iter_ancestors(plate)
            .any(|ancestor| ancestor == root)
        {
            continue;
        }

        // 板心节点。ros2/plugin.rs 用的是同一条路径（"CENTER" 后缀节点的子节点），
        // 但那边直接 unwrap；这里任何一步取不到就跳过这块板，不能让真值发布
        // 因为场景资产换了个节点名就 panic 掉整个仿真。
        // `?` 在这里是错的：它会从整个函数返回 None，把已经选出来的 best 一起丢掉。
        // 一块板缺 CENTER（资产换了节点名、或该板的子树还没加载完）就会让整车的板心
        // 真值全部消失，消费端只能退回整车中心，误差里凭空多出 2 度量级的几何偏差。
        // 缺板必须只跳过这一块板。
        let center = qq
            .of(plate)
            .suffix("CENTER")
            .any()
            .one()
            .or_else(|| qq.of(plate).suffix("CENTER").one());
        let node = match (center, reference) {
            (Some(center), _) => center,
            // 这份资产就是没有 CENTER 节点，板根即板位（见 `PlateReference` 的实测）。
            (None, PlateReference::CenterNodeOrPlateRoot) => plate,
            (None, PlateReference::CenterNode) => continue,
        };
        let Ok(center_tf) = transforms.get(node) else {
            continue;
        };

        let world = center_tf.translation();
        let outpost_radius_m = rotor_tf.map_or(0.0, |rotor_tf| {
            // Convert the candidate back to the *current* rotating local frame.  This remains
            // invariant after any yaw of the outpost or an ancestor, unlike comparing world x/z.
            let local = rotor_tf.rotation().inverse() * (world - rotor_tf.translation());
            Vec2::new(local.x, local.z).length()
        });
        candidates.push(PlateCandidate {
            distance_squared: world.distance_squared(camera_pos),
            world,
            outpost_radius_m,
        });
    }

    let world = match reference {
        PlateReference::CenterNode => candidates
            .iter()
            .min_by(|a, b| a.distance_squared.total_cmp(&b.distance_squared))
            .map(|candidate| (candidate.world, candidates.len())),
        PlateReference::CenterNodeOrPlateRoot => {
            select_outpost_plate_reference_with_support(&candidates)
        }
    }?;
    Some((to_ros_vec3(world.0), world.1))
}

/// 把场景里的前哨站写进真值批次。
///
/// 单独成函数是为了能被单测直接驱动：`collect_ground_truth_system` 需要
/// `TalosCaptureContext`，而它持有真实的共享内存发布器，不适合在单测里构造。
#[allow(clippy::too_many_arguments)]
fn push_outpost_targets(
    batch: &mut GroundTruthBatch,
    frame_seq: u64,
    timestamp_ns: u64,
    camera_pos: Option<Vec3>,
    outpost_mode: RotationMode,
    qq: &HierarchyQuery,
    outpost_query: &Query<(Entity, &Outpost)>,
    outpost_rotors: &Query<(Entity, &OutpostRotator, &GlobalTransform)>,
    armor_roots: &Query<(Entity, &Armor), With<ArmorRoot>>,
    transforms: &Query<&GlobalTransform>,
) {
    // ---- 前哨站真值 -------------------------------------------------------------
    //
    // 为什么发的是"转动节点"而不是前哨站根节点：根节点是静止的安装基座，自瞄侧
    // 的前哨站解算要估的是那个 0.275 m 半径圆周运动的**回转中心与相位**。回转轴
    // 正好过转动节点的原点（`rotate_y` 作用在它的局部 Transform 上），所以
    // position / yaw / vyaw 三个量取自同一个节点，彼此自洽；取根节点的话 yaw 恒为
    // 常数，vyaw 会变成一个无处对应的数。
    //
    // vyaw 的符号：`M_ALIGN_MAT3` 是行列式 +1 的真旋转，且把 Bevy 的 +Y 映到 ROS
    // 的 +Z，所以"绕 Bevy 局部 +Y 转 θ"conjugate 之后就是"绕 ROS +Z 转 θ"，与
    // `ros_yaw` 取出的 yaw 同一个符号约定，直接搬过来即可（有单测钉住）。
    for (rotor, rotator, rotor_tf) in outpost_rotors.iter() {
        if (batch.target_count as usize) >= GROUND_TRUTH_MAX_TARGETS {
            break;
        }
        let Some((_outpost_root, outpost)) = qq
            .child_of
            .iter_ancestors(rotor)
            .find_map(|ancestor| outpost_query.get(ancestor).ok())
        else {
            continue;
        };

        let pos_ros = to_ros_vec3(rotor_tf.translation());
        let team = outpost.team();

        // 板位：三块板里距相机最近的那一块。OUTPOST.glb 没有 CENTER 节点，
        // 退回板根（见 `PlateReference::CenterNodeOrPlateRoot` 的实测依据）。
        let armor = camera_pos.and_then(|camera_pos| {
            select_plate_reference_with_support(
                rotor,
                camera_pos,
                PlateReference::CenterNodeOrPlateRoot,
                qq,
                armor_roots,
                transforms,
            )
        });
        let armor_position = armor.map(|(a, _)| a);
        let armor_support = armor.map(|(_, support)| support).unwrap_or(0);

        let idx = batch.target_count as usize;
        batch.targets[idx] = GroundTruthTarget {
            frame_seq,
            timestamp_ns,
            team: team_to_u8(&team),
            armor_label: ArmorLabel::Outpost as u8,
            is_outpost: 1,
            _pad1: 0,
            position: [pos_ros.x, pos_ros.y, pos_ros.z],
            vyaw: rotator.signed_yaw_rate(outpost_mode),
            yaw: ros_yaw(rotor_tf),
            armor_position: armor_position.map(|a| [a.x, a.y, a.z]).unwrap_or([0.0; 3]),
            armor_position_valid: u8::from(armor_position.is_some()),
            armor_position_degraded: u8::from(armor_position.is_none() || armor_support < 3),
            identity: rotor.to_bits() as u16,
            _pad: [0; 8],
        };
        batch.target_count += 1;
    }
}

/// 采集本帧真值，写进 [`TalosGroundTruthFrame`]，**不**自己发布。
///
/// 发布由图像那一侧的事务完成（`TalosSnapshot::captured` 的 `before_commit` 回调），
/// 这样图像、同帧姿态、同帧真值三者的 `frame_seq` 严格相等。见
/// [`TalosGroundTruthFrame`] 的说明，以及它替换掉的那套"真值历史 + 按已发布图像
/// 帧号回放"的做法为什么做不到同帧。
///
/// 本系统必须排在 `advance_talos_frame_stamp` 之后、且与紧随主世界的
/// ExtractSchedule 同一帧内，否则 `extract_pose_data` 的帧号一致性检查会把这一批
/// 真值丢掉（宁可少样本，也不能发错帧的真值）。
pub fn collect_ground_truth_system(
    context: Option<Res<TalosCaptureContext>>,
    frame_stamp: Res<TalosFrameStamp>,
    mut out: ResMut<TalosGroundTruthFrame>,
    infantry_query: Query<
        (
            Entity,
            &GlobalTransform,
            Option<&AngularVelocity>,
            &Infantry,
        ),
        Without<Controlled>,
    >,
    controlled_query: Query<
        (
            Entity,
            &GlobalTransform,
            Option<&AngularVelocity>,
            &Infantry,
        ),
        With<Controlled>,
    >,
    camera_query: Query<&GlobalTransform, With<CaptureSource>>,
    armor_roots: Query<(Entity, &Armor), With<ArmorRoot>>,
    transforms: Query<&GlobalTransform>,
    qq: HierarchyQuery,
    // 前哨站：队伍/旋向在根节点（`OutpostRoot` -> `Outpost`），转动在名字带 ROTATE
    // 的后代节点上（`OutpostRotator`）。两者分开查，靠祖先链关联。
    outpost_query: Query<(Entity, &Outpost)>,
    outpost_rotors: Query<(Entity, &OutpostRotator, &GlobalTransform)>,
    outpost_mode: Option<Res<OutpostRotationMode>>,
    rune_query: Query<(
        Entity,
        &GlobalTransform,
        &Transform,
        &PowerRune,
        &PowerRuneMechanism,
        &PowerRuneRotation,
    )>,
    named_transforms: Query<(Entity, &Name, &GlobalTransform)>,
) {
    // 没有采集上下文就没有图像事务，真值也无处提交。清掉上一帧的残留，
    // 免得它被误当成本帧的数据。
    if context.is_none() {
        out.batch = None;
        return;
    }

    let frame_seq = frame_stamp.frame_seq;
    let timestamp_ns = frame_stamp.timestamp_ns;

    let mut batch = GroundTruthBatch::default();
    batch.frame_seq = frame_seq;
    batch.timestamp_ns = timestamp_ns;

    // 选板要用相机位置。相机取不到就退化成只发整车中心（armor_position_valid=0），
    // 消费端据此知道板心不可用，而不是拿到一个默默用错原点算出来的数。
    let camera_pos = camera_query.single().ok().map(|tf| tf.translation());

    // Collect robot ground truth from all infantry robots
    let all_robots = infantry_query.iter().chain(controlled_query.iter());

    // 整车实体 -> targets[] 下标，供下面回填板心。
    let mut slot_of: HashMap<Entity, usize> = HashMap::new();

    for (vehicle, global_tf, ang_vel, infantry) in all_robots {
        let pos_ros = to_ros_vec3(global_tf.translation());
        let team = &infantry.team;
        let config = infantry.config;

        let vyaw = ang_vel
            .map(|av| {
                let ros_ang = to_ros_vec3(av.0);
                ros_ang.z
            })
            .unwrap_or(0.0);

        let yaw = ros_yaw(global_tf);

        if (batch.target_count as usize) < GROUND_TRUTH_MAX_TARGETS {
            let idx = batch.target_count as usize;
            batch.targets[idx] = GroundTruthTarget {
                frame_seq,
                timestamp_ns,
                team: team_to_u8(team),
                armor_label: config.armor.label() as u8,
                is_outpost: 0,
                _pad1: 0,
                position: [pos_ros.x, pos_ros.y, pos_ros.z],
                vyaw,
                yaw,
                armor_position: [0.0; 3],
                armor_position_valid: 0,
                armor_position_degraded: 0,
                identity: vehicle.to_bits() as u16,
                _pad: [0; 8],
            };
            batch.target_count += 1;
            slot_of.insert(vehicle, idx);
        }
    }

    if let Some(camera_pos) = camera_pos {
        for (vehicle, idx) in slot_of.iter() {
            if let Some(center_ros) = select_plate_reference(
                *vehicle,
                camera_pos,
                PlateReference::CenterNode,
                &qq,
                &armor_roots,
                &transforms,
            ) {
                let t = &mut batch.targets[*idx];
                t.armor_position = [center_ros.x, center_ros.y, center_ros.z];
                t.armor_position_valid = 1;
            }
        }
    }

    push_outpost_targets(
        &mut batch,
        frame_seq,
        timestamp_ns,
        camera_pos,
        outpost_mode.map(|m| m.0).unwrap_or_default(),
        &qq,
        &outpost_query,
        &outpost_rotors,
        &armor_roots,
        &transforms,
    );

    // Collect rune ground truth
    for (rune_entity, global_tf, local_tf, power_rune, mechanism, rotation) in rune_query.iter() {
        if (batch.rune_count as usize) >= GROUND_TRUTH_MAX_RUNES {
            break;
        }

        let pos_ros = to_ros_vec3(global_tf.translation());

        // Extract current rotation angle around the actual rune axis (-1, 0, -1).
        // The rune rotates via `rotate_local_axis(direction, angle)`, so we must
        // project the quaternion back onto that axis — not extract an Euler X angle.
        let rune_axis = Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap();
        let (axis, angle) = local_tf.rotation.to_axis_angle();
        let current_angle = angle * axis.dot(*rune_axis).signum();

        let controller = rotation.controller();
        let direction = if controller.is_clockwise() { 1 } else { -1 };

        let (sin_amplitude, sin_omega, relative_time, sin_offset) = controller
            .variable_params()
            .map(|(a, omega, t)| (a, omega, t, 2.090 - a))
            .unwrap_or((0.0, 0.0, 0.0, 0.0));

        // Scene bullseye nodes are the physical blade reference. A missing/partial glTF
        // load falls back to the rune centre with radius 0 so blade_id stays a stable 0
        // rather than the ambiguous -1 placeholder.
        let geometry = rune_blade_geometry(
            rune_entity,
            global_tf.translation(),
            mechanism.state().current_target_index(),
            &qq,
            &named_transforms,
        );
        let face_index = qq
            .name
            .get(rune_entity)
            .ok()
            .and_then(|name| rune_face_index(name.as_str()))
            .unwrap_or(0);
        let (radius, blade_angle, geometry_blade_id, blade_point, identity) =
            geometry.unwrap_or((
                0.0,
                current_angle,
                0,
                global_tf.translation(),
                rune_blade_identity(face_index, 0),
            ));

        let mut target_activations = [0u8; 5];
        for (i, a) in mechanism.state().target_states().iter().enumerate() {
            if i < 5 {
                target_activations[i] = activation_to_u8(a);
            }
        }

        let idx = batch.rune_count as usize;
        batch.runes[idx] = GroundTruthRune {
            frame_seq,
            timestamp_ns,
            team: team_to_u8(&power_rune.team()),
            rune_mode: rune_mode_to_u8(&power_rune.mode()),
            mechanism_state: mechanism_state_to_u8(mechanism.state()),
            pad0: 0,
            r_center_odom: [pos_ros.x, pos_ros.y, pos_ros.z],
            radius,
            current_angle: blade_angle,
            v_roll: if rotation.last_speed().abs() > 1e-6 {
                rotation.last_speed()
            } else {
                rotation.instantaneous_speed(power_rune.mode())
            },
            direction,
            sin_amplitude,
            sin_omega,
            sin_phase: 0.0,
            sin_offset,
            relative_time,
            blade_id: geometry_blade_id as i32,
            target_activations,
            pad_act: [0; 3],
            target_point_odom: {
                let p = to_ros_vec3(blade_point);
                [p.x, p.y, p.z]
            },
            identity,
            _pad: [0; 34],
        };
        batch.rune_count += 1;
    }

    // 交给图像事务去发布。这里只放下"本帧的真值 + 它属于哪一帧"，
    // `extract_pose_data` 会核对帧号后带进快照。
    //
    // 真值只流向评估器，不进算法输入（YOLO/Solver/Tracker/Planner），这条边界不变。
    out.frame_seq = frame_seq;
    out.batch = Some(Box::new(batch));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 第一块板缺 CENTER、第二块板有 CENTER：缺板只能跳过它自己。
    ///
    /// 修复前 `select_armor_center` 用 `?` 收尾，第一块缺 CENTER 的板会让整个函数
    /// 返回 None，把后面所有板的板心一起丢掉，于是整车的 `armor_position_valid`
    /// 恒为 0，消费端只能退回整车中心去算瞄准误差——凭空多出 2 度量级的固定几何差。
    /// 断言是 `Some(最近板心)`：修复前无论 `armor_roots.iter()` 是什么顺序，只要
    /// 缺板存在结果就是 None，所以这个断言不依赖 ECS 的迭代顺序。
    #[test]
    fn plate_without_center_skips_only_itself() {
        use crate::robomaster::prelude::{ArmorId, ArmorSpec, SmallArmorLabel};
        use bevy::ecs::system::RunSystemOnce;

        fn armor_of(name: &str) -> Armor {
            Armor {
                name: name.to_string(),
                team: Team::Blue,
                spec: ArmorSpec::Small(SmallArmorLabel::Three),
                label: crate::robomaster::prelude::ArmorLabel::Three,
            }
        }

        let mut world = World::new();
        let camera_pos = Vec3::new(0.0, 0.0, 0.0);
        let vehicle = world.spawn(GlobalTransform::default()).id();

        // 三块板都挂在同一辆车下，组件集合完全相同（同一个 archetype）。
        let plate = |world: &mut World, id: usize, at: Vec3| {
            let e = world
                .spawn((
                    armor_of("plate"),
                    ArmorRoot {
                        id: ArmorId::from_raw_for_test(id),
                    },
                    GlobalTransform::from_translation(at),
                ))
                .id();
            world.entity_mut(vehicle).add_child(e);
            e
        };

        // 第一块：没有 CENTER 子节点（资产换了节点名 / 子树没加载完）。
        plate(&mut world, 0, Vec3::new(0.0, 0.0, 1.0));

        // 第二块：有 CENTER，但离相机远。
        let far = plate(&mut world, 1, Vec3::new(0.0, 0.0, 3.0));
        let far_center = Vec3::new(0.0, 0.0, 3.0);
        let far_center_node = world
            .spawn((
                Name::new("PLATE_CENTER"),
                GlobalTransform::from_translation(far_center),
            ))
            .id();
        world.entity_mut(far).add_child(far_center_node);

        // 第三块：有 CENTER 且最近，它才是应该被选中的那块。
        let near = plate(&mut world, 2, Vec3::new(0.0, 0.0, 1.5));
        let near_center = Vec3::new(0.0, 0.0, 1.5);
        let near_center_node = world
            .spawn((
                Name::new("PLATE_CENTER"),
                GlobalTransform::from_translation(near_center),
            ))
            .id();
        world.entity_mut(near).add_child(near_center_node);

        let got = world
            .run_system_once(
                move |qq: HierarchyQuery,
                      armor_roots: Query<(Entity, &Armor), With<ArmorRoot>>,
                      transforms: Query<&GlobalTransform>| {
                    select_plate_reference(
                        vehicle,
                        camera_pos,
                        PlateReference::CenterNode,
                        &qq,
                        &armor_roots,
                        &transforms,
                    )
                },
            )
            .expect("system 运行失败");

        let expected = to_ros_vec3(near_center);
        let got = got.expect("缺 CENTER 的板不得让整车板心失效");
        assert!(
            got.distance(expected) < 1e-6,
            "应选中最近的那块板心 {expected:?}，实际 {got:?}"
        );
    }

    /// 搭一座前哨站：根节点带 `Outpost`，转动节点带 `OutpostRotator`，三块板挂在
    /// 转动节点下且**没有** CENTER 子节点（与 `assets/OUTPOST.glb` 一致）。
    fn spawn_outpost(
        world: &mut World,
        team: Team,
        rotor_at: Vec3,
        plate_offsets: &[Vec3],
        id_base: usize,
    ) -> (Entity, Entity) {
        use crate::robomaster::prelude::{ArmorId, ArmorSpec, RotationDirection, SmallArmorLabel};

        let root = world
            .spawn((Outpost::new(team), GlobalTransform::default()))
            .id();
        // 旋向与 `setup_outpost` 一致：红方顺时针、蓝方逆时针。
        let direction = match team {
            Team::Red => RotationDirection::Clockwise,
            Team::Blue => RotationDirection::CounterClockwise,
        };
        let rotor = world
            .spawn((
                OutpostRotator::new(direction),
                Transform::from_translation(rotor_at),
                GlobalTransform::from_translation(rotor_at),
            ))
            .id();
        world.entity_mut(root).add_child(rotor);

        for (i, off) in plate_offsets.iter().enumerate() {
            let plate = world
                .spawn((
                    Armor {
                        name: format!("{i}_ARMOR_ROOT"),
                        team,
                        spec: ArmorSpec::Small(SmallArmorLabel::Outpost),
                        label: ArmorLabel::Outpost,
                    },
                    ArmorRoot {
                        id: ArmorId::from_raw_for_test(id_base + i),
                    },
                    GlobalTransform::from_translation(rotor_at + *off),
                ))
                .id();
            world.entity_mut(rotor).add_child(plate);
        }
        (root, rotor)
    }

    fn collect_outposts(world: &mut World, camera_pos: Option<Vec3>) -> GroundTruthBatch {
        use bevy::ecs::system::RunSystemOnce;

        world
            .run_system_once(
                move |qq: HierarchyQuery,
                      outpost_query: Query<(Entity, &Outpost)>,
                      outpost_rotors: Query<(Entity, &OutpostRotator, &GlobalTransform)>,
                      armor_roots: Query<(Entity, &Armor), With<ArmorRoot>>,
                      transforms: Query<&GlobalTransform>| {
                    let mut b = GroundTruthBatch::default();
                    push_outpost_targets(
                        &mut b,
                        7,
                        1_234_567_890,
                        camera_pos,
                        RotationMode::Forward,
                        &qq,
                        &outpost_query,
                        &outpost_rotors,
                        &armor_roots,
                        &transforms,
                    );
                    b
                },
            )
            .expect("system 运行失败")
    }

    /// 红蓝两座前哨站都必须出现在真值批次里，且靠 `team` 区分。
    ///
    /// 改动前 `is_outpost` 硬编码 0、前哨站根本不进 `targets[]`，C++ 端
    /// `GroundTruthEvaluator::find_by_label(6, ...)` 恒 0 命中，报告里
    /// `ground_truth.count=0`、`aim_error` 整段缺失——看起来像"评估没开"，
    /// 实际是"没有可评估的目标"。
    #[test]
    fn red_and_blue_outposts_are_published_with_team_and_outpost_label() {
        const R: f32 = 0.2757; // OUTPOST.glb 实测板位半径
        let plates = [
            Vec3::new(R, -0.138, 0.0),
            Vec3::new(-0.5 * R, -0.138, 0.866 * R),
            Vec3::new(-0.5 * R, -0.138, -0.866 * R),
        ];

        let mut world = World::new();
        let red_rotor_at = Vec3::new(3.06, 1.14, -5.50);
        let blue_rotor_at = Vec3::new(-3.06, 1.14, 2.21);
        spawn_outpost(&mut world, Team::Red, red_rotor_at, &plates, 0);
        spawn_outpost(&mut world, Team::Blue, blue_rotor_at, &plates, 10);

        // 相机放在红方前哨站的 +X 侧，第一块板应当是最近的那块。
        let camera_pos = red_rotor_at + Vec3::new(3.0, 0.0, 0.0);
        let batch = collect_outposts(&mut world, Some(camera_pos));

        assert_eq!(batch.target_count, 2, "红蓝两座前哨站都要发");
        let mut by_team = [None, None];
        for t in &batch.targets[..2] {
            assert_eq!(t.is_outpost, 1, "前哨站必须置 is_outpost=1");
            assert_eq!(t.armor_label, 6, "前哨站 armor_label 必须是 Outpost=6");
            assert_eq!(t.frame_seq, 7);
            assert_eq!(t.timestamp_ns, 1_234_567_890);
            by_team[t.team as usize] = Some(*t);
        }
        let red = by_team[0].expect("缺红方前哨站（team=0）");
        let blue = by_team[1].expect("缺蓝方前哨站（team=1）");

        // 位置 = 转动节点的世界位置（回转中心），换算到 ROS 系。
        for (t, at) in [(&red, red_rotor_at), (&blue, blue_rotor_at)] {
            let want = to_ros_vec3(at);
            let got = Vec3::from_array(t.position);
            assert!(
                got.distance(want) < 1e-5,
                "位置应是回转中心 {want:?}，实际 {got:?}"
            );
        }

        // vyaw：同一个转速常量、旋向相反。
        let speed = 0.8 * std::f32::consts::PI;
        assert!(
            (red.vyaw - speed).abs() < 1e-5,
            "红方 vyaw 应为 +{speed}，实际 {}",
            red.vyaw
        );
        assert!(
            (blue.vyaw + speed).abs() < 1e-5,
            "蓝方 vyaw 应为 -{speed}，实际 {}",
            blue.vyaw
        );

        // 板位：OUTPOST.glb 没有 CENTER 节点，必须退回板根而不是判成"板位不可用"。
        assert_eq!(
            red.armor_position_valid, 1,
            "前哨站缺 CENTER 节点也必须给出板位（退回板根）"
        );
        assert_eq!(
            red.armor_position_degraded, 0,
            "正常三板同半径模型不得标记 degraded"
        );
        let want_plate = to_ros_vec3(red_rotor_at + plates[0]);
        let got_plate = Vec3::from_array(red.armor_position);
        assert!(
            got_plate.distance(want_plate) < 1e-5,
            "应选中距相机最近的那块板 {want_plate:?}，实际 {got_plate:?}"
        );
    }

    /// 两块同半径板可以保留诊断板位，但必须显式标为 degraded，不能被当成三板精度基准。
    #[test]
    fn outpost_two_plate_model_is_marked_degraded() {
        const R: f32 = 0.275;
        let rotor_at = Vec3::new(1.0, 1.0, 1.0);
        let mut world = World::new();
        spawn_outpost(
            &mut world,
            Team::Blue,
            rotor_at,
            &[Vec3::new(R, 0.0, 0.0), Vec3::new(-R, 0.0, 0.0)],
            0,
        );
        let batch = collect_outposts(&mut world, Some(rotor_at + Vec3::new(2.0, 0.0, 0.0)));
        assert_eq!(batch.target_count, 1);
        assert_eq!(batch.targets[0].armor_position_valid, 1);
        assert_eq!(
            batch.targets[0].armor_position_degraded, 1,
            "只有两个节点不能默认为三板精度基准"
        );
    }

    /// 三板的 120 度角间隔与回转后的局部半径都必须保持；世界坐标旋转不能绕过半径模型。
    #[test]
    fn outpost_three_plate_angle_spacing_survives_rotation() {
        const R: f32 = 0.275;
        let offsets = [
            Vec3::new(R, 0.0, 0.0),
            Vec3::new(-0.5 * R, 0.0, 0.8660254 * R),
            Vec3::new(-0.5 * R, 0.0, -0.8660254 * R),
        ];
        for yaw_deg in [0.0_f32, 37.0, 123.0] {
            let rotor_at = Vec3::new(-1.0, 1.0, 2.0);
            let mut world = World::new();
            let (_root, rotor) = spawn_outpost(&mut world, Team::Blue, rotor_at, &offsets, 0);
            let rotation = Quat::from_rotation_y(yaw_deg.to_radians());
            let tf = Transform::from_translation(rotor_at).with_rotation(rotation);
            *world.get_mut::<Transform>(rotor).unwrap() = tf;
            *world.get_mut::<GlobalTransform>(rotor).unwrap() = GlobalTransform::from(tf);
            for (i, offset) in offsets.iter().enumerate() {
                let plate = world
                    .query_filtered::<(Entity, &Armor), With<ArmorRoot>>()
                    .iter(&world)
                    .nth(i)
                    .map(|(entity, _)| entity)
                    .unwrap();
                *world.get_mut::<GlobalTransform>(plate).unwrap() =
                    GlobalTransform::from_translation(rotor_at + rotation * *offset);
            }
            let batch = collect_outposts(&mut world, Some(rotor_at + Vec3::new(3.0, 0.0, 0.0)));
            assert_eq!(batch.targets[0].armor_position_valid, 1);
            assert_eq!(batch.targets[0].armor_position_degraded, 0);
            let selected = Vec3::from_array(batch.targets[0].armor_position);
            let selected_bevy = M_ALIGN_MAT3.inverse() * selected;
            let local = rotation.inverse() * (selected_bevy - rotor_at);
            assert!((Vec2::new(local.x, local.z).length() - R).abs() < 1e-4);
        }
    }

    /// `OUTPOST_B_ROTATE/F_ARMOR_ROOT` 的水平半径约为正常板的 6.5 倍。即使它离相机
    /// 最近，也不能把它当成自瞄板心真值；筛选依据必须是转轴局部几何，不是资产名字。
    #[test]
    fn outpost_radius_model_rejects_a_nearest_outlier_after_rotation() {
        use crate::robomaster::prelude::{ArmorId, ArmorSpec, SmallArmorLabel};

        const R: f32 = 0.275;
        let rotor_at = Vec3::new(-3.06, 1.14, 2.21);
        let rotor_rotation = Quat::from_rotation_y(67.0_f32.to_radians());
        let normal_offsets = [
            Vec3::new(R, -0.138, 0.0),
            Vec3::new(-0.5 * R, -0.138, 0.866 * R),
        ];
        // F 的数字来自当前 OUTPOST.glb 的 B 组资产量级；特意不把节点名传给筛选逻辑。
        let outlier_offset = Vec3::new(-0.151, 0.519, 1.777);

        let mut world = World::new();
        let (_root, rotor) = spawn_outpost(&mut world, Team::Blue, rotor_at, &[], 0);
        *world.get_mut::<GlobalTransform>(rotor).unwrap() = GlobalTransform::from(
            Transform::from_translation(rotor_at).with_rotation(rotor_rotation),
        );

        let armor = |name: &str| Armor {
            name: name.to_string(),
            team: Team::Blue,
            spec: ArmorSpec::Small(SmallArmorLabel::Outpost),
            label: ArmorLabel::Outpost,
        };
        let add_armor_root = |world: &mut World, id: usize, name: &str, offset: Vec3| {
            let at = rotor_at + rotor_rotation * offset;
            let entity = world
                .spawn((
                    armor(name),
                    ArmorRoot {
                        id: ArmorId::from_raw_for_test(id),
                    },
                    GlobalTransform::from_translation(at),
                ))
                .id();
            world.entity_mut(rotor).add_child(entity);
            (entity, at)
        };
        let (_, normal_a) = add_armor_root(&mut world, 10, "normal_a", normal_offsets[0]);
        let (_, normal_b) = add_armor_root(&mut world, 11, "normal_b", normal_offsets[1]);
        let (outlier, outlier_at) = add_armor_root(&mut world, 12, "outlier", outlier_offset);

        // This mirrors the current AT/BT boundary: it is an Armor-named scene node but lacks a
        // successfully constructed ArmorRoot, so the production `With<ArmorRoot>` query must not
        // treat it as a plate candidate even when it is closest to the camera.
        let missing_root = world
            .spawn((
                armor("AT_ARMOR_ROOT"),
                GlobalTransform::from_translation(outlier_at),
            ))
            .id();
        world.entity_mut(rotor).add_child(missing_root);
        let armor_root_entities = world
            .query_filtered::<Entity, With<ArmorRoot>>()
            .iter(&world)
            .collect::<Vec<_>>();
        assert!(armor_root_entities.contains(&outlier));
        assert!(
            !armor_root_entities.contains(&missing_root),
            "缺 ArmorRoot 的资产节点不得进入生产板位查询"
        );

        let camera_pos = outlier_at + Vec3::new(0.01, 0.0, 0.0);
        assert!(
            outlier_at.distance_squared(camera_pos)
                < normal_a
                    .distance_squared(camera_pos)
                    .min(normal_b.distance_squared(camera_pos)),
            "回归前提：异常 F 板必须是未过滤最近候选"
        );
        let expected = [normal_a, normal_b]
            .into_iter()
            .min_by(|a, b| {
                a.distance_squared(camera_pos)
                    .total_cmp(&b.distance_squared(camera_pos))
            })
            .unwrap();

        let batch = collect_outposts(&mut world, Some(camera_pos));
        assert_eq!(batch.target_count, 1);
        let target = batch.targets[0];
        assert_eq!(target.armor_position_valid, 1);
        let selected = Vec3::from_array(target.armor_position);
        assert!(
            selected.distance(to_ros_vec3(expected)) < 1e-5,
            "正常板仍应按最近策略选中：expected {:?}, actual {:?}",
            to_ros_vec3(expected),
            selected
        );
        assert!(
            selected.distance(to_ros_vec3(outlier_at)) > 1.0,
            "即使异常板最近，也不得发布它作为 armor_position"
        );
        let local_outlier = rotor_rotation.inverse() * (outlier_at - rotor_at);
        assert!(
            Vec2::new(local_outlier.x, local_outlier.z).length() > 6.0 * R,
            "测试离群半径不足以代表当前资产缺陷"
        );
    }

    /// 取不到相机时只发回转中心，板位标成不可用——不能默默发一个用错原点的板位。
    #[test]
    fn outpost_without_camera_reports_armor_position_invalid() {
        let mut world = World::new();
        spawn_outpost(
            &mut world,
            Team::Red,
            Vec3::new(1.0, 1.0, 1.0),
            &[Vec3::new(0.27, 0.0, 0.0)],
            0,
        );
        let batch = collect_outposts(&mut world, None);
        assert_eq!(batch.target_count, 1);
        assert_eq!(batch.targets[0].armor_position_valid, 0);
        assert_eq!(batch.targets[0].armor_position, [0.0; 3]);
        assert_eq!(batch.targets[0].is_outpost, 1);
    }

    /// yaw 与 vyaw 必须是同一个符号约定：把转动节点按 `+vyaw*dt` 转一步，
    /// 发布出去的 yaw 就得增加 `vyaw*dt`。
    ///
    /// 这条把 ROS↔Bevy 的映射钉住了。`M_ALIGN_MAT3` 把 Bevy +Y 映到 ROS +Z 且
    /// 行列式为 +1，所以"绕 Bevy 局部 +Y 转 θ"应当等于"绕 ROS +Z 转 θ"。如果哪天
    /// 有人给 `to_ros_vec3` 换成一个含反射的矩阵，或者把 `ros_yaw` 的欧拉序改了，
    /// vyaw 的符号就会与 yaw 的走向相反——评估端算出来的预测误差会是转速的两倍，
    /// 而两个数字本身看着都很正常。
    ///
    /// 与 `rotation.rs` 的 `signed_speed_matches_what_step_actually_rotates` 合起来
    /// 闭合整条链：`signed_speed` -> `rotate_y` -> `ros_yaw`。
    #[test]
    fn outpost_yaw_advances_in_the_same_direction_as_vyaw() {
        for team in [Team::Red, Team::Blue] {
            let mut world = World::new();
            let (_, rotor) = spawn_outpost(
                &mut world,
                team,
                Vec3::new(0.0, 1.14, 0.0),
                &[Vec3::new(0.27, 0.0, 0.0)],
                0,
            );

            let before = collect_outposts(&mut world, None).targets[0];

            // 按真值自己报出来的 vyaw 转一步（rotate_y 与生产路径同一个调用）。
            let dt = 0.02_f32;
            let step = before.vyaw * dt;
            let mut tf = *world.get::<Transform>(rotor).unwrap();
            tf.rotate_y(step);
            *world.get_mut::<Transform>(rotor).unwrap() = tf;
            *world.get_mut::<GlobalTransform>(rotor).unwrap() = GlobalTransform::from(tf);

            let after = collect_outposts(&mut world, None).targets[0];
            let d = (after.yaw - before.yaw).rem_euclid(std::f32::consts::TAU);
            let d = if d > std::f32::consts::PI {
                d - std::f32::consts::TAU
            } else {
                d
            };
            assert!(
                (d - step).abs() < 1e-4,
                "{team:?}: vyaw={} 转 {dt}s 应让 yaw 变化 {step}，实际 {d}",
                before.vyaw
            );
        }
    }

    #[test]
    fn duplicate_armor_label_is_disambiguated_by_team() {
        // 红蓝三号步兵共用 armor label=3（场景里同一个 config 被两队复用）。
        // 真值批次必须带上 team，否则消费端"按 label 取第一个命中"必然配错车。
        // 这里锁住的是 team 字段的线上编码：Red=0 / Blue=1。
        let mut b = GroundTruthBatch {
            target_count: 2,
            ..Default::default()
        };
        b.targets[0] = GroundTruthTarget {
            team: team_to_u8(&Team::Red),
            armor_label: 3,
            position: [1.0, 0.0, 0.2],
            ..Default::default()
        };
        b.targets[1] = GroundTruthTarget {
            team: team_to_u8(&Team::Blue),
            armor_label: 3,
            position: [3.0, 0.5, 0.2],
            ..Default::default()
        };

        assert_eq!(b.targets[0].team, 0);
        assert_eq!(b.targets[1].team, 1);
        assert_ne!(
            b.targets[0].team, b.targets[1].team,
            "同 label 必须靠 team 区分"
        );
        assert_eq!(b.targets[0].armor_label, b.targets[1].armor_label);
    }

    #[test]
    fn bullseye_status_filter_rejects_legging_and_padding() {
        let prefix = "FACE_1_TARGET_";
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_1_ACTIVATED"),
            Some(0)
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_3_ACTIVE"),
            Some(2)
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_5_COMPLETED"),
            Some(4)
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_2_DISABLED"),
            Some(1)
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_1_LEGGING_1"),
            None
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_1_PADDING"),
            None
        );
        assert_eq!(
            parse_rune_bullseye_node(prefix, "FACE_1_TARGET_1_LEGGING_PROGRESSING"),
            None
        );
        assert_eq!(parse_rune_bullseye_node(prefix, "FACE_1_R_PADDING"), None);
        assert_eq!(rune_blade_identity(1, 2), (1 << 8) | 2);
    }

    #[test]
    fn current_blade_geometry_ignores_legging_and_uses_mechanism_target() {
        use bevy::ecs::system::RunSystemOnce;

        let mut world = World::new();
        let root = world.spawn(GlobalTransform::default()).id();
        let face = world
            .spawn((Name::new("FACE_1"), GlobalTransform::default()))
            .id();
        world.entity_mut(root).add_child(face);
        let add = |world: &mut World, name: &str, at: Vec3| {
            let entity = world
                .spawn((
                    Name::new(name.to_string()),
                    GlobalTransform::from_translation(at),
                ))
                .id();
            world.entity_mut(face).add_child(entity);
            entity
        };
        add(
            &mut world,
            "FACE_1_TARGET_1_ACTIVATED",
            Vec3::new(0.7, 0.0, 0.0),
        );
        add(
            &mut world,
            "FACE_1_TARGET_1_LEGGING_1",
            Vec3::new(2.5, 0.0, 0.0),
        );
        add(
            &mut world,
            "FACE_1_TARGET_1_PADDING",
            Vec3::new(2.4, 0.0, 0.0),
        );
        add(
            &mut world,
            "FACE_1_TARGET_3_ACTIVATED",
            Vec3::new(0.0, 0.0, 0.7),
        );

        let geometry = world
            .run_system_once(
                move |qq: HierarchyQuery, named: Query<(Entity, &Name, &GlobalTransform)>| {
                    rune_blade_geometry(face, Vec3::ZERO, Some(2), &qq, &named)
                },
            )
            .expect("system");
        let (radius, _angle, blade_id, point, identity) =
            geometry.expect("bullseye geometry");
        assert_eq!(blade_id, 2, "mechanism current blade, not ECS-first blade 0");
        assert!(
            (radius - 0.7).abs() < 1e-4,
            "radius {radius} must be bullseye not LEGGING"
        );
        assert!(
            point.distance(Vec3::new(0.0, 0.0, 0.7)) < 1e-4,
            "target point must be blade 3 bullseye, got {point:?}"
        );
        assert_eq!(identity, rune_blade_identity(1, 2));

        let fallback = world
            .run_system_once(
                move |qq: HierarchyQuery, named: Query<(Entity, &Name, &GlobalTransform)>| {
                    rune_blade_geometry(face, Vec3::ZERO, None, &qq, &named)
                },
            )
            .expect("system")
            .expect("fallback geometry");
        assert_eq!(fallback.2, 0);
        assert!(
            (fallback.0 - 0.7).abs() < 1e-4,
            "fallback must still exclude LEGGING"
        );
    }

    #[test]
    fn power_glb_bullseye_nodes_lock_radius_near_0_7m() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/POWER.glb");
        let data = std::fs::read(&path).expect("POWER.glb");
        assert_eq!(&data[0..4], b"glTF");
        let chunk_len = u32::from_le_bytes(data[12..16].try_into().unwrap()) as usize;
        assert_eq!(&data[16..20], b"JSON");
        let json: serde_json::Value =
            serde_json::from_slice(&data[20..20 + chunk_len]).expect("gltf JSON");
        let nodes = json["nodes"].as_array().expect("nodes");
        let mut parent = vec![None; nodes.len()];
        for (i, node) in nodes.iter().enumerate() {
            if let Some(children) = node.get("children").and_then(|c| c.as_array()) {
                for child in children {
                    parent[child.as_u64().unwrap() as usize] = Some(i);
                }
            }
        }
        let translation = |node: &serde_json::Value| {
            node.get("translation")
                .and_then(|t| t.as_array())
                .map(|t| {
                    Vec3::new(
                        t[0].as_f64().unwrap() as f32,
                        t[1].as_f64().unwrap() as f32,
                        t[2].as_f64().unwrap() as f32,
                    )
                })
                .unwrap_or(Vec3::ZERO)
        };
        let world_of = |mut idx: usize| {
            let mut pos = Vec3::ZERO;
            let mut guard = 0;
            loop {
                pos += translation(&nodes[idx]);
                match parent[idx] {
                    Some(p) if guard < 32 => {
                        idx = p;
                        guard += 1;
                    }
                    _ => break pos,
                }
            }
        };
        let mut bullseye = Vec::new();
        let mut rejected = Vec::new();
        for (i, node) in nodes.iter().enumerate() {
            let Some(name) = node.get("name").and_then(|n| n.as_str()) else {
                continue;
            };
            for face in ["FACE_1", "FACE_2"] {
                let prefix = format!("{face}_TARGET_");
                let Some(center_idx) = nodes.iter().position(|n| {
                    n.get("name").and_then(|v| v.as_str()) == Some(face)
                }) else {
                    continue;
                };
                let center = world_of(center_idx);
                let radius = world_of(i).distance(center);
                if parse_rune_bullseye_node(&prefix, name).is_some() {
                    bullseye.push((name.to_string(), radius));
                } else if name.starts_with(&prefix) {
                    rejected.push((name.to_string(), radius));
                }
            }
        }
        assert_eq!(bullseye.len(), 40, "2 faces × 5 blades × 4 statuses");
        for (name, radius) in &bullseye {
            assert!(
                (*radius - 0.7).abs() < 0.01,
                "{name} radius {radius} must lock to ≈0.7 m"
            );
        }
        let padding_far = rejected
            .iter()
            .filter(|(name, r)| {
                (name.contains("PADDING") || name.contains("LEGGING")) && *r > 1.5
            })
            .count();
        assert!(
            padding_far > 0,
            "POWER.glb must contain far LEGGING/PADDING nodes so the filter is load-bearing"
        );
    }
}
