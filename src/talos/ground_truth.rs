use crate::capture::CaptureSource;
use crate::components::{Controlled, Infantry};
use crate::robomaster::prelude::{
    Activation, Armor, ArmorRoot, MechanismState, PowerRune, PowerRuneMechanism, PowerRuneRotation,
    RuneMode, Team,
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

/// Compute yaw in the ROS reference frame from a Bevy GlobalTransform.
///
/// The alignment matrix maps Bevy (Y-up) → ROS (Z-up).
/// We convert the rotation quaternion through the alignment to extract the Z-up yaw.
fn ros_yaw(global_tf: &GlobalTransform) -> f32 {
    let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);
    let ros_rot = align_quat * global_tf.rotation() * align_quat.inverse();
    let (_, _, yaw) = ros_rot.to_euler(EulerRot::ZYX);
    yaw
}

/// 从整车实体找到"自瞄真正会瞄的那块装甲板"的板心世界位置（ROS 系）。
///
/// 为什么需要它：真值里的 `position` 是整车中心，而自瞄解算出来、planner 追的是
/// 装甲板板心。步兵四块板呈盒状分布，板心偏心半径约 0.2m、比车心高约 0.06m，
/// 1.5m 距离上折算成 2 度量级的固定几何差。消费端拿整车中心算瞄准误差，会把
/// 这段几何差整个算进"闭环残差"，得出的数字既不是估计误差也不是控制误差。
///
/// 选板依据：板心距相机最近。四块板在凸壳上，朝向观察者的那块必然是最近的一块，
/// 这与自瞄"只能看见朝向自己的板"一致。这是几何代理而非复现自瞄的选板逻辑
/// （自瞄按图像里的检测结果和 Tracker 的 armor id 选板），所以两者在换板瞬间
/// 可能不一致——评估侧必须把这一点当作已知误差来源，不能当成闭环精度。
fn select_armor_center(
    vehicle: Entity,
    camera_pos: Vec3,
    qq: &HierarchyQuery,
    armor_roots: &Query<(Entity, &Armor), With<ArmorRoot>>,
    transforms: &Query<&GlobalTransform>,
) -> Option<Vec3> {
    let mut best: Option<(f32, Vec3)> = None;

    for (plate, _armor) in armor_roots.iter() {
        // 这块板属于哪辆车：沿父链找，与 armor/collision.rs 里判定命中的写法一致。
        if !qq
            .child_of
            .iter_ancestors(plate)
            .any(|ancestor| ancestor == vehicle)
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
        let Some(center) = qq
            .of(plate)
            .suffix("CENTER")
            .any()
            .one()
            .or_else(|| qq.of(plate).suffix("CENTER").one())
        else {
            continue;
        };
        let Ok(center_tf) = transforms.get(center) else {
            continue;
        };

        let world = center_tf.translation();
        let d = world.distance_squared(camera_pos);
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, world));
        }
    }

    best.map(|(_, world)| to_ros_vec3(world))
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
    rune_query: Query<(
        &GlobalTransform,
        &Transform,
        &PowerRune,
        &PowerRuneMechanism,
        &PowerRuneRotation,
    )>,
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
                _pad: [0; 11],
            };
            batch.target_count += 1;
            slot_of.insert(vehicle, idx);
        }
    }

    if let Some(camera_pos) = camera_pos {
        for (vehicle, idx) in slot_of.iter() {
            if let Some(center_ros) =
                select_armor_center(*vehicle, camera_pos, &qq, &armor_roots, &transforms)
            {
                let t = &mut batch.targets[*idx];
                t.armor_position = [center_ros.x, center_ros.y, center_ros.z];
                t.armor_position_valid = 1;
            }
        }
    }

    // Collect rune ground truth
    for (global_tf, local_tf, power_rune, mechanism, rotation) in rune_query.iter() {
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
            _pad1: 0,
            r_center_odom: [pos_ros.x, pos_ros.y, pos_ros.z],
            radius: 0.0,
            current_angle,
            v_roll: 0.0,
            direction,
            sin_amplitude,
            sin_omega,
            sin_phase: 0.0,
            sin_offset,
            relative_time,
            blade_id: -1,
            target_activations,
            _pad: [0; 20],
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
        let mut plate = |world: &mut World, id: usize, at: Vec3| {
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
                    select_armor_center(vehicle, camera_pos, &qq, &armor_roots, &transforms)
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
}
