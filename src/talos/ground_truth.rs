use crate::capture::CaptureSource;
use crate::components::{Controlled, Infantry};
use crate::robomaster::prelude::{
    Activation, Armor, ArmorRoot, MechanismState, PowerRune, PowerRuneMechanism, PowerRuneRotation,
    RuneMode, Team,
};
use crate::talos::capture::{NO_PUBLISHED_IMAGE, TalosCaptureContext, TalosFrameStamp};
use crate::talos::plugin::M_ALIGN_MAT3;
use crate::util::entity_query::HierarchyQuery;
use avian3d::prelude::AngularVelocity;
use bevy::prelude::*;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::Ordering;
use talos_ipc::*;

/// 真值历史深度。图像要等 GPU 回读才进共享内存，比这里的当前帧晚若干帧；
/// 深度不够时"已发布图像的同帧真值"会被挤掉，评估直接少样本。单条
/// GroundTruthBatch 1664 字节，64 帧约 106KB，代价可以忽略，所以留足余量。
/// 真的不够时下面会打 warn 而不是静默丢弃。
///
/// 实测：这个深度**不是**评估丢样本的原因，不要为了消除下面那条 warn 去加大它。
///   深度 64  -> seq_mismatches 占 frames_ok 的 10~17%
///   深度 256 -> 13.6%（60s，485 帧），没有改善
/// 真正的成因是发布时序差一拍：消费侧新增的 seq_skew 统计给出 min=-1 max=+2
/// mean=+0.88 帧，即真值与图像基本同帧、只差一个发布节拍，而不是"旧了几十帧被
/// 挤掉"（那样 skew 会是很大的负数）。要修得动发布端的提交顺序，属于协议时序
/// 改动，本次不做，只把可观测量补齐。
///
/// 另外那条溢出 warn 本身也会误导：history 每个 Bevy 帧都 push，而只有图像落地
/// (约 8fps) 才会推进水位并修剪，所以稳态下必然持续淘汰（实测 ~7~10 次/s，
/// 有无消费者都一样）。它是结构性的正常现象，不是故障。
const GT_HISTORY_DEPTH: usize = 64;

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
        let center = qq
            .of(plate)
            .suffix("CENTER")
            .any()
            .one()
            .or_else(|| qq.of(plate).suffix("CENTER").one())?;
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

pub fn publish_ground_truth_system(
    context: Option<Res<TalosCaptureContext>>,
    frame_stamp: Res<TalosFrameStamp>,
    mut history: Local<VecDeque<GroundTruthBatch>>,
    mut last_published_seq: Local<Option<u64>>,
    mut evicted_batches: Local<u64>,
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
    let Some(ctx) = context else {
        return;
    };

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

    // 攒一段历史，只发布"与已落地图像同帧"的那一批。
    //
    // 直接发当前帧的真值是错的：图像要等 GPU 回读，实测比这里晚约 2 帧，等它进
    // 共享内存时真值槽位早被后两帧覆盖了。消费侧 GroundTruthEvaluator::fetch()
    // 要求 gt.frame_seq == image.frame_seq（这是对的，评估必须同帧），于是恒不
    // 命中，--eval 一个样本都取不到。这里改成按图像实际发布进度回放真值，
    // 共享内存布局和 ABI 版本都不用动。
    //
    // 真值只流向评估器，不进算法输入，这条边界不变。
    history.push_back(batch);
    while history.len() > GT_HISTORY_DEPTH {
        // 被挤掉的批次里可能就有下一次要匹配的那一帧。这里计数并按指数间隔告警，
        // 否则历史深度不够只会表现为"评估样本莫名变少"，很难查。
        history.pop_front();
        *evicted_batches += 1;
        if evicted_batches.is_power_of_two() {
            // 降级成 debug：稳态下必然持续淘汰（push 每 Bevy 帧、修剪只在图像
            // 落地时，约 8fps），warn 会让人以为是故障并去加大深度——实测加到
            // 256 对 seq_mismatches 没有任何改善（见 GT_HISTORY_DEPTH 的说明）。
            debug!(
                "真值历史淘汰 {} 次 (深度 {})：稳态下正常。评估少样本请看消费侧 \
                 seq_skew（差一拍是发布时序，负得很大才是深度不够）",
                *evicted_batches, GT_HISTORY_DEPTH
            );
        }
    }

    let published_seq = ctx.published_image_seq.load(Ordering::Acquire);
    match select_ground_truth(&mut history, *last_published_seq, published_seq) {
        GtSelection::Wait => {}
        GtSelection::Skip => *last_published_seq = Some(published_seq),
        GtSelection::Publish => {
            // Publish 的定义保证了 front() 存在且帧号相等。
            let matched = history
                .front()
                .expect("GtSelection::Publish 保证 front 存在");
            if let Ok(mut publisher) = ctx.publisher.lock() {
                publisher.publish_ground_truth(matched);
                *last_published_seq = Some(published_seq);
            }
        }
    }
}

/// `select_ground_truth` 的判定结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GtSelection {
    /// `history.front()` 与已落地图像同帧，发布它并推进水位。
    Publish,
    /// 历史里没有这一帧（发布端跳帧，或深度不足被挤掉）。只推进水位，
    /// 否则下一帧会反复重扫同一个洞。
    Skip,
    /// 尚无图像落地，或这个帧号已经发过。什么都不做。
    Wait,
}

/// 挑出"与已落地图像同帧"的那一批真值，并顺带修剪掉再也匹配不上的旧批次。
///
/// 抽成纯函数是为了能直接测三种边界：历史深度不足、发布端跳帧、以及同一帧号
/// 被反复看到。这些在 Bevy system 里要靠 `Local` 状态复现，测起来很别扭。
fn select_ground_truth(
    history: &mut VecDeque<GroundTruthBatch>,
    last_published_seq: Option<u64>,
    published_seq: u64,
) -> GtSelection {
    // 还没有任何图像真正落地。哨兵不能用 0：frame_seq 从 0 开始（见
    // NO_PUBLISHED_IMAGE 的说明），用 0 会把第 0 帧的真值永久吞掉。
    if published_seq == NO_PUBLISHED_IMAGE {
        return GtSelection::Wait;
    }
    // 图像帧率(约 8fps)远低于调用频率，同一帧号会反复看到；只发一次。
    if last_published_seq == Some(published_seq) {
        return GtSelection::Wait;
    }
    // 丢掉比已发布图像更旧的，它们再也不会被匹配上。
    while history.front().is_some_and(|b| b.frame_seq < published_seq) {
        history.pop_front();
    }
    if history
        .front()
        .is_some_and(|b| b.frame_seq == published_seq)
    {
        GtSelection::Publish
    } else {
        GtSelection::Skip
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn batch(frame_seq: u64) -> GroundTruthBatch {
        GroundTruthBatch {
            frame_seq,
            ..Default::default()
        }
    }

    fn history(seqs: &[u64]) -> VecDeque<GroundTruthBatch> {
        seqs.iter().copied().map(batch).collect()
    }

    #[test]
    fn frame_seq_zero_is_publishable_not_a_sentinel() {
        // FRAME_SEQ.fetch_add 返回自增前的值，所以第一帧的 frame_seq 真的是 0。
        // 旧代码用 `published_seq == 0` 当"还没发布"的哨兵，会把第 0 帧的真值
        // 永久吞掉。哨兵必须是 NO_PUBLISHED_IMAGE。
        let mut h = history(&[0, 1, 2]);
        assert_eq!(select_ground_truth(&mut h, None, 0), GtSelection::Publish);
        assert_eq!(h.front().unwrap().frame_seq, 0);

        let mut h = history(&[0, 1]);
        assert_eq!(
            select_ground_truth(&mut h, None, NO_PUBLISHED_IMAGE),
            GtSelection::Wait,
            "只有 NO_PUBLISHED_IMAGE 才表示尚无图像落地"
        );
        assert_eq!(h.len(), 2, "Wait 不得修剪历史");
    }

    #[test]
    fn same_published_seq_publishes_once() {
        // 真值系统的调用频率远高于图像发布率，同一帧号会被看到很多次。
        let mut h = history(&[7, 8]);
        assert_eq!(select_ground_truth(&mut h, None, 7), GtSelection::Publish);
        assert_eq!(
            select_ground_truth(&mut h, Some(7), 7),
            GtSelection::Wait,
            "同一帧号第二次必须什么都不做"
        );
        assert_eq!(h.front().unwrap().frame_seq, 7, "Wait 不得弹出已发布的批次");
    }

    #[test]
    fn publisher_frame_skip_advances_watermark() {
        // 发布端丢了 8..=11 只发出 12：历史里 12 之前的都该被修剪掉，且因为
        // 12 本身还没入历史（图像比真值早到的极端情况），结果是 Skip 而非 Publish。
        let mut h = history(&[8, 9, 10, 11]);
        assert_eq!(select_ground_truth(&mut h, Some(7), 12), GtSelection::Skip);
        assert!(h.is_empty(), "比已发布图像更旧的批次全部修剪");

        // Skip 之后水位推进，同一个洞不会被反复重扫。
        assert_eq!(select_ground_truth(&mut h, Some(12), 12), GtSelection::Wait);
    }

    #[test]
    fn insufficient_history_depth_skips_instead_of_mismatching() {
        // 历史深度不足时目标帧已被挤掉，剩下的都比它新。绝不能拿一个更新的
        // 批次冒充同帧真值 —— 消费端的 fetch() 只比帧号，配错就是静默错数据。
        let mut h = history(&[20, 21, 22]);
        assert_eq!(
            select_ground_truth(&mut h, Some(18), 19),
            GtSelection::Skip,
            "目标帧被挤掉时必须 Skip"
        );
        assert_eq!(h.len(), 3, "更新的批次不得被误删，它们还要匹配后续图像");
        assert_eq!(h.front().unwrap().frame_seq, 20);
    }

    #[test]
    fn history_depth_bound_is_the_publish_lag_limit() {
        // 深度 GT_HISTORY_DEPTH 能容忍的最大图像滞后就是 GT_HISTORY_DEPTH-1 帧：
        // 队尾是当前帧，队首是最旧的仍在册帧。刚好落在边界上要能命中。
        let seqs: Vec<u64> = (0..GT_HISTORY_DEPTH as u64).collect();
        let mut h = history(&seqs);
        assert_eq!(select_ground_truth(&mut h, None, 0), GtSelection::Publish);

        // 再滞后一帧就落到窗口外了。
        let seqs: Vec<u64> = (1..=GT_HISTORY_DEPTH as u64).collect();
        let mut h = history(&seqs);
        assert_eq!(select_ground_truth(&mut h, None, 0), GtSelection::Skip);
    }

    #[test]
    fn empty_history_skips() {
        let mut h: VecDeque<GroundTruthBatch> = VecDeque::new();
        assert_eq!(select_ground_truth(&mut h, None, 5), GtSelection::Skip);
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
