use crate::capture::{
    CameraFov, CaptureBundle, CaptureSource, ImageHandle, compute_camera_intrinsics,
    driver::{
        CaptureConfig, CaptureFrameId, CapturedFrame, CapturedFrameKind, GpuCaptureHandler,
        SnapshotAsync, SnapshotSync,
    },
    setup_capture_camera, setup_preview_window, sync_capture_camera,
};
use crate::components::{Controlled, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim};
use crate::systems::{ChassisObservationFrame, GameplaySystems};
use crate::talos::plugin::{to_ros_quat, to_ros_translation};
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;
use bevy::render::{Extract, ExtractSchedule, RenderApp, RenderSystems};
use std::f32::consts::PI;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::*;

static FRAME_SEQ: AtomicU64 = AtomicU64::new(0);

/// 本帧采集到的真值批次，等待与图像在**同一次发布事务**里提交。
///
/// 协议约定（v3）：真值批次与图像、同帧姿态三者的 `frame_seq` 严格相等，没有任何
/// 允许的偏移。实现方式是 [`ShmPublisher::publish_image_with`] 的 `before_commit`
/// 回调——图像 meta 是消费端唯一的提交标记，回调里发布的东西一定先于它可见，所以
/// "消费端看见图像 seq=k" 就蕴含 "真值槽位里已经是 seq=k"。
///
/// 之前的做法是攒一段真值历史（`GT_HISTORY_DEPTH`），再按"图像发到哪一帧了"回放
/// 同帧的那一批。它做不到同帧：回放系统在主世界的 `Last` 里跑，而图像是在渲染世界
/// 的回读回调里落地的，两者天然差一个节拍，实测 `seq_skew` 恒为 -1，
/// `seq_mismatches` 占 frames_ok 的 10~17%。加大历史深度对此毫无作用（深度 256
/// 实测 13.6%），因为成因不是"旧批次被挤掉"而是"提交时序差一拍"。
///
/// 真值只能流向评估器，绝不进入 YOLO/Solver/Tracker/Planner，这条边界不变。
#[derive(Resource, Default)]
pub struct TalosGroundTruthFrame {
    /// 采集这一批真值时的 `TalosFrameStamp::frame_seq`。
    pub frame_seq: u64,
    /// `None` = 本帧没有采集到真值（没有 TalosCaptureContext，或系统未运行）。
    /// 装箱是因为 `GroundTruthBatch` 有 1664 字节，不适合按值塞进每帧都要 clone
    /// 的 `ExtractedPoseData`。
    pub batch: Option<Box<GroundTruthBatch>>,
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct TalosFrameStamp {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
}

pub fn advance_talos_frame_stamp(mut stamp: ResMut<TalosFrameStamp>) {
    stamp.frame_seq = FRAME_SEQ.fetch_add(1, Ordering::Relaxed);
    stamp.timestamp_ns = now_ns();
}

/// Extracted pose data from MainApp to RenderApp for synchronized publishing
#[derive(Resource, Clone, Default)]
pub struct ExtractedPoseData {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pose: Option<CapturedPoseData>,
    /// 与 `pose` 同一个快照里的真值批次，随图像在同一次发布事务里提交。
    /// `None` = 本帧没采集到真值（评估侧会看到样本变少，但绝不会拿到错帧的真值）。
    ground_truth: Option<Box<GroundTruthBatch>>,
    pub valid: bool,
}

/// Pose data captured at frame snapshot time
#[derive(Clone)]
struct CapturedPoseData {
    /// 云台回转中心的世界位置（ROS 约定）。
    gimbal_ros: [f32; 3],
    /// 枪管的世界姿态 `world <- gimbal`（ROS 约定，`[w,x,y,z]`）。
    gimbal_quat: [f32; 4],
    /// 枪口的**世界**位置（ROS 约定）。协议 v3 起发布世界量，见 `PoseIndex::Muzzle`。
    muzzle_world: [f32; 3],
    /// 相机相对云台的**局部**平移（ROS 约定）。消费端用它与自己的
    /// `t_camera2gimbal` 外参自检，所以刻意保持局部。
    camera_rel: [f32; 3],
    chassis_observation: ChassisObservation,
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

struct TalosSnapshotSync {
    frame_seq: u64,
    timestamp_ns: u64,
    pose: CapturedPoseData,
    ground_truth: Option<Box<GroundTruthBatch>>,
}

impl SnapshotSync for TalosSnapshotSync {
    fn captured(
        self: Box<Self>,
        world: &mut DeferredWorld,
        _config: &CaptureConfig,
    ) -> Box<dyn SnapshotAsync> {
        let ctx = world.resource::<TalosCaptureContextShared>().0.clone();

        Box::new(TalosSnapshot {
            ctx,
            frame_seq: self.frame_seq,
            timestamp_ns: self.timestamp_ns,
            pose: self.pose,
            ground_truth: self.ground_truth,
        })
    }
}

struct TalosSnapshot {
    ctx: Arc<Mutex<ShmPublisher>>,
    frame_seq: u64,
    timestamp_ns: u64,
    pose: CapturedPoseData,
    ground_truth: Option<Box<GroundTruthBatch>>,
}

impl SnapshotAsync for TalosSnapshot {
    fn captured(&mut self, frame: CapturedFrame<'_>) {
        if frame.kind != CapturedFrameKind::Rgb8 {
            return;
        }

        let expected_size = (frame.width * frame.height * 3) as usize;
        if frame.data.len() != expected_size {
            warn!(
                "图像大小不匹配: expected {} bytes, got {} bytes",
                expected_size,
                frame.data.len()
            );
            return;
        }

        if frame.width != IMAGE_WIDTH || frame.height != IMAGE_HEIGHT {
            warn!(
                "image reesolution mismatched: expected {}x{}, got {}x{}",
                IMAGE_WIDTH, IMAGE_HEIGHT, frame.width, frame.height
            );
            return;
        }

        if let Ok(mut publisher) = self.ctx.lock() {
            // 图像、同帧姿态、同帧真值必须在同一次发布事务里提交。
            //
            // `before_commit` 在像素拷贝之后、图像 meta（消费端唯一的提交标记）之前
            // 运行，所以这里发布的一切都严格先于图像可见。加上背压握手
            // （`synchronized_frame_consumed`：图像与 poses[0..=Camera] 的 FLAG_NEW
            // 全部清掉才允许发下一帧），消费端"看见图像 seq=k"就蕴含"真值槽位里
            // 已经是 seq=k，且在本帧被消费完之前不会前进"。
            //
            // 事务失败（背压未放行 / 帧号未前进）时整个回调不执行，真值也就不会被
            // 提交去对齐一帧根本没发出去的图像。
            // 返回值故意丢弃：事务被背压挡住时"本帧不发"就是正确行为，没有额外
            // 动作可做。之前需要它是为了给真值回放记账（只有发成功才推进水位），
            // 现在真值就在这个事务里，成功与否与它天然一致。
            let _published = publisher.try_publish_synchronized_image(
                frame.data,
                self.frame_seq,
                self.timestamp_ns,
                |publisher| {
                    publish_pose_data(publisher, self.frame_seq, self.timestamp_ns, &self.pose);
                    if let Some(batch) = self.ground_truth.as_deref() {
                        publisher.publish_ground_truth(batch);
                    }
                },
            );
        }
    }
}

#[derive(Default)]
struct TalosSnapshotCreator {}

impl GpuCaptureHandler for TalosSnapshotCreator {
    fn captured(
        &self,
        world: &World,
        _frame_id: Option<CaptureFrameId>,
    ) -> Option<Box<dyn SnapshotSync>> {
        // Timestamp, frame sequence and pose must come from the same ExtractSchedule snapshot.
        let extracted = world.get_resource::<ExtractedPoseData>()?;
        if !extracted.valid {
            return None;
        }
        let pose = extracted.pose.clone()?;

        Some(Box::new(TalosSnapshotSync {
            frame_seq: extracted.frame_seq,
            timestamp_ns: extracted.timestamp_ns,
            pose,
            ground_truth: extracted.ground_truth.clone(),
        }))
    }
}

#[derive(Resource, Clone, Deref, DerefMut)]
pub struct TalosCaptureContextShared(pub Arc<Mutex<ShmPublisher>>);

#[derive(Resource, Clone)]
pub struct TalosCaptureContext {
    pub publisher: Arc<Mutex<ShmPublisher>>,
    pub fov_y: f32,
}

pub struct TalosCapturePlugin {
    pub config: CaptureConfig,
    pub context: TalosCaptureContext,
}

pub fn publish_talos_runtime_state_system(
    context: Option<Res<TalosCaptureContext>>,
    frame_stamp: Res<TalosFrameStamp>,
    following: Res<SubscribeAutoAim>,
) {
    let Some(ctx) = context else {
        return;
    };

    if let Ok(mut publisher) = ctx.publisher.lock() {
        publisher.publish_runtime_state(RuntimeState {
            timestamp_ns: frame_stamp.timestamp_ns,
            following: u8::from(following.load(Ordering::Acquire)),
            _pad: [0; 55],
        });
    }
}

impl Plugin for TalosCapturePlugin {
    fn build(&self, app: &mut App) {
        let capture = CaptureBundle::color(
            app,
            self.config.clone(),
            vec![Box::new(TalosSnapshotCreator::default())],
        );
        let render_target_handle = capture.color_target().unwrap().clone();

        {
            let mut publisher = self.context.publisher.lock().unwrap();
            let intrinsics = compute_camera_intrinsics(
                self.config.width,
                self.config.height,
                self.context.fov_y,
            );

            publisher.set_camera_info(CameraInfo {
                timestamp_ns: now_ns(),
                fx: intrinsics.fx,
                fy: intrinsics.fy,
                cx: intrinsics.cx,
                cy: intrinsics.cy,
                distortion: [0.0; 5],
                width: intrinsics.width,
                height: intrinsics.height,
                _pad: [0; 24],
            });
        }

        app.add_plugins(capture)
            .init_resource::<TalosGroundTruthFrame>()
            .insert_resource(ImageHandle(render_target_handle))
            .insert_resource(CameraFov(self.context.fov_y))
            .insert_resource(self.context.clone())
            .add_systems(Startup, setup_capture_camera)
            .add_systems(Startup, setup_preview_window)
            .add_systems(
                Update,
                sync_capture_camera
                    .after(GameplaySystems::Camera)
                    .before(RenderSystems::Render),
            );

        app.sub_app_mut(RenderApp)
            .insert_resource(TalosCaptureContextShared(self.context.publisher.clone()))
            .insert_resource(self.context.clone())
            .insert_resource(ExtractedPoseData::default())
            .add_systems(ExtractSchedule, extract_pose_data);
    }
}

/// Extract pose data from MainApp to RenderApp
fn extract_pose_data(
    mut pose_data: ResMut<ExtractedPoseData>,
    frame_stamp: Extract<Res<TalosFrameStamp>>,
    camera: Extract<Query<&GlobalTransform, With<CaptureSource>>>,
    gimbal: Extract<Query<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>>,
    muzzle_offset: Extract<
        Query<(&GlobalTransform, &Transform), (With<InfantryLaunchOffset>, With<Controlled>)>,
    >,
    chassis_obs: Extract<Res<ChassisObservationFrame>>,
    ground_truth: Extract<Res<TalosGroundTruthFrame>>,
) {
    pose_data.frame_seq = frame_stamp.frame_seq;
    pose_data.timestamp_ns = frame_stamp.timestamp_ns;

    // 真值必须来自**同一个** frame_seq 的采集。真值采集系统在主世界的 `Last` 里跑、
    // 本函数在紧随其后的 ExtractSchedule 里跑，正常情况下两者帧号相等；一旦不等
    // （采集系统没跑、或调度被改动过）就当作本帧没有真值，绝不发一个错帧的批次。
    pose_data.ground_truth = if ground_truth.frame_seq == frame_stamp.frame_seq {
        ground_truth.batch.clone()
    } else {
        None
    };

    let Ok(cam_transform) = camera.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };
    let Ok(gimbal_transform) = gimbal.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };
    let Ok((muzzle_global, muzzle_local)) = muzzle_offset.single() else {
        pose_data.pose = None;
        pose_data.valid = false;
        return;
    };

    pose_data.pose = Some(captured_pose_data(
        cam_transform,
        gimbal_transform,
        muzzle_global,
        muzzle_local,
        &chassis_obs,
        pose_data.frame_seq,
        pose_data.timestamp_ns,
    ));
    pose_data.valid = true;
}

fn captured_pose_data(
    cam_transform: &GlobalTransform,
    gimbal_transform: &GlobalTransform,
    muzzle_global: &GlobalTransform,
    muzzle_local: &Transform,
    chassis_obs: &ChassisObservationFrame,
    frame_seq: u64,
    timestamp_ns: u64,
) -> CapturedPoseData {
    let cam_rel = cam_transform.reparented_to(gimbal_transform);

    let gimbal_rot = gimbal_transform.rotation()
        * muzzle_local.rotation
        * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0);

    let gimbal_ros = to_ros_translation(gimbal_transform.translation());
    let gimbal_rot = to_ros_quat(gimbal_rot);
    // 直接发枪口的世界位置，不发 reparented_to(gimbal) 的局部平移。
    //
    // SHOT_DIRECTION 是 GIMBAL 的子节点（见 setup.rs），所以 muzzle_global 已经是
    // 枪口在世界系里的真实位置，这里零成本可得。发局部量的代价是消费端会写出
    // `odom + muzzle`——把未经云台旋转的局部平移加到世界坐标上，yaw=90° 时那 0.11 m
    // 的局部 +X 实际指向世界 +Y，误差等于偏移量全长。
    let muzzle = to_ros_translation(muzzle_global.translation());
    let camera = to_ros_translation(cam_rel.translation);

    CapturedPoseData {
        gimbal_ros: [gimbal_ros.x, gimbal_ros.y, gimbal_ros.z],
        gimbal_quat: [gimbal_rot.w, gimbal_rot.x, gimbal_rot.y, gimbal_rot.z],
        muzzle_world: [muzzle.x, muzzle.y, muzzle.z],
        camera_rel: [camera.x, camera.y, camera.z],
        chassis_observation: ChassisObservation {
            frame_seq,
            timestamp_ns,
            dt_s: chassis_obs.dt_s,
            v_body: [chassis_obs.v_body.x, chassis_obs.v_body.y],
            wz_radps: chassis_obs.wz_radps,
            wheel_linear_mps: chassis_obs.wheel_linear_mps,
            wheel_angular_radps: chassis_obs.wheel_angular_radps,
            a_body: [chassis_obs.a_body.x, chassis_obs.a_body.y],
            alpha_z_radps2: chassis_obs.alpha_z_radps2,
            rpy_rad: [
                chassis_obs.rpy_rad.x,
                chassis_obs.rpy_rad.y,
                chassis_obs.rpy_rad.z,
            ],
            gyro_xyz_radps: [
                chassis_obs.gyro_xyz_radps.x,
                chassis_obs.gyro_xyz_radps.y,
                chassis_obs.gyro_xyz_radps.z,
            ],
            accel_xyz_mps2: [
                chassis_obs.accel_xyz_mps2.x,
                chassis_obs.accel_xyz_mps2.y,
                chassis_obs.accel_xyz_mps2.z,
            ],
            _pad: [0; 16],
        },
    }
}

fn publish_pose_data(
    publisher: &mut ShmPublisher,
    frame_seq: u64,
    timestamp_ns: u64,
    pose: &CapturedPoseData,
) {
    publisher.publish_pose(
        PoseIndex::Odom,
        pose.gimbal_ros,
        [1.0, 0.0, 0.0, 0.0],
        frame_seq,
        timestamp_ns,
    );

    publisher.publish_pose(
        PoseIndex::Gimbal,
        [0.0, 0.0, 0.0],
        pose.gimbal_quat,
        frame_seq,
        timestamp_ns,
    );

    // 枪口：世界位置 + 世界姿态。姿态与 Gimbal 通道同一个量（枪管姿态），
    // 这样消费端拿 Muzzle 一个通道就能构造完整的出膛射线，不必再去拼别的通道。
    publisher.publish_pose(
        PoseIndex::Muzzle,
        pose.muzzle_world,
        pose.gimbal_quat,
        frame_seq,
        timestamp_ns,
    );

    publisher.publish_pose(
        PoseIndex::Camera,
        pose.camera_rel,
        [1.0, 0.0, 0.0, 0.0],
        frame_seq,
        timestamp_ns,
    );

    let mut observation = pose.chassis_observation;
    observation.frame_seq = frame_seq;
    observation.timestamp_ns = timestamp_ns;
    publisher.publish_chassis_observation(observation);

    // Legacy compatibility path for consumers still reading pose slot 4.
    publisher.publish_pose_with_aux(
        PoseIndex::ChassisObservation,
        [
            observation.v_body[0],
            observation.v_body[1],
            observation.wz_radps,
        ],
        observation.wheel_angular_radps,
        [
            observation.a_body[0],
            observation.a_body[1],
            observation.alpha_z_radps2,
            observation.dt_s,
        ],
        frame_seq,
        timestamp_ns,
    );
}
