use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::capture::{IMAGE_HEIGHT, IMAGE_WIDTH};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::systems::projectile_launch;
use crate::talos::capture::{
    TalosCaptureContext, TalosCapturePlugin, TalosFrameStamp, advance_talos_frame_stamp,
    publish_talos_runtime_state_system,
};
use crate::talos::link::{AutoAimLink, LinkState, LinkVerdict};
use bevy::ecs::system::RunSystemOnce;
use bevy::image::BevyDefault;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::*;

#[derive(Resource)]
pub struct ShmSubscriberRes(pub Arc<Mutex<ShmSubscriber>>);

#[derive(Resource, Deref, DerefMut)]
pub struct TalosEnabled(pub AtomicBool);

pub struct TalosPluginConfig {
    pub width: u32,
    pub height: u32,
    pub fov_y: f32,
    pub texture_format: TextureFormat,
}

impl Default for TalosPluginConfig {
    fn default() -> Self {
        let config = SimulationConfig::default();
        Self {
            width: IMAGE_WIDTH,
            height: IMAGE_HEIGHT,
            fov_y: config.camera.fov.to_radians(),
            texture_format: TextureFormat::Rgba8UnormSrgb,
        }
    }
}

#[derive(Default)]
pub struct TalosPlugin {
    pub config: TalosPluginConfig,
}

impl Plugin for TalosPlugin {
    fn build(&self, app: &mut App) {
        let publisher = match ShmPublisher::create() {
            Ok(p) => {
                info!("talos shm created");
                p
            }
            Err(e) => {
                error!("cannot create talos shm: {}", e);
                return;
            }
        };

        let publisher = Arc::new(Mutex::new(publisher));

        let capture_config = CaptureConfig {
            width: self.config.width,
            height: self.config.height,
            texture_format: self.config.texture_format,
            frame_kind: CapturedFrameKind::Rgb8,
        };

        let capture_context = TalosCaptureContext {
            publisher: publisher.clone(),
            fov_y: self.config.fov_y,
        };

        app.init_resource::<TalosFrameStamp>();

        app.add_plugins(TalosCapturePlugin {
            config: capture_config,
            context: capture_context,
        });

        match ShmSubscriber::connect() {
            Ok(subscriber) => {
                info!("connected to talos-cpp");
                app.insert_resource(ShmSubscriberRes(Arc::new(Mutex::new(subscriber))));
            }
            Err(_) => {
                info!("could not connect to talos-cpp");
            }
        }

        app.insert_resource(TalosEnabled(AtomicBool::new(true)));
        app.init_resource::<AutoAimLink>();
        app.add_systems(Last, (advance_talos_frame_stamp, heartbeat_system));
        app.add_systems(
            Last,
            publish_talos_runtime_state_system.after(advance_talos_frame_stamp),
        );
        // 真值采集必须在帧戳自增之后：它写进 TalosGroundTruthFrame 的 frame_seq
        // 要与本帧图像、同帧姿态用的是同一个号，紧随其后的 ExtractSchedule 会核对。
        app.add_systems(
            Last,
            crate::talos::ground_truth::collect_ground_truth_system
                .after(advance_talos_frame_stamp)
                .after(publish_talos_runtime_state_system),
        );
        // 租约推进无条件每帧跑：没订阅、没命令的帧同样要让状态机前进，否则
        // "对端断了"这件事只有等下一条命令到达才会被发现——而它可能永远不来。
        app.add_systems(Last, advance_link_lease);
        app.add_systems(
            Last,
            process_subscription
                .after(advance_link_lease)
                .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
        );
    }
}

/// 每帧推进租约与订阅开关。`process_subscription` 之前跑。
fn advance_link_lease(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    subscribed: Res<SubscribeAutoAim>,
    mut link: ResMut<AutoAimLink>,
    mut logged: Local<Option<LinkState>>,
) {
    link.tick(
        subscribed.load(Ordering::Acquire),
        time.elapsed_secs(),
        &config.auto_aim,
    );

    // 与上一次**打过日志**的状态比，而不是与本次 tick 之前的状态比。
    //
    // 状态机有两个驱动源：这里的 tick（租约到期）和 process_subscription 里的
    // ingest（收到命令）。只比 tick 前后的话，ingest 那一半的跃迁全部不进日志，
    // 结果是日志里只看得到"接管 -> 待命"、看不到"待命 -> 接管"，读起来像是链路
    // 掉了再也没回来。这个系统每帧都跑且排在消费之前，所以上一帧 ingest 造成的
    // 变化会在这一帧被记上。
    let after = link.state();
    let before = logged.unwrap_or(LinkState::Unsubscribed);
    if *logged != Some(after) {
        *logged = Some(after);
        info!(
            "外部自瞄链路: {} -> {}（收到 {} 条 / 接受 {} / 拒收 {}）",
            before.label(),
            after.label(),
            link.cmd_count,
            link.accepted,
            link.rejects.total()
        );
    }
}

fn process_subscription(
    context: Option<Res<ShmSubscriberRes>>,
    mut commands: Commands,
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut link: ResMut<AutoAimLink>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (
            With<Controlled>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    muzzle_offset: Single<
        (&GlobalTransform, &Transform),
        (With<InfantryLaunchOffset>, With<Controlled>),
    >,
) {
    let Some(ctx) = context else {
        return;
    };
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    let Some(cmd) = recv_gimbal_cmd(&ctx) else {
        return;
    };

    let now_secs = time.elapsed_secs();
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);

    let verdict = link.ingest(&cmd, now_secs, now_ns, &config.auto_aim);
    let fire = match verdict {
        LinkVerdict::Control { fire } => fire,
        // 安全停止：续租约，不动云台。
        LinkVerdict::SafeStop => return,
        LinkVerdict::Reject(reason) => {
            warn!(
                "丢弃外部云台命令（{}）: yaw={} pitch={} distance={} ts={}",
                reason.label(),
                cmd.yaw_deg,
                cmd.pitch_deg,
                cmd.distance_m,
                cmd.timestamp_ns
            );
            return;
        }
    };

    // 开火许可来自状态机，不来自 `fire_advice` 单独判断：租约外、被拒的命令一律
    // 到不了这里，而 `allows_fire()` 保证只有"接管中"才可能发弹。
    if fire && link.state().allows_fire() {
        commands.queue(|w: &mut World| {
            w.run_system_once(projectile_launch).unwrap();
        });
    }

    let yaw_f32 = (cmd.yaw_deg).to_radians();
    let pitch_f32 = (-cmd.pitch_deg - 90.0).to_radians();
    gimbal_data.local_yaw = yaw_f32;
    gimbal_data.pitch = pitch_f32;
    let expected_rotation = Quat::from_euler(EulerRot::YXZ, yaw_f32, pitch_f32, 0.0);
    let current_rotation = muzzle_offset.0.rotation();
    let delta = expected_rotation * current_rotation.inverse();
    gimbal_transform.rotation = delta * gimbal_transform.rotation;
}

fn heartbeat_system(context: Option<Res<TalosCaptureContext>>) {
    if let Some(ctx) = context {
        if let Ok(mut publisher) = ctx.publisher.lock() {
            publisher.update_heartbeat();
        }
    }
}

pub fn publish_pose(
    context: &TalosCaptureContext,
    index: PoseIndex,
    position: [f32; 3],
    quaternion: [f32; 4],
    frame_seq: u64,
    timestamp_ns: u64,
) {
    if let Ok(mut publisher) = context.publisher.lock() {
        publisher.publish_pose(index, position, quaternion, frame_seq, timestamp_ns);
    }
}

pub fn recv_gimbal_cmd(subscriber: &ShmSubscriberRes) -> Option<GimbalCmd> {
    subscriber.0.lock().ok()?.recv_gimbal_cmd()
}

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn to_ros(bevy_transform: Transform) -> Transform {
    let new_rotation = to_ros_quat(bevy_transform.rotation);
    let new_translation = to_ros_translation(bevy_transform.translation);
    Transform::from_translation(new_translation).with_rotation(new_rotation)
}

pub fn to_ros_translation(vec3: Vec3) -> Vec3 {
    let align_rot_mat = M_ALIGN_MAT3;
    let new_translation = align_rot_mat * vec3;
    new_translation
}

pub fn to_ros_quat(quat: Quat) -> Quat {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * quat * align_quat.inverse();
    new_rotation
}
