use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::capture::{IMAGE_HEIGHT, IMAGE_WIDTH};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::systems::GameplaySystems;
use crate::systems::projectile_launch;
use crate::talos::capture::{
    TalosCaptureContext, TalosCapturePlugin, TalosFrameStamp, advance_talos_frame_stamp,
    publish_talos_runtime_state_system,
};
use crate::talos::link::{AutoAimLink, LinkState, LinkVerdict};
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy::transform::TransformSystems;
use bevy::transform::helper::TransformHelper;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::*;

#[derive(Resource)]
pub struct ShmSubscriberRes(pub Arc<Mutex<ShmSubscriber>>);

#[derive(Resource, Deref, DerefMut)]
pub struct TalosEnabled(pub AtomicBool);

/// A command accepted during Update must launch only after this frame's hierarchy propagation.
/// A bool is sufficient because the shared-memory consumer deliberately accepts at most the
/// latest command once per frame.
#[derive(Resource, Default)]
struct ExternalFireRequest(bool);

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
        app.init_resource::<ExternalFireRequest>();

        // External commands must change local transforms before the single production
        // TransformSystems::Propagate pass.  The former Last-stage placement was after that pass,
        // so a rotating chassis left rendering, projectile launch and Talos poses one frame old.
        app.add_systems(
            Update,
            (
                advance_link_lease,
                process_subscription
                    .after(advance_link_lease)
                    .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
            )
                .chain()
                .after(GameplaySystems::Input)
                .before(GameplaySystems::GameLogic),
        );
        app.add_systems(
            PostUpdate,
            launch_external_projectile.after(TransformSystems::Propagate),
        );
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

/// `gimbal_cmd` 的 yaw/pitch 是**世界（odom）系**的绝对角，不是云台局部角。
///
/// 依据在视觉侧：solver/planner 在 odom 下解算；`SimGimbal::update()` 直接取协议 v3 的
/// 枪口世界四元数，按 "intrinsic ZYX -> [yaw, pitch, roll]" 分解出反馈
/// （`simulation/io/sim_gimbal.cpp:137-144`），`sim_internal_yaw_rad` /
/// `sim_internal_pitch_rad`（同文件 :302-311）又是本函数的镜像。两端是同一套绝对角约定，
/// 所以这里算出来的是"枪口应该有的世界旋转"，而不是云台该转多少。
fn world_aim_rotation(yaw_deg: f32, pitch_deg: f32) -> Quat {
    Quat::from_euler(
        EulerRot::YXZ,
        yaw_deg.to_radians(),
        (-pitch_deg - 90.0).to_radians(),
        0.0,
    )
}

/// 把"枪口的目标世界旋转"换算成云台自己的**局部**旋转。
///
/// 层级是 父节点（车体/底盘）-> 云台 -> …… -> 枪口（`SHOT_DIRECTION`），于是
/// `muzzle_world = parent_world * gimbal_local * mount`，其中 `mount` 是枪口相对云台的
/// 固定安装旋转（场景里不变，用当前姿态反解出来即可）。要让枪口落到 `target_world`：
///
/// ```text
/// gimbal_local' = parent_world⁻¹ * target_world * mount⁻¹
/// ```
///
/// 父节点旋转是单位四元数时它退化成旧写法 `target_world * muzzle_world⁻¹ * gimbal_local`，
/// 这也是为什么静止且未转向的底盘上看不出问题；底盘一转，缺掉的
/// `parent_world⁻¹ … parent_world` 共轭就会把世界旋转错当局部旋转。
fn gimbal_local_rotation(
    target_world: Quat,
    parent_world: Quat,
    gimbal_local: Quat,
    muzzle_world: Quat,
) -> Quat {
    let mount = (parent_world * gimbal_local).inverse() * muzzle_world;
    (parent_world.inverse() * target_world * mount.inverse()).normalize()
}

/// 把一条外部云台命令落到云台的局部 `Transform` 上。
///
/// 单独抽出来是为了让"底盘转起来以后还对不对"能在测试里走**同一条**路径：
/// 父节点是通过 `ChildOf` 查到的，层级多深都无所谓。
fn aim_gimbal_at(
    cmd_yaw_deg: f32,
    cmd_pitch_deg: f32,
    gimbal_transform: &mut Transform,
    gimbal_data: &mut InfantryGimbal,
    parent_world: Quat,
    muzzle_world: Quat,
) {
    let target_world = world_aim_rotation(cmd_yaw_deg, cmd_pitch_deg);
    let local = gimbal_local_rotation(
        target_world,
        parent_world,
        gimbal_transform.rotation,
        muzzle_world,
    );
    gimbal_transform.rotation = local;
    // 这两个字段是**局部**角缓存：`gimbal_controls` 会从局部 Transform 重新分解一次，
    // 把世界角写进去会让手动接管的第一帧跳变。
    let (local_yaw, local_pitch, _) = local.to_euler(EulerRot::YXZ);
    gimbal_data.local_yaw = local_yaw;
    gimbal_data.pitch = local_pitch;
}

fn process_subscription(
    context: Option<Res<ShmSubscriberRes>>,
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut link: ResMut<AutoAimLink>,
    mut transforms: ParamSet<(
        TransformHelper,
        Query<
            (&mut Transform, &mut InfantryGimbal, Option<&ChildOf>),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
        Query<(Entity, &GlobalTransform), (With<InfantryLaunchOffset>, With<Controlled>)>,
    )>,
    mut fire_request: ResMut<ExternalFireRequest>,
    mut fired: Local<u64>,
) {
    let Some(ctx) = context else {
        return;
    };
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
        // Do not run projectile_launch here: Update precedes TransformSystems::Propagate, so its
        // GlobalTransform query would still describe the previous chassis pose.  The PostUpdate
        // wrapper consumes this request after propagation.
        fire_request.0 = true;
        // 发弹次数只存在 HUD 的 `ProjectileStatistics` 里。脚本化/无头跑闭环时
        // 没人能看 HUD，于是"外部火控到底有没有真的开火"在报告里只能靠视觉侧
        // 自己数的 fire 命令数去推断——而那个数只说明命令发出去了。这里按次数
        // 打点（第 1 发 + 每 50 发），让日志本身成为发弹的证据。
        // 不每发都打：闭环 60s 能打 200 发以上，逐发打会把日志刷满。
        *fired += 1;
        if *fired == 1 || *fired % 50 == 0 {
            info!(
                "外部火控开火：第 {} 发（链路状态 {}）",
                *fired,
                link.state().label()
            );
        }
    }

    let parent_entity = {
        let mut gimbals = transforms.p1();
        let (_, _, gimbal_parent) = gimbals
            .single_mut()
            .expect("controlled gimbal query must match exactly one entity");
        gimbal_parent.map(ChildOf::parent)
    };
    let (muzzle_entity, muzzle_previous) = {
        let muzzles = transforms.p2();
        let (entity, global) = muzzles
            .single()
            .expect("controlled muzzle query must match exactly one entity");
        (entity, *global)
    };
    let helper = transforms.p0();
    let parent_world = parent_entity
        .and_then(|entity| helper.compute_global_transform(entity).ok())
        .map(|global| global.rotation())
        .unwrap_or(Quat::IDENTITY);
    let muzzle_world = helper
        .compute_global_transform(muzzle_entity)
        .unwrap_or(muzzle_previous)
        .rotation();
    let mut gimbals = transforms.p1();
    let (mut gimbal_transform, mut gimbal_data, _) = gimbals
        .single_mut()
        .expect("controlled gimbal query must match exactly one entity");

    aim_gimbal_at(
        cmd.yaw_deg,
        cmd.pitch_deg,
        &mut gimbal_transform,
        &mut gimbal_data,
        parent_world,
        muzzle_world,
    );
}

fn launch_external_projectile(
    mut fire_request: ResMut<ExternalFireRequest>,
    mut commands: Commands,
) {
    if !std::mem::take(&mut fire_request.0) {
        return;
    }
    commands.queue(|world: &mut World| {
        world.run_system_once(projectile_launch).unwrap();
    });
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{CaptureCamera, CaptureSource, sync_capture_camera};
    use crate::components::{
        CameraMode, FollowingType, Infantry, InfantryViewOffset, MainCamera, ProjectileCooldown,
        ProjectileSetting,
    };
    use crate::config::SimulationConfig;
    use crate::robomaster::prelude::Projectile;
    use crate::robomaster::prelude::{INFANTRY_THREE_CONFIG, Team};
    use crate::statistic::ProjectileStatistics;
    use crate::systems::{ChassisObservationFrame, update_camera_follow};
    use crate::talos::capture::{captured_pose_data, publish_pose_data};
    use avian3d::prelude::{AngularVelocity, LinearVelocity};
    use bevy::input::gamepad::GamepadRumbleRequest;
    use bevy::transform::TransformPlugin;
    use std::sync::atomic::AtomicU64;
    use talos_ipc::{GimbalCmd, PoseIndex, PoseMeta, ShmPublisher, ShmSubscriber};

    /// SHOT_DIRECTION / CAM_DIRECTION 相对 GIMBAL 的固定安装角。真实场景里是绕局部
    /// X 轴 -65°（见 `systems/camera.rs` 里那段说明），这里照抄，好让"安装角非零"
    /// 这件事也在测试覆盖内——安装角为零时很多错误的写法都能凑对。
    const MOUNT_PITCH_DEG: f32 = -65.0;

    #[derive(Resource, Clone, Copy)]
    struct AimCmd {
        yaw_deg: f32,
        pitch_deg: f32,
    }

    #[derive(Resource, Default)]
    struct AimRuns(u32);

    /// 只替掉共享内存收包这一步，云台查询、`ChildOf` 父节点解析和写入都调用
    /// `process_subscription` 用的同一个 `aim_gimbal_at`。
    fn aim_from_resource(
        cmd: Res<AimCmd>,
        mut runs: ResMut<AimRuns>,
        mut transforms: ParamSet<(
            TransformHelper,
            Query<
                (&mut Transform, &mut InfantryGimbal, Option<&ChildOf>),
                (
                    With<Controlled>,
                    Without<InfantryChassis>,
                    Without<InfantryLaunchOffset>,
                ),
            >,
            Query<(Entity, &GlobalTransform), (With<InfantryLaunchOffset>, With<Controlled>)>,
        )>,
    ) {
        let parent_entity = {
            let mut gimbals = transforms.p1();
            let (_, _, gimbal_parent) = gimbals
                .single_mut()
                .expect("controlled gimbal query must match exactly one entity");
            gimbal_parent.map(ChildOf::parent)
        };
        let (muzzle_entity, muzzle_previous) = {
            let muzzles = transforms.p2();
            let (entity, global) = muzzles
                .single()
                .expect("controlled muzzle query must match exactly one entity");
            (entity, *global)
        };
        let helper = transforms.p0();
        let parent_world = parent_entity
            .and_then(|entity| helper.compute_global_transform(entity).ok())
            .map(|global| global.rotation())
            .unwrap_or(Quat::IDENTITY);
        let muzzle_world = helper
            .compute_global_transform(muzzle_entity)
            .unwrap_or(muzzle_previous)
            .rotation();
        let mut gimbals = transforms.p1();
        let (mut gimbal_transform, mut gimbal_data, _) = gimbals
            .single_mut()
            .expect("controlled gimbal query must match exactly one entity");
        aim_gimbal_at(
            cmd.yaw_deg,
            cmd.pitch_deg,
            &mut gimbal_transform,
            &mut gimbal_data,
            parent_world,
            muzzle_world,
        );
        runs.0 += 1;
    }

    struct Rig {
        app: App,
        gimbal: Entity,
        muzzle: Entity,
        camera: Entity,
    }

    impl Rig {
        /// 层级照着 `setup.rs`：Infantry 根节点 -> VEHICLE 结构节点 -> GIMBAL ->
        /// SHOT_DIRECTION / CAM_DIRECTION。底盘朝向在根节点上，云台不是根的直接子节点。
        fn new(chassis_yaw_deg: f32, cmd: AimCmd) -> Self {
            Self::new_with_vehicle_rotation(chassis_yaw_deg, 0.0, cmd)
        }

        fn new_with_vehicle_rotation(
            chassis_yaw_deg: f32,
            vehicle_yaw_deg: f32,
            cmd: AimCmd,
        ) -> Self {
            let mut app = App::new();
            app.add_plugins((MinimalPlugins, TransformPlugin));
            app.insert_resource(cmd);
            app.init_resource::<AimRuns>();
            app.insert_resource(CameraMode(FollowingType::Robot));
            // 与生产调度一样：命令和相机都在 Update 写本地 Transform，随后由
            // PostUpdate 的 TransformSystems::Propagate 一次性发布当前帧世界姿态。
            app.add_systems(Update, (aim_from_resource, update_camera_follow).chain());

            let mount = Quat::from_euler(EulerRot::YXZ, 0.0, MOUNT_PITCH_DEG.to_radians(), 0.0);
            let world = app.world_mut();
            let root = world
                .spawn((
                    Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                    Controlled,
                    Transform::from_xyz(3.5, 0.0, 1.0)
                        .with_rotation(Quat::from_rotation_y(chassis_yaw_deg.to_radians())),
                ))
                .id();
            let vehicle = world
                .spawn((
                    Controlled,
                    Transform::from_xyz(0.0, 0.08, 0.0)
                        .with_rotation(Quat::from_rotation_y(vehicle_yaw_deg.to_radians())),
                    ChildOf(root),
                ))
                .id();
            let gimbal = world
                .spawn((
                    Controlled,
                    InfantryGimbal::default(),
                    Transform::from_xyz(0.0, 0.21, 0.0),
                    ChildOf(vehicle),
                ))
                .id();
            let muzzle = world
                .spawn((
                    Controlled,
                    InfantryLaunchOffset,
                    Transform::from_xyz(0.05, 0.0, 0.11).with_rotation(mount),
                    ChildOf(gimbal),
                ))
                .id();
            world.spawn((
                Controlled,
                InfantryViewOffset,
                Transform::from_xyz(0.0, 0.05, 0.12).with_rotation(mount),
                ChildOf(gimbal),
            ));
            let camera = world
                .spawn((
                    MainCamera {
                        follow_offset: Vec3::new(0.0, 2.0, 5.0),
                    },
                    Transform::default(),
                ))
                .id();
            let mut rig = Self {
                app,
                gimbal,
                muzzle,
                camera,
            };
            rig.step(2);
            rig
        }

        fn step(&mut self, frames: u32) {
            for _ in 0..frames {
                self.app.update();
            }
        }

        fn runs(&self) -> u32 {
            self.app.world().resource::<AimRuns>().0
        }

        fn gimbal_local(&self) -> Quat {
            self.app
                .world()
                .get::<Transform>(self.gimbal)
                .unwrap()
                .rotation
        }

        /// 云台组件里缓存的（局部 yaw, 局部 pitch），弧度。
        fn gimbal_angles(&self) -> (f32, f32) {
            let data = self.app.world().get::<InfantryGimbal>(self.gimbal).unwrap();
            (data.local_yaw, data.pitch)
        }

        fn gimbal_world(&self) -> Quat {
            self.app
                .world()
                .get::<GlobalTransform>(self.gimbal)
                .unwrap()
                .rotation()
        }

        fn muzzle_world(&self) -> Quat {
            self.app
                .world()
                .get::<GlobalTransform>(self.muzzle)
                .unwrap()
                .rotation()
        }

        /// `projectile_launch` 真正用来发弹的方向：`gimbal_global * launch_local * +Y`。
        fn launch_dir(&self) -> Vec3 {
            let launch_local = self
                .app
                .world()
                .get::<Transform>(self.muzzle)
                .unwrap()
                .rotation;
            (self.gimbal_world() * launch_local)
                .mul_vec3(Vec3::Y)
                .normalize()
        }

        /// 渲染相机的光轴，由生产系统 `update_camera_follow` 写出来。
        fn camera_axis(&self) -> Vec3 {
            let transform = self.app.world().get::<Transform>(self.camera).unwrap();
            transform.forward().as_vec3()
        }
    }

    /// 期望的世界系枪口指向，独立于被测代码推导：
    /// `Ry(yaw) * Rx(-pitch-90°) * +Y`。
    fn expected_world_dir(yaw_deg: f32, pitch_deg: f32) -> Vec3 {
        let p = (-pitch_deg - 90.0).to_radians();
        let y = yaw_deg.to_radians();
        Vec3::new(p.sin() * y.sin(), p.cos(), p.sin() * y.cos()).normalize()
    }

    /// 两个姿态之间的最小旋转角（度）。
    ///
    /// 不用 `Quat::angle_between`：它算的是 `2*acos(|dot|)`，两个姿态几乎相同时
    /// `dot≈1`，而 `acos` 在 1 附近导数发散，f32 里 1e-7 量级的点积误差会被放大成
    /// 0.05° 量级的假误差——比这里要断言的 0.01° 还大。那样就只能靠放宽阈值让断言
    /// 通过，而阈值一放宽，"世界旋转当局部旋转用"这类几度量级的真错误也可能漏掉。
    /// `atan2(|xyz|, |w|)` 形式在零附近数值稳定，阈值才压得住。
    fn quat_angle_deg(a: Quat, b: Quat) -> f32 {
        let d = (a.inverse() * b).normalize();
        let sin_half = Vec3::new(d.x, d.y, d.z).length();
        (2.0 * sin_half.atan2(d.w.abs())).to_degrees()
    }

    /// 两个方向之间的夹角（度）。同样避开 `Vec3::angle_between` 的 `acos`：
    /// `atan2(|a×b|, a·b)` 在近平行时是 O(θ) 精度，acos 是 O(sqrt(eps))。
    fn vec_angle_deg(a: Vec3, b: Vec3) -> f32 {
        let (a, b) = (a.normalize(), b.normalize());
        a.cross(b).length().atan2(a.dot(b)).to_degrees()
    }

    fn assert_dir(actual: Vec3, expected: Vec3, tol: f32, what: &str) {
        let angle = vec_angle_deg(actual, expected);
        assert!(
            angle < tol,
            "{what}: 实际 {actual:?} 与期望 {expected:?} 相差 {angle:.4}°"
        );
    }

    const CHASSIS_YAWS: [f32; 3] = [0.0, 30.0, 90.0];
    const CMD: AimCmd = AimCmd {
        yaw_deg: 40.0,
        pitch_deg: -12.0,
    };

    #[derive(Resource)]
    struct TestChassisYaw {
        entity: Entity,
        yaw_rad: f32,
        yaw_rate_radps: f32,
    }

    #[derive(Resource, Default)]
    struct TestPoseFrame(u64);

    #[derive(Resource, Clone)]
    struct TestPosePublisher(Arc<Mutex<ShmPublisher>>);

    /// This test's Update-input stand-in rotates the chassis before the real Talos command
    /// consumer, matching the phase where production vehicle input changes its local Transform.
    fn drive_test_chassis(
        mut drive: ResMut<TestChassisYaw>,
        mut transforms: Query<&mut Transform>,
    ) {
        drive.yaw_rad += drive.yaw_rate_radps / 60.0;
        let mut chassis = transforms
            .get_mut(drive.entity)
            .expect("test chassis entity disappeared");
        chassis.rotation = Quat::from_rotation_y(drive.yaw_rad);
    }

    fn advance_test_pose_frame(mut frame: ResMut<TestPoseFrame>) {
        frame.0 += 1;
    }

    /// Uses the production serializer and actual Talos triple buffers.  It runs in Last, after
    /// Update input/command/camera and PostUpdate propagation, just before ExtractSchedule would
    /// read these GlobalTransforms in the real simulator.
    fn publish_test_pose_snapshot(
        publisher: Res<TestPosePublisher>,
        frame: Res<TestPoseFrame>,
        camera: Single<&GlobalTransform, With<CaptureSource>>,
        gimbal: Single<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>,
        muzzle: Single<
            (&GlobalTransform, &Transform),
            (With<Controlled>, With<InfantryLaunchOffset>),
        >,
        chassis: Res<ChassisObservationFrame>,
    ) {
        let (muzzle_global, muzzle_local) = muzzle.into_inner();
        let pose = captured_pose_data(
            camera.into_inner(),
            gimbal.into_inner(),
            muzzle_global,
            muzzle_local,
            &chassis,
            frame.0,
            10_000_000_000 + frame.0,
        );
        let mut publisher = publisher.0.lock().expect("test pose publisher poisoned");
        publish_pose_data(&mut publisher, frame.0, 10_000_000_000 + frame.0, &pose);
    }

    struct ProductionScheduleRig {
        app: App,
        command_publisher: Arc<Mutex<ShmPublisher>>,
        pose_reader: ShmSubscriber,
        view: Entity,
        gimbal: Entity,
        muzzle: Entity,
        camera: Entity,
        capture_camera: Entity,
    }

    impl ProductionScheduleRig {
        fn new() -> Self {
            static TEST_IPC_ID: AtomicU64 = AtomicU64::new(0);

            let id = TEST_IPC_ID.fetch_add(1, Ordering::Relaxed);
            let prefix = format!("talos_schedule_{}_{}", std::process::id(), id);
            let meta_name = format!("{prefix}_meta");
            let image_name = format!("{prefix}_image");
            let command_publisher = Arc::new(Mutex::new(
                ShmPublisher::create_named(&meta_name, &image_name)
                    .expect("create isolated Talos publisher"),
            ));
            let command_subscriber = ShmSubscriber::connect_named(&meta_name)
                .expect("connect isolated Talos command subscriber");
            let pose_reader = ShmSubscriber::connect_named(&meta_name)
                .expect("connect isolated Talos pose reader");

            let mut app = App::new();
            app.add_plugins((MinimalPlugins, TransformPlugin));
            app.insert_resource(SimulationConfig::default());
            app.insert_resource(SubscribeAutoAim(AtomicBool::new(true)));
            app.init_resource::<AutoAimLink>();
            app.init_resource::<ExternalFireRequest>();
            app.insert_resource(ProjectileCooldown(Timer::from_seconds(
                0.0,
                TimerMode::Once,
            )));
            app.init_resource::<ProjectileStatistics>();
            app.insert_resource(ProjectileSetting(Handle::default(), Handle::default()));
            app.init_resource::<ChassisObservationFrame>();
            app.init_resource::<TestPoseFrame>();
            app.insert_resource(TestPosePublisher(command_publisher.clone()));
            app.insert_resource(ShmSubscriberRes(Arc::new(Mutex::new(command_subscriber))));
            app.add_message::<GamepadRumbleRequest>();
            app.insert_resource(CameraMode(FollowingType::Robot));

            let world = app.world_mut();
            let root = world
                .spawn((
                    Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                    Controlled,
                    Transform::from_xyz(3.5, 0.0, 1.0),
                    LinearVelocity::default(),
                    AngularVelocity::default(),
                ))
                .id();
            let vehicle = world
                .spawn((
                    Controlled,
                    Transform::from_xyz(0.0, 0.08, 0.0)
                        .with_rotation(Quat::from_rotation_y(17.0_f32.to_radians())),
                    ChildOf(root),
                ))
                .id();
            let gimbal = world
                .spawn((
                    Controlled,
                    InfantryGimbal::default(),
                    Transform::from_xyz(0.0, 0.21, 0.0),
                    ChildOf(vehicle),
                ))
                .id();
            let mount = Quat::from_euler(EulerRot::YXZ, 0.0, MOUNT_PITCH_DEG.to_radians(), 0.0);
            let muzzle = world
                .spawn((
                    Controlled,
                    InfantryLaunchOffset,
                    Transform::from_xyz(0.05, 0.0, 0.11).with_rotation(mount),
                    ChildOf(gimbal),
                ))
                .id();
            let view = world
                .spawn((
                    Controlled,
                    InfantryViewOffset,
                    Transform::from_xyz(0.0, 0.05, 0.12).with_rotation(mount),
                    ChildOf(gimbal),
                ))
                .id();
            let camera = world
                .spawn((
                    MainCamera {
                        follow_offset: Vec3::new(0.0, 2.0, 5.0),
                    },
                    CaptureSource,
                    Transform::default(),
                ))
                .id();
            let capture_camera = world.spawn((CaptureCamera, Transform::default())).id();

            app.insert_resource(TestChassisYaw {
                entity: root,
                yaw_rad: 0.0,
                yaw_rate_radps: 0.0,
            });
            // Production order: input changes chassis -> accepted Talos command changes gimbal ->
            // camera copies current CAM_DIRECTION -> PostUpdate propagates -> external fire
            // request runs projectile_launch -> Last snapshots poses for ExtractSchedule/IPC.
            app.add_systems(Update, drive_test_chassis);
            app.add_systems(
                Update,
                (
                    advance_link_lease,
                    process_subscription
                        .after(advance_link_lease)
                        .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
                )
                    .chain()
                    .after(drive_test_chassis),
            );
            app.add_systems(
                Update,
                (update_camera_follow, sync_capture_camera)
                    .chain()
                    .after(process_subscription),
            );
            app.add_systems(
                PostUpdate,
                launch_external_projectile.after(TransformSystems::Propagate),
            );
            app.add_systems(
                Last,
                (advance_test_pose_frame, publish_test_pose_snapshot).chain(),
            );

            Self {
                app,
                command_publisher,
                pose_reader,
                view,
                gimbal,
                muzzle,
                camera,
                capture_camera,
            }
        }

        fn set_chassis_motion(&mut self, yaw_deg: f32, yaw_rate_radps: f32) {
            let mut drive = self.app.world_mut().resource_mut::<TestChassisYaw>();
            drive.yaw_rad = yaw_deg.to_radians();
            drive.yaw_rate_radps = yaw_rate_radps;
        }

        fn publish_world_command(&self) {
            let timestamp_ns = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("system time before UNIX epoch")
                .as_nanos() as u64;
            self.command_publisher
                .lock()
                .expect("test command publisher poisoned")
                .publish_gimbal_cmd(GimbalCmd {
                    timestamp_ns,
                    yaw_deg: CMD.yaw_deg,
                    pitch_deg: CMD.pitch_deg,
                    distance_m: 5.0,
                    fire_advice: 1,
                    _pad: [0; 11],
                });
        }

        fn step(&mut self) -> (u64, PoseMeta, PoseMeta, PoseMeta, PoseMeta) {
            let expected_frame = self.app.world().resource::<TestPoseFrame>().0 + 1;
            self.publish_world_command();
            self.app.update();
            let gimbal = self
                .pose_reader
                .recv_pose(PoseIndex::Gimbal)
                .expect("gimbal IPC pose missing");
            let odom = self
                .pose_reader
                .recv_pose(PoseIndex::Odom)
                .expect("odom IPC pose missing");
            let muzzle = self
                .pose_reader
                .recv_pose(PoseIndex::Muzzle)
                .expect("muzzle IPC pose missing");
            let camera = self
                .pose_reader
                .recv_pose(PoseIndex::Camera)
                .expect("camera IPC pose missing");
            (expected_frame, gimbal, odom, muzzle, camera)
        }

        fn world_transform(&self, entity: Entity) -> Transform {
            self.app
                .world()
                .get::<GlobalTransform>(entity)
                .expect("test transform missing")
                .compute_transform()
        }

        fn projectile_direction(&mut self) -> Vec3 {
            let world = self.app.world_mut();
            let mut projectiles = world.query_filtered::<&LinearVelocity, With<Projectile>>();
            projectiles
                .iter(world)
                .last()
                .expect("PostUpdate projectile_launch did not spawn a projectile")
                .0
                .normalize()
        }
    }

    #[derive(Default)]
    struct AngleMetrics {
        samples: usize,
        total_deg: f32,
        max_deg: f32,
    }

    impl AngleMetrics {
        fn observe(&mut self, deg: f32) {
            self.samples += 1;
            self.total_deg += deg;
            self.max_deg = self.max_deg.max(deg);
        }

        fn mean_deg(&self) -> f32 {
            self.total_deg / self.samples.max(1) as f32
        }
    }

    fn assert_vec3_near(actual: Vec3, expected: Vec3, tolerance: f32, what: &str) {
        let error = actual.distance(expected);
        assert!(
            error < tolerance,
            "{what}: actual={actual:?} expected={expected:?} error={error:.7}m"
        );
    }

    fn assert_quat_near(actual: Quat, expected: Quat, tolerance_deg: f32, what: &str) -> f32 {
        let error = quat_angle_deg(actual, expected);
        assert!(
            error < tolerance_deg,
            "{what}: orientation error={error:.6} deg"
        );
        error
    }

    /// Drives the actual production Talos command system through static chassis yaw 0/30/90 and
    /// then 3 rad/s continuous rotation.  Every sample crosses the real Update -> PostUpdate ->
    /// Last schedule and checks propagated mount entities, rendering/capture cameras, a spawned
    /// projectile and isolated Talos IPC pose channels.
    #[test]
    fn production_schedule_keeps_dynamic_chassis_camera_projectile_and_ipc_in_same_frame() {
        const TOLERANCE_DEG: f32 = 0.03;
        const TOLERANCE_M: f32 = 1e-4;
        const CONTINUOUS_RATE_RADPS: f32 = 3.0;
        const CONTINUOUS_FRAMES: usize = 60;

        let expected_direction = expected_world_dir(CMD.yaw_deg, CMD.pitch_deg);
        let camera_roll = Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, std::f32::consts::FRAC_PI_2);
        let mut rig = ProductionScheduleRig::new();
        let mut shot_error = AngleMetrics::default();
        let mut rendered_camera_error = AngleMetrics::default();
        let mut projectile_error = AngleMetrics::default();
        let mut ipc_orientation_error = AngleMetrics::default();

        let mut validate = |rig: &mut ProductionScheduleRig, phase: &str| {
            let (frame, gimbal_ipc, odom_ipc, muzzle_ipc, camera_ipc) = rig.step();
            for (name, pose) in [
                ("gimbal", gimbal_ipc),
                ("odom", odom_ipc),
                ("muzzle", muzzle_ipc),
                ("camera", camera_ipc),
            ] {
                assert_eq!(
                    pose.frame_seq, frame,
                    "{phase}: {name} IPC frame_seq must equal Last snapshot frame {frame}"
                );
                assert_eq!(
                    pose.timestamp_ns,
                    10_000_000_000 + frame,
                    "{phase}: {name} IPC timestamp must come from the same Last snapshot"
                );
            }

            let shot = rig.world_transform(rig.muzzle);
            let cam_direction = rig.world_transform(rig.view);
            let rendered_camera = rig.world_transform(rig.camera);
            let capture_camera = rig.world_transform(rig.capture_camera);
            let gimbal = rig.world_transform(rig.gimbal);
            let projectile_direction = rig.projectile_direction();

            let shot_dir = shot.rotation.mul_vec3(Vec3::Y).normalize();
            let rendered_forward = rendered_camera.rotation.mul_vec3(Vec3::NEG_Z).normalize();
            shot_error.observe(vec_angle_deg(shot_dir, expected_direction));
            rendered_camera_error.observe(vec_angle_deg(rendered_forward, shot_dir));
            projectile_error.observe(vec_angle_deg(projectile_direction, shot_dir));
            assert_dir(
                shot_dir,
                expected_direction,
                TOLERANCE_DEG,
                &format!("{phase} SHOT_DIRECTION"),
            );
            assert_dir(
                rendered_forward,
                shot_dir,
                TOLERANCE_DEG,
                &format!("{phase} rendered camera forward"),
            );
            assert_dir(
                projectile_direction,
                shot_dir,
                TOLERANCE_DEG,
                &format!("{phase} projectile_launch direction"),
            );
            assert_vec3_near(
                rendered_camera.translation,
                cam_direction.translation,
                TOLERANCE_M,
                &format!("{phase} rendered camera position vs CAM_DIRECTION"),
            );
            assert_quat_near(
                rendered_camera.rotation,
                cam_direction.rotation * camera_roll,
                TOLERANCE_DEG,
                &format!("{phase} rendered camera rotation vs CAM_DIRECTION"),
            );
            assert_vec3_near(
                capture_camera.translation,
                rendered_camera.translation,
                TOLERANCE_M,
                &format!("{phase} capture camera position"),
            );
            assert_quat_near(
                capture_camera.rotation,
                rendered_camera.rotation,
                TOLERANCE_DEG,
                &format!("{phase} capture camera rotation"),
            );

            let expected_ipc_quat = to_ros_quat(shot.rotation * camera_roll);
            let gimbal_quat = Quat::from_xyzw(
                gimbal_ipc.quaternion[1],
                gimbal_ipc.quaternion[2],
                gimbal_ipc.quaternion[3],
                gimbal_ipc.quaternion[0],
            );
            ipc_orientation_error.observe(assert_quat_near(
                gimbal_quat,
                expected_ipc_quat,
                TOLERANCE_DEG,
                &format!("{phase} Talos Gimbal quaternion"),
            ));
            assert_vec3_near(
                Vec3::from_array(odom_ipc.position),
                to_ros_translation(gimbal.translation),
                TOLERANCE_M,
                &format!("{phase} Talos Odom/gimbal pivot position"),
            );
            assert_vec3_near(
                Vec3::from_array(muzzle_ipc.position),
                to_ros_translation(shot.translation),
                TOLERANCE_M,
                &format!("{phase} Talos Muzzle world position"),
            );
            let muzzle_quat = Quat::from_xyzw(
                muzzle_ipc.quaternion[1],
                muzzle_ipc.quaternion[2],
                muzzle_ipc.quaternion[3],
                muzzle_ipc.quaternion[0],
            );
            assert_quat_near(
                muzzle_quat,
                expected_ipc_quat,
                TOLERANCE_DEG,
                &format!("{phase} Talos Muzzle quaternion"),
            );
            let camera_relative = GlobalTransform::from(rendered_camera)
                .reparented_to(&GlobalTransform::from(gimbal))
                .translation;
            assert_vec3_near(
                Vec3::from_array(camera_ipc.position),
                to_ros_translation(camera_relative),
                TOLERANCE_M,
                &format!("{phase} Talos Camera relative position"),
            );
            assert_eq!(camera_ipc.quaternion, [1.0, 0.0, 0.0, 0.0]);
        };

        for yaw_deg in CHASSIS_YAWS {
            rig.set_chassis_motion(yaw_deg, 0.0);
            validate(&mut rig, &format!("static yaw={yaw_deg:.0}deg"));
        }
        rig.set_chassis_motion(90.0, CONTINUOUS_RATE_RADPS);
        for frame in 0..CONTINUOUS_FRAMES {
            validate(&mut rig, &format!("continuous frame={frame}"));
        }

        println!(
            "[talos-schedule] phase=Update[drive,input,command,camera] -> PostUpdate[TransformSystems::Propagate,projectile_launch] -> Last[pose_snapshot]; samples={}; static_yaw_deg=[0,30,90]; continuous_rate_radps={CONTINUOUS_RATE_RADPS:.3}; frame_dt_s={:.7}; legacy_one_frame_lag_rad={:.6}; legacy_one_frame_lag_deg={:.4}; shot(max/mean)={:.6}/{:.6}deg; rendered_camera(max/mean)={:.6}/{:.6}deg; projectile(max/mean)={:.6}/{:.6}deg; ipc_orientation(max/mean)={:.6}/{:.6}deg",
            shot_error.samples,
            1.0 / 60.0,
            CONTINUOUS_RATE_RADPS / 60.0,
            (CONTINUOUS_RATE_RADPS / 60.0).to_degrees(),
            shot_error.max_deg,
            shot_error.mean_deg(),
            rendered_camera_error.max_deg,
            rendered_camera_error.mean_deg(),
            projectile_error.max_deg,
            projectile_error.mean_deg(),
            ipc_orientation_error.max_deg,
            ipc_orientation_error.mean_deg(),
        );
        assert!(shot_error.max_deg < TOLERANCE_DEG);
        assert!(rendered_camera_error.max_deg < TOLERANCE_DEG);
        assert!(projectile_error.max_deg < TOLERANCE_DEG);
        assert!(ipc_orientation_error.max_deg < TOLERANCE_DEG);
    }

    #[test]
    fn a_world_frame_command_aims_the_muzzle_the_same_way_at_any_chassis_yaw() {
        let target = world_aim_rotation(CMD.yaw_deg, CMD.pitch_deg);
        let expected = expected_world_dir(CMD.yaw_deg, CMD.pitch_deg);
        for chassis_yaw in CHASSIS_YAWS {
            let rig = Rig::new(chassis_yaw, CMD);
            assert!(
                rig.runs() >= 2,
                "底盘 yaw={chassis_yaw}°：Single 没匹配上，瞄准系统根本没跑，后面的断言没有意义"
            );
            let muzzle = rig.muzzle_world();
            let angle = quat_angle_deg(muzzle, target);
            assert!(
                angle < 0.01,
                "底盘 yaw={chassis_yaw}°：枪口世界姿态偏离目标 {angle:.5}°"
            );
            assert_dir(
                muzzle.mul_vec3(Vec3::Y),
                expected,
                0.01,
                &format!("底盘 yaw={chassis_yaw}° 的枪口世界指向"),
            );
            // 世界指向与底盘朝向无关，这正是"命令是世界系"的含义。
            assert_dir(
                rig.launch_dir(),
                expected,
                0.01,
                &format!("底盘 yaw={chassis_yaw}° 的弹丸发射方向"),
            );
        }
    }

    #[test]
    fn the_gimbal_keeps_a_local_rotation_instead_of_the_world_rotation() {
        for chassis_yaw in CHASSIS_YAWS {
            let rig = Rig::new(chassis_yaw, CMD);
            // 闭式解：local = Ry(cmd_yaw - chassis_yaw) * Rx(-cmd_pitch - 90° - 安装角)
            let want_yaw = CMD.yaw_deg - chassis_yaw;
            let want_pitch = -CMD.pitch_deg - 90.0 - MOUNT_PITCH_DEG;
            let want_local = Quat::from_euler(
                EulerRot::YXZ,
                want_yaw.to_radians(),
                want_pitch.to_radians(),
                0.0,
            );
            let local = rig.gimbal_local();
            let angle = quat_angle_deg(local, want_local);
            assert!(
                angle < 0.01,
                "底盘 yaw={chassis_yaw}°：云台局部旋转偏离闭式解 {angle:.5}°"
            );
            let (cached_yaw, cached_pitch) = rig.gimbal_angles();
            assert!(
                (cached_yaw - want_yaw.to_radians()).abs() < 1e-4
                    && (cached_pitch - want_pitch.to_radians()).abs() < 1e-4,
                "底盘 yaw={chassis_yaw}°：缓存的局部角是 ({:.5}, {:.5}) rad，期望 ({:.5}, {:.5}) rad",
                cached_yaw,
                cached_pitch,
                want_yaw.to_radians(),
                want_pitch.to_radians()
            );
            // 局部 yaw 必须比命令的世界 yaw 少掉整个底盘朝向；写成世界系就会等于命令值。
            let as_world = (cached_yaw.to_degrees() - CMD.yaw_deg).abs();
            if chassis_yaw.abs() > 0.5 {
                assert!(
                    as_world > 0.5 * chassis_yaw,
                    "底盘 yaw={chassis_yaw}°：局部 yaw {:.3}° 和命令的世界 yaw {:.3}° 太接近，世界旋转被当成局部旋转写下去了",
                    cached_yaw.to_degrees(),
                    CMD.yaw_deg
                );
            }
            // 云台世界姿态 = 父节点朝向 * 局部旋转，这一条也顺手钉住层级没有别的旋转。
            let gimbal_world = rig.gimbal_world();
            let recomposed = Quat::from_rotation_y(chassis_yaw.to_radians()) * local;
            assert!(
                quat_angle_deg(gimbal_world, recomposed) < 0.01,
                "底盘 yaw={chassis_yaw}°：云台世界姿态和 父朝向*局部 不一致"
            );
        }
    }

    #[test]
    fn the_camera_optical_axis_matches_the_launch_direction_on_a_rotated_chassis() {
        let expected = expected_world_dir(CMD.yaw_deg, CMD.pitch_deg);
        for chassis_yaw in CHASSIS_YAWS {
            let rig = Rig::new(chassis_yaw, CMD);
            let launch = rig.launch_dir();
            let axis = rig.camera_axis();
            assert_dir(
                axis,
                launch,
                0.01,
                &format!("底盘 yaw={chassis_yaw}° 的相机光轴 vs 发射方向"),
            );
            assert_dir(
                axis,
                expected,
                0.01,
                &format!("底盘 yaw={chassis_yaw}° 的相机光轴 vs 期望世界指向"),
            );
        }
    }

    #[test]
    fn the_camera_follows_intermediate_mount_rotations() {
        // 真实 GLB 层级不应被假定为 VEHICLE/GIMBAL 之间只有单位旋转。相机必须
        // 与弹丸共同走完整的 CAM_DIRECTION 层级，否则图像光轴和发布姿态会分叉。
        for chassis_yaw in CHASSIS_YAWS {
            let rig = Rig::new_with_vehicle_rotation(chassis_yaw, 17.0, CMD);
            assert_dir(
                rig.camera_axis(),
                rig.launch_dir(),
                0.01,
                &format!("底盘 yaw={chassis_yaw}°、中间节点 yaw=17° 的相机光轴 vs 发射方向"),
            );
        }
    }

    #[test]
    fn repeating_the_same_command_does_not_walk_the_gimbal() {
        let mut rig = Rig::new(30.0, CMD);
        let first = rig.muzzle_world();
        rig.step(8);
        let angle = quat_angle_deg(first, rig.muzzle_world());
        assert!(angle < 0.01, "同一条命令重复 8 帧后姿态漂了 {angle:.5}°");
    }

    #[test]
    fn the_old_world_delta_formula_only_works_on_an_unrotated_chassis() {
        // 旧写法：delta = target * muzzle_world⁻¹，然后 gimbal_local' = delta * gimbal_local。
        // 它少了 parent⁻¹ … parent 这层共轭，父节点一有朝向就把世界旋转当局部旋转用。
        let mount = Quat::from_euler(EulerRot::YXZ, 0.0, MOUNT_PITCH_DEG.to_radians(), 0.0);
        let gimbal_local = Quat::from_euler(EulerRot::YXZ, 0.2, -0.1, 0.0);
        let target = world_aim_rotation(CMD.yaw_deg, CMD.pitch_deg);
        for chassis_yaw in CHASSIS_YAWS {
            let parent = Quat::from_rotation_y(chassis_yaw.to_radians());
            let muzzle_world = parent * gimbal_local * mount;

            let fixed = gimbal_local_rotation(target, parent, gimbal_local, muzzle_world);
            let fixed_muzzle = parent * fixed * mount;
            assert!(
                quat_angle_deg(fixed_muzzle, target) < 0.01,
                "底盘 yaw={chassis_yaw}°：修正后的解没把枪口放到目标世界姿态"
            );

            let old_local = (target * muzzle_world.inverse()) * gimbal_local;
            let old_muzzle = parent * old_local * mount;
            let old_error = quat_angle_deg(old_muzzle, target);
            if chassis_yaw.abs() < 0.5 {
                assert!(
                    old_error < 0.01,
                    "底盘不转时旧写法本该是对的，却差了 {old_error:.5}°"
                );
            } else {
                // 阈值 1° 是"远超数值噪声、且远小于实测偏差"的取值：这条命令
                // （yaw=40°, pitch=-12°）下实测旧写法差 3.76°(底盘 30°)、
                // 10.28°(底盘 90°)，而 `quat_angle_deg` 的噪声在 1e-3° 量级。
                // 不写成 >5° 是因为偏差随底盘角与命令角一起变，5° 会在底盘 30°
                // 这一档误报；1° 仍能把"少了 parent⁻¹…parent 共轭"这个错误钉住。
                assert!(
                    old_error > 1.0,
                    "底盘 yaw={chassis_yaw}°：旧写法只差 {old_error:.5}°，这个回归用例失去意义了"
                );
            }
        }
    }

    #[test]
    fn world_aim_rotation_matches_the_measured_probe_table() {
        // 实测探针（`sim_gimbal.hpp` 里记的那张表）：命令 pitch 为正是低头。
        let elev = |yaw: f32, pitch: f32| {
            world_aim_rotation(yaw, pitch)
                .mul_vec3(Vec3::Y)
                .normalize()
                .y
                .asin()
                .to_degrees()
        };
        assert!(elev(0.0, 0.0).abs() < 0.01, "pitch=0 应该水平");
        assert!(
            (elev(0.0, -20.0) - 20.0).abs() < 0.01,
            "pitch=-20 应该抬 20°"
        );
        assert!(
            (elev(0.0, 20.0) + 20.0).abs() < 0.01,
            "pitch=+20 应该压 20°"
        );
        assert!(
            (elev(0.0, -90.0) - 90.0).abs() < 0.01,
            "pitch=-90 应该指天顶"
        );
        // yaw=0 指世界 -Z（ROS +X 前），yaw=90° 指世界 -X（ROS +Y 左），
        // 与 sim_geometry_test 对枪口视差的要求一致。
        assert_dir(
            world_aim_rotation(0.0, 0.0).mul_vec3(Vec3::Y),
            Vec3::NEG_Z,
            0.01,
            "yaw=0 的枪口指向",
        );
        assert_dir(
            world_aim_rotation(90.0, 0.0).mul_vec3(Vec3::Y),
            Vec3::NEG_X,
            0.01,
            "yaw=90° 的枪口指向",
        );
    }

    #[test]
    fn a_gimbal_without_a_parent_falls_back_to_the_world_frame() {
        // 层级里没有父节点时（或父节点没有 GlobalTransform），世界系就是局部系，
        // 解应当退化成"直接写目标姿态 * 安装角逆"。
        let mount = Quat::from_euler(EulerRot::YXZ, 0.0, MOUNT_PITCH_DEG.to_radians(), 0.0);
        let gimbal_local = Quat::from_rotation_y(1.1);
        let target = world_aim_rotation(-25.0, 6.0);
        let muzzle_world = gimbal_local * mount;
        let solved = gimbal_local_rotation(target, Quat::IDENTITY, gimbal_local, muzzle_world);
        assert!(
            quat_angle_deg(solved * mount, target) < 0.01,
            "无父节点时的解不对"
        );
    }
}
