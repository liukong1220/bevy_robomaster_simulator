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
    gimbal_parent: Option<&ChildOf>,
    parents: &Query<&GlobalTransform>,
    muzzle_world: Quat,
) {
    let target_world = world_aim_rotation(cmd_yaw_deg, cmd_pitch_deg);
    let parent_world = gimbal_parent
        .and_then(|child_of| parents.get(child_of.parent()).ok())
        .map(|global| global.rotation())
        .unwrap_or(Quat::IDENTITY);
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
    mut commands: Commands,
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut link: ResMut<AutoAimLink>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal, Option<&ChildOf>),
        (
            With<Controlled>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    parents: Query<&GlobalTransform>,
    muzzle_offset: Single<&GlobalTransform, (With<InfantryLaunchOffset>, With<Controlled>)>,
    mut fired: Local<u64>,
) {
    let Some(ctx) = context else {
        return;
    };
    let (mut gimbal_transform, mut gimbal_data, gimbal_parent) = gimbal.into_inner();

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

    aim_gimbal_at(
        cmd.yaw_deg,
        cmd.pitch_deg,
        &mut gimbal_transform,
        &mut gimbal_data,
        gimbal_parent,
        &parents,
        muzzle_offset.rotation(),
    );
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
    use crate::components::{CameraMode, FollowingType, Infantry, InfantryViewOffset, MainCamera};
    use crate::robomaster::prelude::{INFANTRY_THREE_CONFIG, Team};
    use crate::systems::update_camera_follow;
    use bevy::transform::{TransformPlugin, TransformSystems};

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
        gimbal: Single<
            (&mut Transform, &mut InfantryGimbal, Option<&ChildOf>),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
        parents: Query<&GlobalTransform>,
        muzzle_offset: Single<&GlobalTransform, (With<InfantryLaunchOffset>, With<Controlled>)>,
    ) {
        let (mut gimbal_transform, mut gimbal_data, gimbal_parent) = gimbal.into_inner();
        aim_gimbal_at(
            cmd.yaw_deg,
            cmd.pitch_deg,
            &mut gimbal_transform,
            &mut gimbal_data,
            gimbal_parent,
            &parents,
            muzzle_offset.rotation(),
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
            app.add_systems(Update, aim_from_resource);
            // 相机跟随是生产代码，放在本帧传播之后，才能拿到刚写下去的云台姿态。
            app.add_systems(
                PostUpdate,
                update_camera_follow.after(TransformSystems::Propagate),
            );

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
