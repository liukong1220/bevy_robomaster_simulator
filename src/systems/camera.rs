use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::transform::helper::TransformHelper;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};
use core::f32::consts::PI;

use crate::components::{
    CameraMode, Controlled, FollowingType, Infantry, InfantryGimbal, InfantryLaunchOffset,
    InfantryViewOffset, MainCamera, MouseCapture,
};
use crate::config::SimulationConfig;
use crate::systems::ControllerState;

pub fn following_controls(mut mode: ResMut<CameraMode>, controller: Res<ControllerState>) {
    if controller.controlled.switch_camera_just_pressed {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

pub fn update_camera_follow(
    mut camera_queries: ParamSet<(
        TransformHelper,
        Query<(&mut Transform, &MainCamera), Without<Controlled>>,
    )>,
    infantry: Single<&Transform, (With<Infantry>, With<Controlled>)>,
    gimbal: Single<&Transform, (With<Controlled>, With<InfantryGimbal>)>,
    view_offset: Single<(Entity, &GlobalTransform), (With<Controlled>, With<InfantryViewOffset>)>,
    launch_offset: Single<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>,
    gimbal_global: Single<(Entity, &GlobalTransform), (With<Controlled>, With<InfantryGimbal>)>,
    mode: Res<CameraMode>,
    mut dbg_init: Local<bool>,
    mut dbg_left: Local<i32>,
) {
    let gimbal_transform = gimbal.into_inner();
    let (view_entity, view_previous) = view_offset.into_inner();
    // `Update` receives the current local input (chassis and external gimbal command), while
    // Bevy's stored GlobalTransform is not propagated until PostUpdate.  Compute the current
    // hierarchy now, write both camera Transforms, and let the normal propagation publish this
    // exact pose to rendering and Talos later in the same frame.
    let view_global = camera_queries
        .p0()
        .compute_global_transform(view_entity)
        .unwrap_or(*view_previous);
    let (gimbal_entity, gimbal_previous) = gimbal_global.into_inner();
    let current_gimbal_global = camera_queries
        .p0()
        .compute_global_transform(gimbal_entity)
        .unwrap_or(*gimbal_previous);
    let mut cameras = camera_queries.p1();
    let (mut camera_transform, camera_offset) = cameras
        .single_mut()
        .expect("MainCamera query must match exactly one entity");

    match mode.0 {
        FollowingType::Robot => {
            // 直接取 CAM_DIRECTION 这个挂载点的世界位姿。
            //
            // 原来是 `infantry.translation + gimbal_world_rotation * cam_local`，
            // 这么算会漏掉 GIMBAL 节点自己相对车体根节点的平移（以及中间层级的
            // 任何平移），结果相机被放低约 0.2m，正好落在车体壳子里面：
            // 渲染出来上下两半都是自己的装甲板，只中间留一条缝，视觉算法完全看
            // 不到别的车。用挂载点的 GlobalTransform 就没有这个系统性偏差。
            //
            // `TransformHelper` 按本帧的完整层级计算 CAM_DIRECTION，随后本帧的正常
            // PostUpdate 传播把同一姿态写回 GlobalTransform。这样不会再让运动底盘留下
            // 一帧陈旧的渲染/IPC 姿态。旧实现只拼
            // `infantry * gimbal_local`，中间节点一旦带旋转就会让图像光轴与发布给
            // 算法的姿态分叉。
            camera_transform.translation = view_global.translation();
            // 光轴必须与**枪口**重合，所以要带上 Rx(90°)。
            //
            // 关键点：在 LAUNCH/CAM 这个挂载点的局部系里，枪管前向是 +Y，而 bevy
            // 相机的视线方向是 -Z。两者差的正是绕局部 X 轴的 90°：
            //     Rx(90°) * (0,0,-1) = (0,1,0)
            // 所以只有右乘 Rx(90°)，相机的 forward 才等于 `projectile_launch` 真正
            // 用来发射弹丸的 `(gimbal_rot * launch_rot) * Vec3::Y`。
            //
            // 曾经把这一项当成"发布链路特有的多余滚转"删掉，那是把两个不同的
            // 前向约定混为一谈了。删掉之后相机 forward = M*(0,0,-1)，与枪口
            // M*(0,1,0) 相差 90°：本机场景初始 launch_local 是绕 X -65°，于是枪口
            // 指向 (0,0.4226,-0.9063) 而镜头指向 (0,-0.9063,-0.4226)，镜头被压进自己
            // 底盘里 —— 渲染出来满屏都是本车装甲板，算法自然什么也看不到。
            //
            // 算法侧 R_camera2gimbal = [0,0,1, -1,0,0, 0,-1,0] 是纯轴置换、没有安装
            // 倾角，即它假设光轴与枪口轴重合；talos/capture.rs 发布反馈时用的也是
            // 同一个 Rx(90°)。注意 CAM_DIRECTION 节点自身已经带了相机安装旋转，不能
            // 再拿 `view_global.rotation()` 追加一次 Rx(90°)，否则会把实际镜头翻到
            // 约 Rx(180°)。姿态应从 SHOT_DIRECTION（与发弹同一安装基准）计算，位置
            // 仍取 CAM_DIRECTION 的完整层级结果。
            let camera_roll = Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0);
            camera_transform.rotation =
                current_gimbal_global.rotation() * launch_offset.rotation * camera_roll;

            // 临时诊断：本地量算出来的朝向 vs 挂载点真实 GlobalTransform。
            // 视觉链路要求"图像的相机位姿"和"发布给算法的相机位姿"是同一个，
            // 这里就是用来确认两条链是否一致。DAEDALUS_DEBUG_CAMERA=N 打印 N 帧。
            if !*dbg_init {
                *dbg_init = true;
                *dbg_left = std::env::var("DAEDALUS_DEBUG_CAMERA")
                    .ok()
                    .and_then(|v| v.trim().parse::<i32>().ok())
                    .unwrap_or(0);
            }
            {
                if *dbg_left > 0 {
                    *dbg_left -= 1;
                    let cam_g = view_global;
                    let gim_g = current_gimbal_global;
                    let q = |r: Quat| format!("[{:.5},{:.5},{:.5},{:.5}]", r.x, r.y, r.z, r.w);
                    info!(
                        "[camdbg] rendered_q={} launch_local_q={} gimbal_local_q={} infantry_q={} gimbal_global_q={} cam_global_q={} cam_pos={:?} gim_pos={:?}",
                        q(camera_transform.rotation),
                        q(launch_offset.rotation),
                        q(gimbal_transform.rotation),
                        q(infantry.rotation),
                        q(gim_g.rotation()),
                        q(cam_g.rotation()),
                        cam_g.translation(),
                        gim_g.translation(),
                    );
                }
            }
        }
        FollowingType::ThirdPerson => {
            let base_transform = infantry.into_inner();
            let offset = base_transform.rotation * camera_offset.follow_offset;
            camera_transform.translation = base_transform.translation + offset;
            camera_transform.look_at(base_transform.translation, Vec3::Y);
        }
        FollowingType::Free => {}
    }
}

/// 左键捕获鼠标 / Esc 释放，并把捕获状态同步到窗口。
///
/// 不捕获时指针会撞到屏幕边缘，`MouseMotion` 随之停下，视角只能转一小段就卡住；
/// 所以"鼠标能瞄准"必须连着光标捕获一起做，只加事件读取是不够的。
///
/// 反过来也不能一启动就捕获：那样窗口一获得焦点指针就被锁住，调 egui inspector
/// 或者切到别的窗口都要先想起按 Esc。这里用显式的左键 / Esc，HUD 里写明。
pub fn update_cursor_capture(
    mouse_button: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut capture: ResMut<MouseCapture>,
    window: Option<Single<&Window, With<PrimaryWindow>>>,
    cursor: Option<Single<&mut CursorOptions, With<PrimaryWindow>>>,
) {
    let mut want = capture.captured;
    let focused = window.map(|w| w.focused).unwrap_or(true);
    if keyboard.just_pressed(KeyCode::Escape) || !focused {
        want = false;
    } else if mouse_button.just_pressed(MouseButton::Left) {
        want = true;
    }

    let Some(cursor) = cursor else {
        // 无窗口（离屏采集）时只记状态，不去写不存在的 CursorOptions。
        capture.captured = want;
        return;
    };
    let mut cursor = cursor.into_inner();

    if want != capture.captured {
        capture.captured = want;
    }
    // 每帧对齐一次而不是只在切换时写：窗口失焦时 winit 会自己把 grab 放掉，
    // 只在边沿写的话回到窗口后状态就和 `capture.captured` 不一致了。
    // X11 不支持 Locked，bevy 内部会退化成 Confined。
    // 失焦时把 captured 清掉：OpenCV 检测窗口抢走焦点后，必须重新左键捕获，
    // 避免带着卡住的右键 pressed 一回来就再次打开自瞄。
    let grab_mode = if capture.captured {
        CursorGrabMode::Locked
    } else {
        CursorGrabMode::None
    };
    if cursor.grab_mode != grab_mode {
        cursor.grab_mode = grab_mode;
    }
    let visible = !capture.captured;
    if cursor.visible != visible {
        cursor.visible = visible;
    }
}

pub fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    config: Res<SimulationConfig>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<Infantry>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * config.camera.mouse_sensitivity;
        let new_pitch = (pitch - mouse_delta.y * config.camera.mouse_sensitivity).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    let speed = config.camera.free_move_speed * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}
