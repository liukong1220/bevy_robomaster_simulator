#![allow(dead_code)]
mod capture;
mod components;
mod config;
mod handler;
mod metalfx;
mod robomaster;
mod setup;
mod statistic;
mod systems;
mod telemetry;
mod util;

#[cfg(feature = "ros2")]
mod ros2;
#[cfg(feature = "talos")]
mod talos;

use avian3d::prelude::*;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::render::settings::{InstanceFlags, RenderCreation, WgpuSettings, WgpuSettingsPriority};
use bevy::render::{RenderPlugin, RenderSystems};
use bevy::window::PresentMode;
use bevy::winit::WinitSettings;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use std::sync::atomic::AtomicBool;

use crate::components::{
    CameraMode, FollowingType, MouseCapture, ProjectileCooldown, SubscribeAutoAim,
};
use crate::config::{ConfigPlugin, SimulationConfig};
use crate::handler::{on_activate, on_hit};
use crate::metalfx::MetalFxTemporalPlugin;
use crate::robomaster::prelude::RoboMasterPlugins;
use crate::setup::{setup, setup_collision, setup_dart_launch, setup_ground, setup_vehicle};
use crate::statistic::ProjectileStatistics;
use crate::systems::{
    ChassisObservationFrame, ControllerState, GameplaySystems, PreviousKinematicState,
    change_appearance, cleanup_projectiles, clear_controller_input, controller_dart_just_pressed,
    controller_shoot_pressed, dart_launch, following_controls, freecam_controls, gimbal_controls,
    log_projectile_stats, projectile_aerodynamics, projectile_launch, remote_gimbal_controls,
    remote_vehicle_controls, sample_gamepad_controller, sample_keyboard_controller,
    sample_mouse_buttons, sample_mouse_controller, screenshot_on_f2, screenshot_on_timer,
    screenshot_saving, setup_projectile, switch_slapper_control, uav_launch,
    update_auto_aim_subscription, update_chassis_observation, update_cursor_capture,
    update_help_text, vehicle_controls,
};

#[cfg(feature = "ros2")]
use crate::ros2::plugin::ROS2Plugin;
#[cfg(feature = "talos")]
use talos::TalosPlugin;

fn present_mode_from_config(value: &str) -> Option<PresentMode> {
    match value.trim().to_ascii_lowercase().as_str() {
        "auto_vsync" | "vsync" => Some(PresentMode::AutoVsync),
        "auto_no_vsync" | "no_vsync" | "novsync" => Some(PresentMode::AutoNoVsync),
        "fifo" => Some(PresentMode::Fifo),
        "fifo_relaxed" | "fifo-relaxed" => Some(PresentMode::FifoRelaxed),
        "mailbox" => Some(PresentMode::Mailbox),
        "immediate" => Some(PresentMode::Immediate),
        _ => None,
    }
}

fn is_wsl() -> bool {
    std::env::var_os("WSL_DISTRO_NAME").is_some()
        || std::env::var_os("WSL_INTEROP").is_some()
        || std::fs::read_to_string("/proc/sys/kernel/osrelease")
            .map(|release| release.to_ascii_lowercase().contains("microsoft"))
            .unwrap_or(false)
}

fn render_plugin_for_platform() -> RenderPlugin {
    if cfg!(target_os = "linux") && is_wsl() {
        return RenderPlugin {
            render_creation: RenderCreation::Automatic(Box::new(WgpuSettings {
                instance_flags: InstanceFlags::default()
                    | InstanceFlags::ALLOW_UNDERLYING_NONCOMPLIANT_ADAPTER,
                priority: WgpuSettingsPriority::Functionality,
                ..default()
            })),
            ..default()
        };
    }

    RenderPlugin::default()
}

fn fixed_time_from_config(config: &SimulationConfig) -> Time<Fixed> {
    Time::<Fixed>::from_hz(config.physics.fixed_hz.max(1.0))
}

#[cfg(feature = "talos")]
fn should_enable_talos_plugin(app: &App) -> bool {
    #[cfg(feature = "ros2")]
    let ros_capture_active = app
        .world()
        .contains_resource::<crate::ros2::capture::RosCaptureContext>();
    #[cfg(not(feature = "ros2"))]
    let ros_capture_active = false;

    let force_talos_capture = std::env::var("DAEDALUS_FORCE_TALOS_CAPTURE")
        .map(|v| v == "1")
        .unwrap_or(false);

    !ros_capture_active || force_talos_capture
}

fn main() {
    let config = SimulationConfig::default();
    let present_mode = present_mode_from_config(&config.window.present_mode).unwrap_or_else(|| {
        warn!(
            "Unknown window.present_mode {:?}, falling back to auto_no_vsync",
            config.window.present_mode
        );
        PresentMode::AutoNoVsync
    });
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins
            .set(LogPlugin {
                // parley/icu_segmenter 在给中日韩文本做断行时会去找 "cjdict"
                // 分词模型；bevy 编译进来的 icu 数据里没有它，于是每次文字整形
                // 都报一条 "No segmentation model for language: ja"。HUD 在开火
                // 时计数每帧都变 -> 每帧重新整形 -> 每帧一条，实测 60s 闭环跑出
                // 7005 行里有 6208 行是它，真正有用的日志（真值溢出、故障、诊断）
                // 全被冲掉。缺这个模型只影响 CJK 断行位置的优选，HUD 是单行短
                // 文本，没有可见影响。
                //
                // 光有这条指令不够，还必须在 Cargo.toml 里给 icu_provider 打开
                // logging feature：不开时它内部用的是 `pub use std::eprintln as
                // warn`，裸 eprintln 没有 tracing target，任何按 target 过滤的
                // 指令都匹配不到（这就是之前以为压住了、开火时又刷回来的原因）。
                // 两处是一对，改一处等于没改。
                filter: format!("{}icu_provider=error", bevy::log::DEFAULT_FILTER),
                ..default()
            })
            .set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode,
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            })
            .set(render_plugin_for_platform()),
        PhysicsPlugins::default(),
    ));
    app.insert_resource(WinitSettings::continuous());

    if config.debug.egui {
        app.add_plugins(EguiPlugin::default());
        if config.debug.inspector {
            app.add_plugins(WorldInspectorPlugin::new());
        }
    }

    app.add_plugins(RoboMasterPlugins)
        .add_plugins(MetalFxTemporalPlugin)
        .add_plugins(ConfigPlugin)
        .init_resource::<CameraMode>()
        .init_resource::<MouseCapture>()
        .init_resource::<ProjectileStatistics>()
        .init_resource::<ChassisObservationFrame>()
        .init_resource::<PreviousKinematicState>()
        .init_resource::<ControllerState>()
        .register_type::<ProjectileStatistics>()
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(SubstepCount(config.physics.substep_count))
        .insert_resource(fixed_time_from_config(&config))
        .insert_resource(SubscribeAutoAim(AtomicBool::new(false)))
        .insert_resource(ProjectileCooldown(Timer::from_seconds(
            config.projectile.cooldown,
            TimerMode::Once,
        )))
        .add_systems(Startup, (setup, setup_projectile))
        .add_observer(setup_ground)
        .add_observer(setup_dart_launch)
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(on_hit)
        .add_observer(on_activate)
        .configure_sets(
            Update,
            (
                GameplaySystems::Input,
                GameplaySystems::GameLogic,
                GameplaySystems::Camera,
                GameplaySystems::Cleanup,
            )
                .chain(),
        )
        .add_systems(
            Update,
            (
                // Input phase
                (
                    clear_controller_input,
                    // 鼠标按键要在捕获状态更新**之前**采：左键既是开火键又是
                    // "捕获指针"键，先更新捕获状态的话，那一次用来捕获窗口的点击
                    // 会在同一帧被当成开火（见 sample_mouse_buttons 注释）。
                    sample_mouse_buttons,
                    // 捕获状态要先更新：sample_mouse_controller 只在捕获中把鼠标
                    // 位移当瞄准输入，否则要把事件读掉丢弃（见该函数注释）。
                    update_cursor_capture,
                    sample_keyboard_controller,
                    sample_mouse_controller,
                    sample_gamepad_controller,
                    update_auto_aim_subscription,
                    following_controls,
                    switch_slapper_control,
                    vehicle_controls.run_if(|mode: Res<CameraMode>| mode.0 != FollowingType::Free),
                    remote_vehicle_controls,
                    gimbal_controls,
                    remote_gimbal_controls,
                )
                    .chain()
                    .in_set(GameplaySystems::Input),
                // GameLogic phase
                (change_appearance, update_help_text, log_projectile_stats)
                    .in_set(GameplaySystems::GameLogic),
                // Camera phase
                (
                    freecam_controls.run_if(|mode: Res<CameraMode>| mode.0 == FollowingType::Free),
                    systems::update_camera_follow
                        .run_if(|mode: Res<CameraMode>| mode.0 != FollowingType::Free),
                )
                    .in_set(GameplaySystems::Camera)
                    .before(RenderSystems::Render),
                // Cleanup phase
                (
                    cleanup_projectiles,
                    screenshot_on_f2
                        .run_if(|input: Res<ButtonInput<KeyCode>>| input.just_pressed(KeyCode::F2)),
                    screenshot_on_timer,
                    screenshot_saving,
                )
                    .in_set(GameplaySystems::Cleanup),
            ),
        )
        .add_systems(
            PostUpdate,
            update_chassis_observation.after(TransformSystems::Propagate),
        )
        .add_systems(
            PostUpdate,
            projectile_launch
                .after(TransformSystems::Propagate)
                .run_if(controller_shoot_pressed),
        )
        .add_systems(
            PostUpdate,
            dart_launch
                .after(TransformSystems::Propagate)
                .run_if(controller_dart_just_pressed),
        )
        .add_systems(PostUpdate, uav_launch.after(TransformSystems::Propagate))
        .add_systems(FixedUpdate, projectile_aerodynamics);

    if config.debug.diagnostics {
        app.add_plugins((
            FrameTimeDiagnosticsPlugin::default(),
            LogDiagnosticsPlugin::default(),
        ));
    }

    #[cfg(feature = "ros2")]
    {
        app.add_plugins(ROS2Plugin::default());
        info!("ROS2 integration enabled");
    }
    #[cfg(not(feature = "ros2"))]
    {
        info!("ROS2 integration disabled");
    }

    #[cfg(feature = "talos")]
    {
        if should_enable_talos_plugin(&app) {
            app.add_plugins(TalosPlugin::default());
            info!("talos integration enabled");
        } else {
            info!(
                "talos integration skipped: ROS2 capture already active \
                 (set DAEDALUS_FORCE_TALOS_CAPTURE=1 to override)"
            );
        }
    }

    app.run();
}
