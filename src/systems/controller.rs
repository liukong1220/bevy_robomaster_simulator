use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use core::time::Duration;
use std::sync::atomic::Ordering;

use crate::components::{CameraMode, FollowingType, MouseCapture, SubscribeAutoAim};
use crate::config::SimulationConfig;

const GAMEPAD_STICK_DEADZONE: f32 = 0.12;
const GAMEPAD_TRIGGER_THRESHOLD: f32 = 0.35;
const PRECISE_GIMBAL_SCALE: f32 = 0.35;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerHelp {
    source: &'static str,
    manual: &'static str,
    auto_aim: &'static str,
}

impl ControllerHelp {
    const fn keyboard() -> Self {
        Self {
            source: "keyboard",
            manual: "F3 视角 | WASD 移动 | 左键 捕获鼠标 / Esc 释放 | 鼠标或方向键 瞄准 | 空格 射击 | G 飞镖 | Q 小陀螺 | U 远程小陀螺 | F5 自瞄 | Tab 拍打",
            auto_aim: "F5 关闭自瞄（自瞄期间鼠标与方向键不控云台）| WASD 移动 | Q 小陀螺 | U 远程小陀螺 | 由外部 fire_advice 控制射击 | Tab 拍打",
        }
    }

    const fn xbox() -> Self {
        Self {
            source: "xbox",
            manual: "View 视角 | LS 移动 | L3 加速 | 十字键 拍打移动 | RS 瞄准 | R3+RS 拍打翻滚/俯仰 | LB 小陀螺 | Y 拍打小陀螺 | RB 射击 | X 飞镖 | 按住 RT 自瞄",
            auto_aim: "松开 RT 关闭自瞄 | LS 移动 | L3 加速 | 十字键 拍打移动 | R3+RS 拍打翻滚/俯仰 | LB 小陀螺 | Y 拍打小陀螺 | 由外部 fire_advice 控制射击",
        }
    }
}

impl Default for ControllerHelp {
    fn default() -> Self {
        Self::keyboard()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ChassisSpinMode {
    #[default]
    Off,
    On,
}

impl ChassisSpinMode {
    fn toggle(&mut self) {
        *self = match self {
            Self::Off => Self::On,
            Self::On => Self::Off,
        };
    }

    fn yaw_input(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::On => 1.0,
        }
    }

    fn is_on(self) -> bool {
        self == Self::On
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerInput {
    pub movement: Vec2,
    pub gimbal: Vec2,
    /// 一帧内的云台角度增量（弧度），与 `gimbal` 是**两种不同的量**：
    /// `gimbal` 是被 clamp 到 [-1,1] 的速率轴（方向键、摇杆），要乘
    /// `gimbal_rotation_speed * dt`；这个字段是鼠标那种已经带位移大小的增量，
    /// 直接累加。把鼠标塞进 `gimbal` 里会被 clamp 成"按住方向键"，
    /// 于是轻推和猛甩一个速度，手感完全糊掉。
    pub gimbal_delta: Vec2,
    pub chassis_yaw: f32,
    pub chassis_roll: f32,
    pub chassis_pitch: f32,
    pub boost: bool,
    pub precise_gimbal: bool,
    pub shoot: bool,
    pub dart_just_pressed: bool,
    pub switch_slapper_just_pressed: bool,
    pub switch_camera_just_pressed: bool,
    pub auto_aim: bool,
}

impl Default for ControllerInput {
    fn default() -> Self {
        Self {
            movement: Vec2::ZERO,
            gimbal: Vec2::ZERO,
            gimbal_delta: Vec2::ZERO,
            chassis_yaw: 0.0,
            chassis_roll: 0.0,
            chassis_pitch: 0.0,
            boost: false,
            precise_gimbal: false,
            shoot: false,
            dart_just_pressed: false,
            switch_slapper_just_pressed: false,
            switch_camera_just_pressed: false,
            auto_aim: false,
        }
    }
}

impl ControllerInput {
    pub fn boost_multiplier(self) -> f32 {
        if self.boost { 2.0 } else { 1.0 }
    }

    pub fn gimbal_scale(self) -> f32 {
        if self.precise_gimbal {
            PRECISE_GIMBAL_SCALE
        } else {
            1.0
        }
    }

    fn add_movement(&mut self, movement: Vec2) {
        self.movement = clamp_axes_vec2(self.movement + movement);
    }

    fn add_gimbal(&mut self, gimbal: Vec2) {
        self.gimbal = clamp_axes_vec2(self.gimbal + gimbal);
    }

    /// 累加角度增量。不 clamp：这是"鼠标移动了多少"，本身就没有归一化上界。
    fn add_gimbal_delta(&mut self, delta: Vec2) {
        self.gimbal_delta += delta;
    }

    fn add_chassis(&mut self, yaw: f32, roll: f32, pitch: f32) {
        self.chassis_yaw = (self.chassis_yaw + yaw).clamp(-1.0, 1.0);
        self.chassis_roll = (self.chassis_roll + roll).clamp(-1.0, 1.0);
        self.chassis_pitch = (self.chassis_pitch + pitch).clamp(-1.0, 1.0);
    }
}

#[derive(Resource, Debug, Default)]
pub struct ControllerState {
    pub controlled: ControllerInput,
    pub remote: ControllerInput,
    keyboard_auto_aim: bool,
    controlled_chassis_spin: ChassisSpinMode,
    remote_chassis_spin: ChassisSpinMode,
    active_gamepad: Option<Entity>,
    help: ControllerHelp,
}

impl ControllerState {
    pub fn reset_frame(&mut self) {
        self.controlled = ControllerInput::default();
        self.remote = ControllerInput::default();
    }

    pub fn auto_aim_active(&self) -> bool {
        self.keyboard_auto_aim || self.controlled.auto_aim
    }

    pub fn help_source(&self) -> &'static str {
        self.help.source
    }

    /// HUD 的模式与操作提示必须按**实际生效**的订阅状态显示，而不是
    /// `auto_aim_active()`。后者只看 F5/RT，而 `DAEDALUS_FORCE_AUTO_AIM=1`
    /// 也会打开订阅（见 `update_auto_aim_subscription`）：脚本化启动时云台
    /// 已经在吃共享内存里的 gimbal_cmd，HUD 却仍显示"模式=manual / 方向键
    /// 瞄准"，与同一行的"自瞄=开"自相矛盾，也会让人误判自瞄没生效。
    pub fn help_mode(&self, auto_aim: bool) -> &'static str {
        if auto_aim { "自瞄" } else { "手动" }
    }

    pub fn help_controls(&self, auto_aim: bool) -> &'static str {
        if auto_aim {
            self.help.auto_aim
        } else {
            self.help.manual
        }
    }

    pub fn controlled_chassis_spin(&self) -> bool {
        self.controlled_chassis_spin.is_on()
    }

    pub fn remote_chassis_spin(&self) -> bool {
        self.remote_chassis_spin.is_on()
    }

    pub fn active_gamepad(&self) -> Option<Entity> {
        self.active_gamepad
    }

    fn toggle_keyboard_auto_aim(&mut self) {
        self.keyboard_auto_aim = !self.keyboard_auto_aim;
    }

    fn toggle_controlled_chassis_spin(&mut self) {
        self.controlled_chassis_spin.toggle();
    }

    fn toggle_remote_chassis_spin(&mut self) {
        self.remote_chassis_spin.toggle();
    }

    fn use_help(&mut self, help: ControllerHelp) {
        self.help = help;
    }

    fn use_gamepad(&mut self, gamepad: Entity) {
        self.active_gamepad = Some(gamepad);
    }

    fn clear_gamepad(&mut self) {
        self.active_gamepad = None;
    }
}

pub fn clear_controller_input(mut controller: ResMut<ControllerState>) {
    controller.reset_frame();
}

pub fn sample_keyboard_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut controller: ResMut<ControllerState>,
) {
    let keyboard_used = keyboard_controller_active(&keyboard);
    if keyboard_used {
        controller.use_help(ControllerHelp::keyboard());
        controller.clear_gamepad();
    }
    if keyboard.just_pressed(KeyCode::KeyQ) {
        controller.toggle_controlled_chassis_spin();
    }
    if keyboard.just_pressed(KeyCode::KeyU) {
        controller.toggle_remote_chassis_spin();
    }

    let controlled_chassis_yaw = controller.controlled_chassis_spin.yaw_input();
    let controlled = &mut controller.controlled;
    controlled.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
    ));
    controlled.add_chassis(controlled_chassis_yaw, 0.0, 0.0);
    controlled.add_gimbal(Vec2::new(
        keyboard_axis(&keyboard, KeyCode::ArrowLeft, KeyCode::ArrowRight),
        keyboard_axis(&keyboard, KeyCode::ArrowUp, KeyCode::ArrowDown),
    ));
    controlled.boost |= keyboard.pressed(KeyCode::ShiftLeft);
    controlled.shoot |= keyboard.pressed(KeyCode::Space);
    controlled.dart_just_pressed |= keyboard.just_pressed(KeyCode::KeyG);
    controlled.switch_slapper_just_pressed |= keyboard.just_pressed(KeyCode::Tab);
    controlled.switch_camera_just_pressed |= keyboard.just_pressed(KeyCode::F3);

    let remote_chassis_yaw = controller.remote_chassis_spin.yaw_input();
    let remote = &mut controller.remote;
    remote.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
    ));
    remote.add_chassis(
        remote_chassis_yaw,
        keyboard_axis(&keyboard, KeyCode::BracketLeft, KeyCode::BracketRight),
        keyboard_axis(&keyboard, KeyCode::Semicolon, KeyCode::Quote),
    );
    if !keyboard.pressed(KeyCode::ShiftLeft) {
        remote.add_gimbal(Vec2::new(
            keyboard_axis(&keyboard, KeyCode::KeyC, KeyCode::KeyB),
            0.0,
        ));
    }
    remote.add_gimbal(Vec2::new(
        0.0,
        keyboard_axis(&keyboard, KeyCode::KeyF, KeyCode::KeyV),
    ));
    remote.boost |= keyboard.pressed(KeyCode::ShiftRight);

    if keyboard.just_pressed(KeyCode::F5) {
        controller.toggle_keyboard_auto_aim();
    }
}

/// 鼠标位移 -> 云台瞄准增量。
///
/// 第一人称（`FollowingType::Robot`）的相机是刚性挂在云台挂载点上的
/// （见 `update_camera_follow`），所以"用鼠标转视角"在这个仿真器里就等于
/// "用鼠标转云台"，和真车操作手的手感一致，也不需要给相机再加一套独立朝向。
///
/// 之前这条通路完全不存在：全树只有 `freecam_controls` 读 `MouseMotion`，
/// 而它开头就 `if mode.0 != FollowingType::Free { return; }`。于是第一人称下
/// 鼠标怎么动都没反应，只能用方向键瞄准。
///
/// Free 视角刻意不处理：那里鼠标归 `freecam_controls`，两边都吃同一批
/// `MouseMotion` 会让自由相机和云台一起转。
pub fn sample_mouse_controller(
    mut mouse_motion: MessageReader<MouseMotion>,
    capture: Res<MouseCapture>,
    mode: Res<CameraMode>,
    config: Res<SimulationConfig>,
    mut controller: ResMut<ControllerState>,
) {
    if mode.0 == FollowingType::Free || !capture.captured {
        // 事件必须照样读掉。留在队列里下次一起读，就会在重新捕获的瞬间
        // 把释放期间攒下的全部位移一次性甩到云台上。
        mouse_motion.clear();
        return;
    }

    let mut delta = Vec2::ZERO;
    for motion in mouse_motion.read() {
        delta += motion.delta;
    }
    if delta == Vec2::ZERO {
        return;
    }

    controller.use_help(ControllerHelp::keyboard());

    // 符号与 `freecam_controls` 保持一致：鼠标右移 = 视角右转 = yaw 减小
    // （bevy 绕 +Y 为左转），鼠标上移（`delta.y` 为负）= 抬头 = pitch 增大。
    // `gimbal_controls` 里 `local_yaw += gimbal.x * speed`、方向键左是 +1，
    // 所以这里取负号后两种输入方向一致。
    let scale = config.camera.mouse_sensitivity * controller.controlled.gimbal_scale();
    controller
        .controlled
        .add_gimbal_delta(Vec2::new(-delta.x, -delta.y) * scale);
}

pub fn sample_gamepad_controller(
    gamepads: Query<(Entity, &Gamepad)>,
    mut controller: ResMut<ControllerState>,
    mut rumble_requests: MessageWriter<GamepadRumbleRequest>,
) {
    let Some((gamepad_entity, gamepad)) = gamepads.iter().next() else {
        return;
    };

    let left_stick = apply_stick_deadzone(gamepad.left_stick());
    let right_stick = apply_stick_deadzone(gamepad.right_stick());
    let dpad = gamepad.dpad();
    if gamepad_controller_active(gamepad, left_stick, right_stick, dpad) {
        controller.use_help(ControllerHelp::xbox());
        controller.use_gamepad(gamepad_entity);
    }
    if gamepad.just_pressed(GamepadButton::LeftTrigger) {
        controller.toggle_controlled_chassis_spin();
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::weak_motor(0.25),
            Duration::from_millis(70),
        );
    }
    if gamepad.just_pressed(GamepadButton::North) {
        controller.toggle_remote_chassis_spin();
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::weak_motor(0.25),
            Duration::from_millis(70),
        );
    }
    if gamepad.just_pressed(GamepadButton::RightTrigger2) {
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::strong_motor(0.12),
            Duration::from_millis(60),
        );
    }

    let controlled_chassis_yaw = controller.controlled_chassis_spin.yaw_input();
    let remote_chassis_yaw = controller.remote_chassis_spin.yaw_input();
    let adjusting_chassis_tilt = gamepad.pressed(GamepadButton::RightThumb);
    let controlled = &mut controller.controlled;
    controlled.add_movement(left_stick);
    controlled.add_chassis(controlled_chassis_yaw, 0.0, 0.0);
    if !adjusting_chassis_tilt {
        controlled.add_gimbal(Vec2::new(-right_stick.x, right_stick.y));
    }
    controlled.precise_gimbal |=
        gamepad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD;
    controlled.boost |= gamepad.pressed(GamepadButton::LeftThumb);
    controlled.auto_aim |=
        gamepad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD;
    controlled.shoot |= gamepad.pressed(GamepadButton::RightTrigger);
    controlled.dart_just_pressed |= gamepad.just_pressed(GamepadButton::West);
    controlled.switch_slapper_just_pressed |= gamepad.just_pressed(GamepadButton::Start);
    controlled.switch_camera_just_pressed |= gamepad.just_pressed(GamepadButton::Select);

    let remote = &mut controller.remote;
    remote.add_movement(-dpad);
    if adjusting_chassis_tilt {
        remote.add_chassis(remote_chassis_yaw, right_stick.x, right_stick.y);
    } else {
        remote.add_chassis(remote_chassis_yaw, 0.0, 0.0);
    }
}

/// Automated verification (headless CI, scripted closed-loop runs) has no way to
/// press F5 or hold RT, so allow the subscription to be forced on from the
/// environment. Mirrors `DAEDALUS_FORCE_TALOS_CAPTURE`; interactive behaviour is
/// unchanged when the variable is absent.
fn force_auto_aim_from_env() -> bool {
    std::env::var("DAEDALUS_FORCE_AUTO_AIM")
        .map(|v| v == "1")
        .unwrap_or(false)
}

pub fn update_auto_aim_subscription(
    controller: Res<ControllerState>,
    enabled: Res<SubscribeAutoAim>,
) {
    let active = controller.auto_aim_active() || force_auto_aim_from_env();
    if enabled.swap(active, Ordering::AcqRel) != active {
        info!(
            "Auto-aim subscription is now {}.",
            if active { "ENABLED" } else { "DISABLED" }
        );
    }
}

pub fn controller_shoot_pressed(controller: Res<ControllerState>) -> bool {
    controller.controlled.shoot
}

pub fn controller_dart_just_pressed(controller: Res<ControllerState>) -> bool {
    controller.controlled.dart_just_pressed
}

pub fn request_controller_rumble(
    controller: Option<&ControllerState>,
    rumble_requests: &mut MessageWriter<GamepadRumbleRequest>,
    intensity: GamepadRumbleIntensity,
    duration: Duration,
) {
    let Some(gamepad) = controller.and_then(ControllerState::active_gamepad) else {
        return;
    };
    request_gamepad_rumble(gamepad, rumble_requests, intensity, duration);
}

fn request_gamepad_rumble(
    gamepad: Entity,
    rumble_requests: &mut MessageWriter<GamepadRumbleRequest>,
    intensity: GamepadRumbleIntensity,
    duration: Duration,
) {
    rumble_requests.write(GamepadRumbleRequest::Add {
        gamepad,
        intensity,
        duration,
    });
}

fn keyboard_vec2(
    keyboard: &ButtonInput<KeyCode>,
    forward: KeyCode,
    left: KeyCode,
    backward: KeyCode,
    right: KeyCode,
) -> Vec2 {
    let mut input = Vec2::ZERO;
    if keyboard.pressed(forward) {
        input.y += 1.0;
    }
    if keyboard.pressed(backward) {
        input.y -= 1.0;
    }
    if keyboard.pressed(right) {
        input.x += 1.0;
    }
    if keyboard.pressed(left) {
        input.x -= 1.0;
    }
    input
}

fn keyboard_axis(keyboard: &ButtonInput<KeyCode>, positive: KeyCode, negative: KeyCode) -> f32 {
    let mut input = 0.0;
    if keyboard.pressed(positive) {
        input += 1.0;
    }
    if keyboard.pressed(negative) {
        input -= 1.0;
    }
    input
}

fn keyboard_controller_active(keyboard: &ButtonInput<KeyCode>) -> bool {
    const HELD_KEYS: [KeyCode; 22] = [
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
        KeyCode::ArrowLeft,
        KeyCode::ArrowRight,
        KeyCode::ArrowUp,
        KeyCode::ArrowDown,
        KeyCode::ShiftLeft,
        KeyCode::Space,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
        KeyCode::BracketLeft,
        KeyCode::BracketRight,
        KeyCode::Semicolon,
        KeyCode::Quote,
        KeyCode::KeyC,
        KeyCode::KeyB,
        KeyCode::KeyF,
        KeyCode::KeyV,
    ];
    const EDGE_KEYS: [KeyCode; 6] = [
        KeyCode::KeyG,
        KeyCode::Tab,
        KeyCode::F3,
        KeyCode::F5,
        KeyCode::KeyQ,
        KeyCode::KeyU,
    ];

    HELD_KEYS.iter().any(|&key| keyboard.pressed(key))
        || EDGE_KEYS.iter().any(|&key| keyboard.just_pressed(key))
}

fn gamepad_controller_active(
    gamepad: &Gamepad,
    left_stick: Vec2,
    right_stick: Vec2,
    dpad: Vec2,
) -> bool {
    left_stick != Vec2::ZERO
        || right_stick != Vec2::ZERO
        || dpad != Vec2::ZERO
        || gamepad.pressed(GamepadButton::LeftTrigger)
        || gamepad.pressed(GamepadButton::LeftThumb)
        || gamepad.pressed(GamepadButton::RightThumb)
        || gamepad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD
        || gamepad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD
        || gamepad.pressed(GamepadButton::RightTrigger)
        || gamepad.just_pressed(GamepadButton::North)
        || gamepad.just_pressed(GamepadButton::West)
        || gamepad.just_pressed(GamepadButton::Start)
        || gamepad.just_pressed(GamepadButton::Select)
}

fn apply_stick_deadzone(input: Vec2) -> Vec2 {
    let length = input.length();
    if length <= GAMEPAD_STICK_DEADZONE {
        return Vec2::ZERO;
    }
    let scaled = ((length - GAMEPAD_STICK_DEADZONE) / (1.0 - GAMEPAD_STICK_DEADZONE)).min(1.0);
    input / length * scaled
}

fn clamp_axes_vec2(input: Vec2) -> Vec2 {
    Vec2::new(input.x.clamp(-1.0, 1.0), input.y.clamp(-1.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadzone_filters_small_stick_noise() {
        assert_eq!(apply_stick_deadzone(Vec2::new(0.05, 0.05)), Vec2::ZERO);
    }

    #[test]
    fn deadzone_preserves_full_stick_deflection() {
        assert_eq!(apply_stick_deadzone(Vec2::X), Vec2::X);
        assert_eq!(apply_stick_deadzone(Vec2::Y), Vec2::Y);
    }

    #[test]
    fn controller_input_clamps_each_axis_without_changing_diagonal_input() {
        let mut input = ControllerInput::default();
        input.add_movement(Vec2::X);
        input.add_movement(Vec2::Y);

        assert_eq!(input.movement, Vec2::ONE);

        input.add_movement(Vec2::ONE);
        assert_eq!(input.movement, Vec2::ONE);
    }

    #[test]
    fn help_provider_switches_between_manual_and_auto_aim_modes() {
        let mut controller = ControllerState::default();
        controller.use_help(ControllerHelp::xbox());

        assert_eq!(controller.help_source(), "xbox");
        assert_eq!(controller.help_mode(false), "手动");
        assert!(controller.help_controls(false).contains("按住 RT"));

        // 生效标志由调用方传入，与 auto_aim_active() 解耦：DAEDALUS_FORCE_AUTO_AIM
        // 场景下 F5/RT 都没按过，但订阅是开的，HUD 必须显示"自瞄"。
        assert_eq!(controller.help_mode(true), "自瞄");
        assert!(controller.help_controls(true).contains("松开 RT"));

        // auto_aim_active() 仍然只反映 F5/RT，这正是它不能直接用于 HUD 的原因。
        assert!(!controller.auto_aim_active());
        controller.controlled.auto_aim = true;
        assert!(controller.auto_aim_active());
    }

    #[test]
    fn reset_frame_preserves_last_help_provider() {
        let mut controller = ControllerState::default();
        controller.use_help(ControllerHelp::xbox());

        controller.reset_frame();

        assert_eq!(controller.help_source(), "xbox");
    }

    /// 起一个只装了鼠标采样所需资源的最小 App。
    ///
    /// 直接调函数测不了这条链路的关键部分：`MessageReader` 的游标、"未捕获时
    /// 必须把事件读掉"这两件事都只在真的跑 schedule 时才成立。
    fn mouse_app(mode: FollowingType, captured: bool) -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_message::<MouseMotion>();
        app.insert_resource(CameraMode(mode));
        app.insert_resource(MouseCapture { captured });
        app.insert_resource(SimulationConfig::default());
        app.init_resource::<ControllerState>();
        app.add_systems(Update, sample_mouse_controller);
        app
    }

    fn push_motion(app: &mut App, delta: Vec2) {
        app.world_mut().write_message(MouseMotion { delta });
    }

    fn gimbal_delta(app: &App) -> Vec2 {
        app.world().resource::<ControllerState>().controlled.gimbal_delta
    }

    #[test]
    fn mouse_motion_aims_gimbal_in_first_person() {
        let mut app = mouse_app(FollowingType::Robot, true);
        let sensitivity = app.world().resource::<SimulationConfig>().camera.mouse_sensitivity;

        push_motion(&mut app, Vec2::new(10.0, 4.0));
        app.update();

        // 鼠标右移 -> yaw 减小（bevy 绕 +Y 为左转）；鼠标下移(delta.y>0) -> 低头。
        // 这两个符号必须与 freecam_controls 一致，否则切视角时手感反向。
        let delta = gimbal_delta(&app);
        assert!((delta.x - (-10.0 * sensitivity)).abs() < 1e-6, "yaw 方向或标度不对: {delta:?}");
        assert!((delta.y - (-4.0 * sensitivity)).abs() < 1e-6, "pitch 方向或标度不对: {delta:?}");
    }

    #[test]
    fn mouse_motion_accumulates_within_one_frame() {
        let mut app = mouse_app(FollowingType::Robot, true);
        let sensitivity = app.world().resource::<SimulationConfig>().camera.mouse_sensitivity;

        push_motion(&mut app, Vec2::new(3.0, 0.0));
        push_motion(&mut app, Vec2::new(4.0, 0.0));
        app.update();

        // 一帧内可能来多个 MouseMotion，只取最后一个就会丢掉大部分位移。
        assert!((gimbal_delta(&app).x - (-7.0 * sensitivity)).abs() < 1e-6);
    }

    #[test]
    fn released_cursor_drops_motion_instead_of_buffering_it() {
        let mut app = mouse_app(FollowingType::Robot, false);

        push_motion(&mut app, Vec2::new(500.0, 0.0));
        app.update();
        assert_eq!(gimbal_delta(&app), Vec2::ZERO, "未捕获时不应产生瞄准输入");

        // 重新捕获后，释放期间攒下的位移不能被一次性甩到云台上。
        app.world_mut().resource_mut::<MouseCapture>().captured = true;
        app.update();
        assert_eq!(gimbal_delta(&app), Vec2::ZERO, "释放期间的位移被缓存后补发了");
    }

    #[test]
    fn free_camera_keeps_mouse_for_itself() {
        let mut app = mouse_app(FollowingType::Free, true);

        push_motion(&mut app, Vec2::new(10.0, 10.0));
        app.update();

        // Free 视角下鼠标归 freecam_controls；两边都吃会让相机和云台一起转。
        assert_eq!(gimbal_delta(&app), Vec2::ZERO);
    }

    #[test]
    fn reset_frame_clears_mouse_aim_delta() {
        let mut controller = ControllerState::default();
        controller.controlled.add_gimbal_delta(Vec2::new(0.1, 0.1));

        controller.reset_frame();

        // 不清零的话鼠标停下后云台会按最后一帧的增量一直转。
        assert_eq!(controller.controlled.gimbal_delta, Vec2::ZERO);
    }

    #[test]
    fn reset_frame_preserves_chassis_spin_modes() {
        let mut controller = ControllerState::default();
        controller.toggle_controlled_chassis_spin();
        controller.toggle_remote_chassis_spin();

        controller.reset_frame();

        assert!(controller.controlled_chassis_spin());
        assert!(controller.remote_chassis_spin());
    }
}
