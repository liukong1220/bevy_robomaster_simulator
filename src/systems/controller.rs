use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
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
            manual: "F3 视角 | WASD 移动 | 左键 开火（未捕获指针时先捕获）/ Esc 释放 | 右键按住 自瞄+火控开火（先左键捕获指针）| 鼠标或方向键 瞄准 | 空格 射击 | G 飞镖 | Q 小陀螺 | U 远程小陀螺 | F5 自瞄常开 | Tab 拍打",
            auto_aim: "松开右键 / F5 关闭自瞄（外部接管期间鼠标与方向键不控云台）| 识别到目标由外部火控自动开火，不用按左键（视觉侧须带 --allow-fire）| 左键 开火 | WASD 移动 | Q 小陀螺 | U 远程小陀螺 | Tab 拍打",
        }
    }

    const fn xbox() -> Self {
        Self {
            source: "xbox",
            manual: "View 视角 | LS 移动 | L3 加速 | 十字键 拍打移动 | RS 瞄准 | R3+RS 拍打翻滚/俯仰 | LB 小陀螺 | Y 拍打小陀螺 | RB 射击 | X 飞镖 | 按住 RT 自瞄+火控开火",
            auto_aim: "松开 RT 关闭自瞄 | LS 移动 | L3 加速 | 十字键 拍打移动 | R3+RS 拍打翻滚/俯仰 | LB 小陀螺 | Y 拍打小陀螺 | 识别到目标由外部火控自动开火（视觉侧须带 --allow-fire）",
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

    fn clear_keyboard_auto_aim(&mut self) {
        self.keyboard_auto_aim = false;
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

/// 鼠标按键 -> 开火与自瞄。
///
/// 约定按真车操作手的习惯：**左键开火、按住右键自瞄、松开右键回手动**。
///
/// 两个细节必须在这里说清楚，否则表现就是"点一下就走火"或者"按了右键没反应"：
///
/// 1. 这个系统刻意排在 `update_cursor_capture` **之前**，读到的是本帧点击之前的
///    捕获状态。左键同时还是"捕获指针"的按键（见 `update_cursor_capture`），
///    如果先更新捕获状态，那次用来捕获窗口的点击会在同一帧被当成开火。
///    排在前面之后，捕获用的那一下不开火，继续按住则从下一帧起连续开火
///    （连发节流仍由 `ProjectileCooldown` 负责，与空格键同一条路径）。
/// 2. 按下右键会清掉 F5 的常开锁存。`auto_aim_active()` 是"锁存 or 按住"的或，
///    锁存着的时候松开右键并不会关掉自瞄——那与"松开右键关闭自瞄"直接矛盾。
///    所以让按住的那一方接管：右键一按下就把锁存清零，之后松手必定回手动。
/// 3. `--show-detect` 的 OpenCV 窗口会抢走焦点。Released 进不了 Bevy 时
///    `pressed(Right)` 会卡住。窗口失焦时主动 release 鼠标键，回到窗口必须重新按下。
pub fn sample_mouse_buttons(
    mut mouse_button: ResMut<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    capture: Res<MouseCapture>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut controller: ResMut<ControllerState>,
) {
    let focused = windows.iter().next().map(|window| window.focused).unwrap_or(true);

    // `--show-detect` 的 OpenCV 窗口会抢走焦点：松开右键的 Released 落到别的窗口，
    // Bevy 的 `pressed(Right)` 会一直为 true，表现为"松开右键却不取消自瞄"。
    // 失焦时主动清掉鼠标键状态；回到窗口后必须重新按下，不能沿用卡住的 pressed。
    if !focused || keyboard.just_pressed(KeyCode::Escape) {
        mouse_button.release(MouseButton::Right);
        mouse_button.release(MouseButton::Left);
        return;
    }

    // 没捕获指针时不接受开火/自瞄：那时候指针可能正在点 egui inspector 或别的窗口。
    if !capture.captured {
        return;
    }
    if mouse_button.just_pressed(MouseButton::Right) {
        controller.clear_keyboard_auto_aim();
    }
    let hold_auto_aim = mouse_button.pressed(MouseButton::Right);
    let fire = mouse_button.pressed(MouseButton::Left);
    controller.controlled.auto_aim |= hold_auto_aim;
    controller.controlled.shoot |= fire;
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
    use bevy::input::ButtonState;
    use bevy::input::keyboard::{Key, KeyboardInput};
    use bevy::input::mouse::MouseButtonInput;

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
        app.world()
            .resource::<ControllerState>()
            .controlled
            .gimbal_delta
    }

    #[test]
    fn mouse_motion_aims_gimbal_in_first_person() {
        let mut app = mouse_app(FollowingType::Robot, true);
        let sensitivity = app
            .world()
            .resource::<SimulationConfig>()
            .camera
            .mouse_sensitivity;

        push_motion(&mut app, Vec2::new(10.0, 4.0));
        app.update();

        // 鼠标右移 -> yaw 减小（bevy 绕 +Y 为左转）；鼠标下移(delta.y>0) -> 低头。
        // 这两个符号必须与 freecam_controls 一致，否则切视角时手感反向。
        let delta = gimbal_delta(&app);
        assert!(
            (delta.x - (-10.0 * sensitivity)).abs() < 1e-6,
            "yaw 方向或标度不对: {delta:?}"
        );
        assert!(
            (delta.y - (-4.0 * sensitivity)).abs() < 1e-6,
            "pitch 方向或标度不对: {delta:?}"
        );
    }

    #[test]
    fn mouse_motion_accumulates_within_one_frame() {
        let mut app = mouse_app(FollowingType::Robot, true);
        let sensitivity = app
            .world()
            .resource::<SimulationConfig>()
            .camera
            .mouse_sensitivity;

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
        assert_eq!(
            gimbal_delta(&app),
            Vec2::ZERO,
            "释放期间的位移被缓存后补发了"
        );
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

    /// 只装鼠标按键采样所需资源的最小 App。用 `InputPlugin` 而不是手搓
    /// `ButtonInput`，因为 `just_pressed` 的生命周期（每帧清一次）只有真的跑
    /// schedule 才成立，而"按下右键清锁存"正好依赖它。
    fn buttons_app(captured: bool) -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::input::InputPlugin);
        app.insert_resource(MouseCapture { captured });
        app.init_resource::<ControllerState>();
        app.add_systems(
            Update,
            (clear_controller_input, sample_mouse_buttons).chain(),
        );
        app
    }

    /// 必须走真实的输入消息，不能直接 `ButtonInput::press`：`InputPlugin` 的
    /// `mouse_button_input_system` 在 PreUpdate 里先 `clear()` 再消费消息，手写的
    /// `press()` 会在同一帧被那次 clear 抹掉 `just_pressed`，于是"按下右键清锁存"
    /// 这条判据在测试里永远看不到——而真实运行里它是成立的。
    fn send(app: &mut App, button: MouseButton, state: ButtonState) {
        app.world_mut().write_message(MouseButtonInput {
            button,
            state,
            window: Entity::PLACEHOLDER,
        });
    }

    fn press(app: &mut App, button: MouseButton) {
        send(app, button, ButtonState::Pressed);
    }

    fn release(app: &mut App, button: MouseButton) {
        send(app, button, ButtonState::Released);
    }

    fn state(app: &App) -> (bool, bool) {
        let c = app.world().resource::<ControllerState>();
        (c.controlled.shoot, c.auto_aim_active())
    }

    #[test]
    fn left_button_fires_while_held() {
        let mut app = buttons_app(true);
        press(&mut app, MouseButton::Left);
        app.update();
        assert_eq!(state(&app), (true, false));
        // 按住就连发（节流交给 ProjectileCooldown，与空格键同一条路径）。
        app.update();
        assert_eq!(state(&app), (true, false));
        release(&mut app, MouseButton::Left);
        app.update();
        assert_eq!(state(&app), (false, false));
    }

    #[test]
    fn right_button_holds_auto_aim_and_releasing_it_returns_manual() {
        let mut app = buttons_app(true);
        press(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, true));
        app.update();
        assert_eq!(state(&app), (false, true), "一直按住就一直自瞄");
        release(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, false), "松开右键必须回手动");
    }

    #[test]
    fn holding_the_right_button_clears_the_f5_latch() {
        // 否则 F5 常开着的时候松开右键还在自瞄，与"松开右键关闭自瞄"矛盾。
        let mut app = buttons_app(true);
        app.world_mut()
            .resource_mut::<ControllerState>()
            .toggle_keyboard_auto_aim();
        assert!(app.world().resource::<ControllerState>().auto_aim_active());

        press(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, true));
        release(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, false));
    }

    /// Esc 也必须走真实消息，理由同 `send`：`just_pressed` 只有真的跑一遍
    /// schedule 才有意义。
    fn press_escape(app: &mut App) {
        app.world_mut().write_message(KeyboardInput {
            key_code: KeyCode::Escape,
            logical_key: Key::Escape,
            state: ButtonState::Pressed,
            text: None,
            repeat: false,
            window: Entity::PLACEHOLDER,
        });
    }

    #[test]
    fn escape_while_the_buttons_are_held_does_not_fire_or_aim_again() {
        // 边沿：本系统排在 `update_cursor_capture` 之前，所以 Esc 释放指针的那一帧
        // 这里读到的 `captured` 还是 true。没有这道判据，"按住左键按 Esc" 会在交还
        // 指针的同一帧再吐一发，"按住右键按 Esc" 会再续一帧自瞄订阅。
        let mut app = buttons_app(true);
        press(&mut app, MouseButton::Left);
        press(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (true, true));

        press_escape(&mut app);
        app.update();
        assert_eq!(
            state(&app),
            (false, false),
            "按 Esc 的同一帧不能再产生开火或自瞄"
        );

        // 指针交还之后，键还按着也不再生效。
        app.world_mut().resource_mut::<MouseCapture>().captured = false;
        app.update();
        assert_eq!(state(&app), (false, false));
    }

    #[test]
    fn escape_leaves_the_f5_latch_alone() {
        // Esc 只是交还指针，不是"关自瞄"键。F5 是显式开关，按 Esc 之后仍然算生效，
        // 否则操作手会以为自瞄已经关了。松开右键才是关掉鼠标那一路自瞄。
        let mut app = buttons_app(true);
        app.world_mut()
            .resource_mut::<ControllerState>()
            .toggle_keyboard_auto_aim();
        press(&mut app, MouseButton::Left);
        app.update();
        assert_eq!(state(&app), (true, true));

        press_escape(&mut app);
        app.update();
        assert_eq!(
            state(&app),
            (false, true),
            "Esc 那一帧不再开火，但 F5 的锁存不受影响"
        );
    }

    #[test]
    fn unfocus_cancels_held_right_button_auto_aim() {
        // `--show-detect` 的 OpenCV 窗口抢走焦点后，Released 进不了 Bevy，
        // pressed(Right) 会卡住。失焦必须清掉右键自瞄，回到窗口也不能沿用。
        let mut app = buttons_app(true);
        let window = app
            .world_mut()
            .spawn((
                Window {
                    focused: true,
                    ..default()
                },
                PrimaryWindow,
            ))
            .id();

        press(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, true));

        app.world_mut().entity_mut(window).insert(Window {
            focused: false,
            ..default()
        });
        app.update();
        assert_eq!(state(&app), (false, false), "失焦必须取消右键自瞄");

        app.world_mut().entity_mut(window).insert(Window {
            focused: true,
            ..default()
        });
        app.update();
        assert_eq!(
            state(&app),
            (false, false),
            "失焦期间松开的右键不能在回到窗口后卡住"
        );
    }

    #[test]
    fn mouse_buttons_do_nothing_until_the_pointer_is_captured() {
        // 左键同时是"捕获指针"键。这个系统排在 update_cursor_capture 之前，所以
        // 用来捕获窗口的那一下读到的仍是 captured=false，不会走火。
        let mut app = buttons_app(false);
        press(&mut app, MouseButton::Left);
        press(&mut app, MouseButton::Right);
        app.update();
        assert_eq!(state(&app), (false, false));

        // 捕获之后（下一帧）按住的左键才开始开火。
        app.world_mut().resource_mut::<MouseCapture>().captured = true;
        app.update();
        assert_eq!(state(&app), (true, true));
    }

    #[test]
    fn left_and_right_buttons_work_together() {
        // 真车操作手的常用组合：按住右键自瞄、同时点左键开火。
        let mut app = buttons_app(true);
        press(&mut app, MouseButton::Right);
        press(&mut app, MouseButton::Left);
        app.update();
        assert_eq!(state(&app), (true, true));
    }
}
