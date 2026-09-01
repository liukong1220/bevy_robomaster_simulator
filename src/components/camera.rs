use bevy::prelude::*;
use std::sync::atomic::AtomicBool;

#[derive(Component)]
pub struct MainCamera {
    pub follow_offset: Vec3,
}

#[derive(Resource, PartialEq, Deref, DerefMut)]
pub struct CameraMode(pub FollowingType);

impl Default for CameraMode {
    fn default() -> Self {
        Self(FollowingType::Robot)
    }
}

#[derive(Resource, Deref, DerefMut)]
pub struct SubscribeAutoAim(pub AtomicBool);

#[derive(PartialEq, Clone, Copy)]
pub enum FollowingType {
    Free,
    Robot,
    ThirdPerson,
}

/// 鼠标是否被窗口捕获。捕获中才把鼠标位移当成瞄准输入。
///
/// 不做"进窗口就自动捕获"：那样一启动指针就被锁住，egui inspector
/// (`debug.egui`)、拖窗口、切别的程序都会变得别扭，而这个仿真器经常是开着
/// 窗口在旁边跑脚本化闭环的。改成显式的左键捕获 / Esc 释放，HUD 里写清楚。
///
/// X11 不支持 `CursorGrabMode::Locked`，bevy 会退化到 `Confined`：指针被限制
/// 在窗口内而不是钉在中心。这对"连续转视角"够用（指针撞不到屏幕边缘），
/// 但捕获期间仍要 `visible = false`，否则会看到指针在窗口里乱窜。
#[derive(Resource, Default)]
pub struct MouseCapture {
    pub captured: bool,
}
