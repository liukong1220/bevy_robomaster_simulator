use bevy::prelude::*;
use bevy::render::view::screenshot::{Capturing, Screenshot, save_to_disk};
use bevy::window::{CursorIcon, SystemCursorIcon, Window};

use crate::components::{SlapperInfantry, SubscribeAutoAim};
use crate::robomaster::prelude::{Armor, ArmorStickerSelection};
use crate::statistic::ProjectileStatistics;
use crate::systems::ControllerState;

fn create_help_text(
    auto_aim: bool,
    stats: &ProjectileStatistics,
    controller: &ControllerState,
) -> Text {
    format!(
        "自瞄={} 发弹总数={} 命中={} 命中率={:.2}%\n控制器={} 模式={} 小陀螺={} 远程小陀螺={}\n{}",
        if auto_aim { "开" } else { "关" },
        stats.launch_count,
        stats.accurate_count,
        stats.accurate_pct(),
        controller.help_source(),
        controller.help_mode(auto_aim),
        if controller.controlled_chassis_spin() {
            "开"
        } else {
            "关"
        },
        if controller.remote_chassis_spin() {
            "开"
        } else {
            "关"
        },
        controller.help_controls(auto_aim)
    )
    .into()
}

pub fn spawn_text(commands: &mut Commands, asset_server: &AssetServer) {
    commands.spawn((
        Text::new(""),
        // bevy 内置的默认字体只有拉丁字形，中文会整段渲染成缺字框。
        // assets/fonts/hud_cjk.otf 是从 Noto Sans CJK SC 子集化出来的
        // （402 个字形 = src/ 下所有字符串字面量里的非 ASCII 字符 + 可打印
        // ASCII，174KB），避免把 19MB 的 CJK 全字库入库。
        //
        // 子集必须按源码里实际出现的字符生成，不能手写字表：第一版是手列的，
        // 漏了"数""方向键""空格"等等，HUD 上就是一个个缺字框（□），而 Rust
        // 编译期完全不会提示。改动任何界面文案后要重新生成，判据是
        // fontTools 校验"字面量里的每个非 ASCII 字符都在 cmap 里"。
        TextFont {
            font: FontSource::Handle(asset_server.load("fonts/hud_cjk.otf")),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

pub fn update_help_text(
    mut text: Query<&mut Text>,
    auto_aim: Res<SubscribeAutoAim>,
    stats: Res<ProjectileStatistics>,
    controller: Res<ControllerState>,
) {
    let next = create_help_text(
        auto_aim.load(std::sync::atomic::Ordering::Acquire),
        &stats,
        &controller,
    );
    for mut text in text.iter_mut() {
        // 只在内容真的变了才写。无条件 `*text = ...` 每帧都触发变更检测，
        // 于是 bevy 每帧重新做一次文字整形（parley/icu 断行 + 字形光栅化），
        // 这段文本大部分帧完全没变。中文之后单帧成本更高，也是那条
        // icu warn 每帧刷一遍的直接原因。
        if text.0 != next.0 {
            *text = next.clone();
        }
    }
}

/// 只写日志的命中统计，供闭环评估读取，不参与任何算法输入。
///
/// `launch_count` / `accurate_count` 原来只渲染到窗口左下角的帮助文字里
/// （见 `update_help_text`）。闭环跑的是离屏采集，没人看那个窗口，于是
/// "打了多少发、命中多少发" 在日志和共享内存里都拿不到，虚拟开火只能确认
/// "指令发出去了"，没法确认 "打中了"。这里在计数变化时打一行 info，
/// 由 DAEDALUS_LOG_HITS=1 开启，默认完全不输出。
///
/// 注意这是**真值**，只允许流向评估：它既不进共享内存，也不回灌算法。
pub fn log_projectile_stats(
    stats: Res<ProjectileStatistics>,
    mut enabled: Local<Option<bool>>,
    mut last: Local<(u32, u32)>,
) {
    let on = *enabled.get_or_insert_with(|| {
        std::env::var("DAEDALUS_LOG_HITS")
            .map(|v| v.trim() == "1")
            .unwrap_or(false)
    });
    if !on {
        return;
    }
    let now = (stats.launch_count, stats.accurate_count);
    if now == *last {
        return;
    }
    *last = now;
    info!(
        "[hits] launch={} accurate={} pct={:.2}",
        now.0,
        now.1,
        stats.accurate_pct()
    );
}

pub fn change_appearance(
    keyboard: Res<ButtonInput<KeyCode>>,
    selections: Query<&mut ArmorStickerSelection, With<SlapperInfantry>>,
    owned: Query<&mut Armor, With<SlapperInfantry>>,
) {
    if keyboard.pressed(KeyCode::ShiftLeft) && keyboard.just_pressed(KeyCode::KeyC) {
        let mut n_type = None;
        for mut selection in selections {
            let new_typ = selection.advance_debug_sequence();
            n_type = Some(new_typ);
        }
        if let Some(n_type) = n_type {
            for mut own in owned {
                own.label = n_type;
            }
        }
    }
}

/// 到点自动截一张图，路径与延迟由环境变量给出：
///   DAEDALUS_SCREENSHOT_PATH=/tmp/x.png DAEDALUS_SCREENSHOT_AFTER_S=8
///
/// F2 那条路径要求有人按键。脚本化运行（无头 CI、闭环回归）没法按键，而
/// 用 x11grab 截屏又只能拿到合成器最上层的窗口——仿真窗口被别的应用盖住时
/// 抓到的是全黑，本机就是这样。这里走 bevy 自己的 Screenshot，直接读渲染
/// 目标，与窗口是否可见、是否被遮挡无关。
/// 不设 DAEDALUS_SCREENSHOT_PATH 时完全不生效，交互行为不变。
pub fn screenshot_on_timer(
    mut commands: Commands,
    time: Res<Time>,
    mut cfg: Local<Option<Option<(String, f32)>>>,
    mut done: Local<bool>,
) {
    let cfg = cfg.get_or_insert_with(|| {
        let path = std::env::var("DAEDALUS_SCREENSHOT_PATH").ok()?;
        let after = std::env::var("DAEDALUS_SCREENSHOT_AFTER_S")
            .ok()
            .and_then(|v| v.trim().parse::<f32>().ok())
            .unwrap_or(8.0);
        Some((path, after))
    });
    let Some((path, after)) = cfg.as_ref() else {
        return;
    };
    if *done || time.elapsed_secs() < *after {
        return;
    }
    *done = true;
    let path = path.clone();
    info!("[screenshot] 定时截图 -> {}", path);
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
}

pub fn screenshot_on_f2(mut commands: Commands, mut counter: Local<u32>) {
    let path = format!("./screenshot-{}.png", *counter);
    *counter += 1;
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
}

pub fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}
