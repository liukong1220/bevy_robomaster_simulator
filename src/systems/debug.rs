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
        "auto-aim={} total={} accurate={} pct={:.2}\ncontroller={} mode={} gyro={} remote-gyro={}\n{}",
        if auto_aim { "ON " } else { "OFF" },
        stats.launch_count,
        stats.accurate_count,
        stats.accurate_pct(),
        controller.help_source(),
        controller.help_mode(),
        if controller.controlled_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        if controller.remote_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        controller.help_controls()
    )
    .into()
}

pub fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        Text::new(""),
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
    for mut text in text.iter_mut() {
        *text = create_help_text(
            auto_aim.load(std::sync::atomic::Ordering::Acquire),
            &stats,
            &controller,
        );
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
