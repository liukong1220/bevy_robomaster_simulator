//! 外部云台命令的控制权状态机。
//!
//! HUD 与"云台到底听谁的"读的是**同一个** `AutoAimLink`，不是两套逻辑。之前 HUD
//! 只是一行文字诊断，控制权则隐含在 `gimbal_controls` 的一句 `if subscribed
//! { return; }` 里，于是出现过这种情况：HUD 写着"自瞄=开"，视觉侧跑的是
//! `--mode=passive`（按设计只发安全停止、永不发控制），云台一动不动，方向键和鼠标
//! 同时是死的——看起来像"自瞄开了但不锁不跟"，实际是控制权被一条从不下发控制的
//! 链路占着。
//!
//! 所以这里把两件被混在一起的事分开：
//!
//! - **链路活性**：对端还在不在。任何通过校验的命令都算，包括安全停止。
//! - **控制权**：云台该听外部还是听人。只有**控制命令**才拿得到，而且只在租约内
//!   有效。安全停止是"我还活着，但我不控制"，它续活性、不续控制权。
//!
//! 于是 passive 下人能自己动云台（HUD 显示"待命"），closed_loop 下外部接管
//! （显示"接管"），而对端真的断了之后进入 `中断` 安全状态。

use crate::config::AutoAimLinkConfig;
use bevy::prelude::*;
use talos_ipc::GimbalCmd;

/// 链路状态。**这一个枚举同时决定 HUD 文案和实际控制权**。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinkState {
    /// 没订阅（F5 关）。云台归手动。
    #[default]
    Unsubscribed,
    /// 已订阅，但一条有效命令都没收到过：视觉侧没启动，或者发来的全被校验挡掉了。
    ///
    /// 控制权仍在手动。这是刻意的：外部从没接管过，就没有什么可以"交还"，
    /// 让操作者在这个阶段动不了云台只会让"忘了起视觉侧"看起来像自瞄坏了。
    Waiting,
    /// 对端活着，但租约内没有任何控制命令（passive 的安全停止流，或 closed_loop
    /// 丢目标且没开驻留）。控制权在手动。
    Standby,
    /// 外部接管中：租约内有过控制命令。云台归外部，允许虚拟开火。
    Engaged,
    /// `external_link_lost`：外部接管过，然后活性租约到期。
    ///
    /// 这是一个**独立的安全状态**，不是"回到手动"：云台保持在最后一条外部命令的
    /// 姿态上，禁止虚拟开火，控制权也不隐式交还给操作者。要拿回手动控制得显式按
    /// F5 关掉订阅。
    ///
    /// 为什么不自动恢复手动：一次脚本化闭环里，外部卡了半秒就把控制权丢回键盘，
    /// 之后这段轨迹到底是谁转的、这颗弹是谁打的，事后从记录里分不出来；而
    /// `gimbal_controls` 的手动路径一进来就会用手动限位重写 pitch，把外部写进去的
    /// 姿态直接吃掉。宁可冻住并在 HUD 上说清楚。
    Lost,
}

impl LinkState {
    /// 云台是否归外部。`Lost` 也算：那是"冻在最后一条外部命令上"，不是交还手动。
    pub fn external_owns_gimbal(self) -> bool {
        matches!(self, LinkState::Engaged | LinkState::Lost)
    }

    /// 手动（键盘/鼠标/手柄）能不能动云台。与上面严格互补。
    pub fn allows_manual_gimbal(self) -> bool {
        !self.external_owns_gimbal()
    }

    /// 是否允许虚拟开火。只有接管中才允许。
    pub fn allows_fire(self) -> bool {
        matches!(self, LinkState::Engaged)
    }

    /// HUD 文案。与状态一一对应，不另起一套判断。
    pub fn label(self) -> &'static str {
        match self {
            LinkState::Unsubscribed => "未订阅",
            LinkState::Waiting => "未收到",
            LinkState::Standby => "待命",
            LinkState::Engaged => "接管",
            LinkState::Lost => "中断",
        }
    }
}

/// 命令被拒的原因。逐类计数：报告里"被拒了多少条"没有原因是没法排查的。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectReason {
    /// yaw/pitch/distance 里有非有限值。
    NonFinite,
    /// `timestamp_ns == 0`：对端没填时间戳，年龄无从判断。
    TimestampZero,
    /// 时间戳没有前进（`<=` 上一条被接受的命令）。
    TimestampRegressed,
    /// 命令自身的时间戳太旧，超过租约。
    Expired,
    /// 时间戳落在未来超过一个租约：两端 wall clock 不一致，年龄判断已经不可信。
    TimestampFuture,
}

impl RejectReason {
    pub fn label(self) -> &'static str {
        match self {
            RejectReason::NonFinite => "非有限值",
            RejectReason::TimestampZero => "时间戳为0",
            RejectReason::TimestampRegressed => "时间戳倒退",
            RejectReason::Expired => "命令过期",
            RejectReason::TimestampFuture => "时间戳超前",
        }
    }
}

/// 一条命令的处置结论。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkVerdict {
    /// 按这条命令控制云台；`fire` 为真时才允许虚拟开火。
    Control { fire: bool },
    /// 安全停止：对端活着但要求不动。续活性租约，不动云台，不开火。
    SafeStop,
    /// 丢弃。
    Reject(RejectReason),
}

/// 各类拒收计数。
#[derive(Debug, Clone, Copy, Default)]
pub struct RejectCounts {
    pub non_finite: u64,
    pub timestamp_zero: u64,
    pub timestamp_regressed: u64,
    pub expired: u64,
    pub timestamp_future: u64,
}

impl RejectCounts {
    pub fn total(&self) -> u64 {
        self.non_finite
            + self.timestamp_zero
            + self.timestamp_regressed
            + self.expired
            + self.timestamp_future
    }

    fn record(&mut self, reason: RejectReason) {
        match reason {
            RejectReason::NonFinite => self.non_finite += 1,
            RejectReason::TimestampZero => self.timestamp_zero += 1,
            RejectReason::TimestampRegressed => self.timestamp_regressed += 1,
            RejectReason::Expired => self.expired += 1,
            RejectReason::TimestampFuture => self.timestamp_future += 1,
        }
    }
}

/// 外部链路的全部状态。**唯一的真相来源**：HUD、控制权、开火许可都读它。
#[derive(Resource, Debug)]
pub struct AutoAimLink {
    state: LinkState,
    /// 收到过的命令总数，含被拒的。
    pub cmd_count: u64,
    /// 通过校验的命令数。
    pub accepted: u64,
    /// 其中的控制命令数。
    pub control: u64,
    /// 其中的安全停止数。
    pub safe_stops: u64,
    pub rejects: RejectCounts,
    /// 进入 `Lost` 的次数。
    pub link_lost_count: u64,
    /// 最近一条命令的到达时刻（含被拒的），按 `Time::elapsed_secs`。
    pub last_cmd_secs: Option<f32>,
    /// 最近一条**被接受**的命令的到达时刻。活性租约看它。
    pub last_accept_secs: Option<f32>,
    /// 最近一条**控制**命令的到达时刻。控制权租约看它。
    pub last_control_secs: Option<f32>,
    /// 最近一条被接受的命令自带的时间戳。倒退检查的水位线。
    pub last_timestamp_ns: u64,
    /// 最近一次拒收的原因，给 HUD 用。
    pub last_reject: Option<RejectReason>,
}

impl Default for AutoAimLink {
    fn default() -> Self {
        Self {
            state: LinkState::Unsubscribed,
            cmd_count: 0,
            accepted: 0,
            control: 0,
            safe_stops: 0,
            rejects: RejectCounts::default(),
            link_lost_count: 0,
            last_cmd_secs: None,
            last_accept_secs: None,
            last_control_secs: None,
            last_timestamp_ns: 0,
            last_reject: None,
        }
    }
}

impl AutoAimLink {
    pub fn state(&self) -> LinkState {
        self.state
    }

    /// 订阅开关变化 + 租约到期。每帧都要调，包括没有命令到达的帧。
    pub fn tick(&mut self, subscribed: bool, now_secs: f32, cfg: &AutoAimLinkConfig) {
        if !subscribed {
            if self.state != LinkState::Unsubscribed {
                self.enter(LinkState::Unsubscribed);
            }
            return;
        }
        if self.state == LinkState::Unsubscribed {
            self.enter(LinkState::Waiting);
        }

        let lease = cfg.lease_secs();
        let alive = self.last_accept_secs.is_some_and(|t| now_secs - t <= lease);
        let controlling = self
            .last_control_secs
            .is_some_and(|t| now_secs - t <= lease);

        match self.state {
            // 接管中：先看还有没有控制命令，再看对端是否整体失联。
            // 对端还活着只是不控制 -> 降到待命（控制权回手动）；
            // 对端整体没了 -> Lost（冻住，不交还）。
            LinkState::Engaged if !controlling => {
                if alive {
                    self.enter(LinkState::Standby);
                } else {
                    self.enter(LinkState::Lost);
                }
            }
            // 待命时对端也没了：控制权本来就在手动，没什么可冻的，退回"未收到"。
            LinkState::Standby if !alive => self.enter(LinkState::Waiting),
            _ => {}
        }
    }

    /// 校验并处置一条命令。`now_ns` 是本地 wall clock（与对端同一纪元：
    /// C++ 侧用 `system_clock`，这边用 `SystemTime`/`UNIX_EPOCH`）。
    pub fn ingest(
        &mut self,
        cmd: &GimbalCmd,
        now_secs: f32,
        now_ns: u64,
        cfg: &AutoAimLinkConfig,
    ) -> LinkVerdict {
        self.cmd_count += 1;
        self.last_cmd_secs = Some(now_secs);

        if let Some(reason) = self.validate(cmd, now_ns, cfg) {
            self.rejects.record(reason);
            self.last_reject = Some(reason);
            return LinkVerdict::Reject(reason);
        }

        self.accepted += 1;
        self.last_accept_secs = Some(now_secs);
        self.last_timestamp_ns = cmd.timestamp_ns;

        // 安全停止只续活性，不给控制权：它的语义是"我活着，但我不控制"。
        if cmd.distance_m == SAFE_STOP_DISTANCE {
            self.safe_stops += 1;
            // Lost 是外部接管后断线的安全锁存。即使对端随后补发一条安全停止，
            // 也不能把控制权隐式交还给手动：注释和 HUD 的约定是必须由操作者
            // 显式关闭订阅（F5）才能解锁，避免断线恢复窗口出现控制权跳变。
            if self.state != LinkState::Engaged && self.state != LinkState::Lost {
                self.enter(LinkState::Standby);
            }
            return LinkVerdict::SafeStop;
        }

        self.control += 1;
        self.last_control_secs = Some(now_secs);
        self.enter(LinkState::Engaged);
        LinkVerdict::Control {
            fire: cmd.fire_advice == 1,
        }
    }

    /// 只做校验，不改状态。返回 `None` 表示通过。
    fn validate(
        &self,
        cmd: &GimbalCmd,
        now_ns: u64,
        cfg: &AutoAimLinkConfig,
    ) -> Option<RejectReason> {
        // 非有限值绝不能进 Transform：`to_radians()` 之后写进 rotation 会把四元数
        // 永久污染成 NaN，之后每一帧的相机、弹丸出膛点、真值全都跟着变 NaN，而且
        // 关掉订阅也回不来。distance 也要查：它会进弹道解算。
        if !cmd.yaw_deg.is_finite() || !cmd.pitch_deg.is_finite() || !cmd.distance_m.is_finite() {
            return Some(RejectReason::NonFinite);
        }
        if cmd.timestamp_ns == 0 {
            return Some(RejectReason::TimestampZero);
        }
        if cmd.timestamp_ns <= self.last_timestamp_ns {
            return Some(RejectReason::TimestampRegressed);
        }
        let lease_ns = cfg.lease_ns();
        if cmd.timestamp_ns > now_ns.saturating_add(lease_ns) {
            return Some(RejectReason::TimestampFuture);
        }
        if now_ns.saturating_sub(cmd.timestamp_ns) > lease_ns {
            return Some(RejectReason::Expired);
        }
        None
    }

    fn enter(&mut self, next: LinkState) {
        if self.state == next {
            return;
        }
        if next == LinkState::Lost {
            self.link_lost_count += 1;
        }
        // 离开"有过有效命令"的状态时清掉时间戳水位线。
        //
        // 不清的话对端重启后就再也接不上：新进程的第一条命令时间戳只要比断线前的
        // 水位线低（wall clock 回拨、或者只是同一毫秒内的先后），就会被判成"时间戳
        // 倒退"而永久拒收，链路卡在 Lost 上，表现是"重启视觉侧之后自瞄再也不接管"。
        if matches!(
            next,
            LinkState::Unsubscribed | LinkState::Waiting | LinkState::Lost
        ) {
            self.last_timestamp_ns = 0;
        }
        self.state = next;
    }
}

/// 安全停止的距离哨兵值，与 C++ 侧 `SimGimbal::encode` 一致。
pub const SAFE_STOP_DISTANCE: f32 = -1.0;

#[cfg(test)]
mod tests {
    use super::*;

    const LEASE_MS: f32 = 500.0;
    /// 一个远离 0 的 wall clock 基准，避免负年龄被夹成 0
    /// 而让"时间戳超前"这条判据看起来永远不触发。
    const T0_NS: u64 = 1_700_000_000_000_000_000;

    fn cfg() -> AutoAimLinkConfig {
        AutoAimLinkConfig { lease_ms: LEASE_MS }
    }

    /// 控制命令：distance 是正数。
    fn control(ts_ns: u64) -> GimbalCmd {
        GimbalCmd {
            timestamp_ns: ts_ns,
            yaw_deg: 10.0,
            pitch_deg: -2.0,
            distance_m: 3.5,
            fire_advice: 0,
            _pad: [0; 3],
            command_seq: 1,
        }
    }

    fn safe_stop(ts_ns: u64) -> GimbalCmd {
        GimbalCmd {
            distance_m: SAFE_STOP_DISTANCE,
            ..control(ts_ns)
        }
    }

    /// 订阅上、还没收过任何命令的链路。
    fn subscribed() -> AutoAimLink {
        let mut link = AutoAimLink::default();
        link.tick(true, 0.0, &cfg());
        link
    }

    #[test]
    fn unsubscribed_link_leaves_gimbal_to_the_operator() {
        let link = AutoAimLink::default();
        assert_eq!(link.state(), LinkState::Unsubscribed);
        assert!(link.state().allows_manual_gimbal());
        assert!(!link.state().allows_fire());
    }

    #[test]
    fn subscribed_but_nothing_received_stays_manual() {
        let link = subscribed();
        assert_eq!(link.state(), LinkState::Waiting);
        assert_eq!(link.state().label(), "未收到");
        // 关键：外部从没接管过，就不能把云台冻在这里。忘了起视觉侧的时候，
        // 云台不动 + 方向键也不动，看起来就是"自瞄坏了"。
        assert!(link.state().allows_manual_gimbal());
        assert!(!link.state().allows_fire());
    }

    #[test]
    fn control_command_puts_the_link_online_and_takes_the_gimbal() {
        let mut link = subscribed();
        let v = link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        assert_eq!(v, LinkVerdict::Control { fire: false });
        assert_eq!(link.state(), LinkState::Engaged);
        assert_eq!(link.state().label(), "接管");
        assert!(link.state().external_owns_gimbal());
        assert!(!link.state().allows_manual_gimbal());
        assert!(link.state().allows_fire());
        assert_eq!((link.accepted, link.control, link.safe_stops), (1, 1, 0));
    }

    #[test]
    fn safe_stop_only_link_is_alive_but_does_not_hold_the_gimbal() {
        // passive 模式就是这样：20ms 一发安全停止，永不下发控制。
        let mut link = subscribed();
        let mut t = 0.0f32;
        let mut ts = T0_NS;
        for _ in 0..10 {
            assert_eq!(
                link.ingest(&safe_stop(ts), t, ts, &cfg()),
                LinkVerdict::SafeStop
            );
            t += 0.02;
            ts += 20_000_000;
        }
        assert_eq!(link.state(), LinkState::Standby);
        assert_eq!(link.state().label(), "待命");
        assert!(
            link.state().allows_manual_gimbal(),
            "安全停止流不该占着云台"
        );
        assert!(!link.state().allows_fire());
        assert_eq!(link.safe_stops, 10);
        assert_eq!(link.control, 0);
    }

    #[test]
    fn losing_the_peer_while_engaged_enters_external_link_lost() {
        let mut link = subscribed();
        link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        assert_eq!(link.state(), LinkState::Engaged);

        // 租约内没有新命令：一帧一帧推进，直到跨过 500ms。
        link.tick(true, 0.4, &cfg());
        assert_eq!(link.state(), LinkState::Engaged, "租约内不该判断线");
        link.tick(true, 0.6, &cfg());

        assert_eq!(link.state(), LinkState::Lost);
        assert_eq!(link.state().label(), "中断");
        assert_eq!(link.link_lost_count, 1);
        // 独立安全状态：云台仍归外部（冻在最后一条命令上），但禁止虚拟开火。
        assert!(link.state().external_owns_gimbal());
        assert!(!link.state().allows_manual_gimbal(), "不隐式交还手动");
        assert!(!link.state().allows_fire(), "超过租约禁止虚拟开火");
    }

    #[test]
    fn safe_stops_keep_the_peer_alive_and_demote_to_standby_not_lost() {
        // 对端还活着、只是不再控制（closed_loop 丢目标转安全停止）：
        // 这不是链路中断，应该把控制权交回手动而不是冻住。
        let mut link = subscribed();
        link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        link.ingest(
            &safe_stop(T0_NS + 300_000_000),
            0.3,
            T0_NS + 300_000_000,
            &cfg(),
        );
        link.tick(true, 0.6, &cfg());

        assert_eq!(link.state(), LinkState::Standby);
        assert_eq!(link.link_lost_count, 0);
        assert!(link.state().allows_manual_gimbal());
        assert!(!link.state().allows_fire());

        // 再往后连安全停止也停了：待命时控制权本来就在手动，退回"未收到"。
        link.tick(true, 1.0, &cfg());
        assert_eq!(link.state(), LinkState::Waiting);
        assert_eq!(link.link_lost_count, 0);
    }

    #[test]
    fn safe_stop_after_link_loss_does_not_unlock_manual_control() {
        // Lost 是显式解锁前的安全锁存；恢复后到达的安全停止不能隐式转回 Standby。
        let mut link = subscribed();
        link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        link.tick(true, 0.6, &cfg());
        assert_eq!(link.state(), LinkState::Lost);

        let stop = safe_stop(T0_NS + 700_000_000);
        assert_eq!(
            link.ingest(&stop, 0.7, T0_NS + 700_000_000, &cfg()),
            LinkVerdict::SafeStop
        );
        assert_eq!(link.state(), LinkState::Lost);
        assert!(!link.state().allows_manual_gimbal());

        // 只有显式关闭订阅才解锁手动控制。
        link.tick(false, 0.8, &cfg());
        assert_eq!(link.state(), LinkState::Unsubscribed);
        assert!(link.state().allows_manual_gimbal());
    }

    #[test]
    fn a_restarted_peer_recovers_from_lost_even_with_a_lower_timestamp() {
        let mut link = subscribed();
        link.ingest(
            &control(T0_NS + 900_000_000),
            0.0,
            T0_NS + 900_000_000,
            &cfg(),
        );
        link.tick(true, 0.6, &cfg());
        assert_eq!(link.state(), LinkState::Lost);

        // 重启后的第一条命令时间戳比断线前的水位线低（wall clock 回拨，或者干脆是
        // 另一个进程）。如果水位线不清，这里会被永久判成"时间戳倒退"，表现是
        // "重启视觉侧之后自瞄再也不接管"。
        let ts = T0_NS + 700_000_000;
        let v = link.ingest(&control(ts), 5.0, ts, &cfg());
        assert_eq!(v, LinkVerdict::Control { fire: false });
        assert_eq!(link.state(), LinkState::Engaged);
        assert!(link.state().allows_fire());
        assert_eq!(link.rejects.timestamp_regressed, 0);
        assert_eq!(link.link_lost_count, 1, "恢复不该重复计数");
    }

    #[test]
    fn toggling_the_subscription_off_returns_the_gimbal_to_the_operator() {
        let mut link = subscribed();
        link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        link.tick(true, 0.6, &cfg());
        assert_eq!(link.state(), LinkState::Lost);

        // F5 关订阅是拿回手动控制的显式动作。
        link.tick(false, 0.7, &cfg());
        assert_eq!(link.state(), LinkState::Unsubscribed);
        assert!(link.state().allows_manual_gimbal());

        link.tick(true, 0.8, &cfg());
        assert_eq!(link.state(), LinkState::Waiting);
    }

    #[test]
    fn a_regressed_timestamp_is_rejected_and_does_not_renew_the_lease() {
        let mut link = subscribed();
        let first = T0_NS + 400_000_000;
        link.ingest(&control(first), 0.0, first, &cfg());

        // 重放/乱序：时间戳不前进的命令一律拒收，相等也算。
        for (ts, now) in [(first - 100_000_000, 0.1), (first, 0.2)] {
            let v = link.ingest(&control(ts), now, first, &cfg());
            assert_eq!(v, LinkVerdict::Reject(RejectReason::TimestampRegressed));
        }
        assert_eq!(link.rejects.timestamp_regressed, 2);
        assert_eq!(link.rejects.total(), 2);
        assert_eq!(link.accepted, 1, "被拒的命令不算已接受");
        assert_eq!(link.last_timestamp_ns, first, "水位线不被倒退命令改写");
        assert_eq!(link.last_reject, Some(RejectReason::TimestampRegressed));

        // 关键：拒收不续租约。只发倒退命令的对端会照常走到中断。
        link.tick(true, 0.6, &cfg());
        assert_eq!(link.state(), LinkState::Lost);
    }

    #[test]
    fn zero_timestamp_is_rejected() {
        // 全 0 的槽位（发布端还没写过）看上去是一条 yaw=pitch=0 的命令。
        let mut link = subscribed();
        let v = link.ingest(&GimbalCmd::default(), 0.0, T0_NS, &cfg());
        assert_eq!(v, LinkVerdict::Reject(RejectReason::TimestampZero));
        assert_eq!(link.state(), LinkState::Waiting);
        assert_eq!(link.rejects.timestamp_zero, 1);
    }

    #[test]
    fn non_finite_commands_are_rejected_on_every_field() {
        // NaN 进 Transform.rotation 会把四元数永久污染，之后相机姿态、出膛点、
        // 真值全跟着 NaN，关订阅也回不来。所以在写进世界之前就要挡掉。
        let cases: [(&str, GimbalCmd); 6] = [
            (
                "yaw NaN",
                GimbalCmd {
                    yaw_deg: f32::NAN,
                    ..control(T0_NS + 1)
                },
            ),
            (
                "yaw inf",
                GimbalCmd {
                    yaw_deg: f32::INFINITY,
                    ..control(T0_NS + 2)
                },
            ),
            (
                "pitch NaN",
                GimbalCmd {
                    pitch_deg: f32::NAN,
                    ..control(T0_NS + 3)
                },
            ),
            (
                "pitch -inf",
                GimbalCmd {
                    pitch_deg: f32::NEG_INFINITY,
                    ..control(T0_NS + 4)
                },
            ),
            (
                "distance NaN",
                GimbalCmd {
                    distance_m: f32::NAN,
                    ..control(T0_NS + 5)
                },
            ),
            (
                "distance inf",
                GimbalCmd {
                    distance_m: f32::INFINITY,
                    ..control(T0_NS + 6)
                },
            ),
        ];
        let mut link = subscribed();
        for (name, cmd) in cases {
            let v = link.ingest(&cmd, 0.0, T0_NS + 10, &cfg());
            assert_eq!(v, LinkVerdict::Reject(RejectReason::NonFinite), "{name}");
        }
        assert_eq!(link.rejects.non_finite, 6);
        assert_eq!(link.accepted, 0);
        // 一条都没接受过，控制权必须留在手动。
        assert_eq!(link.state(), LinkState::Waiting);
        assert!(link.state().allows_manual_gimbal());
        assert!(!link.state().allows_fire());
    }

    #[test]
    fn nan_cannot_take_over_an_engaged_link_either() {
        let mut link = subscribed();
        link.ingest(&control(T0_NS), 0.0, T0_NS, &cfg());
        let bad = GimbalCmd {
            pitch_deg: f32::NAN,
            ..control(T0_NS + 1)
        };
        assert_eq!(
            link.ingest(&bad, 0.01, T0_NS + 1, &cfg()),
            LinkVerdict::Reject(RejectReason::NonFinite)
        );
        // 姿态还是上一条合法命令的，而且这条不续租约。
        assert_eq!(link.last_timestamp_ns, T0_NS);
        link.tick(true, 0.6, &cfg());
        assert_eq!(link.state(), LinkState::Lost);
    }

    #[test]
    fn a_stale_command_is_rejected_and_cannot_fire() {
        let mut link = subscribed();
        let ts = T0_NS;
        let mut cmd = control(ts);
        cmd.fire_advice = 1;
        // 命令自身在链路上放了 700ms > 500ms 租约。
        let now_ns = ts + 700_000_000;
        let v = link.ingest(&cmd, 0.7, now_ns, &cfg());
        assert_eq!(v, LinkVerdict::Reject(RejectReason::Expired));
        assert!(
            !matches!(v, LinkVerdict::Control { .. }),
            "过期命令不得开火"
        );
        assert_eq!(link.rejects.expired, 1);
        assert_eq!(link.state(), LinkState::Waiting);
    }

    #[test]
    fn a_command_from_the_future_is_rejected() {
        // 对端时钟比本机超前一个租约以上：不能当"很新"接受，否则 age 判据就废了。
        let mut link = subscribed();
        let ts = T0_NS + 2_000_000_000;
        let v = link.ingest(&control(ts), 0.0, T0_NS, &cfg());
        assert_eq!(v, LinkVerdict::Reject(RejectReason::TimestampFuture));
        assert_eq!(link.rejects.timestamp_future, 1);
        assert_eq!(link.state(), LinkState::Waiting);
    }

    #[test]
    fn fire_advice_is_only_honoured_on_control_commands() {
        let mut link = subscribed();
        let mut cmd = control(T0_NS);
        cmd.fire_advice = 1;
        assert_eq!(
            link.ingest(&cmd, 0.0, T0_NS, &cfg()),
            LinkVerdict::Control { fire: true }
        );
        assert!(link.state().allows_fire());

        // 安全停止即使带着 fire_advice 也只是安全停止。
        let mut stop = safe_stop(T0_NS + 1);
        stop.fire_advice = 1;
        assert_eq!(
            link.ingest(&stop, 0.01, T0_NS + 1, &cfg()),
            LinkVerdict::SafeStop
        );
    }
}
