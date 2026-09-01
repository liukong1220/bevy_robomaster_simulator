use bevy::prelude::*;

#[derive(Resource, Default, Reflect)]
#[reflect(Resource)]
pub struct ProjectileStatistics {
    pub launch_count: u32,
    pub accurate_count: u32,
}

impl ProjectileStatistics {
    pub fn increase_launch(&mut self) {
        self.launch_count += 1;
    }

    pub fn increase_accurate(&mut self) {
        self.accurate_count += 1;
    }

    /// 命中率，单位是**百分数**（0~100），名字里的 pct 就是这个意思。
    ///
    /// 之前这里返回的是比值，HUD 却按 `{:.2}%` 打印，于是 335/438 显示成
    /// "命中率=0.76%"——差了 100 倍，看上去像是几乎全打空。
    pub fn accurate_pct(&self) -> f32 {
        if self.launch_count == 0 {
            return 0.0;
        }
        100.0 * (self.accurate_count as f32) / (self.launch_count as f32)
    }
}
