use crate::robomaster::outpost::consts::ROTATION_SPEED;
use bevy::prelude::Transform;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum RotationDirection {
    Clockwise,
    CounterClockwise,
}

impl RotationDirection {
    pub const fn sign(self) -> f32 {
        match self {
            Self::Clockwise => 1.0,
            Self::CounterClockwise => -1.0,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, Default)]
pub enum RotationMode {
    #[default]
    Forward,
    Stopped,
    Reverse,
}

impl RotationMode {
    pub const fn scale(self) -> f32 {
        match self {
            Self::Forward => 1.0,
            Self::Stopped => 0.0,
            Self::Reverse => -1.0,
        }
    }

    pub const fn next(self) -> Self {
        match self {
            Self::Forward => Self::Stopped,
            Self::Stopped => Self::Reverse,
            Self::Reverse => Self::Forward,
        }
    }
}

pub struct RotationController {
    speed: f32,
    direction: RotationDirection,
}

impl RotationController {
    pub fn new(direction: RotationDirection) -> Self {
        Self {
            speed: ROTATION_SPEED,
            direction,
        }
    }

    fn rotate(&self, transform: &mut Transform, angle: f32) {
        transform.rotate_y(angle);
    }

    /// 当前有符号角速度，rad/s，绕 **Bevy 局部 +Y**。
    ///
    /// 提出来是为了让真值发布的 vyaw 与场景实际转动同源：[`Self::step`] 现在也走
    /// 这个表达式，所以改了转速常量或旋向之后，画面转速和真值 vyaw 一起变，不会
    /// 出现"真值说在转、画面没转"这种只能靠肉眼发现的分叉。
    pub fn signed_speed(&self, mode: RotationMode) -> f32 {
        self.direction.sign() * mode.scale() * self.speed
    }

    pub fn step(&self, transform: &mut Transform, dt: f32, mode: RotationMode) {
        self.rotate(transform, self.signed_speed(mode) * dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_direction_sign_matches_legacy_bool() {
        assert_eq!(RotationDirection::Clockwise.sign(), 1.0);
        assert_eq!(RotationDirection::CounterClockwise.sign(), -1.0);
    }

    /// `signed_speed` 必须与 `step` 实际转的角一致——真值 vyaw 用的是前者，
    /// 画面转的是后者，两者一旦分叉，评估端会拿一个不存在的角速度去算预测误差。
    #[test]
    fn signed_speed_matches_what_step_actually_rotates() {
        for (direction, mode) in [
            (RotationDirection::Clockwise, RotationMode::Forward),
            (RotationDirection::Clockwise, RotationMode::Reverse),
            (RotationDirection::CounterClockwise, RotationMode::Forward),
            (RotationDirection::CounterClockwise, RotationMode::Stopped),
        ] {
            let c = RotationController::new(direction);
            let dt = 0.01;
            let mut t = Transform::IDENTITY;
            c.step(&mut t, dt, mode);
            // rotate_y(θ) 绕 +Y 转 θ：从四元数取回带符号的角。
            let (axis, angle) = t.rotation.to_axis_angle();
            let signed = angle * axis.y.signum();
            let expected = c.signed_speed(mode) * dt;
            assert!(
                (signed - expected).abs() < 1e-6,
                "{direction:?}/{mode:?}: step 转了 {signed}，signed_speed 说 {expected}"
            );
        }
    }

    #[test]
    fn rotation_mode_cycles_in_debug_order() {
        assert_eq!(RotationMode::Forward.next(), RotationMode::Stopped);
        assert_eq!(RotationMode::Stopped.next(), RotationMode::Reverse);
        assert_eq!(RotationMode::Reverse.next(), RotationMode::Forward);
    }
}
