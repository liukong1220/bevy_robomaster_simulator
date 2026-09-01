use crate::robomaster::outpost::construct::OutpostConstructorPlugin;
use bevy::app::plugin_group;

pub use crate::robomaster::outpost::construct::*;
pub use crate::robomaster::outpost::rotation::{RotationDirection, RotationMode};
use crate::robomaster::outpost::update::OutpostUpdatePlugin;
pub use crate::robomaster::outpost::update::{Outpost, OutpostRotationMode, OutpostRotator};

plugin_group! {
    #[derive(Default)]
    pub struct OutpostPlugins {
        :OutpostConstructorPlugin,
        :OutpostUpdatePlugin,
    }
}
