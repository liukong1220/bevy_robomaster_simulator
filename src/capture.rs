pub mod depth;
pub mod driver;
pub mod view_copy;

use bevy::camera::{Exposure, RenderTarget};
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;

use crate::config::SimulationConfig;
use crate::metalfx::MetalFxTemporalUpscaling;

pub use driver::CaptureBundle;

#[derive(Component)]
pub struct CaptureSource;

#[derive(Component)]
pub struct CaptureCamera;

#[derive(Resource, Deref, Clone)]
pub struct ImageHandle(pub Handle<Image>);

#[derive(Resource, Clone, Copy)]
pub struct CameraFov(pub f32);

pub const CAPTURE_CAMERA_ORDER: isize = -100;

pub fn setup_capture_camera(world: &mut World) {
    let capture_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<CaptureCamera>>();
        query.iter(world).next().is_some()
    };
    if capture_camera_exists {
        return;
    }

    let render_target_handle = world.resource::<ImageHandle>().0.clone();
    let fov = world.resource::<CameraFov>().0;
    // 曝光必须显式给：Bevy 默认 Exposure::BLENDER(EV100 9.7) 是按白天户外测的光，
    // 配上赛场量级的 illuminance(几百 lux)，所有非自发光表面都会挤在个位数灰度上，
    // 检测器只能看到装甲灯条(灯条 emissive_exposure_weight=-1，不受曝光影响)。
    // 见 RenderConfig::capture_ev100。
    let exposure = Exposure {
        ev100: capture_ev100(world),
    };
    let metalfx = world
        .get_resource::<SimulationConfig>()
        .filter(|config| cfg!(target_os = "macos") && config.render.metalfx_temporal)
        .map(|config| MetalFxTemporalUpscaling {
            scale_factor: config.render.metalfx_scale,
            frame_generation: config.render.metalfx_frame_generation,
            reset: true,
        });

    let mut capture_camera = world.spawn((
        Camera3d::default(),
        Tonemapping::None,
        exposure,
        RenderTarget::Image(render_target_handle.into()),
        Camera {
            order: CAPTURE_CAMERA_ORDER,
            // clear_color: ClearColorConfig::Custom(Color::BLACK),
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov,
            near: 0.1,
            far: 10000.0,
            ..default()
        }),
        Msaa::Off,
        DepthPrepass,
        CaptureCamera,
    ));
    if let Some(metalfx) = metalfx {
        capture_camera.insert(metalfx);
    }
}

#[derive(Component)]
pub struct PreviewCamera;

#[derive(Component)]
pub struct PreviewImageNode;

pub fn setup_preview_window(world: &mut World) {
    let preview_enabled = world
        .resource::<crate::config::SimulationConfig>()
        .preview
        .enabled;
    if !preview_enabled {
        return;
    }

    let render_target_handle = world.resource::<ImageHandle>().0.clone();
    let preview_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<PreviewCamera>>();
        query.iter(world).next().is_some()
    };
    if !preview_camera_exists {
        world.spawn((
            Camera2d::default(),
            Camera {
                order: 1,
                ..default()
            },
            PreviewCamera,
        ));
    }

    let preview_node_exists = {
        let mut query = world.query_filtered::<Entity, With<PreviewImageNode>>();
        query.iter(world).next().is_some()
    };
    if !preview_node_exists {
        world.spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            // Render as a background; help text UI remains on top.
            GlobalZIndex(-1),
            ImageNode::new(render_target_handle),
            PreviewImageNode,
        ));
    }
}

/// 采集相机的 EV100：优先取配置，否则按入射光公式从 illuminance 推。
///
/// `ev100 = log2(lux / 2.5)` 能复现 Bevy 自己的档位（100k lux -> 15 = SUNLIGHT，
/// 400 lux -> 7 = INDOOR），所以这不是拍脑袋的系数，而是和引擎同一套约定。
fn capture_ev100(world: &World) -> f32 {
    // 标定/排查用的临时覆盖，优先级最高，不需要改配置文件。
    if let Ok(raw) = std::env::var("DAEDALUS_CAPTURE_EV100") {
        match raw.trim().parse::<f32>() {
            Ok(ev100) if ev100.is_finite() => return ev100,
            _ => warn!("DAEDALUS_CAPTURE_EV100 不是有效数字，已忽略: {raw:?}"),
        }
    }
    let Some(config) = world.get_resource::<SimulationConfig>() else {
        return Exposure::default().ev100;
    };
    if let Some(ev100) = config.render.capture_ev100 {
        return ev100;
    }
    let lux = config.render.illuminance;
    if lux > 0.0 {
        (lux / 2.5).log2()
    } else {
        Exposure::default().ev100
    }
}

pub fn copy_transform(target: &Transform, our: &mut Transform) {
    our.translation = target.translation;
    our.scale = target.scale;
    our.rotation = target.rotation;
}

pub fn sync_capture_camera(
    target: Single<&Transform, (With<CaptureSource>, Without<CaptureCamera>)>,
    mut our: Single<&mut Transform, (With<CaptureCamera>, Without<CaptureSource>)>,
) {
    copy_transform(&target, &mut our);
}

#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: u32,
    pub height: u32,
}

pub fn compute_camera_intrinsics(width: u32, height: u32, fov_y: f32) -> CameraIntrinsics {
    let fov_y = fov_y as f64;
    let aspect = width as f64 / height as f64;
    let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();

    let fx = width as f64 / (2.0 * (fov_x / 2.0).tan());
    let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());

    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;

    CameraIntrinsics {
        fx,
        fy,
        cx,
        cy,
        width,
        height,
    }
}

pub const IMAGE_WIDTH: u32 = 1440;
pub const IMAGE_HEIGHT: u32 = 1080;
