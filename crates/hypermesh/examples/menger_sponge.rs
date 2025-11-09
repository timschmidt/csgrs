mod utils;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use crate::utils::{menger_sponge};

#[derive(Component)]
struct ToggleableMesh;

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut cmds: Commands,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    let res = menger_sponge(4);
    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));
    let mut m = Mesh::new(bevy::render::mesh::PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    m.insert_indices(Indices::U32(res.hs.chunks(3).flat_map(|t| [t[0].tail as u32, t[1].tail as u32, t[2].tail as u32]).collect()));
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, res.ps.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect::<Vec<_>>());
    cmds.spawn((
        Mesh3d(meshes.add(m).clone()),
        MeshMaterial3d(mats.add(StandardMaterial { ..default()})),
        Transform::default(),
        Wireframe,
        WireframeColor { color: GRAY.into() },
        ToggleableMesh,
    ));
}
