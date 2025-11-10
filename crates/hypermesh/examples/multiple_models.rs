mod utils;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use boolean::{compute_boolean, Manifold};
use boolean::common::{OpType};

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
        .add_systems(Update, toggle_mesh_visibility)
        .run();
}

fn setup(
    mut cmds: Commands,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    let (m0, _) = tobj::load_obj("assets/models/fertility.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    let (m1, _) = tobj::load_obj("assets/models/rolling_stage.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");

    let mut mfs = vec![];
    for (m, s) in vec![(&m0[0].mesh, 0.014), (&m1[0].mesh, 1.)] {
        mfs.push(Manifold::new(
            &m.positions.iter().map(|&v| (v * s) as f64).collect::<Vec<f64>>(),
            &m.indices.iter().map(|&v| v as usize).collect::<Vec<usize>>(),
        ).unwrap());
    }
    let res = compute_boolean(&mfs[0], &mfs[1], OpType::Subtract).unwrap();
    mfs.push(res);

    for (i, mf) in mfs.iter().enumerate() {
        let mut bm = Mesh::new(bevy::render::mesh::PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        bm.insert_indices(Indices::U32(mf.hs.iter().map(|h| h.tail as u32).collect()));
        bm.insert_attribute(Mesh::ATTRIBUTE_POSITION, mf.ps.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]).collect::<Vec<_>>());
        cmds.spawn((
            Mesh3d(meshes.add(bm).clone()),
            MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
            Transform::default(),
            Wireframe,
            WireframeColor { color: GRAY.into() },
            ToggleableMesh,
            if i == 2 { Visibility::Visible } else { Visibility::Hidden },
        ));
    }
    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 2., 3.)), PanOrbitCamera::default(),));
}

fn toggle_mesh_visibility(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Visibility, With<ToggleableMesh>>,
) {
    let cb = |visibility: &mut Visibility| {
        *visibility = match visibility {
            Visibility::Visible => Visibility::Hidden,
            Visibility::Hidden => Visibility::Visible,
            Visibility::Inherited => Visibility::Hidden,
        };
    };
    let mut vis: Vec<_> = query.iter_mut().collect();
    if keyboard.just_pressed(KeyCode::Space)  { cb(&mut vis[0]); cb(&mut vis[1]); }
}

