use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::basic::{BLACK, WHITE};
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::{color::palettes::css::*, math::Isometry2d};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use nalgebra::DMatrix;
use mfd::Hmesh;
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::Arc;

#[derive(Resource)]
struct HmeshHandle(Arc<Hmesh>);

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, setup)
        .add_systems(Update, draw_example_collection.run_if(resource_exists::<HmeshHandle>))
        .run();
}

fn setup(
    mut cmds: Commands,
    mut gizmos: Gizmos,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    let (models, _) = tobj::load_obj("assets/models/icosphere_3.obj", &tobj::LoadOptions { ..Default::default() },
    ).expect("Failed to OBJ load file");
    let model = &models[0];
    let mesh = &model.mesh;
    let pos_buf: Vec<f64>   = mesh.positions.iter().map(|&v| v as f64).collect();
    let idx_buf: Vec<usize> = mesh.indices.iter().map(|&v| v as usize).collect();
    let pos: DMatrix<f64>   = DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into();
    let idx: DMatrix<usize> = DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into();
    let hmesh = Hmesh::new(pos, idx);

    let mut bevy_mesh = Mesh::new(
        bevy::render::mesh::PrimitiveTopology::TriangleList,
        RenderAssetUsages::default()
    );

    bevy_mesh.insert_indices(
        Indices::U32(mesh.indices.iter().map(|&i| i).collect())
    );
    bevy_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        (0..mesh.positions.len() / 3).map(|i| [
            mesh.positions[i * 3],
            mesh.positions[i * 3 + 1],
            mesh.positions[i * 3 + 2],
        ]).collect::<Vec<_>>()
    );


    cmds.spawn((
        Mesh3d(meshes.add(bevy_mesh).clone()),
        MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
        Transform::default(),
        Wireframe,
        WireframeColor { color: WHITE.into() },
    ));

    let sin_t_scaled = ops::sin(0.) * 50.;
    gizmos.line_2d(Vec2::Y * -sin_t_scaled, Vec2::splat(-80.), WHITE);

    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));
    cmds.insert_resource(HmeshHandle(hmesh));

}

fn draw_example_collection(
    hmesh_handle: Res<HmeshHandle>,
    mut gizmos: Gizmos,
) {
    for i in 0..hmesh_handle.0.n_vert {
        let p0 = hmesh_handle.0.verts[i].pos();
        let p1 = p0 + hmesh_handle.0.vert_normal.row(i) * 0.05;
        gizmos.line(
            Vec3::new(p0.x as f32, p0.y as f32, p0.z as f32),
            Vec3::new(p1.x as f32, p1.y as f32, p1.z as f32),
            RED
        );
    }
}
