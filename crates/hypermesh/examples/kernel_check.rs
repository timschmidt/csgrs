use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::basic::{BLACK, WHITE};
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::{color::palettes::css::*, math::Isometry2d};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use nalgebra::{DMatrix, RowVector3};
use mfd::Hmesh;
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::Arc;

#[derive(Resource)]
struct HmeshHandle(Vec<Arc<Hmesh>>);

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, setup)
        //.add_systems(Update, draw_example_collection.run_if(resource_exists::<HmeshHandle>))
        .run();
}


fn setup(
    mut cmds: Commands,
    mut gizmos: Gizmos,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    let tet_a = mfd::boolean::test_data::gen_tet_a();
    let tet_c = mfd::boolean::test_data::gen_tet_c();
    let mut hmesh_handles = Vec::new();

    for hm in vec![tet_a, tet_c] {
        let mut bevy_mesh = Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default()
        );

        let mut idcs = vec![];
        for i in 0..hm.idx.nrows() {
            idcs.push(hm.idx[(i, 0)] as u32);
            idcs.push(hm.idx[(i, 1)] as u32);
            idcs.push(hm.idx[(i, 2)] as u32);
        }

        bevy_mesh.insert_indices(Indices::U32(idcs));
        bevy_mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            (0..hm.pos.nrows()).map(|i| [
                hm.pos[(i, 0)] as f32,
                hm.pos[(i, 1)] as f32,
                hm.pos[(i, 2)] as f32,
            ]).collect::<Vec<_>>()
        );


        cmds.spawn((
            Mesh3d(meshes.add(bevy_mesh).clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(1.0, 0.0, 0.0, 0.5),
                alpha_mode: AlphaMode::Blend,
                ..default() })),
            Transform::default(),
            Wireframe,
            WireframeColor { color: WHITE.into() },
        ));

        hmesh_handles.push(hm);
    }

    cmds.insert_resource(HmeshHandle(hmesh_handles));
    let sin_t_scaled = ops::sin(0.) * 50.;
    gizmos.line_2d(Vec2::Y * -sin_t_scaled, Vec2::splat(-80.), WHITE);
    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));

}

fn draw_example_collection(
    hmesh_handle: Res<HmeshHandle>,
    mut gizmos: Gizmos,
) {
    for hmesh in &hmesh_handle.0 {
        for i in 0..hmesh.n_vert {
            let p0 = hmesh.verts[i].pos();
            let p1 = p0 + hmesh.vert_normal.row(i) * 0.1;
            gizmos.line(
                Vec3::new(p0.x as f32, p0.y as f32, p0.z as f32),
                Vec3::new(p1.x as f32, p1.y as f32, p1.z as f32),
                RED
            );
        }

        for i in 0..hmesh.n_face {
            let p0 = hmesh.bary_center.row(i);
            let p1 = p0 + hmesh.face_normal.row(i) * 0.1;
            gizmos.line(
                Vec3::new(p0[0] as f32, p0[1] as f32, p0[2] as f32),
                Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32),
                BLUE
            );
        }
    }
}
