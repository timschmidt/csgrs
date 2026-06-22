//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::css::*;
use bevy::pbr::wireframe::{Wireframe, WireframeColor, WireframePlugin};
use bevy::prelude::*;
use bevy::render::mesh::PrimitiveTopology;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hypermesh::legacy::{ExactBooleanOperation, ExactBooleanRequest, ExactBooleanWorkspace};
use hypermesh::{ExactMesh, ValidationPolicy};

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
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let left = exact_box([-3, -2, -2], [1, 2, 2]);
    let right = exact_box([0, -1, -1], [3, 1, 1]);
    let request =
        ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED);
    let mut workspace = ExactBooleanWorkspace::new(&left, &right);
    let difference = workspace
        .materialize_ref(request)
        .expect("overlapping boxes should materialize exact boolean output")
        .mesh()
        .clone();

    let meshes_to_draw = [left, right, difference];
    let colors = [BLUE, GREEN, WHITE];

    for (i, mesh) in meshes_to_draw.iter().enumerate() {
        cmds.spawn((
            Mesh3d(meshes.add(bevy_mesh_from_exact(mesh))),
            MeshMaterial3d(mats.add(StandardMaterial {
                base_color: colors[i].into(),
                ..default()
            })),
            Transform::default(),
            Wireframe,
            WireframeColor {
                color: BLACK.into(),
            },
            ToggleableMesh,
            if i == 2 {
                Visibility::Visible
            } else {
                Visibility::Hidden
            },
        ));
    }
    cmds.spawn((PointLight::default(), Transform::from_xyz(2., 5., 2.)));
    cmds.spawn((
        Transform::from_translation(Vec3::new(0., 3., 6.)),
        PanOrbitCamera::default(),
    ));
}

fn toggle_mesh_visibility(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut Visibility, With<ToggleableMesh>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        for mut visibility in query.iter_mut() {
            *visibility = match *visibility {
                Visibility::Visible => Visibility::Hidden,
                Visibility::Hidden | Visibility::Inherited => Visibility::Visible,
            };
        }
    }
}

fn exact_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned integer box should be closed")
}

fn bevy_mesh_from_exact(mesh: &ExactMesh) -> Mesh {
    let package = mesh
        .handoff_package()
        .expect("exact mesh should have a fresh handoff");
    let view = package
        .approximate_f64_view
        .as_ref()
        .expect("exact mesh handoff should include a fresh f64 view");
    let mut positions = Vec::with_capacity(view.indices.len());
    let mut normals = Vec::with_capacity(view.indices.len());

    for triangle in view.indices.chunks_exact(3) {
        let p0 = position_at(&view.positions, triangle[0]);
        let p1 = position_at(&view.positions, triangle[1]);
        let p2 = position_at(&view.positions, triangle[2]);
        let normal = (Vec3::from_array(p1) - Vec3::from_array(p0))
            .cross(Vec3::from_array(p2) - Vec3::from_array(p0))
            .normalize_or_zero()
            .to_array();
        positions.extend_from_slice(&[p0, p1, p2]);
        normals.extend_from_slice(&[normal, normal, normal]);
    }

    let mut out = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    out.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    out.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    out
}

fn position_at(positions: &[f64], index: usize) -> [f32; 3] {
    let base = index * 3;
    [
        positions[base] as f32,
        positions[base + 1] as f32,
        positions[base + 2] as f32,
    ]
}
