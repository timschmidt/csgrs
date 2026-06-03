//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::css::*;
use bevy::pbr::wireframe::{Wireframe, WireframeColor, WireframePlugin};
use bevy::prelude::*;
use bevy::render::mesh::PrimitiveTopology;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hypermesh::{ExactMesh, ValidationPolicy, approximate_mesh_f64_view};

use std::time::Instant;

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
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let now = Instant::now();

    let level = 3;
    let sponge = menger_sponge(level);

    println!(
        "computed exact integer-grid Menger sponge level {}, elapsed time: {:?}",
        level,
        now.elapsed()
    );

    cmds.spawn((DirectionalLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((
        Transform::from_translation(Vec3::new(0., 0., 2.)),
        PanOrbitCamera::default(),
    ));

    cmds.spawn((
        Mesh3d(meshes.add(bevy_mesh_from_exact(&sponge))),
        MeshMaterial3d(mats.add(StandardMaterial {
            base_color: GRAY.into(),
            ..default()
        })),
        Transform::from_scale(Vec3::splat(1.0 / 27.0)).with_translation(Vec3::splat(-0.5)),
        Wireframe,
        WireframeColor {
            color: Srgba::rgb(0.3, 0.3, 0.3).into(),
        },
        ToggleableMesh,
    ));
}

fn menger_sponge(level: usize) -> ExactMesh {
    let side = 3_i64.pow(level as u32);
    let mut coordinates = Vec::new();
    let mut indices = Vec::new();
    append_menger_cells(level, [0, 0, 0], side, &mut coordinates, &mut indices);
    ExactMesh::from_i64_triangles_with_policy(&coordinates, &indices, ValidationPolicy::CLOSED)
        .expect("generated Menger sponge should be a closed exact mesh")
}

fn append_menger_cells(
    level: usize,
    origin: [i64; 3],
    side: i64,
    coordinates: &mut Vec<i64>,
    indices: &mut Vec<usize>,
) {
    if level == 0 {
        append_box(
            origin,
            [origin[0] + side, origin[1] + side, origin[2] + side],
            coordinates,
            indices,
        );
        return;
    }

    let step = side / 3;
    for x in 0..3 {
        for y in 0..3 {
            for z in 0..3 {
                let middle_axes = usize::from(x == 1) + usize::from(y == 1) + usize::from(z == 1);
                if middle_axes >= 2 {
                    continue;
                }
                append_menger_cells(
                    level - 1,
                    [
                        origin[0] + i64::from(x) * step,
                        origin[1] + i64::from(y) * step,
                        origin[2] + i64::from(z) * step,
                    ],
                    step,
                    coordinates,
                    indices,
                );
            }
        }
    }
}

fn append_box(min: [i64; 3], max: [i64; 3], coordinates: &mut Vec<i64>, indices: &mut Vec<usize>) {
    let base = coordinates.len() / 3;
    coordinates.extend_from_slice(&[
        min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
        min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
        max[1], max[2],
    ]);
    indices.extend(
        [
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ]
        .into_iter()
        .map(|index| base + index),
    );
}

fn bevy_mesh_from_exact(mesh: &ExactMesh) -> Mesh {
    let view = approximate_mesh_f64_view(mesh).expect("exact mesh should have a fresh f64 view");
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
