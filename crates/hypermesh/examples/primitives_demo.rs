//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::asset::RenderAssetUsages;
use bevy::render::mesh::PrimitiveTopology;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use boolmesh::prelude::*;

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

    let mut mfd0 = generate_cylinder(1., 1., 30, 10).unwrap();
    let mut mfd1 = generate_uv_sphere(30, 30).unwrap();
    let mut mfd2 = generate_torus(1., 0.1, 30, 30).unwrap();
    mfd1.translate(0., 0.5, 0.);
    let res = compute_boolean(&mfd0, &mfd1, OpType::Add).unwrap();
    let res = compute_boolean(&res,  &mfd2, OpType::Subtract).unwrap();

    cmds.spawn((DirectionalLight::default(), Transform::from_xyz(30., 40., 30.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 0., 2.)), PanOrbitCamera::default(),));

    let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    let mut pos = vec![];
    let mut vns = vec![];
    for (fid, hs) in res.hs.chunks(3).enumerate() {
        let p0 = res.ps[hs[0].tail];
        let p1 = res.ps[hs[1].tail];
        let p2 = res.ps[hs[2].tail];
        let n  = res.face_normals[fid];
        pos.push([p0.x as f32, p0.y as f32, p0.z as f32]);
        pos.push([p1.x as f32, p1.y as f32, p1.z as f32]);
        pos.push([p2.x as f32, p2.y as f32, p2.z as f32]);
        vns.push([n.x as f32, n.y as f32, n.z as f32]);
        vns.push([n.x as f32, n.y as f32, n.z as f32]);
        vns.push([n.x as f32, n.y as f32, n.z as f32]);
    }
    m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vns);

    cmds.spawn((
        Mesh3d(meshes.add(m).clone()),
        MeshMaterial3d(mats.add(StandardMaterial { base_color: GRAY.into(), ..default() })),
        Transform::default(),
        Wireframe,
        WireframeColor { color: Srgba::rgb(0.3, 0.3, 0.3).into() },
        ToggleableMesh,
    ));
}

