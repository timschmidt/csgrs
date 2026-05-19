//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::css::*;
use bevy::pbr::wireframe::{Wireframe, WireframeColor, WireframePlugin};
use bevy::prelude::*;
use bevy::render::mesh::PrimitiveTopology;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hypermesh::prelude::*;
use std::f64::consts::PI;
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

    let num = 4;
    let res = menger_sponge(num);

    println!(
        ">>>>>>>>>>>>>> Compute a menger sponge of level {}, elapsed time: {:?}",
        num,
        now.elapsed()
    );

    cmds.spawn((DirectionalLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((
        Transform::from_translation(Vec3::new(0., 0., 2.)),
        PanOrbitCamera::default(),
    ));

    let mut m = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    let mut pos = vec![];
    let mut vns = vec![];
    for (fid, hs) in res.hs.chunks(3).enumerate() {
        let p0 = res.ps[hs[0].tail];
        let p1 = res.ps[hs[1].tail];
        let p2 = res.ps[hs[2].tail];
        let n = res.face_normals[fid];
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
        MeshMaterial3d(mats.add(StandardMaterial {
            base_color: GRAY.into(),
            ..default()
        })),
        Transform::default(),
        Wireframe,
        WireframeColor {
            color: Srgba::rgb(0.3, 0.3, 0.3).into(),
        },
        ToggleableMesh,
    ));
}

pub fn menger_sponge(n: usize) -> Manifold {
    let res = Manifold::new(&PS, &TS).unwrap();
    let mut holes = vec![];
    fractal(&res, &mut holes, 0., 0., 1., 1, n);
    let holes_z = compose(&holes).unwrap();

    let rot = |rx: f64, ry: f64, rz: f64| {
        let ts = holes_z.hs.iter().map(|h| h.tail).collect::<Vec<_>>();
        let mut ps = holes_z.ps.clone();
        let x = rx as hypermesh::Real;
        let y = ry as hypermesh::Real;
        let z = rz as hypermesh::Real;
        let r = hypermesh::Mat3::from_euler(glam::EulerRot::XYZ, x, y, z);
        for p in ps.iter_mut() {
            *p = r * *p;
        }
        let mut flat = vec![];
        for p in ps {
            flat.push(if (p.x - 0.5).abs() < 1e-4 {
                0.5
            } else if (p.x + 0.5).abs() < 1e-4 {
                -0.5
            } else {
                p.x as f64
            });
            flat.push(if (p.y - 0.5).abs() < 1e-4 {
                0.5
            } else if (p.y + 0.5).abs() < 1e-4 {
                -0.5
            } else {
                p.y as f64
            });
            flat.push(if (p.z - 0.5).abs() < 1e-4 {
                0.5
            } else if (p.z + 0.5).abs() < 1e-4 {
                -0.5
            } else {
                p.z as f64
            });
        }
        Manifold::new(&flat, &ts).unwrap()
    };

    let holes_x = rot(PI / 2., 0., 0.);
    let holes_y = rot(0., PI / 2., 0.);

    let res = compute_boolean_with_report(&res, &holes_z, OpType::Subtract)
        .unwrap()
        .mesh;
    let res = compute_boolean_with_report(&res, &holes_x, OpType::Subtract)
        .unwrap()
        .mesh;
    let res = compute_boolean_with_report(&res, &holes_y, OpType::Subtract)
        .unwrap()
        .mesh;
    res
}

const PS: [f64; 24] = [
    -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5,
    0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5,
];

const TS: [usize; 36] = [
    1, 0, 4, 2, 4, 0, 1, 3, 0, 3, 1, 5, 3, 2, 0, 3, 7, 2, 5, 4, 6, 5, 1, 4, 6, 4, 2, 7, 6, 2, 7, 3,
    5, 7, 5, 6,
];

pub fn compose(ms: &Vec<Manifold>) -> std::result::Result<Manifold, String> {
    let mut ps = vec![];
    let mut ts = vec![];
    let mut offset = 0;
    for m in ms {
        for h in m.hs.iter() {
            ts.push(h.tail + offset);
        }
        for p in m.ps.iter() {
            ps.push(p.x as f64);
            ps.push(p.y as f64);
            ps.push(p.z as f64);
        }
        offset += m.nv;
    }
    Manifold::new(&ps, &ts)
}

pub fn fractal(
    hole: &Manifold,
    holes: &mut Vec<Manifold>,
    x: f64,
    y: f64,
    w: f64,
    depth: usize,
    depth_max: usize,
) {
    let w = w / 3.;
    let p = hole
        .ps
        .iter()
        .map(|p| [p.x as f64 * w + x, p.y as f64 * w + y, p.z as f64])
        .flatten()
        .collect::<Vec<f64>>();
    holes.push(Manifold::new(&p, &TS).unwrap());

    if depth == depth_max {
        return;
    }

    for xy in [
        (x - w, y - w),
        (x - w, y),
        (x - w, y + w),
        (x, y + w),
        (x + w, y + w),
        (x + w, y),
        (x + w, y - w),
        (x, y - w),
    ] {
        fractal(hole, holes, xy.0, xy.1, w, depth + 1, depth_max);
    }
}
