mod utils;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::Instant;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use boolean::Manifold;
use boolean::common::{Row2f, Row3f};
use crate::utils::{compose, fractal, menger_sponge, rotate, translate};

#[derive(Resource)] struct MfdHandle0(Manifold);
#[derive(Resource)] struct MfdHandle1(Manifold);

#[derive(Component)]
struct ToggleableMesh;

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

#[derive(Clone)]
struct DrawingUnit {
    pts: Vec<Vec3>,
    lines: Vec<(Vec3, Vec3)>,
    boxes: Vec<(Vec3, Vec3)>,
    col: Srgba,
}

#[derive(Resource)]
struct DrawingData {
    units: Vec<DrawingUnit>
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
    if keyboard.just_pressed(KeyCode::Digit0) { cb(&mut vis[0]); }
    if keyboard.just_pressed(KeyCode::Digit1) { cb(&mut vis[1]); }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_plugins(PanOrbitCameraPlugin)
        .init_gizmo_group::<MyRoundGizmos>()
        .add_systems(Startup, setup)
        .add_systems(Update, draw_example_collection
            .run_if(resource_exists::<MfdHandle0>)
            .run_if(resource_exists::<MfdHandle1>)
        )
        .add_systems(Update, toggle_mesh_visibility)
        .run();
}

fn setup(
    mut cmds: Commands,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    //let (m0, _) = tobj::load_obj("assets/models/cube_x_plus.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    //let (m1, _) = tobj::load_obj("assets/models/fertility_004.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");

    //let mut mfs = vec![];
    //for (m, s) in vec![(m0, 1.), (m1, 1.)] {
    //    let mesh = &m[0].mesh;
    //    let pos_buf = mesh.positions.iter().map(|&v| (v * s) as f64).collect::<Vec<f64>>();
    //    let idx_buf = mesh.indices.iter().map(|&v| v as usize).collect::<Vec<usize>>();
    //    let mf = Manifold::new(&pos_buf, &idx_buf, None, None).unwrap();
    //    mfs.push(mf);
    //}

    let res = menger_sponge(4);

    cmds.insert_resource(
        DrawingData{ units: vec![
            //DrawingUnit { pts: pts_i, lines: vec![], boxes: vec![], col: RED },
            //DrawingUnit { pts: pts_w, lines: vec![], boxes: vec![], col: WHITE },
            //DrawingUnit { pts: vec![], lines: edges, boxes: vec![], col: BLUE },
        ]});

    //for mf in mfs.iter() {
    //    let mut bm = Mesh::new(
    //        bevy::render::mesh::PrimitiveTopology::TriangleList,
    //        RenderAssetUsages::default()
    //    );
    //    bm.insert_indices(Indices::U32(mf.hs.iter().map(|h| h.tail as u32).collect()));
    //    bm.insert_attribute(
    //        Mesh::ATTRIBUTE_POSITION,
    //        mf.ps.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]).collect::<Vec<_>>()
    //    );
    //    cmds.spawn((
    //        Mesh3d(meshes.add(bm).clone()),
    //        MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
    //        Transform::default(),
    //        Wireframe,
    //        WireframeColor { color: GRAY.into() },
    //        ToggleableMesh,
    //    ));
    //}
    //let mf1 = mfs.pop().expect("missing second manifold");
    //let mf0 = mfs.pop().expect("missing first manifold");
    //cmds.insert_resource(MfdHandle0(mf0));
    //cmds.insert_resource(MfdHandle1(mf1));

    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));

    let mut bm = Mesh::new(bevy::render::mesh::PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bm.insert_indices(Indices::U32(res.hs.chunks(3).flat_map(|t| [t[0].tail as u32, t[1].tail as u32, t[2].tail as u32]).collect()));
    bm.insert_attribute(Mesh::ATTRIBUTE_POSITION, res.ps.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect::<Vec<_>>());
    cmds.spawn((
        Mesh3d(meshes.add(bm).clone()),
        MeshMaterial3d(mats.add(StandardMaterial { ..default()})),
        Transform::default(),
        Wireframe,
        WireframeColor { color: GRAY.into() },
        ToggleableMesh,
    ));
}

fn draw_example_collection(drawings: Res<DrawingData>, mut gizmos: Gizmos) {
    for unit in &drawings.units {
        for pt in &unit.pts { gizmos.sphere(*pt, 0.01, unit.col); }
        for (sta, end) in &unit.lines { gizmos.line(*sta, *end, unit.col); }
    }
}

pub fn save_obj(
    path: impl AsRef<Path>,
    pos: &[[f32; 3]],
    idx: &[[usize; 3]],
    nml: Option<&[[f32; 3]]>,
    uvs: Option<&[[f32; 2]]>,
) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    for p in pos {
        writeln!(&mut w, "v {:.8} {:.8} {:.8}", p[0], p[1], p[2])?;
    }

    if let Some(ts) = uvs {
        for t in ts { writeln!(&mut w, "vt {:.8} {:.8}", t[0], t[1])?; }
    }

    if let Some(ns) = nml {
        for n in ns { writeln!(&mut w, "vn {:.8} {:.8} {:.8}", n[0], n[1], n[2])?; }
    }

    match (uvs.is_some(), nml.is_some()) {
        (true, true) => {
            for tri in idx {
                writeln!(
                    &mut w,
                    "f {}/{}/{} {}/{}/{} {}/{}/{}",
                    tri[0] + 1, tri[0] + 1, tri[0] + 1,
                    tri[1] + 1, tri[1] + 1, tri[1] + 1,
                    tri[2] + 1, tri[2] + 1, tri[2] + 1,
                )?;
            }
        }
        (true, false) => {
            for tri in idx {
                writeln!(
                    &mut w,
                    "f {}/{} {}/{} {}/{}",
                    tri[0] + 1, tri[0] + 1,
                    tri[1] + 1, tri[1] + 1,
                    tri[2] + 1, tri[2] + 1,
                )?;
            }
        }
        (false, true) => {
            for tri in idx {
                writeln!(
                    &mut w,
                    "f {}//{} {}//{} {}//{}",
                    tri[0] + 1, tri[0] + 1,
                    tri[1] + 1, tri[1] + 1,
                    tri[2] + 1, tri[2] + 1,
                )?;
            }
        }
        (false, false) => {
            for tri in idx {
                writeln!(&mut w, "f {} {} {}", tri[0] + 1, tri[1] + 1, tri[2] + 1)?;
            }
        }
    }

    w.flush()?;
    Ok(())
}
