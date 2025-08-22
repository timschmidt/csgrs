use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::basic::{BLACK, WHITE};
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::{color::palettes::css::*, math::Isometry2d};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use nalgebra::{DMatrix, RowVector3};
use mfd::{intersect12, winding03, Boolean3, Hmesh, Manifold, OpType};
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::Arc;
use rand::Rng;
use mfd::kernel02::Kernel02;
use mfd::test_data::{gen_tet_a, gen_tet_b, gen_tet_c};

#[derive(Resource)]
struct MfdHandle0(Manifold);
#[derive(Resource)]
struct MfdHandle1(Manifold);

#[derive(Component)]
struct ToggleableMesh;

#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

#[derive(Clone)]
struct DrawingUnit {
    pts: Vec<Vec3>,
    lines: Vec<(Vec3, Vec3)>,
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
            //.run_if(resource_exists::<MfdHandle0>)
            //.run_if(resource_exists::<MfdHandle1>)
        )
        .add_systems(Update, toggle_mesh_visibility)
        .run();
}

fn setup(
    mut cmds: Commands,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    //let mut hms = vec![];
    //let (m1, _) = tobj::load_obj("assets/models/cube_x_plus.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    //let (m0, _) = tobj::load_obj("assets/models/tet_b.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");

    /*
    for m in vec![m0, m1] {
        let mesh = &m[0].mesh;
        let pos_buf = mesh.positions.iter().map(|&v| v as f64).collect::<Vec<f64>>();
        let idx_buf = mesh.indices.iter().map(|&v| v as usize).collect::<Vec<usize>>();
        let hm = Hmesh::new(
            DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into(),
            DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into()
        );
        for f in hm.faces.iter() {
            assert_eq!(f.id * 3 + 0, f.half().id);
            assert_eq!(f.id * 3 + 1, f.half().next().id);
            assert_eq!(f.id * 3 + 2, f.half().prev().id);
        }
        hms.push(hm);
    }


    //hms.push(gen_tet_a());
    //hms.push(gen_tet_b());
    let mf0 = Manifold::new(&hms[0]);
    let mf1 = Manifold::new(&hms[1]);
    let mfs = vec![&mf0, &mf1];

    let expand = 1.;
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    let (x12, v12) = intersect12(&mfs[0], &mfs[1], &mut p1q2, expand, true);
    let (x21, v21) = intersect12(&mfs[0], &mfs[1], &mut p2q1, expand, false);
    let w03 = winding03(mfs[0], mfs[1], expand, true);
    let w30 = winding03(&mfs[0], &mfs[1], expand, false);

    let mut pts_i = Vec::new();
    let mut pts_w = Vec::new();
    let mut edges = Vec::new();

    for p in v12.iter() { pts_i.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)); }
    for p in v21.iter() { pts_i.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)); }

    for (i, w) in w03.iter().enumerate() {
        if *w == 0 { continue; }
        let p = mfs[0].hmesh.verts[i].pos();
        pts_w.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }
    for (i, w) in w30.iter().enumerate() {
        if *w == 0 { continue; }
        let p = mfs[1].hmesh.verts[i].pos();
        pts_w.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }
    */

    //let ker02 = Kernel02{
    //    verts_p: &mf0.hmesh.verts,
    //    verts_q: &mf1.hmesh.verts,
    //    halfs_q: &mf1.hmesh.halfs,
    //    normals: &mf0.hmesh.verts.iter().map(|v| v.normal()).collect::<Vec<_>>(),
    //    expand: -1.,
    //    forward: true
    //};

    //for i in 0..mf0.hmesh.n_vert {
    //    for j in 0..mf1.hmesh.n_face {
    //        let (s, z) = ker02.op(i, j);
    //        println!("s: {}, z: {:?}", s, z);
    //    }
    //}

    /*
    let boolean = Boolean3{
        mfd_p: &mfs[0], mfd_q: &mfs[1],
        p1q2, p2q1, x12, x21, w03, w30, v12, v21 };
    let (pos, halfs) = boolean.get_result(OpType::Subtract);
    for h in halfs {
        let p0 = pos[h.tail as usize];
        let p1 = pos[h.head as usize];
        println!("p0: {:?}", p0);
        println!("p1: {:?}", p1);
        edges.push((
            Vec3::new(p0[0] as f32, p0[1] as f32, p0[2] as f32),
            Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32),
        ));
    }


    for hm in hms {
        let mut bm = Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default()
        );

        bm.insert_indices(Indices::U32(
            (0..hm.idx.nrows()).flat_map(|i| [
                hm.idx[(i, 0)] as u32,
                hm.idx[(i, 1)] as u32,
                hm.idx[(i, 2)] as u32,
            ]).collect()
        ));
        bm.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            (0..hm.pos.nrows()).map(|i| [
                hm.pos[(i, 0)] as f32,
                hm.pos[(i, 1)] as f32,
                hm.pos[(i, 2)] as f32,
            ]).collect::<Vec<_>>()
        );

        //cmds.spawn((
        //    Mesh3d(meshes.add(bm).clone()),
        //    MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
        //    Transform::default(),
        //    Wireframe,
        //    WireframeColor { color: WHITE.into() },
        //    ToggleableMesh,
        //));
    }
    */

    let mut rnd = vec![];
    for i in 0..10000 {
        let mut rng = rand::rng();
        let x: f64 = rng.random();
        let y: f64 = rng.random();
        let z: f64 = rng.random();
        rnd.push(Vec3::new(x as f32, y as f32, z as f32));
    }

    cmds.insert_resource(
        DrawingData{ units: vec![
            DrawingUnit { pts: rnd, lines: vec![], col: RED },
            //DrawingUnit { pts: pts_i, lines: vec![], col: RED },
            //DrawingUnit { pts: pts_w, lines: vec![], col: BLUE },
            //DrawingUnit { pts: vec![], lines: edges, col: WHITE },
        ]});

    //cmds.insert_resource(MfdHandle0(mf0));
    //cmds.insert_resource(MfdHandle1(mf1));
    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));
}

fn draw_example_collection(
    //mfd_handle0: Res<MfdHandle0>,
    //mfd_handle1: Res<MfdHandle1>,
    drawings: Res<DrawingData>,
    mut gizmos: Gizmos,
) {

    let pos = vec![
        0., 0., 0.,
        1., 0., 0.,
        0., 1., 1.
    ];
    let idx = vec![0, 1, 2];
    let tails = vec![0, 1, 2, 1, 2, 0];
    let heads = vec![1, 2, 0, 0, 1, 2];
    let f_nor = vec![0., 0., 1.];
    let v_nor = vec![0., 0., 1., 0., 0., 1., 0., 0., 1.];
    let hm = Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(3, 3, &pos),
        DMatrix::from_row_slice(1, 3, &idx),
        tails,
        heads,
        Some(vec![3, 4, 5, 0, 1, 2]),
        DMatrix::from_row_slice(3, 3, &v_nor),
        DMatrix::from_row_slice(1, 3, &f_nor),
    );
    let mf = Manifold::new(&hm);
    let p0 = Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32);
    let p1 = Vec3::new(pos[3] as f32, pos[4] as f32, pos[5] as f32);
    let p2 = Vec3::new(pos[6] as f32, pos[7] as f32, pos[8] as f32);
    gizmos.line(p0, p1, WHITE);
    gizmos.line(p1, p2, WHITE);
    gizmos.line(p2, p0, WHITE);


    let rand_pts = drawings.units[0].pts.iter().map(|p| RowVector3::new(p.x as f64, p.y as f64, p.z as f64)).collect::<Vec<_>>();

    let k02 = Kernel02 {
        vpos_p: &rand_pts,
        vpos_q: &mf.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        half_q: &mf.hmesh.halfs,
        normal: &rand_pts.iter().map(|_| RowVector3::new(0., 0., 1.)).collect::<Vec<_>>(),
        expand: 1.,
        forward: true,
    };

    for i in 0..rand_pts.len() {
        let (s, z) = k02.op(i, 0);
        if z.is_some() {
            if s > 0 {
                gizmos.sphere(drawings.units[0].pts[i], 0.005, YELLOW);
            } else {
                gizmos.sphere(drawings.units[0].pts[i], 0.005, GREEN);
            }
        } else {
            gizmos.sphere(drawings.units[0].pts[i], 0.005, BLUE);
        }
    }
}
