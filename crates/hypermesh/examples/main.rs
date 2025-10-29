use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::Indices;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use nalgebra::DMatrix;
use mfd::{intersect12, winding03, Boolean3, Manifold, OpType};
use mfd::hmesh::Hmesh;

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
    if keyboard.just_pressed(KeyCode::Space)  { cb(&mut vis[1]); cb(&mut vis[2]); }
    if keyboard.just_pressed(KeyCode::Digit0) { cb(&mut vis[1]); }
    if keyboard.just_pressed(KeyCode::Digit1) { cb(&mut vis[2]); }
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

    let (m0, _) = tobj::load_obj("assets/models/gargoyle.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    let (m1, _) = tobj::load_obj("assets/models/fertility.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    let mut hms_ = vec![];
    for (m, s) in vec![(m0, 1.), (m1, 0.01)] {
        let mesh = &m[0].mesh;
        let pos_buf = mesh.positions.iter().map(|&v| (v * s) as f64).collect::<Vec<f64>>();
        let idx_buf = mesh.indices.iter().map(|&v| v as usize).collect::<Vec<usize>>();
        let hm = Hmesh::new(
            DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into(),
            DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into()
        );
        hms_.push(hm);
    }


    //hms.push(gen_tet_a());
    //hms.push(gen_tet_c());
    let mf0 = Manifold::new(&hms_[0]);
    let mf1 = Manifold::new(&hms_[1]);
    let mfs = vec![&mf0, &mf1];

    let expand = -1.;
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    let (x12, v12) = intersect12(mfs[0], mfs[1], &mut p1q2, expand, true);
    let (x21, v21) = intersect12(mfs[0], mfs[1], &mut p2q1, expand, false);
    let w03 = winding03(mfs[0], mfs[1], expand, true);
    let w30 = winding03(mfs[0], mfs[1], expand, false);

    let mut pts_i = Vec::new();
    let mut pts_w = Vec::new();
    let mut edges = Vec::new();

    for p in v12.iter() { pts_i.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)); }
    for p in v21.iter() { pts_i.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)); }

    for (i, w) in w03.iter().enumerate() {
        if *w == 0 { continue; }
        let p = mfs[0].pos[i];
        pts_w.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }
    for (i, w) in w30.iter().enumerate() {
        if *w == 0 { continue; }
        let p = mfs[1].pos[i];
        pts_w.push(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }

    let boolean = Boolean3{
        mfd_p: &mfs[0], mfd_q: &mfs[1],
        p1q2, p2q1, x12, x21, w03, w30, v12, v21 };
    let (pos, halfs, tris) = boolean.get_result(OpType::Subtract);
    for h in halfs {
        let p0 = pos[h.tail];
        let p1 = pos[h.head];
        edges.push((
            Vec3::new(p0[0] as f32, p0[1] as f32, p0[2] as f32),
            Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32),
        ));
    }

    let mut normals = vec![];
    for (i, t) in tris.iter().enumerate() {
        let p0 = pos[t[0]];
        let p1 = pos[t[1]];
        let p2 = pos[t[2]];
        let c = (p0 + p1 + p2) / 3.;
        let n = (p1 - p0).cross(&(p2 - p0)).normalize() * 0.1;
        normals.push((
            Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32),
            Vec3::new((c[0] + n[0]) as f32, (c[1] + n[1]) as f32, (c[2] + n[2]) as f32),
        ));
    }

    {
        //let pos_buf: Vec<f64> = pos.iter().flat_map(|row| row.iter().copied()).collect();
        //let idx_buf: Vec<usize> = tris.iter().flat_map(|row| row.iter().copied()).collect();
        //let hm = Hmesh::new(
        //    DMatrix::from_row_slice(pos.len(), 3, &pos_buf).into(),
        //    DMatrix::from_row_slice(tris.len(), 3, &idx_buf).into()
        //);
        //hms_.push(hm);
        let mut bm = Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default()
        );

        bm.insert_indices(Indices::U32(
            tris.iter().flat_map(|t| [
                t[0] as u32,
                t[1] as u32,
                t[2] as u32,
            ]).collect()
        ));
        bm.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            pos.iter().map(|p| [
                p[0] as f32,
                p[1] as f32,
                p[2] as f32,
            ]).collect::<Vec<_>>()
        );

        cmds.spawn((
            Mesh3d(meshes.add(bm).clone()),
            MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
            Transform::default(),
            Wireframe,
            WireframeColor { color: GRAY.into() },
            ToggleableMesh,
        ));

    }

    cmds.insert_resource(
        DrawingData{ units: vec![
            //DrawingUnit { pts: pts_i, lines: vec![], boxes: vec![], col: RED },
            //DrawingUnit { pts: pts_w, lines: vec![], boxes: vec![], col: BLUE },
            //DrawingUnit { pts: vec![], lines: edges, boxes: vec![], col: WHITE },
            //DrawingUnit { pts: vec![], lines: normals, boxes: vec![], col: RED },
        ]});

    for hm in hms_ {
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

        cmds.spawn((
            Mesh3d(meshes.add(bm).clone()),
            MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
            Transform::default(),
            Wireframe,
            WireframeColor { color: GRAY.into() },
            ToggleableMesh,
        ));
    }

    cmds.insert_resource(MfdHandle0(mf0));
    cmds.insert_resource(MfdHandle1(mf1));
    cmds.spawn((PointLight::default(), Transform::from_xyz(3., 4., 3.)));
    cmds.spawn((Transform::from_translation(Vec3::new(0., 1.5, 5.)), PanOrbitCamera::default(),));
}

fn draw_example_collection(
    mfd_handle0: Res<MfdHandle0>,
    mfd_handle1: Res<MfdHandle1>,
    drawings: Res<DrawingData>,
    mut gizmos: Gizmos,
) {

    //for (i, aabb) in mfd_handle0.0.collider.aabbs.iter().enumerate() {
    //    let min = aabb.bbox.min.cast::<f32>();
    //    let max = aabb.bbox.max.cast::<f32>();
    //    let cen = (min + max) / 2.0;
    //    let size = max - min;
    //    let transform = Transform::from_translation(Vec3::new(cen.x, cen.y, cen.z)).with_scale(Vec3::new(size.x, size.y, size.z));
    //    gizmos.cuboid(transform, RED);
    //}

    for unit in &drawings.units {
        for pt in &unit.pts { gizmos.sphere(*pt, 0.001, unit.col); }
        for (sta, end) in &unit.lines { gizmos.line(*sta, *end, unit.col); }
    }

    /*
        for i in 0..hmesh_handle.0.n_vert {
            let p0 = hmesh_handle.0.verts[i].pos();
            let p1 = p0 + hmesh_handle.0.vert_normal.row(i) * 0.1;
            gizmos.line(
                Vec3::new(p0.x as f32, p0.y as f32, p0.z as f32),
                Vec3::new(p1.x as f32, p1.y as f32, p1.z as f32),
                RED
            );
        }

        for i in 0..hmesh_handle.0.n_face {
            let p0 = hmesh_handle.0.bary_center.row(i);
            let p1 = p0 + hmesh_handle.0.face_normal.row(i) * 0.1;
            gizmos.line(
                Vec3::new(p0[0] as f32, p0[1] as f32, p0[2] as f32),
                Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32),
                BLUE
            );
        }
    */
}
