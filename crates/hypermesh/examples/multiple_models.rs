use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::PrimitiveTopology;
use bevy::color::palettes::css::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use boolmesh::{Real, compute_boolean, Manifold, OpType};

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
    //let obj_path_1 = "/path/to/obj/file.obj";
    //let obj_path_2 = "/path/to/obj/file.obj";
    let obj_path_1 = "/Users/saki/dev/models/gargoyle.obj";
    let obj_path_2 = "/Users/saki/dev/models/double-torus.obj";
    let (m0, _) = tobj::load_obj(obj_path_1, &tobj::LoadOptions { ..Default::default() }).expect("Failed to load the first obj file");
    let (m1, _) = tobj::load_obj(obj_path_2, &tobj::LoadOptions { ..Default::default() }).expect("Failed to load the second obj file");

    let mut mfs = vec![];
    for m in vec![&m0[0].mesh, &m1[0].mesh] {
        mfs.push(Manifold::new(
            &m.positions.iter().map(|&v| v as Real).collect::<Vec<_>>(),
            &m.indices.iter().map(|&v| v as usize).collect::<Vec<_>>(),
        ).unwrap());
    }
    mfs.push(compute_boolean(&mfs[0], &mfs[1], OpType::Subtract).unwrap());

    for (i, mf) in mfs.iter().enumerate() {
        let mut m = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        let mut pos = vec![];
        let mut vns = vec![];
        for (fid, hs) in mf.hs.chunks(3).enumerate() {
            let p0 = mf.ps[hs[0].tail];
            let p1 = mf.ps[hs[1].tail];
            let p2 = mf.ps[hs[2].tail];
            let n  = mf.face_normals[fid];
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
            MeshMaterial3d(mats.add(StandardMaterial { ..default() })),
            Transform::default(),
            Wireframe,
            WireframeColor { color: BLACK.into() },
            ToggleableMesh,
            if i == 2 { Visibility::Visible } else { Visibility::Hidden },
        ));
    }
    cmds.spawn((PointLight::default(), Transform::from_xyz(2., 5., 2.)));
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

