use std::sync::Arc;
use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::basic::{BLACK, WHITE};
use bevy::prelude::*;
use bevy::pbr::wireframe::{WireframePlugin, Wireframe, WireframeColor};
use bevy::render::mesh::{Indices, VertexAttributeValues};
use nalgebra::DMatrix;
use mfd::Hmesh;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_systems(Startup, (load_mesh, setup))
        .add_systems(Update, convert_to_hmesh.run_if(
            resource_exists::<BunnyMeshHandle>.and(not(resource_exists::<BunnyHmesh>))
        ))
        .add_systems(Update, spin)
        .run();
}

fn load_mesh(
    mut cmds: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let (models, _) = tobj::load_obj(
        "assets/models/quad_with_hole.obj",
        &tobj::LoadOptions {
            //triangulate: true,
            //single_index: true, // This is important!
            ..Default::default()
        },
    ).expect("Failed to OBJ load file");
    let model = &models[0];
    let mesh = &model.mesh;

    //println!("tobj positions len: {}", mesh.positions.len());
    //println!("tobj indices   len: {}", mesh.indices.len());
    //println!("tobj normals   len: {}", mesh.normals.len());

    let mut bevy_mesh = Mesh::new(
        bevy::render::mesh::PrimitiveTopology::TriangleList,
        RenderAssetUsages::default()
    );

    let positions: Vec<[f32; 3]> = (0..mesh.positions.len() / 3)
        .map(|i| [
                mesh.positions[i * 3],
                mesh.positions[i * 3 + 1],
                mesh.positions[i * 3 + 2],
            ])
        .collect();
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

    let indices: Vec<u32> = mesh.indices.iter().map(|&i| i).collect();
    bevy_mesh.insert_indices(Indices::U32(indices));

    let mesh_handle = meshes.add(bevy_mesh);
    //let mesh_handle = asset_server.load("models/double-torus.obj");
    let mesh = Mesh3d(mesh_handle.clone());
    cmds.spawn((
        mesh,
        MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
        Transform::from_xyz(0., 0., 0.5),
        Spin,
        Wireframe,
        WireframeColor { color: WHITE.into() },
    ));
    cmds.insert_resource(BunnyMeshHandle(mesh_handle));
}

#[derive(Resource)]
struct BunnyMeshHandle(Handle<Mesh>);

#[derive(Resource)]
struct BunnyHmesh(Arc<Hmesh>);

#[derive(Component)]
struct Spin;

fn convert_to_hmesh(
    mesh_handle: Res<BunnyMeshHandle>,
    meshes: Res<Assets<Mesh>>,
    mut cmds: Commands,
) {
    if let Some(mesh) = meshes.get(&mesh_handle.0) {
        let verts = if let Some(attr) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            match attr {
                VertexAttributeValues::Float32x3(pos) => {
                    let mut m = DMatrix::zeros(pos.len(), 3);
                    for (i, [x, y, z]) in pos.iter().enumerate() {
                        m[(i, 0)] = *x as f64;
                        m[(i, 1)] = *y as f64;
                        m[(i, 2)] = *z as f64;
                    }
                    m
                },
                _ => panic!("Unexpected vertex format"),
            }
        } else { return; };

        let idcs = if let Some(indices) = mesh.indices() {
            match indices {
                Indices::U16(idx) => {
                    let nf = idx.len() / 3;
                    let mut m = DMatrix::zeros(nf, 3);
                    for i in 0..nf {
                        m[(i, 0)] = idx[i * 3] as usize;
                        m[(i, 1)] = idx[i * 3 + 1] as usize;
                        m[(i, 2)] = idx[i * 3 + 2] as usize;
                    }
                    m
                },
                Indices::U32(idx) => {
                    let nf = idx.len() / 3;
                    let mut m = DMatrix::zeros(nf, 3);
                    for i in 0..nf {
                        m[(i, 0)] = idx[i * 3] as usize;
                        m[(i, 1)] = idx[i * 3 + 1] as usize;
                        m[(i, 2)] = idx[i * 3 + 2] as usize;
                    }
                    m
                },
            }
        } else { return; };

        let hmesh = Hmesh::new(verts, idcs);
        println!("num vert: {}", hmesh.n_vert);
        println!("num face: {}", hmesh.n_face);
        println!("num edge: {}", hmesh.n_edge);
        println!("Euler characteristic: {}", hmesh.n_vert as i32 - hmesh.n_edge as i32 + hmesh.n_face as i32);
        cmds.insert_resource(BunnyHmesh(hmesh));
    }
}

fn setup(mut cmds: Commands) {
    cmds.spawn((PointLight::default(), Transform::from_xyz(3.0, 4.0, 3.0)));
    cmds.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 2., 4.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn spin(time: Res<Time>, mut query: Query<&mut Transform, With<Spin>>) {
    for mut transform in query.iter_mut() {
        transform.rotate_local_y(0.5 * time.delta_secs());
    }
}