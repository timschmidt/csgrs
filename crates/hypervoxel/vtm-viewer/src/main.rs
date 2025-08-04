use std::path::Path;
use std::time::Instant;

use bevy::color::palettes;
use bevy::core_pipeline::Skybox;
use bevy::core_pipeline::bloom::Bloom;
use bevy::core_pipeline::experimental::taa::{TemporalAntiAliasPlugin, TemporalAntiAliasing};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::pbr::wireframe::{WireframeConfig, WireframePlugin};
use bevy::pbr::{
    ScreenSpaceAmbientOcclusion, ScreenSpaceAmbientOcclusionQualityLevel, VolumetricFog,
};
use bevy::render::camera::TemporalJitter;
use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*, window::PresentMode};
use bevy_egui::EguiPlugin;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use bevy_screen_diagnostics::{
    ScreenDiagnosticsPlugin, ScreenEntityDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin,
};
use voxelis::{
    Lod,
    io::import::import_model_from_vtm,
    utils::mesh::{MeshData, generate_greedy_mesh_arrays_stride},
    world::VoxModel,
};

struct GamePlugin;

#[derive(Component)]
struct Chunk;

#[derive(Resource)]
pub struct ModelResource(pub VoxModel<i32>);

#[derive(Resource)]
pub struct ModelSettings {
    pub lod: Lod,
}

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
        app.add_systems(Update, toggle_wireframe);
        #[cfg(feature = "tracy")]
        app.add_systems(Last, tracy_mark_frame);
    }
}

#[cfg(feature = "tracy")]
fn tracy_mark_frame() {
    tracy_client::frame_mark();
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut model: ResMut<ModelResource>,
    model_settings: Res<ModelSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("setup");

    let model = &mut model.0;

    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.98, 0.95, 0.82),
            shadows_enabled: true,
            // illuminance: 3_000., //light_consts::lux::OVERCAST_DAY,
            illuminance: 6_000., //light_consts::lux::OVERCAST_DAY,
            // illuminance: light_consts::lux::AMBIENT_DAYLIGHT,
            ..default()
        },
        Transform::from_xyz(0.0, 0.0, 0.0).looking_at(Vec3::new(-0.15, -0.05, 0.25), Vec3::Y),
    ));

    commands
        .spawn((
            Camera3d::default(),
            Camera {
                hdr: true,
                ..default()
            },
            Msaa::Off,
            // transform: Transform::from_xyz(0.0, 7., 14.0)
            Transform::from_xyz(2.2716377, 1.2876732, 3.9676127)
                // transform: Transform::from_xyz(-1.9573995, 1.9533201, -1.9587312)
                .looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
            TemporalJitter::default(),
            PanOrbitCamera::default(),
        ))
        .insert(Tonemapping::TonyMcMapface)
        .insert(Bloom::default())
        .insert(Skybox {
            image: asset_server.load("environment_maps/pisa_specular_rgb9e5_zstd.ktx2"),
            brightness: 1000.0,
            ..default()
        })
        .insert(VolumetricFog {
            ambient_intensity: 0.1,
            ..default()
        })
        .insert(ScreenSpaceAmbientOcclusion {
            quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Ultra,
            ..default()
        })
        .insert(TemporalAntiAliasing::default());

    commands.spawn((
        Text::new("Press space to toggle wireframes"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));

    let ground_material = materials.add(StandardMaterial {
        base_color: Color::from(palettes::css::GRAY),
        perceptual_roughness: 0.7,
        reflectance: 0.4,
        ..default()
    });

    let now = Instant::now();

    println!("Generating meshes...");

    let interner = model.get_interner();
    let interner = interner.read();

    let mut mesh_data = MeshData::default();

    generate_greedy_mesh_arrays_stride(model, &interner, Lod::new(0), &mut mesh_data);

    let total_vertices = mesh_data.vertices.len();
    let total_indices = mesh_data.indices.len();
    let total_normals = mesh_data.normals.len();

    let mesh = Mesh::new(
        bevy::render::mesh::PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_indices(bevy::render::mesh::Indices::U32(mesh_data.indices))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, mesh_data.vertices)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, mesh_data.normals);

    let mesh = meshes.add(mesh);

    let mesh_material = materials.add(StandardMaterial {
        base_color: Color::srgba_u8(191, 157, 133, 255),
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        ..default()
    });

    commands
        .spawn((Mesh3d(mesh), MeshMaterial3d(mesh_material.clone())))
        .insert(Chunk);

    println!(
        " Vertices: {}",
        humanize_bytes::humanize_quantity!(total_vertices),
    );
    println!(
        " Normals: {}",
        humanize_bytes::humanize_quantity!(total_normals),
    );
    println!(
        " Indices: {}",
        humanize_bytes::humanize_quantity!(total_indices),
    );

    println!("Generating meshes took {:?}", now.elapsed());

    commands.spawn((
        Mesh3d(
            meshes.add(
                Plane3d::default()
                    .mesh()
                    .size(250.0, 250.0)
                    .subdivisions(32),
            ),
        ),
        MeshMaterial3d(ground_material),
    ));
}

fn toggle_wireframe(
    mut wireframe_config: ResMut<WireframeConfig>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        wireframe_config.global = !wireframe_config.global;
    }
}

fn main() {
    #[cfg(feature = "tracy")]
    tracy_client::Client::start();

    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("vtm-viewer");

    if std::env::args().len() < 2 {
        println!("Usage: vtm-viewer <vtm-file> <chunk size in m> <lod-level (0-7)>");
        std::process::exit(1);
    }

    let input = std::env::args().nth(1).unwrap();
    let input = Path::new(&input);

    let chunk_world_size = if let Some(chunk_size) = std::env::args().nth(2) {
        let chunk_size: f32 = chunk_size.parse().unwrap();
        chunk_size
    } else {
        1.28
    };

    let lod = if let Some(lod) = std::env::args().nth(3) {
        let lod: u8 = lod.parse().unwrap();
        Lod::new(lod)
    } else {
        Lod::new(0)
    };
    println!("Using LOD level {lod}");

    println!("Opening VTM model {}", input.display());
    let model = import_model_from_vtm(&input, 1024 * 1024 * 1024 * 4, Some(chunk_world_size));

    #[cfg(feature = "memory_stats")]
    {
        let interner = model.interner_stats();
        println!("Interner stats: {interner:#?}");
    }

    App::new()
        // .insert_resource(AmbientLight {
        //     brightness: 1000.,
        //     ..default()
        // })
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "VoxTreeModel Viewer".to_string(),
                    present_mode: PresentMode::Immediate,
                    ..default()
                }),
                ..default()
            }),
            TemporalAntiAliasPlugin,
            EguiPlugin::default(),
            GamePlugin,
            PanOrbitCameraPlugin,
            WireframePlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
            ScreenDiagnosticsPlugin::default(),
            ScreenFrameDiagnosticsPlugin,
            ScreenEntityDiagnosticsPlugin,
        ))
        .insert_resource(ClearColor(Color::Srgba(Srgba {
            red: 0.02,
            green: 0.02,
            blue: 0.02,
            alpha: 1.0,
        })))
        .insert_resource(ModelResource(model))
        .insert_resource(ModelSettings { lod })
        .run();

    println!("Exiting...");
}
