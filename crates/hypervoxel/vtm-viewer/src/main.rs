use std::collections::HashMap;
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

use voxelis::{
    BlockId, Lod,
    io::import::import_model_from_vtm,
    spatial::{VoxOpsSpatial3D, VoxOpsState},
    world::VoxModel,
};

struct GamePlugin;

#[derive(Component)]
struct Chunk;

#[derive(Resource)]
pub struct ModelResource(pub VoxModel);

#[derive(Eq, PartialEq)]
pub enum MaterialType {
    Checkered,
    Gradient,
    Clay,
}

#[derive(Resource)]
pub struct ModelSettings {
    pub material_type: MaterialType,
    pub lod: Lod,
}

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
        app.add_systems(Update, toggle_wireframe);
    }
}

fn generate_chunk_color_gradient(index: usize, total: usize) -> Color {
    // Normalize the index to generate distinct hues
    let hue = (index as f32 / total as f32) * 360.0;

    // Convert HSL to RGB
    let rgb = hsl_to_rgb(hue, 0.7, 0.5); // Saturation: 0.7, Lightness: 0.5
    Color::srgba(rgb.0, rgb.1, rgb.2, 1.0)
}

fn hsl_to_rgb(hue: f32, saturation: f32, lightness: f32) -> (f32, f32, f32) {
    let c = (1.0 - (2.0 * lightness - 1.0).abs()) * saturation;
    let x = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
    let m = lightness - c / 2.0;

    let (r, g, b) = match hue {
        0.0..=60.0 => (c, x, 0.0),
        60.0..=120.0 => (x, c, 0.0),
        120.0..=180.0 => (0.0, c, x),
        180.0..=240.0 => (0.0, x, c),
        240.0..=300.0 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };

    (r + m, g + m, b + m)
}

fn generate_chunk_color_checkered(x: i32, y: i32, z: i32) -> Color {
    // Determine if the chunk should be black based on the sum of its coordinates
    let is_black = (x + y + z) % 2 == 0;

    if is_black {
        Color::from(palettes::css::SILVER)
    } else {
        Color::WHITE
    }
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut model: ResMut<ModelResource>,
    model_settings: Res<ModelSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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

    let single_mesh = false;

    let interner = model.get_interner();
    let interner = interner.read();

    if single_mesh {
        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        for (_, chunk) in model.chunks.iter() {
            if chunk.is_empty() {
                continue;
            }

            chunk.generate_mesh_arrays(
                &interner,
                &mut vertices,
                &mut normals,
                &mut indices,
                chunk.world_position_3d(),
                model_settings.lod,
            );
        }

        let mesh = Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_indices(bevy::render::mesh::Indices::U32(indices))
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

        let mesh = meshes.add(mesh);

        let base_color = Color::from(palettes::css::SILVER);

        let mesh_material = materials.add(StandardMaterial {
            base_color,
            perceptual_roughness: 1.0,
            reflectance: 0.0,
            ..default()
        });

        commands
            .spawn((Mesh3d(mesh), MeshMaterial3d(mesh_material.clone())))
            .insert(Chunk);
    } else {
        let mut existing_meshes: HashMap<BlockId, (Handle<Mesh>, usize, usize, usize)> =
            HashMap::new();

        let mut duplicates_found: usize = 0;
        let mut total_vertices: usize = 0;
        let mut total_indices: usize = 0;
        let mut total_normals: usize = 0;
        let mut saved_vertices: usize = 0;
        let mut saved_indices: usize = 0;
        let mut saved_normals: usize = 0;

        for (chunk_position, chunk) in model.chunks.iter() {
            if chunk.is_empty() {
                continue;
            }

            let mesh = if let Some((chunk_mesh, vertices, indices, normals)) =
                existing_meshes.get(&chunk.get_root_id())
            {
                duplicates_found += 1;
                saved_vertices += vertices;
                saved_indices += indices;
                saved_normals += normals;
                chunk_mesh.clone()
            } else {
                let mut vertices = Vec::new();
                let mut normals = Vec::new();
                let mut indices = Vec::new();

                chunk.generate_mesh_arrays(
                    &interner,
                    &mut vertices,
                    &mut normals,
                    &mut indices,
                    Vec3::ZERO,
                    model_settings.lod,
                );

                let vertices_len = vertices.len();
                let indices_len = indices.len();
                let normals_len = normals.len();

                total_vertices += vertices_len;
                total_indices += indices_len;
                total_normals += normals_len;

                let chunk_mesh = Mesh::new(
                    bevy::render::mesh::PrimitiveTopology::TriangleList,
                    bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
                )
                .with_inserted_indices(bevy::render::mesh::Indices::U32(indices))
                .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
                .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

                let mesh = meshes.add(chunk_mesh);

                existing_meshes.insert(
                    chunk.get_root_id(),
                    (mesh.clone(), vertices_len, indices_len, normals_len),
                );

                mesh
            };

            let chunk_world_position = chunk.world_position_3d();

            let base_color = if model_settings.material_type == MaterialType::Checkered {
                generate_chunk_color_checkered(chunk_position.x, chunk_position.y, chunk_position.z)
            } else if model_settings.material_type == MaterialType::Gradient {
                let bounds = model.world_bounds;
                let area = bounds.z * bounds.x;
                let i = chunk_position.x + chunk_position.y * bounds.x + chunk_position.z * area;
                generate_chunk_color_gradient(i as usize, model.chunks.len())
            } else {
                Color::srgba_u8(191, 157, 133, 255)
            };

            let mesh_material = materials.add(StandardMaterial {
                base_color,
                perceptual_roughness: 1.0,
                reflectance: 0.0,
                ..default()
            });

            commands
                .spawn((
                    Mesh3d(mesh),
                    MeshMaterial3d(mesh_material.clone()),
                    Transform::from_translation(chunk_world_position),
                ))
                .insert(Chunk)
                .insert(Name::new(
                    format!(
                        "Chunk {}x{}x{}",
                        chunk_position.x, chunk_position.y, chunk_position.z
                    )
                    .to_string(),
                ));
        }

        println!(
            "Found {duplicates_found} duplicates out of {} chunks ({:.1}%)",
            model.chunks.len(),
            (duplicates_found as f32 / model.chunks.len() as f32) * 100.0
        );
        println!(
            " Vertices: {} - {} saved",
            humanize_bytes::humanize_quantity!(total_vertices),
            humanize_bytes::humanize_quantity!(saved_vertices),
        );
        println!(
            " Normals: {} - {} saved",
            humanize_bytes::humanize_quantity!(total_normals),
            humanize_bytes::humanize_quantity!(saved_normals),
        );
        println!(
            " Indices: {} - {} saved",
            humanize_bytes::humanize_quantity!(total_indices),
            humanize_bytes::humanize_quantity!(saved_indices),
        );
    }

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
            EguiPlugin,
            GamePlugin,
            PanOrbitCameraPlugin,
            WireframePlugin,
            FrameTimeDiagnosticsPlugin,
            // ScreenDiagnosticsPlugin::default(),
            // ScreenFrameDiagnosticsPlugin,
            // ScreenEntityDiagnosticsPlugin,
        ))
        .insert_resource(ClearColor(Color::Srgba(Srgba {
            red: 0.02,
            green: 0.02,
            blue: 0.02,
            alpha: 1.0,
        })))
        .insert_resource(ModelResource(model))
        .insert_resource(ModelSettings {
            material_type: MaterialType::Gradient,
            lod,
        })
        .run();

    println!("Exiting...");
}
