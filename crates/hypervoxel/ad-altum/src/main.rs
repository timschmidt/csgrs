use std::time::Instant;

use bevy::color::palettes::basic::SILVER;
use bevy::color::palettes::css::GRAY;
use bevy::core_pipeline::bloom::BloomSettings;
// use bevy::core_pipeline::experimental::taa::{TemporalAntiAliasBundle, TemporalAntiAliasPlugin};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::core_pipeline::Skybox;
use bevy::pbr::wireframe::{WireframeConfig, WireframePlugin};
use bevy::pbr::{
    // CascadeShadowConfigBuilder, ScreenSpaceAmbientOcclusionBundle,
    // ScreenSpaceAmbientOcclusionQualityLevel, ScreenSpaceAmbientOcclusionSettings,
    VolumetricFogSettings,
};
use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*, window::PresentMode};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_screen_diagnostics::{
    ScreenDiagnosticsPlugin, ScreenEntityDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin,
};
use voxelis::obj_reader;
use voxelis::voxelizer::Voxelizer;

struct GamePlugin;

#[derive(Component)]
struct Chunk;

#[derive(Resource)]
pub struct VoxelizerResource(pub Voxelizer);

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
        app.add_systems(Update, (rotate, toggle_wireframe));
    }
}

// fn generate_chunk_color(index: usize, total: usize) -> Color {
//     // Normalize the index to generate distinct hues
//     let hue = (index as f32 / total as f32) * 360.0;

//     // Convert HSL to RGB
//     let rgb = hsl_to_rgb(hue, 0.7, 0.5); // Saturation: 0.7, Lightness: 0.5
//     Color::srgba(rgb.0, rgb.1, rgb.2, 1.0)
// }

// fn hsl_to_rgb(hue: f32, saturation: f32, lightness: f32) -> (f32, f32, f32) {
//     let c = (1.0 - (2.0 * lightness - 1.0).abs()) * saturation;
//     let x = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
//     let m = lightness - c / 2.0;

//     let (r, g, b) = match hue {
//         0.0..=60.0 => (c, x, 0.0),
//         60.0..=120.0 => (x, c, 0.0),
//         120.0..=180.0 => (0.0, c, x),
//         180.0..=240.0 => (0.0, x, c),
//         240.0..=300.0 => (x, 0.0, c),
//         _ => (c, 0.0, x),
//     };

//     (r + m, g + m, b + m)
// }

fn generate_base_chunk_color(x: i32, y: i32, z: i32) -> Color {
    // Use prime numbers to create a unique seed for each coordinate
    let seed = x as f32 * 31.0 + y as f32 * 37.0 + z as f32 * 41.0;

    // Generate color components using trigonometric functions
    let r = (seed.sin() * 0.5 + 0.5).fract();
    let g = (seed.cos() * 0.5 + 0.5).fract();
    let b = ((seed * 0.1).tan() * 0.5 + 0.5).fract();

    Color::srgba(r, g, b, 1.0)
}

// fn generate_chunk_color(x: i32, y: i32, z: i32) -> Color {
//     let base_color = generate_base_chunk_color(x, y, z);
//     // Shift the color based on the chunk's position in the grid
//     let shift = (x + y + z) as f32 * 0.1;
//     let r = (base_color.to_srgba().red + shift).fract();
//     let g = (base_color.to_srgba().green + shift * 1.5).fract();
//     let b = (base_color.to_srgba().blue + shift * 2.0).fract();

//     Color::srgba(r, g, b, 1.0)
// }

fn generate_chunk_color(x: i32, y: i32, z: i32) -> Color {
    // Determine if the chunk should be black based on the sum of its coordinates
    let is_black = (x + y + z) % 2 == 0;

    if is_black {
        Color::from(SILVER)
    } else {
        Color::WHITE
    }
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut voxelizer: ResMut<VoxelizerResource>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let voxelizer = &mut voxelizer.0;

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(0.98, 0.95, 0.82),
            shadows_enabled: true,
            // illuminance: 5_000., //light_consts::lux::OVERCAST_DAY,
            illuminance: light_consts::lux::AMBIENT_DAYLIGHT,

            ..default()
        },
        transform: Transform::from_xyz(0.0, 0.0, 0.0)
            .looking_at(Vec3::new(-0.15, -0.05, 0.25), Vec3::Y),
        ..default()
    });

    commands
        .spawn((
            Camera3dBundle {
                camera: Camera {
                    hdr: true,
                    ..default()
                },
                // transform: Transform::from_xyz(0.0, 7., 14.0)
                transform: Transform::from_xyz(2.2716377, 1.2876732, 3.9676127)
                // transform: Transform::from_xyz(-1.9573995, 1.9533201, -1.9587312)
                    .looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
                ..default()
            },
            PanOrbitCamera::default(),
        ))
        .insert(Tonemapping::TonyMcMapface)
        .insert(BloomSettings::default())
        .insert(Skybox {
            image: asset_server.load("environment_maps/pisa_specular_rgb9e5_zstd.ktx2"),
            brightness: 1000.0,
        })
        .insert(VolumetricFogSettings {
            // This value is explicitly set to 0 since we have no environment map light
            ambient_intensity: 0.5,
            ..default()
        })
        // .insert(ScreenSpaceAmbientOcclusionBundle::default())
        // .insert(ScreenSpaceAmbientOcclusionSettings {
        //     quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Ultra,
        // })
        // .insert(TemporalAntiAliasBundle::default())
        ;

    // commands.spawn(EnvironmentMapLight {
    //     diffuse_map: asset_server.load("environment_maps/pisa_diffuse_rgb9e5_zstd.ktx2"),
    //     specular_map: asset_server.load("environment_maps/pisa_specular_rgb9e5_zstd.ktx2"),
    //     intensity: 2_000.0,
    // });
    //
    commands.spawn(
        TextBundle::from_section("Press space to toggle wireframes", TextStyle::default())
            .with_style(Style {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            }),
    );

    let ground_material = materials.add(StandardMaterial {
        base_color: Color::from(GRAY),
        // base_color: Color::WHITE,
        perceptual_roughness: 0.7,
        reflectance: 0.4,
        ..default()
    });

    let mesh_material = materials.add(StandardMaterial {
        base_color: Color::from(SILVER),
        // base_color: Color::WHITE,
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        ..default()
    });

    let now = Instant::now();

    println!("Generating meshes...");

    let chunks_len = voxelizer.chunks.len();
    for (i, chunk) in voxelizer.chunks.iter().enumerate() {
        print!("Generating mesh for chunk {}/{}\r", i + 1, chunks_len);

        let chunk_position = chunk.get_position();

        let mesh = chunk.generate_mesh();
        let mesh = meshes.add(mesh);

        let chunk_world_position = Vec3::new(
            chunk_position.x as f32,
            chunk_position.y as f32,
            chunk_position.z as f32,
        );

        let mesh_material = materials.add(StandardMaterial {
            base_color: generate_chunk_color(chunk_position.x, chunk_position.y, chunk_position.z),
            perceptual_roughness: 1.0,
            reflectance: 0.0,
            ..default()
        });

        commands
            .spawn(PbrBundle {
                mesh,
                material: mesh_material.clone(),
                // transform: Transform::from_xyz(0.0, 0.0, -5.0),
                transform: Transform::from_translation(chunk_world_position),
                ..default()
            })
            .insert(Chunk)
            .insert(Name::new(
                format!(
                    "Chunk {}x{}x{}",
                    chunk_position.x, chunk_position.y, chunk_position.z
                )
                .to_string(),
            ));
    }

    println!("\nGenerating meshes took {:?}", now.elapsed());

    commands.spawn(PbrBundle {
        mesh: meshes.add(
            Plane3d::default()
                .mesh()
                .size(250.0, 250.0)
                .subdivisions(32),
        ),
        material: ground_material,
        ..default()
    });
}

fn rotate(mut query: Query<&mut Transform, With<Camera>>, _time: Res<Time>) {
    // let cam = query.iter_mut().next().unwrap();
    // println!("Cam: {:?}", cam.translation);
    // for mut transform in &mut query {
    //     transform.rotate_y(time.delta_seconds() / 2.);
    // }
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
    // let obj = obj_reader::Obj::parse("ad-altum/assets/procedural_brick_wall.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/barn_0.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/column.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/cylinder.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/default_cube.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/fence_0.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/gear.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/icosphere.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/polonez.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/rhino.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/sphere.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_01.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_02.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_02_huge.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_02_human_reference.obj");
    let obj = obj_reader::Obj::parse("ad-altum/assets/statue_03.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_04.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_05.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_06.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_07.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_08.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_09.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/statue_10.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/suzanne.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/torus.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/torus_knot.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/wall.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/wall_arc.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/wall_dome.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/wall_floor.obj");
    // let obj = obj_reader::Obj::parse("ad-altum/assets/worm_gear.obj");
    let mut voxelizer = Voxelizer::new(obj);
    voxelizer.voxelize();
    // voxelizer.simple_voxelize();

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Ad Altum".to_string(),
                    present_mode: PresentMode::Immediate,
                    ..default()
                }),
                ..default()
            }),
            EguiPlugin,
            // TemporalAntiAliasPlugin,
            GamePlugin,
            PanOrbitCameraPlugin,
            WireframePlugin,
            FrameTimeDiagnosticsPlugin,
            // LogDiagnosticsPlugin::default(),
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
        .insert_resource(VoxelizerResource(voxelizer))
        .insert_resource(Msaa::Off)
        .run();

    println!("Exiting...");
}
