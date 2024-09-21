use bevy::color::palettes::basic::SILVER;
use bevy::core_pipeline::bloom::BloomSettings;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::core_pipeline::Skybox;
use bevy::pbr::wireframe::{WireframeConfig, WireframePlugin};
use bevy::pbr::{CascadeShadowConfigBuilder, VolumetricFogSettings};
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    window::PresentMode,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_screen_diagnostics::{
    ScreenDiagnosticsPlugin, ScreenEntityDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin,
};
use voxelis::obj_reader;
use voxelis::voxelizer::Voxelizer;

struct GamePlugin;

#[derive(Component)]
struct Chunk;

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
        app.add_systems(Update, (rotate, toggle_wireframe));
    }
}

fn generate_chunk_color(index: usize, total: usize) -> Color {
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

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    println!("cwd: {:?}", std::env::current_dir().unwrap().display());

    // let obj = obj_reader::Obj::parse("assets/procedural_brick_wall.obj");
    // let obj = obj_reader::Obj::parse("assets/column.obj");
    // let obj = obj_reader::Obj::parse("assets/cylinder.obj");
    // let obj = obj_reader::Obj::parse("assets/default_cube.obj");
    // let obj = obj_reader::Obj::parse("assets/gear.obj");
    // let obj = obj_reader::Obj::parse("assets/icosphere.obj");
    // let obj = obj_reader::Obj::parse("assets/sphere.obj");
    // let obj = obj_reader::Obj::parse("assets/suzanne.obj");
    // let obj = obj_reader::Obj::parse("assets/torus.obj");
    // let obj = obj_reader::Obj::parse("assets/torus_knot.obj");
    // let obj = obj_reader::Obj::parse("assets/wall.obj");
    // let obj = obj_reader::Obj::parse("assets/wall_arc.obj");
    // let obj = obj_reader::Obj::parse("assets/wall_dome.obj");
    // let obj = obj_reader::Obj::parse("assets/wall_floor.obj");
    let obj = obj_reader::Obj::parse("assets/worm_gear.obj");
    let mut voxelizer = Voxelizer::new(obj);
    voxelizer.voxelize();

    commands
        .spawn((
            Camera3dBundle {
                transform: Transform::from_xyz(0.0, 7., 14.0)
                    .looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
                camera: Camera {
                    hdr: true,
                    ..default()
                },
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
            ambient_intensity: 0.0,
            ..default()
        });

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

    let white = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 1.0,
        ..default()
    });

    let chunks_len = voxelizer.chunks.len();
    for (i, chunk) in voxelizer.chunks.iter().enumerate() {
        let chunk_position = chunk.get_position();

        let mesh = chunk.generate_mesh();
        let mesh = meshes.add(mesh);

        let chunk_world_position = Vec3::new(
            chunk_position.x as f32,
            chunk_position.y as f32,
            chunk_position.z as f32,
        );

        let color = generate_chunk_color(i, chunks_len);
        let color = materials.add(StandardMaterial {
            base_color: color,
            perceptual_roughness: 1.0,
            ..default()
        });

        commands
            .spawn(PbrBundle {
                mesh,
                material: color,
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

    // let cascade_shadow_config = CascadeShadowConfigBuilder {
    //     first_cascade_far_bound: 0.3,
    //     maximum_distance: 3.0,
    //     ..default()
    // }
    // .build();

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(0.98, 0.95, 0.82),
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 0.0, 0.0)
            .looking_at(Vec3::new(-0.15, -0.05, 0.25), Vec3::Y),
        // cascade_shadow_config,
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(
            Plane3d::default()
                .mesh()
                .size(250.0, 250.0)
                .subdivisions(32),
        ),
        material: white,
        ..default()
    });
}

fn rotate(mut query: Query<&mut Transform, With<Chunk>>, time: Res<Time>) {
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
        .run();
}
