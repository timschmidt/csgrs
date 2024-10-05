use bevy::color::palettes::basic::SILVER;
use bevy::color::palettes::css::{GRAY, GREEN, RED};
use bevy::core_pipeline::bloom::BloomSettings;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::core_pipeline::Skybox;
use bevy::pbr::wireframe::{WireframeConfig, WireframePlugin};
use bevy::pbr::{
    ScreenSpaceAmbientOcclusionBundle, ScreenSpaceAmbientOcclusionQualityLevel,
    ScreenSpaceAmbientOcclusionSettings, VolumetricFogSettings,
};
use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*, window::PresentMode};
use bevy_egui::EguiPlugin;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_screen_diagnostics::{
    ScreenDiagnosticsPlugin, ScreenEntityDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin,
};

struct GamePlugin;

#[derive(Component)]
struct Chunk;

impl Plugin for GamePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup);
        app.add_systems(Update, (toggle_wireframe, rotate));
    }
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::srgb(0.98, 0.95, 0.82),
            shadows_enabled: true,
            // illuminance: 3_000., //light_consts::lux::OVERCAST_DAY,
            illuminance: 6_000., //light_consts::lux::OVERCAST_DAY,
            // illuminance: light_consts::lux::AMBIENT_DAYLIGHT,
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
                transform: Transform {
                    translation: Vec3::new(1.494671, 1.4369639, -2.2307386),
                    rotation: Quat::from_xyzw(-0.0011287899, 0.98234415, 0.18698518, 0.0059302035),
                    scale: Vec3::new(1.0, 1.0, 1.0),
                },
                // transform: Transform::from_xyz(-1.9573995, 1.9533201, -1.9587312)
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
            ambient_intensity: 0.1,
            ..default()
        })
        .insert(ScreenSpaceAmbientOcclusionBundle::default())
        .insert(ScreenSpaceAmbientOcclusionSettings {
            quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Ultra,
        });

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
        perceptual_roughness: 0.7,
        reflectance: 0.4,
        ..default()
    });

    let mut naive_chunk = voxelis::Chunk::new();
    naive_chunk.generate_test_data();
    let naive_mesh = naive_chunk.generate_mesh().unwrap();

    let mut greedy_chunk = voxelis::Chunk::new();
    greedy_chunk.generate_test_data();
    let greedy_mesh = greedy_chunk.generate_mesh().unwrap();

    let naive_mesh = meshes.add(naive_mesh);
    let greedy_mesh = meshes.add(greedy_mesh);

    let naive_chunk_position = IVec3::new(1, 0, -1);
    let greedy_chunk_position = IVec3::new(-1, 0, -1);

    let naive_mesh_material = materials.add(StandardMaterial {
        base_color: Color::from(RED),
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        ..default()
    });

    let greedy_mesh_material = materials.add(StandardMaterial {
        base_color: Color::from(GREEN),
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        ..default()
    });

    commands
        .spawn(PbrBundle {
            mesh: naive_mesh,
            material: naive_mesh_material,
            transform: Transform::from_translation(naive_chunk_position.as_vec3()),
            ..default()
        })
        .insert(Chunk)
        .insert(Name::new("Naive Chunk".to_string()));

    commands
        .spawn(PbrBundle {
            mesh: greedy_mesh,
            material: greedy_mesh_material,
            transform: Transform::from_translation(greedy_chunk_position.as_vec3()),
            ..default()
        })
        .insert(Chunk)
        .insert(Name::new("Greedy Chunk".to_string()));

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

fn toggle_wireframe(
    mut wireframe_config: ResMut<WireframeConfig>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        wireframe_config.global = !wireframe_config.global;
    }
}

fn rotate(query: Query<&mut Transform, With<Camera>>) {
    println!("cam: {:?}", query.iter().next());
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "VoxTreeModel Viewer".to_string(),
                    present_mode: PresentMode::Immediate,
                    ..default()
                }),
                ..default()
            }),
            EguiPlugin,
            GamePlugin,
            PanOrbitCameraPlugin,
            WireframePlugin,
            FrameTimeDiagnosticsPlugin,
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
        .insert_resource(Msaa::Off)
        .run();

    println!("Exiting...");
}
