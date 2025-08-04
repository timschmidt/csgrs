use std::{f32::consts::PI, time::Duration};

use bevy::{
    core_pipeline::{
        Skybox,
        bloom::Bloom,
        experimental::taa::{TemporalAntiAliasPlugin, TemporalAntiAliasing},
        tonemapping::Tonemapping,
    },
    diagnostic::FrameTimeDiagnosticsPlugin,
    pbr::{
        CascadeShadowConfigBuilder, DirectionalLightShadowMap, ScreenSpaceAmbientOcclusion,
        ScreenSpaceAmbientOcclusionQualityLevel, VolumetricFog,
        wireframe::{WireframeConfig, WireframePlugin},
    },
    prelude::*,
    render::{
        camera::TemporalJitter,
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
    window::PresentMode,
};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use bevy_screen_diagnostics::{
    ScreenDiagnosticsPlugin, ScreenEntityDiagnosticsPlugin, ScreenFrameDiagnosticsPlugin,
};
use egui_plot::format_number;
use voxelis::{
    Batch, Lod, MaxDepth, VoxInterner,
    spatial::{VoxOpsBatch, VoxOpsBulkWrite, VoxOpsMesh},
    utils::{
        mesh::{MeshData, chunk_generate_greedy_mesh_arrays_ext},
        shapes,
    },
    world::VoxChunk,
};

#[cfg(feature = "trace_greedy_timings")]
use voxelis::utils::mesh::GreedyTimings;

const MAX_DEPTH: MaxDepth = MaxDepth::new(6);
const CHUNK_SIZE: f32 = 1.28;

#[derive(Debug, Default, PartialEq, Eq)]
pub enum Shape {
    Uniform,
    Sphere,
    Checkerboard,
    SparseFill,
    HollowCube,
    Diagonal,
    TerrainSurface,
    #[default]
    TerrainFull,
    Perlin3D,
}

#[derive(Resource, Default)]
pub struct StatsWindowState {
    pub visible: bool,
    pub show_details: bool,
}

#[derive(Debug, Default)]
pub struct MeshStats {
    pub vertices: usize,
    pub normals: usize,
    pub indices: usize,
    pub triangles: usize,
    pub time: Duration,
}

#[derive(Resource)]
pub struct World {
    pub interner: VoxInterner<i32>,
    pub chunk: VoxChunk<i32>,
    pub normal_mesh: Handle<Mesh>,
    pub greedy_mesh: Handle<Mesh>,
    pub normal_mesh_data: MeshData,
    pub greedy_mesh_data: MeshData,
    pub batch: Batch<i32>,
    pub normal_stats: MeshStats,
    pub greedy_stats: MeshStats,
    #[cfg(feature = "trace_greedy_timings")]
    pub timings: GreedyTimings,
}

#[derive(Resource, Debug)]
pub struct GeneralSettings {
    pub depth: usize,
    pub max_depth: MaxDepth,
    pub lod_level: usize,
    pub voxels_per_axis: usize,
    pub voxel_size: f32,
    pub shape: Shape,
}

#[derive(Resource, Debug)]
pub struct RenderSettings {
    pub draw_grid: bool,
    pub draw_axes: bool,
    pub draw_aabb: bool,
    pub draw_wireframe: bool,
}

#[derive(Resource, Debug, Default)]
pub struct ClearFacesMask {
    pub yz_pos: bool,
    pub yz_neg: bool,
    pub xz_pos: bool,
    pub xz_neg: bool,
    pub xy_pos: bool,
    pub xy_neg: bool,
}

#[derive(Resource, Debug, Default)]
pub struct GreedyMeshSettings {
    pub clear_faces: ClearFacesMask,
}

#[derive(Resource, Debug)]
pub struct SphereParams {
    pub center: IVec3,
    pub radius: i32,
    pub value: i32,
}

#[derive(Resource, Debug)]
pub struct CheckerboardParams {
    pub size: i32,
    pub value1: i32,
    pub value2: i32,
}

#[derive(Resource, Debug)]
pub struct SparseFillParams {
    pub size: i32,
    pub value: i32,
}

#[derive(Resource, Debug)]
pub struct HollowCubeParams {
    pub size: i32,
    pub thickness: i32,
    pub value: i32,
}

#[derive(Resource, Debug)]
pub struct DiagonalParams {
    pub size: i32,
    pub value: i32,
}

#[derive(Resource, Debug)]
pub struct TerrainParams {
    pub offset: Vec3,
    pub scale: f32,
}

#[derive(Resource, Debug)]
pub struct Perlin3DParams {
    pub offset: Vec3,
    pub scale: f32,
    pub threshold: f32,
}

#[derive(Resource, Default, Debug)]
pub struct WorldSettings {
    pub general: GeneralSettings,
    pub render: RenderSettings,
    pub greedy_mesh: GreedyMeshSettings,

    pub sphere_params: SphereParams,
    pub checkerboard_params: CheckerboardParams,
    pub sparse_fill_params: SparseFillParams,
    pub hollow_cube_params: HollowCubeParams,
    pub diagonal_params: DiagonalParams,
    pub terrain_params: TerrainParams,
    pub perlin_params: Perlin3DParams,
}

impl Default for GeneralSettings {
    fn default() -> Self {
        let max_depth = MaxDepth::new(5);
        let voxels_per_axis = 1 << max_depth.as_usize();

        Self {
            depth: max_depth.as_usize(),
            max_depth,
            lod_level: 0,
            voxels_per_axis,
            voxel_size: CHUNK_SIZE / voxels_per_axis as f32,
            shape: Shape::default(),
        }
    }
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            draw_grid: true,
            draw_axes: true,
            draw_aabb: true,
            draw_wireframe: false,
        }
    }
}

impl Default for SphereParams {
    fn default() -> Self {
        Self {
            center: IVec3::ZERO,
            radius: 1,
            value: 1,
        }
    }
}

impl Default for CheckerboardParams {
    fn default() -> Self {
        Self {
            size: 8,
            value1: 1,
            value2: 0,
        }
    }
}

impl Default for SparseFillParams {
    fn default() -> Self {
        Self { size: 8, value: 1 }
    }
}

impl Default for HollowCubeParams {
    fn default() -> Self {
        Self {
            size: 8,
            thickness: 1,
            value: 1,
        }
    }
}

impl Default for DiagonalParams {
    fn default() -> Self {
        Self { size: 8, value: 1 }
    }
}

impl Default for TerrainParams {
    fn default() -> Self {
        Self {
            offset: Vec3::ZERO,
            scale: 26.0,
        }
    }
}

impl Default for Perlin3DParams {
    fn default() -> Self {
        Self {
            offset: Vec3::ZERO,
            scale: 183.0,
            threshold: 0.51,
        }
    }
}

fn generate_uniform(_settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_uniform");

    let now = std::time::Instant::now();
    world.chunk.fill(&mut world.interner, 1);
    let elapsed = now.elapsed();

    println!("[UNIFORM] Uniform generation took: {elapsed:.2?}");
}

fn generate_sphere(settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_sphere");

    let voxels_per_axis = settings.general.voxels_per_axis;
    let half_size = voxels_per_axis as i32 / 2;
    let center = IVec3::new(half_size, half_size, half_size);
    let radius = (voxels_per_axis / 2) as i32;
    let value = 1;

    let now = std::time::Instant::now();
    let batch = shapes::generate_sphere(&world.chunk, center, radius, value);
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[SPHERE] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_checkerboard(_settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_checkerboard");

    let now = std::time::Instant::now();
    let batch = shapes::generate_checkerboard(&world.chunk);
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[CHECKERBOARD] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_sparse_fill(_settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_sparse_fill");

    let now = std::time::Instant::now();
    let batch = shapes::generate_sparse_fill(&world.chunk);
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[SPARSE_FILL] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_hollow_cube(_settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_hollow_cube");

    let now = std::time::Instant::now();
    let batch = shapes::generate_hollow_cube(&world.chunk);
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[HOLLOW_CUBE] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_diagonal(_settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_diagonal");

    let now = std::time::Instant::now();
    let batch = shapes::generate_diagonal(&world.chunk);
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[DIAGONAL] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_terrain_surface(settings: &WorldSettings, world: &mut World, surface_only: bool) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_terrain_surface");

    let now = std::time::Instant::now();
    let batch = shapes::generate_terrain(
        &world.chunk,
        settings.general.voxel_size,
        settings.terrain_params.scale,
        settings.terrain_params.offset,
        surface_only,
    );
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[TERRAIN] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_perlin_3d(settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_perlin_3d");

    let now = std::time::Instant::now();
    let batch = shapes::generate_perlin_3d(
        &world.chunk,
        settings.general.voxel_size,
        settings.perlin_params.scale,
        settings.perlin_params.offset,
        settings.perlin_params.threshold,
    );
    let elapsed_batch = now.elapsed();

    world.chunk.clear(&mut world.interner);
    let now = std::time::Instant::now();
    world.chunk.apply_batch(&mut world.interner, &batch);
    let elapsed_apply = now.elapsed();

    println!(
        "[PERLIN_3D] Batch generation took: {elapsed_batch:.2?}, Apply took: {elapsed_apply:.2?}"
    );
}

fn generate_shape(settings: &WorldSettings, world: &mut World) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_shape");

    match settings.general.shape {
        Shape::Uniform => generate_uniform(settings, world),
        Shape::Sphere => generate_sphere(settings, world),
        Shape::Checkerboard => generate_checkerboard(settings, world),
        Shape::SparseFill => generate_sparse_fill(settings, world),
        Shape::HollowCube => generate_hollow_cube(settings, world),
        Shape::Diagonal => generate_diagonal(settings, world),
        Shape::TerrainSurface => generate_terrain_surface(settings, world, true),
        Shape::TerrainFull => generate_terrain_surface(settings, world, false),
        Shape::Perlin3D => generate_perlin_3d(settings, world),
    }
}

impl World {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("World::new");

        let interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 256);

        let settings = WorldSettings::default();

        let chunk = VoxChunk::with_position(CHUNK_SIZE, settings.general.max_depth, 0, 0, 0);
        let batch = Batch::<i32>::new(settings.general.max_depth);

        let mut world = Self {
            interner,
            chunk,
            normal_mesh: Handle::default(),
            greedy_mesh: Handle::default(),
            normal_mesh_data: MeshData::default(),
            greedy_mesh_data: MeshData::default(),
            batch,
            normal_stats: MeshStats::default(),
            greedy_stats: MeshStats::default(),
            #[cfg(feature = "trace_greedy_timings")]
            timings: GreedyTimings::default(),
        };

        generate_shape(&WorldSettings::default(), &mut world);

        world
    }

    pub fn regenerate_chunks(&mut self, settings: &WorldSettings) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("World::regenerate_chunks");

        self.chunk.clear(&mut self.interner);
        self.chunk = VoxChunk::with_position(CHUNK_SIZE, settings.general.max_depth, 0, 0, 0);
    }

    pub fn generate_mesh(&mut self, meshes: &mut ResMut<Assets<Mesh>>, lod: Lod, first_time: bool) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("World::generate_mesh");

        let offset = Vec3::new(0.0, 0.0, 0.0);

        self.normal_mesh_data.clear();

        let now = std::time::Instant::now();
        self.chunk.generate_naive_mesh_arrays(
            &self.interner,
            &mut self.normal_mesh_data,
            offset,
            lod,
        );
        self.normal_stats.time = now.elapsed();

        println!(
            "[NORMAL] Mesh generation took: {:.2?}",
            self.normal_stats.time
        );

        self.normal_stats.vertices = self.normal_mesh_data.vertices.len();
        self.normal_stats.normals = self.normal_mesh_data.normals.len();
        self.normal_stats.indices = self.normal_mesh_data.indices.len();

        println!(
            "[NORMAL] vertices: {} normals: {} indices: {}",
            humanize_bytes::humanize_quantity!(self.normal_mesh_data.vertices.len()),
            humanize_bytes::humanize_quantity!(self.normal_mesh_data.normals.len()),
            humanize_bytes::humanize_quantity!(self.normal_mesh_data.indices.len())
        );

        let normal_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_indices(Indices::U32(self.normal_mesh_data.indices.clone()))
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            self.normal_mesh_data.vertices.clone(),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_NORMAL,
            self.normal_mesh_data.normals.clone(),
        );

        self.normal_stats.triangles = normal_mesh.triangles().unwrap().count();

        if !first_time {
            meshes.insert(&mut self.normal_mesh, normal_mesh);
        } else {
            self.normal_mesh = meshes.add(normal_mesh);
        }
    }

    pub fn generate_greedy_mesh(
        &mut self,
        settings: &WorldSettings,
        meshes: &mut ResMut<Assets<Mesh>>,
        lod: Lod,
        first_time: bool,
    ) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("World::generate_greedy_mesh");

        #[cfg(feature = "trace_greedy_timings")]
        {
            self.timings.reset();
        }

        self.greedy_mesh_data.clear();

        let offset = Vec3::new(0.0, 0.0, 0.0);
        let external = [
            settings.greedy_mesh.clear_faces.yz_pos,
            settings.greedy_mesh.clear_faces.yz_neg,
            settings.greedy_mesh.clear_faces.xz_pos,
            settings.greedy_mesh.clear_faces.xz_neg,
            settings.greedy_mesh.clear_faces.xy_pos,
            settings.greedy_mesh.clear_faces.xy_neg,
        ];

        let now = std::time::Instant::now();

        chunk_generate_greedy_mesh_arrays_ext(
            &self.chunk,
            &self.interner,
            &mut self.greedy_mesh_data,
            offset,
            lod,
            external,
            #[cfg(feature = "trace_greedy_timings")]
            &mut self.timings,
        );
        self.greedy_stats.time = now.elapsed();

        println!(
            "[GREEDY] Mesh generation took: {:.2?}",
            self.greedy_stats.time
        );

        self.greedy_stats.vertices = self.greedy_mesh_data.vertices.len();
        self.greedy_stats.normals = self.greedy_mesh_data.normals.len();
        self.greedy_stats.indices = self.greedy_mesh_data.indices.len();

        println!(
            "[GREEDY] vertices: {} normals: {} indices: {}",
            humanize_bytes::humanize_quantity!(self.greedy_mesh_data.vertices.len()),
            humanize_bytes::humanize_quantity!(self.greedy_mesh_data.normals.len()),
            humanize_bytes::humanize_quantity!(self.greedy_mesh_data.indices.len())
        );

        #[cfg(feature = "trace_greedy_timings")]
        let now = Instant::now();

        let greedy_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_indices(Indices::U32(self.greedy_mesh_data.indices.clone()))
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            self.greedy_mesh_data.vertices.clone(),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_NORMAL,
            self.greedy_mesh_data.normals.clone(),
        );

        #[cfg(feature = "trace_greedy_timings")]
        {
            self.timings.bevy_mesh = now.elapsed();
            self.timings.sum();

            println!("timings: {:?}", self.timings);
        }

        self.greedy_stats.triangles = greedy_mesh.triangles().unwrap().count();

        if !first_time {
            meshes.insert(&mut self.greedy_mesh, greedy_mesh);
        } else {
            self.greedy_mesh = meshes.add(greedy_mesh);
        }
    }
}

fn vec3_editor(ui: &mut egui::Ui, label: &str, vec: &mut Vec3) -> bool {
    let mut changed = false;
    ui.group(|ui| {
        ui.label(label);
        ui.horizontal(|ui| {
            changed |= ui
                .add(egui::DragValue::new(&mut vec.x).prefix("X: ").speed(0.1))
                .changed();
            changed |= ui
                .add(egui::DragValue::new(&mut vec.y).prefix("Y: ").speed(0.1))
                .changed();
            changed |= ui
                .add(egui::DragValue::new(&mut vec.z).prefix("Z: ").speed(0.1))
                .changed();
        });
    });

    changed
}

fn world_ui(
    mut contexts: EguiContexts,
    mut settings: ResMut<WorldSettings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut world: ResMut<World>,
) -> Result {
    egui::Window::new("World Settings").show(contexts.ctx_mut()?, |ui| {
        ui.vertical_centered(|ui| {
            ui.heading("Mesh Statistics");
        });

        ui.columns(3, |columns| {
            let normal_tris = world.normal_stats.triangles as f32;
            let greedy_tris = (world.greedy_stats.triangles as f32).max(1.0);
            let reduction = (normal_tris / greedy_tris) * 100.0;
            let color = if reduction < 100.0 {
                egui::Color32::RED
            } else if reduction > 100.0 {
                egui::Color32::GREEN
            } else {
                egui::Color32::YELLOW
            };

            columns[0].label("");
            columns[0].label("Vertices");
            columns[0].label("Triangles");
            columns[0].label("Time");
            columns[0].separator();
            columns[0].label("Reduction");

            columns[1].label("Normal");
            columns[1].label(format_number(world.normal_stats.vertices as f64, 1));
            columns[1].label(format_number(world.normal_stats.triangles as f64, 1));
            columns[1].label(format!("{:.2?}", world.normal_stats.time));
            columns[1].separator();
            columns[1].label("");

            columns[2].label("Greedy");
            columns[2].label(format_number(world.greedy_stats.vertices as f64, 1));
            columns[2].label(format_number(world.greedy_stats.triangles as f64, 1));
            columns[2].label(format!("{:.2?}", world.greedy_stats.time));
            columns[2].separator();
            columns[2].colored_label(color, format!("{reduction:.1}%"));
        });

        ui.separator();

        #[cfg(feature = "trace_greedy_timings")]
        ui.collapsing("Detailed Timings", |ui| {
            ui.columns(2, |columns| {
                columns[0].label("Builder Default");
                columns[1].label(format!("{:.1?}", world.timings.builder_default));
                columns[0].label("Filling External");
                columns[1].label(format!("{:.1?}", world.timings.filling_external));
                columns[0].label("Generate Occupancy Masks");
                columns[1].label(format!("{:.1?}", world.timings.generate_occupancy_masks));
                columns[0].label("Build Occupancy Masks");
                columns[1].label(format!("{:.1?}", world.timings.build_occupancy_masks));
                columns[0].label("Enclosed");
                columns[1].label(format!("{:.1?}", world.timings.enclosed));
                columns[0].label("Prep");
                columns[1].label(format!("{:.1?}", world.timings.prep));
                columns[0].label("Phase 1");
                columns[1].label(format!("{:.1?}", world.timings.phase_1));
                columns[0].label("Phase 2");
                columns[1].label(format!("{:.1?}", world.timings.phase_2));
                columns[0].label("Phase 3");
                columns[1].label(format!("{:.1?}", world.timings.phase_3));
                columns[0].label("Bevy Mesh");
                columns[1].label(format!("{:.1?}", world.timings.bevy_mesh));
                columns[0].label("Total");
                columns[1].label(format!("{:.1?}", world.timings.total));
            });
        });

        let mut shape_changed = false;
        let mut mesh_changed = false;

        ui.collapsing("Render Settings", |ui| {
            ui.columns(4, |columns| {
                columns[0].checkbox(&mut settings.render.draw_axes, "Axes");
                columns[1].checkbox(&mut settings.render.draw_grid, "Grid");
                columns[2].checkbox(&mut settings.render.draw_aabb, "AABB");
                columns[3].checkbox(&mut settings.render.draw_wireframe, "Wireframe");
            });
        });

        ui.collapsing("Greedy Mesh Settings", |ui| {
            ui.columns(3, |columns| {
                mesh_changed |= columns[0]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.yz_pos, "YZ+")
                    .changed();
                mesh_changed |= columns[0]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.yz_neg, "YZ-")
                    .changed();
                mesh_changed |= columns[1]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.xz_pos, "XZ+")
                    .changed();
                mesh_changed |= columns[1]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.xz_neg, "XZ-")
                    .changed();
                mesh_changed |= columns[2]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.xy_pos, "XY+")
                    .changed();
                mesh_changed |= columns[2]
                    .checkbox(&mut settings.greedy_mesh.clear_faces.xy_neg, "XY-")
                    .changed();
            });
        });

        let response = ui.add(
            egui::Slider::new(&mut settings.general.depth, 2..=MAX_DEPTH.as_usize())
                .text("Max Depth"),
        );

        if response.changed() {
            settings.general.max_depth = MaxDepth::new(settings.general.depth as u8);
            settings.general.voxels_per_axis = 1 << settings.general.depth;
            settings.general.voxel_size = CHUNK_SIZE / settings.general.voxels_per_axis as f32;

            world.regenerate_chunks(&settings);
            shape_changed = true;

            println!(
                "Max Depth: {} voxels per axis: {}",
                settings.general.depth, settings.general.voxels_per_axis
            );
        }

        let max_depth = settings.general.max_depth.as_usize();

        let response = ui.add(
            egui::Slider::new(&mut settings.general.lod_level, 0..=max_depth).text("LOD Level"),
        );

        if response.changed() {
            mesh_changed = true;
            println!(
                "LOD Level: {} voxels per axis: {}",
                settings.general.lod_level,
                1 << (MAX_DEPTH.as_usize() - settings.general.lod_level)
            );
        }

        let response = egui::ComboBox::from_label("Shape")
            .selected_text(format!("{:?}", settings.general.shape))
            .show_ui(ui, |ui| {
                let mut changed = false;
                changed |= ui
                    .selectable_value(&mut settings.general.shape, Shape::Uniform, "Uniform")
                    .changed();
                changed |= ui
                    .selectable_value(&mut settings.general.shape, Shape::Sphere, "Sphere")
                    .changed();
                changed |= ui
                    .selectable_value(
                        &mut settings.general.shape,
                        Shape::Checkerboard,
                        "Checkerboard",
                    )
                    .changed();
                changed |= ui
                    .selectable_value(
                        &mut settings.general.shape,
                        Shape::SparseFill,
                        "Sparse Fill",
                    )
                    .changed();
                changed |= ui
                    .selectable_value(
                        &mut settings.general.shape,
                        Shape::HollowCube,
                        "Hollow Cube",
                    )
                    .changed();
                changed |= ui
                    .selectable_value(&mut settings.general.shape, Shape::Diagonal, "Diagonal")
                    .changed();
                changed |= ui
                    .selectable_value(
                        &mut settings.general.shape,
                        Shape::TerrainSurface,
                        "Terrain Surface",
                    )
                    .changed();
                changed |= ui
                    .selectable_value(
                        &mut settings.general.shape,
                        Shape::TerrainFull,
                        "Terrain Full",
                    )
                    .changed();
                changed |= ui
                    .selectable_value(&mut settings.general.shape, Shape::Perlin3D, "Perlin 3D")
                    .changed();

                changed
            });

        shape_changed |= response.inner.unwrap_or_default();

        match settings.general.shape {
            Shape::TerrainSurface | Shape::TerrainFull => {
                if ui
                    .add(
                        egui::Slider::new(&mut settings.terrain_params.scale, 0.01..=256.0)
                            .text("Scale"),
                    )
                    .changed()
                {
                    shape_changed = true;
                }
            }
            Shape::Perlin3D => {
                shape_changed |= vec3_editor(ui, "Offset", &mut settings.perlin_params.offset);
                shape_changed |= ui
                    .add(
                        egui::Slider::new(&mut settings.perlin_params.scale, 0.01..=256.0)
                            .text("Scale"),
                    )
                    .changed();
                shape_changed |= ui
                    .add(
                        egui::Slider::new(&mut settings.perlin_params.threshold, 0.001..=1.0)
                            .text("Threshold"),
                    )
                    .changed();
            }
            _ => {}
        }

        ui.separator();

        let mut params_changed = false;

        ui.columns(3, |columns| {
            shape_changed |= columns[0].button("Generate Shape").clicked();
            mesh_changed |= columns[1].button("Generate Mesh").clicked();
            params_changed |= columns[2].button("Reset Params").clicked();
        });

        if params_changed {
            match settings.general.shape {
                Shape::Uniform => {}
                Shape::Sphere => {
                    settings.sphere_params = SphereParams::default();
                }
                Shape::Checkerboard => {
                    settings.checkerboard_params = CheckerboardParams::default();
                }
                Shape::SparseFill => {
                    settings.sparse_fill_params = SparseFillParams::default();
                }
                Shape::HollowCube => {
                    settings.hollow_cube_params = HollowCubeParams::default();
                }
                Shape::Diagonal => {
                    settings.diagonal_params = DiagonalParams::default();
                }
                Shape::TerrainSurface | Shape::TerrainFull => {
                    settings.terrain_params = TerrainParams::default();
                }
                Shape::Perlin3D => {
                    settings.perlin_params = Perlin3DParams::default();
                }
            }

            shape_changed = true;
        }

        if shape_changed {
            println!("Shape changed to: {:?}", settings.general.shape);
            generate_shape(&settings, &mut world);
            mesh_changed = true;
        }

        if mesh_changed {
            let lod = Lod::new(settings.general.lod_level as u8);
            world.generate_mesh(&mut meshes, lod, false);
            world.generate_greedy_mesh(&settings, &mut meshes, lod, false);
        }
    });

    Ok(())
}

fn draw_axes(mut gizmos: Gizmos, settings: Res<WorldSettings>) {
    if settings.render.draw_axes {
        gizmos.axes(Transform::from_xyz(0.0, 0.01, 0.0), 2.0 * CHUNK_SIZE);
    }
    if settings.render.draw_grid {
        gizmos
            .grid(
                Quat::from_rotation_x(PI / 2.),
                UVec2::splat(10),
                Vec2::new(CHUNK_SIZE, CHUNK_SIZE),
                // Light gray
                LinearRgba::gray(0.65),
            )
            .outer_edges();
    }
    if settings.render.draw_aabb {
        gizmos.cuboid(
            Transform::from_xyz(CHUNK_SIZE / 2.0, CHUNK_SIZE / 2.0, CHUNK_SIZE / 2.0)
                .with_scale(Vec3::splat(CHUNK_SIZE)),
            LinearRgba::WHITE,
        );
        gizmos.cuboid(
            Transform::from_xyz(
                -(2.0 * CHUNK_SIZE) + CHUNK_SIZE / 2.0,
                CHUNK_SIZE / 2.0,
                CHUNK_SIZE / 2.0,
            )
            .with_scale(Vec3::splat(CHUNK_SIZE)),
            LinearRgba::WHITE,
        );
    }
}

fn setup_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    settings: Res<WorldSettings>,
    mut world: ResMut<World>,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("setup_world");

    let mut cascade_shadow_config_builder = CascadeShadowConfigBuilder::default();
    cascade_shadow_config_builder.first_cascade_far_bound = 50.0;
    cascade_shadow_config_builder.minimum_distance = 0.1;
    cascade_shadow_config_builder.maximum_distance = 100_000.0;

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
            Transform::from_xyz(2.2716377, 1.2876732, 3.9676127)
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

    let material = materials.add(StandardMaterial {
        base_color: Color::from(bevy::color::palettes::tailwind::AMBER_700),
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        thickness: 0.1,
        ior: 1.46,
        ..default()
    });

    world.generate_mesh(&mut meshes, Lod::new(0), true);
    world.generate_greedy_mesh(&settings, &mut meshes, Lod::new(0), true);

    let normal_pos = Vec3::new(-2.56, 0.0, 0.0);
    let greedy_pos = Vec3::new(0.0, 0.0, 0.0);

    commands
        .spawn((
            Mesh3d(world.normal_mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(normal_pos),
        ))
        .insert(Name::new("Normal Chunk".to_string()));
    commands
        .spawn((
            Mesh3d(world.greedy_mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(greedy_pos),
        ))
        .insert(Name::new("Greedy Chunk".to_string()));
}

fn toggle_wireframe(
    mut wireframe_config: ResMut<WireframeConfig>,
    mut settings: ResMut<WorldSettings>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        settings.render.draw_wireframe = !settings.render.draw_wireframe;
    }

    if wireframe_config.global != settings.render.draw_wireframe {
        wireframe_config.global = settings.render.draw_wireframe;
    }
}

fn tracy_mark_frame() {
    #[cfg(feature = "tracy")]
    tracy_client::frame_mark();
}

fn main() {
    #[cfg(feature = "tracy")]
    tracy_client::Client::start();
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("main");

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Greedy Meshing".to_string(),
                    present_mode: PresentMode::AutoNoVsync,
                    ..default()
                }),
                ..default()
            }),
            TemporalAntiAliasPlugin,
            EguiPlugin::default(),
            PanOrbitCameraPlugin,
            WireframePlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
            ScreenDiagnosticsPlugin::default(),
            ScreenFrameDiagnosticsPlugin,
            ScreenEntityDiagnosticsPlugin,
        ))
        .insert_resource(ClearColor(Color::from(
            bevy::color::palettes::tailwind::BLUE_300,
        )))
        .insert_resource(DirectionalLightShadowMap { size: 8192 })
        .insert_resource(World::new())
        .init_resource::<WorldSettings>()
        .add_systems(Startup, setup_world)
        .add_systems(Update, (toggle_wireframe, draw_axes))
        .add_systems(EguiPrimaryContextPass, world_ui)
        .add_systems(Last, tracy_mark_frame)
        .run();
}
