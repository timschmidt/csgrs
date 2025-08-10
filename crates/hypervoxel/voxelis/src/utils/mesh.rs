use std::collections::HashMap;

#[cfg(feature = "trace_greedy_timings")]
use std::time::{Duration, Instant};

use glam::{IVec3, UVec2, UVec3, Vec3};

use crate::{
    BlockId, Lod, MaxDepth, TraversalDepth, VoxInterner, VoxelTrait,
    spatial::{VoxOpsChunkConfig, VoxOpsChunkLocalContainer, VoxOpsConfig, VoxOpsState},
    utils::common::get_at_depth,
    world::VoxChunk,
};

pub const CUBE_VERTS: [Vec3; 8] = [
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(1.0, 1.0, 0.0),
    Vec3::new(1.0, 1.0, 1.0),
    Vec3::new(0.0, 1.0, 1.0),
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 1.0),
    Vec3::new(0.0, 0.0, 1.0),
];

pub const VEC_RIGHT: Vec3 = Vec3::X;
pub const VEC_LEFT: Vec3 = Vec3::NEG_X;
pub const VEC_UP: Vec3 = Vec3::Y;
pub const VEC_DOWN: Vec3 = Vec3::NEG_Y;
pub const VEC_FORWARD: Vec3 = Vec3::NEG_Z;
pub const VEC_BACK: Vec3 = Vec3::Z;
pub const CUBE_NORMALS: [Vec3; 6] = [VEC_UP, VEC_RIGHT, VEC_DOWN, VEC_LEFT, VEC_FORWARD, VEC_BACK];

pub const VERTS_YZ_POS: [usize; 4] = [2, 5, 6, 1];
pub const VERTS_YZ_NEG: [usize; 4] = [0, 7, 4, 3];
pub const VERTS_XZ_POS: [usize; 4] = [0, 2, 3, 1];
pub const VERTS_XZ_NEG: [usize; 4] = [7, 5, 4, 6];
pub const VERTS_XY_POS: [usize; 4] = [3, 6, 7, 2];
pub const VERTS_XY_NEG: [usize; 4] = [1, 4, 5, 0];
pub const IJK_YZ: [usize; 3] = [2, 0, 1];
pub const IJK_XZ: [usize; 3] = [0, 2, 1];
pub const IJK_XY: [usize; 3] = [0, 1, 2];
pub const NORMAL_YZ_POS: usize = 1;
pub const NORMAL_YZ_NEG: usize = 3;
pub const NORMAL_XZ_POS: usize = 0;
pub const NORMAL_XZ_NEG: usize = 2;
pub const NORMAL_XY_POS: usize = 5;
pub const NORMAL_XY_NEG: usize = 4;

pub const MAX_VOXELS_PER_AXIS: usize = 64;
pub const PLANE_SIZE: usize = MAX_VOXELS_PER_AXIS * MAX_VOXELS_PER_AXIS;
pub const PLANE_SIZE_ALL_AXES: usize = PLANE_SIZE * 3;
pub const VOLUME_SIZE: usize = PLANE_SIZE * MAX_VOXELS_PER_AXIS;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ExternalPlane {
    YZPos,
    YZNeg,
    XZPos,
    XZNeg,
    XYPos,
    XYNeg,
}

pub const PLANE_YZ_OFFSET: usize = 0;
pub const PLANE_XZ_OFFSET: usize = PLANE_SIZE;
pub const PLANE_XY_OFFSET: usize = PLANE_SIZE * 2;

struct PlaneData {
    pub plane: Plane,
    pub offset: usize,
    pub pos: ExternalPlane,
    pub neg: ExternalPlane,
}

const PLANES: [PlaneData; 3] = [
    PlaneData {
        plane: Plane::YZ,
        offset: PLANE_YZ_OFFSET,
        pos: ExternalPlane::YZPos,
        neg: ExternalPlane::YZNeg,
    },
    PlaneData {
        plane: Plane::XZ,
        offset: PLANE_XZ_OFFSET,
        pos: ExternalPlane::XZPos,
        neg: ExternalPlane::XZNeg,
    },
    PlaneData {
        plane: Plane::XY,
        offset: PLANE_XY_OFFSET,
        pos: ExternalPlane::XYPos,
        neg: ExternalPlane::XYNeg,
    },
];

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Dir {
    Pos,
    Neg,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Plane {
    XY,
    XZ,
    YZ,
}

struct DirectionData<'a> {
    dir: Dir,
    masks: &'a [u64],
    total: usize,
    active_row: u64,
    active_col: u64,
    active_depth: u64,
    active_row_min: usize,
    active_row_max: usize,
    active_col_min: usize,
    active_col_max: usize,
    active_depth_min: usize,
    active_depth_max: usize,
}

#[derive(Default)]
pub struct MeshData {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub indices: Vec<u32>,
}

#[cfg(feature = "trace_greedy_timings")]
#[derive(Default, Debug)]
pub struct GreedyTimings {
    pub builder_default: Duration,
    pub filling_external: Duration,
    pub generate_occupancy_masks: Duration,
    pub build_occupancy_masks: Duration,
    pub enclosed: Duration,
    pub prep: Duration,
    pub phase_1: Duration,
    pub phase_2: Duration,
    pub phase_3: Duration,
    pub bevy_mesh: Duration,
    pub total_mesh_arrays: Duration,
    pub total: Duration,
}

pub type ExternalOccupancyMasks = [u64; MAX_VOXELS_PER_AXIS];

pub struct OccupancyData {
    pub global: Vec<u64>,
    pub external: [ExternalOccupancyMasks; 6],
    pub external_exists: [bool; 6],
    pub per_material: Vec<Vec<u64>>,
    pub materials: Vec<(usize, usize)>,
}

pub struct OccupancyDataBuilder {
    /// Global occupancy masks for all axes.
    pub global: Vec<u64>,
    /// External occupancy masks for each axis - front, back, top, bottom, left, right.
    pub external: [ExternalOccupancyMasks; 6],
    /// Flags indicating if there are any voxels on the external sides.
    pub external_exists: [bool; 6],
    /// Per material occupancy masks for each axis.
    pub per_material: HashMap<usize, Vec<u64>>,
    /// Materials with their counts.
    pub materials: HashMap<usize, usize>,
}

impl MeshData {
    pub fn clear(&mut self) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("MeshData::clear");

        self.vertices.clear();
        self.normals.clear();
        self.indices.clear();
    }
}

#[cfg(feature = "trace_greedy_timings")]
impl GreedyTimings {
    pub fn reset(&mut self) {
        self.builder_default = Duration::ZERO;
        self.filling_external = Duration::ZERO;
        self.generate_occupancy_masks = Duration::ZERO;
        self.build_occupancy_masks = Duration::ZERO;
        self.enclosed = Duration::ZERO;
        self.prep = Duration::ZERO;
        self.phase_1 = Duration::ZERO;
        self.phase_2 = Duration::ZERO;
        self.phase_3 = Duration::ZERO;
        self.bevy_mesh = Duration::ZERO;
        self.total_mesh_arrays = Duration::ZERO;
        self.total = Duration::ZERO;
    }

    pub fn sum(&mut self) {
        self.total = self.builder_default
            + self.filling_external
            + self.generate_occupancy_masks
            + self.build_occupancy_masks
            + self.enclosed
            + self.prep
            + self.phase_1
            + self.phase_2
            + self.phase_3
            + self.bevy_mesh
            + self.total_mesh_arrays;
    }
}

impl Default for OccupancyDataBuilder {
    fn default() -> Self {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("OccupancyDataBuilder::default");

        Self {
            global: vec![0u64; PLANE_SIZE_ALL_AXES],
            external: [const { [0u64; MAX_VOXELS_PER_AXIS] }; 6],
            external_exists: [false; 6],
            per_material: HashMap::new(),
            materials: HashMap::new(),
        }
    }
}

impl OccupancyDataBuilder {
    pub fn build(mut self) -> OccupancyData {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("OccupancyDataBuilder::build");

        let mut materials = self.materials.into_iter().collect::<Vec<_>>();
        materials.sort_by(|a, b| a.0.cmp(&b.0));

        let mut per_material = Vec::with_capacity(materials.len());

        for (material_id, _) in &materials {
            per_material.push(self.per_material.remove(material_id).unwrap());
        }

        OccupancyData {
            global: self.global,
            external: self.external,
            external_exists: self.external_exists,
            per_material,
            materials,
        }
    }

    pub fn fill_external_side(&mut self, external_plane: ExternalPlane) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("OccupancyDataBuilder::fill_external_side");

        self.external[external_plane as usize].fill(u64::MAX);
    }

    pub fn clear_external_side(&mut self, external_plane: ExternalPlane) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("OccupancyDataBuilder::clear_external_side");

        self.external[external_plane as usize].fill(0);
    }
}

#[inline(always)]
pub const fn extract_plane_dir(external_plane: ExternalPlane) -> (Plane, Dir) {
    match external_plane {
        ExternalPlane::YZPos => (Plane::YZ, Dir::Pos),
        ExternalPlane::YZNeg => (Plane::YZ, Dir::Neg),
        ExternalPlane::XZPos => (Plane::XZ, Dir::Pos),
        ExternalPlane::XZNeg => (Plane::XZ, Dir::Neg),
        ExternalPlane::XYPos => (Plane::XY, Dir::Pos),
        ExternalPlane::XYNeg => (Plane::XY, Dir::Neg),
    }
}

pub fn generate_external_occupancy_mask<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    builder: &mut OccupancyDataBuilder,
    root_id: &BlockId,
    max_depth: MaxDepth,
    external_plane: ExternalPlane,
    offset: UVec2,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_external_occupancy_mask");

    let max_depth = max_depth.max();
    let voxels_per_axis = 1 << max_depth;

    let start_x = offset.x as usize;
    let start_y = offset.y as usize;

    let (plane, dir) = extract_plane_dir(external_plane);

    let external_plane = &mut builder.external[external_plane as usize];

    let depth = TraversalDepth::new(0, max_depth);

    let pos_vox = match dir {
        Dir::Pos => 0,
        Dir::Neg => (voxels_per_axis - 1) as i32,
    };

    if !root_id.is_empty() {
        if root_id.is_branch() {
            let mut pos = IVec3::default();

            match plane {
                Plane::YZ => {
                    pos.x = pos_vox;

                    for y in 0..voxels_per_axis {
                        pos.z = y as i32;
                        let mask_y = start_y + y;

                        for z in 0..voxels_per_axis {
                            pos.y = z as i32;

                            if get_at_depth(interner, *root_id, &pos, &depth).is_some() {
                                external_plane[mask_y] |= 1u64 << (start_x + z);
                            }
                        }
                    }
                }
                Plane::XZ => {
                    pos.y = pos_vox;

                    for z in 0..voxels_per_axis {
                        pos.z = z as i32;
                        let mask_z = start_y + z;

                        for x in 0..voxels_per_axis {
                            pos.x = x as i32;

                            if get_at_depth(interner, *root_id, &pos, &depth).is_some() {
                                external_plane[mask_z] |= 1u64 << (start_x + x);
                            }
                        }
                    }
                }
                Plane::XY => {
                    pos.z = pos_vox;

                    for y in 0..voxels_per_axis {
                        pos.y = y as i32;
                        let mask_y = start_y + y;

                        for x in 0..voxels_per_axis {
                            pos.x = x as i32;

                            if get_at_depth(interner, *root_id, &pos, &depth).is_some() {
                                external_plane[mask_y] |= 1u64 << (start_x + x);
                            }
                        }
                    }
                }
            }
        } else {
            match plane {
                Plane::YZ => {
                    let bit_mask = ((1u64 << voxels_per_axis) - 1) << start_x;
                    for y in 0..voxels_per_axis {
                        let mask_y = start_y + y;
                        external_plane[mask_y] |= bit_mask;
                    }
                }
                Plane::XZ => {
                    let bit_mask = ((1u64 << voxels_per_axis) - 1) << start_x;
                    for z in 0..voxels_per_axis {
                        let mask_z = start_y + z;
                        external_plane[mask_z] |= bit_mask;
                    }
                }
                Plane::XY => {
                    let bit_mask = ((1u64 << voxels_per_axis) - 1) << start_x;
                    for y in 0..voxels_per_axis {
                        let mask_y = start_y + y;
                        external_plane[mask_y] |= bit_mask;
                    }
                }
            }
        }
    }
}

fn fill_masks_for_region(
    builder: &mut OccupancyDataBuilder,
    region_offset: UVec3,
    side: u32,
    material_id: usize,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("fill_masks_for_region");

    let side = side as usize;
    let volume = side * side * side;

    builder
        .materials
        .entry(material_id)
        .and_modify(|count| *count += volume)
        .or_insert(volume);

    if side != MAX_VOXELS_PER_AXIS {
        let occupancy_per_axis = builder
            .per_material
            .entry(material_id)
            .or_insert_with(|| vec![0; PLANE_SIZE_ALL_AXES]);

        let run_mask = (1u64 << side) - 1;

        let start_x = region_offset.x as usize;
        let x_mask = run_mask << start_x;

        let start_y = region_offset.y as usize;
        let y_mask = run_mask << start_y;

        let start_z = region_offset.z as usize;
        let z_mask = run_mask << start_z;

        for i in 0..side {
            let z = start_z + i;
            let y = start_y + i;

            let index_base_y = PLANE_XZ_OFFSET + z * MAX_VOXELS_PER_AXIS + start_x;
            let index_base_z = PLANE_XY_OFFSET + y * MAX_VOXELS_PER_AXIS + start_x;
            let index_base_x = z * MAX_VOXELS_PER_AXIS + start_y;

            for j in 0..side {
                let index_y = index_base_y + j;
                builder.global[index_y] |= y_mask;
                occupancy_per_axis[index_y] |= y_mask;

                let index_z = index_base_z + j;
                builder.global[index_z] |= z_mask;
                occupancy_per_axis[index_z] |= z_mask;

                let index_x = index_base_x + j;
                builder.global[index_x] |= x_mask;
                occupancy_per_axis[index_x] |= x_mask;
            }
        }
    } else {
        use std::collections::hash_map::Entry;

        match builder.per_material.entry(material_id) {
            Entry::Occupied(mut e) => {
                e.get_mut().fill(u64::MAX);
            }
            Entry::Vacant(e) => {
                e.insert(vec![u64::MAX; PLANE_SIZE_ALL_AXES]);
            }
        }

        builder.global.fill(u64::MAX);
    }
}

pub fn generate_occupancy_masks<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    builder: &mut OccupancyDataBuilder,
    root_id: &BlockId,
    max_depth: MaxDepth,
    offset: UVec3,
    #[cfg(feature = "trace_greedy_timings")] timings: &mut GreedyTimings,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_occupancy_masks");

    #[cfg(feature = "trace_greedy_timings")]
    let now = std::time::Instant::now();

    if root_id.is_empty() {
        #[cfg(feature = "trace_greedy_timings")]
        {
            timings.generate_occupancy_masks = now.elapsed();
        }

        return;
    }

    let default_t = T::default();

    let max_depth = max_depth.max() as u32;

    if !root_id.is_branch() {
        let value = *interner.get_value(root_id);
        if value != default_t {
            let material_id = value.material_id();
            let voxels_per_axis = 1 << max_depth;
            fill_masks_for_region(builder, offset, voxels_per_axis, material_id);
        }

        #[cfg(feature = "trace_greedy_timings")]
        {
            timings.generate_occupancy_masks = now.elapsed();
        }

        return;
    }

    let mut stack: Vec<(BlockId, IVec3, u32)> = Vec::with_capacity(64);
    stack.push((*root_id, IVec3::ZERO, 0));

    while let Some((node_id, pos, depth)) = stack.pop() {
        if node_id.is_branch() && (depth < max_depth) {
            let child_cube_half_side = 1 << (max_depth - depth - 1);
            let childs = interner.get_children_ref(&node_id);
            for i in (0..8).rev() {
                let child_id = unsafe { childs.get_unchecked(i) };

                let i = i as i32;

                if !child_id.is_empty() {
                    let x = (i & 1) * child_cube_half_side;
                    let y = ((i & 2) >> 1) * child_cube_half_side;
                    let z = ((i & 4) >> 2) * child_cube_half_side;
                    let offset = IVec3::new(x, y, z);
                    let pos = pos + offset;
                    let depth = depth + 1;

                    stack.push((*child_id, pos, depth));
                }
            }
        } else {
            let value = *interner.get_value(&node_id);
            if value != default_t {
                let cube_side = 1 << (max_depth - depth);
                let global_pos = offset + pos.as_uvec3();
                let material_id = value.material_id();
                fill_masks_for_region(builder, global_pos, cube_side, material_id);
            }
        }
    }

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.generate_occupancy_masks = now.elapsed();
    }
}

// This function generates greedy mesh arrays from occupancy data.
// It can process simultaneosly multiple chunks with same size
// -     1 ( 1x 1x 1) chunks of 64x64x64 voxels
// -     8 ( 2x 2x 2) chunks of 32x32x32 voxels
// -    64 ( 4x 4x 4) chunks of 16x16x16 voxels
// -   512 ( 8x 8x 8) chunks of  8x 8x 8 voxels
// -  4096 (16x16x16) chunks of  4x 4x 4 voxels
// - 32768 (32x32x32) chunks of  2x 2x 2 voxels
pub fn generate_greedy_mesh_arrays(
    occupancy_data: &OccupancyData,
    mesh_data: &mut MeshData,
    max_depth: MaxDepth,
    offset: Vec3,
    voxel_size: f32,
    #[cfg(feature = "trace_greedy_timings")] timings: &mut GreedyTimings,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_greedy_mesh_arrays");

    #[cfg(feature = "trace_greedy_timings")]
    let now = std::time::Instant::now();

    let enclosed = occupancy_data
        .external
        .iter()
        .all(|&exists| exists.iter().all(|&mask| mask == u64::MAX));

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.enclosed = now.elapsed();
    }

    if enclosed {
        return;
    }

    #[cfg(feature = "trace_greedy_timings")]
    let now = std::time::Instant::now();

    let materials_len = occupancy_data.materials.len();

    let max_depth = max_depth.as_usize();
    let max_voxels_per_axis = 1 << max_depth;

    let mut global_face_masks_pos = [const { 0u64 }; PLANE_SIZE];
    let mut global_face_masks_neg = [const { 0u64 }; PLANE_SIZE];
    let mut active_row_pos: u64 = 0;
    let mut active_row_neg: u64 = 0;
    let mut active_col_pos: u64 = 0;
    let mut active_col_neg: u64 = 0;
    let mut active_depth_pos: u64 = 0;
    let mut active_depth_neg: u64 = 0;
    let mut material_face_masks_pos = vec![0; PLANE_SIZE * materials_len];
    let mut material_face_masks_neg = vec![0; PLANE_SIZE * materials_len];
    let mut material_count_opt_pos = vec![0; materials_len];
    let mut material_count_opt_neg = vec![0; materials_len];
    let mut faces = [const { 0u64 }; MAX_VOXELS_PER_AXIS];
    let mut used = [const { 0u64 }; MAX_VOXELS_PER_AXIS];

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.prep = now.elapsed();
    }

    // we iterate over all planes first, to not process global occupancy masks for each material separately
    for plane_data in &PLANES {
        material_count_opt_pos.fill(0);
        material_count_opt_neg.fill(0);

        let external_pos = &occupancy_data.external[plane_data.pos as usize];
        let external_neg = &occupancy_data.external[plane_data.neg as usize];

        #[cfg(feature = "trace_greedy_timings")]
        let now = Instant::now();

        // process global & per material occupancy masks for the plane
        for row in 0..max_voxels_per_axis {
            let base_idx = row * MAX_VOXELS_PER_AXIS;

            for col in 0..max_voxels_per_axis {
                let idx = base_idx + col;

                // process global occupancy masks for the plane
                let mask = occupancy_data.global[plane_data.offset + idx];

                // find voxel boundaries: where occupied meets empty
                let mut global_mask_pos = !(mask >> 1) & mask; // +AXIS faces
                let mut global_mask_neg = !(mask << 1) & mask; // -AXIS faces

                // remove faces adjacent to external neighbors
                global_mask_pos &= !((external_pos[row] >> col & 1) << (max_voxels_per_axis - 1));
                global_mask_neg &= !(external_neg[row] >> col & 1);

                let mask_pos_active = (global_mask_pos != 0) as u64;
                let mask_neg_active = (global_mask_neg != 0) as u64;

                active_row_pos |= mask_pos_active << row;
                active_col_pos |= mask_pos_active << col;
                active_depth_pos |= global_mask_pos;

                active_row_neg |= mask_neg_active << row;
                active_col_neg |= mask_neg_active << col;
                active_depth_neg |= global_mask_neg;

                global_face_masks_pos[idx] = global_mask_pos;
                global_face_masks_neg[idx] = global_mask_neg;

                // process per material occupancy masks for the plane
                for material_idx in 0..materials_len {
                    // take bitmask occupancy for the current material
                    //   for example - material mask: 0 0 0 1 1 1 0 0 <- 3 voxels with material 'X'
                    //            global voxels mask: 0 0 1 1 1 1 0 0 <- 4 voxels in total
                    //               global mask pos: 0 0 1 0 0 0 0 0 <- first face for column
                    //               global mask neg: 0 0 0 0 0 1 0 0 <- last face for column
                    let mask = occupancy_data.per_material[material_idx][plane_data.offset + idx];
                    // select only faces if they are at the same spot as start/end on the global mask
                    // for example - global mask pos: 0 0 1 0 0 0 0 0
                    //                 material mask: 0 0 0 1 1 1 0 0 &
                    //                      pos_mask: 0 0 0 0 0 0 0 0
                    let material_mask_pos = mask & global_mask_pos;
                    // for example - global mask neg: 0 0 0 0 0 1 0 0
                    //                 material mask: 0 0 0 1 1 1 0 0 &
                    //                      neg_mask: 0 0 0 0 0 1 0 0
                    let material_mask_neg = mask & global_mask_neg;

                    // count how many faces we have for the material
                    material_count_opt_pos[material_idx] += material_mask_pos.count_ones() as usize;
                    material_count_opt_neg[material_idx] += material_mask_neg.count_ones() as usize;

                    // put masks into the material face masks
                    let material_idx_offset = idx + material_idx * PLANE_SIZE;

                    material_face_masks_pos[material_idx_offset] = material_mask_pos;
                    material_face_masks_neg[material_idx_offset] = material_mask_neg;
                }
            }
        }

        #[cfg(feature = "trace_greedy_timings")]
        {
            timings.phase_1 += now.elapsed();
        }

        #[cfg(feature = "trace_greedy_timings")]
        let now = Instant::now();

        let min_col_pos = active_col_pos.trailing_zeros() as usize;
        let max_col_pos = (64 - active_col_pos.leading_zeros()) as usize;
        let min_col_neg = active_col_neg.trailing_zeros() as usize;
        let max_col_neg = (64 - active_col_neg.leading_zeros()) as usize;
        let min_row_pos = active_row_pos.trailing_zeros() as usize;
        let max_row_pos = (64 - active_row_pos.leading_zeros()) as usize;
        let min_row_neg = active_row_neg.trailing_zeros() as usize;
        let max_row_neg = (64 - active_row_neg.leading_zeros()) as usize;
        let min_depth_pos = active_depth_pos.trailing_zeros() as usize;
        let max_depth_pos = (64 - active_depth_pos.leading_zeros()) as usize;
        let min_depth_neg = active_depth_neg.trailing_zeros() as usize;
        let max_depth_neg = (64 - active_depth_neg.leading_zeros()) as usize;

        #[cfg(feature = "trace_greedy_timings")]
        {
            timings.phase_2 += now.elapsed();
        }

        #[cfg(feature = "trace_greedy_timings")]
        let now = Instant::now();

        // generate greedy faces per material and direction of the plane
        for material_idx in 0..materials_len {
            let material_faces_start = material_idx * PLANE_SIZE;
            let material_faces_end = material_faces_start + PLANE_SIZE;

            let dirs_data = [
                DirectionData {
                    dir: Dir::Pos,
                    masks: &material_face_masks_pos[material_faces_start..material_faces_end],
                    total: material_count_opt_pos[material_idx],
                    active_row: active_row_pos,
                    active_row_min: min_row_pos,
                    active_row_max: max_row_pos,
                    active_col: active_col_pos,
                    active_col_min: min_col_pos,
                    active_col_max: max_col_pos,
                    active_depth: active_depth_pos,
                    active_depth_min: min_depth_pos,
                    active_depth_max: max_depth_pos,
                },
                DirectionData {
                    dir: Dir::Neg,
                    masks: &material_face_masks_neg[material_faces_start..material_faces_end],
                    total: material_count_opt_neg[material_idx],
                    active_row: active_row_neg,
                    active_row_min: min_row_neg,
                    active_row_max: max_row_neg,
                    active_col: active_col_neg,
                    active_col_min: min_col_neg,
                    active_col_max: max_col_neg,
                    active_depth: active_depth_neg,
                    active_depth_min: min_depth_neg,
                    active_depth_max: max_depth_neg,
                },
            ];

            for dir_data in &dirs_data {
                let active_row = dir_data.active_row;
                let active_row_min = dir_data.active_row_min;
                let active_row_max = dir_data.active_row_max;
                let active_col = dir_data.active_col;
                let active_col_min = dir_data.active_col_min;
                let active_col_max = dir_data.active_col_max;
                let active_depth = dir_data.active_depth;
                let active_depth_min = dir_data.active_depth_min;
                let active_depth_max = dir_data.active_depth_max;

                let mut faces_left = dir_data.total;

                for slice in active_depth_min..active_depth_max {
                    if (active_depth >> slice) & 1 == 0 {
                        continue;
                    }

                    let mut faces_in_slice = 0;

                    if faces_left == 0 {
                        break;
                    }

                    faces.fill(0);
                    used.fill(0);

                    let mut have_faces = false;

                    for row in active_row_min..active_row_max {
                        if (active_row >> row) & 1 == 0 {
                            continue;
                        }

                        let base_idx = row * MAX_VOXELS_PER_AXIS;
                        for col in active_col_min..active_col_max {
                            if (active_col >> col) & 1 == 0 {
                                continue;
                            }

                            let idx = base_idx + col;
                            if (dir_data.masks[idx] >> slice) & 1 != 0 {
                                faces[row] |= 1 << col;
                                faces_left -= 1;
                                faces_in_slice += 1;
                                have_faces = true;
                            }
                        }
                    }

                    if !have_faces {
                        continue;
                    }

                    generate_greedy_faces_for_slice(
                        mesh_data,
                        (plane_data.plane, dir_data.dir, offset),
                        (voxel_size, faces_in_slice, slice as f32),
                        &mut used,
                        &faces,
                        max_voxels_per_axis,
                    );
                }
            }
        }

        #[cfg(feature = "trace_greedy_timings")]
        {
            timings.phase_3 += now.elapsed();
        }
    }
}

pub fn generate_greedy_faces_for_slice(
    mesh_data: &mut MeshData,
    (plane, dir, global_offset): (Plane, Dir, Vec3),
    (voxel_size, faces_total, slice): (f32, usize, f32),
    used: &mut [u64; MAX_VOXELS_PER_AXIS],
    faces: &[u64; MAX_VOXELS_PER_AXIS],
    max_voxels_per_axis: usize,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("generate_greedy_faces_for_slice");

    let mut faces_left = faces_total;

    'main: for start_row in 0..max_voxels_per_axis {
        let mut available = faces[start_row] & !used[start_row];

        while available != 0 {
            let start_col = available.trailing_zeros() as usize;
            let width_mask = find_contiguous_bits(available, start_col);
            let width = width_mask.count_ones() as usize;

            let mut height = 1;

            for row in start_row + 1..max_voxels_per_axis {
                let row_mask = faces[row] & !used[row];
                if (row_mask & width_mask) == width_mask {
                    height += 1;
                    used[row] |= width_mask;
                } else {
                    break;
                }
            }

            let ijk_scale = [
                voxel_size * width as f32,
                voxel_size * height as f32,
                voxel_size,
            ];
            let ijk_offset = [
                voxel_size * start_col as f32,
                voxel_size * start_row as f32,
                voxel_size * slice,
            ];

            let (v_ids, ijk_ids, normal_id) = match (plane, dir) {
                (Plane::YZ, Dir::Pos) => (VERTS_YZ_POS, IJK_YZ, NORMAL_YZ_POS),
                (Plane::YZ, Dir::Neg) => (VERTS_YZ_NEG, IJK_YZ, NORMAL_YZ_NEG),
                (Plane::XZ, Dir::Pos) => (VERTS_XZ_POS, IJK_XZ, NORMAL_XZ_POS),
                (Plane::XZ, Dir::Neg) => (VERTS_XZ_NEG, IJK_XZ, NORMAL_XZ_NEG),
                (Plane::XY, Dir::Pos) => (VERTS_XY_POS, IJK_XY, NORMAL_XY_POS),
                (Plane::XY, Dir::Neg) => (VERTS_XY_NEG, IJK_XY, NORMAL_XY_NEG),
            };

            let scale = Vec3::new(
                ijk_scale[ijk_ids[0]],
                ijk_scale[ijk_ids[1]],
                ijk_scale[ijk_ids[2]],
            );
            let offset = Vec3::new(
                ijk_offset[ijk_ids[0]],
                ijk_offset[ijk_ids[1]],
                ijk_offset[ijk_ids[2]],
            );

            let v0 = CUBE_VERTS[v_ids[0]] * scale + offset + global_offset;
            let v1 = CUBE_VERTS[v_ids[1]] * scale + offset + global_offset;
            let v2 = CUBE_VERTS[v_ids[2]] * scale + offset + global_offset;
            let v3 = CUBE_VERTS[v_ids[3]] * scale + offset + global_offset;

            add_quad(mesh_data, [v0, v1, v2, v3], &CUBE_NORMALS[normal_id]);

            used[start_row] |= width_mask;
            available &= !width_mask;
            faces_left -= width * height;

            if faces_left == 0 {
                break 'main;
            }
        }
    }
}

#[inline(always)]
pub fn add_quad(mesh_data: &mut MeshData, quad: [Vec3; 4], normal: &Vec3) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("add_quad");

    let index = mesh_data.vertices.len() as u32;

    mesh_data.vertices.extend(quad);
    mesh_data.normals.extend([normal, normal, normal, normal]);
    mesh_data
        .indices
        .extend([index + 2, index + 1, index, index + 3, index, index + 1]);
}

#[inline(always)]
pub const fn find_contiguous_bits(mask: u64, start: usize) -> u64 {
    // if the mask is all ones, return it as is
    if mask == u64::MAX {
        return !0u64 << start;
    }

    // find all bits from start to the first gap
    let shifted = mask >> start;
    let inverted = !shifted;
    let first_zero = inverted.trailing_zeros() as u64;

    // mask of bits from start to start+width
    ((1u64 << first_zero) - 1) << start
}

pub fn generate_greedy_mesh_arrays_stride<
    T: VoxelTrait,
    C: VoxOpsChunkLocalContainer<T> + VoxOpsConfig + VoxOpsChunkConfig,
>(
    container: &C,
    store: &VoxInterner<T>,
    lod: Lod,
    mesh_data: &mut MeshData,
) {
    let mut offset = Vec3::new(0.0, 0.0, 0.0);

    let chunk_size = container.chunk_size();
    let voxels_per_axis = container.voxels_per_axis(lod);
    let max_depth = container.max_depth(lod);
    let voxel_size = container.voxel_size(lod);

    let mesh_max_depth = MaxDepth::new(6);

    let stride = (64 / voxels_per_axis) as i32;

    let chunks_size = container.chunk_dimensions().as_ivec3();

    for y in (0..chunks_size.y).step_by(stride as usize) {
        for z in (0..chunks_size.z).step_by(stride as usize) {
            for x in (0..chunks_size.x).step_by(stride as usize) {
                let chunk_pos = IVec3::new(x, y, z);

                offset.x = chunk_pos.x as f32 * chunk_size;
                offset.y = chunk_pos.y as f32 * chunk_size;
                offset.z = chunk_pos.z as f32 * chunk_size;

                let mut builder = OccupancyDataBuilder::default();

                #[cfg(feature = "trace_greedy_timings")]
                let mut timings = GreedyTimings::default();

                let mut got_something = false;

                let temp_y = y - 1;
                if temp_y >= 0 {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new((x + j) as u32, temp_y as u32, (z + i) as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::XZNeg,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                let temp_y = y + stride;
                if temp_y < chunks_size.y {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new((x + j) as u32, temp_y as u32, (z + i) as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::XZPos,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                let temp_x = x - 1;
                if temp_x >= 0 {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new(temp_x as u32, (y + j) as u32, (z + i) as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::YZNeg,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                let temp_x = x + stride;
                if temp_x < chunks_size.x {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new(temp_x as u32, (y + j) as u32, (z + i) as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::YZPos,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                let temp_z = z - 1;
                if temp_z >= 0 {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new((x + j) as u32, (y + i) as u32, temp_z as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::XYNeg,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                let temp_z = z + stride;
                if temp_z < chunks_size.z {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new((x + j) as u32, (y + i) as u32, temp_z as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            generate_external_occupancy_mask(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                ExternalPlane::XYPos,
                                UVec2::new(j as u32 * voxels_per_axis, i as u32 * voxels_per_axis),
                            );
                        }
                    }
                }

                for k in 0..stride {
                    for i in 0..stride {
                        for j in 0..stride {
                            let local_position =
                                UVec3::new((x + j) as u32, (y + k) as u32, (z + i) as u32);
                            let chunk = if let Some(chunk) = container.local_chunk(local_position) {
                                chunk
                            } else {
                                continue;
                            };

                            if chunk.is_empty() {
                                continue;
                            }

                            got_something = true;

                            generate_occupancy_masks(
                                store,
                                &mut builder,
                                &chunk.get_root_id(),
                                max_depth,
                                UVec3::new(
                                    j as u32 * voxels_per_axis,
                                    k as u32 * voxels_per_axis,
                                    i as u32 * voxels_per_axis,
                                ),
                                #[cfg(feature = "trace_greedy_timings")]
                                &mut timings,
                            );
                        }
                    }
                }

                if !got_something {
                    continue;
                }

                let occupancy_data = builder.build();

                generate_greedy_mesh_arrays(
                    &occupancy_data,
                    mesh_data,
                    mesh_max_depth,
                    offset,
                    voxel_size,
                    #[cfg(feature = "trace_greedy_timings")]
                    &mut timings,
                );
            }
        }
    }
}

// debug version, to be removed later
pub fn chunk_generate_greedy_mesh_arrays_ext<T: VoxelTrait>(
    chunk: &VoxChunk<T>,
    interner: &VoxInterner<T>,
    mesh_data: &mut MeshData,
    offset: Vec3,
    lod: Lod,
    clear_external_planes: [bool; 6],
    #[cfg(feature = "trace_greedy_timings")] timings: &mut GreedyTimings,
) {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("chunk_generate_greedy_mesh_arrays_ext");

    let voxel_size = chunk.voxel_size(lod);

    #[cfg(feature = "trace_greedy_timings")]
    let timing_builder_default = Instant::now();

    let mut builder = OccupancyDataBuilder::default();

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.builder_default = timing_builder_default.elapsed();
    }

    const EXTERNAL_PLANES: [ExternalPlane; 6] = [
        ExternalPlane::YZPos,
        ExternalPlane::YZNeg,
        ExternalPlane::XZPos,
        ExternalPlane::XZNeg,
        ExternalPlane::XYPos,
        ExternalPlane::XYNeg,
    ];

    #[cfg(feature = "trace_greedy_timings")]
    let timing_filling_external = Instant::now();

    for external_plane in EXTERNAL_PLANES {
        if clear_external_planes[external_plane as usize] {
            builder.fill_external_side(external_plane);
        }
    }

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.filling_external = timing_filling_external.elapsed();
    }

    let max_depth = chunk.max_depth(lod);

    generate_occupancy_masks(
        interner,
        &mut builder,
        &chunk.get_root_id(),
        max_depth,
        UVec3::ZERO,
        #[cfg(feature = "trace_greedy_timings")]
        timings,
    );

    #[cfg(feature = "trace_greedy_timings")]
    let timing_build_occupancy_masks = Instant::now();

    let occupancy_data = builder.build();

    #[cfg(feature = "trace_greedy_timings")]
    {
        timings.build_occupancy_masks = timing_build_occupancy_masks.elapsed();
    }

    generate_greedy_mesh_arrays(
        &occupancy_data,
        mesh_data,
        max_depth,
        offset,
        voxel_size,
        #[cfg(feature = "trace_greedy_timings")]
        timings,
    );
}
