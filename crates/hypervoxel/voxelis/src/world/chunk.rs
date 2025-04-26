use std::io::{BufReader, Read, Write};

use byteorder::BigEndian;
use byteorder::{ReadBytesExt, WriteBytesExt};
use glam::{IVec3, Vec3};
use rustc_hash::FxHashMap;
use wide::f32x8;

use crate::io::consts::VTC_MAGIC;
use crate::io::varint::{decode_varint_u32_from_reader, encode_varint};
use crate::spatial::{
    Octree, OctreeOpsBatch, OctreeOpsConfig, OctreeOpsDirty, OctreeOpsMesh, OctreeOpsRead,
    OctreeOpsState, OctreeOpsWrite,
};
use crate::{Batch, BlockId, DagInterner, Lod, MaxDepth};

const CUBE_VERTS: [Vec3; 8] = [
    Vec3::new(-1.0, 1.0, -1.0),
    Vec3::new(1.0, 1.0, -1.0),
    Vec3::new(1.0, 1.0, 1.0),
    Vec3::new(-1.0, 1.0, 1.0),
    Vec3::new(-1.0, -1.0, -1.0),
    Vec3::new(1.0, -1.0, -1.0),
    Vec3::new(1.0, -1.0, 1.0),
    Vec3::new(-1.0, -1.0, 1.0),
];

const VEC_UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const VEC_RIGHT: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const VEC_DOWN: Vec3 = Vec3::new(0.0, -1.0, 0.0);
const VEC_LEFT: Vec3 = Vec3::new(-1.0, 0.0, 0.0);
const VEC_FORWARD: Vec3 = Vec3::new(0.0, 0.0, -1.0);
const VEC_BACK: Vec3 = Vec3::new(0.0, 0.0, 1.0);

pub struct Chunk {
    data: Octree,
    position: IVec3,
    chunk_size: f32,
    max_depth: MaxDepth,
}

impl Chunk {
    pub fn with_position(chunk_size: f32, max_depth: MaxDepth, x: i32, y: i32, z: i32) -> Self {
        Self {
            data: Octree::make_static(max_depth),
            position: IVec3::new(x, y, z),
            chunk_size,
            max_depth,
        }
    }

    pub fn voxel_size(&self, lod: Lod) -> f32 {
        self.chunk_size / self.voxels_per_axis(lod) as f32
    }

    pub fn chunk_size(&self) -> f32 {
        self.chunk_size
    }

    pub fn set_position(&mut self, x: i32, y: i32, z: i32) {
        self.position = IVec3::new(x, y, z);
    }

    pub fn get_position(&self) -> IVec3 {
        self.position
    }

    pub fn get_world_position(&self) -> Vec3 {
        self.position.as_vec3() * self.chunk_size
    }

    pub fn get_root_id(&self) -> BlockId {
        self.data.get_root_id()
    }

    pub fn generate_test_data(&mut self, interner: &mut DagInterner<i32>) {
        let voxels_per_axis = self.voxels_per_axis(Lod::new(0)) as i32;
        let mut position = IVec3::ZERO;
        for y in 0..voxels_per_axis {
            position.y = y;
            let offset = y % 2;
            for z in offset..voxels_per_axis - offset {
                position.z = z;
                for x in offset..voxels_per_axis - offset {
                    position.x = x;
                    self.set(interner, position, y + 1);
                }
            }
        }
    }

    pub fn generate_test_sphere(
        &mut self,
        interner: &mut DagInterner<i32>,
        center: IVec3,
        radius: i32,
        value: i32,
    ) {
        debug_assert!(radius > 0);

        let (cx, cy, cz) = (center.x, center.y, center.z);
        let radius_squared = radius * radius;

        let voxels_per_axis = self.voxels_per_axis(Lod::new(0)) as i32;

        let mut position = IVec3::ZERO;

        let mut batch = self.create_batch();

        for y in 0..voxels_per_axis {
            position.y = y;
            for z in 0..voxels_per_axis {
                position.z = z;
                for x in 0..voxels_per_axis {
                    let dx = x - cx;
                    let dy = y - cy;
                    let dz = z - cz;

                    let distance_squared = dx * dx + dy * dy + dz * dz;

                    if distance_squared <= radius_squared {
                        position.x = x;
                        batch.set(interner, position, value);
                    }
                }
            }
        }

        self.apply_batch(interner, &batch);
    }

    #[inline(always)]
    fn add_quad(
        vertices: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        normals: &mut Vec<Vec3>,
        quad: [Vec3; 4],
        normal: &Vec3,
    ) {
        let index = vertices.len() as u32;

        vertices.extend(quad);
        normals.extend([normal, normal, normal, normal]);
        indices.extend([index + 2, index + 1, index, index + 3, index, index + 1]);
    }

    pub fn generate_mesh_arrays(
        &self,
        interner: &DagInterner<i32>,
        vertices: &mut Vec<Vec3>,
        normals: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        offset: Vec3,
        lod: Lod,
    ) {
        if self.data.is_leaf() {
            let half_voxel_offset = 0.5 + offset;
            let chunk_v0 = CUBE_VERTS[0] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v1 = CUBE_VERTS[1] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v2 = CUBE_VERTS[2] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v3 = CUBE_VERTS[3] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v4 = CUBE_VERTS[4] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v5 = CUBE_VERTS[5] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v6 = CUBE_VERTS[6] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v7 = CUBE_VERTS[7] * 0.5 * self.chunk_size + half_voxel_offset;

            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v0, chunk_v2, chunk_v3, chunk_v1],
                &VEC_UP,
            );
            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v2, chunk_v5, chunk_v6, chunk_v1],
                &VEC_RIGHT,
            );
            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v7, chunk_v5, chunk_v4, chunk_v6],
                &VEC_DOWN,
            );
            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v0, chunk_v7, chunk_v4, chunk_v3],
                &VEC_LEFT,
            );
            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v3, chunk_v6, chunk_v7, chunk_v2],
                &VEC_BACK,
            );
            Self::add_quad(
                vertices,
                indices,
                normals,
                [chunk_v1, chunk_v4, chunk_v5, chunk_v0],
                &VEC_FORWARD,
            );

            return;
        }

        let max_depth = self.max_depth.for_lod(lod);
        let voxels_per_axis = self.voxels_per_axis(lod);
        let voxel_size = self.voxel_size(lod);

        let half_voxel_size = voxel_size / 2.0;
        let voxel_size_vec3 = Vec3::splat(voxel_size);
        let half_voxel_size_vec3 = Vec3::splat(half_voxel_size);
        let shift_y = 1 << (2 * max_depth.as_usize());
        let shift_z = 1 << max_depth.as_usize();

        let half_voxel_offset = half_voxel_size_vec3 + offset;
        let chunk_v0 = CUBE_VERTS[0] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v1 = CUBE_VERTS[1] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v2 = CUBE_VERTS[2] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v3 = CUBE_VERTS[3] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v4 = CUBE_VERTS[4] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v5 = CUBE_VERTS[5] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v6 = CUBE_VERTS[6] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v7 = CUBE_VERTS[7] * half_voxel_size_vec3 + half_voxel_offset;

        let chunk_v_x = f32x8::from([
            chunk_v0.x, chunk_v1.x, chunk_v2.x, chunk_v3.x, chunk_v4.x, chunk_v5.x, chunk_v6.x,
            chunk_v7.x,
        ]);
        let chunk_v_y = f32x8::from([
            chunk_v0.y, chunk_v1.y, chunk_v2.y, chunk_v3.y, chunk_v4.y, chunk_v5.y, chunk_v6.y,
            chunk_v7.y,
        ]);
        let chunk_v_z = f32x8::from([
            chunk_v0.z, chunk_v1.z, chunk_v2.z, chunk_v3.z, chunk_v4.z, chunk_v5.z, chunk_v6.z,
            chunk_v7.z,
        ]);

        let data = self.data.to_vec(interner, lod);

        for y in 0..voxels_per_axis {
            let base_index_y = y as usize * shift_y;
            let v_y = f32x8::splat(y as f32 * voxel_size_vec3.y) + chunk_v_y;
            let v_y_array = v_y.to_array();

            for z in 0..voxels_per_axis {
                let base_index_z = base_index_y + z as usize * shift_z;
                let v_z = f32x8::splat(z as f32 * voxel_size_vec3.z) + chunk_v_z;
                let v_z_array = v_z.to_array();

                for x in 0..voxels_per_axis {
                    let index = base_index_z + x as usize;

                    if unsafe { *data.get_unchecked(index) } == 0 {
                        continue;
                    }

                    let has_top = y + 1 >= voxels_per_axis
                        || unsafe { *data.get_unchecked(index + shift_y) } == 0;
                    let has_bottom = y == 0 || unsafe { *data.get_unchecked(index - shift_y) } == 0;
                    let has_front = z + 1 >= voxels_per_axis
                        || unsafe { *data.get_unchecked(index + shift_z) } == 0;
                    let has_back = z == 0 || unsafe { *data.get_unchecked(index - shift_z) } == 0;
                    let has_right =
                        x + 1 >= voxels_per_axis || unsafe { *data.get_unchecked(index + 1) } == 0;
                    let has_left = x == 0 || unsafe { *data.get_unchecked(index - 1) } == 0;

                    // let has_top = false;
                    // let has_bottom = false;
                    // let has_front = false;
                    // let has_back = false;
                    // let has_right = false;
                    // let has_left = false;

                    if !(has_top || has_bottom || has_left || has_right || has_back || has_front) {
                        continue;
                    }

                    let v_x = f32x8::splat(x as f32 * voxel_size_vec3.x) + chunk_v_x;
                    let v_x_array = v_x.to_array();

                    let v0 = Vec3::new(v_x_array[0], v_y_array[0], v_z_array[0]);
                    let v1 = Vec3::new(v_x_array[1], v_y_array[1], v_z_array[1]);
                    let v2 = Vec3::new(v_x_array[2], v_y_array[2], v_z_array[2]);
                    let v3 = Vec3::new(v_x_array[3], v_y_array[3], v_z_array[3]);
                    let v4 = Vec3::new(v_x_array[4], v_y_array[4], v_z_array[4]);
                    let v5 = Vec3::new(v_x_array[5], v_y_array[5], v_z_array[5]);
                    let v6 = Vec3::new(v_x_array[6], v_y_array[6], v_z_array[6]);
                    let v7 = Vec3::new(v_x_array[7], v_y_array[7], v_z_array[7]);

                    if has_top {
                        Self::add_quad(vertices, indices, normals, [v0, v2, v3, v1], &VEC_UP);
                    }
                    if has_right {
                        Self::add_quad(vertices, indices, normals, [v2, v5, v6, v1], &VEC_RIGHT);
                    }
                    if has_bottom {
                        Self::add_quad(vertices, indices, normals, [v7, v5, v4, v6], &VEC_DOWN);
                    }
                    if has_left {
                        Self::add_quad(vertices, indices, normals, [v0, v7, v4, v3], &VEC_LEFT);
                    }
                    if has_front {
                        Self::add_quad(vertices, indices, normals, [v3, v6, v7, v2], &VEC_BACK);
                    }
                    if has_back {
                        Self::add_quad(vertices, indices, normals, [v1, v4, v5, v0], &VEC_FORWARD);
                    }
                }
            }
        }
    }

    pub fn serialize(&self, id_map: &FxHashMap<u32, u32>, data: &mut Vec<u8>) {
        let mut writer = std::io::BufWriter::new(data);

        writer.write_all(&VTC_MAGIC).unwrap();

        let position = self.position;

        writer.write_i32::<BigEndian>(position.x).unwrap();
        writer.write_i32::<BigEndian>(position.y).unwrap();
        writer.write_i32::<BigEndian>(position.z).unwrap();

        let root_id = self.data.get_root_id();
        let new_id = *id_map.get(&root_id.index()).unwrap();
        let new_id_bytes = encode_varint(new_id as usize);

        writer.write_all(&new_id_bytes).unwrap();
    }

    pub fn deserialize(
        interner: &mut DagInterner<i32>,
        leaf_patterns: &FxHashMap<u32, (BlockId, i32)>,
        patterns: &FxHashMap<u32, (BlockId, [u32; 8], i32)>,
        reader: &mut BufReader<&[u8]>,
        chunk_size: f32,
        max_depth: MaxDepth,
    ) -> Self {
        let mut magic = [0; VTC_MAGIC.len()];
        reader.read_exact(&mut magic).unwrap();
        assert_eq!(magic, VTC_MAGIC);

        // println!("Magic: {:?}", std::str::from_utf8(&magic).unwrap());

        let x = reader.read_i32::<BigEndian>().unwrap();
        let y = reader.read_i32::<BigEndian>().unwrap();
        let z = reader.read_i32::<BigEndian>().unwrap();

        let mut chunk = Chunk::with_position(chunk_size, max_depth, x, y, z);

        let root_id = decode_varint_u32_from_reader(reader).unwrap();
        match &mut chunk.data {
            Octree::Static(octree) => {
                if let Some((block_id, _, _)) = patterns.get(&root_id) {
                    octree.set_root_id(interner, *block_id);
                } else {
                    let (block_id, _) = leaf_patterns.get(&root_id).unwrap();
                    octree.set_root_id(interner, *block_id);
                }
            }
            Octree::Dynamic(_) => {
                panic!("Dynamic octree not supported");
            }
        }

        chunk
    }
}

impl OctreeOpsRead<i32> for Chunk {
    #[inline(always)]
    fn get(&self, interner: &DagInterner<i32>, position: IVec3) -> Option<i32> {
        self.data.get(interner, position)
    }
}

impl OctreeOpsWrite<i32> for Chunk {
    #[inline(always)]
    fn set(&mut self, interner: &mut DagInterner<i32>, position: IVec3, voxel: i32) -> bool {
        self.data.set(interner, position, voxel)
    }

    #[inline(always)]
    fn fill(&mut self, interner: &mut DagInterner<i32>, value: i32) {
        self.data.fill(interner, value)
    }

    #[inline(always)]
    fn clear(&mut self, interner: &mut DagInterner<i32>) {
        self.data.clear(interner)
    }
}

impl OctreeOpsBatch<i32> for Chunk {
    #[inline(always)]
    fn create_batch(&self) -> Batch<i32> {
        self.data.create_batch()
    }

    #[inline(always)]
    fn apply_batch(&mut self, interner: &mut DagInterner<i32>, batch: &Batch<i32>) -> bool {
        self.data.apply_batch(interner, batch)
    }
}

impl OctreeOpsMesh<i32> for Chunk {
    #[inline(always)]
    fn to_vec(&self, interner: &DagInterner<i32>, lod: Lod) -> Vec<i32> {
        self.data.to_vec(interner, lod)
    }
}

impl OctreeOpsConfig for Chunk {
    #[inline(always)]
    fn max_depth(&self, lod: Lod) -> MaxDepth {
        self.data.max_depth(lod)
    }

    #[inline(always)]
    fn voxels_per_axis(&self, lod: Lod) -> u32 {
        self.data.voxels_per_axis(lod)
    }
}

impl OctreeOpsState for Chunk {
    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[inline(always)]
    fn is_leaf(&self) -> bool {
        self.data.is_leaf()
    }
}

impl OctreeOpsDirty for Chunk {
    #[inline(always)]
    fn is_dirty(&self) -> bool {
        self.data.is_dirty()
    }

    #[inline(always)]
    fn mark_dirty(&mut self) {
        self.data.mark_dirty();
    }

    #[inline(always)]
    fn clear_dirty(&mut self) {
        self.data.clear_dirty()
    }
}
