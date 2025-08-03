use std::io::{BufReader, Read, Write};

use byteorder::BigEndian;
use byteorder::{ReadBytesExt, WriteBytesExt};
use glam::{IVec3, UVec3, Vec3};
use rustc_hash::FxHashMap;
use wide::f32x8;

use crate::io::consts::VTC_MAGIC;
use crate::io::varint::{decode_varint_u32_from_reader, encode_varint};
use crate::spatial::{
    VoxOpsBatch, VoxOpsBulkWrite, VoxOpsChunkConfig, VoxOpsConfig, VoxOpsDirty, VoxOpsMesh,
    VoxOpsRead, VoxOpsSpatial3D, VoxOpsState, VoxOpsWrite, VoxTree,
};
use crate::utils::mesh::{self, MeshData};
use crate::{Batch, BlockId, Lod, MaxDepth, VoxInterner};

pub struct VoxChunk {
    data: VoxTree,
    position: IVec3,
    chunk_size: f32,
    max_depth: MaxDepth,
}

impl VoxChunk {
    pub fn with_position(chunk_size: f32, max_depth: MaxDepth, x: i32, y: i32, z: i32) -> Self {
        Self {
            data: VoxTree::new(max_depth),
            position: IVec3::new(x, y, z),
            chunk_size,
            max_depth,
        }
    }

    pub fn set_position(&mut self, x: i32, y: i32, z: i32) {
        self.position = IVec3::new(x, y, z);
    }

    pub fn get_root_id(&self) -> BlockId {
        self.data.get_root_id()
    }

    pub fn generate_mesh_arrays(
        &self,
        interner: &VoxInterner<i32>,
        mesh_data: &mut MeshData,
        offset: Vec3,
        lod: Lod,
    ) {
        if self.data.is_leaf() {
            let half_voxel_offset = 0.5 + offset;
            let chunk_v0 = mesh::CUBE_VERTS[0] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v1 = mesh::CUBE_VERTS[1] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v2 = mesh::CUBE_VERTS[2] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v3 = mesh::CUBE_VERTS[3] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v4 = mesh::CUBE_VERTS[4] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v5 = mesh::CUBE_VERTS[5] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v6 = mesh::CUBE_VERTS[6] * 0.5 * self.chunk_size + half_voxel_offset;
            let chunk_v7 = mesh::CUBE_VERTS[7] * 0.5 * self.chunk_size + half_voxel_offset;

            mesh::add_quad(
                mesh_data,
                [chunk_v0, chunk_v2, chunk_v3, chunk_v1],
                &mesh::VEC_UP,
            );
            mesh::add_quad(
                mesh_data,
                [chunk_v2, chunk_v5, chunk_v6, chunk_v1],
                &mesh::VEC_RIGHT,
            );
            mesh::add_quad(
                mesh_data,
                [chunk_v7, chunk_v5, chunk_v4, chunk_v6],
                &mesh::VEC_DOWN,
            );
            mesh::add_quad(
                mesh_data,
                [chunk_v0, chunk_v7, chunk_v4, chunk_v3],
                &mesh::VEC_LEFT,
            );
            mesh::add_quad(
                mesh_data,
                [chunk_v3, chunk_v6, chunk_v7, chunk_v2],
                &mesh::VEC_BACK,
            );
            mesh::add_quad(
                mesh_data,
                [chunk_v1, chunk_v4, chunk_v5, chunk_v0],
                &mesh::VEC_FORWARD,
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
        let chunk_v0 = mesh::CUBE_VERTS[0] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v1 = mesh::CUBE_VERTS[1] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v2 = mesh::CUBE_VERTS[2] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v3 = mesh::CUBE_VERTS[3] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v4 = mesh::CUBE_VERTS[4] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v5 = mesh::CUBE_VERTS[5] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v6 = mesh::CUBE_VERTS[6] * half_voxel_size_vec3 + half_voxel_offset;
        let chunk_v7 = mesh::CUBE_VERTS[7] * half_voxel_size_vec3 + half_voxel_offset;

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
                        mesh::add_quad(mesh_data, [v0, v2, v3, v1], &mesh::VEC_UP);
                    }
                    if has_right {
                        mesh::add_quad(mesh_data, [v2, v5, v6, v1], &mesh::VEC_RIGHT);
                    }
                    if has_bottom {
                        mesh::add_quad(mesh_data, [v7, v5, v4, v6], &mesh::VEC_DOWN);
                    }
                    if has_left {
                        mesh::add_quad(mesh_data, [v0, v7, v4, v3], &mesh::VEC_LEFT);
                    }
                    if has_front {
                        mesh::add_quad(mesh_data, [v3, v6, v7, v2], &mesh::VEC_BACK);
                    }
                    if has_back {
                        mesh::add_quad(mesh_data, [v1, v4, v5, v0], &mesh::VEC_FORWARD);
                    }
                }
            }
        }
    }
}

impl VoxOpsRead<i32> for VoxChunk {
    #[inline(always)]
    fn get(&self, interner: &VoxInterner<i32>, position: IVec3) -> Option<i32> {
        self.data.get(interner, position)
    }
}

impl VoxOpsWrite<i32> for VoxChunk {
    #[inline(always)]
    fn set(&mut self, interner: &mut VoxInterner<i32>, position: IVec3, voxel: i32) -> bool {
        self.data.set(interner, position, voxel)
    }
}

impl VoxOpsBulkWrite<i32> for VoxChunk {
    #[inline(always)]
    fn fill(&mut self, interner: &mut VoxInterner<i32>, value: i32) {
        self.data.fill(interner, value)
    }

    #[inline(always)]
    fn clear(&mut self, interner: &mut VoxInterner<i32>) {
        self.data.clear(interner)
    }
}

impl VoxOpsBatch<i32> for VoxChunk {
    #[inline(always)]
    fn create_batch(&self) -> Batch<i32> {
        self.data.create_batch()
    }

    #[inline(always)]
    fn apply_batch(&mut self, interner: &mut VoxInterner<i32>, batch: &Batch<i32>) -> bool {
        self.data.apply_batch(interner, batch)
    }
}

impl VoxOpsSpatial3D for VoxChunk {
    #[inline(always)]
    fn position_3d(&self) -> IVec3 {
        self.position
    }

    #[inline(always)]
    fn world_position_3d(&self) -> Vec3 {
        self.position.as_vec3() * Vec3::splat(self.chunk_size)
    }

    #[inline(always)]
    fn world_center_position_3d(&self) -> Vec3 {
        let half_size = self.chunk_size / 2.0;
        self.world_position_3d() + Vec3::splat(half_size)
    }

    #[inline(always)]
    fn world_size_3d(&self) -> Vec3 {
        Vec3::splat(self.chunk_size)
    }
}

impl VoxOpsMesh<i32> for VoxChunk {
    #[inline(always)]
    fn to_vec(&self, interner: &VoxInterner<i32>, lod: Lod) -> Vec<i32> {
        self.data.to_vec(interner, lod)
    }
}

impl VoxOpsConfig for VoxChunk {
    #[inline(always)]
    fn max_depth(&self, lod: Lod) -> MaxDepth {
        self.data.max_depth(lod)
    }

    #[inline(always)]
    fn voxels_per_axis(&self, lod: Lod) -> u32 {
        self.data.voxels_per_axis(lod)
    }
}

impl VoxOpsState for VoxChunk {
    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[inline(always)]
    fn is_leaf(&self) -> bool {
        self.data.is_leaf()
    }
}

impl VoxOpsDirty for VoxChunk {
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

impl VoxOpsChunkConfig for VoxChunk {
    #[inline(always)]
    fn chunk_dimensions(&self) -> UVec3 {
        UVec3::splat(1)
    }

    #[inline(always)]
    fn chunk_size(&self) -> f32 {
        self.chunk_size
    }

    #[inline(always)]
    fn voxel_size(&self, lod: Lod) -> f32 {
        self.chunk_size / self.data.voxels_per_axis(lod) as f32
    }
}

pub fn chunk_serialize(chunk: &VoxChunk, id_map: &FxHashMap<u32, u32>, data: &mut Vec<u8>) {
    let mut writer = std::io::BufWriter::new(data);

    writer.write_all(&VTC_MAGIC).unwrap();

    let position = chunk.position_3d();

    writer.write_i32::<BigEndian>(position.x).unwrap();
    writer.write_i32::<BigEndian>(position.y).unwrap();
    writer.write_i32::<BigEndian>(position.z).unwrap();

    let root_id = chunk.get_root_id();
    let new_id = *id_map.get(&root_id.index()).unwrap();
    let new_id_bytes = encode_varint(new_id as usize);

    writer.write_all(&new_id_bytes).unwrap();
}

pub fn chunk_deserialize(
    interner: &mut VoxInterner<i32>,
    leaf_patterns: &FxHashMap<u32, (BlockId, i32)>,
    patterns: &FxHashMap<u32, (BlockId, [u32; 8], i32)>,
    reader: &mut BufReader<&[u8]>,
    chunk_size: f32,
    max_depth: MaxDepth,
) -> VoxChunk {
    let mut magic = [0; VTC_MAGIC.len()];
    reader.read_exact(&mut magic).unwrap();
    assert_eq!(magic, VTC_MAGIC);

    // println!("Magic: {:?}", std::str::from_utf8(&magic).unwrap());

    let x = reader.read_i32::<BigEndian>().unwrap();
    let y = reader.read_i32::<BigEndian>().unwrap();
    let z = reader.read_i32::<BigEndian>().unwrap();

    let mut chunk = VoxChunk::with_position(chunk_size, max_depth, x, y, z);

    let root_id = decode_varint_u32_from_reader(reader).unwrap();
    if let Some((block_id, _, _)) = patterns.get(&root_id) {
        chunk.data.set_root_id(interner, *block_id);
    } else {
        let (block_id, _) = leaf_patterns.get(&root_id).unwrap();
        chunk.data.set_root_id(interner, *block_id);
    }

    chunk
}
