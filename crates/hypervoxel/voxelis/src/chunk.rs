use std::io::{BufReader, Read, Write};

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use byteorder::BigEndian;
use byteorder::{ReadBytesExt, WriteBytesExt};
use wide::f32x8;

use crate::io::VTC_MAGIC;
use crate::io::{decode_varint, encode_varint};
use crate::voxtree::calculate_voxels_per_axis;
use crate::voxtree::VoxTree;

pub type Vec3 = bevy::math::Vec3;
pub type Freal = f32;

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

pub const MAX_LOD_LEVEL: usize = 6;
pub const VOXELS_PER_AXIS: u8 = calculate_voxels_per_axis(MAX_LOD_LEVEL) as u8;
pub const VOXELS_PER_AXIS_MINUS_ONE: u8 = VOXELS_PER_AXIS - 1;
pub const VOXEL_SIZE: Freal = 1.0 / VOXELS_PER_AXIS as Freal;
pub const VOXEL_SIZE_VEC3: Vec3 = Vec3::splat(VOXEL_SIZE);
pub const HALF_VOXEL_SIZE: Freal = VOXEL_SIZE / 2.0;
pub const HALF_VOXEL_SIZE_VEC3: Vec3 = Vec3::splat(HALF_VOXEL_SIZE);
pub const INV_VOXEL_SIZE: Freal = 1.0 / VOXEL_SIZE;
pub const SHIFT_Y: usize = 1 << (2 * MAX_LOD_LEVEL);
pub const SHIFT_Z: usize = 1 << MAX_LOD_LEVEL;

#[derive(Default)]
pub struct Chunk {
    data: VoxTree<MAX_LOD_LEVEL>,
    position: IVec3,
}

impl Chunk {
    pub fn new() -> Self {
        Self {
            data: VoxTree::<MAX_LOD_LEVEL>::default(),
            position: IVec3::ZERO,
        }
    }

    pub fn with_position(x: i32, y: i32, z: i32) -> Self {
        Self {
            data: VoxTree::<MAX_LOD_LEVEL>::default(),
            position: IVec3::new(x, y, z),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn clear(&mut self) {
        self.data.clear();
    }

    pub fn set_position(&mut self, x: i32, y: i32, z: i32) {
        self.position = IVec3::new(x, y, z);
    }

    pub fn get_position(&self) -> IVec3 {
        self.position
    }

    pub fn set_value(&mut self, x: u8, y: u8, z: u8, value: i32) {
        self.data.set_value(0, x, y, z, value);
    }

    pub fn get_value(&self, x: u8, y: u8, z: u8) -> i32 {
        self.data.get_value(0, x, y, z)
    }

    pub fn update_lods(&mut self) {
        self.data.update_lods();
    }

    pub fn generate_test_data(&mut self) {
        for y in 0..VOXELS_PER_AXIS {
            let offset = y % 2;
            for z in offset..VOXELS_PER_AXIS - offset {
                for x in offset..VOXELS_PER_AXIS - offset {
                    self.data.set_value(0, x, y, z, y as i32 + 1);
                }
            }
        }

        self.data.update_lods();
    }

    #[inline(always)]
    fn add_quad(
        vertices: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        normals: &mut Vec<Vec3>,
        quad: [Vec3; 4],
        normal: Vec3,
    ) {
        let index = vertices.len() as u32;

        vertices.extend_from_slice(&quad);
        normals.extend_from_slice(&[normal, normal, normal, normal]);
        indices.extend_from_slice(&[index + 2, index + 1, index, index + 3, index, index + 1]);
    }

    #[inline(always)]
    fn get_index(x: u8, y: u8, z: u8) -> usize {
        ((y as usize) << (2 * MAX_LOD_LEVEL)) + ((z as usize) << MAX_LOD_LEVEL) + x as usize
    }

    pub fn to_vec(&self, lod: usize) -> Vec<i32> {
        self.data.to_vec(lod)
    }

    pub fn generate_mesh_arrays(
        data: &[i32],
        vertices: &mut Vec<Vec3>,
        normals: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        offset: Vec3,
    ) {
        let half_voxel_offset = HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v0 = CUBE_VERTS[0] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v1 = CUBE_VERTS[1] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v2 = CUBE_VERTS[2] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v3 = CUBE_VERTS[3] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v4 = CUBE_VERTS[4] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v5 = CUBE_VERTS[5] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v6 = CUBE_VERTS[6] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v7 = CUBE_VERTS[7] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;

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

        for y in 0..VOXELS_PER_AXIS {
            let base_index_y = (y as usize) * SHIFT_Y;
            let v_y = f32x8::splat(y as f32 * VOXEL_SIZE_VEC3.y) + chunk_v_y;
            let v_y_array = v_y.to_array();

            for z in 0..VOXELS_PER_AXIS {
                let base_index_z = base_index_y + (z as usize) * SHIFT_Z;
                let v_z = f32x8::splat(z as f32 * VOXEL_SIZE_VEC3.z) + chunk_v_z;
                let v_z_array = v_z.to_array();

                for x in 0..VOXELS_PER_AXIS {
                    let index = base_index_z + x as usize;

                    if unsafe { *data.get_unchecked(index) } == 0 {
                        continue;
                    }

                    let has_top = y + 1 >= VOXELS_PER_AXIS
                        || unsafe { *data.get_unchecked(index + SHIFT_Y) } == 0;
                    let has_bottom = y == 0 || unsafe { *data.get_unchecked(index - SHIFT_Y) } == 0;
                    let has_front = z + 1 >= VOXELS_PER_AXIS
                        || unsafe { *data.get_unchecked(index + SHIFT_Z) } == 0;
                    let has_back = z == 0 || unsafe { *data.get_unchecked(index - SHIFT_Z) } == 0;
                    let has_right =
                        x + 1 >= VOXELS_PER_AXIS || unsafe { *data.get_unchecked(index + 1) } == 0;
                    let has_left = x == 0 || unsafe { *data.get_unchecked(index - 1) } == 0;

                    if !(has_top || has_bottom || has_left || has_right || has_back || has_front) {
                        continue;
                    }

                    let v_x = f32x8::splat(x as f32 * VOXEL_SIZE_VEC3.x) + chunk_v_x;
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
                        Self::add_quad(vertices, indices, normals, [v0, v2, v3, v1], VEC_UP);
                    }
                    if has_right {
                        Self::add_quad(vertices, indices, normals, [v2, v5, v6, v1], VEC_RIGHT);
                    }
                    if has_bottom {
                        Self::add_quad(vertices, indices, normals, [v7, v5, v4, v6], VEC_DOWN);
                    }
                    if has_left {
                        Self::add_quad(vertices, indices, normals, [v0, v7, v4, v3], VEC_LEFT);
                    }
                    if has_front {
                        Self::add_quad(vertices, indices, normals, [v3, v6, v7, v2], VEC_BACK);
                    }
                    if has_back {
                        Self::add_quad(vertices, indices, normals, [v1, v4, v5, v0], VEC_FORWARD);
                    }
                }
            }
        }
    }

    pub fn generate_greedy_mesh_arrays(
        data: &[i32],
        vertices: &mut Vec<Vec3>,
        normals: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        offset: Vec3,
    ) {
        let half_voxel_offset = HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v0 = CUBE_VERTS[0] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v1 = CUBE_VERTS[1] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v2 = CUBE_VERTS[2] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v3 = CUBE_VERTS[3] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v4 = CUBE_VERTS[4] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v5 = CUBE_VERTS[5] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v6 = CUBE_VERTS[6] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;
        let chunk_v7 = CUBE_VERTS[7] * HALF_VOXEL_SIZE_VEC3 + half_voxel_offset;

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

        for y in 0..VOXELS_PER_AXIS {
            let base_index_y = (y as usize) * SHIFT_Y;
            let v_y = f32x8::splat(y as f32 * VOXEL_SIZE_VEC3.y) + chunk_v_y;
            let v_y_array = v_y.to_array();

            for z in 0..VOXELS_PER_AXIS {
                let base_index_z = base_index_y + (z as usize) * SHIFT_Z;
                let v_z = f32x8::splat(z as f32 * VOXEL_SIZE_VEC3.z) + chunk_v_z;
                let v_z_array = v_z.to_array();

                for x in 0..VOXELS_PER_AXIS {
                    let index = base_index_z + x as usize;

                    if unsafe { *data.get_unchecked(index) } == 0 {
                        continue;
                    }

                    let has_top = y + 1 >= VOXELS_PER_AXIS
                        || unsafe { *data.get_unchecked(index + SHIFT_Y) } == 0;
                    let has_bottom = y == 0 || unsafe { *data.get_unchecked(index - SHIFT_Y) } == 0;
                    let has_front = z + 1 >= VOXELS_PER_AXIS
                        || unsafe { *data.get_unchecked(index + SHIFT_Z) } == 0;
                    let has_back = z == 0 || unsafe { *data.get_unchecked(index - SHIFT_Z) } == 0;
                    let has_right =
                        x + 1 >= VOXELS_PER_AXIS || unsafe { *data.get_unchecked(index + 1) } == 0;
                    let has_left = x == 0 || unsafe { *data.get_unchecked(index - 1) } == 0;

                    if !(has_top || has_bottom || has_left || has_right || has_back || has_front) {
                        continue;
                    }

                    let v_x = f32x8::splat(x as f32 * VOXEL_SIZE_VEC3.x) + chunk_v_x;
                    let v_x_array = v_x.to_array();

                    let v0 = Vec3::new(v_x_array[0], v_y_array[0], v_z_array[0]);
                    let v1 = Vec3::new(v_x_array[1], v_y_array[1], v_z_array[1]);
                    let v2 = Vec3::new(v_x_array[2], v_y_array[2], v_z_array[2]);
                    let v3 = Vec3::new(v_x_array[3], v_y_array[3], v_z_array[3]);
                    let v4 = Vec3::new(v_x_array[4], v_y_array[4], v_z_array[4]);
                    let v5 = Vec3::new(v_x_array[5], v_y_array[5], v_z_array[5]);
                    let v6 = Vec3::new(v_x_array[6], v_y_array[6], v_z_array[6]);
                    let v7 = Vec3::new(v_x_array[7], v_y_array[7], v_z_array[7]);

                    // if has_top {
                    //     Self::add_quad(vertices, indices, normals, [v0, v2, v3, v1], VEC_UP);
                    // }
                    // if has_right {
                    //     Self::add_quad(vertices, indices, normals, [v2, v5, v6, v1], VEC_RIGHT);
                    // }
                    // if has_bottom {
                    //     Self::add_quad(vertices, indices, normals, [v7, v5, v4, v6], VEC_DOWN);
                    // }
                    // if has_left {
                    //     Self::add_quad(vertices, indices, normals, [v0, v7, v4, v3], VEC_LEFT);
                    // }
                    // if has_front {
                    //     Self::add_quad(vertices, indices, normals, [v3, v6, v7, v2], VEC_BACK);
                    // }
                    // if has_back {
                    //     Self::add_quad(vertices, indices, normals, [v1, v4, v5, v0], VEC_FORWARD);
                    // }
                }
            }
        }
    }

    pub fn generate_mesh(&self) -> Option<Mesh> {
        if self.is_empty() {
            return None;
        }

        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        let data = self.to_vec(0);

        Self::generate_mesh_arrays(&data, &mut vertices, &mut normals, &mut indices, Vec3::ZERO);

        Some(
            Mesh::new(
                PrimitiveTopology::TriangleList,
                RenderAssetUsages::RENDER_WORLD,
            )
            .with_inserted_indices(Indices::U32(indices))
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals),
        )
    }

    pub fn generate_greedy_mesh(&self) -> Option<Mesh> {
        if self.is_empty() {
            return None;
        }

        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        let data = self.to_vec(0);

        Self::generate_greedy_mesh_arrays(
            &data,
            &mut vertices,
            &mut normals,
            &mut indices,
            Vec3::ZERO,
        );

        Some(
            Mesh::new(
                PrimitiveTopology::TriangleList,
                RenderAssetUsages::RENDER_WORLD,
            )
            .with_inserted_indices(Indices::U32(indices))
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals),
        )
    }

    pub fn serialize(&self, data: &mut Vec<u8>) {
        let mut writer = std::io::BufWriter::new(data);

        writer.write_all(&VTC_MAGIC).unwrap();

        let position = self.position;

        writer
            .write_u16::<BigEndian>(position.x.try_into().unwrap())
            .unwrap();
        writer
            .write_u16::<BigEndian>(position.y.try_into().unwrap())
            .unwrap();
        writer
            .write_u16::<BigEndian>(position.z.try_into().unwrap())
            .unwrap();

        let rle_data = self.encode_rle();

        writer
            .write_u32::<BigEndian>(rle_data.len().try_into().unwrap())
            .unwrap();
        writer.write_all(&rle_data).unwrap();
    }

    pub fn deserialize(&mut self, data: &[u8], chunk_index: usize, offsets: &[usize]) {
        let offset = offsets[chunk_index];
        let mut reader = BufReader::new(&data[offset..]);

        let mut magic = [0; VTC_MAGIC.len()];
        reader.read_exact(&mut magic).unwrap();
        assert_eq!(magic, VTC_MAGIC);

        let x = reader.read_u16::<BigEndian>().unwrap() as i32;
        let y = reader.read_u16::<BigEndian>().unwrap() as i32;
        let z = reader.read_u16::<BigEndian>().unwrap() as i32;
        self.position = IVec3::new(x, y, z);

        let data_len = reader.read_u32::<BigEndian>().unwrap() as usize;
        let mut data = vec![0; data_len];
        reader.read_exact(&mut data).unwrap();

        let rle_data = Self::decode_rle(&data);

        self.data.for_each_mut(|index, value| {
            *value = rle_data[index];
        });
    }

    fn encode_rle(&self) -> Vec<u8> {
        let mut buffer = Vec::new();

        let mut iter = self.data.iter().peekable();

        while let Some(value) = iter.next() {
            // Initialize count for the current run
            let mut count = 1;

            // Count how many times the current value repeats consecutively
            while let Some(&next_value) = iter.peek() {
                if next_value == value {
                    iter.next();
                    count += 1;
                } else {
                    break;
                }
            }

            // Encode the count using variable-length encoding
            let count_bytes = encode_varint(count);
            buffer.extend(count_bytes);

            // Encode the value using variable-length encoding
            let value_bytes = encode_varint(value as usize);
            buffer.extend_from_slice(&value_bytes);
        }

        buffer
    }

    fn decode_rle(input: &[u8]) -> Vec<i32> {
        let mut output = Vec::new();
        let mut iter = input.iter();

        while let Some(count) = decode_varint(&mut iter) {
            // Read the next 4 bytes for the i32 value
            let value =
                decode_varint(&mut iter).expect("Unexpected end of input during value read") as i32;

            // Append 'value' to the output 'count' times
            output.extend(std::iter::repeat(value).take(count));
        }

        output
    }
}
