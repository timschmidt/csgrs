use std::io::{BufReader, Read, Write};

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use byteorder::BigEndian;
use byteorder::{ReadBytesExt, WriteBytesExt};

use crate::io::VTC_MAGIC;
use crate::io::{decode_varint, encode_varint};
use crate::math::Freal;
use crate::voxtree::calculate_voxels_per_axis;
use crate::voxtree::VoxTree;

pub type Vec3 = bevy::math::Vec3;

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

const VECTOR_UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const VECTOR_RIGHT: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const VECTOR_DOWN: Vec3 = Vec3::new(0.0, -1.0, 0.0);
const VECTOR_LEFT: Vec3 = Vec3::new(-1.0, 0.0, 0.0);
const VECTOR_FORWARD: Vec3 = Vec3::new(0.0, 0.0, -1.0);
const VECTOR_BACK: Vec3 = Vec3::new(0.0, 0.0, 1.0);

pub const MAX_LOD_LEVEL: usize = 6;
pub const VOXELS_PER_AXIS: u8 = calculate_voxels_per_axis(MAX_LOD_LEVEL) as u8;
pub const VOXELS_PER_AXIS_MINUS_ONE: u8 = VOXELS_PER_AXIS - 1;
pub const VOXEL_SIZE: Freal = 1.0 / VOXELS_PER_AXIS as Freal;
pub const VOXEL_SIZE_VEC3: Vec3 = Vec3::splat(VOXEL_SIZE);
pub const HALF_VOXEL_SIZE: Freal = VOXEL_SIZE / 2.0;
pub const HALF_VOXEL_SIZE_VEC3: Vec3 = Vec3::splat(HALF_VOXEL_SIZE);
pub const INV_VOXEL_SIZE: Freal = 1.0 / VOXEL_SIZE;

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

        // self.data.set_value(0, 0, 0, 0, 1);
        // self.data.set_value(0, 1, 0, 0, 1);
        // self.data.set_value(0, 0, 0, 1, 1);
        // self.data.set_value(0, 1, 0, 1, 1);
        // self.data.set_value(0, 0, 1, 0, 1);
        // self.data.set_value(0, 1, 1, 0, 1);
        // self.data.set_value(0, 0, 1, 1, 1);
        // self.data.set_value(0, 1, 1, 1, 1);

        // self.data.set_value(0, 2, 0, 0, 1);
        // self.data.set_value(0, 3, 0, 0, 1);
        // self.data.set_value(0, 2, 0, 1, 1);
        // self.data.set_value(0, 3, 0, 1, 1);
        // self.data.set_value(0, 2, 1, 0, 1);
        // self.data.set_value(0, 3, 1, 0, 1);
        // self.data.set_value(0, 2, 1, 1, 1);

        // self.data.set_value(0, 0, 0, 2, 1);
        // self.data.set_value(0, 1, 0, 2, 1);
        // self.data.set_value(0, 0, 0, 3, 1);
        // self.data.set_value(0, 1, 0, 3, 1);
        // self.data.set_value(0, 0, 1, 2, 1);
        // self.data.set_value(0, 1, 1, 2, 1);

        // self.data.set_value(0, 2, 0, 2, 1);
        // self.data.set_value(0, 3, 0, 2, 1);
        // self.data.set_value(0, 2, 0, 3, 1);
        // self.data.set_value(0, 3, 0, 3, 1);
        // self.data.set_value(0, 2, 1, 2, 1);

        // self.data.set_value(0, 0, 2, 0, 1);
        // self.data.set_value(0, 1, 2, 0, 1);
        // self.data.set_value(0, 0, 2, 1, 1);
        // self.data.set_value(0, 1, 2, 1, 1);

        // self.data.set_value(0, 2, 2, 0, 1);
        // self.data.set_value(0, 3, 2, 0, 1);
        // self.data.set_value(0, 2, 2, 1, 1);

        // self.data.set_value(0, 0, 2, 2, 1);
        // self.data.set_value(0, 1, 2, 2, 1);

        // self.data.set_value(0, 2, 2, 2, 1);

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

        normals.extend(std::iter::repeat(normal).take(4));

        indices.extend_from_slice(&[index + 2, index + 1, index, index + 3, index, index + 1]);
    }

    #[inline(always)]
    fn get_index(x: u8, y: u8, z: u8) -> usize {
        ((y as usize) << (2 * MAX_LOD_LEVEL)) + ((z as usize) << MAX_LOD_LEVEL) + x as usize
    }

    pub fn generate_mesh_arrays(
        &self,
        vertices: &mut Vec<Vec3>,
        normals: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        offset: Vec3,
    ) {
        let chunk_v0 = CUBE_VERTS[0] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v1 = CUBE_VERTS[1] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v2 = CUBE_VERTS[2] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v3 = CUBE_VERTS[3] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v4 = CUBE_VERTS[4] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v5 = CUBE_VERTS[5] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v6 = CUBE_VERTS[6] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v7 = CUBE_VERTS[7] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;

        let data = self.data.to_vec(0);

        for y in 0..VOXELS_PER_AXIS {
            for z in 0..VOXELS_PER_AXIS {
                for x in 0..VOXELS_PER_AXIS {
                    let value = data[Self::get_index(x, y, z)];

                    if value == 0 {
                        continue;
                    }

                    let has_top =
                        y + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x, y + 1, z)] == 0;
                    let has_bottom = y == 0 || data[Self::get_index(x, y - 1, z)] == 0;
                    let has_left = x == 0 || data[Self::get_index(x - 1, y, z)] == 0;
                    let has_right =
                        x + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x + 1, y, z)] == 0;
                    let has_back = z == 0 || data[Self::get_index(x, y, z - 1)] == 0;
                    let has_front =
                        z + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x, y, z + 1)] == 0;

                    let has_something =
                        has_top || has_bottom || has_left || has_right || has_back || has_front;

                    if !has_something {
                        continue;
                    }

                    let position = Vec3::new(x as f32, y as f32, z as f32) * VOXEL_SIZE_VEC3;

                    if has_top {
                        let v0 = position + chunk_v0;
                        let v1 = position + chunk_v1;
                        let v2 = position + chunk_v2;
                        let v3 = position + chunk_v3;
                        Self::add_quad(vertices, indices, normals, [v0, v2, v3, v1], VECTOR_UP);
                    }
                    if has_right {
                        let v1 = position + chunk_v1;
                        let v2 = position + chunk_v2;
                        let v5 = position + chunk_v5;
                        let v6 = position + chunk_v6;
                        Self::add_quad(vertices, indices, normals, [v2, v5, v6, v1], VECTOR_RIGHT);
                    }
                    if has_bottom {
                        let v4 = position + chunk_v4;
                        let v5 = position + chunk_v5;
                        let v6 = position + chunk_v6;
                        let v7 = position + chunk_v7;
                        Self::add_quad(vertices, indices, normals, [v7, v5, v4, v6], VECTOR_DOWN);
                    }
                    if has_left {
                        let v0 = position + chunk_v0;
                        let v3 = position + chunk_v3;
                        let v4 = position + chunk_v4;
                        let v7 = position + chunk_v7;
                        Self::add_quad(vertices, indices, normals, [v0, v7, v4, v3], VECTOR_LEFT);
                    }
                    if has_front {
                        let v2 = position + chunk_v2;
                        let v3 = position + chunk_v3;
                        let v6 = position + chunk_v6;
                        let v7 = position + chunk_v7;
                        Self::add_quad(vertices, indices, normals, [v3, v6, v7, v2], VECTOR_BACK);
                    }
                    if has_back {
                        let v0 = position + chunk_v0;
                        let v1 = position + chunk_v1;
                        let v4 = position + chunk_v4;
                        let v5 = position + chunk_v5;
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            [v1, v4, v5, v0],
                            VECTOR_FORWARD,
                        );
                    }
                }
            }
        }
    }

    pub fn generate_greedy_mesh_arrays(
        &self,
        vertices: &mut Vec<Vec3>,
        normals: &mut Vec<Vec3>,
        indices: &mut Vec<u32>,
        offset: Vec3,
    ) {
        let chunk_v0 = CUBE_VERTS[0] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v1 = CUBE_VERTS[1] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v2 = CUBE_VERTS[2] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v3 = CUBE_VERTS[3] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v4 = CUBE_VERTS[4] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v5 = CUBE_VERTS[5] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v6 = CUBE_VERTS[6] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;
        let chunk_v7 = CUBE_VERTS[7] * HALF_VOXEL_SIZE_VEC3 + HALF_VOXEL_SIZE_VEC3 + offset;

        let data = self.data.to_vec(0);

        for y in 0..VOXELS_PER_AXIS {
            for z in 0..VOXELS_PER_AXIS {
                for x in 0..VOXELS_PER_AXIS {
                    let value = data[Self::get_index(x, y, z)];

                    if value == 0 {
                        continue;
                    }

                    let has_top =
                        y + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x, y + 1, z)] == 0;
                    let has_bottom = y == 0 || data[Self::get_index(x, y - 1, z)] == 0;
                    let has_left = x == 0 || data[Self::get_index(x - 1, y, z)] == 0;
                    let has_right =
                        x + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x + 1, y, z)] == 0;
                    let has_back = z == 0 || data[Self::get_index(x, y, z - 1)] == 0;
                    let has_front =
                        z + 1 >= VOXELS_PER_AXIS || data[Self::get_index(x, y, z + 1)] == 0;

                    // let has_something =
                    //     has_top || has_bottom || has_left || has_right || has_back || has_front;

                    // if !has_something {
                    //     continue;
                    // }

                    // let position = Vec3::new(x as f32, y as f32, z as f32) * VOXEL_SIZE_VEC3;

                    // if has_top {
                    //     let v0 = position + chunk_v0;
                    //     let v1 = position + chunk_v1;
                    //     let v2 = position + chunk_v2;
                    //     let v3 = position + chunk_v3;
                    //     Self::add_quad(vertices, indices, normals, [v0, v2, v3, v1], VECTOR_UP);
                    // }
                    // if has_right {
                    //     let v1 = position + chunk_v1;
                    //     let v2 = position + chunk_v2;
                    //     let v5 = position + chunk_v5;
                    //     let v6 = position + chunk_v6;
                    //     Self::add_quad(vertices, indices, normals, [v2, v5, v6, v1], VECTOR_RIGHT);
                    // }
                    // if has_bottom {
                    //     let v4 = position + chunk_v4;
                    //     let v5 = position + chunk_v5;
                    //     let v6 = position + chunk_v6;
                    //     let v7 = position + chunk_v7;
                    //     Self::add_quad(vertices, indices, normals, [v7, v5, v4, v6], VECTOR_DOWN);
                    // }
                    // if has_left {
                    //     let v0 = position + chunk_v0;
                    //     let v3 = position + chunk_v3;
                    //     let v4 = position + chunk_v4;
                    //     let v7 = position + chunk_v7;
                    //     Self::add_quad(vertices, indices, normals, [v0, v7, v4, v3], VECTOR_LEFT);
                    // }
                    // if has_front {
                    //     let v2 = position + chunk_v2;
                    //     let v3 = position + chunk_v3;
                    //     let v6 = position + chunk_v6;
                    //     let v7 = position + chunk_v7;
                    //     Self::add_quad(vertices, indices, normals, [v3, v6, v7, v2], VECTOR_BACK);
                    // }
                    // if has_back {
                    //     let v0 = position + chunk_v0;
                    //     let v1 = position + chunk_v1;
                    //     let v4 = position + chunk_v4;
                    //     let v5 = position + chunk_v5;
                    //     Self::add_quad(
                    //         vertices,
                    //         indices,
                    //         normals,
                    //         [v1, v4, v5, v0],
                    //         VECTOR_FORWARD,
                    //     );
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

        self.generate_mesh_arrays(&mut vertices, &mut normals, &mut indices, Vec3::ZERO);

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
