use crate::math::Freal;
use crate::voxtree::calculate_voxels_per_axis;
use crate::voxtree::VoxTree;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;

const CUBE_VERTS: [bevy::math::Vec3; 8] = [
    bevy::math::Vec3::new(-1.0, 1.0, -1.0),
    bevy::math::Vec3::new(1.0, 1.0, -1.0),
    bevy::math::Vec3::new(1.0, 1.0, 1.0),
    bevy::math::Vec3::new(-1.0, 1.0, 1.0),
    bevy::math::Vec3::new(-1.0, -1.0, -1.0),
    bevy::math::Vec3::new(1.0, -1.0, -1.0),
    bevy::math::Vec3::new(1.0, -1.0, 1.0),
    bevy::math::Vec3::new(-1.0, -1.0, 1.0),
];

const VECTOR_UP: Vec3 = bevy::math::Vec3::new(0.0, 1.0, 0.0);
const VECTOR_RIGHT: Vec3 = bevy::math::Vec3::new(1.0, 0.0, 0.0);
const VECTOR_DOWN: Vec3 = bevy::math::Vec3::new(0.0, -1.0, 0.0);
const VECTOR_LEFT: Vec3 = bevy::math::Vec3::new(-1.0, 0.0, 0.0);
const VECTOR_FORWARD: Vec3 = bevy::math::Vec3::new(0.0, 0.0, -1.0);
const VECTOR_BACK: Vec3 = bevy::math::Vec3::new(0.0, 0.0, 1.0);

pub const MAX_LOD_LEVEL: usize = 7;
pub const VOXELS_PER_AXIS: u8 = calculate_voxels_per_axis(MAX_LOD_LEVEL) as u8;
pub const VOXELS_PER_AXIS_MINUS_ONE: u8 = VOXELS_PER_AXIS - 1;
pub const VOXEL_SIZE: Freal = 1.0 / VOXELS_PER_AXIS as Freal;

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

    pub fn generate_data(&mut self) {
        for y in 0..VOXELS_PER_AXIS {
            for z in 0..VOXELS_PER_AXIS {
                for x in 0..VOXELS_PER_AXIS {
                    self.data.set_value(0, x, y, z, (x + y + z) as i32);
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

    pub fn generate_mesh_arrays(
        &self,
        vertices: &mut Vec<bevy::math::Vec3>,
        normals: &mut Vec<bevy::math::Vec3>,
        indices: &mut Vec<u32>,
    ) {
        let voxels_per_axis = VOXELS_PER_AXIS as f32;
        let tile_size = bevy::math::Vec3::new(1.0, 1.0, 1.0) / voxels_per_axis;
        let tile_half_size = tile_size / 2.0;

        let chunk_v0 = CUBE_VERTS[0] * tile_half_size + tile_half_size;
        let chunk_v1 = CUBE_VERTS[1] * tile_half_size + tile_half_size;
        let chunk_v2 = CUBE_VERTS[2] * tile_half_size + tile_half_size;
        let chunk_v3 = CUBE_VERTS[3] * tile_half_size + tile_half_size;
        let chunk_v4 = CUBE_VERTS[4] * tile_half_size + tile_half_size;
        let chunk_v5 = CUBE_VERTS[5] * tile_half_size + tile_half_size;
        let chunk_v6 = CUBE_VERTS[6] * tile_half_size + tile_half_size;
        let chunk_v7 = CUBE_VERTS[7] * tile_half_size + tile_half_size;

        for y in 0..VOXELS_PER_AXIS {
            let is_top = y == VOXELS_PER_AXIS_MINUS_ONE;
            let is_bottom = y == 0;

            for z in 0..VOXELS_PER_AXIS {
                let is_front = z == VOXELS_PER_AXIS_MINUS_ONE;
                let is_back = z == 0;

                for x in 0..VOXELS_PER_AXIS {
                    let value = self.data.get_value(0, x, y, z);

                    if value == 0 {
                        continue;
                    }

                    let position = bevy::math::Vec3::new(
                        x as f32 * tile_size.x,
                        y as f32 * tile_size.y,
                        z as f32 * tile_size.z,
                    );

                    let v0 = position + chunk_v0;
                    let v1 = position + chunk_v1;
                    let v2 = position + chunk_v2;
                    let v3 = position + chunk_v3;
                    let v4 = position + chunk_v4;
                    let v5 = position + chunk_v5;
                    let v6 = position + chunk_v6;
                    let v7 = position + chunk_v7;

                    let is_right = x == VOXELS_PER_AXIS_MINUS_ONE;
                    let is_left = x == 0;

                    let has_top = if is_top {
                        true
                    } else {
                        self.data.get_value(0, x, y + 1, z) == 0
                    };
                    let has_bottom = if is_bottom {
                        true
                    } else {
                        self.data.get_value(0, x, y - 1, z) == 0
                    };
                    let has_left = if is_left {
                        true
                    } else {
                        self.data.get_value(0, x - 1, y, z) == 0
                    };
                    let has_right = if is_right {
                        true
                    } else {
                        self.data.get_value(0, x + 1, y, z) == 0
                    };
                    let has_back = if is_back {
                        true
                    } else {
                        self.data.get_value(0, x, y, z - 1) == 0
                    };
                    let has_front = if is_front {
                        true
                    } else {
                        self.data.get_value(0, x, y, z + 1) == 0
                    };
                    let has_something =
                        has_top || has_bottom || has_left || has_right || has_back || has_front;

                    if !has_something {
                        continue;
                    }

                    if has_top {
                        Self::add_quad(vertices, indices, normals, &v0, &v2, &v3, &v1, &VECTOR_UP);
                    }
                    if has_right {
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            &v2,
                            &v5,
                            &v6,
                            &v1,
                            &VECTOR_RIGHT,
                        );
                    }
                    if has_bottom {
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            &v7,
                            &v5,
                            &v4,
                            &v6,
                            &VECTOR_DOWN,
                        );
                    }
                    if has_left {
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            &v0,
                            &v7,
                            &v4,
                            &v3,
                            &VECTOR_LEFT,
                        );
                    }
                    if has_front {
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            &v3,
                            &v6,
                            &v7,
                            &v2,
                            &VECTOR_BACK,
                        );
                    }
                    if has_back {
                        Self::add_quad(
                            vertices,
                            indices,
                            normals,
                            &v1,
                            &v4,
                            &v5,
                            &v0,
                            &VECTOR_FORWARD,
                        );
                    }
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

        self.generate_mesh_arrays(&mut vertices, &mut normals, &mut indices);

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

    fn add_quad(
        vertices: &mut Vec<bevy::math::Vec3>,
        indices: &mut Vec<u32>,
        normals: &mut Vec<bevy::math::Vec3>,
        v0: &bevy::math::Vec3,
        v1: &bevy::math::Vec3,
        v2: &bevy::math::Vec3,
        v3: &bevy::math::Vec3,
        normal: &bevy::math::Vec3,
    ) {
        let index = vertices.len() as u32;

        vertices.push(*v0);
        vertices.push(*v1);
        vertices.push(*v2);
        vertices.push(*v3);

        normals.push(*normal);
        normals.push(*normal);
        normals.push(*normal);
        normals.push(*normal);

        indices.push(index + 2);
        indices.push(index + 1);
        indices.push(index);
        indices.push(index + 3);
        indices.push(index);
        indices.push(index + 1);
    }
}
