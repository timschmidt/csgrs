use std::time::Instant;

use bevy::math::IVec3;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::math::triangle_cube_intersection;
use crate::math::Freal;
use crate::math::Vec3;
use crate::obj_reader::Obj;
use crate::Chunk;

use crate::chunk::VOXELS_PER_AXIS;
use crate::chunk::VOXEL_SIZE;

pub struct Voxelizer {
    pub mesh: Obj,
    pub chunks_size: IVec3,
    pub chunks: Vec<Chunk>,
}

// Helper function to calculate chunk index from coordinates
fn calculate_chunk_index_from_coords(x: i32, y: i32, z: i32, chunks_size: IVec3) -> usize {
    let chunks_area = chunks_size.x * chunks_size.z;
    let chunk_index = y * chunks_area + z * chunks_size.x + x;
    chunk_index as usize
}

impl Voxelizer {
    fn calculate_chunk_index(
        world_voxel_position: IVec3,
        chunks_size: IVec3,
        chunks_len: usize,
    ) -> usize {
        let chunk_x = world_voxel_position.x / VOXELS_PER_AXIS as i32;
        let chunk_y = world_voxel_position.y / VOXELS_PER_AXIS as i32;
        let chunk_z = world_voxel_position.z / VOXELS_PER_AXIS as i32;

        let chunks_area = chunks_size.x * chunks_size.z;

        let chunk_index = chunk_y * chunks_area + chunk_z * chunks_size.x + chunk_x;

        assert!(chunk_index < chunks_len as i32);

        chunk_index as usize
    }

    fn convert_voxel_world_to_local(current_min_voxel: IVec3) -> IVec3 {
        let chunk_x = current_min_voxel.x % VOXELS_PER_AXIS as i32;
        let chunk_y = current_min_voxel.y % VOXELS_PER_AXIS as i32;
        let chunk_z = current_min_voxel.z % VOXELS_PER_AXIS as i32;

        IVec3::new(chunk_x, chunk_y, chunk_z)
    }

    pub fn new(mesh: Obj) -> Self {
        let chunks_size_x = (mesh.size.x.ceil() as i32) + 1;
        let chunks_size_y = (mesh.size.y.ceil() as i32) + 1;
        let chunks_size_z = (mesh.size.z.ceil() as i32) + 1;

        let chunks_size = IVec3::new(chunks_size_x, chunks_size_y, chunks_size_z);

        Self {
            mesh,
            chunks_size,
            chunks: Vec::new(),
        }
    }

    pub fn simple_voxelize(&mut self) {
        let chunks_len =
            self.chunks_size.x as usize * self.chunks_size.y as usize * self.chunks_size.z as usize;
        let chunks_size = self.chunks_size;
        self.chunks = Vec::with_capacity(chunks_len);

        for y in 0..self.chunks_size.y {
            for z in 0..self.chunks_size.z {
                for x in 0..self.chunks_size.x {
                    self.chunks.push(Chunk::with_position(x, y, z));
                }
            }
        }

        let now = Instant::now();

        let mesh_min = self.mesh.aabb.0;

        for face in self.mesh.faces.iter() {
            for vertex_index in [face.x, face.y, face.z] {
                let vertex = self.mesh.vertices[(vertex_index - 1) as usize] - mesh_min;
                let voxel = (vertex / VOXEL_SIZE).floor().as_ivec3();
                let local_voxel = Self::convert_voxel_world_to_local(voxel);

                let chunk_index = Self::calculate_chunk_index(voxel, chunks_size, chunks_len);
                let chunk = &mut self.chunks[chunk_index];

                chunk.set_value(
                    local_voxel.x as u8,
                    local_voxel.y as u8,
                    local_voxel.z as u8,
                    1,
                );
            }
        }

        for chunk in self.chunks.iter_mut() {
            chunk.update_lods();
        }

        println!("Simple voxelize took: {:?}", now.elapsed());
    }

    pub fn voxelize(&mut self) {
        let chunks_len =
            self.chunks_size.x as usize * self.chunks_size.y as usize * self.chunks_size.z as usize;
        let chunks_size = self.chunks_size;
        self.chunks = Vec::with_capacity(chunks_len);

        // Initialize chunks
        for y in 0..self.chunks_size.y {
            for z in 0..self.chunks_size.z {
                for x in 0..self.chunks_size.x {
                    self.chunks.push(Chunk::with_position(x, y, z));
                }
            }
        }

        let mesh_min = self.mesh.aabb.0;

        println!("Voxelize started");

        let face_to_chunk_map_now = Instant::now();

        println!("Building face-to-chunk mapping");

        // Build face-to-chunk mapping
        let mut chunk_face_map: FxHashMap<usize, Vec<IVec3>> = FxHashMap::default();

        for face in &self.mesh.faces {
            let v1 = self.mesh.vertices[(face.x - 1) as usize] - mesh_min;
            let v2 = self.mesh.vertices[(face.y - 1) as usize] - mesh_min;
            let v3 = self.mesh.vertices[(face.z - 1) as usize] - mesh_min;

            let min = v1.min(v2).min(v3);
            let max = v1.max(v2).max(v3);

            let world_min_voxel = (min / VOXEL_SIZE).floor().as_ivec3();
            let world_max_voxel = (max / VOXEL_SIZE).ceil().as_ivec3();

            // Determine which chunks this face overlaps
            let min_chunk = world_min_voxel / VOXELS_PER_AXIS as i32;
            let max_chunk = world_max_voxel / VOXELS_PER_AXIS as i32;

            for chunk_y in min_chunk.y..=max_chunk.y {
                for chunk_z in min_chunk.z..=max_chunk.z {
                    for chunk_x in min_chunk.x..=max_chunk.x {
                        let chunk_index = calculate_chunk_index_from_coords(
                            chunk_x,
                            chunk_y,
                            chunk_z,
                            chunks_size,
                        );

                        if chunk_index < chunks_len {
                            chunk_face_map.entry(chunk_index).or_default().push(*face);
                        }
                    }
                }
            }
        }

        let face_to_chunk_map_time = face_to_chunk_map_now.elapsed();

        println!("Voxelizing");

        let epsilon = VOXEL_SIZE * 0.001;
        let splat = Vec3::splat(epsilon);

        let now = Instant::now();

        self.chunks
            .par_iter_mut()
            .enumerate()
            .for_each(|(loop_chunk_index, loop_chunk)| {
                let chunk_position = loop_chunk.get_position();
                let chunk_world_position = IVec3::new(
                    chunk_position.x * VOXELS_PER_AXIS as i32,
                    chunk_position.y * VOXELS_PER_AXIS as i32,
                    chunk_position.z * VOXELS_PER_AXIS as i32,
                );

                if let Some(faces) = chunk_face_map.get(&loop_chunk_index) {
                    for face in faces.iter() {
                        let v1 = self.mesh.vertices[(face.x - 1) as usize] - mesh_min;
                        let v2 = self.mesh.vertices[(face.y - 1) as usize] - mesh_min;
                        let v3 = self.mesh.vertices[(face.z - 1) as usize] - mesh_min;

                        let min = v1.min(v2).min(v3);
                        let max = v1.max(v2).max(v3);

                        let world_min_voxel = min / VOXEL_SIZE;
                        let world_max_voxel = max / VOXEL_SIZE;

                        let world_min_voxel = world_min_voxel.as_ivec3();
                        let world_max_voxel = world_max_voxel.as_ivec3() + IVec3::splat(1);
                        let diff_voxel = world_max_voxel - world_min_voxel;

                        let mut affected_voxels = Vec::new();

                        let mut current_chunk_index =
                            Self::calculate_chunk_index(world_min_voxel, chunks_size, chunks_len);
                        let mut current_min_voxel = IVec3::MAX;
                        let mut current_max_voxel = IVec3::MIN;

                        for y in 0..diff_voxel.y {
                            for z in 0..diff_voxel.z {
                                for x in 0..diff_voxel.x {
                                    let world_voxel = world_min_voxel + IVec3::new(x, y, z);

                                    let chunk_index = Self::calculate_chunk_index(
                                        world_voxel,
                                        chunks_size,
                                        chunks_len,
                                    );

                                    if chunk_index != current_chunk_index
                                        && current_min_voxel != IVec3::MAX
                                    {
                                        affected_voxels.push((
                                            current_chunk_index,
                                            Self::convert_voxel_world_to_local(current_min_voxel),
                                            Self::convert_voxel_world_to_local(current_max_voxel),
                                        ));

                                        current_min_voxel = IVec3::MAX;
                                        current_max_voxel = IVec3::MIN;
                                    }

                                    current_chunk_index = chunk_index;
                                    current_min_voxel = current_min_voxel.min(world_voxel);
                                    current_max_voxel = current_max_voxel.max(world_voxel);
                                }
                            }
                        }

                        if current_min_voxel != IVec3::MAX {
                            affected_voxels.push((
                                current_chunk_index,
                                Self::convert_voxel_world_to_local(current_min_voxel),
                                Self::convert_voxel_world_to_local(current_max_voxel),
                            ));
                        }

                        let chunk_position = loop_chunk.get_position();
                        for (chunk_index, min_voxel, max_voxel) in affected_voxels.iter() {
                            if *chunk_index != loop_chunk_index {
                                continue;
                            }

                            for y in min_voxel.y..=max_voxel.y {
                                for z in min_voxel.z..=max_voxel.z {
                                    for x in min_voxel.x..=max_voxel.x {
                                        let world_voxel_position = Vec3::new(
                                            (chunk_world_position.x + x) as Freal * VOXEL_SIZE,
                                            (chunk_world_position.y + y) as Freal * VOXEL_SIZE,
                                            (chunk_world_position.z + z) as Freal * VOXEL_SIZE,
                                        );

                                        let world_min_position = world_voxel_position - splat;
                                        let world_max_position =
                                            world_voxel_position + Vec3::splat(VOXEL_SIZE) + splat;

                                        let intersects = triangle_cube_intersection(
                                            (v1, v2, v3),
                                            (world_min_position, world_max_position),
                                        );

                                        if intersects {
                                            loop_chunk.set_value(x as u8, y as u8, z as u8, 1);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            });
        let voxelize_time = now.elapsed();

        println!(
            "Voxelize finished, updating LODs for {} chunks",
            self.chunks.len()
        );

        let update_lods_now = Instant::now();

        // Update LODs in parallel
        self.chunks.par_iter_mut().for_each(|chunk| {
            chunk.update_lods();
        });

        let update_lods_time = update_lods_now.elapsed();
        let total = face_to_chunk_map_time + voxelize_time + update_lods_time;

        println!(
            "Done, {} chunks, face-to-chunk: {:?}, voxelized: {:?}, update lods: {:?}, total: {:?}",
            self.chunks.len(),
            face_to_chunk_map_time,
            voxelize_time,
            update_lods_time,
            total
        );
    }
}
