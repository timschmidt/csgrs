use std::time::Instant;

use glam::{DVec3, IVec3};
// use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::core::triangle_cube_intersection;
use crate::io::Obj;
use crate::model::Model;

// Helper function to calculate chunk index from coordinates
fn calculate_chunk_index_from_coords(x: i32, y: i32, z: i32, chunks_size: IVec3) -> usize {
    let chunks_area = chunks_size.x * chunks_size.z;
    let chunk_index = y * chunks_area + z * chunks_size.x + x;
    chunk_index as usize
}

fn calculate_chunk_index(
    voxels_per_axis: i32,
    world_voxel_position: IVec3,
    chunks_size: IVec3,
    chunks_len: usize,
) -> usize {
    let chunk_x = world_voxel_position.x / voxels_per_axis;
    let chunk_y = world_voxel_position.y / voxels_per_axis;
    let chunk_z = world_voxel_position.z / voxels_per_axis;

    let chunks_area = chunks_size.x * chunks_size.z;

    let chunk_index = chunk_y * chunks_area + chunk_z * chunks_size.x + chunk_x;

    assert!(chunk_index < chunks_len as i32);

    chunk_index as usize
}

fn convert_voxel_world_to_local(voxels_per_axis: i32, current_min_voxel: IVec3) -> IVec3 {
    let chunk_x = current_min_voxel.x % voxels_per_axis;
    let chunk_y = current_min_voxel.y % voxels_per_axis;
    let chunk_z = current_min_voxel.z % voxels_per_axis;

    IVec3::new(chunk_x, chunk_y, chunk_z)
}

pub struct Voxelizer {
    pub mesh: Obj,
    pub model: Model,
}

impl Voxelizer {
    pub fn new(max_depth: usize, chunk_size: f32, mesh: Obj) -> Self {
        let chunks_size_x = (mesh.size.x.ceil() as i32) + 1;
        let chunks_size_y = (mesh.size.y.ceil() as i32) + 1;
        let chunks_size_z = (mesh.size.z.ceil() as i32) + 1;

        let chunks_size = IVec3::new(chunks_size_x, chunks_size_y, chunks_size_z);

        Self {
            mesh,
            model: Model::with_size(max_depth, chunk_size, chunks_size),
        }
    }

    pub fn clear(&mut self) {
        self.model.clear();
    }

    pub fn build_face_to_chunk_map(&mut self) -> FxHashMap<usize, Vec<IVec3>> {
        let mut chunk_face_map: FxHashMap<usize, Vec<IVec3>> = FxHashMap::default();

        let mesh_min = self.mesh.aabb.0;

        let voxels_per_axis = self.model.voxels_per_axis();
        let voxel_size: f64 = 1.0 / voxels_per_axis as f64;
        let inv_voxel_size: f64 = 1.0 / voxel_size;

        for face in &self.mesh.faces {
            let v1 = self.mesh.vertices[(face.x - 1) as usize] - mesh_min;
            let v2 = self.mesh.vertices[(face.y - 1) as usize] - mesh_min;
            let v3 = self.mesh.vertices[(face.z - 1) as usize] - mesh_min;

            let min = v1.min(v2).min(v3);
            let max = v1.max(v2).max(v3);

            let world_min_voxel = (min * inv_voxel_size).floor().as_ivec3();
            let world_max_voxel = (max * inv_voxel_size).ceil().as_ivec3();

            // Determine which chunks this face overlaps
            let min_chunk = world_min_voxel / voxels_per_axis as i32;
            let max_chunk = world_max_voxel / voxels_per_axis as i32;

            for chunk_y in min_chunk.y..=max_chunk.y {
                for chunk_z in min_chunk.z..=max_chunk.z {
                    for chunk_x in min_chunk.x..=max_chunk.x {
                        let chunk_index = calculate_chunk_index_from_coords(
                            chunk_x,
                            chunk_y,
                            chunk_z,
                            self.model.chunks_size,
                        );

                        if chunk_index < self.model.chunks_len {
                            chunk_face_map.entry(chunk_index).or_default().push(*face);
                        }
                    }
                }
            }
        }

        chunk_face_map
    }

    pub fn voxelize_mesh(&mut self, chunk_face_map: &FxHashMap<usize, Vec<IVec3>>) {
        let voxels_per_axis = self.model.voxels_per_axis();
        let voxel_size: f64 = 1.0 / voxels_per_axis as f64;

        let epsilon = voxel_size * 1e-7;
        let splat = DVec3::splat(epsilon);

        let mesh_min = self.mesh.aabb.0;
        let voxels_per_axis = self.model.voxels_per_axis();

        self.model
            .chunks
            .iter_mut()
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                if let Some(faces) = chunk_face_map.get(&chunk_index) {
                    if faces.is_empty() {
                        return;
                    }

                    let chunk_position = chunk.get_position();
                    let chunk_world_position = DVec3::new(
                        chunk_position.x as f64 * voxels_per_axis as f64,
                        chunk_position.y as f64 * voxels_per_axis as f64,
                        chunk_position.z as f64 * voxels_per_axis as f64,
                    );

                    // Compute the chunk's world bounding box
                    let chunk_world_min = DVec3::new(
                        chunk_world_position.x * voxel_size,
                        chunk_world_position.y * voxel_size,
                        chunk_world_position.z * voxel_size,
                    );
                    let chunk_world_max =
                        chunk_world_min + DVec3::splat(voxels_per_axis as f64 * voxel_size);

                    for face in faces.iter() {
                        let v1 = self.mesh.vertices[(face.x - 1) as usize] - mesh_min;
                        let v2 = self.mesh.vertices[(face.y - 1) as usize] - mesh_min;
                        let v3 = self.mesh.vertices[(face.z - 1) as usize] - mesh_min;

                        // Compute the face's bounding box in world coordinates
                        let face_min = v1.min(v2).min(v3);
                        let face_max = v1.max(v2).max(v3);

                        // Compute the overlapping region between the face and the chunk
                        let overlap_min = face_min.max(chunk_world_min) - splat;
                        let overlap_max = face_max.min(chunk_world_max) + splat;

                        // Check if there is any overlap
                        if overlap_min.x >= overlap_max.x
                            || overlap_min.y >= overlap_max.y
                            || overlap_min.z >= overlap_max.z
                        {
                            // No overlap with the current chunk, skip to the next face
                            continue;
                        }

                        // Map the overlapping region to voxel indices within the chunk
                        let local_min_voxel = ((overlap_min - chunk_world_min) / voxel_size)
                            .floor()
                            .as_ivec3();
                        let local_max_voxel = ((overlap_max - chunk_world_min) / voxel_size)
                            .ceil()
                            .as_ivec3();

                        // Clamp voxel indices to valid range [0, VOXELS_PER_AXIS - 1]
                        let local_min_voxel = local_min_voxel
                            .clamp(IVec3::ZERO, IVec3::splat(voxels_per_axis as i32 - 1));
                        let local_max_voxel = local_max_voxel
                            .clamp(IVec3::ZERO, IVec3::splat(voxels_per_axis as i32 - 1));

                        // Iterate over the voxels within the overlapping region
                        for y in local_min_voxel.y..=local_max_voxel.y {
                            for z in local_min_voxel.z..=local_max_voxel.z {
                                for x in local_min_voxel.x..=local_max_voxel.x {
                                    // Compute world position of the voxel
                                    let world_voxel_position = chunk_world_min
                                        + DVec3::new(x as f64, y as f64, z as f64) * voxel_size;

                                    // Expand voxel bounds slightly by epsilon (if needed)
                                    let world_min_position = world_voxel_position - splat;
                                    let world_max_position =
                                        world_voxel_position + DVec3::splat(voxel_size) + splat;

                                    // Perform the intersection test
                                    if triangle_cube_intersection(
                                        (v1, v2, v3),
                                        (world_min_position, world_max_position),
                                    ) {
                                        chunk.set_value(x as u8, y as u8, z as u8, 1);
                                    }
                                }
                            }
                        }
                    }
                }
            });
    }

    pub fn update_lods(&mut self) {
        // self.model.update_lods();
    }

    pub fn simple_voxelize(&mut self) {
        let chunks_size = self.model.chunks_size;
        let chunks_len = self.model.chunks_len;

        let voxels_per_axis = self.model.voxels_per_axis();
        let voxel_size: f64 = 1.0 / voxels_per_axis as f64;
        let inv_voxel_size: f64 = 1.0 / voxel_size;

        let now = Instant::now();

        let mesh_min = self.mesh.aabb.0;

        for face in self.mesh.faces.iter() {
            for vertex_index in [face.x, face.y, face.z] {
                let vertex = self.mesh.vertices[(vertex_index - 1) as usize] - mesh_min;
                let voxel = (vertex * inv_voxel_size).floor().as_ivec3();
                let local_voxel = convert_voxel_world_to_local(voxels_per_axis as i32, voxel);

                let chunk_index =
                    calculate_chunk_index(voxels_per_axis as i32, voxel, chunks_size, chunks_len);
                let chunk = &mut self.model.chunks[chunk_index];

                chunk.set_value(
                    local_voxel.x as u8,
                    local_voxel.y as u8,
                    local_voxel.z as u8,
                    1,
                );
            }
        }

        self.update_lods();

        println!("Simple voxelize took: {:?}", now.elapsed());
    }

    pub fn voxelize(&mut self) {
        println!("Voxelize started");

        let face_to_chunk_map_time = Instant::now();

        println!("Building face-to-chunk mapping");

        // Build face-to-chunk mapping
        let chunk_face_map = self.build_face_to_chunk_map();

        let face_to_chunk_map_time = face_to_chunk_map_time.elapsed();

        let voxelize_time = Instant::now();

        println!("Voxelizing mesh");

        self.voxelize_mesh(&chunk_face_map);

        let voxelize_time = voxelize_time.elapsed();

        println!(
            "Voxelize finished, updating LODs for {} chunks",
            self.model.chunks.len()
        );

        let update_lods_time = Instant::now();

        self.update_lods();

        let empty_chunks = self
            .model
            .chunks
            .iter()
            .filter(|chunk| chunk.is_empty())
            .count();

        let update_lods_time = update_lods_time.elapsed();
        let total = face_to_chunk_map_time + voxelize_time + update_lods_time;

        println!(
            "Done, {} chunks, empty: {}, face-to-chunk: {:?}, voxelized: {:?}, update lods: {:?}, total: {:?}",
            self.model.chunks.len(),
            empty_chunks,
            face_to_chunk_map_time,
            voxelize_time,
            update_lods_time,
            total
        );
    }
}
