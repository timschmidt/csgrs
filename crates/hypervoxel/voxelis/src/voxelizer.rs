use std::time::Instant;

use bevy::math::IVec3;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::math::triangle_cube_intersection;
use crate::math::Freal;
use crate::math::Vec3;
use crate::obj_reader::Obj;
use crate::Chunk;

use crate::chunk::INV_VOXEL_SIZE;
use crate::chunk::VOXELS_PER_AXIS;
use crate::chunk::VOXEL_SIZE;

// Helper function to calculate chunk index from coordinates
fn calculate_chunk_index_from_coords(x: i32, y: i32, z: i32, chunks_size: IVec3) -> usize {
    let chunks_area = chunks_size.x * chunks_size.z;
    let chunk_index = y * chunks_area + z * chunks_size.x + x;
    chunk_index as usize
}

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

pub struct Voxelizer {
    pub mesh: Obj,
    pub chunks_size: IVec3,
    pub chunks_len: usize,
    pub chunks: Vec<Chunk>,
}

impl Voxelizer {
    pub fn new(mesh: Obj) -> Self {
        let chunks_size_x = (mesh.size.x.ceil() as i32) + 1;
        let chunks_size_y = (mesh.size.y.ceil() as i32) + 1;
        let chunks_size_z = (mesh.size.z.ceil() as i32) + 1;

        let chunks_size = IVec3::new(chunks_size_x, chunks_size_y, chunks_size_z);

        Self {
            mesh,
            chunks_size,
            chunks_len: 0,
            chunks: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        for chunk in self.chunks.iter_mut() {
            chunk.clear();
        }
    }

    pub fn prepare_chunks(&mut self) {
        self.chunks_len =
            self.chunks_size.x as usize * self.chunks_size.y as usize * self.chunks_size.z as usize;
        self.chunks = Vec::with_capacity(self.chunks_len);

        println!("Initializing {} chunks", self.chunks_len);

        // Initialize chunks
        for y in 0..self.chunks_size.y {
            for z in 0..self.chunks_size.z {
                for x in 0..self.chunks_size.x {
                    self.chunks.push(Chunk::with_position(x, y, z));
                }
            }
        }
    }

    pub fn build_face_to_chunk_map(&mut self) -> FxHashMap<usize, Vec<IVec3>> {
        let mut chunk_face_map: FxHashMap<usize, Vec<IVec3>> = FxHashMap::default();

        let mesh_min = self.mesh.aabb.0;

        for face in &self.mesh.faces {
            let v1 = self.mesh.vertices[(face.x - 1) as usize] - mesh_min;
            let v2 = self.mesh.vertices[(face.y - 1) as usize] - mesh_min;
            let v3 = self.mesh.vertices[(face.z - 1) as usize] - mesh_min;

            let min = v1.min(v2).min(v3);
            let max = v1.max(v2).max(v3);

            let world_min_voxel = (min * INV_VOXEL_SIZE).floor().as_ivec3();
            let world_max_voxel = (max * INV_VOXEL_SIZE).ceil().as_ivec3();

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
                            self.chunks_size,
                        );

                        if chunk_index < self.chunks_len {
                            chunk_face_map.entry(chunk_index).or_default().push(*face);
                        }
                    }
                }
            }
        }

        chunk_face_map
    }

    pub fn voxelize_mesh(&mut self, chunk_face_map: &FxHashMap<usize, Vec<IVec3>>) {
        let epsilon = VOXEL_SIZE * 1e-7;
        let splat = Vec3::splat(epsilon);

        let mesh_min = self.mesh.aabb.0;

        self.chunks
            .par_iter_mut()
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                if let Some(faces) = chunk_face_map.get(&chunk_index) {
                    if faces.is_empty() {
                        return;
                    }

                    let chunk_position = chunk.get_position();
                    let chunk_world_position = IVec3::new(
                        chunk_position.x * VOXELS_PER_AXIS as i32,
                        chunk_position.y * VOXELS_PER_AXIS as i32,
                        chunk_position.z * VOXELS_PER_AXIS as i32,
                    );

                    // Compute the chunk's world bounding box
                    let chunk_world_min = Vec3::new(
                        chunk_world_position.x as Freal * VOXEL_SIZE,
                        chunk_world_position.y as Freal * VOXEL_SIZE,
                        chunk_world_position.z as Freal * VOXEL_SIZE,
                    );
                    let chunk_world_max =
                        chunk_world_min + Vec3::splat(VOXELS_PER_AXIS as Freal * VOXEL_SIZE);

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
                        let local_min_voxel = ((overlap_min - chunk_world_min) / VOXEL_SIZE)
                            .floor()
                            .as_ivec3();
                        let local_max_voxel = ((overlap_max - chunk_world_min) / VOXEL_SIZE)
                            .ceil()
                            .as_ivec3();

                        // Clamp voxel indices to valid range [0, VOXELS_PER_AXIS - 1]
                        let local_min_voxel = local_min_voxel
                            .clamp(IVec3::ZERO, IVec3::splat(VOXELS_PER_AXIS as i32 - 1));
                        let local_max_voxel = local_max_voxel
                            .clamp(IVec3::ZERO, IVec3::splat(VOXELS_PER_AXIS as i32 - 1));

                        // Iterate over the voxels within the overlapping region
                        for y in local_min_voxel.y..=local_max_voxel.y {
                            for z in local_min_voxel.z..=local_max_voxel.z {
                                for x in local_min_voxel.x..=local_max_voxel.x {
                                    // Compute world position of the voxel
                                    let world_voxel_position = chunk_world_min
                                        + Vec3::new(x as Freal, y as Freal, z as Freal)
                                            * VOXEL_SIZE;

                                    // Expand voxel bounds slightly by epsilon (if needed)
                                    let world_min_position = world_voxel_position - splat;
                                    let world_max_position =
                                        world_voxel_position + Vec3::splat(VOXEL_SIZE) + splat;

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
        // Update LODs in parallel
        self.chunks.par_iter_mut().for_each(|chunk| {
            if !chunk.is_empty() {
                chunk.update_lods();
            }
        });
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
                let voxel = (vertex * INV_VOXEL_SIZE).floor().as_ivec3();
                let local_voxel = convert_voxel_world_to_local(voxel);

                let chunk_index = calculate_chunk_index(voxel, chunks_size, chunks_len);
                let chunk = &mut self.chunks[chunk_index];

                chunk.set_value(
                    local_voxel.x as u8,
                    local_voxel.y as u8,
                    local_voxel.z as u8,
                    1,
                );
            }
        }

        // Update LODs in parallel
        self.chunks.par_iter_mut().for_each(|chunk| {
            if !chunk.is_empty() {
                chunk.update_lods();
            }
        });

        println!("Simple voxelize took: {:?}", now.elapsed());
    }

    pub fn voxelize(&mut self) {
        println!("Voxelize started");

        self.prepare_chunks();

        let face_to_chunk_map_time = Instant::now();

        println!("Building face-to-chunk mapping");

        // Build face-to-chunk mapping
        let chunk_face_map = self.build_face_to_chunk_map();

        let face_to_chunk_map_time = face_to_chunk_map_time.elapsed();

        let voxelize_time = Instant::now();

        self.voxelize_mesh(&chunk_face_map);

        let voxelize_time = voxelize_time.elapsed();

        println!(
            "Voxelize finished, updating LODs for {} chunks",
            self.chunks.len()
        );

        let update_lods_time = Instant::now();

        self.update_lods();

        let empty_chunks = self
            .chunks
            .par_iter()
            .filter(|chunk| chunk.is_empty())
            .count();

        let update_lods_time = update_lods_time.elapsed();
        let total = face_to_chunk_map_time + voxelize_time + update_lods_time;

        println!(
            "Done, {} chunks, empty: {}, face-to-chunk: {:?}, voxelized: {:?}, update lods: {:?}, total: {:?}",
            self.chunks.len(),
            empty_chunks,
            face_to_chunk_map_time,
            voxelize_time,
            update_lods_time,
            total
        );
    }
}
