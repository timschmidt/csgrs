use glam::IVec3;
use rayon::prelude::*;

use crate::world::Chunk;

#[derive(Default)]
pub struct Model {
    pub max_depth: usize,
    pub voxels_per_axis: usize,
    pub chunk_size: f32,
    pub chunks_size: IVec3,
    pub chunks_len: usize,
    pub chunks: Vec<Chunk>,
}

impl Model {
    pub fn new(max_depth: usize, chunk_size: f32) -> Self {
        let chunks_size = IVec3::new(32, 32, 32);
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(max_depth, chunk_size, chunks_size, chunks_len);
        let voxels_per_axis = 1 << max_depth;

        Self {
            max_depth,
            voxels_per_axis,
            chunk_size,
            chunks_size,
            chunks_len,
            chunks,
        }
    }

    pub fn with_size(max_depth: usize, chunk_size: f32, chunks_size: IVec3) -> Self {
        println!(
            "Creating model with size {:?}, chunk: {} depth: {}",
            chunks_size, chunk_size, max_depth
        );
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(max_depth, chunk_size, chunks_size, chunks_len);
        let voxels_per_axis = 1 << max_depth;

        Self {
            max_depth,
            voxels_per_axis,
            chunk_size,
            chunks_size,
            chunks_len,
            chunks,
        }
    }

    pub fn clear(&mut self) {
        self.chunks_size = IVec3::ZERO;
        self.chunks_len = 0;
        self.chunks.clear();
    }

    pub fn resize(&mut self, size: IVec3) {
        self.chunks.clear();

        self.chunks_size = size;
        self.chunks_len = size.x as usize * size.y as usize * size.z as usize;
        self.chunks = Self::init_chunks(
            self.max_depth,
            self.chunk_size,
            self.chunks_size,
            self.chunks_len,
        );
    }

    fn init_chunks(max_depth: usize, chunk_size: f32, size: IVec3, len: usize) -> Vec<Chunk> {
        let mut chunks = Vec::with_capacity(len);

        for y in 0..size.y {
            for z in 0..size.z {
                for x in 0..size.x {
                    chunks.push(Chunk::with_position(chunk_size, max_depth, x, y, z));
                }
            }
        }

        chunks
    }

    pub fn serialize(&self, data: &mut Vec<u8>, sizes: &mut Vec<u32>) {
        const BUFFER_SIZE: usize = 1024 * 256;

        let chunks_data: Vec<Vec<u8>> = self
            .chunks
            .par_iter()
            .map(|chunk| {
                let mut buffer = Vec::with_capacity(BUFFER_SIZE);
                chunk.serialize(&mut buffer);
                buffer
            })
            .collect();

        for chunk_data in chunks_data.iter() {
            sizes.push(chunk_data.len().try_into().unwrap());
            data.extend(chunk_data);
        }
    }

    pub fn deserialize(&mut self, data: &[u8], offsets: &[usize]) {
        self.chunks
            .par_iter_mut()
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                chunk.deserialize(data, chunk_index, offsets);
            });
    }

    pub fn max_depth(&self) -> usize {
        self.max_depth
    }

    pub fn voxels_per_axis(&self) -> usize {
        self.voxels_per_axis
    }
}
