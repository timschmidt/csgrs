use bevy::math::IVec3;
use rayon::prelude::*;

use crate::chunk::Chunk;

#[derive(Default)]
pub struct Model {
    pub chunks_size: IVec3,
    pub chunks_len: usize,
    pub chunks: Vec<Chunk>,
}

impl Model {
    pub fn new() -> Self {
        let chunks_size = IVec3::new(32, 32, 32);
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(chunks_size, chunks_len);

        Self {
            chunks_size,
            chunks_len,
            chunks,
        }
    }

    pub fn with_size(chunks_size: IVec3) -> Self {
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(chunks_size, chunks_len);

        Self {
            chunks_size,
            chunks_len,
            chunks,
        }
    }

    pub fn clear(&mut self) {
        self.chunks.clear();
    }

    pub fn resize(&mut self, size: IVec3) {
        self.chunks_size = size;
        self.chunks_len = size.x as usize * size.y as usize * size.z as usize;
        self.chunks = Self::init_chunks(self.chunks_size, self.chunks_len);
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

    fn init_chunks(size: IVec3, len: usize) -> Vec<Chunk> {
        let mut chunks = Vec::with_capacity(len);

        for y in 0..size.y {
            for z in 0..size.z {
                for x in 0..size.x {
                    chunks.push(Chunk::with_position(x, y, z));
                }
            }
        }

        chunks
    }

    pub fn deserialize(&mut self, data: &[u8], offsets: &[usize]) {
        self.chunks
            .par_iter_mut()
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                chunk.deserialize(data, chunk_index, offsets);
            });
    }
}
