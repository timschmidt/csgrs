use glam::IVec3;

use crate::VoxelTrait;

use super::VoxChunk;

#[derive(Default)]
pub struct VoxWorld<T: VoxelTrait> {
    pub chunks_size: IVec3,
    pub chunks_len: usize,
    pub chunks: Vec<VoxChunk<T>>,
}

impl<T: VoxelTrait> VoxWorld<T> {
    pub fn new() -> Self {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxWorld::new");

        let chunks_size = IVec3::new(32, 32, 32);
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Vec::with_capacity(chunks_len);

        Self {
            chunks_size,
            chunks_len,
            chunks,
        }
    }

    pub fn with_size(size: IVec3) -> Self {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxWorld::with_size");

        let chunks_len = size.x as usize * size.y as usize * size.z as usize;
        let chunks = Vec::with_capacity(chunks_len);

        Self {
            chunks_size: size,
            chunks_len,
            chunks,
        }
    }

    pub fn clear(&mut self) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxWorld::clear");

        self.chunks.clear();
    }

    pub fn resize(&mut self, size: IVec3) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxWorld::resize");

        self.chunks_size = size;
        self.chunks_len = size.x as usize * size.y as usize * size.z as usize;
        self.chunks = Vec::with_capacity(self.chunks_len);
    }
}
