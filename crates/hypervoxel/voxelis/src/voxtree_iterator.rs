use rustc_hash::FxHashMap;

use crate::{
    chunk::VOXELS_PER_AXIS,
    voxtree::{calculate_lod_data_index, calculate_voxels_per_axis},
};

pub struct VoxTreeIterator<'a, const MAX_LOD_LEVEL: usize> {
    data: &'a FxHashMap<usize, i32>,
    index: usize,
}

impl<'a, const MAX_LOD_LEVEL: usize> VoxTreeIterator<'a, MAX_LOD_LEVEL> {
    const VOXELS_PER_AXIS: usize = calculate_voxels_per_axis(MAX_LOD_LEVEL);
    const MAX_SIZE: usize = Self::VOXELS_PER_AXIS * Self::VOXELS_PER_AXIS * Self::VOXELS_PER_AXIS;
    const MIN_INDEX: usize = calculate_lod_data_index(0, MAX_LOD_LEVEL);
    const MAX_INDEX: usize = Self::MIN_INDEX + Self::MAX_SIZE;

    pub fn new(data: &'a FxHashMap<usize, i32>) -> Self {
        Self {
            data,
            index: Self::MIN_INDEX,
        }
    }
}

impl<'a, const MAX_LOD_LEVEL: usize> Iterator for VoxTreeIterator<'a, MAX_LOD_LEVEL> {
    type Item = i32;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= Self::MAX_INDEX {
            return None;
        }

        let value = *self.data.get(&self.index).unwrap_or(&0);
        self.index += 1;

        Some(value)
    }
}
