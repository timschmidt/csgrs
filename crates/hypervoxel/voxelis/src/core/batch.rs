use glam::IVec3;

use crate::{
    NodeStore, VoxelTrait, spatial::OctreeOpsWrite, storage::node::MAX_CHILDREN,
    utils::common::encode_child_index_path,
};

#[derive(Debug)]
pub struct Batch<T: VoxelTrait> {
    masks: Vec<(u8, u8)>,
    values: Vec<[T; MAX_CHILDREN]>,
    to_fill: Option<T>,
    max_depth: u8,
    has_patches: bool,
}

impl<T: VoxelTrait> Batch<T> {
    pub fn new(max_depth: u8) -> Self {
        let lower_depth = if max_depth > 0 { max_depth - 1 } else { 0 };
        let size = 1 << (3 * lower_depth);

        Self {
            masks: vec![(0, 0); size],
            values: vec![[T::default(); MAX_CHILDREN]; size],
            to_fill: None,
            max_depth,
            has_patches: false,
        }
    }

    #[inline(always)]
    pub fn masks(&self) -> &Vec<(u8, u8)> {
        &self.masks
    }

    #[inline(always)]
    pub fn values(&self) -> &Vec<[T; MAX_CHILDREN]> {
        &self.values
    }

    #[inline(always)]
    pub fn to_fill(&self) -> Option<T> {
        self.to_fill
    }

    pub fn size(&self) -> usize {
        self.masks
            .iter()
            .filter(|(set_mask, clear_mask)| *set_mask != 0 || *clear_mask != 0)
            .count()
    }

    pub fn has_patches(&self) -> bool {
        self.has_patches
    }
}

impl<T: VoxelTrait> OctreeOpsWrite<T> for Batch<T> {
    fn set(&mut self, _store: &mut NodeStore<T>, position: IVec3, voxel: T) -> bool {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth));

        let full_path = encode_child_index_path(&position);

        let path = full_path & !0b111;
        let path_index = (path >> 3) as usize;
        let index = (full_path & 0b111) as usize;
        let bit = 1 << index;

        let (set_mask, clear_mask) = &mut self.masks[path_index];

        if voxel != T::default() {
            *set_mask |= bit;
            *clear_mask &= !bit;
        } else {
            *set_mask &= !bit;
            *clear_mask |= bit;
        }

        self.values[path_index][index] = voxel;

        self.has_patches = true;

        true
    }

    fn fill(&mut self, store: &mut NodeStore<T>, value: T) {
        self.clear(store);
        self.to_fill = Some(value);
    }

    fn clear(&mut self, _store: &mut NodeStore<T>) {
        self.masks.fill((0, 0));
        self.values.fill([T::default(); MAX_CHILDREN]);
        self.to_fill = None;
        self.has_patches = false;
    }
}
