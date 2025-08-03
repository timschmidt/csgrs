//! Module `core::batch`
//!
//! This module provides a buffer for batching set, clear, and fill operations on an octree node interner.
//! It is designed to optimize voxel modifications by accumulating changes before applying them to the octree.
//!
//! # Examples
//!
//! ```
//! use voxelis::{Batch, MaxDepth, VoxInterner, spatial::{VoxOpsBulkWrite, VoxOpsWrite}};
//! use glam::IVec3;
//!
//! // Create interner for 8-bit voxels
//! let mut interner = VoxInterner::<u8>::with_memory_budget(1024);
//!
//! let mut batch = Batch::<u8>::new(MaxDepth::new(4));
//! // Fill the octree with a uniform voxel value
//! batch.fill(&mut interner, 2);
//! // Set a voxel at position (1, 2, 3)
//! batch.set(&mut interner, IVec3::new(1, 2, 3), 1);
//! // Clear a voxel at position (4, 5, 6)
//! batch.set(&mut interner, IVec3::new(4, 5, 6), 0);
//! ```

use glam::IVec3;

use crate::{
    MaxDepth, VoxInterner, VoxelTrait,
    interner::MAX_CHILDREN,
    spatial::{VoxOpsBulkWrite, VoxOpsWrite},
    utils::common::encode_child_index_path,
};

/// Accumulates per-node voxel modifications, enabling efficient bulk updates for an octree.
///
/// # Type parameters
///
/// * `T` - The voxel type implementing [`VoxelTrait`].
#[derive(Debug)]
pub struct Batch<T: VoxelTrait> {
    masks: Vec<(u8, u8)>,
    values: Vec<[T; MAX_CHILDREN]>,
    to_fill: Option<T>,
    max_depth: MaxDepth,
    has_patches: bool,
}

impl<T: VoxelTrait> Batch<T> {
    /// Creates a new [`Batch`] for a tree of the given maximum depth.
    /// Returns a new, empty [`Batch`] ready to record set, clear, or fill operations.
    ///
    /// # Arguments
    ///
    /// * `max_depth` - Maximum depth (levels) of the target octree.
    ///
    /// # Example
    ///
    /// ```rust
    /// use voxelis::{Batch, MaxDepth};
    ///
    /// let batch = Batch::<u8>::new(MaxDepth::new(4));
    /// ```
    #[must_use]
    pub fn new(max_depth: MaxDepth) -> Self {
        let lower_depth = if max_depth.max() > 0 {
            max_depth.max() - 1
        } else {
            0
        };
        let size = 1 << (3 * lower_depth);

        Self {
            masks: vec![(0, 0); size],
            values: vec![[T::default(); MAX_CHILDREN]; size],
            to_fill: None,
            max_depth,
            has_patches: false,
        }
    }

    #[must_use]
    #[inline(always)]
    /// Returns the internal vector of (`set_mask`, `clear_mask`) pairs per node.
    pub fn masks(&self) -> &Vec<(u8, u8)> {
        &self.masks
    }

    #[must_use]
    #[inline(always)]
    /// Returns the buffered voxel values array for each child of every node.
    pub fn values(&self) -> &Vec<[T; MAX_CHILDREN]> {
        &self.values
    }

    #[must_use]
    #[inline(always)]
    /// Returns the uniform fill value if `fill` was invoked; otherwise `None`.
    pub fn to_fill(&self) -> Option<T> {
        self.to_fill
    }

    /// Counts and returns the number of recorded set or clear operations.
    #[must_use]
    pub fn size(&self) -> usize {
        self.masks
            .iter()
            .filter(|(set_mask, clear_mask)| *set_mask != 0 || *clear_mask != 0)
            .count()
    }

    /// Indicates whether any operations have been recorded in this batch.
    #[must_use]
    pub fn has_patches(&self) -> bool {
        self.has_patches
    }

    /// Records a voxel set or clear operation at the specified 3D position.
    /// Returns `true` indicating that the state has changed.
    ///
    /// # Arguments
    ///
    /// * `position` - 3D coordinates of the voxel to modify.
    /// * `voxel` - The voxel value to set; `T::default()` clears the voxel.
    ///
    /// # Panics
    ///
    /// Panics if `position` is out of bounds for the configured `max_depth`.
    pub fn just_set(&mut self, position: IVec3, voxel: T) -> bool {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth.max()));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth.max()));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth.max()));

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
}

impl<T: VoxelTrait> VoxOpsWrite<T> for Batch<T> {
    /// Records a set or clear operation for the given `position`, delegating to `just_set`.
    /// Records a voxel set or clear operation at the specified 3D position.
    /// Returns `true` indicating that the state has changed.
    ///
    /// # Arguments
    ///
    /// * `position` - 3D coordinates of the voxel to modify.
    /// * `voxel` - The voxel value to set; `T::default()` clears the voxel.
    ///
    /// # Panics
    ///
    /// Panics if `position` is out of bounds for the configured `max_depth`.
    fn set(&mut self, _interner: &mut VoxInterner<T>, position: IVec3, voxel: T) -> bool {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth.max()));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth.max()));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth.max()));

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
}

impl<T: VoxelTrait> VoxOpsBulkWrite<T> for Batch<T> {
    /// Clears existing operations and sets a uniform fill value for the batch.
    fn fill(&mut self, interner: &mut VoxInterner<T>, value: T) {
        self.clear(interner);
        self.to_fill = Some(value);
    }

    /// Resets all recorded operations, clearing masks, values, and fill state.
    fn clear(&mut self, _interner: &mut VoxInterner<T>) {
        self.masks.fill((0, 0));
        self.values.fill([T::default(); MAX_CHILDREN]);
        self.to_fill = None;
        self.has_patches = false;
    }
}
