//! Buffered voxel edits for one fixed-depth tree.
//!
//! A [`Batch`] stores the last set or clear operation for each addressed voxel.
//! Applying the batch lets a tree path-copy related edits together.
//!
//! ```
//! use voxelis::{Batch, MaxDepth, VoxInterner, spatial::{VoxOpsBulkWrite, VoxOpsWrite}};
//! use glam::IVec3;
//!
//! let mut interner = VoxInterner::<u8>::with_memory_budget(1024);
//! let mut batch = Batch::<u8>::new(MaxDepth::new(4));
//! batch.fill(&mut interner, 2);
//! batch.set(&mut interner, IVec3::new(1, 2, 3), 1);
//! batch.set(&mut interner, IVec3::new(4, 5, 6), 0);
//! ```

use glam::IVec3;

use crate::{
    Lod, MaxDepth, VoxInterner, VoxelTrait,
    interner::MAX_CHILDREN,
    spatial::{VoxOpsBulkWrite, VoxOpsConfig, VoxOpsWrite},
    utils::common::encode_child_index_path,
};

/// Accumulates voxel modifications for a tree with one configured depth.
#[derive(Debug)]
pub struct Batch<T: VoxelTrait> {
    masks: Vec<(u8, u8)>,
    values: Vec<[T; MAX_CHILDREN]>,
    to_fill: Option<T>,
    max_depth: MaxDepth,
    has_patches: bool,
}

impl<T: VoxelTrait> Batch<T> {
    /// Creates an empty batch for a tree with `max_depth` levels.
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
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::new");

        let lower_depth = if max_depth.max() > 0 {
            max_depth.max() - 1
        } else {
            0
        };
        let size = 1 << (3 * lower_depth);

        Self {
            masks: vec![const { (0, 0) }; size],
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
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::masks");

        &self.masks
    }

    #[must_use]
    #[inline(always)]
    /// Returns the buffered voxel values array for each child of every node.
    pub fn values(&self) -> &Vec<[T; MAX_CHILDREN]> {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::values");

        &self.values
    }

    #[must_use]
    #[inline(always)]
    /// Returns the uniform fill value if `fill` was invoked; otherwise `None`.
    pub fn to_fill(&self) -> Option<T> {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::to_fill");

        self.to_fill
    }

    /// Returns the number of leaf-parent entries containing one or more edits.
    ///
    /// This is not necessarily the number of edited voxels because each mask
    /// entry represents up to eight sibling voxels.
    #[must_use]
    pub fn size(&self) -> usize {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::size");

        self.masks
            .iter()
            .filter(|(set_mask, clear_mask)| *set_mask != 0 || *clear_mask != 0)
            .count()
    }

    /// Returns whether the batch contains any per-voxel patches.
    ///
    /// A uniform fill alone does not count as a patch; inspect [`Self::to_fill`]
    /// when both forms of pending work matter.
    #[must_use]
    pub fn has_patches(&self) -> bool {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::has_patches");

        self.has_patches
    }

    /// Records a set or clear at `position` and returns `true`.
    ///
    /// # Arguments
    ///
    /// * `position` - 3D coordinates of the voxel to modify.
    /// * `voxel` - Value to set; `T::default()` records a clear.
    ///
    /// # Panics
    ///
    /// Panics if `position` is out of bounds for the configured `max_depth`.
    pub fn just_set(&mut self, position: IVec3, voxel: T) -> bool {
        debug_assert!(position.x >= 0 && position.x < (1 << self.max_depth.max()));
        debug_assert!(position.y >= 0 && position.y < (1 << self.max_depth.max()));
        debug_assert!(position.z >= 0 && position.z < (1 << self.max_depth.max()));

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::just_set");

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

    /// Clears existing operations and sets a uniform fill value for the batch.
    pub fn just_fill(&mut self, value: T) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::just_fill");

        self.just_clear();
        self.to_fill = Some(value);
    }

    /// Resets all recorded operations, clearing masks, values, and fill state.
    pub fn just_clear(&mut self) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("Batch::just_clear");

        self.masks.fill((0, 0));
        self.values.fill([T::default(); MAX_CHILDREN]);
        self.to_fill = None;
        self.has_patches = false;
    }
}

impl<T: VoxelTrait> VoxOpsWrite<T> for Batch<T> {
    fn set(&mut self, _interner: &mut VoxInterner<T>, position: IVec3, voxel: T) -> bool {
        self.just_set(position, voxel)
    }
}

impl<T: VoxelTrait> VoxOpsBulkWrite<T> for Batch<T> {
    fn fill(&mut self, _interner: &mut VoxInterner<T>, value: T) {
        self.just_fill(value);
    }

    fn clear(&mut self, _interner: &mut VoxInterner<T>) {
        self.just_clear();
    }
}

impl<T: VoxelTrait> VoxOpsConfig for Batch<T> {
    #[inline(always)]
    fn max_depth(&self, lod: Lod) -> MaxDepth {
        self.max_depth.for_lod(lod)
    }

    #[inline(always)]
    fn voxels_per_axis(&self, lod: Lod) -> u32 {
        1 << self.max_depth.for_lod(lod).max()
    }
}
