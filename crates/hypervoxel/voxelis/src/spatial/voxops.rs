use glam::IVec3;

use crate::{Batch, Lod, MaxDepth, VoxInterner, VoxelTrait};

/// Trait for reading voxels.
pub trait VoxOpsRead<T: VoxelTrait> {
    /// Gets a voxel at the given position.
    fn get(&self, interner: &VoxInterner<T>, position: IVec3) -> Option<T>;
}

/// Trait for writing voxels.
pub trait VoxOpsWrite<T: VoxelTrait> {
    /// Sets a voxel at the given position.
    fn set(&mut self, interner: &mut VoxInterner<T>, position: IVec3, voxel: T) -> bool;
}

/// Trait for bulk operations on voxels.
pub trait VoxOpsBulkWrite<T: VoxelTrait> {
    /// Fills a region with the given value.
    fn fill(&mut self, interner: &mut VoxInterner<T>, value: T);

    /// Clears the voxels in the region.
    fn clear(&mut self, interner: &mut VoxInterner<T>);
}

/// Trait for batch operations on voxels.
pub trait VoxOpsBatch<T: VoxelTrait> {
    /// Creates a new batch for voxel operations.
    fn create_batch(&self) -> Batch<T>;

    /// Applies a batch of voxel operations.
    fn apply_batch(&mut self, interner: &mut VoxInterner<T>, batch: &Batch<T>) -> bool;
}

/// Trait for generating meshes from voxels.
pub trait VoxOpsMesh<T: VoxelTrait> {
    fn to_vec(&self, interner: &VoxInterner<T>, lod: Lod) -> Vec<T>;
}

/// Trait for configuration of voxel operations.
pub trait VoxOpsConfig {
    /// Returns the maximum depth for the given level of detail.
    fn max_depth(&self, lod: Lod) -> MaxDepth;

    /// Returns the number of voxels per axis for the given level of detail.
    fn voxels_per_axis(&self, lod: Lod) -> u32;
}

/// Trait for state operations on voxels.
pub trait VoxOpsState {
    /// Returns true if the voxel structure is empty.
    fn is_empty(&self) -> bool;

    /// Return true if the voxel structure is a leaf node.
    fn is_leaf(&self) -> bool;
}

/// Trait for dirty state management of voxels.
pub trait VoxOpsDirty {
    /// Returns true if the voxel structure is dirty.
    fn is_dirty(&self) -> bool;

    /// Marks the voxel structure as dirty.
    fn mark_dirty(&mut self);

    /// Clears the dirty state of the voxel structure.
    fn clear_dirty(&mut self);
}

/// Combined trait for all voxel operations.
pub trait VoxOps<T: VoxelTrait>:
    VoxOpsRead<T> + VoxOpsWrite<T> + VoxOpsConfig + VoxOpsState + VoxOpsDirty
{
}
