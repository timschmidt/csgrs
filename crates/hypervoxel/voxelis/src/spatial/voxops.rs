use glam::{IVec2, IVec3, UVec3, Vec2, Vec3};

use crate::{Batch, Lod, MaxDepth, VoxInterner, VoxelTrait, utils::mesh::MeshData};

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
    /// Generates a naive mesh from the voxels.
    fn generate_naive_mesh_arrays(
        &self,
        interner: &VoxInterner<T>,
        mesh_data: &mut MeshData,
        offset: Vec3,
        lod: Lod,
    );
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

/// Trait for spatial operations in 2D.
pub trait VoxOpsSpatial2D {
    /// Returns the position in 2D.
    /// For columns, the units are columns, for sectors, the units are sectors.
    fn position_2d(&self) -> IVec2;

    /// Returns the world position in 2D.
    fn world_position_2d(&self) -> Vec2;

    /// Returns the world position of the center in 2D.
    fn world_center_position_2d(&self) -> Vec2;

    /// Returns the world size in 2D.
    fn world_size_2d(&self) -> Vec2;
}

/// Trait for spatial operations in 3D.
pub trait VoxOpsSpatial3D {
    /// Returns the position in 3D.
    /// For chunks, the units are chunks.
    fn position_3d(&self) -> IVec3;

    /// Returns the world position in 3D.
    fn world_position_3d(&self) -> Vec3;

    /// Returns the world position of the center in 3D.
    fn world_center_position_3d(&self) -> Vec3;

    /// Returns the world size in 3D.
    fn world_size_3d(&self) -> Vec3;
}

/// Combined trait for spatial operations in both 2D and 3D.
pub trait VoxOpsSpatial: VoxOpsSpatial2D + VoxOpsSpatial3D {}

/// Trait for converting positions between local and world coordinates.
pub trait VoxOpsConvertPositions {
    /// Converts a local position to a world position.
    fn local_to_world(&self, position: UVec3) -> IVec3;

    /// Converts a world position to a local position.
    fn world_to_local(&self, position: IVec3) -> UVec3;
}

/// Trait for chunk configuration in voxel operations.
pub trait VoxOpsChunkConfig {
    /// Returns the chunk dimensions in chunks.
    fn chunk_dimensions(&self) -> UVec3;

    /// Returns the chunk size in world units.
    fn chunk_size(&self) -> f32;

    /// Returns the voxel size in world units for the given level of detail.
    fn voxel_size(&self, lod: Lod) -> f32;
}
