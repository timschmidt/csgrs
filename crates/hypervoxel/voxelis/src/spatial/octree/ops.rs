use glam::IVec3;

use crate::{Batch, DagInterner, Lod, MaxDepth, VoxelTrait};

pub trait OctreeOpsRead<T: VoxelTrait> {
    fn get(&self, interner: &DagInterner<T>, position: IVec3) -> Option<T>;
}

pub trait OctreeOpsWrite<T: VoxelTrait> {
    fn set(&mut self, interner: &mut DagInterner<T>, position: IVec3, voxel: T) -> bool;
    fn fill(&mut self, interner: &mut DagInterner<T>, value: T);
    fn clear(&mut self, interner: &mut DagInterner<T>);
}

pub trait OctreeOpsBatch<T: VoxelTrait> {
    fn create_batch(&self) -> Batch<T>;
    fn apply_batch(&mut self, interner: &mut DagInterner<T>, batch: &Batch<T>) -> bool;
}

pub trait OctreeOpsMesh<T: VoxelTrait> {
    fn to_vec(&self, interner: &DagInterner<T>, lod: Lod) -> Vec<T>;
}

pub trait OctreeOpsConfig {
    fn max_depth(&self, lod: Lod) -> MaxDepth;
    fn voxels_per_axis(&self, lod: Lod) -> u32;
}

pub trait OctreeOpsState {
    fn is_empty(&self) -> bool;
    fn is_leaf(&self) -> bool;
}

pub trait OctreeOpsDirty {
    fn is_dirty(&self) -> bool;
    fn mark_dirty(&mut self);
    fn clear_dirty(&mut self);
}

pub trait OctreeOps<T: VoxelTrait>:
    OctreeOpsRead<T> + OctreeOpsWrite<T> + OctreeOpsConfig + OctreeOpsState + OctreeOpsDirty
{
}
