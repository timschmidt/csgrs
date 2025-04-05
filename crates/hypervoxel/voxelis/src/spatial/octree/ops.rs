use glam::IVec3;

use crate::{NodeStore, VoxelTrait};

pub trait OctreeOpsRead<T: VoxelTrait> {
    fn get(&self, store: &NodeStore<T>, position: IVec3) -> Option<T>;
}

pub trait OctreeOpsWrite<T: VoxelTrait> {
    fn set(&mut self, store: &mut NodeStore<T>, position: IVec3, voxel: T) -> bool;
    fn fill(&mut self, store: &mut NodeStore<T>, value: T);
    fn clear(&mut self, store: &mut NodeStore<T>);
}

pub trait OctreeOpsMesh<T: VoxelTrait> {
    fn to_vec(&self, store: &NodeStore<T>) -> Vec<T>;
}

pub trait OctreeOpsConfig {
    fn max_depth(&self) -> u8;
    fn voxels_per_axis(&self) -> u32;
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
