use glam::IVec3;

use crate::{Batch, BlockId, Lod, MaxDepth, NodeStore, VoxelTrait};

mod dag;
mod ops;
mod svo;

pub use dag::SvoDag;
pub use ops::{
    OctreeOps, OctreeOpsBatch, OctreeOpsConfig, OctreeOpsDirty, OctreeOpsMesh, OctreeOpsRead,
    OctreeOpsState, OctreeOpsWrite,
};
pub use svo::Svo;

pub enum Octree {
    Static(SvoDag),
    Dynamic(Svo),
}

impl Octree {
    #[inline(always)]
    pub fn make_static(max_depth: MaxDepth) -> Self {
        Self::Static(SvoDag::new(max_depth))
    }

    #[inline(always)]
    pub fn make_dynamic(max_depth: MaxDepth) -> Self {
        Self::Dynamic(Svo::new(max_depth))
    }

    #[inline(always)]
    pub fn is_static(&self) -> bool {
        matches!(self, Self::Static(_))
    }

    #[inline(always)]
    pub fn is_dynamic(&self) -> bool {
        matches!(self, Self::Dynamic(_))
    }

    pub fn to_static<T: VoxelTrait>(&self, store: &mut NodeStore<T>) -> Self {
        match self {
            Self::Static(_) => panic!("Already static"),
            Self::Dynamic(svo) => {
                let mut dag = SvoDag::new(svo.max_depth(Lod::new(0)));
                copy_octree(svo, &mut dag, store);
                Self::Static(dag)
            }
        }
    }

    pub fn to_dynamic<T: VoxelTrait>(&self, store: &mut NodeStore<T>) -> Self {
        match self {
            Self::Static(dag) => {
                let mut svo = Svo::new(dag.max_depth(Lod::new(0)));
                copy_octree(dag, &mut svo, store);
                Self::Dynamic(svo)
            }
            Self::Dynamic(_) => panic!("Already dynamic"),
        }
    }

    pub fn get_root_id(&self) -> BlockId {
        match self {
            Self::Static(octree) => octree.get_root_id(),
            Self::Dynamic(octree) => octree.get_root_id(),
        }
    }
}

impl<T: VoxelTrait> OctreeOpsRead<T> for Octree {
    #[inline(always)]
    fn get(&self, store: &NodeStore<T>, position: IVec3) -> Option<T> {
        match self {
            Self::Static(octree) => octree.get(store, position),
            Self::Dynamic(octree) => octree.get(store, position),
        }
    }
}

impl<T: VoxelTrait> OctreeOpsWrite<T> for Octree {
    #[inline(always)]
    fn set(&mut self, store: &mut NodeStore<T>, position: IVec3, value: T) -> bool {
        match self {
            Self::Static(octree) => octree.set(store, position, value),
            Self::Dynamic(octree) => octree.set(store, position, value),
        }
    }

    #[inline(always)]
    fn fill(&mut self, store: &mut NodeStore<T>, value: T) {
        match self {
            Self::Static(octree) => octree.fill(store, value),
            Self::Dynamic(octree) => octree.fill(store, value),
        }
    }

    #[inline(always)]
    fn clear(&mut self, store: &mut NodeStore<T>) {
        match self {
            Self::Static(octree) => octree.clear(store),
            Self::Dynamic(octree) => octree.clear(store),
        }
    }
}

impl<T: VoxelTrait> OctreeOpsBatch<T> for Octree {
    #[inline(always)]
    fn create_batch(&self) -> Batch<T> {
        match self {
            Self::Static(octree) => octree.create_batch(),
            Self::Dynamic(octree) => octree.create_batch(),
        }
    }

    #[inline(always)]
    fn apply_batch(&mut self, store: &mut NodeStore<T>, batch: &Batch<T>) -> bool {
        match self {
            Self::Static(octree) => octree.apply_batch(store, batch),
            Self::Dynamic(octree) => octree.apply_batch(store, batch),
        }
    }
}

impl<T: VoxelTrait> OctreeOpsMesh<T> for Octree {
    #[inline(always)]
    fn to_vec(&self, store: &NodeStore<T>, lod: Lod) -> Vec<T> {
        match self {
            Self::Static(octree) => octree.to_vec(store, lod),
            Self::Dynamic(octree) => octree.to_vec(store, lod),
        }
    }
}

impl OctreeOpsConfig for Octree {
    #[inline(always)]
    fn max_depth(&self, lod: Lod) -> MaxDepth {
        match self {
            Self::Static(octree) => octree.max_depth(lod),
            Self::Dynamic(octree) => octree.max_depth(lod),
        }
    }

    #[inline(always)]
    fn voxels_per_axis(&self, lod: Lod) -> u32 {
        match self {
            Self::Static(octree) => octree.voxels_per_axis(lod),
            Self::Dynamic(octree) => octree.voxels_per_axis(lod),
        }
    }
}

impl OctreeOpsState for Octree {
    #[inline(always)]
    fn is_empty(&self) -> bool {
        match self {
            Self::Static(octree) => octree.is_empty(),
            Self::Dynamic(octree) => octree.is_empty(),
        }
    }

    #[inline(always)]
    fn is_leaf(&self) -> bool {
        match self {
            Self::Static(octree) => octree.is_leaf(),
            Self::Dynamic(octree) => octree.is_leaf(),
        }
    }
}

impl OctreeOpsDirty for Octree {
    #[inline(always)]
    fn is_dirty(&self) -> bool {
        match self {
            Self::Static(octree) => octree.is_dirty(),
            Self::Dynamic(octree) => octree.is_dirty(),
        }
    }

    #[inline(always)]
    fn mark_dirty(&mut self) {
        match self {
            Self::Static(octree) => octree.mark_dirty(),
            Self::Dynamic(octree) => octree.mark_dirty(),
        }
    }

    #[inline(always)]
    fn clear_dirty(&mut self) {
        match self {
            Self::Static(octree) => octree.clear_dirty(),
            Self::Dynamic(octree) => octree.clear_dirty(),
        }
    }
}

fn copy_octree<
    T: VoxelTrait,
    S: OctreeOpsRead<T> + OctreeOpsConfig + OctreeOpsState,
    D: OctreeOpsWrite<T>,
>(
    src: &S,
    dst: &mut D,
    store: &mut NodeStore<T>,
) {
    if src.is_empty() {
        return;
    }

    let voxels_per_axis = src.voxels_per_axis(Lod::new(0)) as i32;

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                let position = IVec3::new(x, y, z);
                if let Some(voxel) = src.get(store, position) {
                    dst.set(store, position, voxel);
                }
            }
        }
    }
}
