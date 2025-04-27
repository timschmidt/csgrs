use glam::IVec3;

use crate::{Batch, BlockId, DagInterner, Lod, MaxDepth, VoxelTrait};

mod svo;

use super::VoxTree;
pub use super::{
    VoxOpsBatch, VoxOpsConfig, VoxOpsDirty, VoxOpsMesh, VoxOpsRead, VoxOpsState, VoxOpsWrite,
};
pub use svo::Svo;

pub enum Octree {
    Static(VoxTree),
    Dynamic(Svo),
}

impl Octree {
    #[inline(always)]
    pub fn make_static(max_depth: MaxDepth) -> Self {
        Self::Static(VoxTree::new(max_depth))
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

    pub fn to_static<T: VoxelTrait>(&self, interner: &mut DagInterner<T>) -> Self {
        match self {
            Self::Static(_) => panic!("Already static"),
            Self::Dynamic(svo) => {
                let mut dag = VoxTree::new(svo.max_depth(Lod::new(0)));
                copy_octree(svo, &mut dag, interner);
                Self::Static(dag)
            }
        }
    }

    pub fn to_dynamic<T: VoxelTrait>(&self, interner: &mut DagInterner<T>) -> Self {
        match self {
            Self::Static(dag) => {
                let mut svo = Svo::new(dag.max_depth(Lod::new(0)));
                copy_octree(dag, &mut svo, interner);
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

impl<T: VoxelTrait> VoxOpsRead<T> for Octree {
    #[inline(always)]
    fn get(&self, interner: &DagInterner<T>, position: IVec3) -> Option<T> {
        match self {
            Self::Static(octree) => octree.get(interner, position),
            Self::Dynamic(octree) => octree.get(interner, position),
        }
    }
}

impl<T: VoxelTrait> VoxOpsWrite<T> for Octree {
    #[inline(always)]
    fn set(&mut self, interner: &mut DagInterner<T>, position: IVec3, value: T) -> bool {
        match self {
            Self::Static(octree) => octree.set(interner, position, value),
            Self::Dynamic(octree) => octree.set(interner, position, value),
        }
    }

    #[inline(always)]
    fn fill(&mut self, interner: &mut DagInterner<T>, value: T) {
        match self {
            Self::Static(octree) => octree.fill(interner, value),
            Self::Dynamic(octree) => octree.fill(interner, value),
        }
    }

    #[inline(always)]
    fn clear(&mut self, interner: &mut DagInterner<T>) {
        match self {
            Self::Static(octree) => octree.clear(interner),
            Self::Dynamic(octree) => octree.clear(interner),
        }
    }
}

impl<T: VoxelTrait> VoxOpsBatch<T> for Octree {
    #[inline(always)]
    fn create_batch(&self) -> Batch<T> {
        match self {
            Self::Static(octree) => octree.create_batch(),
            Self::Dynamic(octree) => octree.create_batch(),
        }
    }

    #[inline(always)]
    fn apply_batch(&mut self, interner: &mut DagInterner<T>, batch: &Batch<T>) -> bool {
        match self {
            Self::Static(octree) => octree.apply_batch(interner, batch),
            Self::Dynamic(octree) => octree.apply_batch(interner, batch),
        }
    }
}

impl<T: VoxelTrait> VoxOpsMesh<T> for Octree {
    #[inline(always)]
    fn to_vec(&self, interner: &DagInterner<T>, lod: Lod) -> Vec<T> {
        match self {
            Self::Static(octree) => octree.to_vec(interner, lod),
            Self::Dynamic(octree) => octree.to_vec(interner, lod),
        }
    }
}

impl VoxOpsConfig for Octree {
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

impl VoxOpsState for Octree {
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

impl VoxOpsDirty for Octree {
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
    S: VoxOpsRead<T> + VoxOpsConfig + VoxOpsState,
    D: VoxOpsWrite<T> + VoxOpsBatch<T>,
>(
    src: &S,
    dst: &mut D,
    interner: &mut DagInterner<T>,
) {
    if src.is_empty() {
        return;
    }

    let voxels_per_axis = src.voxels_per_axis(Lod::new(0)) as i32;

    let mut batch = dst.create_batch();

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                let position = IVec3::new(x, y, z);
                if let Some(voxel) = src.get(interner, position) {
                    batch.set(interner, position, voxel);
                }
            }
        }
    }

    if batch.has_patches() {
        dst.apply_batch(interner, &batch);
    }
}
