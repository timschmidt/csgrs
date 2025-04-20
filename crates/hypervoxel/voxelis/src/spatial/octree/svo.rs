use glam::IVec3;

use crate::{
    Batch, BlockId, NodeStore, TraversalDepth, VoxelTrait, child_index_macro, child_index_macro_2,
    storage::node::{EMPTY_CHILD, MAX_ALLOWED_DEPTH, MAX_CHILDREN},
    utils::common::{get_at_depth, to_vec},
};

use super::{
    OctreeOpsBatch, OctreeOpsConfig, OctreeOpsDirty, OctreeOpsMesh, OctreeOpsRead, OctreeOpsState,
    OctreeOpsWrite,
};

pub struct Svo {
    max_depth: u8,
    root_id: BlockId,
    dirty: bool,
}

impl Svo {
    pub fn new(max_depth: u8) -> Self {
        Self {
            max_depth,
            root_id: BlockId::EMPTY,
            dirty: false,
        }
    }

    pub fn get_root_id(&self) -> BlockId {
        self.root_id
    }
}

impl<T: VoxelTrait> OctreeOpsRead<T> for Svo {
    fn get(&self, store: &NodeStore<T>, position: IVec3) -> Option<T> {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth));

        get_at_depth(
            store,
            self.root_id,
            &position,
            &TraversalDepth::new(0, self.max_depth),
        )
    }
}

impl<T: VoxelTrait> OctreeOpsWrite<T> for Svo {
    fn set(&mut self, store: &mut NodeStore<T>, position: IVec3, voxel: T) -> bool {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth));

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("\n");
            println!("set_nolock position: {:?} voxel: {}", position, voxel);
        }

        let new_root_id = if !self.root_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!("Some(root) set position: {:?} voxel: {}", position, voxel);
                store.dump_node(self.root_id, 0, "  ");
            }

            // Existing root - modify
            set_at_root(store, &self.root_id, &position, self.max_depth, voxel)
        } else if voxel != T::default() {
            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!("None set position: {:?} voxel: {}", position, voxel);
                store.dump_node(self.root_id, 0, "  ");
            }

            set_at_root(store, &self.root_id, &position, self.max_depth, voxel)
        } else {
            return false;
        };

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("\n setting new_root");

        if new_root_id != BlockId::INVALID {
            // if !self.root_id.is_empty() {
            //     #[cfg(feature = "debug_trace_ref_counts")]
            //     {
            //         println!("existing_root_id:");
            //         storage.dump_node(self.root_id, 0, "  ");
            //         println!("new_root_id:");
            //         storage.dump_node(new_root_id, 0, "  ");
            //         println!(
            //             "  existing_root_id: {:?} new_root: {:?}",
            //             self.root_id, new_root_id,
            //         );
            //     }

            //     // storage.dec_ref_recursive(&self.root_id);

            //     #[cfg(feature = "debug_trace_ref_counts")]
            //     {
            //         println!(
            //             "new_root after recycling existing_root position: {:?}",
            //             position
            //         );
            //         storage.dump_node(new_root_id, 0, "  ");
            //     }
            // } else {
            //     #[cfg(feature = "debug_trace_ref_counts")]
            //     {
            //         println!("new_root_id:");
            //         storage.dump_node(new_root_id, 0, "  ");
            //         println!("  new_root: {:?}", new_root_id);
            //     }
            // }

            assert!(
                store.is_valid_block_id(&new_root_id),
                "Invalid new root id: {:?}",
                new_root_id
            );

            self.root_id = new_root_id;
            self.dirty = true;

            true
        } else {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!("new_root is None");

            false
        }
    }

    fn fill(&mut self, store: &mut NodeStore<T>, value: T) {
        if value != T::default() {
            if !self.root_id.is_empty() {
                store.dec_ref_recursive(&self.root_id);
            }
            self.root_id = store.get_or_create_leaf(value);
            self.dirty = true;
        } else {
            self.clear(store);
        }
    }

    fn clear(&mut self, store: &mut NodeStore<T>) {
        if !self.root_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!("clear root_id: {:?}", self.root_id);

            store.dec_ref_recursive(&self.root_id);

            #[cfg(feature = "debug_trace_ref_counts")]
            store.dump_patterns();

            assert!(store.patterns_empty());

            self.root_id = BlockId::EMPTY;
            self.dirty = true;
        }
    }
}

impl<T: VoxelTrait> OctreeOpsBatch<T> for Svo {
    #[inline(always)]
    fn create_batch(&self) -> Batch<T> {
        Batch::new(self.max_depth)
    }

    #[inline(always)]
    fn apply_batch(&mut self, _store: &mut NodeStore<T>, _batch: &Batch<T>) -> bool {
        false
    }
}

impl<T: VoxelTrait> OctreeOpsMesh<T> for Svo {
    fn to_vec(&self, store: &NodeStore<T>) -> Vec<T> {
        to_vec(store, &self.root_id, self.max_depth as usize)
    }
}

impl OctreeOpsConfig for Svo {
    #[inline(always)]
    fn max_depth(&self) -> u8 {
        self.max_depth
    }

    #[inline(always)]
    fn voxels_per_axis(&self) -> u32 {
        1 << self.max_depth
    }
}

impl OctreeOpsState for Svo {
    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.root_id.is_empty()
    }

    #[inline(always)]
    fn is_leaf(&self) -> bool {
        self.root_id.is_leaf()
    }
}

impl OctreeOpsDirty for Svo {
    #[inline(always)]
    fn is_dirty(&self) -> bool {
        self.dirty
    }

    #[inline(always)]
    fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    #[inline(always)]
    fn clear_dirty(&mut self) {
        self.dirty = false;
    }
}

#[inline(always)]
fn set_at_root<T: VoxelTrait>(
    store: &mut NodeStore<T>,
    node_id: &BlockId,
    position: &IVec3,
    max_depth: u8,
    voxel: T,
) -> BlockId {
    assert!(*node_id != BlockId::INVALID);

    let depth = TraversalDepth::new(0, max_depth);
    if voxel != T::default() {
        set_at_depth_iterative(store, node_id, position, &depth, voxel)
    } else {
        remove_at_depth(store, node_id, position, &depth)
    }
}

fn set_at_depth_iterative<T: VoxelTrait>(
    store: &mut NodeStore<T>,
    initial_id: &BlockId,
    position: &IVec3,
    initial_depth: &TraversalDepth,
    voxel: T,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "set_at_depth_iterative initial_node: {:?} position: {:?} voxel: {}",
        initial_id, position, voxel
    );

    let mut stack = [const { (BlockId::INVALID, BlockId::INVALID, u8::MAX) }; MAX_ALLOWED_DEPTH];
    let mut current_id = *initial_id;

    let max_depth = initial_depth.max() as usize;
    let initial_depth = initial_depth.current() as usize;
    let mut current_depth = initial_depth;

    // Phase 1: descend down the tree and build the chain
    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 1 - Find the spot");
        store.dump_node(current_id, 0, "  ");
    }

    let mut leaf_id = BlockId::EMPTY;
    while current_depth < max_depth {
        #[cfg(feature = "debug_trace_ref_counts")]
        let current_ref_count = store.get_ref(&current_id);

        let index = child_index_macro_2!(position, current_depth, max_depth);

        if current_id.is_branch() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [1]",
                current_depth, max_depth, index, current_id, leaf_id, current_ref_count
            );

            stack[current_depth] = (current_id, leaf_id, index as u8);

            current_id = store.get_child_id(&current_id, index);
        } else {
            if store.get_value(&current_id) == &voxel {
                return BlockId::INVALID; // Nothing changed
            }

            // Split leaf node
            leaf_id = current_id;
            current_id = BlockId::EMPTY;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [2]",
                current_depth, max_depth, index, current_id, leaf_id, current_ref_count
            );

            stack[current_depth] = (current_id, leaf_id, index as u8);
        }

        current_depth += 1;
    }

    // Phase 2: create a leaf at the correct depth
    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 2 - Create leaf");
        println!(
            "  depth: {}/{} current: {:?} ref_count: {} is_valid: {}",
            current_depth,
            max_depth,
            current_id,
            store.get_ref(&current_id),
            store.is_valid_block_id(&current_id)
        );
    }

    if current_id.is_leaf() {
        if store.get_value(&current_id) == &voxel {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!("  voxel already exists, no change required");
            return BlockId::INVALID; // Nothing changed
        } else {
            store.dec_ref(&current_id);
        }
    } else if !leaf_id.is_empty() && store.get_value(&leaf_id) == &voxel {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("  voxel already exists, no change required");
        return BlockId::INVALID; // Nothing changed
    }

    current_id = store.get_or_create_leaf(voxel);

    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "   current: {:?} ref_count: {}",
        current_id,
        store.get_ref(&current_id)
    );

    // Phase 3: propagate upwards and create new branch nodes
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(" Phase 3 - Propagate upwards");
    while current_depth > initial_depth {
        current_depth -= 1;

        let (parent_id, leaf_id, parent_index) = stack[current_depth];

        if !parent_id.is_empty() {
            let existing_child_id = store.get_child_id(&parent_id, parent_index as usize);

            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} existing: {:?} leaf: {:?} [B]",
                current_depth,
                max_depth,
                parent_index,
                parent_id,
                current_id,
                existing_child_id,
                leaf_id,
            );

            assert!(parent_id.is_branch(), "Parent node is not a branch");

            if existing_child_id == current_id {
                current_id = parent_id;

                #[cfg(feature = "debug_trace_ref_counts")]
                println!("  no change required, skipping");

                continue;
            }

            #[cfg(feature = "debug_trace_ref_counts")]
            for (child_idx, child) in store.get_children(&parent_id).iter().enumerate() {
                if child_idx == parent_index as usize {
                    if store.is_valid_block_id(child) {
                        println!(
                            "   child[{}]: {:?} [{}] => {:?} [{}]",
                            child_idx,
                            child,
                            store.get_ref(child),
                            current_id,
                            store.get_ref(&current_id),
                        );
                    } else {
                        println!(
                            "   child[{}]: {:?} [INVALID] => {:?} [{}]",
                            child_idx,
                            child,
                            current_id,
                            store.get_ref(&current_id),
                        );
                    }
                } else {
                    println!(
                        "   child[{}]: {:?} [{}]",
                        child_idx,
                        child,
                        store.get_ref(child)
                    );
                }
            }

            let types = parent_id.types() | ((current_id.is_leaf() as u8) << parent_index);
            let mask = parent_id.mask() | (1 << parent_index);

            current_id =
                store.update_branch(&parent_id, &current_id, parent_index as usize, types, mask);
        } else if leaf_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} [E]",
                current_depth, max_depth, parent_index, parent_id, current_id,
            );

            let mut children = EMPTY_CHILD;
            children[parent_index as usize] = current_id;
            let types = (current_id.is_leaf() as u8) << parent_index;
            let mask = 1 << parent_index;
            current_id = store.create_branch(children, types, mask);
        } else {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} leaf: {:?} [ESL]",
                current_depth, max_depth, parent_index, parent_id, current_id, leaf_id,
            );

            let mut children = [leaf_id; MAX_CHILDREN];
            store.inc_ref_by(&leaf_id, 7);
            children[parent_index as usize] = current_id;
            let types = !(1 << parent_index) | ((current_id.is_leaf() as u8) << parent_index);
            let mask = 0xFF;
            current_id = store.create_branch(children, types, mask);
        }
    }

    if initial_id.is_leaf() {
        store.dec_ref(initial_id);
    }

    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 4 - Finalize");
        println!("  new_root: {:?}", current_id);
        store.dump_node(current_id, 0, "  ");
    }

    current_id
}

fn remove_at_depth<T: VoxelTrait>(
    store: &mut NodeStore<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    assert!(*node_id != BlockId::INVALID);

    if node_id.is_branch() {
        remove_at_depth_branch(store, node_id, position, depth)
    } else {
        remove_at_depth_leaf(store, node_id, position, depth)
    }
}

fn remove_at_depth_branch<T: VoxelTrait>(
    store: &mut NodeStore<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "remove_at_depth_branch node_id: {:?} position: {:?} depth: {:?}",
        node_id, position, depth
    );

    assert!(store.is_valid_block_id(node_id));
    assert!(depth.current() < depth.max(), "Branch node at max depth");

    let index = child_index_macro!(position, depth);

    if node_id.has_child(index as u8) {
        let mut branch = store.get_children(node_id);
        let child_id = branch[index];

        let new_child_id = remove_at_depth(store, &child_id, position, &depth.increment());

        if new_child_id != BlockId::INVALID {
            assert!(store.is_valid_block_id(&new_child_id));

            let is_empty = new_child_id.is_empty();

            // let current_mask = node_id.mask();

            let mask = if is_empty {
                node_id.mask() & !(1 << index)
            } else {
                node_id.mask()
            };

            // Check if all children are empty
            if mask == 0 {
                // All children are empty, remove this branch node
                BlockId::EMPTY
            } else {
                // Return the updated branch node
                let is_branch = new_child_id.is_branch();
                assert!(is_branch, "Removing voxel should never produce a leaf node");

                // let current_types = node_id.types();
                let types = node_id.types() & !(1 << index);

                // println!(
                //     "before types: {:08b} mask: {:08b}",
                //     current_types, current_mask
                // );
                // println!(" after types: {:08b} mask: {:08b}", types, mask);

                branch[index] = new_child_id;

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    ".. [new branch] remove_at_depth_branch node_id: {:?} position: {:?} depth: {:?}",
                    node_id, position, depth
                );

                store.inc_child_refs(&branch, index);

                store.get_or_create_branch(branch, types, mask)
            }
        } else {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                ".. [invalid new_child_id] remove_at_depth_branch node_id: {:?} position: {:?} depth: {:?}",
                node_id, position, depth
            );
            BlockId::INVALID
        }
    } else {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            ".. [no child] remove_at_depth_branch node_id: {:?} position: {:?} depth: {:?}",
            node_id, position, depth
        );

        BlockId::INVALID
    }
}

fn remove_at_depth_leaf<T: VoxelTrait>(
    store: &mut NodeStore<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "remove_at_depth_leaf node_id: {:?} position: {:?} depth: {:?}",
        node_id, position, depth
    );

    assert!(store.is_valid_block_id(node_id));

    if depth.current() == depth.max() {
        // At max depth, just remove the leaf
        BlockId::EMPTY
    } else {
        // Remove the voxel in the appropriate child, splitting the leaf always results in a new branch
        let new_node_id = remove_at_depth_leaf(store, node_id, position, &depth.increment());

        assert!(store.is_valid_block_id(&new_node_id));

        // Convert leaf to branch
        let index = child_index_macro!(position, depth);
        let mut children = [*node_id; MAX_CHILDREN];
        children[index] = new_node_id;

        let is_leaf = new_node_id.is_leaf();
        let is_empty = new_node_id.is_empty();
        let types = !(1 << index) | ((is_leaf as u8) << index);
        let mask = !(1 << index) | ((!is_empty as u8) << index);

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("incrementing refs for node_id: {:?}", node_id);

        // println!("types: {:b} mask: {:b}", types, mask);

        store.inc_ref_by(node_id, 7);

        // Create new branch node
        store.get_or_create_branch(children, types, mask)
    }
}

#[cfg(test)]
mod tests {
    use crate::utils::common::child_index;

    use super::*;

    #[test]
    fn test_create() {
        let octree = Svo::new(3);
        assert!(octree.is_empty());
        assert_eq!(octree.max_depth(), 3);
        assert_eq!(octree.voxels_per_axis(), 8);
    }

    #[test]
    fn test_child_index() {
        for max_depth in 0..(MAX_ALLOWED_DEPTH as u8) {
            let voxels_per_axis = 1 << max_depth as i32;
            for depth in 0..max_depth {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        for x in 0..voxels_per_axis {
                            let position = IVec3::new(x, y, z);
                            let result =
                                child_index(&position, &TraversalDepth::new(depth, max_depth));
                            assert!(result < 8);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_set_and_get() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024 * 1024 * 128);

        let mut octree = Svo::new(3);
        let position = IVec3::new(0, 0, 0);

        // Test setting and getting a value
        assert!(octree.set(&mut store, position, 42));
        assert_eq!(octree.get(&store, position), Some(42));

        // Test overwriting a value
        assert!(octree.set(&mut store, position, 24));
        assert_eq!(octree.get(&store, position), Some(24));

        // Test getting from an empty position
        assert_eq!(octree.get(&store, IVec3::new(1, 1, 1)), None);

        // Test setting at max depth
        let max_pos = IVec3::new(7, 7, 7); // 2^3 - 1
        assert!(octree.set(&mut store, max_pos, 99));
        assert_eq!(octree.get(&store, max_pos), Some(99));

        octree.clear(&mut store);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.set(&mut store, pos, (i + 1) as u8);
        }

        for (i, &pos) in positions.iter().enumerate() {
            assert_eq!(octree.get(&store, pos).unwrap(), (i + 1) as u8);
        }
    }

    #[test]
    fn test_is_empty() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024 * 1024 * 128);

        let mut octree = Svo::new(3);
        assert!(octree.is_empty());

        // Setting a value makes it non-empty
        assert!(octree.set(&mut store, IVec3::new(0, 0, 0), 1));
        assert!(!octree.is_empty());

        // Clearing makes it empty again
        octree.clear(&mut store);
        assert!(octree.is_empty());
    }

    #[test]
    fn test_clear() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024 * 1024 * 128);

        let mut octree = Svo::new(3);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.set(&mut store, pos, (i + 1) as u8);
        }

        octree.clear(&mut store);
        assert!(octree.is_empty());

        for &pos in positions.iter() {
            assert!(octree.get(&store, pos).is_none());
        }
    }

    #[test]
    fn test_no_default_leaf_nodes() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024 * 1024 * 128);

        let mut octree = Svo::new(3);

        // Set a value and then set it back to default
        let position = IVec3::new(0, 0, 0);
        assert!(octree.set(&mut store, position, 42));
        assert!(octree.set(&mut store, position, 0)); // 0 is default for u8

        // The node should be removed when set to default
        assert_eq!(octree.get(&store, position), None);
        assert!(octree.is_empty());
    }

    #[test]
    fn test_dirty_flag() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024 * 1024 * 128);

        let mut octree = Svo::new(3);
        assert!(!octree.is_dirty());

        // Setting a value should make it dirty
        assert!(octree.set(&mut store, IVec3::new(0, 0, 0), 1));
        assert!(octree.is_dirty());

        // Clearing the dirty flag
        octree.clear_dirty();
        assert!(!octree.is_dirty());

        // Clearing the octree should make it dirty again
        octree.clear(&mut store);
        assert!(octree.is_dirty());
    }

    #[test]
    fn test_shared_storage() {
        let mut store = NodeStore::<u8>::with_memory_budget(1024);

        let mut octree1 = Svo::new(3);
        let mut octree2 = Svo::new(3);

        // Both trees should be empty initially
        assert!(octree1.is_empty());
        assert!(octree2.is_empty());

        // Setting in one tree should not affect the other
        assert!(octree1.set(&mut store, IVec3::new(0, 0, 0), 42));
        assert_eq!(octree1.get(&store, IVec3::new(0, 0, 0)), Some(42));
        assert_eq!(octree2.get(&store, IVec3::new(0, 0, 0)), None);

        // But they should share the same storage for efficiency
        assert!(octree2.set(&mut store, IVec3::new(1, 1, 1), 24));
        assert_eq!(octree2.get(&store, IVec3::new(1, 1, 1)), Some(24));
    }
}
