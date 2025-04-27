use glam::IVec3;

use crate::{
    Batch, BlockId, Lod, MaxDepth, TraversalDepth, VoxInterner, VoxelTrait, child_index_macro,
    child_index_macro_2,
    interner::{EMPTY_CHILD, MAX_ALLOWED_DEPTH, MAX_CHILDREN},
    utils::common::{get_at_depth, to_vec},
};

use super::{
    VoxOpsBatch, VoxOpsConfig, VoxOpsDirty, VoxOpsMesh, VoxOpsRead, VoxOpsState, VoxOpsWrite,
};

/// Lookup table for fast sibling scanning in octree traversal using Morton-encoded paths.
///
/// `PATH_MASKS[max_depth][level]` provides a bitmask indicating which sibling nodes
/// (at a given tree level) are affected by a batch operation or traversal, based on the
/// Morton encoding of the path. Each mask allows to quickly select all siblings up to
/// a given position, enabling efficient batch updates and queries.
///
/// - The outer array index (`max_depth`) corresponds to the maximum octree depth.
/// - The inner array index (`level`) corresponds to the current level within the octree (0-based).
/// - Each mask is a bitfield where set bits indicate affected siblings at that level.
///
/// # Usage
///
/// Used internally in SVO DAG batch algorithms for fast propagation of changes
/// across sibling nodes, leveraging the spatial locality of Morton codes.
///
/// # Morton Encoding
///
/// Morton codes (Z-order curve) interleave the bits of the 3D coordinates,
/// enabling efficient spatial indexing and traversal in octrees.
///
/// # See also
///
/// - [`encode_child_index_path`]
/// - SVO DAG batch update logic
const PATH_MASKS: [[u32; MAX_ALLOWED_DEPTH - 1]; MAX_ALLOWED_DEPTH] = [
    // max_depth == 0
    [
        0b00_000_000_000_000_000_000_000_000_000_000, // 0
        0b00_000_000_000_000_000_000_000_000_000_000, // 1
        0b00_000_000_000_000_000_000_000_000_000_000, // 2
        0b00_000_000_000_000_000_000_000_000_000_000, // 3
        0b00_000_000_000_000_000_000_000_000_000_000, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 1
    [
        0b00_000_000_000_000_000_000_000_000_000_111, // 0
        0b00_000_000_000_000_000_000_000_000_000_000, // 1
        0b00_000_000_000_000_000_000_000_000_000_000, // 2
        0b00_000_000_000_000_000_000_000_000_000_000, // 3
        0b00_000_000_000_000_000_000_000_000_000_000, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 2
    [
        0b00_000_000_000_000_000_000_000_000_111_000, // 0
        0b00_000_000_000_000_000_000_000_000_111_111, // 1
        0b00_000_000_000_000_000_000_000_000_000_000, // 2
        0b00_000_000_000_000_000_000_000_000_000_000, // 3
        0b00_000_000_000_000_000_000_000_000_000_000, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 3
    [
        0b00_000_000_000_000_000_000_000_111_000_000, // 0
        0b00_000_000_000_000_000_000_000_111_111_000, // 1
        0b00_000_000_000_000_000_000_000_111_111_111, // 2
        0b00_000_000_000_000_000_000_000_000_000_000, // 3
        0b00_000_000_000_000_000_000_000_000_000_000, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 4
    [
        0b00_000_000_000_000_000_000_111_000_000_000, // 0
        0b00_000_000_000_000_000_000_111_111_000_000, // 1
        0b00_000_000_000_000_000_000_111_111_111_000, // 2
        0b00_000_000_000_000_000_000_111_111_111_111, // 3
        0b00_000_000_000_000_000_000_000_000_000_000, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 5
    [
        0b00_000_000_000_000_000_111_000_000_000_000, // 0
        0b00_000_000_000_000_000_111_111_000_000_000, // 1
        0b00_000_000_000_000_000_111_111_111_000_000, // 2
        0b00_000_000_000_000_000_111_111_111_111_000, // 3
        0b00_000_000_000_000_000_111_111_111_111_111, // 4
        0b00_000_000_000_000_000_000_000_000_000_000, // 5
    ],
    // max_depth == 6
    [
        0b00_000_000_000_000_111_000_000_000_000_000, // 0
        0b00_000_000_000_000_111_111_000_000_000_000, // 1
        0b00_000_000_000_000_111_111_111_000_000_000, // 2
        0b00_000_000_000_000_111_111_111_111_000_000, // 3
        0b00_000_000_000_000_111_111_111_111_111_000, // 4
        0b00_000_000_000_000_111_111_111_111_111_111, // 5
    ],
];

/// VoxTree - a high performance, SVO DAG (Sparse Voxel Octree Directed Acyclic Graph) structure.
pub struct VoxTree {
    max_depth: MaxDepth,
    root_id: BlockId,
    dirty: bool,
}

impl VoxTree {
    pub fn new(max_depth: MaxDepth) -> Self {
        Self {
            max_depth,
            root_id: BlockId::EMPTY,
            dirty: false,
        }
    }

    pub fn get_root_id(&self) -> BlockId {
        self.root_id
    }

    pub fn set_root_id<T: VoxelTrait>(&mut self, interner: &mut VoxInterner<T>, root_id: BlockId) {
        self.root_id = root_id;
        interner.inc_ref(&self.root_id);
    }
}

impl<T: VoxelTrait> VoxOpsRead<T> for VoxTree {
    fn get(&self, interner: &VoxInterner<T>, position: IVec3) -> Option<T> {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth.max()));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth.max()));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth.max()));

        get_at_depth(
            interner,
            self.root_id,
            &position,
            &TraversalDepth::new(0, self.max_depth.max()),
        )
    }
}

impl<T: VoxelTrait> VoxOpsWrite<T> for VoxTree {
    fn set(&mut self, interner: &mut VoxInterner<T>, position: IVec3, voxel: T) -> bool {
        assert!(position.x >= 0 && position.x < (1 << self.max_depth.max()));
        assert!(position.y >= 0 && position.y < (1 << self.max_depth.max()));
        assert!(position.z >= 0 && position.z < (1 << self.max_depth.max()));

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("\n");
            println!("set position: {:?} voxel: {}", position, voxel);
        }

        let new_root_id = if !self.root_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!("Some(root) set position: {:?} voxel: {}", position, voxel);
                interner.dump_node(self.root_id, 0, "  ");
            }

            // Existing root - modify
            set_at_root(
                interner,
                &self.root_id,
                &position,
                self.max_depth.max(),
                voxel,
            )
        } else if voxel != T::default() {
            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!("None set position: {:?} voxel: {}", position, voxel);
                interner.dump_node(self.root_id, 0, "  ");
            }

            set_at_root(
                interner,
                &self.root_id,
                &position,
                self.max_depth.max(),
                voxel,
            )
        } else {
            return false;
        };

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("\n setting new_root");

        if new_root_id != BlockId::INVALID {
            if !self.root_id.is_empty() {
                assert_ne!(new_root_id, self.root_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                {
                    println!("existing_root_id:");
                    interner.dump_node(self.root_id, 0, "  ");
                    println!("new_root_id:");
                    interner.dump_node(new_root_id, 0, "  ");
                    println!(
                        "  existing_root_id: {:?} new_root: {:?}",
                        self.root_id, new_root_id,
                    );
                }

                interner.dec_ref_recursive(&self.root_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                {
                    println!(
                        "new_root after recycling existing_root position: {:?}",
                        position
                    );
                    interner.dump_node(new_root_id, 0, "  ");
                }
            } else {
                #[cfg(feature = "debug_trace_ref_counts")]
                {
                    println!("new_root_id:");
                    interner.dump_node(new_root_id, 0, "  ");
                    println!("  new_root: {:?}", new_root_id);
                }
            }

            assert!(
                interner.is_valid_block_id(&new_root_id),
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

    fn fill(&mut self, interner: &mut VoxInterner<T>, value: T) {
        if value != T::default() {
            if !self.root_id.is_empty() {
                interner.dec_ref_recursive(&self.root_id);
            }
            self.root_id = interner.get_or_create_leaf(value);
            self.dirty = true;
        } else {
            self.clear(interner);
        }
    }

    fn clear(&mut self, interner: &mut VoxInterner<T>) {
        if !self.root_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!("clear root_id: {:?}", self.root_id);

            interner.dec_ref_recursive(&self.root_id);

            #[cfg(feature = "debug_trace_ref_counts")]
            interner.dump_patterns();

            assert!(interner.patterns_empty());

            self.root_id = BlockId::EMPTY;
            self.dirty = true;
        }
    }
}

impl<T: VoxelTrait> VoxOpsBatch<T> for VoxTree {
    fn create_batch(&self) -> Batch<T> {
        Batch::new(self.max_depth)
    }

    fn apply_batch(&mut self, interner: &mut VoxInterner<T>, batch: &Batch<T>) -> bool {
        let new_root_id = set_batch_at_root(interner, &self.root_id, self.max_depth.max(), batch);

        if new_root_id != BlockId::INVALID {
            if !self.root_id.is_empty() {
                assert_ne!(new_root_id, self.root_id);

                interner.dec_ref_recursive(&self.root_id);
            }

            assert!(
                interner.is_valid_block_id(&new_root_id),
                "Invalid new root id: {:?}",
                new_root_id
            );

            self.root_id = new_root_id;
            self.dirty = true;

            true
        } else {
            false
        }
    }
}

impl<T: VoxelTrait> VoxOpsMesh<T> for VoxTree {
    fn to_vec(&self, interner: &VoxInterner<T>, lod: Lod) -> Vec<T> {
        to_vec(interner, &self.root_id, self.max_depth.for_lod(lod))
    }
}

impl VoxOpsConfig for VoxTree {
    #[inline(always)]
    fn max_depth(&self, lod: Lod) -> MaxDepth {
        self.max_depth.for_lod(lod)
    }

    #[inline(always)]
    fn voxels_per_axis(&self, lod: Lod) -> u32 {
        1 << self.max_depth.for_lod(lod).max()
    }
}

impl VoxOpsState for VoxTree {
    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.root_id.is_empty()
    }

    #[inline(always)]
    fn is_leaf(&self) -> bool {
        self.root_id.is_leaf()
    }
}

impl VoxOpsDirty for VoxTree {
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
    interner: &mut VoxInterner<T>,
    node_id: &BlockId,
    position: &IVec3,
    max_depth: u8,
    voxel: T,
) -> BlockId {
    assert!(*node_id != BlockId::INVALID);

    let depth = TraversalDepth::new(0, max_depth);
    if voxel != T::default() {
        set_at_depth_iterative(interner, node_id, position, &depth, voxel)
    } else {
        remove_at_depth(interner, node_id, position, &depth)
    }
}

fn set_at_depth_iterative<T: VoxelTrait>(
    interner: &mut VoxInterner<T>,
    initial_node_id: &BlockId,
    position: &IVec3,
    initial_depth: &TraversalDepth,
    voxel: T,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "set_at_depth_iterative initial_node: {:?} position: {:?} voxel: {}",
        initial_node_id, position, voxel
    );

    let mut stack = [const { (BlockId::INVALID, BlockId::INVALID, u8::MAX) }; MAX_ALLOWED_DEPTH];
    let mut current_node_id = *initial_node_id;

    let max_depth = initial_depth.max() as usize;
    let initial_depth = initial_depth.current() as usize;
    let mut current_depth = initial_depth;

    // Phase 1: descend down the tree and build the chain
    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 1 - Find the spot");
        interner.dump_node(current_node_id, 0, "  ");
    }

    let mut leaf_node_id = BlockId::EMPTY;
    while current_depth < max_depth {
        #[cfg(feature = "debug_trace_ref_counts")]
        let current_node_ref_count = interner.get_ref(&current_node_id);

        let index = child_index_macro_2!(position, current_depth, max_depth);

        if current_node_id.is_branch() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [1]",
                current_depth,
                max_depth,
                index,
                current_node_id,
                leaf_node_id,
                current_node_ref_count
            );

            stack[current_depth] = (current_node_id, leaf_node_id, index as u8);

            current_node_id = interner.get_child_id(&current_node_id, index);
        } else {
            if interner.get_value(&current_node_id) == &voxel {
                return BlockId::INVALID; // Nothing changed
            }

            // Split leaf node
            leaf_node_id = current_node_id;
            current_node_id = BlockId::EMPTY;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [2]",
                current_depth,
                max_depth,
                index,
                current_node_id,
                leaf_node_id,
                current_node_ref_count
            );

            stack[current_depth] = (current_node_id, leaf_node_id, index as u8);
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
            current_node_id,
            interner.get_ref(&current_node_id),
            interner.is_valid_block_id(&current_node_id)
        );
    }

    if current_node_id.is_leaf() && interner.get_value(&current_node_id) == &voxel {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("  voxel already exists, no change required");
        return BlockId::INVALID; // Nothing changed
    }

    current_node_id = interner.get_or_create_leaf(voxel);

    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "   current: {:?} ref_count: {}",
        current_node_id,
        interner.get_ref(&current_node_id)
    );

    // Phase 3: propagate upwards and create new branch nodes
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(" Phase 3 - Propagate upwards");
    while current_depth > initial_depth {
        current_depth -= 1;

        let (parent_node_id, leaf_node_id, parent_node_index) = stack[current_depth];

        if !parent_node_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} leaf: {:?} [B]",
                current_depth,
                max_depth,
                parent_node_index,
                parent_node_id,
                current_node_id,
                leaf_node_id,
            );

            #[cfg(feature = "debug_trace_ref_counts")]
            for (child_idx, child) in interner.get_children(&parent_node_id).iter().enumerate() {
                if child_idx == parent_node_index as usize {
                    println!(
                        "   child[{}]: {:?} [{}] => {:?} [{}]",
                        child_idx,
                        child,
                        interner.get_ref(child),
                        current_node_id,
                        interner.get_ref(&current_node_id),
                    );
                } else {
                    println!(
                        "   child[{}]: {:?} [{}]",
                        child_idx,
                        child,
                        interner.get_ref(child)
                    );
                }
            }

            let types =
                parent_node_id.types() | ((current_node_id.is_leaf() as u8) << parent_node_index);

            let mut branch = interner.get_children(&parent_node_id);
            branch[parent_node_index as usize] = current_node_id;

            if !(types == 0xFF && branch.iter().all(|item| item == &current_node_id)) {
                let mask = parent_node_id.mask() | (1 << parent_node_index);

                interner.inc_child_refs(&branch, parent_node_index as usize);

                current_node_id = interner.get_or_create_branch(branch, types, mask);
            }
        } else if leaf_node_id.is_empty() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} leaf: {:?} [E]",
                current_depth,
                max_depth,
                parent_node_index,
                parent_node_id,
                current_node_id,
                leaf_node_id,
            );

            let mut children = EMPTY_CHILD;
            children[parent_node_index as usize] = current_node_id;
            let types = (current_node_id.is_leaf() as u8) << parent_node_index;
            let mask = 1 << parent_node_index;
            current_node_id = interner.get_or_create_branch(children, types, mask);
        } else {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  depth: {}/{} i: {:2x} parent: {:?} current: {:?} leaf: {:?} [ESL]",
                current_depth,
                max_depth,
                parent_node_index,
                parent_node_id,
                current_node_id,
                leaf_node_id,
            );

            let mut children = [leaf_node_id; MAX_CHILDREN];
            interner.inc_ref_by(&leaf_node_id, 7);
            children[parent_node_index as usize] = current_node_id;
            let types = !(1 << parent_node_index)
                | ((current_node_id.is_leaf() as u8) << parent_node_index);
            let mask = 0xFF;
            current_node_id = interner.get_or_create_branch(children, types, mask);
        }
    }

    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 4 - Finalize");
        println!("  new_root: {:?}", current_node_id);
        interner.dump_node(current_node_id, 0, "  ");
    }

    current_node_id
}

fn remove_at_depth<T: VoxelTrait>(
    interner: &mut VoxInterner<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    assert!(*node_id != BlockId::INVALID);

    if node_id.is_branch() {
        remove_at_depth_branch(interner, node_id, position, depth)
    } else {
        remove_at_depth_leaf(interner, node_id, position, depth)
    }
}

fn remove_at_depth_branch<T: VoxelTrait>(
    interner: &mut VoxInterner<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "remove_at_depth_branch node_id: {:?} position: {:?} depth: {:?}",
        node_id, position, depth
    );

    assert!(interner.is_valid_block_id(node_id));
    assert!(depth.current() < depth.max(), "Branch node at max depth");

    let index = child_index_macro!(position, depth);

    if node_id.has_child(index as u8) {
        let mut branch = interner.get_children(node_id);
        let child_id = branch[index];

        let new_child_id = remove_at_depth(interner, &child_id, position, &depth.increment());

        if new_child_id != BlockId::INVALID {
            assert!(interner.is_valid_block_id(&new_child_id));

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

                interner.inc_child_refs(&branch, index);

                interner.get_or_create_branch(branch, types, mask)
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
    interner: &mut VoxInterner<T>,
    node_id: &BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "remove_at_depth_leaf node_id: {:?} position: {:?} depth: {:?}",
        node_id, position, depth
    );

    assert!(interner.is_valid_block_id(node_id));

    if depth.current() == depth.max() {
        // At max depth, just remove the leaf
        BlockId::EMPTY
    } else {
        // Remove the voxel in the appropriate child, splitting the leaf always results in a new branch
        let new_node_id = remove_at_depth_leaf(interner, node_id, position, &depth.increment());

        assert!(interner.is_valid_block_id(&new_node_id));

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

        interner.inc_ref_by(node_id, 7);

        // Create new branch node
        interner.get_or_create_branch(children, types, mask)
    }
}

#[inline(always)]
fn set_batch_at_root<T: VoxelTrait>(
    interner: &mut VoxInterner<T>,
    node_id: &BlockId,
    max_depth: u8,
    batch: &Batch<T>,
) -> BlockId {
    assert!(*node_id != BlockId::INVALID);

    let depth = TraversalDepth::new(0, max_depth);

    set_batch_at_depth_iterative(interner, node_id, &depth, batch)
}

fn set_batch_at_depth_iterative<T: VoxelTrait>(
    interner: &mut VoxInterner<T>,
    initial_node_id: &BlockId,
    initial_depth: &TraversalDepth,
    batch: &Batch<T>,
) -> BlockId {
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(
        "set_batch_at_depth_iterative initial node: {:?} depth: {:?} batch size: {}",
        initial_node_id,
        initial_depth,
        batch.size()
    );

    // Phase 0: handle fill
    #[cfg(feature = "debug_trace_ref_counts")]
    println!(" Phase 0 - Handle fill",);
    let initial_node_id = if let Some(voxel) = batch.to_fill() {
        let node_id = interner.get_or_create_leaf(voxel);

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            let ref_count = interner.get_ref(&node_id);
            println!(
                "  current: {:?} value: {:?} ref_count: {}",
                node_id, voxel, ref_count
            );
        }

        node_id
    } else {
        *initial_node_id
    };

    if !batch.has_patches() {
        return initial_node_id;
    }

    let max_depth = initial_depth.max() as usize;
    let initial_depth = initial_depth.current() as usize;

    // Phase 1: prepare the chain & build dangling branches
    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 1 - Prepare the chain & build dangling branches",);
        interner.dump_node(initial_node_id, 0, "  ");
    }

    let data_len = batch.masks().len();

    let mut current_level_data = vec![BlockId::INVALID; data_len];
    let mut next_level_data = vec![BlockId::INVALID; data_len];

    let mut paths = Vec::with_capacity(data_len);
    let mut next_paths = Vec::with_capacity(data_len);

    for (path_index, (set_mask, _clear_mask)) in batch.masks().iter().enumerate() {
        if *set_mask == 0 {
            continue;
        }

        let path = path_index << 3;

        let mut current_node_id = initial_node_id;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("  path: 0x{:08X} {:09b}", path, path);

        let mut leaf_node_id = BlockId::EMPTY;

        for current_depth in initial_depth..(max_depth - 1) {
            if current_node_id.is_branch() {
                let index = (path >> ((max_depth - current_depth - 1) * 3)) & 0b111;

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "   depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [1a]",
                    current_depth,
                    max_depth,
                    index,
                    current_node_id,
                    leaf_node_id,
                    interner.get_ref(&current_node_id)
                );

                current_node_id = interner.get_child_id(&current_node_id, index);
            } else {
                // Split leaf node
                leaf_node_id = current_node_id;
                current_node_id = BlockId::EMPTY;

                #[cfg(feature = "debug_trace_ref_counts")]
                {
                    let index = (path >> ((max_depth - current_depth - 1) * 3)) & 0b111;

                    println!(
                        "   depth: {}/{} i: {:2x} current: {:?} leaf: {:?} ref_count: {} [2]",
                        current_depth,
                        max_depth,
                        index,
                        current_node_id,
                        leaf_node_id,
                        interner.get_ref(&current_node_id)
                    );
                }
            }

            if current_node_id.is_empty() {
                break;
            }
        }

        let values = &batch.values()[path_index];

        let all_same = *set_mask == 0xFF && values.iter().all(|v| *v == values[0]);

        if !all_same {
            let (mut children, mut types, mut mask) = if !current_node_id.is_empty() {
                (
                    interner.get_children(&current_node_id),
                    current_node_id.types(),
                    current_node_id.mask(),
                )
            } else if leaf_node_id.is_leaf() {
                let data_len = set_mask.count_ones() as usize;
                interner.inc_ref_by(&leaf_node_id, (MAX_CHILDREN - data_len) as u32);

                ([leaf_node_id; MAX_CHILDREN], 0xFF, 0xFF)
            } else {
                (EMPTY_CHILD, 0, 0)
            };

            let mut modified_childs: u8 = 0;

            let mut set_mask_bits = *set_mask;
            while set_mask_bits != 0 {
                let idx = set_mask_bits.trailing_zeros() as usize;
                set_mask_bits &= !(1 << idx);

                let value = &values[idx];

                if !children[idx].is_empty() && interner.get_value(&children[idx]) == value {
                    // No change needed
                    continue;
                }

                children[idx] = interner.get_or_create_leaf(*value);

                types |= 1 << idx;
                mask |= 1 << idx;
                modified_childs |= 1 << idx;
            }

            if modified_childs == 0 {
                // No changes made
                continue;
            }

            if leaf_node_id.is_empty() {
                let mut non_modified_childs_bits = !modified_childs;
                while non_modified_childs_bits != 0 {
                    let idx = non_modified_childs_bits.trailing_zeros() as usize;
                    non_modified_childs_bits &= !(1 << idx);

                    if !children[idx].is_empty() {
                        // If the child was not modified, we need to increment its ref count
                        interner.inc_ref_by(&children[idx], 1);
                    }
                }
            }

            let branch_id = interner.get_or_create_branch(children, types, mask);

            current_level_data[path_index] = branch_id;
            paths.push(path);
        } else {
            #[cfg(feature = "memory_stats")]
            interner.bump_collapsed_branches();

            let first_value = values[set_mask.trailing_zeros() as usize];
            let leaf_id = interner.get_or_create_leaf(first_value);

            current_level_data[path_index] = leaf_id;
            paths.push(path);
        };
    }

    // Phase 2: Integrate dangling branches
    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 2 - Integrate dangling branches");
    }

    if paths.is_empty() {
        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("  No paths to process");
        }
        return BlockId::INVALID;
    }

    let mut target_depth = max_depth - 1;

    'main: while let Some(mut path) = paths.pop() {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "starting with path: {:08X} {:09b} target_depth: {} paths: {}",
            path,
            path,
            target_depth,
            paths.len(),
        );

        let path_mask_depth = if target_depth > 1 {
            target_depth - 2
        } else {
            0
        };

        let path_mask = PATH_MASKS[max_depth][path_mask_depth] as usize;

        let mut equivalent_id = initial_node_id;
        let mut leaf_id = BlockId::EMPTY;

        // Find equivalent node in the current tree
        for current_depth in 0..(target_depth - 1) {
            if equivalent_id.is_leaf() {
                leaf_id = equivalent_id;
                equivalent_id = BlockId::EMPTY;
                break;
            } else {
                let index = (path >> ((max_depth - current_depth - 1) * 3)) & 0b111;
                equivalent_id = interner.get_child_id(&equivalent_id, index);
            }

            if equivalent_id.is_empty() {
                break;
            }
        }

        if equivalent_id.is_leaf() {
            leaf_id = equivalent_id;
            equivalent_id = BlockId::EMPTY;
        };

        let mut children = EMPTY_CHILD;
        let mut types = 0;
        let mut mask = 0;
        let mut has_next_sibling = true;

        while has_next_sibling {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!(" path: {:08X} {:09b}", path, path);

            let target_index = (path >> ((max_depth - target_depth) * 3)) & 0b111;
            let next_path = paths.last();

            #[cfg(feature = "debug_trace_ref_counts")]
            if let Some(next_path) = next_path {
                println!("  next path: {:08X} {:09b}", next_path, next_path);
            }

            has_next_sibling = if target_depth == 1 {
                next_path.is_some()
            } else if let Some(next_path) = next_path {
                (path & path_mask) == (*next_path & path_mask)
            } else {
                false
            };

            let current_path_index = path >> 3;
            let current_level_id = current_level_data[current_path_index];
            children[target_index] = current_level_id;
            current_level_data[current_path_index] = BlockId::INVALID;

            types |= (current_level_id.is_leaf() as u8) << target_index;
            mask |= 1 << target_index;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!(
                "  new_path: {:08X} {:09b}",
                path & path_mask,
                path & path_mask
            );

            if has_next_sibling && !paths.is_empty() {
                path = paths.pop().expect("No path found");
            }

            #[cfg(feature = "debug_trace_ref_counts")]
            println!("     has_more_paths: {}", !paths.is_empty());
        }

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!(
                "     types: {:08b} mask: {:08b} current_path: {:09b}",
                types,
                mask,
                path & path_mask
            );
            for (child_idx, child) in children.iter().enumerate() {
                println!("     child[{}]: {:?}", child_idx, child);
            }
        }

        let existing_mask = equivalent_id.mask();
        let inv_mask = !mask;
        let cloned_nodes = existing_mask & inv_mask;

        if mask != 0xFF {
            if cloned_nodes != 0 {
                let existing_children = interner.get_children_ref(&equivalent_id);

                let mut cloned_nodes_bits = cloned_nodes;

                while cloned_nodes_bits != 0 {
                    let idx = cloned_nodes_bits.trailing_zeros() as usize;
                    cloned_nodes_bits &= !(1 << idx);

                    children[idx] = existing_children[idx];

                    types |= (children[idx].is_leaf() as u8) << idx;
                    mask |= 1 << idx;
                }
            } else if !leaf_id.is_empty() {
                let leafs_to_clone = inv_mask.count_ones();

                let mut leafs_to_clone_bits = inv_mask;

                types |= inv_mask;
                mask |= inv_mask;

                while leafs_to_clone_bits != 0 {
                    let idx = leafs_to_clone_bits.trailing_zeros() as usize;
                    children[idx] = leaf_id;
                    leafs_to_clone_bits &= !(1 << idx);
                }

                interner.inc_ref_by(&leaf_id, leafs_to_clone);
            }
        }

        let all_same = types == 0xFF && children.iter().all(|item| item == &children[0]);

        let new_node_id = if !all_same {
            let mut cloned_nodes_bits = cloned_nodes;
            while cloned_nodes_bits != 0 {
                let idx = cloned_nodes_bits.trailing_zeros() as usize;
                cloned_nodes_bits &= !(1 << idx);

                interner.inc_ref(&children[idx]);
            }

            interner.get_or_create_branch(children, types, mask)
        } else {
            #[cfg(feature = "memory_stats")]
            interner.bump_collapsed_branches();

            let dec_ref = if cloned_nodes != 0 {
                cloned_nodes.count_ones().min(7)
            } else {
                7
            };

            interner.dec_ref_by(&children[0], dec_ref);

            children[0]
        };

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "     new_node_id: {:?} ref_count: {}",
            new_node_id,
            interner.get_ref(&new_node_id)
        );

        let next_path = path & path_mask;
        next_paths.push(next_path);
        next_level_data[next_path >> 3] = new_node_id;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "     has_more_paths: {} target_depth: {}",
            !paths.is_empty(),
            target_depth
        );

        if paths.is_empty() {
            std::mem::swap(&mut current_level_data, &mut next_level_data);
            std::mem::swap(&mut paths, &mut next_paths);
            next_paths.clear();

            #[cfg(feature = "debug_trace_ref_counts")]
            println!("  paths: {:#?}", paths);

            target_depth -= 1;
            if target_depth == 0 {
                break 'main;
            }
        }
    }

    let final_node_id = current_level_data[paths[0] >> 3];

    #[cfg(feature = "debug_trace_ref_counts")]
    {
        println!(" Phase 3 - Finalize");
        println!("  new_root: {:?}", final_node_id);
        interner.dump_node(final_node_id, 0, "  ");
    }

    final_node_id
}

#[cfg(test)]
mod tests {
    use rand::Rng;

    use crate::utils::common::child_index;

    use super::*;

    #[test]
    fn test_create() {
        let tree = VoxTree::new(MaxDepth::new(3));
        assert!(tree.is_empty());
        assert_eq!(tree.max_depth(Lod::new(0)).max(), 3);
        assert_eq!(tree.voxels_per_axis(Lod::new(0)), 8);
    }

    #[test]
    fn test_child_index() {
        for max_depth in 0..(MAX_ALLOWED_DEPTH as u8) {
            let voxels_per_axis = 1 << max_depth;
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
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024 * 2);

        let mut tree = VoxTree::new(MaxDepth::new(3));
        let position = IVec3::new(0, 0, 0);

        // Test setting and getting a value
        assert!(tree.set(&mut interner, position, 42));
        assert_eq!(tree.get(&interner, position), Some(42));

        // Test overwriting a value
        assert!(tree.set(&mut interner, position, 24));
        assert_eq!(tree.get(&interner, position), Some(24));

        // Test getting from an empty position
        assert_eq!(tree.get(&interner, IVec3::new(1, 1, 1)), None);

        // Test setting at max depth
        let max_pos = IVec3::new(7, 7, 7); // 2^3 - 1
        assert!(tree.set(&mut interner, max_pos, 99));
        assert_eq!(tree.get(&interner, max_pos), Some(99));

        tree.clear(&mut interner);

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
            tree.set(&mut interner, pos, (i + 1) as u8);
        }

        for (i, &pos) in positions.iter().enumerate() {
            assert_eq!(tree.get(&interner, pos).unwrap(), (i + 1) as u8);
        }
    }

    #[test]
    fn test_is_empty() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024);

        let mut tree = VoxTree::new(MaxDepth::new(3));
        assert!(tree.is_empty());

        // Setting a value makes it non-empty
        assert!(tree.set(&mut interner, IVec3::new(0, 0, 0), 1));
        assert!(!tree.is_empty());

        // Clearing makes it empty again
        tree.clear(&mut interner);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_clear() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024 * 2);

        let mut tree = VoxTree::new(MaxDepth::new(3));

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
            tree.set(&mut interner, pos, (i + 1) as u8);
        }

        tree.clear(&mut interner);
        assert!(tree.is_empty());

        for &pos in positions.iter() {
            assert!(tree.get(&interner, pos).is_none());
        }
    }

    #[test]
    fn test_no_default_leaf_nodes() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024);

        let mut tree = VoxTree::new(MaxDepth::new(3));

        // Set a value and then set it back to default
        let position = IVec3::new(0, 0, 0);
        assert!(tree.set(&mut interner, position, 42));
        assert_eq!(tree.get(&interner, position), Some(42));
        assert!(!tree.is_empty());

        // 0 is default for u8
        assert!(tree.set(&mut interner, position, 0));
        // The node should be removed when set to default
        assert_eq!(tree.get(&interner, position), None);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_dirty_flag() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024);

        let mut tree = VoxTree::new(MaxDepth::new(3));
        assert!(!tree.is_dirty());

        // Setting a value should make it dirty
        assert!(tree.set(&mut interner, IVec3::new(0, 0, 0), 1));
        assert!(tree.is_dirty());

        // Clearing the dirty flag
        tree.clear_dirty();
        assert!(!tree.is_dirty());

        // Clearing the voxtree should make it dirty again
        tree.clear(&mut interner);
        assert!(tree.is_dirty());
    }

    #[test]
    fn test_shared_interner_uniqueness() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024);

        let mut tree1 = VoxTree::new(MaxDepth::new(3));
        let mut tree2 = VoxTree::new(MaxDepth::new(3));

        // Both trees should be empty initially
        assert!(tree1.is_empty());
        assert!(tree2.is_empty());

        // Setting in one tree should not affect the other
        assert!(tree1.set(&mut interner, IVec3::new(0, 0, 0), 42));
        assert_eq!(tree1.get(&interner, IVec3::new(0, 0, 0)), Some(42));
        assert_eq!(tree2.get(&interner, IVec3::new(0, 0, 0)), None);

        // But they should share the same interner for efficiency
        assert!(tree2.set(&mut interner, IVec3::new(0, 0, 0), 24));
        assert_eq!(tree2.get(&interner, IVec3::new(0, 0, 0)), Some(24));
        assert_ne!(tree1.get_root_id(), tree2.get_root_id());
    }

    #[test]
    fn test_shared_interner_deduplication() {
        let mut interner = VoxInterner::<u8>::with_memory_budget(1024);

        let mut tree1 = VoxTree::new(MaxDepth::new(3));
        let mut tree2 = VoxTree::new(MaxDepth::new(3));

        // Both trees should be empty initially
        assert!(tree1.is_empty());
        assert!(tree2.is_empty());

        // Setting same value in both trees should result in the same root id (deduplication)
        assert!(tree1.set(&mut interner, IVec3::new(0, 0, 0), 42));
        assert!(tree2.set(&mut interner, IVec3::new(0, 0, 0), 42));
        assert_eq!(tree1.get_root_id(), tree2.get_root_id());
    }

    #[test]
    fn test_set_behaviour() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);

        let position = IVec3::new(0, 0, 0);
        assert!(tree.set(&mut interner, position, TEST_VALUE));
        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));

        // Test overwriting a value
        assert!(tree.set(&mut interner, position, TEST_VALUE + 1));
        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE + 1));

        // Test setting same value
        assert!(!tree.set(&mut interner, position, TEST_VALUE + 1));
        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE + 1));
    }

    #[test]
    fn test_batch_double_apply() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);

        let position = IVec3::new(0, 0, 0);

        let mut batch = tree.create_batch();

        batch.set(&mut interner, position, TEST_VALUE);

        assert!(tree.apply_batch(&mut interner, &batch));
        assert!(!tree.apply_batch(&mut interner, &batch));
    }

    #[test]
    fn test_patterns_set_expand_shared_leaf() {
        const START_VALUE: u8 = 1;
        const END_VALUE: u8 = 6;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for value in START_VALUE..END_VALUE {
            tree.fill(&mut interner, value * 10);
            tree.set(&mut interner, IVec3::new(0, 0, 0), value);

            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let position = IVec3::new(x, y, z);
                        let value = if x == 0 && y == 0 && z == 0 {
                            value
                        } else {
                            value * 10
                        };
                        assert_eq!(tree.get(&interner, position), Some(value));
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_expand_shared_leaf() {
        const FILL_VALUE: u8 = 1;
        const TEST_VALUE: u8 = 2;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut branch = tree.create_batch();

        branch.fill(&mut interner, FILL_VALUE);
        branch.set(&mut interner, IVec3::new(0, 0, 0), TEST_VALUE);

        assert!(tree.apply_batch(&mut interner, &branch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    let value = if x == 0 && y == 0 && z == 0 {
                        TEST_VALUE
                    } else {
                        FILL_VALUE
                    };
                    assert_eq!(tree.get(&interner, position), Some(value));
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_checkerboard() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        // Create a checkerboard pattern
        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    if (x + y + z) % 2 == 0 {
                        let position = IVec3::new(x, y, z);
                        let value = 2;
                        assert!(tree.set(&mut interner, position, value));
                        assert_eq!(tree.get(&interner, position), Some(value));
                    }
                }
            }
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    let value = if (x + y + z) % 2 == 0 { Some(2) } else { None };
                    assert_eq!(tree.get(&interner, position), value);
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_checkerboard() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        // Create a checkerboard pattern
        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    let value = if (x + y + z) % 2 == 0 { 2 } else { 1 };
                    assert!(batch.set(&mut interner, position, value));
                }
            }
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    let value = if (x + y + z) % 2 == 0 { 2 } else { 1 };
                    assert_eq!(tree.get(&interner, position), Some(value));
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_solid_fill_one_by_one() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert!(tree.set(&mut interner, position, TEST_VALUE));
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }
        assert!(!tree.is_empty());
        assert!(tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_solid_fill_one_by_one() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert!(batch.set(&mut interner, position, TEST_VALUE));
                }
            }
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }
        assert!(!tree.is_empty());
        assert!(tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_solid_fill_half_one_by_one() {
        const START_VALUE: u8 = 1;
        const END_VALUE: u8 = 6;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;
        let half_voxels_per_axis = voxels_per_axis / 2;

        for value in START_VALUE..END_VALUE {
            for y in 0..half_voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let position = IVec3::new(x, y, z);
                        assert!(tree.set(&mut interner, position, value));
                        assert_eq!(tree.get(&interner, position), Some(value));
                    }
                }
            }

            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let position = IVec3::new(x, y, z);
                        assert_eq!(
                            tree.get(&interner, position),
                            if y < half_voxels_per_axis {
                                Some(value)
                            } else {
                                None
                            }
                        );
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_solid_fill_half_one_by_one() {
        const START_VALUE: u8 = 1;
        const END_VALUE: u8 = 6;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;
        let half_voxels_per_axis = voxels_per_axis / 2;

        for value in START_VALUE..END_VALUE {
            let mut batch = tree.create_batch();

            for y in 0..half_voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let position = IVec3::new(x, y, z);
                        assert!(batch.set(&mut interner, position, value));
                    }
                }
            }

            assert!(tree.apply_batch(&mut interner, &batch));

            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let position = IVec3::new(x, y, z);
                        assert_eq!(
                            tree.get(&interner, position),
                            if y < half_voxels_per_axis {
                                Some(value)
                            } else {
                                None
                            }
                        );
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_solid_fill_fill_op() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        tree.fill(&mut interner, TEST_VALUE);

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_solid_fill_fill_op() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        batch.fill(&mut interner, TEST_VALUE);

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_sparse_fill() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for y in (0..voxels_per_axis).step_by(4) {
            for z in (0..voxels_per_axis).step_by(4) {
                for x in (0..voxels_per_axis).step_by(4) {
                    let position = IVec3::new(x, y, z);
                    assert!(tree.set(&mut interner, position, TEST_VALUE));
                    assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                }
            }
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);

                    if x % 4 == 0 && y % 4 == 0 && z % 4 == 0 {
                        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                    } else {
                        assert_eq!(tree.get(&interner, position), None);
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_sparse_fill() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        for y in (0..voxels_per_axis).step_by(4) {
            for z in (0..voxels_per_axis).step_by(4) {
                for x in (0..voxels_per_axis).step_by(4) {
                    let position = IVec3::new(x, y, z);
                    assert!(batch.set(&mut interner, position, TEST_VALUE));
                }
            }
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);

                    if x % 4 == 0 && y % 4 == 0 && z % 4 == 0 {
                        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                    } else {
                        assert_eq!(tree.get(&interner, position), None);
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_gradient_fill() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for x in 0..voxels_per_axis {
            let value = (x % 256) as u8;
            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(tree.set(&mut interner, position, value), value > 0);
                    assert_eq!(
                        tree.get(&interner, position),
                        if value > 0 { Some(value) } else { None }
                    );
                }
            }
        }

        for x in 0..voxels_per_axis {
            let value = (x % 256) as u8;
            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if value > 0 { Some(value) } else { None }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_gradient_fill() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        for x in 0..voxels_per_axis {
            let value = (x % 256) as u8;
            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert!(batch.set(&mut interner, position, value));
                }
            }
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for x in 0..voxels_per_axis {
            let value = (x % 256) as u8;
            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if value > 0 { Some(value) } else { None }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_hollow_cube() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let is_edge = x == 0
                        || x == voxels_per_axis - 1
                        || y == 0
                        || y == voxels_per_axis - 1
                        || z == 0
                        || z == voxels_per_axis - 1;

                    let position = IVec3::new(x, y, z);
                    if is_edge {
                        assert!(tree.set(&mut interner, position, TEST_VALUE));
                        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                    } else {
                        assert_eq!(tree.get(&interner, position), None);
                    }
                }
            }
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let is_edge = x == 0
                        || x == voxels_per_axis - 1
                        || y == 0
                        || y == voxels_per_axis - 1
                        || z == 0
                        || z == voxels_per_axis - 1;

                    let position = IVec3::new(x, y, z);
                    if is_edge {
                        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                    } else {
                        assert_eq!(tree.get(&interner, position), None);
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_hollow_cube() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let is_edge = x == 0
                        || x == voxels_per_axis - 1
                        || y == 0
                        || y == voxels_per_axis - 1
                        || z == 0
                        || z == voxels_per_axis - 1;

                    let position = IVec3::new(x, y, z);
                    if is_edge {
                        assert!(batch.set(&mut interner, position, TEST_VALUE));
                    }
                }
            }
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let is_edge = x == 0
                        || x == voxels_per_axis - 1
                        || y == 0
                        || y == voxels_per_axis - 1
                        || z == 0
                        || z == voxels_per_axis - 1;

                    let position = IVec3::new(x, y, z);
                    if is_edge {
                        assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
                    } else {
                        assert_eq!(tree.get(&interner, position), None);
                    }
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_diagonal() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        for i in 0..voxels_per_axis {
            let position = IVec3::new(i, i, i);
            assert!(tree.set(&mut interner, position, TEST_VALUE));
            assert_eq!(tree.get(&interner, position), Some(TEST_VALUE));
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if x == y && x == z {
                            Some(TEST_VALUE)
                        } else {
                            None
                        }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_diagonal() {
        const TEST_VALUE: u8 = 3;
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);
        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;

        let mut batch = tree.create_batch();

        for i in 0..voxels_per_axis {
            let position = IVec3::new(i, i, i);
            assert!(batch.set(&mut interner, position, TEST_VALUE));
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if x == y && x == z {
                            Some(TEST_VALUE)
                        } else {
                            None
                        }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_set_random_noise() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);

        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;
        let size = 1 << (3 * MAX_DEPTH.max());
        let mut data = vec![0; size as usize];

        let mut rng = rand::rng();

        for _ in 0..1000 {
            let x = rng.random_range(0..voxels_per_axis);
            let y = rng.random_range(0..voxels_per_axis);
            let z = rng.random_range(0..voxels_per_axis);
            let index = z * voxels_per_axis * voxels_per_axis + y * voxels_per_axis + x;
            let value = rng.random_range(1..=255) as u8;
            data[index as usize] = value;
            let position = IVec3::new(x, y, z);

            assert!(tree.set(&mut interner, position, value));
            assert_eq!(tree.get(&interner, position), Some(value));
        }

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let index = z * voxels_per_axis * voxels_per_axis + y * voxels_per_axis + x;
                    let expected_value = data[index as usize];
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if expected_value != 0 {
                            Some(expected_value)
                        } else {
                            None
                        }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_patterns_batch_random_noise() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(5);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);

        let voxels_per_axis = tree.voxels_per_axis(Lod::new(0)) as i32;
        let size = 1 << (3 * MAX_DEPTH.max());
        let mut data = vec![0; size as usize];

        let mut batch = tree.create_batch();

        let mut rng = rand::rng();

        for _ in 0..1000 {
            let x = rng.random_range(0..voxels_per_axis);
            let y = rng.random_range(0..voxels_per_axis);
            let z = rng.random_range(0..voxels_per_axis);
            let index = z * voxels_per_axis * voxels_per_axis + y * voxels_per_axis + x;
            let value = rng.random_range(1..=255) as u8;
            data[index as usize] = value;
            let position = IVec3::new(x, y, z);

            assert!(batch.set(&mut interner, position, value));
        }

        assert!(tree.apply_batch(&mut interner, &batch));

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let index = z * voxels_per_axis * voxels_per_axis + y * voxels_per_axis + x;
                    let expected_value = data[index as usize];
                    let position = IVec3::new(x, y, z);
                    assert_eq!(
                        tree.get(&interner, position),
                        if expected_value != 0 {
                            Some(expected_value)
                        } else {
                            None
                        }
                    );
                }
            }
        }

        assert!(!tree.is_empty());
        assert!(!tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }

    #[test]
    fn test_max_depth_zero() {
        const MAX_DEPTH: MaxDepth = MaxDepth::new(0);
        const MEMORY_BUDGET: usize = 1024 * 1024;

        let mut interner = VoxInterner::<u8>::with_memory_budget(MEMORY_BUDGET);
        let mut tree = VoxTree::new(MAX_DEPTH);

        let position = IVec3::new(0, 0, 0);
        assert!(tree.set(&mut interner, position, 1));
        assert_eq!(tree.get(&interner, position), Some(1));

        assert!(!tree.is_empty());
        assert!(tree.is_leaf());
        assert_eq!(interner.get_ref(&tree.get_root_id()), 1);
    }
}
