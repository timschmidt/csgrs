//! Hash-consed storage shared by Voxelis trees and chunks.
//!
//! [`VoxInterner`] assigns generation-tagged [`BlockId`] values, deduplicates
//! equal leaves and branches, and recycles slots when reference counts reach
//! zero. IDs are meaningful only with the interner that issued them.

use std::collections::{HashMap, hash_map::Entry};

use voxelis_memory::PoolAllocatorLite;

use crate::{BlockId, VoxelTrait, get_next_index_macro};

mod consts;
mod hash;
mod macros;
#[cfg(feature = "memory_stats")]
mod stats;

pub use consts::*;
pub use hash::PatternsHashmap;
#[cfg(feature = "memory_stats")]
pub use stats::InternerStats;

use hash::{
    IdentityHasherBuilder, compute_branch_hash_for_children, compute_empty_branch_hash,
    compute_leaf_hash_for_value,
};

pub type Children = [BlockId; MAX_CHILDREN];

/// Advances through a full-period probe sequence when two distinct node keys
/// have the same 64-bit fingerprint. The odd increment visits every `u64`
/// value before repeating while dispersing probes for the identity hasher.
#[inline(always)]
const fn next_pattern_key(key: u64) -> u64 {
    key.wrapping_add(0x9E37_79B9_7F4A_7C15)
}

/// Memory-budgeted owner of deduplicated voxel DAG nodes.
pub struct VoxInterner<T> {
    patterns: [PatternsHashmap; 2],
    free_indices: Vec<u32>,
    next_index: u32,
    ref_counts: PoolAllocatorLite<u32>,
    generations: PoolAllocatorLite<u16>,
    children: PoolAllocatorLite<Children>,
    values: PoolAllocatorLite<T>,
    hashes: PoolAllocatorLite<u64>,
    has_pattern_collisions: [bool; 2],
    capacity: usize,
    empty_branch_id: BlockId,
    empty_branch_hash: u64,
    dec_ref_rec_stack: Vec<BlockId>,
    #[cfg(feature = "memory_stats")]
    stats: InternerStats,
}

impl<T: VoxelTrait> VoxInterner<T> {
    // Avoid early hash-table growth in normal model workloads.
    const INITIAL_CAPACITY: usize = 16384;

    /// Creates an interner whose node pools fit within `requested_budget`.
    ///
    /// The usable budget is rounded down to a whole number of node slots.
    ///
    /// # Panics
    ///
    /// Panics when the budget cannot hold one node or would require more than
    /// `u32::MAX` slots.
    pub fn with_memory_budget(requested_budget: usize) -> Self {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::with_memory_budget");

        let single_node_size = Self::node_size();

        let nodes_capacity = requested_budget / single_node_size;
        let actual_budget = nodes_capacity * single_node_size;

        assert!(nodes_capacity > 0, "Requested budget is too small");
        assert!(actual_budget > 0, "Requested budget is too small");
        assert!(
            nodes_capacity <= u32::MAX as usize,
            "Requested budget is too large"
        );

        let free_indices = Vec::with_capacity(nodes_capacity);

        let mut ref_counts = PoolAllocatorLite::new(nodes_capacity);
        let mut generations = PoolAllocatorLite::new(nodes_capacity);
        let mut children = PoolAllocatorLite::new(nodes_capacity);
        let mut values = PoolAllocatorLite::new(nodes_capacity);
        let mut hashes = PoolAllocatorLite::new(nodes_capacity);

        let mut branch_patterns =
            HashMap::with_capacity_and_hasher(Self::INITIAL_CAPACITY, IdentityHasherBuilder);
        let leafs_patterns =
            HashMap::with_capacity_and_hasher(Self::INITIAL_CAPACITY, IdentityHasherBuilder);

        let empty_branch_hash = compute_empty_branch_hash();

        let empty_branch_index = 0;
        let empty_branch_generation = 0;
        let empty_branch_id =
            BlockId::new_branch(empty_branch_index, empty_branch_generation, 0, 0);
        assert_eq!(empty_branch_id, BlockId::EMPTY, "Empty branch id mismatch");

        *ref_counts.get_mut(empty_branch_index) = 0;
        *generations.get_mut(empty_branch_index) = empty_branch_generation;
        *children.get_mut(empty_branch_index) = EMPTY_CHILD;
        *values.get_mut(empty_branch_index) = T::default();
        *hashes.get_mut(empty_branch_index) = empty_branch_hash;
        branch_patterns.insert(empty_branch_hash, empty_branch_id);

        let dec_ref_rec_stack = vec![BlockId::EMPTY; PREALLOCATED_STACK_SIZE];

        let next_index = empty_branch_index + 1;

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("empty_branch_hash: {empty_branch_hash:X}");
            println!("empty_branch_id: {empty_branch_id:?}");
            println!("next_index: {next_index}");
        }

        #[cfg(feature = "memory_stats")]
        let stats = InternerStats {
            requested_budget,
            actual_budget,
            node_size: single_node_size,
            nodes_capacity,
            total_allocations: 1,
            total_deallocations: 0,
            allocated_nodes: 1,
            recycled_nodes: 0,
            alive_nodes: 1,
            patterns: 1,
            total_cache_hits: 0,
            total_cache_misses: 0,
            branch_cache_hits: 0,
            branch_cache_misses: 0,
            leaf_cache_hits: 0,
            leaf_cache_misses: 0,
            collapsed_branches: 0,
            leaf_nodes: 0,
            branch_nodes: 1,
            max_alive_nodes: 0,
            max_node_id: 0,
            max_branch_ref_count: 0,
            max_leaf_ref_count: 0,
            max_generation: 0,
            generations_overflows: 0,
        };

        Self {
            free_indices,
            next_index,
            ref_counts,
            generations,
            children,
            values,
            hashes,
            patterns: [branch_patterns, leafs_patterns],
            has_pattern_collisions: [false; 2],
            capacity: nodes_capacity,
            empty_branch_id,
            empty_branch_hash,
            dec_ref_rec_stack,
            #[cfg(feature = "memory_stats")]
            stats,
        }
    }

    #[inline(always)]
    pub const fn node_size() -> usize {
        PoolAllocatorLite::<u32>::block_size() + // ref_count
        PoolAllocatorLite::<u16>::block_size() + // generation
        PoolAllocatorLite::<Children>::block_size() + // children
        PoolAllocatorLite::<T>::block_size() + // value
        PoolAllocatorLite::<u64>::block_size() // hash
    }

    #[inline(always)]
    pub fn get_value(&self, block_id: &BlockId) -> &T {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_value");

        self.values.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_children(&self, block_id: &BlockId) -> Children {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_children");

        *self.children.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_children_ref(&self, block_id: &BlockId) -> &Children {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_children_ref");

        self.children.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_child_id(&self, block_id: &BlockId, index: usize) -> BlockId {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_child_id");

        self.children.get(block_id.index())[index]
    }

    #[inline(always)]
    pub fn get_ref(&self, block_id: &BlockId) -> u32 {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_ref");

        *self.ref_counts.get(block_id.index())
    }

    #[inline(always)]
    pub fn inc_ref(&mut self, block_id: &BlockId) {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::inc_ref");

        *self.ref_counts.get_mut(block_id.index()) += 1;

        #[cfg(feature = "memory_stats")]
        {
            if block_id.is_branch() {
                self.stats.max_branch_ref_count = self
                    .stats
                    .max_branch_ref_count
                    .max(*self.ref_counts.get(block_id.index()) as usize);
            } else {
                self.stats.max_leaf_ref_count = self
                    .stats
                    .max_leaf_ref_count
                    .max(*self.ref_counts.get(block_id.index()) as usize);
            }
        }
    }

    pub fn dec_ref(&mut self, block_id: &BlockId) -> bool {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::dec_ref");

        let block_index = block_id.index();

        let ref_count = self.ref_counts.get_mut(block_index);

        debug_assert!(
            *ref_count > 0,
            "Ref count should be greater than zero, id: {block_id:?}"
        );

        *ref_count -= 1;

        if *ref_count == 0 {
            self.remove_pattern(*block_id);

            #[cfg(feature = "memory_stats")]
            {
                self.stats.patterns -= 1;
            }

            self.recycle(block_id);

            true
        } else {
            false
        }
    }

    pub fn inc_child_refs(&mut self, children: &Children, index: usize) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("Incrementing ref count for children: {children:?}");

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::inc_child_refs");

        for (i, child_id) in children.iter().enumerate() {
            if i == index {
                #[cfg(feature = "debug_trace_ref_counts")]
                println!("  [{i}] Skipping child_id: {child_id:?}");

                continue;
            }

            if !child_id.is_empty() {
                #[cfg(feature = "debug_trace_ref_counts")]
                let current_ref_count = self.get_ref(child_id);

                self.inc_ref(child_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "  [{i}] Incrementing ref count for child_id: {child_id:?} ref_count: {current_ref_count} -> {}",
                    self.get_ref(child_id),
                );
            }
        }
    }

    pub fn inc_all_child_refs(&mut self, children: &Children) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("Incrementing ref count for children: {children:?}");

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::inc_all_child_refs");

        for child_id in children.iter() {
            if !child_id.is_empty() {
                #[cfg(feature = "debug_trace_ref_counts")]
                let current_ref_count = self.get_ref(child_id);

                self.inc_ref(child_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "  Incrementing ref count for child_id: {child_id:?} ref_count: {current_ref_count} -> {}",
                    self.get_ref(child_id),
                );
            }
        }
    }

    pub fn inc_ref_by(&mut self, block_id: &BlockId, count: u32) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("Incrementing ref count for block: {block_id:?} by {count}");

        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::inc_ref_by");

        *self.ref_counts.get_mut(block_id.index()) += count;

        #[cfg(feature = "memory_stats")]
        {
            if block_id.is_branch() {
                self.stats.max_branch_ref_count = self
                    .stats
                    .max_branch_ref_count
                    .max(*self.ref_counts.get(block_id.index()) as usize);
            } else {
                self.stats.max_leaf_ref_count = self
                    .stats
                    .max_leaf_ref_count
                    .max(*self.ref_counts.get(block_id.index()) as usize);
            }
        }
    }

    pub fn dec_ref_by(&mut self, block_id: &BlockId, count: u32) {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::dec_ref_by");

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("Decrementing ref count for block: {block_id:?} by {count}");

        let block_index = block_id.index();

        let ref_count = self.ref_counts.get_mut(block_index);

        debug_assert!(
            ((*ref_count as i64) + count as i64) >= 0,
            "Ref count should be greater or equal than zero, id: {block_id:?}"
        );

        *ref_count -= count;

        if *ref_count == 0 {
            self.remove_pattern(*block_id);

            #[cfg(feature = "memory_stats")]
            {
                self.stats.patterns -= 1;
            }

            self.recycle(block_id);
        }
    }

    pub fn dec_ref_recursive(&mut self, block_id: &BlockId) {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {block_id:?}"
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::dec_ref_recursive");

        let stack_len = self.dec_ref_rec_stack.len();
        assert!(stack_len > 0, "dec_ref_rec_stack must not be empty");

        let stack_ptr = self.dec_ref_rec_stack.as_mut_ptr();

        // SAFETY: the stack has at least one initialized element, and the raw
        // pointer avoids holding a mutable Vec borrow while other `self`
        // fields are accessed during traversal.
        unsafe {
            *stack_ptr.add(0) = *block_id;
        }

        let mut read_idx = 0;
        let mut write_idx = 1;

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("dec_ref_recursive block_id: {block_id:?}");
            self.dump_node(*block_id, 0, "  ");
        }

        while read_idx < write_idx {
            // SAFETY: entries below `write_idx` were initialized before the
            // counter advanced, and every write is checked against `stack_len`.
            let current_id = unsafe { *stack_ptr.add(read_idx) };
            read_idx += 1;

            #[cfg(debug_assertions)]
            assert!(self.is_valid_block_id(&current_id));

            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!(
                    " {}/{write_idx}/{stack_len} Processing: {current_id:?}",
                    read_idx - 1,
                );
                self.dump_node(current_id, 0, "  ");
            }

            let current_index = current_id.index();
            let ref_count = self.ref_counts.get_mut(current_index);

            #[cfg(feature = "debug_trace_ref_counts")]
            let current_ref_count = *ref_count;

            debug_assert!(
                *ref_count > 0,
                "Ref count should be greater than zero, id: {current_id:?}"
            );

            *ref_count -= 1;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!("    ref_count = {current_ref_count} -> {ref_count}");

            if *ref_count == 0 {
                for child in self.children.get(current_index) {
                    if !child.is_empty() {
                        #[cfg(debug_assertions)]
                        assert!(self.is_valid_block_id(child));

                        let child_ref_count = self.ref_counts.get_mut(child.index());
                        if *child_ref_count > 1 {
                            #[cfg(feature = "debug_trace_ref_counts")]
                            println!(
                                "      handling child: {child:?} at {write_idx} ref_count: {} -> {} in place",
                                *child_ref_count,
                                *child_ref_count - 1,
                            );
                            *child_ref_count -= 1;
                        } else {
                            #[cfg(feature = "debug_trace_ref_counts")]
                            println!(
                                "      adding child:   {child:?} at {write_idx} ref_count: {child_ref_count}",
                            );
                            assert!(
                                write_idx < stack_len,
                                "dec_ref_rec_stack overflow: {write_idx}, length: {stack_len}"
                            );
                            // SAFETY: the preceding check places `write_idx`
                            // within the initialized backing array. This entry
                            // becomes readable only after `write_idx` advances.
                            unsafe {
                                *stack_ptr.add(write_idx) = *child;
                            }
                            write_idx += 1;
                        }
                    }
                }

                self.remove_pattern(current_id);

                #[cfg(feature = "memory_stats")]
                {
                    self.stats.patterns -= 1;
                }

                self.recycle(&current_id);
            }
        }

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            " ...done read_idx: {read_idx}, write_idx: {write_idx} capacity: {}",
            self.dec_ref_rec_stack.capacity()
        );
    }

    pub fn dec_child_refs(&mut self, children: &Children) {
        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("Decrementing ref count for children: {children:?}");

            for (i, child_id) in children.iter().enumerate() {
                if !child_id.is_empty() {
                    let current_ref_count = self.get_ref(child_id);

                    let _recycled = self.dec_ref(child_id);

                    println!(
                        "  [{i}] Decrementing ref count for child_id: {child_id:?} ref_count: {current_ref_count} -> {}",
                        if _recycled { 0 } else { self.get_ref(child_id) },
                    );
                }
            }
        }

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::dec_child_refs");

        #[cfg(not(feature = "debug_trace_ref_counts"))]
        for child_id in children.iter() {
            if !child_id.is_empty() {
                self.dec_ref(child_id);
            }
        }
    }

    pub fn recycle(&mut self, block_id: &BlockId) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("recycle block_id: {block_id:?}");

        debug_assert!(
            block_id != &self.empty_branch_id,
            "Cannot recycle empty branch",
        );

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::recycle");

        let block_index = block_id.index();

        // Reset every payload before making the slot available for reuse.
        *self.values.get_mut(block_index) = T::default();
        *self.children.get_mut(block_index) = EMPTY_CHILD;
        *self.hashes.get_mut(block_index) = 0;
        *self.ref_counts.get_mut(block_index) = 0;
        let generation = self.generations.get_mut(block_index);
        *generation += 1;

        if *generation >= BlockId::MAX_GENERATION {
            *generation = 0;

            #[cfg(feature = "memory_stats")]
            {
                self.stats.generations_overflows += 1;
            }
        }

        #[cfg(feature = "memory_stats")]
        {
            self.stats.max_generation = self.stats.max_generation.max(*generation as usize);
        }

        debug_assert!(
            !self.free_indices.contains(&block_index),
            "Double free detected!"
        );

        self.free_indices.push(block_index);

        #[cfg(feature = "memory_stats")]
        {
            self.stats.alive_nodes -= 1;
            self.stats.total_deallocations += 1;
            self.stats.recycled_nodes += 1;
            let is_leaf = block_id.is_leaf();
            self.stats.leaf_nodes -= is_leaf as usize;
            self.stats.branch_nodes -= (!is_leaf) as usize;
            if self.stats.alive_nodes > 1 {
                debug_assert!(self.stats.leaf_nodes > 0);
            }
        }

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("  Node recycled");
    }

    #[inline(always)]
    pub fn get_or_create_leaf(&mut self, value: T) -> BlockId {
        debug_assert_ne!(value, T::default(), "Leaf value should not be default");

        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_or_create_leaf");

        let fingerprint = compute_leaf_hash_for_value(&value);
        self.get_or_create_leaf_with_fingerprint(value, fingerprint)
    }

    #[inline(always)]
    fn get_or_create_leaf_with_fingerprint(&mut self, value: T, fingerprint: u64) -> BlockId {
        let mut pattern_key = fingerprint;
        loop {
            match self.patterns[PATTERNS_TYPE_LEAF].entry(pattern_key) {
                Entry::Occupied(entry) => {
                    let existing_id = *entry.get();
                    if !self.is_valid_block_id(&existing_id) {
                        #[cfg(feature = "debug_trace_ref_counts")]
                        self.dump_patterns();

                        panic!(
                            "Expired node in patterns: {existing_id:?} pattern key = {pattern_key:X}"
                        );
                    }
                    if self.values.get(existing_id.index()) == &value {
                        self.inc_ref(&existing_id);

                        #[cfg(feature = "debug_trace_ref_counts")]
                        println!(
                            "get_or_create_leaf: value: {value} fingerprint = {fingerprint:X} pattern_key = {pattern_key:X} existing_id = {existing_id:?} ref_count: {}",
                            self.get_ref(&existing_id)
                        );

                        #[cfg(feature = "memory_stats")]
                        {
                            self.stats.total_cache_hits += 1;
                            self.stats.leaf_cache_hits += 1;
                        }

                        return existing_id;
                    }

                    pattern_key = next_pattern_key(pattern_key);
                }
                Entry::Vacant(entry) => {
                    if pattern_key != fingerprint {
                        self.has_pattern_collisions[PATTERNS_TYPE_LEAF] = true;
                    }
                    let index = get_next_index_macro!(self);

                    let generation = *self.generations.get(index);

                    let block_id = BlockId::new_leaf(index, generation);

                    entry.insert(block_id);

                    *self.values.get_mut(index) = value;
                    *self.hashes.get_mut(index) = pattern_key;

                    debug_assert_eq!(
                        self.get_ref(&block_id),
                        0,
                        "New node should have zero ref count"
                    );

                    self.inc_ref(&block_id);

                    #[cfg(feature = "debug_trace_ref_counts")]
                    println!(
                        "get_or_create_leaf: value: {value} fingerprint = {fingerprint:X} pattern_key = {pattern_key:X} new_id = {block_id:?} ref_count: {}",
                        self.get_ref(&block_id)
                    );

                    #[cfg(feature = "memory_stats")]
                    {
                        self.stats.leaf_nodes += 1;
                        self.stats.patterns += 1;
                        self.stats.total_cache_misses += 1;
                        self.stats.leaf_cache_misses += 1;
                    }

                    return block_id;
                }
            }
        }
    }

    /// Interns a branch and transfers ownership of the supplied child references.
    ///
    /// Every non-empty child must already have one reference held for this
    /// call. A cache hit consumes those references and returns a reference to
    /// the existing branch; a miss retains them in the newly created branch.
    #[inline(always)]
    pub fn get_or_create_branch(&mut self, children: Children, types: u8, mask: u8) -> BlockId {
        #[cfg(feature = "tracy")]
        let _span = tracy_client::span!("VoxInterner::get_or_create_branch");

        let fingerprint = compute_branch_hash_for_children(&children, types, mask);
        self.get_or_create_branch_with_fingerprint(children, types, mask, fingerprint)
    }

    #[inline(always)]
    fn get_or_create_branch_with_fingerprint(
        &mut self,
        children: Children,
        types: u8,
        mask: u8,
        fingerprint: u64,
    ) -> BlockId {
        let mut pattern_key = fingerprint;
        loop {
            match self.patterns[PATTERNS_TYPE_BRANCH].entry(pattern_key) {
                Entry::Occupied(entry) => {
                    let existing_id = *entry.get();
                    assert!(
                        self.is_valid_block_id(&existing_id),
                        "Expired node in patterns: {existing_id:?} pattern key = {pattern_key:X}"
                    );
                    if existing_id.types() == types
                        && existing_id.mask() == mask
                        && self.children.get(existing_id.index()) == &children
                    {
                        #[cfg(feature = "debug_trace_ref_counts")]
                        println!(
                            "get_or_create_branch: children: {children:?} fingerprint = {fingerprint:X} pattern_key = {pattern_key:X} existing_id = {existing_id:?}"
                        );

                        self.dec_child_refs(&children);

                        #[cfg(debug_assertions)]
                        self.ensure_valid_children(&children);

                        self.inc_ref(&existing_id);

                        #[cfg(feature = "memory_stats")]
                        {
                            self.stats.total_cache_hits += 1;
                            self.stats.branch_cache_hits += 1;
                        }

                        return existing_id;
                    }

                    pattern_key = next_pattern_key(pattern_key);
                }
                Entry::Vacant(entry) => {
                    if pattern_key != fingerprint {
                        self.has_pattern_collisions[PATTERNS_TYPE_BRANCH] = true;
                    }
                    let index = get_next_index_macro!(self);

                    let generation = *self.generations.get(index);

                    let block_id = BlockId::new_branch(index, generation, types, mask);

                    debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

                    entry.insert(block_id);

                    // Cache the aggregate value used by coarser-LOD reads.
                    let values: [T; 8] =
                        std::array::from_fn(|i| *self.values.get(children[i].index()));
                    let average = T::average(&values);

                    *self.children.get_mut(index) = children;
                    *self.values.get_mut(index) = average;
                    *self.hashes.get_mut(index) = pattern_key;

                    #[cfg(feature = "debug_trace_ref_counts")]
                    println!(
                        "get_or_create_branch: children: {children:?} fingerprint = {fingerprint:X} pattern_key = {pattern_key:X} new_id: {block_id:?} types: {types:2X} mask: {mask:2X}"
                    );

                    debug_assert_eq!(
                        self.get_ref(&block_id),
                        0,
                        "New node should have zero ref count"
                    );

                    self.inc_ref(&block_id);

                    #[cfg(feature = "memory_stats")]
                    {
                        self.stats.branch_nodes += 1;
                        self.stats.patterns += 1;
                        self.stats.total_cache_misses += 1;
                        self.stats.branch_cache_misses += 1;
                    }

                    return block_id;
                }
            }
        }
    }

    #[cfg(feature = "memory_stats")]
    pub fn bump_collapsed_branches(&mut self) {
        self.stats.collapsed_branches += 1;
    }

    pub fn create_empty_branch(&mut self) -> BlockId {
        let index = get_next_index_macro!(self);

        let generation = *self.generations.get(index);

        let block_id = BlockId::new_branch(index, generation, 0, 0);

        debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

        *self.children.get_mut(index) = EMPTY_CHILD;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("create_empty_branch: new_id: {block_id:?}");

        #[cfg(feature = "memory_stats")]
        {
            self.stats.branch_nodes += 1;
        }

        debug_assert_eq!(
            self.get_ref(&block_id),
            0,
            "New node should have zero ref count"
        );

        self.inc_ref(&block_id);

        block_id
    }

    pub fn create_branch(&mut self, children: Children, types: u8, mask: u8) -> BlockId {
        let index = get_next_index_macro!(self);

        let generation = *self.generations.get(index);

        let block_id = BlockId::new_branch(index, generation, types, mask);

        debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

        *self.children.get_mut(index) = children;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "create_branch: children: {children:?} new_id: {block_id:?} types: {types:2X} mask: {mask:2X}"
        );

        #[cfg(feature = "memory_stats")]
        {
            self.stats.branch_nodes += 1;
        }

        debug_assert_eq!(
            self.get_ref(&block_id),
            0,
            "New node should have zero ref count"
        );

        self.inc_ref(&block_id);

        block_id
    }

    pub fn update_branch(
        &mut self,
        block_id: &BlockId,
        child_id: &BlockId,
        child_index: usize,
        types: u8,
        mask: u8,
    ) -> BlockId {
        let block_index = block_id.index();

        let new_block_id = BlockId::new_branch(block_index, block_id.generation(), types, mask);

        debug_assert_ne!(new_block_id, BlockId::INVALID, "Invalid block id");

        self.children.get_mut(block_index)[child_index] = *child_id;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "update_branch: block_id: {block_id:?} child_id: {child_id:?} child_index: {child_index} new_id: {new_block_id:?} types: {types:2X} mask: {mask:2X}"
        );

        new_block_id
    }

    pub fn preallocate_branch_id(&mut self, index: u32, types: u8, mask: u8) -> BlockId {
        let next_id = get_next_index_macro!(self);
        assert_eq!(next_id, index, "Invalid block id");

        let generation = *self.generations.get(index);
        assert_eq!(generation, 0, "Invalid generation");

        BlockId::new_branch(index, generation, types, mask)
    }

    pub fn deserialize_leaf(&mut self, index: u32, value: T) -> BlockId {
        debug_assert_ne!(value, T::default(), "Leaf value should not be default");

        let fingerprint = compute_leaf_hash_for_value(&value);
        let pattern_key = self.vacant_pattern_key(PATTERNS_TYPE_LEAF, fingerprint);

        let next_id = get_next_index_macro!(self);
        assert_eq!(next_id, index, "Invalid block id");

        let generation = *self.generations.get(index);
        assert_eq!(generation, 0, "Invalid generation");

        let block_id = BlockId::new_leaf(index, generation);

        *self.values.get_mut(index) = value;
        *self.hashes.get_mut(index) = pattern_key;

        self.patterns[PATTERNS_TYPE_LEAF].insert(pattern_key, block_id);
        if pattern_key != fingerprint {
            self.has_pattern_collisions[PATTERNS_TYPE_LEAF] = true;
        }

        block_id
    }

    pub fn deserialize_branch(
        &mut self,
        block_id: BlockId,
        children: Children,
        types: u8,
        mask: u8,
        average: T,
    ) {
        let fingerprint = compute_branch_hash_for_children(&children, types, mask);
        let pattern_key = self.vacant_pattern_key(PATTERNS_TYPE_BRANCH, fingerprint);

        let index = block_id.index();

        *self.children.get_mut(index) = children;
        *self.values.get_mut(index) = average;
        *self.hashes.get_mut(index) = pattern_key;

        self.patterns[PATTERNS_TYPE_BRANCH].insert(pattern_key, block_id);
        if pattern_key != fingerprint {
            self.has_pattern_collisions[PATTERNS_TYPE_BRANCH] = true;
        }

        self.inc_all_child_refs(&children);
    }

    #[inline]
    fn vacant_pattern_key(&self, pattern_type: usize, fingerprint: u64) -> u64 {
        let mut pattern_key = fingerprint;
        while self.patterns[pattern_type].contains_key(&pattern_key) {
            pattern_key = next_pattern_key(pattern_key);
        }
        pattern_key
    }

    /// Removes one logical open-addressed entry and closes the following probe
    /// cluster. Closing the cluster is necessary because later colliding keys
    /// must remain discoverable from their original fingerprints.
    #[inline]
    fn remove_pattern(&mut self, block_id: BlockId) {
        let pattern_type = block_id.is_leaf() as usize;
        let removed_key = *self.hashes.get(block_id.index());
        let removed = self.patterns[pattern_type].remove(&removed_key);
        debug_assert_eq!(removed, Some(block_id));

        if !self.has_pattern_collisions[pattern_type] {
            return;
        }

        let mut displaced = Vec::new();
        let mut pattern_key = next_pattern_key(removed_key);
        while let Some(displaced_id) = self.patterns[pattern_type].remove(&pattern_key) {
            displaced.push(displaced_id);
            pattern_key = next_pattern_key(pattern_key);
        }

        for displaced_id in displaced {
            let fingerprint = if displaced_id.is_leaf() {
                compute_leaf_hash_for_value(self.values.get(displaced_id.index()))
            } else {
                compute_branch_hash_for_children(
                    self.children.get(displaced_id.index()),
                    displaced_id.types(),
                    displaced_id.mask(),
                )
            };
            let pattern_key = self.vacant_pattern_key(pattern_type, fingerprint);
            self.patterns[pattern_type].insert(pattern_key, displaced_id);
            *self.hashes.get_mut(displaced_id.index()) = pattern_key;
        }
    }

    #[inline(always)]
    #[cfg(debug_assertions)]
    pub fn is_valid_block_id(&self, block_id: &BlockId) -> bool {
        *self.generations.get(block_id.index()) == block_id.generation()
            && !self.free_indices.contains(&block_id.index())
    }

    #[inline(always)]
    #[cfg(not(debug_assertions))]
    pub fn is_valid_block_id(&self, block_id: &BlockId) -> bool {
        // Generation comparison is constant-time and rejects recycled IDs.
        // Debug builds additionally scan the free list to expose bookkeeping
        // errors before a slot is reallocated.
        *self.generations.get(block_id.index()) == block_id.generation()
    }

    pub fn ensure_valid_children(&self, children: &Children) {
        for child_id in children.iter() {
            if !child_id.is_empty() {
                assert!(
                    self.is_valid_block_id(child_id),
                    "Invalid child id: {child_id:?}"
                );
            }
        }
    }

    #[inline]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    pub fn patterns_empty(&self) -> bool {
        self.patterns[PATTERNS_TYPE_BRANCH].len() == 1
            && self.patterns[PATTERNS_TYPE_LEAF].is_empty()
    }

    pub fn leaf_patterns(&self) -> &PatternsHashmap {
        &self.patterns[PATTERNS_TYPE_LEAF]
    }

    pub fn branch_patterns(&self) -> &PatternsHashmap {
        &self.patterns[PATTERNS_TYPE_BRANCH]
    }

    #[cfg(feature = "memory_stats")]
    pub fn stats(&self) -> InternerStats {
        self.stats
    }

    pub fn dump_patterns(&self) {
        println!("=== Leaf Patterns ===");
        for (hash, id) in self.patterns[PATTERNS_TYPE_LEAF].iter() {
            println!("{hash:X} -> {id:?}");
            self.dump_node(*id, 0, "  ");
        }
        println!("=== End of Patterns ===\n");

        println!("=== Branch Patterns ===");
        for (hash, id) in self.patterns[PATTERNS_TYPE_BRANCH].iter() {
            if *hash == self.empty_branch_hash {
                println!("{hash:X} -> EMPTY");
                continue;
            } else {
                println!("{hash:X} -> {id:?}");
                self.dump_node(*id, 0, "  ");
            }
        }
        println!("=== End of Patterns ===\n");
    }

    pub fn dump_node(&self, node_id: BlockId, depth: u8, prefix: &str) {
        let discovered_nodes = self.dump_node_internal(node_id, depth, prefix);
        println!("{prefix}Discovered nodes: {discovered_nodes}");
    }

    pub fn count_nodes(&self, node_id: BlockId) -> u32 {
        self.count_nodes_internal(node_id)
    }

    fn count_nodes_internal(&self, node_id: BlockId) -> u32 {
        if !self.is_valid_block_id(&node_id) {
            return 0;
        }

        let mut discovered_nodes = 1;

        if !node_id.is_leaf() {
            for child_id in self.get_children_ref(&node_id).iter() {
                if !child_id.is_empty() {
                    discovered_nodes += self.count_nodes_internal(*child_id);
                }
            }
        }

        discovered_nodes
    }

    pub fn dump_node_internal(&self, node_id: BlockId, depth: u8, prefix: &str) -> u32 {
        let current_prefix = prefix.repeat((depth + 1) as usize).to_string();

        if !self.is_valid_block_id(&node_id) {
            println!("{current_prefix}Invalid block id: {node_id:?}");
            return 0;
        }

        if depth > MAX_ALLOWED_DEPTH as u8 {
            panic!("{current_prefix}Max depth reached");
        }

        let mut discovered_nodes = 1;

        let is_leaf = node_id.is_leaf();
        let node_hash = self.hashes.get(node_id.index());
        let node_ref_count = self.get_ref(&node_id);

        if depth == 0 {
            if !is_leaf {
                println!(
                    "{current_prefix}Branch[{}, {}] hash: {node_hash:X}, ref_count: {node_ref_count}",
                    node_id.index(),
                    node_id.generation(),
                );
            } else {
                let value = self.get_value(&node_id);
                println!(
                    "{current_prefix}Leaf[{}, {}] value: {value}, hash: {node_hash:X}, ref_count: {node_ref_count}",
                    node_id.index(),
                    node_id.generation(),
                );
                return discovered_nodes;
            }
        }

        let children = self.get_children(&node_id);

        let new_prefix = format!("{current_prefix}{prefix}");

        for (idx, child_id) in children.iter().enumerate() {
            match child_id {
                &BlockId::EMPTY => {
                    println!("{new_prefix}[{idx}]: -");
                }
                _ => {
                    if !self.is_valid_block_id(child_id) {
                        println!("{new_prefix}[{idx}]: Invalid id: {child_id:?}");
                    } else if child_id.is_leaf() {
                        let value = self.get_value(child_id);
                        let node_hash = self.hashes.get(child_id.index());
                        let node_ref_count = self.get_ref(child_id);
                        println!(
                            "{new_prefix}[{idx}]: Leaf[{}, {}] value: {value}, hash: {node_hash:X}, ref_count: {node_ref_count}",
                            child_id.index(),
                            child_id.generation(),
                        );
                        discovered_nodes += 1;
                    } else {
                        let node_hash = self.hashes.get(child_id.index());
                        let node_ref_count = self.get_ref(child_id);
                        println!(
                            "{new_prefix}[{idx}]: Branch[{}, {}] hash: {node_hash:X}, ref_count: {node_ref_count}",
                            child_id.index(),
                            child_id.generation(),
                        );
                        discovered_nodes += self.dump_node_internal(*child_id, depth + 1, prefix);
                    }
                }
            }
        }

        discovered_nodes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn colliding_leaf_fingerprints_preserve_identity_across_removal() {
        let mut interner = VoxInterner::<i32>::with_memory_budget(4096);
        let fingerprint = 0x1234_5678_9ABC_DEF0;

        let first = interner.get_or_create_leaf_with_fingerprint(11, fingerprint);
        let second = interner.get_or_create_leaf_with_fingerprint(22, fingerprint);
        assert_ne!(first, second);
        assert_eq!(*interner.get_value(&first), 11);
        assert_eq!(*interner.get_value(&second), 22);
        assert_eq!(
            interner.get_or_create_leaf_with_fingerprint(11, fingerprint),
            first
        );

        assert!(!interner.dec_ref(&first));
        assert!(interner.dec_ref(&first));
        assert_eq!(interner.get_or_create_leaf(22), second);
        assert_eq!(interner.leaf_patterns().len(), 1);
    }

    #[test]
    fn branch_fingerprint_collision_with_empty_and_peer_remains_exact() {
        let mut interner = VoxInterner::<i32>::with_memory_budget(4096);
        let first_leaf = interner.get_or_create_leaf(1);
        let second_leaf = interner.get_or_create_leaf(2);
        let mut first_children = EMPTY_CHILD;
        first_children[0] = first_leaf;
        let mut second_children = EMPTY_CHILD;
        second_children[0] = second_leaf;
        let fingerprint = interner.empty_branch_hash;

        let first = interner.get_or_create_branch_with_fingerprint(
            first_children,
            0b0000_0001,
            0b0000_0001,
            fingerprint,
        );
        let second = interner.get_or_create_branch_with_fingerprint(
            second_children,
            0b0000_0001,
            0b0000_0001,
            fingerprint,
        );
        assert_ne!(first, BlockId::EMPTY);
        assert_ne!(second, BlockId::EMPTY);
        assert_ne!(first, second);

        interner.inc_ref(&first_leaf);
        assert_eq!(
            interner.get_or_create_branch_with_fingerprint(
                first_children,
                0b0000_0001,
                0b0000_0001,
                fingerprint,
            ),
            first
        );
        assert!(!interner.dec_ref(&first));
        assert!(interner.dec_ref(&first));

        interner.inc_ref(&second_leaf);
        assert_eq!(
            interner.get_or_create_branch(second_children, 0b0000_0001, 0b0000_0001),
            second
        );
        assert_eq!(interner.branch_patterns().len(), 2);
    }
}
