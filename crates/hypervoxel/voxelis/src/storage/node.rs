use std::collections::{HashMap, hash_map::Entry};

use crate::{BlockId, VoxelTrait, get_next_index_macro};

use super::PoolAllocatorLite;

mod consts;
mod hash;
mod macros;
#[cfg(feature = "memory_stats")]
mod stats;

pub use consts::*;
pub use hash::PatternsHashmap;
#[cfg(feature = "memory_stats")]
pub use stats::StoreStats;

use hash::{
    IdentityHasherBuilder, compute_branch_hash_for_children, compute_empty_branch_hash,
    compute_leaf_hash_for_value,
};

pub type Children = [BlockId; MAX_CHILDREN];

pub struct NodeStore<T> {
    patterns: [PatternsHashmap; 2],
    free_indices: Vec<u32>,
    next_index: u32,
    ref_counts: PoolAllocatorLite<u32>,
    generations: PoolAllocatorLite<u16>,
    children: PoolAllocatorLite<Children>,
    values: PoolAllocatorLite<T>,
    hashes: PoolAllocatorLite<u64>,
    capacity: usize,
    empty_branch_id: BlockId,
    empty_branch_hash: u64,
    dec_ref_rec_stack: Vec<BlockId>,
    #[cfg(feature = "memory_stats")]
    stats: StoreStats,
}

impl<T: VoxelTrait> NodeStore<T> {
    const INITIAL_CAPACITY: usize = 16384; // 43ms

    pub fn with_memory_budget(requested_budget: usize) -> Self {
        let single_node_size = Self::node_size();

        // Calculate how many complete nodes fit in the budget
        let nodes_capacity = requested_budget / single_node_size;
        let actual_budget = nodes_capacity * single_node_size;

        // println!(
        //     "Requested budget: {}, actual budget: {}, single node size: {}, capacity: {}",
        //     humanize_bytes::humanize_bytes_decimal!(requested_budget),
        //     humanize_bytes::humanize_bytes_decimal!(actual_budget),
        //     humanize_bytes::humanize_bytes_decimal!(single_node_size),
        //     humanize_bytes::humanize_quantity!(nodes_capacity),
        // );

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

        // *types.get_mut(empty_branch_index) = 0;
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
            println!("empty_branch_hash: {:X}", empty_branch_hash);
            println!("empty_branch_id: {:?}", empty_branch_id);
            println!("next_index: {}", next_index);
        }

        #[cfg(feature = "memory_stats")]
        let stats = StoreStats {
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
            "Invalid block id: {:?}",
            block_id
        );

        self.values.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_children(&self, block_id: &BlockId) -> Children {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id
        );

        *self.children.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_children_ref(&self, block_id: &BlockId) -> &Children {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id
        );

        self.children.get(block_id.index())
    }

    #[inline(always)]
    pub fn get_child_id(&self, block_id: &BlockId, index: usize) -> BlockId {
        debug_assert!(block_id.is_branch(), "Cannot get children for value node",);
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id
        );

        self.children.get(block_id.index())[index]
    }

    #[inline(always)]
    pub fn get_ref(&self, block_id: &BlockId) -> u32 {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id
        );

        *self.ref_counts.get(block_id.index())
    }

    #[inline(always)]
    pub fn inc_ref(&mut self, block_id: &BlockId) {
        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id
        );

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
            "Invalid block id: {:?}",
            block_id
        );

        let block_index = block_id.index();

        let ref_count = self.ref_counts.get_mut(block_index);

        debug_assert!(
            *ref_count > 0,
            "Ref count should be greater than zero, id: {:?}",
            block_id,
        );

        *ref_count -= 1;

        if *ref_count == 0 {
            self.patterns[block_id.is_leaf() as usize].remove(self.hashes.get(block_index));

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
        println!("Incrementing ref count for children: {:?}", children);

        for (i, child_id) in children.iter().enumerate() {
            if i == index {
                #[cfg(feature = "debug_trace_ref_counts")]
                println!("  [{}] Skipping child_id: {:?}", i, child_id);

                continue;
            }

            if !child_id.is_empty() {
                #[cfg(feature = "debug_trace_ref_counts")]
                let current_ref_count = self.get_ref(child_id);

                self.inc_ref(child_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "  [{}] Incrementing ref count for child_id: {:?} ref_count: {} -> {}",
                    i,
                    child_id,
                    current_ref_count,
                    self.get_ref(child_id),
                );
            }
        }
    }

    pub fn inc_all_child_refs(&mut self, children: &Children) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("Incrementing ref count for children: {:?}", children);

        #[cfg(feature = "debug_trace_ref_counts")]
        let mut i = 0;

        for child_id in children.iter() {
            if !child_id.is_empty() {
                #[cfg(feature = "debug_trace_ref_counts")]
                let current_ref_count = self.get_ref(child_id);

                self.inc_ref(child_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "  [{}] Incrementing ref count for child_id: {:?} ref_count: {} -> {}",
                    i,
                    child_id,
                    current_ref_count,
                    self.get_ref(child_id),
                );
            }

            #[cfg(feature = "debug_trace_ref_counts")]
            {
                i += 1;
            }
        }
    }

    pub fn inc_ref_by(&mut self, block_id: &BlockId, count: u32) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "Incrementing ref count for block: {:?} by {}",
            block_id, count
        );

        debug_assert!(
            self.is_valid_block_id(block_id),
            "Invalid block id: {:?}",
            block_id,
        );

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
        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "Decrementing ref count for block: {:?} by {}",
            block_id, count
        );

        let block_index = block_id.index();

        let ref_count = self.ref_counts.get_mut(block_index);

        debug_assert!(
            ((*ref_count as i64) + count as i64) >= 0,
            "Ref count should be greater or equal than zero, id: {:?}",
            block_id,
        );

        *ref_count -= count;

        if *ref_count == 0 {
            self.patterns[block_id.is_leaf() as usize].remove(self.hashes.get(block_index));

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
            "Invalid block id: {:?}",
            block_id
        );

        #[cfg(debug_assertions)]
        let max_idx = self.dec_ref_rec_stack.capacity();

        let stack_ptr = self.dec_ref_rec_stack.as_mut_ptr();

        unsafe {
            *stack_ptr.add(0) = *block_id;
        }

        let mut read_idx = 0;
        let mut write_idx = 1;

        // #[cfg(feature = "debug_trace_ref_counts")]
        // let mut total_processed = 0;

        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("dec_ref_recursive block_id: {:?}", block_id);
            self.dump_node(*block_id, 0, "  ");
        }

        while read_idx < write_idx {
            let current_id = unsafe { *stack_ptr.add(read_idx) };
            read_idx += 1;

            #[cfg(debug_assertions)]
            assert!(
                read_idx < max_idx,
                "dec_ref_rec_stack overflow: {}",
                read_idx
            );

            #[cfg(debug_assertions)]
            assert!(self.is_valid_block_id(&current_id));

            #[cfg(feature = "debug_trace_ref_counts")]
            {
                println!(
                    " {}/{}/{} Processing: {:?}",
                    read_idx - 1,
                    write_idx,
                    max_idx,
                    current_id
                );
                self.dump_node(current_id, 0, "  ");
            }

            let current_index = current_id.index();
            let ref_count = self.ref_counts.get_mut(current_index);

            #[cfg(feature = "debug_trace_ref_counts")]
            let current_ref_count = *ref_count;

            debug_assert!(
                *ref_count > 0,
                "Ref count should be greater than zero, id: {:?}",
                current_id,
            );

            *ref_count -= 1;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!("    ref_count = {} -> {}", current_ref_count, ref_count);

            if *ref_count == 0 {
                for child in self.children.get(current_index) {
                    if !child.is_empty() {
                        #[cfg(debug_assertions)]
                        assert!(self.is_valid_block_id(child));

                        let child_ref_count = self.ref_counts.get_mut(child.index());
                        if *child_ref_count > 1 {
                            #[cfg(feature = "debug_trace_ref_counts")]
                            println!(
                                "      handling child: {:?} at {} ref_count: {} -> {} in place",
                                child,
                                write_idx,
                                *child_ref_count,
                                *child_ref_count - 1,
                            );
                            *child_ref_count -= 1;
                        } else {
                            #[cfg(feature = "debug_trace_ref_counts")]
                            println!(
                                "      adding child:   {:?} at {} ref_count: {}",
                                child, write_idx, child_ref_count
                            );
                            unsafe {
                                *stack_ptr.add(write_idx) = *child;
                            }
                            write_idx += 1;

                            #[cfg(debug_assertions)]
                            assert!(
                                write_idx < max_idx,
                                "dec_ref_rec_stack overflow: {}, capacity: {}",
                                write_idx,
                                max_idx
                            );
                        }
                    }
                }

                self.patterns[current_id.is_leaf() as usize].remove(self.hashes.get(current_index));

                #[cfg(feature = "memory_stats")]
                {
                    self.stats.patterns -= 1;
                }

                self.recycle(&current_id);
            }
        }

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            " ...done read_idx: {}, write_idx: {} capacity: {}",
            read_idx,
            write_idx,
            self.dec_ref_rec_stack.capacity()
        );
    }

    pub fn dec_child_refs(&mut self, children: &Children) {
        #[cfg(feature = "debug_trace_ref_counts")]
        {
            println!("Decrementing ref count for children: {:?}", children);

            for (i, child_id) in children.iter().enumerate() {
                if !child_id.is_empty() {
                    let current_ref_count = self.get_ref(child_id);

                    let _recycled = self.dec_ref(child_id);

                    println!(
                        "  [{}] Decrementing ref count for child_id: {:?} ref_count: {} -> {}",
                        i,
                        child_id,
                        current_ref_count,
                        if _recycled { 0 } else { self.get_ref(child_id) },
                    );
                }
            }
        }

        #[cfg(not(feature = "debug_trace_ref_counts"))]
        for child_id in children.iter() {
            if !child_id.is_empty() {
                self.dec_ref(child_id);
            }
        }
    }

    pub fn recycle(&mut self, block_id: &BlockId) {
        #[cfg(feature = "debug_trace_ref_counts")]
        println!("recycle block_id: {:?}", block_id);

        debug_assert!(
            block_id != &self.empty_branch_id,
            "Cannot recycle empty branch",
        );

        let block_index = block_id.index();

        // Clear node data
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

        // Mark index as free
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

    pub fn get_or_create_leaf(&mut self, value: T) -> BlockId {
        debug_assert_ne!(value, T::default(), "Leaf value should not be default");

        // Compute hash for the new node
        let hash = compute_leaf_hash_for_value(&value);

        match self.patterns[PATTERNS_TYPE_LEAF].entry(hash) {
            Entry::Occupied(entry) => {
                let existing_id = *entry.get();
                // Verify the node is still valid
                if self.is_valid_block_id(&existing_id) {
                    self.inc_ref(&existing_id);

                    #[cfg(feature = "debug_trace_ref_counts")]
                    println!(
                        "get_or_create_leaf: value: {} hash = {:X} existing_id = {:?} ref_count: {}",
                        value,
                        hash,
                        existing_id,
                        self.get_ref(&existing_id)
                    );

                    debug_assert_eq!(
                        self.values.get(existing_id.index()),
                        &value,
                        "Value mismatch for existing leaf node"
                    );

                    #[cfg(feature = "memory_stats")]
                    {
                        self.stats.total_cache_hits += 1;
                        self.stats.leaf_cache_hits += 1;
                    }

                    existing_id
                } else {
                    #[cfg(feature = "debug_trace_ref_counts")]
                    self.dump_patterns();

                    panic!(
                        "Expired node in patterns: {:?} hash = {:X}",
                        existing_id, hash
                    );
                }
            }
            Entry::Vacant(entry) => {
                // Get new index
                let index = get_next_index_macro!(self);

                let generation = *self.generations.get(index);

                // Create new block id
                let block_id = BlockId::new_leaf(index, generation);

                // Cache the new node
                entry.insert(block_id);

                // Set up the new leaf node
                *self.values.get_mut(index) = value;
                *self.hashes.get_mut(index) = hash;

                debug_assert_eq!(
                    self.get_ref(&block_id),
                    0,
                    "New node should have zero ref count"
                );

                self.inc_ref(&block_id);

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "get_or_create_leaf: value: {} hash = {:X} new_id = {:?} ref_count: {}",
                    value,
                    hash,
                    block_id,
                    self.get_ref(&block_id)
                );

                #[cfg(feature = "memory_stats")]
                {
                    self.stats.leaf_nodes += 1;
                    self.stats.patterns += 1;
                    self.stats.total_cache_misses += 1;
                    self.stats.leaf_cache_misses += 1;
                }

                block_id
            }
        }
    }

    /// All non-empty blocks inside `children` must have bumped ref counts:
    /// - if there is no branch, ref counts will be kept
    /// - if there is a branch, ref counts will be decremented, and branch ref count will be bumped
    /// There is no other way, since we can't act like Arc without access to store
    pub fn get_or_create_branch(&mut self, children: Children, types: u8, mask: u8) -> BlockId {
        // Compute hash for the new node
        let hash = compute_branch_hash_for_children(&children, types, mask);

        debug_assert_ne!(
            hash, self.empty_branch_hash,
            "Empty branch hash collision: {:?}",
            children
        );

        match self.patterns[PATTERNS_TYPE_BRANCH].entry(hash) {
            Entry::Occupied(entry) => {
                let existing_id = *entry.get();

                debug_assert_ne!(
                    existing_id,
                    BlockId::EMPTY,
                    "Empty branch id in patterns: {:?}",
                    children
                );

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "get_or_create_branch: children: {:?} hash = {:X} existing_id = {:?}",
                    children, hash, existing_id
                );

                // Verify the node is still valid
                debug_assert!(
                    self.is_valid_block_id(&existing_id),
                    "Expired node in patterns: {:?} hash = {:X}",
                    existing_id,
                    hash
                );

                debug_assert_eq!(
                    existing_id.types(),
                    types,
                    "Types mismatch for existing branch node"
                );

                debug_assert_eq!(
                    existing_id.mask(),
                    mask,
                    "Mask mismatch for existing branch node"
                );

                debug_assert_eq!(
                    self.children.get(existing_id.index()),
                    &children,
                    "Children mismatch for existing branch node"
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

                existing_id
            }
            Entry::Vacant(entry) => {
                // Get new index
                let index = get_next_index_macro!(self);

                let generation = *self.generations.get(index);

                // Create new block id
                let block_id = BlockId::new_branch(index, generation, types, mask);

                debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

                // Cache the new node
                entry.insert(block_id);

                // Compute average value for the children - free LODs
                let values: [T; 8] = std::array::from_fn(|i| *self.values.get(children[i].index()));
                let average = T::average(&values);

                // Set up the new branch node
                *self.children.get_mut(index) = children;
                *self.values.get_mut(index) = average;
                *self.hashes.get_mut(index) = hash;

                #[cfg(feature = "debug_trace_ref_counts")]
                println!(
                    "get_or_create_branch: children: {:?} hash = {:X} new_id: {:?} types: {:2X} mask: {:2X}",
                    children, hash, block_id, types, mask
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

                block_id
            }
        }
    }

    #[cfg(feature = "memory_stats")]
    pub fn bump_collapsed_branches(&mut self) {
        self.stats.collapsed_branches += 1;
    }

    pub fn create_empty_branch(&mut self) -> BlockId {
        // Get new index
        let index = get_next_index_macro!(self);

        let generation = *self.generations.get(index);

        // Create new block id
        let block_id = BlockId::new_branch(index, generation, 0, 0);

        debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

        // Set up the new branch node
        *self.children.get_mut(index) = EMPTY_CHILD;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!("create_empty_branch: new_id: {:?}", block_id);

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
        // Get new index
        let index = get_next_index_macro!(self);

        let generation = *self.generations.get(index);

        // Create new block id
        let block_id = BlockId::new_branch(index, generation, types, mask);

        debug_assert_ne!(block_id, BlockId::INVALID, "Invalid block id");

        // Set up the new branch node
        *self.children.get_mut(index) = children;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "create_branch: children: {:?} new_id: {:?} types: {:2X} mask: {:2X}",
            children, block_id, types, mask
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

        // Create new block id
        let new_block_id = BlockId::new_branch(block_index, block_id.generation(), types, mask);

        debug_assert_ne!(new_block_id, BlockId::INVALID, "Invalid block id");

        // Set up the new branch node
        self.children.get_mut(block_index)[child_index] = *child_id;

        #[cfg(feature = "debug_trace_ref_counts")]
        println!(
            "update_branch: block_id: {:?} child_id: {:?} child_index: {} new_id: {:?} types: {:2X} mask: {:2X}",
            block_id, child_id, child_index, new_block_id, types, mask
        );

        new_block_id
    }

    pub fn preallocate_branch_id(&mut self, index: u32, types: u8, mask: u8) -> BlockId {
        let next_id = get_next_index_macro!(self);
        assert_eq!(next_id, index, "Invalid block id");

        let generation = *self.generations.get(index);
        assert_eq!(generation, 0, "Invalid generation");

        // Create new block id
        BlockId::new_branch(index, generation, types, mask)
    }

    pub fn deserialize_leaf(&mut self, index: u32, value: T) -> BlockId {
        debug_assert_ne!(value, T::default(), "Leaf value should not be default");

        // Compute hash for the new node
        let hash = compute_leaf_hash_for_value(&value);

        let next_id = get_next_index_macro!(self);
        assert_eq!(next_id, index, "Invalid block id");

        let generation = *self.generations.get(index);
        assert_eq!(generation, 0, "Invalid generation");

        // Create new block id
        let block_id = BlockId::new_leaf(index, generation);

        // Set up the new leaf node
        *self.values.get_mut(index) = value;
        *self.hashes.get_mut(index) = hash;

        // Cache the new node
        self.patterns[PATTERNS_TYPE_LEAF].insert(hash, block_id);

        block_id
    }

    pub fn deserialize_branch(
        &mut self,
        block_id: BlockId,
        children: Children,
        types: u8,
        mask: u8,
    ) {
        // Compute hash for the new node
        let hash = compute_branch_hash_for_children(&children, types, mask);

        let index = block_id.index();

        debug_assert_ne!(
            hash, self.empty_branch_hash,
            "Empty branch hash collision: {:?}",
            children
        );

        // Set up the new branch node
        *self.children.get_mut(index) = children;
        *self.hashes.get_mut(index) = hash;

        // Cache the new node
        self.patterns[PATTERNS_TYPE_BRANCH].insert(hash, block_id);

        self.inc_all_child_refs(&children);
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
        *self.generations.get(block_id.index()) == block_id.generation()
        // TODO(aljen): Disable for final?
        // true
    }

    pub fn ensure_valid_children(&self, children: &Children) {
        for child_id in children.iter() {
            if !child_id.is_empty() {
                assert!(
                    self.is_valid_block_id(child_id),
                    "Invalid child id: {:?}",
                    child_id
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
    pub fn stats(&self) -> StoreStats {
        self.stats
    }

    pub fn dump_patterns(&self) {
        println!("=== Leaf Patterns ===");
        for (hash, id) in self.patterns[PATTERNS_TYPE_LEAF].iter() {
            println!("{:X} -> {:?}", hash, id);
            self.dump_node(*id, 0, "  ");
        }
        println!("=== End of Patterns ===\n");

        println!("=== Branch Patterns ===");
        for (hash, id) in self.patterns[PATTERNS_TYPE_BRANCH].iter() {
            if *hash == self.empty_branch_hash {
                println!("{:X} -> EMPTY", hash);
                continue;
            } else {
                println!("{:X} -> {:?}", hash, id);
                self.dump_node(*id, 0, "  ");
            }
        }
        println!("=== End of Patterns ===\n");
    }

    pub fn dump_node(&self, node_id: BlockId, depth: u8, prefix: &str) {
        let discovered_nodes = self.dump_node_internal(node_id, depth, prefix);
        println!("{}Discovered nodes: {}", prefix, discovered_nodes);
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
            println!("{}Invalid block id: {:?}", current_prefix, node_id);
            return 0;
        }

        if depth > MAX_ALLOWED_DEPTH as u8 {
            panic!("{}Max depth reached", current_prefix);
        }

        let mut discovered_nodes = 1;

        let is_leaf = node_id.is_leaf();
        let node_hash = self.hashes.get(node_id.index());
        let node_ref_count = self.get_ref(&node_id);

        if depth == 0 {
            if !is_leaf {
                println!(
                    "{}Branch[{}, {}] hash: {:X}, ref_count: {}",
                    current_prefix,
                    node_id.index(),
                    node_id.generation(),
                    node_hash,
                    node_ref_count,
                );
            } else {
                let value = self.get_value(&node_id);
                println!(
                    "{}Leaf[{}, {}] value: {}, hash: {:X}, ref_count: {}",
                    current_prefix,
                    node_id.index(),
                    node_id.generation(),
                    value,
                    node_hash,
                    node_ref_count,
                );
                return discovered_nodes;
            }
        }

        let children = self.get_children(&node_id);

        let new_prefix = format!("{}{}", current_prefix, prefix);

        for (idx, child_id) in children.iter().enumerate() {
            match child_id {
                &BlockId::EMPTY => {
                    println!("{}[{}]: -", new_prefix, idx);
                }
                _ => {
                    if !self.is_valid_block_id(child_id) {
                        println!("{}[{}]: Invalid id: {:?}", new_prefix, idx, child_id);
                    } else if child_id.is_leaf() {
                        let value = self.get_value(child_id);
                        let node_hash = self.hashes.get(child_id.index());
                        let node_ref_count = self.get_ref(child_id);
                        println!(
                            "{}[{}]: Leaf[{}, {}] value: {}, hash: {:X}, ref_count: {}",
                            new_prefix,
                            idx,
                            child_id.index(),
                            child_id.generation(),
                            value,
                            node_hash,
                            node_ref_count,
                        );
                        discovered_nodes += 1;
                    } else {
                        let node_hash = self.hashes.get(child_id.index());
                        let node_ref_count = self.get_ref(child_id);
                        println!(
                            "{}[{}]: Branch[{}, {}] hash: {:X}, ref_count: {}",
                            new_prefix,
                            idx,
                            child_id.index(),
                            child_id.generation(),
                            node_hash,
                            node_ref_count,
                        );
                        discovered_nodes += self.dump_node_internal(*child_id, depth + 1, prefix);
                    }
                }
            }
        }

        discovered_nodes
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn test_basic() {
//         let mut storage = NodeStorage::<u32>::new(1024);

//         let id1 = storage.allocate_node();
//         let id2 = storage.allocate_node();

//         assert_ne!(id1, id2);

//         *storage.values.get_mut(id1.index()) = 42;
//         *storage.values.get_mut(id2.index()) = 24;

//         assert_eq!(*storage.values.get(id1.index()), 42);
//         assert_eq!(*storage.values.get(id2.index()), 24);

//         storage.recycle(id1);
//         let id3 = storage.allocate_node();
//         assert_eq!(id3.index(), id1.index());
//         assert_eq!(id3.generation(), id1.generation() + 1);
//     }

//     #[test]
//     fn test_capacity() {
//         let requested_budget = 1024;
//         let storage = NodeStorage::<u32>::new(requested_budget);
//         let node_size = NodeStorage::<u32>::node_size();
//         let nodes_count = requested_budget / node_size;
//         assert!(storage.capacity() > 0);
//         assert!(storage.capacity() < 1024);
//         assert_eq!(storage.capacity(), nodes_count);
//     }

//     #[test]
//     #[should_panic(expected = "Requested budget is too small")]
//     fn test_budget_too_small() {
//         let _storage = NodeStorage::<u32>::new(1); // Should panic
//     }

//     #[test]
//     fn test_generation_flow() {
//         let mut storage = NodeStorage::<u32>::new(1024);

//         // Test normal generation increment
//         let id1 = storage.allocate_node();
//         let gen1 = id1.generation();
//         storage.recycle(id1);

//         let id2 = storage.allocate_node();
//         assert_eq!(id2.index(), id1.index());
//         assert_eq!(id2.generation(), gen1 + 1);

//         // Test generation overflow
//         storage.recycle(id2);

//         *storage.generations.get_mut(id2.index()) = u32::MAX;

//         // Allocating new node, should have the same index but generation 0
//         let id3 = storage.allocate_node();
//         assert_eq!(id3.index(), id2.index());
//         assert_eq!(id3.generation(), u32::MAX);

//         // Deallocating and allocating again - now we should have overflow to 0
//         storage.recycle(id3);
//         let id4 = storage.allocate_node();
//         assert_eq!(id4.index(), id3.index());
//         assert_eq!(id4.generation(), 0);
//     }

//     #[test]
//     fn test_reuse_pattern() {
//         let mut storage = NodeStorage::<u32>::new(1024);
//         let mut ids = Vec::new();

//         // Allocate few nodes
//         for _ in 0..3 {
//             ids.push(storage.allocate_node());
//         }

//         // Deallocate in specific order
//         storage.recycle(ids[1]); // Middle
//         storage.recycle(ids[0]); // First

//         // New allocations should reuse in LIFO order
//         let new_id1 = storage.allocate_node();
//         let new_id2 = storage.allocate_node();

//         assert_eq!(new_id1.index(), ids[0].index());
//         assert_eq!(new_id2.index(), ids[1].index());
//     }

//     #[test]
//     #[should_panic(expected = "Double free detected")]
//     fn test_double_free() {
//         let mut storage = NodeStorage::<u32>::new(1024);
//         let id = storage.allocate_node();
//         storage.recycle(id);
//         storage.recycle(id); // Should panic
//     }

//     #[test]
//     fn test_node_hash_computation() {
//         let mut storage = NodeStorage::<u32>::new(1024);

//         // Test leaf node hash
//         let leaf1 = storage.get_or_create(42, true);
//         let leaf2 = storage.get_or_create(42, true);
//         assert_eq!(storage.get_hash(leaf1.index()), storage.get_hash(leaf2.index()),
//             "Same leaf value should have same hash");

//         let leaf3 = storage.get_or_create(43, true);
//         assert_ne!(storage.get_hash(leaf1.index()), storage.get_hash(leaf3.index()),
//             "Different leaf values should have different hashes");

//         // Test branch node hash
//         let mut children1 = [None; MAX_CHILDREN];
//         children1[0] = Some(leaf1);
//         children1[1] = Some(leaf2);

//         let mut children2 = [None; MAX_CHILDREN];
//         children2[0] = Some(leaf1);
//         children2[1] = Some(leaf2);

//         let branch1 = storage.get_or_create(Default::default(), false);
//         storage.set_children(branch1.index(), children1);
//         let branch2 = storage.get_or_create(Default::default(), false);
//         storage.set_children(branch2.index(), children2);

//         assert_eq!(storage.get_hash(branch1.index()), storage.get_hash(branch2.index()),
//             "Same branch structure should have same hash");

//         let mut children3 = children1;
//         children3[1] = Some(leaf3);
//         let branch3 = storage.get_or_create(Default::default(), false);
//         storage.set_children(branch3.index(), children3);

//         assert_ne!(storage.get_hash(branch1.index()), storage.get_hash(branch3.index()),
//             "Different branch structures should have different hashes");
//     }
// }
