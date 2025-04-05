use std::{
    collections::HashMap,
    hash::{BuildHasher, Hash, Hasher},
};

use rustc_hash::FxHasher;

use crate::{BlockId, VoxelTrait};

use super::{
    Children,
    consts::{CHILD_ABSENT, EMPTY_CHILD, NODE_TYPE_BRANCH, NODE_TYPE_LEAF},
};

pub struct IdentityHasher(u64);

impl Hasher for IdentityHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        self.0 = u64::from_ne_bytes(bytes.try_into().unwrap());
    }
}

#[derive(Default)]
pub struct IdentityHasherBuilder;

impl BuildHasher for IdentityHasherBuilder {
    type Hasher = IdentityHasher;

    fn build_hasher(&self) -> IdentityHasher {
        IdentityHasher(0)
    }
}

pub type PatternsHashmap = HashMap<u64, BlockId, IdentityHasherBuilder>;
// type PatternsHashmap = FxHashMap<u64, BlockId>;

pub fn compute_empty_branch_hash() -> u64 {
    let mut hasher = FxHasher::default();

    // Hash branch tag and child presence
    hasher.write_u8(NODE_TYPE_BRANCH);
    for _ in EMPTY_CHILD.iter() {
        hasher.write_u8(CHILD_ABSENT);
    }

    hasher.finish()
}

#[inline(always)]
pub fn compute_leaf_hash_for_value<T: VoxelTrait>(value: &T) -> u64 {
    debug_assert!(*value != T::default(), "Leaf value should not be default");

    let mut hasher = FxHasher::default();

    // Hash leaf tag and value
    hasher.write_u8(NODE_TYPE_LEAF);
    value.hash(&mut hasher);

    hasher.finish()
}

#[inline(always)]
pub fn compute_branch_hash_for_children(children: &Children, types: u8, mask: u8) -> u64 {
    debug_assert!(children != &EMPTY_CHILD, "Empty children array");

    let mut hasher = FxHasher::default();

    // Hash branch tag and child hashes
    hasher.write_u8(NODE_TYPE_BRANCH);
    hasher.write_u16(((types as u16) << 8) | mask as u16);

    for child_id in children.iter() {
        child_id.raw().hash(&mut hasher);
    }

    hasher.finish()
}
