use crate::BlockId;

use super::Children;

pub const MAX_ALLOWED_DEPTH: usize = 7;
pub const MAX_CHILDREN: usize = 8;
pub const NODE_TYPE_LEAF: u8 = 0;
pub const NODE_TYPE_BRANCH: u8 = 1;

pub const PATTERNS_TYPE_BRANCH: usize = 0;
pub const PATTERNS_TYPE_LEAF: usize = 1;

pub const CHILD_ABSENT: u8 = 0;

pub const PREALLOCATED_STACK_SIZE: usize = 32768;

pub const EMPTY_CHILD: Children = [const { BlockId::EMPTY }; MAX_CHILDREN];
