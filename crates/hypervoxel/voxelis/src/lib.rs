//! Sparse voxel octree/DAG storage, editing, and meshing.
//!
//! Start with [`VoxTree`] for one fixed-depth volume or [`world::VoxModel`] for
//! a chunked model. Shared nodes are owned by [`VoxInterner`], and operations
//! are exposed through the traits in [`spatial`].

#![warn(clippy::cargo)]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::if_not_else)]

pub mod core;
pub mod interner;
pub mod io;
pub mod spatial;
pub mod utils;
pub mod world;

pub use core::{Batch, BlockId, Lod, MaxDepth, TraversalDepth, VoxelTrait};
pub use interner::VoxInterner;
