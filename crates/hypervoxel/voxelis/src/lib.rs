// #![warn(missing_docs)]
// #![warn(rustdoc::missing_crate_level_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
// #![allow(clippy::module_name_repetitions)]
#![warn(clippy::cargo)]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::if_not_else)]

pub mod core;
pub mod io;
pub mod model;
pub mod spatial;
pub mod storage;
pub mod utils;
pub mod voxel;
pub mod world;

pub use core::{Batch, BlockId, TraversalDepth};
pub use storage::NodeStore;
pub use voxel::VoxelTrait;
