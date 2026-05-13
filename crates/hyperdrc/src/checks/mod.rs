//! Design-readiness checks grouped by the data model they operate on.
//!
//! `layer` checks work on already-flattened 2D geometry such as Gerber-derived
//! `Sketch` layers. `board` checks use richer board context such as KiCad nets,
//! drills, vias, and panel features.

mod artifacts;
pub mod board;
mod distance;
mod excellon;
pub mod layer;
pub mod manifest;

pub use artifacts::*;
pub use board::*;
pub use excellon::*;
pub use layer::*;
pub use manifest::*;
