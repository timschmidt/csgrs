//! Design-readiness checks grouped by the data model they operate on.
//!
//! `layer` checks work on already-flattened 2D geometry such as Gerber-derived
//! `Sketch` layers. `board` checks use richer board context such as KiCad nets,
//! drills, vias, and panel features.

pub mod board;
mod distance;
pub mod layer;
pub mod manifest;

pub use board::*;
pub use layer::*;
pub use manifest::*;
