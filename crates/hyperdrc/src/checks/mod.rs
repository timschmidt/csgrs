//! Design-readiness checks grouped by the data model they operate on.
//!
//! `layer` checks work on already-flattened 2D geometry such as Gerber-derived
//! `Sketch` layers. `drill` checks focus on holes, slots, and cross-source drill
//! tables. `board` checks use richer board context such as KiCad nets, vias,
//! component features, and panel intent.

mod artifacts;
pub mod assembly;
pub mod board;
mod constraints;
mod distance;
pub mod drill;
mod excellon;
pub mod layer;
pub mod manifest;
pub mod stencil;
mod surface_finish;

pub use artifacts::*;
pub use assembly::*;
pub use board::*;
pub use constraints::*;
pub use drill::*;
pub use excellon::*;
pub use layer::*;
pub use manifest::*;
pub use stencil::*;
