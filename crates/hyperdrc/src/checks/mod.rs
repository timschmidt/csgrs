//! Design-readiness checks grouped by the data model they operate on.
//!
//! `layer` checks work on already-flattened 2D geometry such as Gerber-derived
//! `Sketch` layers. `drill` checks focus on holes, slots, and cross-source drill
//! tables. `board` checks use richer board context such as KiCad nets, vias,
//! component features, and panel intent.
//! `safety` checks focus on voltage, board-edge, and ESD protective-interface
//! readiness that benefits from board context but has a distinct review owner.
//! `signal` checks focus on mixed-signal partitioning and quiet-net guard or
//! return-path proximity.
//! `mechanical` checks focus on chassis, mounting, and keepout intent that uses
//! board context but is primarily mechanical production readiness.

mod artifact_handoff;
mod artifact_table;
mod artifacts;
pub mod assembly;
pub mod board;
mod constraints;
pub mod dense_pad;
mod distance;
pub mod drill;
mod excellon;
pub mod layer;
pub mod manifest;
pub mod mechanical;
pub mod power;
pub mod rf;
pub mod safety;
pub mod signal;
pub mod stencil;
mod surface_finish;
pub mod thermal;

pub use artifacts::*;
pub use assembly::*;
pub use board::*;
pub use constraints::*;
pub use dense_pad::*;
pub use drill::*;
pub use excellon::*;
pub use layer::*;
pub use manifest::*;
pub use mechanical::*;
pub use power::*;
pub use rf::*;
pub use safety::*;
pub use signal::*;
pub use stencil::*;
pub use thermal::*;
