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
//! `continuity` checks focus on same-net geometry that may be electrically
//! severed even when source net names still match.
//! `differential` checks focus on differential-pair coupling and spacing review
//! that benefits from net-name intent plus exact copper geometry.
//! `return_path` checks focus on split-plane and reference-island hazards for
//! high-speed copper.
//! `power_integrity` checks focus on high-current pad-entry and copper-spreading
//! readiness.
//! `mechanical` checks focus on chassis, mounting, and keepout intent that uses
//! board context but is primarily mechanical production readiness.

mod artifact_handoff;
mod artifact_table;
mod artifacts;
pub mod assembly;
pub mod board;
mod constraints;
pub mod continuity;
pub mod dense_pad;
pub mod differential;
mod distance;
pub mod drill;
mod excellon;
pub mod layer;
pub mod manifest;
pub mod mechanical;
pub mod power;
pub mod power_integrity;
pub mod return_path;
pub mod rf;
pub mod safety;
pub mod signal;
mod spatial;
mod spread;
pub mod stencil;
mod surface_finish;
pub mod thermal;

pub use artifacts::*;
pub use assembly::*;
pub use board::*;
pub use constraints::*;
pub use continuity::*;
pub use dense_pad::*;
pub use differential::*;
pub use drill::*;
pub use excellon::*;
pub use layer::*;
pub use manifest::*;
pub use mechanical::*;
pub use power::*;
pub use power_integrity::*;
pub use return_path::*;
pub use rf::*;
pub use safety::*;
pub use signal::*;
pub use stencil::*;
pub use thermal::*;
