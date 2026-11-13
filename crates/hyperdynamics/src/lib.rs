//! Owned vector fields with explicit time/state interpretation and native enclosures.
//!
//! Hyperdynamics assigns field semantics. Hypersolve validates symbolic source,
//! derivatives and images; Hyperlattice carries Cartesian sets; Hyperinterval
//! owns scalar enclosure arithmetic. Bounded Picard evolution retains source-bound
//! tubes, endpoints and exact clocks; broader reachability and whole-resource bounds
//! remain open.
#![forbid(unsafe_code)]
#![deny(missing_docs)]

/// Owned dyadic vector fields with validated expression/Jacobian association.
#[cfg(feature = "field")]
pub mod field;
