//! Scalar sets with explicit empty states, exact endpoints and checked ingress.
//!
//! Hyperinterval encloses; Hyperlimit decides; Hypersolve certifies. A set
//! containing zero is not evidence that an exact point equals zero. No carrier
//! implements scalar ordering or implicitly selects a midpoint.
//!
//! The default build is allocation-free `no_std`. `integer` enables discrete
//! integer sets; `fixed` enables closed real intervals on an exact binary
//! lattice. Neither feature depends on Hyperreal. Optional `serde` validates
//! endpoint order and scale on import. There is no ambient rounding state.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

#[cfg(any(feature = "integer", feature = "fixed"))]
mod bounds;
#[cfg(feature = "fixed")]
mod fixed;
#[cfg(feature = "integer")]
mod integer;
mod state;

#[cfg(feature = "fixed")]
pub use fixed::{FixedEndpoint, FixedInterval};
#[cfg(feature = "integer")]
pub use integer::{IntegerEndpoint, IntegerRange};
pub use state::{DomainCoverage, IntervalError, RangeEnclosure, SignSet};
