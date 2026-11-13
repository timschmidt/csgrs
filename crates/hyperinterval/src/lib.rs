//! Scalar sets with explicit empty states, exact endpoints and checked ingress.
//!
//! Hyperinterval encloses; Hyperlimit decides; Hypersolve certifies. A set
//! containing zero is not evidence that an exact point equals zero. Set carriers
//! do not implement scalar ordering or implicitly select a midpoint. Exact width
//! values have a separate measurement contract and mathematical value ordering.
//!
//! The default build is allocation-free `no_std`. `integer` enables discrete
//! integer sets; `fixed` enables closed real intervals on an exact binary
//! lattice. Neither feature depends on Hyperreal. Optional `serde` validates
//! endpoint order and scale on import. There is no ambient rounding state.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

#[cfg(feature = "dyadic")]
mod arctangent;
#[cfg(any(
    feature = "integer",
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-proposal",
    feature = "float-enclosure"
))]
mod bounds;
#[cfg(feature = "dyadic")]
mod dyadic;
#[cfg(feature = "dyadic")]
mod dyadic_order;
#[cfg(feature = "dyadic")]
mod exponential;
mod extended;
#[cfg(feature = "dyadic")]
mod extended_dyadic;
#[cfg(feature = "fixed")]
mod fixed;
#[cfg(feature = "fixed")]
mod fixed_arithmetic;
#[cfg(feature = "float-enclosure")]
mod float;
#[cfg(feature = "dyadic")]
mod fractional_power;
#[cfg(any(feature = "float-proposal", feature = "float-enclosure"))]
mod ieee;
#[cfg(feature = "integer")]
mod integer;
#[cfg(feature = "integer")]
mod integer_arithmetic;
#[cfg(feature = "dyadic")]
mod inverse_functions;
#[cfg(feature = "dyadic")]
mod logarithm;
#[cfg(feature = "float-proposal")]
mod proposal;
#[cfg(feature = "rational")]
mod rational;
#[cfg(feature = "real-ball")]
mod real_ball;
mod split;
#[cfg(any(feature = "fixed", feature = "float-enclosure"))]
mod sqrt;
mod state;
#[cfg(feature = "dyadic")]
mod tangent;
#[cfg(feature = "dyadic")]
mod trigonometry;
pub use extended::{ExtendedInterval, ExtendedIntervalView, FiniteRealInterval};
pub use split::IntervalSet2;

#[cfg(feature = "dyadic")]
pub use dyadic::{DyadicBudget, DyadicInterval};
#[cfg(feature = "fixed")]
pub use fixed::{FixedEndpoint, FixedInterval, FixedWidth};
#[cfg(feature = "integer")]
pub use integer::{IntegerEndpoint, IntegerRange};
#[cfg(feature = "float-proposal")]
pub use proposal::{F32ProposalInterval, F64ProposalInterval};
#[cfg(feature = "rational")]
pub use rational::RationalInterval;
#[cfg(feature = "real-ball")]
pub use real_ball::{BallRelation, RealBall};
pub use state::{DomainCoverage, IntervalError, MidpointRounding, RangeEnclosure, SignSet};

#[cfg(feature = "float-enclosure")]
pub use float::{F32Enclosure, F32Width, F64Enclosure, F64Width};
