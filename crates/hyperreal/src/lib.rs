//! Exact rational, symbolic real, and computable real arithmetic.
//!
//! `hyperreal` represents values as a mix of exact rationals, recognized
//! symbolic forms such as `pi`, `e`, logarithms, and trigonometric special
//! forms, and lazily evaluated computable expressions. The public structural
//! query APIs expose cheap conservative facts for callers that need to avoid
//! forcing high-precision evaluation. The lazy approximation layer follows the
//! exact-real arithmetic model described by Boehm et al.,
//! https://doi.org/10.1145/319838.319860.

mod rational;
pub use crate::rational::Rational;

mod structural;
pub use crate::structural::{MagnitudeBits, RealSign, RealStructuralFacts, ZeroKnowledge};

#[cfg(feature = "dispatch-trace")]
pub mod dispatch_trace;

#[cfg(feature = "dispatch-trace")]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {
        $crate::dispatch_trace::record($layer, $operation, $path);
    };
}

#[cfg(not(feature = "dispatch-trace"))]
macro_rules! trace_dispatch {
    ($layer:expr, $operation:expr, $path:expr) => {};
}

pub(crate) use trace_dispatch;

mod computable;
pub use crate::computable::Computable;

mod real;
pub use crate::real::Real;

#[cfg(feature = "simple")]
mod simple;
#[cfg(feature = "simple")]
pub use crate::simple::Simple;

mod problem;
pub use crate::problem::Problem;

mod serde;
