//! Shared exact scalar and cancellation fixtures for Hyperlattice integration tests.
//!
//! These constructors do not assert geometric validity, policy decisions, or a
//! particular dispatch count. The near-zero sine fixture is mathematically nonzero;
//! its name refers to uncertain structural knowledge, not an exact-zero construction.
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use hyperlattice::Real;

/// Lift a signed i32 test literal exactly into Real.
pub fn r(value: i32) -> Real {
    value.into()
}

/// Construct an exact rational test value; unwrap panics for an invalid denominator such as zero.
#[allow(dead_code)]
pub fn frac(numerator: i64, denominator: u64) -> Real {
    hyperlattice::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

/// Create a fresh false cancellation flag; later Arc clones share that AtomicBool.
#[allow(dead_code)]
pub fn abort_signal() -> hyperlattice::AbortSignal {
    Arc::new(AtomicBool::new(false))
}

/// Construct sin(1)-sin(1+2^-60) for tests of unavailable cheap zero knowledge.
///
/// The arguments are radians and the exact difference is negative, not zero.
/// This helper does not assert the resulting zero_status or refine its sign;
/// consumer tests establish the structural behavior they require on this revision.
#[allow(dead_code)]
pub fn unknown_zero() -> Real {
    let one = r(1);
    let nearby = one.clone() + frac(1, 1_u64 << 60);
    Real::sin(one) - Real::sin(nearby)
}
