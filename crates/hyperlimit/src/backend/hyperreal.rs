//! Hyperreal integration sketch.
//!
//! This module is intentionally a shape, not a hard dependency. Once the
//! `hyperreal` public API is stable, replace `HyperrealLike` implementations
//! with concrete impls for the crate's scalar type.

use crate::backend::BackendCapabilities;
use crate::predicate::SignKnowledge;
use crate::scalar::{MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar};

/// Capabilities expected from a mature Hyperreal adapter.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: true,
    exact_zero: true,
    magnitude_bounds: true,
    exact_arithmetic: true,
    adaptive_refinement: true,
    robust_fallback: false,
};

/// Minimal facade this crate would need from `hyperreal`.
pub trait HyperrealLike:
    Clone
    + core::fmt::Debug
    + core::ops::Add<Output = Self>
    + core::ops::Sub<Output = Self>
    + core::ops::Mul<Output = Self>
{
    fn known_sign(&self) -> SignKnowledge;
    fn is_exact_zero(&self) -> Option<bool>;
    fn is_provably_nonzero(&self) -> Option<bool>;
    fn is_rational_only(&self) -> Option<bool>;
    fn magnitude_bounds(&self) -> Option<MagnitudeBounds>;
    fn to_f64_approx(&self) -> Option<f64>;
}

/// Newtype adapter to avoid orphan-rule conflicts while the integration settles.
#[derive(Clone, Debug)]
pub struct HyperrealScalar<T>(pub T);

impl<T: HyperrealLike> StructuralScalar for HyperrealScalar<T> {
    fn scalar_facts(&self) -> ScalarFacts {
        ScalarFacts {
            sign: self.0.known_sign().sign(),
            exact_zero: self.0.is_exact_zero(),
            provably_nonzero: self.0.is_provably_nonzero(),
            exact: None,
            rational_only: self.0.is_rational_only(),
            magnitude: self.0.magnitude_bounds(),
        }
    }

    fn known_sign(&self) -> SignKnowledge {
        self.0.known_sign()
    }
}

impl<T: HyperrealLike> PredicateScalar for HyperrealScalar<T> {
    fn to_f64(&self) -> Option<f64> {
        self.0.to_f64_approx()
    }
}

impl<T: HyperrealLike> core::ops::Add for HyperrealScalar<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl<T: HyperrealLike> core::ops::Sub for HyperrealScalar<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl<T: HyperrealLike> core::ops::Mul for HyperrealScalar<T> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}
