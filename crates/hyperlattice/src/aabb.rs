//! Unvalidated axis-aligned box bounds stored as exact Point3 corners.
//!
//! Ordering, emptiness conventions, containment, and boundary semantics belong to
//! consumers. This carrier performs no comparisons or rounding and has no policy or
//! certainty receipt. Public corner mutation can invalidate a consumer's bounds facts.

use crate::{Point3, Real};

/// Raw 3D minimum/maximum corner carrier, without a validated ordering or fill contract.
///
/// Derived equality compares stored coordinates; it does not compare represented
/// sets under a consumer's corner-normalization or empty-box conventions.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb {
    /// Intended lower corner; componentwise ordering against maxs is not enforced.
    pub mins: Point3,
    /// Intended upper corner; it may equal or precede mins in the supplied coordinates.
    pub maxs: Point3,
}

impl Aabb {
    /// Store the supplied corners unchanged; does not sort coordinates, prove mins<=maxs, or diagnose an inverted box.
    #[inline]
    pub const fn new(mins: Point3, maxs: Point3) -> Self {
        Self { mins, maxs }
    }

    /// Construct coincident exact-zero corners, representing the degenerate origin bounds rather than a canonical empty-set sentinel.
    #[inline]
    pub fn origin() -> Self {
        let zero = Real::zero();
        Self::new(
            Point3::new(zero.clone(), zero.clone(), zero.clone()),
            Point3::new(zero.clone(), zero.clone(), zero),
        )
    }
}
