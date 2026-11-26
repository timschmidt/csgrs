//! Internal scalar-boundary conveniences shared by optional CSGRS features.
//!
//! Native constants and arithmetic remain exact. Primitive float export is lossy;
//! predicate helpers use the crate's strict policy, not an ambient session policy.
//! Several legacy-shaped helpers erase errors into `Option` or return only whether
//! a predicate was positively certified. Read those contracts before using their
//! results for geometric decisions. The internal [`IntoReal`] has a different
//! input set and native `Problem` error surface from [`crate::IntoReal`].

use hyperlattice::{Point3, Real, Vector3};
use hyperreal::{Problem, RealSign};
use std::cmp::Ordering;

/// Return the native symbolic value of π without a floating-point conversion.
pub(crate) fn pi() -> Real {
    Real::pi()
}

/// Return the native symbolic value 2π without a floating-point conversion.
pub(crate) fn tau() -> Real {
    Real::tau()
}

/// Return the exact binary value of the `1.0e-9_f64` test threshold.
/// This test-only helper is not a modeling coincidence tolerance or exact decimal 10⁻⁹.
#[cfg(test)]
pub(crate) fn tolerance() -> Real {
    hreal_from_f64(1.0e-9).expect("finite test tolerance")
}

/// Internal native-error conversion interface for boundary convenience helpers.
/// Finite primitive floats preserve their exact binary value. The supported
/// primitive list is defined by the implementations below; it is not the public
/// session trait's domain and this trait does not wrap failures in `ValidationError`.
pub(crate) trait IntoReal {
    /// Convert to the native scalar, preserving a native scalar failure if rejected.
    fn into_real(self) -> Result<Real, Problem>;
}

impl IntoReal for Real {
    /// Move an existing native scalar unchanged.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(self)
    }
}

impl IntoReal for &Real {
    /// Clone an existing native scalar without approximation.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(self.clone())
    }
}

impl IntoReal for f64 {
    /// Import the finite binary float value exactly; reject NaN and infinity.
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(self)
    }
}

impl IntoReal for &f64 {
    /// Import the finite binary float value exactly; reject NaN and infinity.
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(*self)
    }
}

impl IntoReal for f32 {
    /// Import the finite binary float value exactly; reject NaN and infinity.
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(self)
    }
}

impl IntoReal for &f32 {
    /// Import the finite binary float value exactly; reject NaN and infinity.
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(*self)
    }
}

impl IntoReal for usize {
    /// Cast through `u64`, then import exactly; lossless on targets with at most 64-bit `usize`.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self as u64))
    }
}

impl IntoReal for i64 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u64 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u32 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u16 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u8 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i16 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i32 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i8 {
    /// Import every value of this integer type exactly and infallibly.
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

/// Import any input supported by the internal conversion trait.
/// Despite the historical name, this also accepts native reals and listed integer
/// types. Float inputs are imported as exact binary values; invalid floats fail typed.
pub(crate) fn hreal_from_f64<T: IntoReal>(value: T) -> Result<Real, Problem> {
    value.into_real()
}

/// Import a finite binary32 value exactly; reject NaN and infinity with `Problem`.
pub(crate) fn hreal_from_f32(value: f32) -> Result<Real, Problem> {
    Real::try_from(value)
}

/// Return a finite lossy binary64 view, or `None` when no finite view is returned.
/// Rounding and underflow may lose source information; this supplies no predicate proof.
pub(crate) fn hreal_to_f64(value: &Real) -> Option<f64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

/// Return a strict decided sign, or `None` for an unresolved predicate.
/// The native uncertainty reason is discarded; no approximate terminal is consumed.
pub(crate) fn hreal_sign(value: &Real) -> Option<RealSign> {
    hyperlimit::classify_real_sign(value, crate::PREDICATE_POLICY)
        .value()
        .map(|sign| match sign {
            hyperlimit::Sign::Negative => RealSign::Negative,
            hyperlimit::Sign::Zero => RealSign::Zero,
            hyperlimit::Sign::Positive => RealSign::Positive,
        })
}

/// Import operands left then right and compare them under strict predicates.
/// `None` conflates failed scalar ingress with an undecided comparison and retains
/// neither diagnostic. No ordering is inferred from absence.
pub(crate) fn hreal_try_cmp<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Ordering> {
    let lhs = lhs.into_real().ok()?;
    let rhs = rhs.into_real().ok()?;
    hyperlimit::compare_reals(&lhs, &rhs, crate::PREDICATE_POLICY).value()
}

/// Return whether strict comparison certifies `lhs > rhs`.
/// `false` also covers invalid scalar input or undecided comparison; it is not
/// necessarily proof of `lhs <= rhs`. Both input types follow the internal trait.
pub(crate) fn hreal_gt_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_try_cmp(lhs, rhs), Some(Ordering::Greater))
}

/// Import operands in order and construct their exact difference.
/// Scalar ingress failures become `None`, discarding their native diagnostic.
pub(crate) fn hreal_sub<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Real> {
    Some(lhs.into_real().ok()? - rhs.into_real().ok()?)
}

/// Import operands and perform native checked division.
/// `None` covers either ingress failure or any division error, including an
/// unresolved denominator; the original cause is discarded.
pub(crate) fn hreal_div<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Real> {
    (lhs.into_real().ok()? / rhs.into_real().ok()?).ok()
}

/// Fold exact addition from zero in slice order and wrap the result in `Some`.
/// An empty slice returns `Some(0)`; this helper currently has no `None` branch
/// and performs no predicate or validity check on the resulting expression.
pub(crate) fn hreal_sum(values: &[Real]) -> Option<Real> {
    Some(
        values
            .iter()
            .cloned()
            .fold(Real::zero(), |sum, value| sum + value),
    )
}

/// Import a scalar and construct its native absolute-value expression.
/// Returns `None` on ingress failure; constructing the expression is not a sign proof.
pub(crate) fn hreal_abs<T: IntoReal>(value: T) -> Option<Real> {
    Some(value.into_real().ok()?.abs())
}

/// Select a minimum using strict pairwise ordering in slice order.
/// Returns `None` for empty input or the first undecided comparison. Ties retain
/// the earlier value; uncertainty is not treated as equality.
pub(crate) fn hreal_min(values: &[Real]) -> Option<Real> {
    let (first, rest) = values.split_first()?;
    rest.iter().try_fold(first.clone(), |best, value| {
        Some(match hreal_try_cmp(value, &best)? {
            Ordering::Less => value.clone(),
            Ordering::Equal | Ordering::Greater => best,
        })
    })
}

/// Select a maximum using strict pairwise ordering in slice order.
/// Returns `None` for empty input or the first undecided comparison. Ties retain
/// the earlier value; uncertainty is not treated as equality.
pub(crate) fn hreal_max(values: &[Real]) -> Option<Real> {
    let (first, rest) = values.split_first()?;
    rest.iter().try_fold(first.clone(), |best, value| {
        Some(match hreal_try_cmp(value, &best)? {
            Ordering::Greater => value.clone(),
            Ordering::Equal | Ordering::Less => best,
        })
    })
}

/// Clone point coordinates into a vector and wrap it in `Some`.
/// This conversion has no `None` branch and does not normalize or validate coordinates.
pub(crate) fn hvector3_from_point3(point: &Point3) -> Option<Vector3> {
    Some(point.to_vector())
}

/// Clone a native vector and wrap it in `Some`; no validity or length test is made.
pub(crate) fn hvector3_from_vector3(vector: &Vector3) -> Option<Vector3> {
    Some(vector.clone())
}

/// Construct `from + t * (to - from)` and wrap it in `Some`.
/// The parameter is not restricted to [0, 1], so extrapolation is allowed. No
/// predicate or range check is performed and the helper has no `None` branch.
pub(crate) fn hpoint_lerp(from: &Point3, to: &Point3, t: Real) -> Option<Point3> {
    Some(from.lerp(to, &t))
}

/// Return whether strict triangle classification certifies nondegeneracy.
/// A degenerate or undecided triangle returns `false`; false alone is not a
/// proof of zero area. No approximate terminal is consumed.
pub(crate) fn htriangle_area2_is_nonzero(a: &Point3, b: &Point3, c: &Point3) -> bool {
    matches!(
        hyperlimit::classify_triangle3_degeneracy(a, b, c, crate::PREDICATE_POLICY).value(),
        Some(hyperlimit::TriangleDegeneracy::NonDegenerate)
    )
}
