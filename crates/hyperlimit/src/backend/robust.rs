//! Adapter for the `robust` crate's adaptive floating-point predicates.

use crate::backend::BackendCapabilities;
use crate::orient::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, Sign};
use crate::scalar::PredicateScalar;

/// Capabilities provided by the `robust` crate for primitive-like coordinates.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: false,
    exact_zero: false,
    magnitude_bounds: false,
    exact_arithmetic: false,
    adaptive_refinement: false,
    robust_fallback: true,
};

/// Evaluate a 2D orientation with adaptive robust arithmetic.
pub fn orient2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let pa = coord2(a)?;
    let pb = coord2(b)?;
    let pc = coord2(c)?;

    sign_outcome(robust::orient2d(pa, pb, pc))
}

/// Evaluate a 3D orientation with adaptive robust arithmetic.
pub fn orient3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let pa = coord3(a)?;
    let pb = coord3(b)?;
    let pc = coord3(c)?;
    let pd = coord3(d)?;

    sign_outcome(robust::orient3d(pa, pb, pc, pd))
}

/// Evaluate a 2D incircle predicate with adaptive robust arithmetic.
pub fn incircle2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let pa = coord2(a)?;
    let pb = coord2(b)?;
    let pc = coord2(c)?;
    let pd = coord2(d)?;

    sign_outcome(robust::incircle(pa, pb, pc, pd))
}

/// Evaluate a 3D insphere predicate with adaptive robust arithmetic.
pub fn insphere3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let pa = coord3(a)?;
    let pb = coord3(b)?;
    let pc = coord3(c)?;
    let pd = coord3(d)?;
    let pe = coord3(e)?;

    sign_outcome(robust::insphere(pa, pb, pc, pd, pe))
}

fn coord2<S: PredicateScalar>(point: &Point2<S>) -> Option<robust::Coord<f64>> {
    Some(robust::Coord {
        x: finite(point.x.to_f64()?)?,
        y: finite(point.y.to_f64()?)?,
    })
}

fn coord3<S: PredicateScalar>(point: &Point3<S>) -> Option<robust::Coord3D<f64>> {
    Some(robust::Coord3D {
        x: finite(point.x.to_f64()?)?,
        y: finite(point.y.to_f64()?)?,
        z: finite(point.z.to_f64()?)?,
    })
}

fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

fn sign_outcome(value: f64) -> Option<PredicateOutcome<Sign>> {
    Some(PredicateOutcome::decided(
        Sign::from_f64(value)?,
        Certainty::Exact,
        Escalation::RobustFallback,
    ))
}
