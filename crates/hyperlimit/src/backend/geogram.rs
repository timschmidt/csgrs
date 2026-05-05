//! Adapter for the Rust-port branch of `geogram_predicates`.

use crate::backend::BackendCapabilities;
use crate::orient::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, Sign};
use crate::scalar::PredicateScalar;

/// Capabilities provided by Geogram's predicate module.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: false,
    exact_zero: false,
    magnitude_bounds: false,
    exact_arithmetic: false,
    adaptive_refinement: false,
    robust_fallback: true,
};

/// Evaluate a 2D orientation with Geogram predicates.
pub fn orient2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let a = point2(a)?;
    let b = point2(b)?;
    let c = point2(c)?;

    sign_outcome(geogram_predicates::orient_2d(&a, &b, &c))
}

/// Evaluate a 3D orientation with Geogram predicates.
pub fn orient3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let a = point3(a)?;
    let b = point3(b)?;
    let c = point3(c)?;
    let d = point3(d)?;

    sign_outcome(geogram_predicates::orient_3d(&a, &b, &c, &d))
}

/// Evaluate a 2D in-circle predicate with Geogram predicates.
///
/// Geogram's Rust port exposes the in-circle API with symbolic perturbation.
/// Calling both perturbation polarities preserves this crate's exact boundary
/// semantics: equal signs are nonzero, disagreement means exact zero.
pub fn incircle2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let a = point2(a)?;
    let b = point2(b)?;
    let c = point2(c)?;
    let d = point2(d)?;

    sign_outcome_from_sos(
        geogram_predicates::in_circle_2d_sos::<false>(&a, &b, &c, &d),
        geogram_predicates::in_circle_2d_sos::<true>(&a, &b, &c, &d),
    )
}

/// Evaluate a 3D in-sphere predicate with Geogram predicates.
///
/// As with in-circle, both perturbation polarities are evaluated so unperturbed
/// boundary cases return `Sign::Zero`.
pub fn insphere3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let a = point3(a)?;
    let b = point3(b)?;
    let c = point3(c)?;
    let d = point3(d)?;
    let e = point3(e)?;

    sign_outcome_from_sos(
        geogram_predicates::in_sphere_3d_sos::<false>(&a, &b, &c, &d, &e),
        geogram_predicates::in_sphere_3d_sos::<true>(&a, &b, &c, &d, &e),
    )
}

fn point2<S: PredicateScalar>(point: &Point2<S>) -> Option<[f64; 2]> {
    Some([finite(point.x.to_f64()?)?, finite(point.y.to_f64()?)?])
}

fn point3<S: PredicateScalar>(point: &Point3<S>) -> Option<[f64; 3]> {
    Some([
        finite(point.x.to_f64()?)?,
        finite(point.y.to_f64()?)?,
        finite(point.z.to_f64()?)?,
    ])
}

fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

fn sign_outcome(value: geogram_predicates::Sign) -> Option<PredicateOutcome<Sign>> {
    Some(PredicateOutcome::decided(
        map_sign(value),
        Certainty::RobustFloat,
        Escalation::RobustFallback,
    ))
}

fn sign_outcome_from_sos(
    negative_perturb: geogram_predicates::Sign,
    positive_perturb: geogram_predicates::Sign,
) -> Option<PredicateOutcome<Sign>> {
    if negative_perturb == positive_perturb {
        sign_outcome(negative_perturb)
    } else {
        Some(PredicateOutcome::decided(
            Sign::Zero,
            Certainty::RobustFloat,
            Escalation::RobustFallback,
        ))
    }
}

fn map_sign(value: geogram_predicates::Sign) -> Sign {
    match value {
        geogram_predicates::Sign::Negative => Sign::Negative,
        geogram_predicates::Sign::Zero => Sign::Zero,
        geogram_predicates::Sign::Positive => Sign::Positive,
    }
}
