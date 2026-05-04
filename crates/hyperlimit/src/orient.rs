//! Orientation predicates.

use crate::filter::det_sign_filter;
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::scalar::PredicateScalar;

/// 2D point with scalar coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2<S> {
    pub x: S,
    pub y: S,
}

impl<S> Point2<S> {
    pub const fn new(x: S, y: S) -> Self {
        Self { x, y }
    }
}

/// 3D point with scalar coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3<S> {
    pub x: S,
    pub y: S,
    pub z: S,
}

impl<S> Point3<S> {
    pub const fn new(x: S, y: S, z: S) -> Self {
        Self { x, y, z }
    }
}

/// Orientation of three 2D points.
pub fn orient2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
) -> PredicateOutcome<Sign> {
    orient2d_with_policy(a, b, c, PredicatePolicy::default())
}

/// Orientation of three 2D points with an explicit escalation policy.
pub fn orient2d_with_policy<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let abx = b.x.clone() - a.x.clone();
    let aby = b.y.clone() - a.y.clone();
    let acx = c.x.clone() - a.x.clone();
    let acy = c.y.clone() - a.y.clone();
    let det = abx.clone() * acy.clone() - aby.clone() * acx.clone();

    decide_scalar_sign(&det, Escalation::Structural)
        .or_else(|| orient2d_filter(&abx, &aby, &acx, &acy))
        .or_else(|| approximate_if_allowed(&det, policy))
        .unwrap_or_else(|| {
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        })
}

/// Orientation of four 3D points. Positive means `d` is on the positive side
/// of the oriented plane through `a`, `b`, and `c`.
pub fn orient3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
) -> PredicateOutcome<Sign> {
    orient3d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// Orientation of four 3D points with an explicit escalation policy.
pub fn orient3d_with_policy<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let adx = a.x.clone() - d.x.clone();
    let ady = a.y.clone() - d.y.clone();
    let adz = a.z.clone() - d.z.clone();
    let bdx = b.x.clone() - d.x.clone();
    let bdy = b.y.clone() - d.y.clone();
    let bdz = b.z.clone() - d.z.clone();
    let cdx = c.x.clone() - d.x.clone();
    let cdy = c.y.clone() - d.y.clone();
    let cdz = c.z.clone() - d.z.clone();

    let det = adx.clone() * (bdy.clone() * cdz.clone() - bdz.clone() * cdy.clone())
        + ady.clone() * (bdz.clone() * cdx.clone() - bdx.clone() * cdz.clone())
        + adz.clone() * (bdx.clone() * cdy.clone() - bdy.clone() * cdx.clone());

    decide_scalar_sign(&det, Escalation::Structural)
        .or_else(|| orient3d_filter(&adx, &ady, &adz, &bdx, &bdy, &bdz, &cdx, &cdy, &cdz))
        .or_else(|| approximate_if_allowed(&det, policy))
        .unwrap_or_else(|| {
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        })
}

fn decide_scalar_sign<S: PredicateScalar>(
    value: &S,
    stage: Escalation,
) -> Option<PredicateOutcome<Sign>> {
    match value.known_sign() {
        SignKnowledge::Known { sign, certainty } => {
            Some(PredicateOutcome::decided(sign, certainty, stage))
        }
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

fn approximate_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_approximate {
        return None;
    }

    let sign = Sign::from_f64(value.to_f64()?)?;
    Some(PredicateOutcome::decided(
        sign,
        Certainty::Approximate,
        Escalation::Undecided,
    ))
}

fn orient2d_filter<S: PredicateScalar>(
    abx: &S,
    aby: &S,
    acx: &S,
    acy: &S,
) -> Option<PredicateOutcome<Sign>> {
    let abx = abx.to_f64()?;
    let aby = aby.to_f64()?;
    let acx = acx.to_f64()?;
    let acy = acy.to_f64()?;

    let det = abx.mul_add(acy, -(aby * acx));
    let scale = abx.abs() * acy.abs() + aby.abs() * acx.abs();
    match det_sign_filter(det, scale, 8.0) {
        SignKnowledge::Known { sign, certainty } => Some(PredicateOutcome::decided(
            sign,
            certainty,
            Escalation::Filter,
        )),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

#[allow(clippy::too_many_arguments)]
fn orient3d_filter<S: PredicateScalar>(
    adx: &S,
    ady: &S,
    adz: &S,
    bdx: &S,
    bdy: &S,
    bdz: &S,
    cdx: &S,
    cdy: &S,
    cdz: &S,
) -> Option<PredicateOutcome<Sign>> {
    let adx = adx.to_f64()?;
    let ady = ady.to_f64()?;
    let adz = adz.to_f64()?;
    let bdx = bdx.to_f64()?;
    let bdy = bdy.to_f64()?;
    let bdz = bdz.to_f64()?;
    let cdx = cdx.to_f64()?;
    let cdy = cdy.to_f64()?;
    let cdz = cdz.to_f64()?;

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);
    let scale = adz.abs() * (bdxcdy.abs() + cdxbdy.abs())
        + bdz.abs() * (cdxady.abs() + adxcdy.abs())
        + cdz.abs() * (adxbdy.abs() + bdxady.abs());

    match det_sign_filter(det, scale, 32.0) {
        SignKnowledge::Known { sign, certainty } => Some(PredicateOutcome::decided(
            sign,
            certainty,
            Escalation::Filter,
        )),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orient2d_classifies_simple_triangle() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        let c = Point2::new(0.0, 1.0);

        assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));
    }

    #[test]
    fn orient2d_can_return_unknown_for_strict_degenerate_float() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 1.0);
        let c = Point2::new(2.0, 2.0);

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        );
    }

    #[test]
    fn orient3d_classifies_simple_tetrahedron() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);
        let d = Point3::new(0.0, 0.0, 1.0);

        assert_eq!(orient3d(&a, &b, &c, &d).value(), Some(Sign::Negative));
    }
}
