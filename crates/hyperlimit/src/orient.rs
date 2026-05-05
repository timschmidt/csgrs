//! Orientation predicates.

use crate::classify::LineSide;
use crate::filter::det_sign_filter;
use crate::predicate::{
    Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::resolve::{map_outcome, resolve_scalar_sign, signed_term_filter};
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
    let left = abx.clone() * acy.clone();
    let right = aby.clone() * acx.clone();
    let det = left.clone() - right.clone();

    resolve_scalar_sign(
        &det,
        policy,
        || {
            signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)])
                .or_else(|| orient2d_filter(&abx, &aby, &acx, &acy))
        },
        || exact_orient2d(a, b, c),
        || fallback_orient2d_if_allowed(a, b, c, policy),
        RefinementNeed::RobustFallback,
    )
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

    let t0 = adx.clone() * (bdy.clone() * cdz.clone() - bdz.clone() * cdy.clone());
    let t1 = ady.clone() * (bdz.clone() * cdx.clone() - bdx.clone() * cdz.clone());
    let t2 = adz.clone() * (bdx.clone() * cdy.clone() - bdy.clone() * cdx.clone());
    let det = t0.clone() + t1.clone() + t2.clone();

    resolve_scalar_sign(
        &det,
        policy,
        || {
            signed_term_filter(&[
                (&t0, Sign::Positive),
                (&t1, Sign::Positive),
                (&t2, Sign::Positive),
            ])
            .or_else(|| orient3d_filter(&adx, &ady, &adz, &bdx, &bdy, &bdz, &cdx, &cdy, &cdz))
        },
        || exact_orient3d(a, b, c, d),
        || fallback_orient3d_if_allowed(a, b, c, d, policy),
        RefinementNeed::RobustFallback,
    )
}

/// Classify `point` relative to the oriented line from `from` to `to`.
pub fn classify_point_line<S: PredicateScalar>(
    from: &Point2<S>,
    to: &Point2<S>,
    point: &Point2<S>,
) -> PredicateOutcome<LineSide> {
    classify_point_line_with_policy(from, to, point, PredicatePolicy::default())
}

/// Classify `point` relative to the oriented line from `from` to `to` with an
/// explicit escalation policy.
pub fn classify_point_line_with_policy<S: PredicateScalar>(
    from: &Point2<S>,
    to: &Point2<S>,
    point: &Point2<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<LineSide> {
    map_outcome(
        orient2d_with_policy(from, to, point, policy),
        LineSide::from,
    )
}

/// In-circle predicate for four 2D points.
///
/// Positive means `d` lies inside the oriented circumcircle through `a`, `b`,
/// and `c` when those three points are counter-clockwise. Reversing the
/// orientation of `a`, `b`, and `c` reverses the sign.
pub fn incircle2d<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
) -> PredicateOutcome<Sign> {
    incircle2d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// In-circle predicate for four 2D points with an explicit escalation policy.
pub fn incircle2d_with_policy<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let adx = a.x.clone() - d.x.clone();
    let ady = a.y.clone() - d.y.clone();
    let bdx = b.x.clone() - d.x.clone();
    let bdy = b.y.clone() - d.y.clone();
    let cdx = c.x.clone() - d.x.clone();
    let cdy = c.y.clone() - d.y.clone();

    let adx2 = adx.clone() * adx.clone();
    let ady2 = ady.clone() * ady.clone();
    let bdx2 = bdx.clone() * bdx.clone();
    let bdy2 = bdy.clone() * bdy.clone();
    let cdx2 = cdx.clone() * cdx.clone();
    let cdy2 = cdy.clone() * cdy.clone();
    let alift = adx2 + ady2;
    let blift = bdx2 + bdy2;
    let clift = cdx2 + cdy2;

    let det = alift.clone() * (bdx.clone() * cdy.clone() - cdx.clone() * bdy.clone())
        + blift.clone() * (cdx.clone() * ady.clone() - adx.clone() * cdy.clone())
        + clift.clone() * (adx.clone() * bdy.clone() - bdx.clone() * ady.clone());

    resolve_scalar_sign(
        &det,
        policy,
        || None,
        || exact_incircle2d(a, b, c, d),
        || fallback_incircle2d_if_allowed(a, b, c, d, policy),
        RefinementNeed::RobustFallback,
    )
}

/// In-sphere predicate for five 3D points.
///
/// Positive means `e` lies inside the oriented circumsphere through `a`, `b`,
/// `c`, and `d` when the tetrahedron orientation matches the robust backend's
/// convention. Reversing that orientation reverses the sign.
pub fn insphere3d<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
) -> PredicateOutcome<Sign> {
    insphere3d_with_policy(a, b, c, d, e, PredicatePolicy::default())
}

/// In-sphere predicate for five 3D points with an explicit escalation policy.
pub fn insphere3d_with_policy<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let aex = a.x.clone() - e.x.clone();
    let bex = b.x.clone() - e.x.clone();
    let cex = c.x.clone() - e.x.clone();
    let dex = d.x.clone() - e.x.clone();
    let aey = a.y.clone() - e.y.clone();
    let bey = b.y.clone() - e.y.clone();
    let cey = c.y.clone() - e.y.clone();
    let dey = d.y.clone() - e.y.clone();
    let aez = a.z.clone() - e.z.clone();
    let bez = b.z.clone() - e.z.clone();
    let cez = c.z.clone() - e.z.clone();
    let dez = d.z.clone() - e.z.clone();

    let ab = aex.clone() * bey.clone() - bex.clone() * aey.clone();
    let bc = bex.clone() * cey.clone() - cex.clone() * bey.clone();
    let cd = cex.clone() * dey.clone() - dex.clone() * cey.clone();
    let da = dex.clone() * aey.clone() - aex.clone() * dey.clone();
    let ac = aex.clone() * cey.clone() - cex.clone() * aey.clone();
    let bd = bex.clone() * dey.clone() - dex.clone() * bey.clone();

    let abc = aez.clone() * bc.clone() - bez.clone() * ac.clone() + cez.clone() * ab.clone();
    let bcd = bez.clone() * cd.clone() - cez.clone() * bd.clone() + dez.clone() * bc.clone();
    let cda = cez.clone() * da.clone() + dez.clone() * ac.clone() + aez.clone() * cd.clone();
    let dab = dez.clone() * ab.clone() + aez.clone() * bd.clone() + bez.clone() * da.clone();

    let alift = aex.clone() * aex + aey.clone() * aey + aez.clone() * aez;
    let blift = bex.clone() * bex + bey.clone() * bey + bez.clone() * bez;
    let clift = cex.clone() * cex + cey.clone() * cey + cez.clone() * cez;
    let dlift = dex.clone() * dex + dey.clone() * dey + dez.clone() * dez;

    let left = dlift.clone() * abc + blift.clone() * cda;
    let right = clift.clone() * dab + alift.clone() * bcd;
    let det = left.clone() - right.clone();

    resolve_scalar_sign(
        &det,
        policy,
        || signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)]),
        || exact_insphere3d(a, b, c, d, e),
        || fallback_insphere3d_if_allowed(a, b, c, d, e, policy),
        RefinementNeed::RobustFallback,
    )
}

fn exact_orient2d<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
) -> Option<Sign> {
    None
}

fn exact_orient3d<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
) -> Option<Sign> {
    None
}

fn exact_incircle2d<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
    _d: &Point2<S>,
) -> Option<Sign> {
    None
}

fn exact_insphere3d<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
    _e: &Point3<S>,
) -> Option<Sign> {
    None
}

#[cfg(feature = "robust")]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| {
            #[cfg(feature = "geogram")]
            {
                crate::backend::geogram::orient2d(a, b, c)
            }

            #[cfg(not(feature = "geogram"))]
            {
                crate::backend::robust::orient2d(a, b, c)
            }
        })
        .flatten()
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| crate::backend::geogram::orient2d(a, b, c))
        .flatten()
}

#[cfg(not(feature = "robust"))]
#[cfg(not(feature = "geogram"))]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
    _policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    None
}

#[cfg(feature = "robust")]
fn fallback_orient3d_if_allowed<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| {
            #[cfg(feature = "geogram")]
            {
                crate::backend::geogram::orient3d(a, b, c, d)
            }

            #[cfg(not(feature = "geogram"))]
            {
                crate::backend::robust::orient3d(a, b, c, d)
            }
        })
        .flatten()
}

#[cfg(feature = "robust")]
fn fallback_incircle2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| {
            #[cfg(feature = "geogram")]
            {
                crate::backend::geogram::incircle2d(a, b, c, d)
            }

            #[cfg(not(feature = "geogram"))]
            {
                crate::backend::robust::incircle2d(a, b, c, d)
            }
        })
        .flatten()
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_incircle2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| crate::backend::geogram::incircle2d(a, b, c, d))
        .flatten()
}

#[cfg(not(feature = "robust"))]
#[cfg(not(feature = "geogram"))]
fn fallback_incircle2d_if_allowed<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
    _d: &Point2<S>,
    _policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    None
}

#[cfg(feature = "robust")]
fn fallback_insphere3d_if_allowed<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| {
            #[cfg(feature = "geogram")]
            {
                crate::backend::geogram::insphere3d(a, b, c, d, e)
            }

            #[cfg(not(feature = "geogram"))]
            {
                crate::backend::robust::insphere3d(a, b, c, d, e)
            }
        })
        .flatten()
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_insphere3d_if_allowed<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| crate::backend::geogram::insphere3d(a, b, c, d, e))
        .flatten()
}

#[cfg(not(feature = "robust"))]
#[cfg(not(feature = "geogram"))]
fn fallback_insphere3d_if_allowed<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
    _e: &Point3<S>,
    _policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    None
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_orient3d_if_allowed<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    policy
        .allow_robust_fallback
        .then(|| crate::backend::geogram::orient3d(a, b, c, d))
        .flatten()
}

#[cfg(not(feature = "robust"))]
#[cfg(not(feature = "geogram"))]
fn fallback_orient3d_if_allowed<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
    _policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    None
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
    #[cfg(any(
        feature = "robust",
        feature = "geogram",
        feature = "hyperreal",
        feature = "interval",
        feature = "realistic-blas"
    ))]
    use crate::predicate::Certainty;

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

        #[cfg(not(any(feature = "robust", feature = "geogram")))]
        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        );

        #[cfg(any(feature = "robust", feature = "geogram"))]
        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::RobustFallback)
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

    #[test]
    fn classifies_point_line() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);

        assert_eq!(
            classify_point_line(&a, &b, &Point2::new(0.0, 1.0)).value(),
            Some(LineSide::Left)
        );
        assert_eq!(
            classify_point_line(&a, &b, &Point2::new(0.0, -1.0)).value(),
            Some(LineSide::Right)
        );
    }

    #[cfg(feature = "hyperreal")]
    #[test]
    fn hyperreal_exact_rational_zero_decides_structurally() {
        let a = Point2::new(hyperreal::Real::from(0), hyperreal::Real::from(0));
        let b = Point2::new(hyperreal::Real::from(1), hyperreal::Real::from(1));
        let c = Point2::new(hyperreal::Real::from(2), hyperreal::Real::from(2));

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural)
        );
    }

    #[cfg(feature = "realistic-blas")]
    #[test]
    fn realistic_blas_exact_rational_zero_decides_structurally() {
        type Scalar = realistic_blas::Scalar<realistic_blas::DefaultBackend>;

        let a: Point2<Scalar> = Point2::new(
            realistic_blas::Scalar::from(0),
            realistic_blas::Scalar::from(0),
        );
        let b: Point2<Scalar> = Point2::new(
            realistic_blas::Scalar::from(1),
            realistic_blas::Scalar::from(1),
        );
        let c: Point2<Scalar> = Point2::new(
            realistic_blas::Scalar::from(2),
            realistic_blas::Scalar::from(2),
        );

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural)
        );
    }

    #[cfg(feature = "interval")]
    #[test]
    fn interval_orientation_uses_interval_sign_without_fallback() {
        let zero = inari::const_interval!(0.0, 0.0);
        let one = inari::const_interval!(1.0, 1.0);
        let y = inari::const_interval!(1.0, 2.0);

        let a = Point2::new(zero, zero);
        let b = Point2::new(one, zero);
        let c = Point2::new(zero, y);

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::decided(Sign::Positive, Certainty::Filtered, Escalation::Structural)
        );
    }

    #[cfg(feature = "interval")]
    #[test]
    fn interval_orientation_does_not_fallback_to_midpoints() {
        let zero = inari::const_interval!(0.0, 0.0);
        let one = inari::const_interval!(1.0, 1.0);
        let spanning = inari::const_interval!(-1.0, 1.0);

        let a = Point2::new(zero, zero);
        let b = Point2::new(one, zero);
        let c = Point2::new(zero, spanning);

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        );
    }

    #[cfg(any(feature = "robust", feature = "geogram"))]
    #[test]
    fn fallback_incircle_classifies_inside_outside_and_boundary() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        let c = Point2::new(0.0, 1.0);

        assert_eq!(
            incircle2d(&a, &b, &c, &Point2::new(0.25, 0.25)).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            incircle2d(&a, &b, &c, &Point2::new(2.0, 2.0)).value(),
            Some(Sign::Negative)
        );
        assert_eq!(
            incircle2d(&a, &b, &c, &Point2::new(1.0, 1.0)).value(),
            Some(Sign::Zero)
        );
    }

    #[cfg(any(feature = "robust", feature = "geogram"))]
    #[test]
    fn fallback_insphere_classifies_boundary() {
        let a = Point3::new(1.0, 0.0, 0.0);
        let b = Point3::new(0.0, 1.0, 0.0);
        let c = Point3::new(0.0, 0.0, 1.0);
        let d = Point3::new(-1.0, 0.0, 0.0);
        let e = Point3::new(0.0, -1.0, 0.0);

        assert_eq!(insphere3d(&a, &b, &c, &d, &e).value(), Some(Sign::Zero));
    }

    #[cfg(any(feature = "robust", feature = "geogram"))]
    #[test]
    fn robust_fallback_respects_policy() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 1.0);
        let c = Point2::new(2.0, 2.0);
        let policy = PredicatePolicy {
            allow_robust_fallback: false,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, policy),
            PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
        );
    }
}
