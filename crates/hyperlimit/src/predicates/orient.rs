//! Orientation predicates.

use crate::classify::LineSide;
use crate::filter::det_sign_filter;
pub use crate::geometry::{Point2, Point3};
use crate::predicate::{
    Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::resolve::{map_outcome, resolve_scalar_sign, signed_term_filter};
use crate::scalar::{BorrowedPredicateScalar, PredicateScalar};

pub use crate::batch::{
    Incircle2dCase, Insphere3dCase, Orient2dCase, Orient3dCase, classify_point_line_batch,
    classify_point_line_batch_with_policy, incircle2d_batch, incircle2d_batch_with_policy,
    insphere3d_batch, insphere3d_batch_with_policy, orient2d_batch, orient2d_batch_with_policy,
    orient3d_batch, orient3d_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use crate::batch::{
    classify_point_line_batch_parallel, classify_point_line_batch_parallel_with_policy,
    incircle2d_batch_parallel, incircle2d_batch_parallel_with_policy, insphere3d_batch_parallel,
    insphere3d_batch_parallel_with_policy, orient2d_batch_parallel,
    orient2d_batch_parallel_with_policy, orient3d_batch_parallel,
    orient3d_batch_parallel_with_policy,
};

/// Orientation of three 2D points.
pub fn orient2d<S: BorrowedPredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
) -> PredicateOutcome<Sign> {
    orient2d_with_policy(a, b, c, PredicatePolicy::default())
}

/// Orientation of three 2D points with an explicit escalation policy.
pub fn orient2d_with_policy<S: BorrowedPredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    // Exact symbolic backends opt into this conservative f64 prefilter so clear decisions
    // avoid constructing subtraction/multiplication expression trees at all.
    if S::prefer_f64_filter_before_arithmetic() {
        if let Some(outcome) = orient2d_point_filter(a, b, c) {
            crate::trace_dispatch!("hyperlimit", "orient2d", "f64-point-filter-hit");
            return outcome;
        }
        crate::trace_dispatch!("hyperlimit", "orient2d", "f64-point-filter-miss");
    }

    crate::trace_dispatch!("hyperlimit", "orient2d", "scalar-determinant");
    let abx = sub(&b.x, &a.x);
    let aby = sub(&b.y, &a.y);
    let acx = sub(&c.x, &a.x);
    let acy = sub(&c.y, &a.y);
    let left = mul(&abx, &acy);
    let right = mul(&aby, &acx);
    let det = sub(&left, &right);

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
pub fn orient3d<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
) -> PredicateOutcome<Sign> {
    orient3d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// Orientation of four 3D points with an explicit escalation policy.
pub fn orient3d_with_policy<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if S::prefer_orientation_f64_filter_before_arithmetic() {
        if let Some(outcome) = orient3d_point_filter(a, b, c, d) {
            crate::trace_dispatch!("hyperlimit", "orient3d", "f64-point-filter-hit");
            return outcome;
        }
        crate::trace_dispatch!("hyperlimit", "orient3d", "f64-point-filter-miss");
    }

    crate::trace_dispatch!("hyperlimit", "orient3d", "scalar-determinant");
    let adx = sub(&a.x, &d.x);
    let ady = sub(&a.y, &d.y);
    let adz = sub(&a.z, &d.z);
    let bdx = sub(&b.x, &d.x);
    let bdy = sub(&b.y, &d.y);
    let bdz = sub(&b.z, &d.z);
    let cdx = sub(&c.x, &d.x);
    let cdy = sub(&c.y, &d.y);
    let cdz = sub(&c.z, &d.z);

    let bdy_cdz = mul(&bdy, &cdz);
    let bdz_cdy = mul(&bdz, &cdy);
    let c0 = sub(&bdy_cdz, &bdz_cdy);
    let t0 = mul(&adx, &c0);

    let bdz_cdx = mul(&bdz, &cdx);
    let bdx_cdz = mul(&bdx, &cdz);
    let c1 = sub(&bdz_cdx, &bdx_cdz);
    let t1 = mul(&ady, &c1);

    let bdx_cdy = mul(&bdx, &cdy);
    let bdy_cdx = mul(&bdy, &cdx);
    let c2 = sub(&bdx_cdy, &bdy_cdx);
    let t2 = mul(&adz, &c2);

    let t01 = add(&t0, &t1);
    let det = add(&t01, &t2);

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
pub fn classify_point_line<S: BorrowedPredicateScalar>(
    from: &Point2<S>,
    to: &Point2<S>,
    point: &Point2<S>,
) -> PredicateOutcome<LineSide> {
    classify_point_line_with_policy(from, to, point, PredicatePolicy::default())
}

/// Classify `point` relative to the oriented line from `from` to `to` with an
/// explicit escalation policy.
pub fn classify_point_line_with_policy<S: BorrowedPredicateScalar>(
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
pub fn incircle2d<S: BorrowedPredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
) -> PredicateOutcome<Sign> {
    incircle2d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// In-circle predicate for four 2D points with an explicit escalation policy.
pub fn incircle2d_with_policy<S: BorrowedPredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if S::prefer_f64_filter_before_arithmetic() {
        if let Some(outcome) = incircle2d_point_filter(a, b, c, d) {
            crate::trace_dispatch!("hyperlimit", "incircle2d", "f64-point-filter-hit");
            return outcome;
        }
        crate::trace_dispatch!("hyperlimit", "incircle2d", "f64-point-filter-miss");
    }

    crate::trace_dispatch!("hyperlimit", "incircle2d", "scalar-determinant");
    let adx = sub(&a.x, &d.x);
    let ady = sub(&a.y, &d.y);
    let bdx = sub(&b.x, &d.x);
    let bdy = sub(&b.y, &d.y);
    let cdx = sub(&c.x, &d.x);
    let cdy = sub(&c.y, &d.y);

    let adx2 = mul(&adx, &adx);
    let ady2 = mul(&ady, &ady);
    let bdx2 = mul(&bdx, &bdx);
    let bdy2 = mul(&bdy, &bdy);
    let cdx2 = mul(&cdx, &cdx);
    let cdy2 = mul(&cdy, &cdy);
    let alift = add(&adx2, &ady2);
    let blift = add(&bdx2, &bdy2);
    let clift = add(&cdx2, &cdy2);

    let bdx_cdy = mul(&bdx, &cdy);
    let cdx_bdy = mul(&cdx, &bdy);
    let minor_a = sub(&bdx_cdy, &cdx_bdy);
    let term_a = mul(&alift, &minor_a);

    let cdx_ady = mul(&cdx, &ady);
    let adx_cdy = mul(&adx, &cdy);
    let minor_b = sub(&cdx_ady, &adx_cdy);
    let term_b = mul(&blift, &minor_b);

    let adx_bdy = mul(&adx, &bdy);
    let bdx_ady = mul(&bdx, &ady);
    let minor_c = sub(&adx_bdy, &bdx_ady);
    let term_c = mul(&clift, &minor_c);

    let term_ab = add(&term_a, &term_b);
    let det = add(&term_ab, &term_c);

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
pub fn insphere3d<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
) -> PredicateOutcome<Sign> {
    insphere3d_with_policy(a, b, c, d, e, PredicatePolicy::default())
}

/// In-sphere predicate for five 3D points with an explicit escalation policy.
pub fn insphere3d_with_policy<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if S::prefer_insphere_f64_filter_before_arithmetic() {
        if let Some(outcome) = insphere3d_point_filter(a, b, c, d, e) {
            crate::trace_dispatch!("hyperlimit", "insphere3d", "f64-point-filter-hit");
            return outcome;
        }
        crate::trace_dispatch!("hyperlimit", "insphere3d", "f64-point-filter-miss");
    }

    crate::trace_dispatch!("hyperlimit", "insphere3d", "scalar-determinant");
    let aex = sub(&a.x, &e.x);
    let bex = sub(&b.x, &e.x);
    let cex = sub(&c.x, &e.x);
    let dex = sub(&d.x, &e.x);
    let aey = sub(&a.y, &e.y);
    let bey = sub(&b.y, &e.y);
    let cey = sub(&c.y, &e.y);
    let dey = sub(&d.y, &e.y);
    let aez = sub(&a.z, &e.z);
    let bez = sub(&b.z, &e.z);
    let cez = sub(&c.z, &e.z);
    let dez = sub(&d.z, &e.z);

    let aex_bey = mul(&aex, &bey);
    let bex_aey = mul(&bex, &aey);
    let ab = sub(&aex_bey, &bex_aey);

    let bex_cey = mul(&bex, &cey);
    let cex_bey = mul(&cex, &bey);
    let bc = sub(&bex_cey, &cex_bey);

    let cex_dey = mul(&cex, &dey);
    let dex_cey = mul(&dex, &cey);
    let cd = sub(&cex_dey, &dex_cey);

    let dex_aey = mul(&dex, &aey);
    let aex_dey = mul(&aex, &dey);
    let da = sub(&dex_aey, &aex_dey);

    let aex_cey = mul(&aex, &cey);
    let cex_aey = mul(&cex, &aey);
    let ac = sub(&aex_cey, &cex_aey);

    let bex_dey = mul(&bex, &dey);
    let dex_bey = mul(&dex, &bey);
    let bd = sub(&bex_dey, &dex_bey);

    let aez_bc = mul(&aez, &bc);
    let bez_ac = mul(&bez, &ac);
    let cez_ab = mul(&cez, &ab);
    let abc_minus = sub(&aez_bc, &bez_ac);
    let abc = add(&abc_minus, &cez_ab);

    let bez_cd = mul(&bez, &cd);
    let cez_bd = mul(&cez, &bd);
    let dez_bc = mul(&dez, &bc);
    let bcd_minus = sub(&bez_cd, &cez_bd);
    let bcd = add(&bcd_minus, &dez_bc);

    let cez_da = mul(&cez, &da);
    let dez_ac = mul(&dez, &ac);
    let aez_cd = mul(&aez, &cd);
    let cda_partial = add(&cez_da, &dez_ac);
    let cda = add(&cda_partial, &aez_cd);

    let dez_ab = mul(&dez, &ab);
    let aez_bd = mul(&aez, &bd);
    let bez_da = mul(&bez, &da);
    let dab_partial = add(&dez_ab, &aez_bd);
    let dab = add(&dab_partial, &bez_da);

    let aex2 = mul(&aex, &aex);
    let aey2 = mul(&aey, &aey);
    let aez2 = mul(&aez, &aez);
    let alift_xy = add(&aex2, &aey2);
    let alift = add(&alift_xy, &aez2);

    let bex2 = mul(&bex, &bex);
    let bey2 = mul(&bey, &bey);
    let bez2 = mul(&bez, &bez);
    let blift_xy = add(&bex2, &bey2);
    let blift = add(&blift_xy, &bez2);

    let cex2 = mul(&cex, &cex);
    let cey2 = mul(&cey, &cey);
    let cez2 = mul(&cez, &cez);
    let clift_xy = add(&cex2, &cey2);
    let clift = add(&clift_xy, &cez2);

    let dex2 = mul(&dex, &dex);
    let dey2 = mul(&dey, &dey);
    let dez2 = mul(&dez, &dez);
    let dlift_xy = add(&dex2, &dey2);
    let dlift = add(&dlift_xy, &dez2);

    let dlift_abc = mul(&dlift, &abc);
    let blift_cda = mul(&blift, &cda);
    let left = add(&dlift_abc, &blift_cda);

    let clift_dab = mul(&clift, &dab);
    let alift_bcd = mul(&alift, &bcd);
    let right = add(&clift_dab, &alift_bcd);
    let det = sub(&left, &right);

    resolve_scalar_sign(
        &det,
        policy,
        || signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)]),
        || exact_insphere3d(a, b, c, d, e),
        || fallback_insphere3d_if_allowed(a, b, c, d, e, policy),
        RefinementNeed::RobustFallback,
    )
}

fn add<S: BorrowedPredicateScalar>(left: &S, right: &S) -> S {
    left.add_ref(right)
}

fn sub<S: BorrowedPredicateScalar>(left: &S, right: &S) -> S {
    left.sub_ref(right)
}

fn mul<S: BorrowedPredicateScalar>(left: &S, right: &S) -> S {
    left.mul_ref(right)
}

fn exact_orient2d<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
) -> Option<Sign> {
    crate::trace_dispatch!("hyperlimit", "exact_orient2d", "unimplemented");
    None
}

fn exact_orient3d<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
) -> Option<Sign> {
    crate::trace_dispatch!("hyperlimit", "exact_orient3d", "unimplemented");
    None
}

fn exact_incircle2d<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
    _d: &Point2<S>,
) -> Option<Sign> {
    crate::trace_dispatch!("hyperlimit", "exact_incircle2d", "unimplemented");
    None
}

fn exact_insphere3d<S: PredicateScalar>(
    _a: &Point3<S>,
    _b: &Point3<S>,
    _c: &Point3<S>,
    _d: &Point3<S>,
    _e: &Point3<S>,
) -> Option<Sign> {
    crate::trace_dispatch!("hyperlimit", "exact_insphere3d", "unimplemented");
    None
}

#[cfg(feature = "robust")]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "disabled");
        return None;
    }
    #[cfg(feature = "geogram")]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "geogram");
        return crate::backend::geogram::orient2d(a, b, c);
    }

    #[cfg(not(feature = "geogram"))]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "robust");
        return crate::backend::robust::orient2d(a, b, c);
    }
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "disabled");
        return None;
    }
    crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "geogram");
    crate::backend::geogram::orient2d(a, b, c)
}

#[cfg(not(feature = "robust"))]
#[cfg(not(feature = "geogram"))]
fn fallback_orient2d_if_allowed<S: PredicateScalar>(
    _a: &Point2<S>,
    _b: &Point2<S>,
    _c: &Point2<S>,
    _policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "fallback_orient2d", "no-backend");
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
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "disabled");
        return None;
    }
    #[cfg(feature = "geogram")]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "geogram");
        return crate::backend::geogram::orient3d(a, b, c, d);
    }

    #[cfg(not(feature = "geogram"))]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "robust");
        return crate::backend::robust::orient3d(a, b, c, d);
    }
}

#[cfg(feature = "robust")]
fn fallback_incircle2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "disabled");
        return None;
    }
    #[cfg(feature = "geogram")]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "geogram");
        return crate::backend::geogram::incircle2d(a, b, c, d);
    }

    #[cfg(not(feature = "geogram"))]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "robust");
        return crate::backend::robust::incircle2d(a, b, c, d);
    }
}

#[cfg(all(feature = "geogram", not(feature = "robust")))]
fn fallback_incircle2d_if_allowed<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "disabled");
        return None;
    }
    crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "geogram");
    crate::backend::geogram::incircle2d(a, b, c, d)
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
    crate::trace_dispatch!("hyperlimit", "fallback_incircle2d", "no-backend");
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
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "disabled");
        return None;
    }
    #[cfg(feature = "geogram")]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "geogram");
        return crate::backend::geogram::insphere3d(a, b, c, d, e);
    }

    #[cfg(not(feature = "geogram"))]
    {
        crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "robust");
        return crate::backend::robust::insphere3d(a, b, c, d, e);
    }
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
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "disabled");
        return None;
    }
    crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "geogram");
    crate::backend::geogram::insphere3d(a, b, c, d, e)
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
    crate::trace_dispatch!("hyperlimit", "fallback_insphere3d", "no-backend");
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
    if !policy.allow_robust_fallback {
        crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "disabled");
        return None;
    }
    crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "geogram");
    crate::backend::geogram::orient3d(a, b, c, d)
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
    crate::trace_dispatch!("hyperlimit", "fallback_orient3d", "no-backend");
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
    sign_filter_outcome(det, scale, 8.0)
}

fn orient2d_point_filter<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let ax = a.x.to_f64()?;
    let ay = a.y.to_f64()?;
    let bx = b.x.to_f64()?;
    let by = b.y.to_f64()?;
    let cx = c.x.to_f64()?;
    let cy = c.y.to_f64()?;

    let abx = bx - ax;
    let aby = by - ay;
    let acx = cx - ax;
    let acy = cy - ay;
    orient2d_float_filter(abx, aby, acx, acy)
}

fn orient2d_float_filter(abx: f64, aby: f64, acx: f64, acy: f64) -> Option<PredicateOutcome<Sign>> {
    let det = abx.mul_add(acy, -(aby * acx));
    let scale = abx.abs() * acy.abs() + aby.abs() * acx.abs();
    sign_filter_outcome(det, scale, 8.0)
}

fn sign_filter_outcome(
    det: f64,
    scale: f64,
    epsilon_multiplier: f64,
) -> Option<PredicateOutcome<Sign>> {
    // Float filters only return when the error bound proves the sign.  Unknown cases still
    // escalate to exact scalar arithmetic, so this is a performance shortcut, not a semantic
    // weakening.
    match det_sign_filter(det, scale, epsilon_multiplier) {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("hyperlimit", "sign_filter_outcome", "decided");
            Some(PredicateOutcome::decided(
                sign,
                certainty,
                Escalation::Filter,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("hyperlimit", "sign_filter_outcome", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("hyperlimit", "sign_filter_outcome", "unknown");
            None
        }
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

    orient3d_float_filter(adx, ady, adz, bdx, bdy, bdz, cdx, cdy, cdz)
}

#[allow(clippy::too_many_arguments)]
fn orient3d_point_filter<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let ax = a.x.to_f64()?;
    let ay = a.y.to_f64()?;
    let az = a.z.to_f64()?;
    let bx = b.x.to_f64()?;
    let by = b.y.to_f64()?;
    let bz = b.z.to_f64()?;
    let cx = c.x.to_f64()?;
    let cy = c.y.to_f64()?;
    let cz = c.z.to_f64()?;
    let dx = d.x.to_f64()?;
    let dy = d.y.to_f64()?;
    let dz = d.z.to_f64()?;

    orient3d_float_filter(
        ax - dx,
        ay - dy,
        az - dz,
        bx - dx,
        by - dy,
        bz - dz,
        cx - dx,
        cy - dy,
        cz - dz,
    )
}

#[allow(clippy::too_many_arguments)]
fn orient3d_float_filter(
    adx: f64,
    ady: f64,
    adz: f64,
    bdx: f64,
    bdy: f64,
    bdz: f64,
    cdx: f64,
    cdy: f64,
    cdz: f64,
) -> Option<PredicateOutcome<Sign>> {
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

    sign_filter_outcome(det, scale, 32.0)
}

fn incircle2d_point_filter<S: PredicateScalar>(
    a: &Point2<S>,
    b: &Point2<S>,
    c: &Point2<S>,
    d: &Point2<S>,
) -> Option<PredicateOutcome<Sign>> {
    let ax = a.x.to_f64()?;
    let ay = a.y.to_f64()?;
    let bx = b.x.to_f64()?;
    let by = b.y.to_f64()?;
    let cx = c.x.to_f64()?;
    let cy = c.y.to_f64()?;
    let dx = d.x.to_f64()?;
    let dy = d.y.to_f64()?;

    let adx = ax - dx;
    let ady = ay - dy;
    let bdx = bx - dx;
    let bdy = by - dy;
    let cdx = cx - dx;
    let cdy = cy - dy;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    let bdx_cdy = bdx * cdy;
    let cdx_bdy = cdx * bdy;
    let cdx_ady = cdx * ady;
    let adx_cdy = adx * cdy;
    let adx_bdy = adx * bdy;
    let bdx_ady = bdx * ady;

    let term_a = alift * (bdx_cdy - cdx_bdy);
    let term_b = blift * (cdx_ady - adx_cdy);
    let term_c = clift * (adx_bdy - bdx_ady);
    let det = term_a + term_b + term_c;
    let scale = alift.abs() * (bdx_cdy.abs() + cdx_bdy.abs())
        + blift.abs() * (cdx_ady.abs() + adx_cdy.abs())
        + clift.abs() * (adx_bdy.abs() + bdx_ady.abs());

    sign_filter_outcome(det, scale, 64.0)
}

fn insphere3d_point_filter<S: PredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    d: &Point3<S>,
    e: &Point3<S>,
) -> Option<PredicateOutcome<Sign>> {
    let ax = a.x.to_f64()?;
    let ay = a.y.to_f64()?;
    let az = a.z.to_f64()?;
    let bx = b.x.to_f64()?;
    let by = b.y.to_f64()?;
    let bz = b.z.to_f64()?;
    let cx = c.x.to_f64()?;
    let cy = c.y.to_f64()?;
    let cz = c.z.to_f64()?;
    let dx = d.x.to_f64()?;
    let dy = d.y.to_f64()?;
    let dz = d.z.to_f64()?;
    let ex = e.x.to_f64()?;
    let ey = e.y.to_f64()?;
    let ez = e.z.to_f64()?;

    let aex = ax - ex;
    let bex = bx - ex;
    let cex = cx - ex;
    let dex = dx - ex;
    let aey = ay - ey;
    let bey = by - ey;
    let cey = cy - ey;
    let dey = dy - ey;
    let aez = az - ez;
    let bez = bz - ez;
    let cez = cz - ez;
    let dez = dz - ez;

    let ab = aex * bey - bex * aey;
    let bc = bex * cey - cex * bey;
    let cd = cex * dey - dex * cey;
    let da = dex * aey - aex * dey;
    let ac = aex * cey - cex * aey;
    let bd = bex * dey - dex * bey;

    let abc = aez * bc - bez * ac + cez * ab;
    let bcd = bez * cd - cez * bd + dez * bc;
    let cda = cez * da + dez * ac + aez * cd;
    let dab = dez * ab + aez * bd + bez * da;

    let alift = aex * aex + aey * aey + aez * aez;
    let blift = bex * bex + bey * bey + bez * bez;
    let clift = cex * cex + cey * cey + cez * cez;
    let dlift = dex * dex + dey * dey + dez * dez;

    let left = dlift * abc + blift * cda;
    let right = clift * dab + alift * bcd;
    let det = left - right;
    let scale = dlift.abs() * abc.abs()
        + blift.abs() * cda.abs()
        + clift.abs() * dab.abs()
        + alift.abs() * bcd.abs();

    sign_filter_outcome(det, scale, 256.0)
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;
    use core::ops::{Add, Mul, Sub};
    use std::rc::Rc;

    use super::*;
    use crate::predicate::Certainty;
    use crate::scalar::{MagnitudeBounds, ScalarFacts, StructuralScalar};

    #[derive(Debug)]
    struct CloneCountingScalar {
        value: f64,
        clones: Rc<Cell<usize>>,
        ops: Rc<Cell<usize>>,
    }

    impl CloneCountingScalar {
        fn new(value: f64, clones: Rc<Cell<usize>>) -> Self {
            Self {
                value,
                clones,
                ops: Rc::new(Cell::new(0)),
            }
        }

        fn with_ops(value: f64, ops: Rc<Cell<usize>>) -> Self {
            Self {
                value,
                clones: Rc::new(Cell::new(0)),
                ops,
            }
        }

        fn derived_from(value: f64, source: &Self) -> Self {
            source.ops.set(source.ops.get() + 1);
            Self {
                value,
                clones: Rc::clone(&source.clones),
                ops: Rc::clone(&source.ops),
            }
        }
    }

    impl Clone for CloneCountingScalar {
        fn clone(&self) -> Self {
            self.clones.set(self.clones.get() + 1);
            Self {
                value: self.value,
                clones: Rc::clone(&self.clones),
                ops: Rc::clone(&self.ops),
            }
        }
    }

    impl StructuralScalar for CloneCountingScalar {
        fn scalar_facts(&self) -> ScalarFacts {
            ScalarFacts {
                sign: None,
                exact_zero: None,
                provably_nonzero: None,
                exact: Some(false),
                rational_only: Some(false),
                magnitude: Some(MagnitudeBounds::exact(self.value.abs())),
            }
        }
    }

    impl crate::scalar::PredicateScalar for CloneCountingScalar {
        fn to_f64(&self) -> Option<f64> {
            Some(self.value)
        }

        fn prefer_f64_filter_before_arithmetic() -> bool {
            true
        }
    }

    impl Add for CloneCountingScalar {
        type Output = Self;

        fn add(self, rhs: Self) -> Self::Output {
            Self::derived_from(self.value + rhs.value, &self)
        }
    }

    impl Sub for CloneCountingScalar {
        type Output = Self;

        fn sub(self, rhs: Self) -> Self::Output {
            Self::derived_from(self.value - rhs.value, &self)
        }
    }

    impl Mul for CloneCountingScalar {
        type Output = Self;

        fn mul(self, rhs: Self) -> Self::Output {
            Self::derived_from(self.value * rhs.value, &self)
        }
    }

    impl<'b> Add<&'b CloneCountingScalar> for &CloneCountingScalar {
        type Output = CloneCountingScalar;

        fn add(self, rhs: &'b CloneCountingScalar) -> Self::Output {
            CloneCountingScalar::derived_from(self.value + rhs.value, self)
        }
    }

    impl<'b> Sub<&'b CloneCountingScalar> for &CloneCountingScalar {
        type Output = CloneCountingScalar;

        fn sub(self, rhs: &'b CloneCountingScalar) -> Self::Output {
            CloneCountingScalar::derived_from(self.value - rhs.value, self)
        }
    }

    impl<'b> Mul<&'b CloneCountingScalar> for &CloneCountingScalar {
        type Output = CloneCountingScalar;

        fn mul(self, rhs: &'b CloneCountingScalar) -> Self::Output {
            CloneCountingScalar::derived_from(self.value * rhs.value, self)
        }
    }

    #[test]
    fn orient2d_classifies_simple_triangle() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        let c = Point2::new(0.0, 1.0);

        assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));
    }

    #[test]
    fn orient2d_uses_borrowed_arithmetic_without_cloning_scalars() {
        let clones = Rc::new(Cell::new(0));
        let scalar = |value| CloneCountingScalar::new(value, Rc::clone(&clones));
        let a = Point2::new(scalar(0.0), scalar(0.0));
        let b = Point2::new(scalar(1.0), scalar(0.0));
        let c = Point2::new(scalar(0.0), scalar(1.0));

        assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));
        assert_eq!(clones.get(), 0);
    }

    #[test]
    fn orient2d_easy_case_uses_float_filter_before_scalar_arithmetic() {
        let ops = Rc::new(Cell::new(0));
        let scalar = |value| CloneCountingScalar::with_ops(value, Rc::clone(&ops));
        let a = Point2::new(scalar(0.0), scalar(0.0));
        let b = Point2::new(scalar(1.0), scalar(0.0));
        let c = Point2::new(scalar(0.0), scalar(1.0));

        assert_eq!(
            orient2d(&a, &b, &c),
            PredicateOutcome::decided(Sign::Positive, Certainty::Filtered, Escalation::Filter)
        );
        assert_eq!(ops.get(), 0);
    }

    #[test]
    fn incircle2d_easy_case_uses_float_filter_before_scalar_arithmetic() {
        let ops = Rc::new(Cell::new(0));
        let scalar = |value| CloneCountingScalar::with_ops(value, Rc::clone(&ops));
        let a = Point2::new(scalar(0.0), scalar(0.0));
        let b = Point2::new(scalar(1.0), scalar(0.0));
        let c = Point2::new(scalar(0.0), scalar(1.0));
        let d = Point2::new(scalar(0.25), scalar(0.25));

        assert_eq!(
            incircle2d(&a, &b, &c, &d),
            PredicateOutcome::decided(Sign::Positive, Certainty::Filtered, Escalation::Filter)
        );
        assert_eq!(ops.get(), 0);
    }

    #[test]
    fn insphere3d_easy_case_uses_float_filter_before_scalar_arithmetic() {
        let ops = Rc::new(Cell::new(0));
        let scalar = |value| CloneCountingScalar::with_ops(value, Rc::clone(&ops));
        let a = Point3::new(scalar(1.0), scalar(0.0), scalar(0.0));
        let b = Point3::new(scalar(0.0), scalar(1.0), scalar(0.0));
        let c = Point3::new(scalar(0.0), scalar(0.0), scalar(1.0));
        let d = Point3::new(scalar(-1.0), scalar(0.0), scalar(0.0));
        let e = Point3::new(scalar(0.0), scalar(0.0), scalar(0.0));

        assert_eq!(
            insphere3d(&a, &b, &c, &d, &e),
            PredicateOutcome::decided(Sign::Positive, Certainty::Filtered, Escalation::Filter)
        );
        assert_eq!(ops.get(), 0);
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
            PredicateOutcome::decided(
                Sign::Zero,
                Certainty::RobustFloat,
                Escalation::RobustFallback
            )
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

    #[cfg(feature = "hyperlattice")]
    #[test]
    fn hyperlattice_exact_rational_zero_decides_structurally() {
        type Scalar = hyperlattice::Scalar<hyperlattice::DefaultBackend>;

        let a: Point2<Scalar> =
            Point2::new(hyperlattice::Scalar::from(0), hyperlattice::Scalar::from(0));
        let b: Point2<Scalar> =
            Point2::new(hyperlattice::Scalar::from(1), hyperlattice::Scalar::from(1));
        let c: Point2<Scalar> =
            Point2::new(hyperlattice::Scalar::from(2), hyperlattice::Scalar::from(2));

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
