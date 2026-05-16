//! Exact determinant kernels for low-dimensional predicates.
//!
//! This module is deliberately private to `hyperlimit`: callers should consume
//! predicate outcomes, not determinant implementation details. Keeping the
//! exact schedules here leaves room for future integer-grid, dyadic, and
//! shared-scale kernels without leaking those representations across crate
//! boundaries.

use crate::geometry::{Point2, Point3};
use crate::predicate::Sign;
use hyperreal::{Rational, Real, RealSign};

/// Try the exact 2D orientation kernel through borrowed point-local
/// shared-scale views.
///
/// This route consumes [`PointSharedScaleView`](crate::geometry::PointSharedScaleView)
/// certificates before falling back to coordinate-by-coordinate rational
/// extraction. The determinant is evaluated as one six-term known-exact
/// product-sum in `hyperreal`, so the scalar layer still owns denominator
/// storage and final reduction. This is the first predicate family wired
/// through borrowed common-scale object facts, following Yap's rule to
/// preserve geometric object structure before scalar expansion; see Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). The fused exact product-sum follows Bareiss-style delayed
/// normalization; see Bareiss, "Sylvester's Identity and Multistep
/// Integer-Preserving Gaussian Elimination," *Mathematics of Computation*
/// 22.103 (1968).
pub(super) fn orient2d_shared_scale(a: &Point2, b: &Point2, c: &Point2) -> Option<Sign> {
    let a = a.shared_scale_view()?;
    let b = b.shared_scale_view()?;
    let c = c.shared_scale_view()?;
    let [ax, ay] = a.coordinates();
    let [bx, by] = b.coordinates();
    let [cx, cy] = c.coordinates();

    crate::trace_dispatch!("hyperlimit", "exact_orient2d", "shared-scale-view-det2");

    let determinant = Real::exact_rational_signed_product_sum_known_exact(
        [true, true, true, false, false, false],
        [[ax, by], [bx, cy], [cx, ay], [ay, bx], [by, cx], [cy, ax]],
    );
    sign_from_real(&determinant)
}

/// Try the exact rational 2D orientation kernel.
pub(super) fn orient2d(a: &Point2, b: &Point2, c: &Point2) -> Option<Sign> {
    let ax = exact_rational_ref(&a.x)?;
    let ay = exact_rational_ref(&a.y)?;
    let bx = exact_rational_ref(&b.x)?;
    let by = exact_rational_ref(&b.y)?;
    let cx = exact_rational_ref(&c.x)?;
    let cy = exact_rational_ref(&c.y)?;

    crate::trace_dispatch!("hyperlimit", "exact_orient2d", "rational-det2");

    // Exact determinant shortcut: evaluate the fixed 2x2 orientation
    // polynomial as a signed product-sum before constructing a generic `Real`
    // expression tree. Yap's EGC model explicitly separates geometric object
    // structure from scalar expansion; see Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7.1-2 (1997). The scalar reducer
    // uses Bareiss-style delayed reduction and shared-denominator/dyadic
    // schedules; see Bareiss, "Sylvester's Identity and Multistep Integer-
    // Preserving Gaussian Elimination," Mathematics of Computation 22.103
    // (1968). Keeping that reducer in `hyperreal` preserves the predicate
    // abstraction boundary while still giving this determinant a common-scale
    // route.
    let abx = bx - ax;
    let aby = by - ay;
    let acx = cx - ax;
    let acy = cy - ay;
    let det = Rational::signed_product_sum([true, false], [[&abx, &acy], [&aby, &acx]]);
    Some(sign_from_rational(&det))
}

/// Try the exact rational 3D orientation kernel.
pub(super) fn orient3d(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Option<Sign> {
    let ax = exact_rational_ref(&a.x)?;
    let ay = exact_rational_ref(&a.y)?;
    let az = exact_rational_ref(&a.z)?;
    let bx = exact_rational_ref(&b.x)?;
    let by = exact_rational_ref(&b.y)?;
    let bz = exact_rational_ref(&b.z)?;
    let cx = exact_rational_ref(&c.x)?;
    let cy = exact_rational_ref(&c.y)?;
    let cz = exact_rational_ref(&c.z)?;
    let dx = exact_rational_ref(&d.x)?;
    let dy = exact_rational_ref(&d.y)?;
    let dz = exact_rational_ref(&d.z)?;

    crate::trace_dispatch!("hyperlimit", "exact_orient3d", "rational-det3");

    // The translated 3D orientation determinant is evaluated as a fixed
    // six-term signed product-sum. This keeps the determinant as a known
    // geometric object until the final sign decision, rather than expanding it
    // through generic symbolic `Real` nodes. See Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7.1-2 (1997). The scalar reducer
    // then chooses dyadic, equal-denominator, or LCM schedules following the
    // delayed-reduction strategy of Bareiss, "Sylvester's Identity and
    // Multistep Integer-Preserving Gaussian Elimination," Mathematics of
    // Computation 22.103 (1968).
    let adx = ax - dx;
    let ady = ay - dy;
    let adz = az - dz;
    let bdx = bx - dx;
    let bdy = by - dy;
    let bdz = bz - dz;
    let cdx = cx - dx;
    let cdy = cy - dy;
    let cdz = cz - dz;

    let det = Rational::signed_product_sum(
        [true, false, true, false, true, false],
        [
            [&adx, &bdy, &cdz],
            [&adx, &bdz, &cdy],
            [&ady, &bdz, &cdx],
            [&ady, &bdx, &cdz],
            [&adz, &bdx, &cdy],
            [&adz, &bdy, &cdx],
        ],
    );
    Some(sign_from_rational(&det))
}

/// Try the exact rational in-circle kernel.
pub(super) fn incircle2d(a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> Option<Sign> {
    let ax = exact_rational_ref(&a.x)?;
    let ay = exact_rational_ref(&a.y)?;
    let bx = exact_rational_ref(&b.x)?;
    let by = exact_rational_ref(&b.y)?;
    let cx = exact_rational_ref(&c.x)?;
    let cy = exact_rational_ref(&c.y)?;
    let dx = exact_rational_ref(&d.x)?;
    let dy = exact_rational_ref(&d.y)?;

    crate::trace_dispatch!("hyperlimit", "exact_incircle2d", "rational-det3-lifted");

    // Exact in-circle determinant over translated rational coordinates. The
    // six product terms match the robust-predicate schedule in Shewchuk,
    // "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
    // Predicates," Discrete & Computational Geometry 18.3 (1997), but the
    // implementation is exact-rational and policy-visible rather than an
    // implicit primitive-float filter. Passing the full signed product-sum to
    // `hyperreal::Rational` preserves Yap's object-shape boundary and lets the
    // scalar layer apply Bareiss-style delayed reduction when denominators
    // share structure.
    let adx = ax - dx;
    let ady = ay - dy;
    let bdx = bx - dx;
    let bdy = by - dy;
    let cdx = cx - dx;
    let cdy = cy - dy;

    let alift = rational_lift2(&adx, &ady);
    let blift = rational_lift2(&bdx, &bdy);
    let clift = rational_lift2(&cdx, &cdy);
    let det = Rational::signed_product_sum(
        [true, false, true, false, true, false],
        [
            [&alift, &bdx, &cdy],
            [&alift, &cdx, &bdy],
            [&blift, &cdx, &ady],
            [&blift, &adx, &cdy],
            [&clift, &adx, &bdy],
            [&clift, &bdx, &ady],
        ],
    );
    Some(sign_from_rational(&det))
}

/// Try the exact rational in-sphere kernel.
pub(super) fn insphere3d(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
) -> Option<Sign> {
    let ax = exact_rational_ref(&a.x)?;
    let ay = exact_rational_ref(&a.y)?;
    let az = exact_rational_ref(&a.z)?;
    let bx = exact_rational_ref(&b.x)?;
    let by = exact_rational_ref(&b.y)?;
    let bz = exact_rational_ref(&b.z)?;
    let cx = exact_rational_ref(&c.x)?;
    let cy = exact_rational_ref(&c.y)?;
    let cz = exact_rational_ref(&c.z)?;
    let dx = exact_rational_ref(&d.x)?;
    let dy = exact_rational_ref(&d.y)?;
    let dz = exact_rational_ref(&d.z)?;
    let ex = exact_rational_ref(&e.x)?;
    let ey = exact_rational_ref(&e.y)?;
    let ez = exact_rational_ref(&e.z)?;

    crate::trace_dispatch!("hyperlimit", "exact_insphere3d", "rational-det4-lifted");

    // The lifted in-sphere determinant is the rational analogue of the
    // adaptive robust predicate described by Shewchuk, "Adaptive Precision
    // Floating-Point Arithmetic and Fast Robust Geometric Predicates,"
    // Discrete & Computational Geometry 18.3 (1997). We use exact rational
    // arithmetic instead of floating expansions, preserving Yap's EGC boundary:
    // the determinant schedule is selected from object facts before generic
    // `Real` expression construction.
    let aex = ax - ex;
    let aey = ay - ey;
    let aez = az - ez;
    let bex = bx - ex;
    let bey = by - ey;
    let bez = bz - ez;
    let cex = cx - ex;
    let cey = cy - ey;
    let cez = cz - ez;
    let dex = dx - ex;
    let dey = dy - ey;
    let dez = dz - ez;

    // These 2x2 and 3x3 minors are the lifted in-sphere cofactors used by
    // Shewchuk's robust predicate schedule. Evaluating each minor through the
    // fixed product-sum reducer keeps denominator sharing visible to
    // `hyperreal::Rational`, instead of hiding it behind already-reduced
    // pairwise products. The reduction policy remains scalar-owned, preserving
    // Yap's EGC abstraction split.
    let ab = rational_det2(&aex, &bey, &bex, &aey);
    let bc = rational_det2(&bex, &cey, &cex, &bey);
    let cd = rational_det2(&cex, &dey, &dex, &cey);
    let da = rational_det2(&dex, &aey, &aex, &dey);
    let ac = rational_det2(&aex, &cey, &cex, &aey);
    let bd = rational_det2(&bex, &dey, &dex, &bey);

    let abc =
        Rational::signed_product_sum([true, false, true], [[&aez, &bc], [&bez, &ac], [&cez, &ab]]);
    let bcd =
        Rational::signed_product_sum([true, false, true], [[&bez, &cd], [&cez, &bd], [&dez, &bc]]);
    let cda =
        Rational::signed_product_sum([true, true, true], [[&cez, &da], [&dez, &ac], [&aez, &cd]]);
    let dab =
        Rational::signed_product_sum([true, true, true], [[&dez, &ab], [&aez, &bd], [&bez, &da]]);

    let alift = rational_lift3(&aex, &aey, &aez);
    let blift = rational_lift3(&bex, &bey, &bez);
    let clift = rational_lift3(&cex, &cey, &cez);
    let dlift = rational_lift3(&dex, &dey, &dez);

    let det = Rational::signed_product_sum(
        [true, true, false, false],
        [
            [&dlift, &abc],
            [&blift, &cda],
            [&clift, &dab],
            [&alift, &bcd],
        ],
    );
    Some(sign_from_rational(&det))
}

fn exact_rational_ref(value: &Real) -> Option<&Rational> {
    value.exact_rational_ref()
}

fn rational_lift2(x: &Rational, y: &Rational) -> Rational {
    &(x * x) + &(y * y)
}

fn rational_lift3(x: &Rational, y: &Rational, z: &Rational) -> Rational {
    &(&(x * x) + &(y * y)) + &(z * z)
}

fn rational_det2(a: &Rational, b: &Rational, c: &Rational, d: &Rational) -> Rational {
    Rational::signed_product_sum([true, false], [[a, b], [c, d]])
}

fn sign_from_rational(value: &Rational) -> Sign {
    let zero = Rational::zero();
    if value < &zero {
        Sign::Negative
    } else if value > &zero {
        Sign::Positive
    } else {
        Sign::Zero
    }
}

fn sign_from_real(value: &Real) -> Option<Sign> {
    match value.structural_facts().sign? {
        RealSign::Negative => Some(Sign::Negative),
        RealSign::Zero => Some(Sign::Zero),
        RealSign::Positive => Some(Sign::Positive),
    }
}
