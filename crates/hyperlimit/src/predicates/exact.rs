//! Exact determinant kernels for low-dimensional predicates.
//!
//! This module is deliberately private to `hyperlimit`: callers should consume
//! predicate outcomes, not determinant implementation details. Keeping the
//! exact schedules here leaves room for integer-grid and dyadic kernels without
//! leaking those representations across crate boundaries.

use crate::geometry::{Point2, Point3};
use crate::predicate::Sign;
use core::cmp::Ordering;
use hyperreal::{Rational, Real};

/// Try the exact rational 2D orientation kernel.
pub(super) fn orient2d(a: &Point2, b: &Point2, c: &Point2) -> Option<Sign> {
    orient2d_coordinates([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y])
}

/// Try the exact rational 2D orientation kernel over borrowed coordinates.
#[inline]
pub(super) fn orient2d_coordinates(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Option<Sign> {
    let [ax, ay] = a.map(exact_rational_ref);
    let [bx, by] = b.map(exact_rational_ref);
    let [cx, cy] = c.map(exact_rational_ref);
    let (ax, ay, bx, by, cx, cy) = (ax?, ay?, bx?, by?, cx?, cy?);

    crate::trace_dispatch!("hyperlimit", "exact_orient2d", "rational-det2");

    // Evaluate the fixed 2x2 orientation polynomial as a signed product-sum
    // before constructing a generic `Real` expression tree. The scalar reducer
    // can then use delayed reduction and shared-denominator or dyadic schedules
    // without leaking scalar representation into the predicate layer.
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

    // The predicate consumes only the determinant sign. Preserve the complete
    // six-term polynomial for the scalar scheduler, but compare its signed
    // magnitudes directly instead of materializing and reducing a dead
    // rational result.
    let adx = ax - dx;
    let ady = ay - dy;
    let adz = az - dz;
    let bdx = bx - dx;
    let bdy = by - dy;
    let bdz = bz - dz;
    let cdx = cx - dx;
    let cdy = cy - dy;
    let cdz = cz - dz;

    let ordering = Rational::signed_product_sum_ordering(
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
    Some(sign_from_ordering(ordering))
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

    // Evaluate the translated in-circle determinant as one six-term exact
    // rational product-sum. Keeping the schedule intact lets the scalar layer
    // delay reduction when denominators share structure.
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

    // Select the lifted in-sphere determinant schedule from object facts and
    // evaluate it with exact rationals before generic `Real` construction.
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

    // Evaluate the lifted in-sphere cofactors through fixed product sums so
    // denominator sharing remains visible to `hyperreal::Rational` instead of
    // being hidden behind already-reduced pairwise products.
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

fn sign_from_ordering(ordering: Ordering) -> Sign {
    match ordering {
        Ordering::Less => Sign::Negative,
        Ordering::Equal => Sign::Zero,
        Ordering::Greater => Sign::Positive,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn exact_kernel_entry_points_cover_signs_and_decline_symbolic_inputs() {
        assert_eq!(
            orient2d(&p2(0, 0), &p2(1, 0), &p2(0, 1)),
            Some(Sign::Positive)
        );
        assert_eq!(
            orient2d(&p2(0, 0), &p2(0, 1), &p2(1, 0)),
            Some(Sign::Negative)
        );
        assert_eq!(orient2d(&p2(0, 0), &p2(1, 1), &p2(2, 2)), Some(Sign::Zero));

        assert_eq!(
            orient3d(&p3(0, 0, 0), &p3(1, 0, 0), &p3(0, 1, 0), &p3(0, 0, 1)),
            Some(Sign::Negative)
        );
        assert_eq!(
            orient3d(&p3(0, 0, 0), &p3(1, 0, 0), &p3(0, 1, 0), &p3(1, 1, 0)),
            Some(Sign::Zero)
        );
        assert_eq!(
            incircle2d(&p2(0, 0), &p2(2, 0), &p2(0, 2), &p2(1, 1)),
            Some(Sign::Positive)
        );
        assert_eq!(
            insphere3d(
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &p3(0, 1, 0),
                &p3(0, 0, 1),
                &p3(0, 0, 0),
            ),
            Some(Sign::Zero)
        );

        let symbolic = Point2::new(Real::pi(), Real::from(0));
        assert_eq!(orient2d(&symbolic, &p2(1, 0), &p2(0, 1)), None);
    }
}
