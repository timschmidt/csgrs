//! Torus cell enclosures are the exact range of the field over the cell, so
//! cells arbitrarily close to the surface are still certified.
use hyperlimit::Point3;
use hyperreal::{Rational, Real};
use hypersdf::{Sdf, SdfCellLocation, SdfCoordinate, SdfExpr};

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn point(x: &Real, y: &Real, z: &Real) -> Point3 {
    Point3::new(x.clone(), y.clone(), z.clone())
}

/// Major radius 4 and minor radius 1 about the z axis through the origin:
/// the outer equator is at radial distance 5, the tube centre circle at 4.
fn torus() -> Sdf {
    Sdf::new(SdfExpr::torus(
        SdfCoordinate::Z,
        point(&Real::zero(), &Real::zero(), &Real::zero()),
        Real::from(16),
        Real::from(1),
    ))
}

/// The field (u + a + c)^2 - 4 R^2 u at a point.
fn field(x: &Real, y: &Real, z: &Real) -> Real {
    let u = &(x * x) + &(y * y);
    let s = &(&u + &(z * z)) + &Real::from(15);
    &(&s * &s) - &(&Real::from(64) * &u)
}

#[test]
fn cells_just_outside_the_outer_equator_are_certified_outside() {
    let field_sdf = torus();
    for k in 1..=40_u32 {
        // A cube of side g at distance g beyond the surface.
        let g = q(1, 1_u64 << k.min(62));
        let x0 = &Real::from(5) + &g;
        let x1 = &x0 + &g;
        let cell = field_sdf.classify_cell(
            &point(&x0, &Real::zero(), &Real::zero()),
            &point(&x1, &g, &g),
        );
        assert_eq!(
            cell.location,
            SdfCellLocation::ConservativeOutside,
            "gap 2^-{k}"
        );
    }
}

#[test]
fn cells_inside_the_tube_are_certified_inside_and_straddling_cells_are_boundary() {
    let field_sdf = torus();
    let h = q(1, 4);
    let lo = &Real::from(4) - &h;
    let hi = &Real::from(4) + &h;
    let inside = field_sdf.classify_cell(&point(&lo, &-&h, &-&h), &point(&hi, &h, &h));
    assert_eq!(inside.location, SdfCellLocation::ConservativeInside);
    let straddle = field_sdf.classify_cell(
        &point(&q(9, 2), &Real::zero(), &Real::zero()),
        &point(&q(11, 2), &h, &h),
    );
    assert_eq!(straddle.location, SdfCellLocation::Boundary);
}

#[test]
fn the_interval_is_the_exact_range_over_the_cell() {
    // Cell [3, 6] x [0, 1] x [0, 1]: u = x^2 + y^2 spans [9, 37] and a = z^2
    // spans [0, 1]. F(u, 0) = (u + 15)^2 - 64 u is least at u = 17 (F = -64);
    // the largest corner value is F(37, 1) = 53^2 - 64 * 37 = 441.
    let field_sdf = torus();
    let min = point(&Real::from(3), &Real::zero(), &Real::zero());
    let max = point(&Real::from(6), &Real::from(1), &Real::from(1));
    let interval = field_sdf.interval_cell(&min, &max).interval.unwrap();
    assert!(
        (&interval.lower - &Real::from(-64)).definitely_zero(),
        "{:?}",
        interval.lower
    );
    assert!(
        (&interval.upper - &Real::from(441)).definitely_zero(),
        "{:?}",
        interval.upper
    );
    // Both are attained: at (sqrt(17), 0, 0) and at the corner (6, 1, 1).
    assert!(
        (&field(&Real::from(6), &Real::from(1), &Real::from(1)) - &Real::from(441))
            .definitely_zero()
    );
}
