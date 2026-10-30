//! Repeated offsets of a convex region bounded by a Pythagorean-hodograph
//! cubic and its chord. Disk offsets of a convex set obey exact identities:
//! dilations compose, closing returns the set, and a dilation followed by a
//! smaller erosion is the difference dilation. The dilated boundary is also
//! rebuilt independently from the materialized rational PH offset.
mod support;

use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurvePath2, CurveRegion2,
    FillRule, LineSeg2, OffsetCornerStyle2, Point2, PredicatePolicy, Real,
};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

fn fraction(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

/// P(t) = (t - t^3/3, t^2) on [0, 1], with P'(t) = (1 - t^2, 2t) and
/// |P'(t)| = 1 + t^2. It turns left everywhere, so with the chord back to the
/// origin it bounds a convex region traversed counterclockwise.
fn ph_cubic() -> CubicBezier2 {
    CubicBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(fraction(1, 3), Real::zero()),
        Point2::new(fraction(2, 3), fraction(1, 3)),
        Point2::new(fraction(2, 3), Real::one()),
    )
}

fn line(start: &Point2, end: &Point2) -> Curve2 {
    Curve2::from(LineSeg2::try_new(start.clone(), end.clone()).unwrap())
}

fn region(paths: &[CurvePath2]) -> CurveRegion2 {
    support::under(&STRICT, || {
        CurveRegion2::try_from_boundary_paths(paths, FillRule::NonZero)
    })
    .unwrap()
    .into_value()
}

fn ph_region() -> CurveRegion2 {
    let end = Point2::new(fraction(2, 3), Real::one());
    region(&[CurvePath2::try_new(vec![
        Curve2::from(ph_cubic()),
        line(&end, &Point2::new(Real::zero(), Real::zero())),
    ])
    .unwrap()])
}

fn offset(region: &CurveRegion2, distance: Real) -> CurveRegion2 {
    support::under(&STRICT, || {
        region.offset(distance, &OffsetCornerStyle2::Round)
    })
    .unwrap()
    .into_value()
}

fn assert_same_set(label: &str, actual: &CurveRegion2, expected: &CurveRegion2) {
    let difference = support::under(&STRICT, || actual.boolean_region(expected, BooleanOp::Xor))
        .unwrap()
        .into_value();
    assert!(difference.is_empty(), "{label}: the regions differ");
}

#[test]
fn ph_region_offsets_obey_convex_disk_identities() {
    let source = ph_region();
    let eighth = offset(&source, fraction(1, 8));
    let quarter = offset(&source, fraction(1, 4));
    assert_same_set(
        "dilations compose",
        &offset(&eighth, fraction(1, 8)),
        &quarter,
    );
    assert_same_set("closing", &offset(&eighth, -fraction(1, 8)), &source);
    assert_same_set(
        "dilation then smaller erosion",
        &offset(&quarter, -fraction(1, 8)),
        &eighth,
    );
    assert!(!offset(&source, -fraction(1, 16)).is_empty());
}

/// The dilation by 1/8 is bounded by the cubic's right parallel, which is the
/// exact rational PH offset; the chord moved by 1/8 along its right unit
/// normal `(-3, 2)/sqrt 13`; and round joins about both corners.
#[test]
fn ph_dilation_matches_its_rational_offset_boundary() {
    let distance = fraction(1, 8);
    let parallel = ph_cubic().parallel_left(-distance.clone()).unwrap();
    let rational = match support::under_classified_result(&STRICT, || {
        parallel.exact_pythagorean_hodograph_offset()
    })
    .unwrap()
    {
        Classification::Decided(Some(exact)) => exact.curve().clone(),
        other => panic!("the PH cubic has an exact rational offset: {other:?}"),
    };
    let sqrt13 = Real::from(13).sqrt().unwrap();
    let (nx, ny) = (
        (Real::from(-3) * &distance / &sqrt13).unwrap(),
        (Real::from(2) * &distance / &sqrt13).unwrap(),
    );
    let origin = Point2::new(Real::zero(), Real::zero());
    let corner = Point2::new(fraction(2, 3), Real::one());
    let cubic_start = Point2::new(Real::zero(), -distance.clone());
    let cubic_end = Point2::new(fraction(2, 3) + &distance, Real::one());
    let chord_start = Point2::new(fraction(2, 3) + &nx, Real::one() + &ny);
    let chord_end = Point2::new(nx, ny);
    let arc = |start: &Point2, end: &Point2, center: &Point2| {
        Curve2::from(
            CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), false)
                .unwrap(),
        )
    };
    let expected = region(&[CurvePath2::try_new(vec![
        Curve2::from(rational),
        arc(&cubic_end, &chord_start, &corner),
        line(&chord_start, &chord_end),
        arc(&chord_end, &cubic_start, &origin),
    ])
    .unwrap()]);
    assert_same_set(
        "rational PH boundary",
        &offset(&ph_region(), distance),
        &expected,
    );
}
