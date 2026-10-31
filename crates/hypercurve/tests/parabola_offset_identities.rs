//! Offsets of a parabola checked against its rational reparameterization.
//!
//! For `C(t) = (t, t^2)`, substituting `t = (u^2 - 1) / (4u)` with `u > 0`
//! makes `1 + 4t^2 = ((u^2 + 1) / (2u))^2` a square, so the unit left normal
//! `(-2t, 1) / sqrt(1 + 4t^2) = (1 - u^2, 2u) / (u^2 + 1)` is rational. The
//! parallel at signed distance `delta` is then the degree-six rational curve
//! `C(t(u)) + delta (1 - u^2, 2u) / (u^2 + 1)`. The segment `t in [-3/8, 3/8]`
//! is `u in [1/2, 2]`, whose end normals are `(+-3/5, 4/5)`.
//!
//! The procedural parallel's exact evaluation is compared with this
//! independently derived formula at thirteen parameters, both ends included.
//! That checks evaluation at those points, not an identity of functions. The
//! region bounded by the segment and its chord is convex, so disk offsets also
//! obey exact set identities, and the dilation's boundary lies exactly on the
//! reparameterized normals.
mod support;

use hypercurve::{
    BooleanOp, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2, OffsetCornerStyle2, Point2,
    PredicatePolicy, QuadraticBezier2, Real, RegionPointLocation,
};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

fn fraction(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

/// `C(t) = (t, t^2)` for `t in [-3/8, 3/8]`; Bezier parameter `(8t + 3) / 6`.
fn parabola() -> QuadraticBezier2 {
    QuadraticBezier2::new(
        Point2::new(fraction(-3, 8), fraction(9, 64)),
        Point2::new(Real::zero(), fraction(-9, 64)),
        Point2::new(fraction(3, 8), fraction(9, 64)),
    )
}

/// The parameter `t(u)` and the closed-form parallel point at `delta`.
fn reparameterized(u: &Real, delta: &Real) -> (Real, Point2) {
    let u2 = u * u;
    let t = ((&u2 - Real::one()) / (Real::from(4) * u)).unwrap();
    let denominator = &u2 + Real::one();
    let normal_x = ((Real::one() - &u2) / &denominator).unwrap();
    let normal_y = ((Real::from(2) * u) / &denominator).unwrap();
    let point = Point2::new(&t + delta * normal_x, &t * &t + delta * normal_y);
    (t, point)
}

/// Thirteen parameters `u = k/8`, `k = 4..=16`, covering `[1/2, 2]`.
fn parameters() -> impl Iterator<Item = Real> {
    (4..=16).map(|k| fraction(k, 8))
}

fn region() -> CurveRegion2 {
    let parabola = parabola();
    let chord = LineSeg2::try_new(parabola.end().clone(), parabola.start().clone()).unwrap();
    let path = CurvePath2::try_new(vec![Curve2::from(parabola), Curve2::from(chord)]).unwrap();
    support::under(&STRICT, || {
        CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)
    })
    .unwrap()
    .into_value()
}

fn offset(region: &CurveRegion2, distance: Real) -> CurveRegion2 {
    support::under(&STRICT, || {
        region.offset(distance, &OffsetCornerStyle2::Round)
    })
    .unwrap()
    .into_value()
}

fn boolean(left: &CurveRegion2, right: &CurveRegion2, op: BooleanOp) -> CurveRegion2 {
    support::under(&STRICT, || left.boolean_region(right, op))
        .unwrap()
        .into_value()
}

fn assert_same_set(label: &str, actual: &CurveRegion2, expected: &CurveRegion2) {
    assert!(
        boolean(actual, expected, BooleanOp::Xor).is_empty(),
        "{label}: the regions differ"
    );
}

fn location(region: &CurveRegion2, point: &Point2) -> RegionPointLocation {
    support::under(&STRICT, || region.classify_point(&point.clone().into()))
        .unwrap()
        .into_value()
}

#[test]
fn parabola_parallels_match_the_rational_reparameterization() {
    // Outside the region (right of travel) and inside it; the smallest radius
    // of curvature is 1/2, so neither parallel has a cusp.
    for delta in [fraction(-1, 4), fraction(1, 8)] {
        let parallel = parabola().parallel_left(delta.clone()).unwrap();
        for u in parameters() {
            let (t, expected) = reparameterized(&u, &delta);
            let bezier = ((Real::from(8) * &t + Real::from(3)) / Real::from(6)).unwrap();
            let actual = parallel.point_at(&bezier).unwrap();
            assert_eq!(actual, expected, "delta {delta:?}, u {u:?}");
        }
    }
}

#[test]
fn parabola_region_offsets_obey_convex_disk_identities() {
    let source = region();
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
}

/// Along each reparameterized normal, the dilation by 1/4 has its boundary at
/// distance exactly 1/4 and contains nearer points only. A convex set's
/// nearest point to an exterior point on a boundary normal is the foot of that
/// normal, including at the two corners, whose normal cones contain the
/// parabola's end normals.
#[test]
fn parabola_dilation_boundary_follows_the_reparameterization() {
    let dilated = offset(&region(), fraction(1, 4));
    for u in parameters() {
        for (delta, expected) in [
            (fraction(-1, 4), RegionPointLocation::Boundary),
            (fraction(-15, 64), RegionPointLocation::Inside),
            (fraction(-17, 64), RegionPointLocation::Outside),
        ] {
            let (_, point) = reparameterized(&u, &delta);
            assert_eq!(
                location(&dilated, &point),
                expected,
                "u {u:?}, delta {delta:?}"
            );
        }
    }
}

/// Native replay of the dilation keeps the generated parallel; Boolean
/// identities and a further erosion back to the source then hold.
#[cfg(feature = "svg")]
#[test]
fn replayed_parabola_dilation_supports_booleans_and_closing() {
    use hypercurve::{SvgGeometry2, export_svg_document, import_svg_document};

    let source = region();
    let dilated = offset(&source, fraction(1, 8));
    let text = export_svg_document(&SvgGeometry2::new(dilated.clone(), vec![], vec![])).unwrap();
    let restored = import_svg_document(&text).unwrap().region().clone();
    assert_same_set("replay", &restored, &dilated);
    let ring = boolean(&restored, &source, BooleanOp::Difference);
    assert!(!ring.is_empty());
    assert_same_set(
        "ring and source",
        &boolean(&ring, &source, BooleanOp::Union),
        &restored,
    );
    assert_same_set(
        "closing after replay",
        &offset(&restored, -fraction(1, 8)),
        &source,
    );
}
