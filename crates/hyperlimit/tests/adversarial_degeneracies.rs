use hyperlimit::{
    LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, Sign, classify_point_line,
    classify_point_oriented_plane, classify_point_plane, incircle2, insphere3, orient2, orient3,
};

const APPROX: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::APPROXIMATE_512;

type Real = hyperreal::Real;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite test scalar")
}

fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(real(x), real(y))
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(real(x), real(y), real(z))
}

fn add2(left: &Point2, right: &Point2) -> Point2 {
    Point2::new(&left.x + &right.x, &left.y + &right.y)
}

fn scale2(point: &Point2, scale: f64) -> Point2 {
    let scale = real(scale);
    Point2::new(&point.x * &scale, &point.y * &scale)
}

fn add3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(&left.x + &right.x, &left.y + &right.y, &left.z + &right.z)
}

fn decided<T: Copy>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("case should decide")
}

#[test]
fn orient2d_translation_and_scaling_invariance_on_near_collinear_rows() {
    let a = p2(-1.0e6, -1.0e6);
    let b = p2(1.0e6, 1.0e6);
    let c = p2(0.0, 1.0e-6);
    let offset = p2(4096.0, -8192.0);

    let sign = decided(orient2(&a, &b, &c, APPROX));
    let translated = decided(orient2(
        &add2(&a, &offset),
        &add2(&b, &offset),
        &add2(&c, &offset),
        APPROX,
    ));
    let scaled = decided(orient2(
        &scale2(&a, 8.0),
        &scale2(&b, 8.0),
        &scale2(&c, 8.0),
        APPROX,
    ));

    assert_eq!(sign, Sign::Positive);
    assert_eq!(translated, sign);
    assert_eq!(scaled, sign);
}

#[test]
fn finite_f64_subnormal_import_remains_exact_dyadic_predicate_input() {
    let tiny = f64::from_bits(1);
    let a = p2(0.0, 0.0);
    let b = p2(tiny, 0.0);
    let c = p2(0.0, tiny);

    // This is an explicit f64-edge regression, not a primitive-float predicate.
    // The raw f64 determinant underflows to zero, but the finite inputs are
    // lifted as exact dyadic `Real` values before orientation is decided. That
    // keeps the predicate decision on the exact-computation side of the boundary.
    let primitive_det = (tiny - 0.0) * (tiny - 0.0);
    assert_eq!(primitive_det, 0.0);
    assert_eq!(decided(orient2(&a, &b, &c, APPROX)), Sign::Positive);
    assert!(a.x.exact_rational().is_some());
    assert!(b.x.exact_rational().is_some());
    assert!(c.y.exact_rational().is_some());
}

#[test]
fn classify_point_line_is_consistent_for_reversed_line_orientation() {
    let a = p2(-3.0, 2.0);
    let b = p2(5.0, -7.0);
    let p = p2(11.0, 13.0);

    assert_eq!(
        decided(classify_point_line(&a, &b, &p, APPROX)),
        LineSide::Left
    );
    assert_eq!(
        decided(classify_point_line(&b, &a, &p, APPROX)),
        LineSide::Right
    );
}

#[test]
fn orient3d_translation_invariance_and_swap_reversal_on_small_height() {
    let a = p3(0.0, 0.0, 0.0);
    let b = p3(1.0e6, 0.0, 0.0);
    let c = p3(0.0, 1.0e6, 0.0);
    let d = p3(0.25e6, 0.25e6, 1.0e-6);
    let offset = p3(4096.0, -8192.0, 16384.0);

    let sign = decided(orient3(&a, &b, &c, &d, APPROX));
    let translated = decided(orient3(
        &add3(&a, &offset),
        &add3(&b, &offset),
        &add3(&c, &offset),
        &add3(&d, &offset),
        APPROX,
    ));
    let swapped = decided(orient3(&b, &a, &c, &d, APPROX));

    assert_eq!(translated, sign);
    assert_eq!(swapped, sign.reversed());
}

#[test]
fn plane_classification_matches_oriented_plane_for_axis_aligned_case() {
    let a = p3(0.0, 0.0, 0.0);
    let b = p3(1.0, 0.0, 0.0);
    let c = p3(0.0, 1.0, 0.0);
    let above = p3(0.25, 0.25, 1.0e-12);
    let below = p3(0.25, 0.25, -1.0e-12);
    let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(0.0));

    assert_eq!(
        decided(classify_point_plane(&above, &plane, APPROX)),
        PlaneSide::Above
    );
    assert_eq!(
        decided(classify_point_plane(&below, &plane, APPROX)),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_oriented_plane(&a, &b, &c, &above, APPROX)),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_oriented_plane(&a, &b, &c, &below, APPROX)),
        PlaneSide::Above
    );
}

#[test]
fn circle_and_sphere_predicates_distinguish_on_boundary_from_tiny_offsets() {
    let a = p2(1.0, 0.0);
    let b = p2(0.0, 1.0);
    let c = p2(-1.0, 0.0);

    assert_eq!(
        decided(incircle2(&a, &b, &c, &p2(0.0, -1.0), APPROX)),
        Sign::Zero
    );
    assert_eq!(
        decided(incircle2(&a, &b, &c, &p2(0.0, -1.0 + 1.0e-6), APPROX)),
        Sign::Positive
    );
    assert_eq!(
        decided(incircle2(&a, &b, &c, &p2(0.0, -1.0 - 1.0e-6), APPROX)),
        Sign::Negative
    );

    let s0 = p3(1.0, 0.0, 0.0);
    let s1 = p3(-1.0, 0.0, 0.0);
    let s2 = p3(0.0, 1.0, 0.0);
    let s3 = p3(0.0, 0.0, 1.0);
    assert_eq!(
        decided(insphere3(&s0, &s1, &s2, &s3, &p3(0.0, 0.0, 0.0), APPROX)),
        Sign::Positive
    );
}
