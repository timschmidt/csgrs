use hyperlimit::{
    LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, Sign, classify_point_line,
    classify_point_oriented_plane, classify_point_plane, compare_reals,
    construct_line_intersection_point, incircle2, insphere3, orient2, orient3,
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

fn perpendicular_bisector(a: &Point2, b: &Point2) -> (Point2, Point2) {
    let two = Real::from(2);
    let midpoint = Point2::new(
        ((&a.x + &b.x) / &two).expect("two is nonzero"),
        ((&a.y + &b.y) / &two).expect("two is nonzero"),
    );
    let dx = &b.x - &a.x;
    let dy = &b.y - &a.y;
    let endpoint = Point2::new(&midpoint.x - &dy, &midpoint.y + &dx);
    (midpoint, endpoint)
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

#[test]
fn exactcore_radical_circle_points_certify_the_boundary() {
    let root_two = Real::from(2).sqrt().expect("positive radicand");
    let root_three = Real::from(3).sqrt().expect("positive radicand");
    assert!(root_two.exact_rational().is_none());
    assert!(root_three.exact_rational().is_none());

    let a = p2(2.0, 0.0);
    let b = p2(4.0, 2.0);
    let c = p2(2.0, 4.0);
    let on_circle = [
        Point2::new(Real::from(2) + &root_three, Real::from(1)),
        Point2::new(Real::from(2) + &root_two, Real::from(2) + &root_two),
        Point2::new(Real::from(2) - &root_two, Real::from(2) + &root_two),
        Point2::new(Real::from(2) - &root_three, Real::from(1)),
    ];
    for point in &on_circle {
        assert_eq!(decided(incircle2(&a, &b, &c, point, APPROX)), Sign::Zero);
    }
    for point in [p2(5.0, 6.0), p2(-2.0, 3.0), p2(-1.0, -2.0), p2(2.0, -3.0)] {
        assert_eq!(
            decided(incircle2(&a, &b, &c, &point, APPROX)),
            Sign::Negative
        );
    }
}

#[test]
fn exactcore_algebraic_perpendicular_bisectors_concur_but_perturbation_does_not() {
    use core::cmp::Ordering;

    let root_three = Real::from(3).sqrt().expect("positive radicand");
    let root_seven = Real::from(7).sqrt().expect("positive radicand");
    let a = p2(0.0, 0.0);
    let b = Point2::new(Real::from(10), root_seven);
    let c = Point2::new(Real::from(5), Real::from(5) * root_three);
    let (g1a, g1b) = perpendicular_bisector(&a, &b);
    let (g2a, g2b) = perpendicular_bisector(&a, &c);
    let (g3a, g3b) = perpendicular_bisector(&b, &c);
    let p1 = construct_line_intersection_point(&g1a, &g1b, &g2a, &g2b)
        .expect("the first two bisectors are not parallel");
    let p2 = construct_line_intersection_point(&g2a, &g2b, &g3a, &g3b)
        .expect("the second and third bisectors are not parallel");
    let p3 = construct_line_intersection_point(&g3a, &g3b, &g1a, &g1b)
        .expect("the third and first bisectors are not parallel");

    for point in [&p2, &p3] {
        assert_eq!(
            decided(compare_reals(&p1.x, &point.x, APPROX)),
            Ordering::Equal
        );
        assert_eq!(
            decided(compare_reals(&p1.y, &point.y, APPROX)),
            Ordering::Equal
        );
    }

    let epsilon = Real::from(hyperreal::Rational::fraction(1, 123_456_789).unwrap());
    let perturbed_c = Point2::new(c.x.clone(), &c.y + epsilon);
    let (perturbed_g3a, perturbed_g3b) = perpendicular_bisector(&b, &perturbed_c);
    let perturbed = construct_line_intersection_point(&perturbed_g3a, &perturbed_g3b, &g1a, &g1b)
        .expect("the perturbed bisector and first bisector are not parallel");
    assert_eq!(
        decided(compare_reals(&p1.x, &perturbed.x, APPROX)),
        Ordering::Greater
    );
    assert_eq!(
        decided(compare_reals(&p1.y, &perturbed.y, APPROX)),
        Ordering::Less
    );
}

#[test]
fn pappus_intersection_triple_is_exactly_collinear() {
    let a = p2(0.0, 0.0);
    let b = p2(2.0, 0.0);
    let c = p2(5.0, 0.0);
    let e = p2(1.0, 1.0);
    let f = p2(4.0, 1.0);
    let g = p2(7.0, 1.0);

    let p = construct_line_intersection_point(&a, &f, &e, &b)
        .expect("AF and EB have a unique intersection");
    let q = construct_line_intersection_point(&a, &g, &e, &c)
        .expect("AG and EC have a unique intersection");
    let s = construct_line_intersection_point(&b, &g, &f, &c)
        .expect("BG and FC have a unique intersection");

    let fraction = |numerator, denominator| {
        Real::from(hyperreal::Rational::fraction(numerator, denominator).unwrap())
    };
    assert_eq!(p, Point2::new(fraction(8, 5), fraction(2, 5)));
    assert_eq!(q, Point2::new(fraction(35, 11), fraction(5, 11)));
    assert_eq!(s, Point2::new(fraction(9, 2), fraction(1, 2)));
    assert!(
        [&p, &q, &s].into_iter().all(|point| {
            point.x.exact_rational().is_some() && point.y.exact_rational().is_some()
        })
    );
    assert_eq!(decided(orient2(&p, &q, &s, APPROX)), Sign::Zero);
}

#[test]
fn affine_pentagon_inner_outer_replay_stays_exact_for_five_rounds() {
    let rational = |numerator, denominator| {
        Real::from(hyperreal::Rational::fraction(numerator, denominator).unwrap())
    };
    let planar = [
        Point2::new(rational(1, 1), rational(0, 1)),
        Point2::new(rational(309_017, 1_000_000), rational(951, 1_000)),
        Point2::new(rational(-809_017, 1_000_000), rational(587_785, 1_000_000)),
        Point2::new(rational(-809_017, 1_000_000), rational(-587_785, 1_000_000)),
        Point2::new(rational(309_017, 1_000_000), rational(-951, 1_000)),
    ];

    // First two coordinates of the historical injective 3D affine transform.
    let original = planar.map(|point| {
        let x = &point.x + &rational(6, 5);
        let y = &point.y + &rational(17, 5);
        let z = rational(28, 5);
        Point2::new(
            &(&rational(617, 500) * &x)
                + &(&(&rational(7_777, 1_000) * &y) + &(&rational(1_111, 1_000) * &z)),
            &(&rational(4, 1) * &x) + &(&(&rational(5, 1) * &y) + &(&rational(3, 1) * &z)),
        )
    });

    let mut outer = original.clone();
    for _ in 0..5 {
        let inner: [Point2; 5] = std::array::from_fn(|index| {
            construct_line_intersection_point(
                &outer[index],
                &outer[(index + 2) % 5],
                &outer[(index + 1) % 5],
                &outer[(index + 4) % 5],
            )
            .expect("pentagon diagonals have a unique intersection")
        });
        assert!(inner.iter().all(|point| {
            point.x.exact_rational().is_some() && point.y.exact_rational().is_some()
        }));
        outer = std::array::from_fn(|index| {
            construct_line_intersection_point(
                &inner[index],
                &inner[(index + 1) % 5],
                &inner[(index + 3) % 5],
                &inner[(index + 4) % 5],
            )
            .expect("nonadjacent pentagon edges have a unique intersection")
        });

        assert_eq!(outer, original);
        assert!(outer.iter().all(|point| {
            point.x.exact_rational().is_some() && point.y.exact_rational().is_some()
        }));
    }
}
