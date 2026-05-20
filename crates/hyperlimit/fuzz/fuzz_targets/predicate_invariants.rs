//! Fuzz exact predicate invariants over small rational coordinate sets.
//!
//! The generated inputs stay in `hyperreal::Real` and never use primitive-float
//! topology. The checks focus on metamorphic laws that should survive every
//! exact kernel and fallback route: orientation reversal/cyclicity, batch/scalar
//! agreement, prepared-line/prepared-incircle/prepared-insphere agreement,
//! versioned prepared-cache freshness, and circle/sphere boundary behavior.
//!
//! Run with: `cargo fuzz run predicate_invariants` from `hyperlimit/fuzz/`.

#![no_main]

use arbitrary::Arbitrary;
use hyperlimit::{
    AabbSphereIntersection, CachePayoff, ConstructionFreshness, ConstructionVersion, LineSide,
    Plane3, Point2, Point3, PredicateApiSemantics, PredicateOutcome, PredicatePolicy, Sign,
    SphereIntersection, certified_ball_sign, certified_interval_sign,
    classify_aabb3_sphere_intersection, classify_circle_line2, classify_circle_segment2,
    classify_circle_line2_batch, classify_circle_segment2_batch,
    classify_homogeneous_point_plane, classify_point_convex_planes3, classify_point_convex_polygon2,
    classify_point_line, classify_point_line_batch, classify_ray_triangle3_intersection,
    classify_ray_triangle3_intersection_batch,
    classify_segment_triangle3_intersection, classify_segment3_intersection,
    classify_segment_triangle3_intersection_batch, classify_segment3_intersection_batch,
    classify_sphere3_intersection, compare_point_line3_distance_squared,
    compare_point_plane_distance_squared, compare_point_segment3_distance_squared,
    compare_point2_lexicographic, compare_point2_lexicographic_report, compare_reals,
    compare_reals_report, incircle2d, insphere3d, intersect_three_planes, intersect_two_planes,
    orient2d, orient2d_batch, point2_equal, point2_equal_report,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawPoint {
    x_num: i16,
    x_den: u8,
    y_num: i16,
    y_den: u8,
}

impl RawPoint {
    fn into_point(self) -> Point2 {
        Point2::new(
            rational(self.x_num, self.x_den),
            rational(self.y_num, self.y_den),
        )
    }
}

/// Generated 3D rational point.
///
/// Keeping this as rational data mirrors Yap's exact-geometric-computation
/// model: fuzzing exercises exact predicate packages and prepared-object reuse,
/// not primitive-float filters. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawPoint3 {
    x_num: i16,
    x_den: u8,
    y_num: i16,
    y_den: u8,
    z_num: i16,
    z_den: u8,
}

impl RawPoint3 {
    fn into_point(self) -> Point3 {
        Point3::new(
            rational(self.x_num, self.x_den),
            rational(self.y_num, self.y_den),
            rational(self.z_num, self.z_den),
        )
    }
}

#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    a: RawPoint,
    b: RawPoint,
    c: RawPoint,
    d: RawPoint,
    p: RawPoint3,
    q: RawPoint3,
    r: RawPoint3,
    s: RawPoint3,
    t: RawPoint3,
}

fuzz_target!(|input: Input| {
    predicate_invariants(input);
});

fn predicate_invariants(input: Input) {
    let a = input.a.into_point();
    let b = input.b.into_point();
    let c = input.c.into_point();
    let d = input.d.into_point();
    let p = input.p.into_point();
    let q = input.q.into_point();
    let r = input.r.into_point();
    let s = input.s.into_point();
    let t = input.t.into_point();

    let abc = orient2d(&a, &b, &c);
    let bca = orient2d(&b, &c, &a);
    let bac = orient2d(&b, &a, &c);

    if let (Some(abc), Some(bca), Some(bac)) = (abc.value(), bca.value(), bac.value()) {
        assert_eq!(abc, bca, "cyclic orientation should preserve sign");
        assert_eq!(
            abc.reversed(),
            bac,
            "swapping two vertices should reverse sign"
        );
    }

    let batch_cases = [
        (a.clone(), b.clone(), c.clone()),
        (b.clone(), a.clone(), c.clone()),
    ];
    let batch = orient2d_batch(&batch_cases);
    assert_eq!(batch[0].value(), orient2d(&a, &b, &c).value());
    assert_eq!(batch[1].value(), orient2d(&b, &a, &c).value());

    let line_side = classify_point_line(&a, &b, &c).value();
    if let Some(sign) = orient2d(&a, &b, &c).value() {
        assert_eq!(line_side, Some(LineSide::from(sign)));
    }

    let session = hyperlimit::ExactGeometrySession::default();
    let payoff = CachePayoff::new(3, 2, 2).expect("generated prepared line should repay");
    let prepared = session.versioned_prepared_with_payoff(session.prepare_line2(&a, &b), payoff);
    assert_eq!(prepared.source_version(), ConstructionVersion::ZERO);
    assert_eq!(prepared.payoff(), Some(payoff));
    assert_eq!(
        prepared.api_semantics(),
        PredicateApiSemantics::CachePopulating
    );
    assert!(prepared.is_current_for(session));
    assert_eq!(
        prepared.freshness_for(session),
        ConstructionFreshness::Current
    );
    assert_eq!(
        session
            .classify_prepared_line2(prepared.prepared(), &c)
            .value(),
        line_side
    );

    let line_batch_cases = [
        (a.clone(), b.clone(), c.clone()),
        (a.clone(), b.clone(), d.clone()),
    ];
    let line_batch = classify_point_line_batch(&line_batch_cases);
    assert_eq!(
        line_batch[0].value(),
        classify_point_line(&a, &b, &c).value()
    );
    assert_eq!(
        line_batch[1].value(),
        classify_point_line(&a, &b, &d).value()
    );

    assert_eq!(
        compare_reals_report(&a.x, &b.x).value(),
        compare_reals(&a.x, &b.x).value(),
        "report-bearing Real ordering must agree with lightweight outcome API"
    );
    assert_eq!(
        compare_point2_lexicographic_report(&a, &b).value(),
        compare_point2_lexicographic(&a, &b).value(),
        "report-bearing point ordering must agree with lightweight outcome API"
    );
    assert_eq!(
        point2_equal_report(&a, &b).value(),
        point2_equal(&a, &b).value(),
        "report-bearing point equality must agree with lightweight outcome API"
    );

    // Any input site lies exactly on its own circumcircle. Degenerate fixed
    // triples may make the circle predicate zero for broader reasons, but the
    // boundary-site law must always hold when the predicate decides.
    assert_decided_zero(incircle2d(&a, &b, &c, &a));
    assert_decided_zero(incircle2d(&a, &b, &c, &b));
    assert_decided_zero(incircle2d(&a, &b, &c, &c));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &p));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &q));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &r));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &s));

    let prepared_incircle = session.versioned_prepared(session.prepare_incircle2(&a, &b, &c));
    assert!(prepared_incircle.is_current_for(session));
    assert_eq!(
        session
            .test_prepared_incircle2(prepared_incircle.prepared(), &d)
            .value(),
        incircle2d(&a, &b, &c, &d).value(),
        "prepared in-circle path must agree with scalar predicate"
    );
    assert!(
        prepared_incircle
            .prepared()
            .coefficient_facts()
            .coefficient_exact
            .all_exact_rational,
        "rational fuzz sites must produce exact rational lifted-circle coefficients"
    );
    assert_eq!(
        prepared_incircle
            .prepared()
            .coefficient_facts()
            .coefficient_unknown_zero_count(),
        0,
        "rational lifted-circle coefficients should have decidable zero status"
    );

    let prepared_insphere = session.versioned_prepared(session.prepare_insphere3(&p, &q, &r, &s));
    assert!(prepared_insphere.is_current_for(session));
    assert_eq!(
        session
            .test_prepared_insphere3(prepared_insphere.prepared(), &t)
            .value(),
        insphere3d(&p, &q, &r, &s, &t).value(),
        "prepared in-sphere path must agree with scalar predicate"
    );
    assert!(
        prepared_insphere
            .prepared()
            .coefficient_facts()
            .coefficient_exact
            .all_exact_rational,
        "rational fuzz sites must produce exact rational lifted-sphere coefficients"
    );
    assert_eq!(
        prepared_insphere
            .prepared()
            .coefficient_facts()
            .coefficient_unknown_zero_count(),
        0,
        "rational lifted-sphere coefficients should have decidable zero status"
    );

    // Fuzz the versioned prepared-cache invalidation boundary. Stale prepared
    // objects are legal Rust borrows, but their retained facts must be treated
    // as scheduling metadata to recompute or bypass, never as topology
    // certificates. This is Yap's construction-object separation in executable
    // form: cached object facts have version provenance, while exact predicates
    // still certify signs. See Yap, "Towards Exact Geometric Computation,"
    // *Computational Geometry* 7.1-2 (1997).
    let mut stale_session = session;
    stale_session.advance_version();
    for freshness in [
        prepared.freshness_for(stale_session),
        prepared_incircle.freshness_for(stale_session),
        prepared_insphere.freshness_for(stale_session),
    ] {
        assert_eq!(
            freshness,
            ConstructionFreshness::StaleSource {
                cached: ConstructionVersion::ZERO,
                current: stale_session.version()
            }
        );
    }

    let interval = certified_interval_sign(&a.x, &b.x);
    let ax_sign = sign_of_rational(&a.x);
    let bx_sign = sign_of_rational(&b.x);
    if ax_sign == bx_sign {
        assert_eq!(
            interval.and_then(PredicateOutcome::value),
            Some(ax_sign),
            "same-sign rational interval endpoints should certify the interval sign"
        );
    }

    let radius = rational((input.a.x_num.unsigned_abs() % 7) as i16, input.a.x_den);
    let ball = certified_ball_sign(&a.x, &radius);
    let lower = a.x.clone() - radius.clone();
    let upper = a.x.clone() + radius;
    assert_eq!(
        ball.and_then(PredicateOutcome::value),
        certified_interval_sign(&lower, &upper).and_then(PredicateOutcome::value),
        "certified ball signs must agree with their exact interval enclosure"
    );

    let strict_no_refine = PredicatePolicy {
        allow_refinement: false,
        ..PredicatePolicy::STRICT
    };
    let no_refine = hyperlimit::orient2d_with_policy(&a, &b, &c, strict_no_refine);
    if let Some(sign) = orient2d(&a, &b, &c).value() {
        // For exact rational fuzz inputs, disabling refinement must not change
        // decided orientation signs. This is the Yap EGC contract in executable
        // form: exact rational predicates are not primitive-float filters.
        assert_eq!(no_refine.value(), Some(sign));
    }

    let common_a = common_scale_point3(input.p.x_num, input.p.y_num, input.p.z_num);
    let common_b = common_scale_point3(input.q.x_num, input.q.y_num, input.q.z_num);
    let common_c = common_scale_point3(input.r.x_num, input.r.y_num, input.r.z_num);
    let common_d = common_scale_point3(input.s.x_num, input.s.y_num, input.s.z_num);
    let common = hyperlimit::orient3d_with_policy(
        &common_a,
        &common_b,
        &common_c,
        &common_d,
        strict_no_refine,
    );
    let swapped = hyperlimit::orient3d_with_policy(
        &common_b,
        &common_a,
        &common_c,
        &common_d,
        strict_no_refine,
    );
    if let (Some(sign), Some(swapped)) = (common.value(), swapped.value()) {
        // These generated points all use one unreduced prime denominator, so
        // they cover the common-scale rational-vector regime Yap identifies
        // before scalar expansion. The public invariant remains purely
        // predicate-level: swapping two vertices reverses the certified
        // tetrahedron orientation sign.
        assert_eq!(sign.reversed(), swapped);
    }

    let x_plane = coordinate_plane(0, &p.x);
    let y_plane = coordinate_plane(1, &p.y);
    let z_plane = coordinate_plane(2, &p.z);
    let homogeneous = intersect_three_planes(&x_plane, &y_plane, &z_plane);
    assert_eq!(
        homogeneous.to_affine_point().ok(),
        Some(p.clone()),
        "coordinate-plane triple should recover the generated rational point"
    );
    for plane in [&x_plane, &y_plane, &z_plane] {
        assert_eq!(
            classify_homogeneous_point_plane(&homogeneous, plane).value(),
            Some(true),
            "homogeneous intersection point must satisfy each source plane"
        );
    }
    let line = intersect_two_planes(&x_plane, &y_plane);
    assert_eq!(
        line.intersect_plane(&z_plane),
        homogeneous,
        "two-plane line plus third-plane intersection should match direct plane triple"
    );

    let segment_relation = classify_segment3_intersection(&p, &q, &r, &s).value();
    let segment_batch_cases = [(p.clone(), q.clone(), r.clone(), s.clone())];
    assert_eq!(
        classify_segment3_intersection_batch(&segment_batch_cases)[0].value(),
        segment_relation,
        "3D segment batch relation must match scalar relation"
    );
    assert_eq!(
        segment_relation,
        classify_segment3_intersection(&r, &s, &p, &q).value(),
        "3D segment intersection classification must be symmetric under segment exchange"
    );
    assert_eq!(
        segment_relation,
        classify_segment3_intersection(&q, &p, &r, &s).value(),
        "3D segment intersection classification must be invariant under endpoint reversal"
    );

    let zero = Real::from(0);
    assert_eq!(
        compare_point_line3_distance_squared(&p, &p, &q, &zero).value(),
        Some(core::cmp::Ordering::Equal),
        "a source endpoint has zero squared distance to its generated line"
    );
    assert_eq!(
        compare_point_segment3_distance_squared(&p, &p, &q, &zero).value(),
        Some(core::cmp::Ordering::Equal),
        "a source endpoint has zero squared distance to its generated segment"
    );
    assert_eq!(
        compare_point_plane_distance_squared(&p, &z_plane, &zero).value(),
        Some(core::cmp::Ordering::Equal),
        "a coordinate-plane source point has zero squared distance to its plane"
    );
    assert_eq!(
        classify_sphere3_intersection(&p, &zero, &p, &zero).value(),
        Some(SphereIntersection::Touching),
        "equal zero-radius spheres touch exactly at their shared center"
    );
    assert_eq!(
        classify_aabb3_sphere_intersection(&p, &p, &p, &zero).value(),
        Some(AabbSphereIntersection::Touching),
        "zero-volume AABB and zero-radius sphere touch exactly at their shared point"
    );

    let ray_direction = Point3::new(&q.x - &p.x, &q.y - &p.y, &q.z - &p.z);
    let segment_triangle =
        classify_segment_triangle3_intersection(&p, &q, &p, &r, &s).value();
    let ray_triangle = classify_ray_triangle3_intersection(&p, &ray_direction, &p, &r, &s).value();
    let segment_triangle_batch = [(
        p.clone(),
        q.clone(),
        p.clone(),
        r.clone(),
        s.clone(),
    )];
    assert_eq!(
        classify_segment_triangle3_intersection_batch(&segment_triangle_batch)[0].value(),
        segment_triangle,
        "segment/triangle batch relation must match scalar relation"
    );
    let ray_triangle_batch = [(
        p.clone(),
        ray_direction.clone(),
        p.clone(),
        r.clone(),
        s.clone(),
    )];
    assert_eq!(
        classify_ray_triangle3_intersection_batch(&ray_triangle_batch)[0].value(),
        ray_triangle,
        "ray/triangle batch relation must match scalar relation"
    );
    if let Some(segment_relation) = segment_triangle {
        assert_eq!(
            ray_triangle.map(|relation| relation.intersects()),
            Some(segment_relation.intersects()),
            "ray from the segment start toward the segment end must preserve endpoint-triangle incidence"
        );
    }

    let unit_x_from_a = Point2::new(&a.x + &Real::from(1), a.y.clone());
    assert_eq!(
        classify_circle_line2(&a, &zero, &a, &unit_x_from_a).value(),
        Some(hyperlimit::CircleLineRelation::Tangent),
        "zero-radius circle centered on a nondegenerate line has one boundary contact"
    );
    let circle_line_batch = [(a.clone(), zero.clone(), a.clone(), unit_x_from_a.clone())];
    assert_eq!(
        classify_circle_line2_batch(&circle_line_batch)[0].value(),
        classify_circle_line2(&a, &zero, &a, &unit_x_from_a).value(),
        "circle/line batch relation must match scalar relation"
    );
    assert_eq!(
        classify_circle_segment2(&a, &zero, &a, &a).value(),
        Some(hyperlimit::CircleSegmentRelation::Tangent),
        "zero-radius circle and degenerate segment at the center touch exactly once"
    );
    let circle_segment_batch = [(a.clone(), zero.clone(), a.clone(), a.clone())];
    assert_eq!(
        classify_circle_segment2_batch(&circle_segment_batch)[0].value(),
        classify_circle_segment2(&a, &zero, &a, &a).value(),
        "circle/segment batch relation must match scalar relation"
    );

    let unit_square = vec![
        Point2::new(0.into(), 0.into()),
        Point2::new(1.into(), 0.into()),
        Point2::new(1.into(), 1.into()),
        Point2::new(0.into(), 1.into()),
    ];
    assert_eq!(
        classify_point_convex_polygon2(&unit_square, &Point2::new(0.into(), 0.into())).value(),
        Some(hyperlimit::ConvexPointLocation::Boundary),
        "convex polygon composition must retain exact boundary points"
    );
    let unit_cube_planes = vec![
        Plane3::new(Point3::new((-1).into(), 0.into(), 0.into()), 0.into()),
        Plane3::new(Point3::new(1.into(), 0.into(), 0.into()), (-1).into()),
        Plane3::new(Point3::new(0.into(), (-1).into(), 0.into()), 0.into()),
        Plane3::new(Point3::new(0.into(), 1.into(), 0.into()), (-1).into()),
        Plane3::new(Point3::new(0.into(), 0.into(), (-1).into()), 0.into()),
        Plane3::new(Point3::new(0.into(), 0.into(), 1.into()), (-1).into()),
    ];
    assert_eq!(
        classify_point_convex_planes3(&unit_cube_planes, &Point3::new(0.into(), 0.into(), 0.into()))
            .value(),
        Some(hyperlimit::ConvexPointLocation::Boundary),
        "convex plane composition must retain exact boundary points"
    );
}

fn rational(numerator: i16, denominator_byte: u8) -> Real {
    let denominator = u64::from(denominator_byte % 16) + 1;
    Rational::fraction(i64::from(numerator), denominator)
        .expect("positive generated denominator")
        .into()
}

fn common_scale_point3(x: i16, y: i16, z: i16) -> Point3 {
    fn nonzero_mod17(value: i16) -> i64 {
        i64::from(value).rem_euclid(16) + 1
    }

    Point3::new(
        Rational::fraction(nonzero_mod17(x), 17)
            .expect("prime denominator")
            .into(),
        Rational::fraction(nonzero_mod17(y), 17)
            .expect("prime denominator")
            .into(),
        Rational::fraction(nonzero_mod17(z), 17)
            .expect("prime denominator")
            .into(),
    )
}

fn coordinate_plane(axis: usize, coordinate: &Real) -> Plane3 {
    let normal = match axis {
        0 => Point3::new(1.into(), 0.into(), 0.into()),
        1 => Point3::new(0.into(), 1.into(), 0.into()),
        2 => Point3::new(0.into(), 0.into(), 1.into()),
        _ => unreachable!("fuzz helper only builds 3D coordinate planes"),
    };
    Plane3::new(normal, -coordinate)
}

fn assert_decided_zero(outcome: PredicateOutcome<Sign>) {
    if let Some(sign) = outcome.value() {
        assert_eq!(sign, Sign::Zero);
    }
}

fn sign_of_rational(value: &Real) -> Sign {
    match value.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => Sign::Negative,
        Some(hyperreal::RealSign::Zero) => Sign::Zero,
        Some(hyperreal::RealSign::Positive) => Sign::Positive,
        None => unreachable!("fuzz inputs are generated as exact rationals"),
    }
}
