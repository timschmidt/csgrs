//! Fuzz exact predicate invariants over small rational coordinate sets.
//!
//! The generated inputs stay in `hyperreal::Real` and never use primitive-float
//! topology. The checks focus on metamorphic laws that should survive every
//! exact kernel and fallback route: orientation reversal/cyclicity, batch/scalar
//! agreement, retained-evidence agreement, and
//! circle/sphere boundary behavior.
//!
//! Run with: `cargo fuzz run predicate_invariants` from `hyperlimit/fuzz/`.

#![no_main]

use arbitrary::Arbitrary;
use hyperlimit::{
    AabbSphereIntersection, CoplanarProjection, LineSide, Plane3, Point2, Point3, PredicateOutcome,
    PredicatePolicy, SegmentPlaneRelation, Sign, SphereIntersection, SupportDop3,
    SupportDopPlaneRelation, SupportDopRelation, SupportSlab3, TriangleDegeneracy,
    certified_ball_sign, certified_interval_sign, classify_aabb3_sphere_intersection,
    classify_circle_line2, classify_circle_line2_batch, classify_circle_segment2,
    classify_circle_segment2_batch, classify_coplanar_triangles, classify_halfspace_feasibility3,
    classify_homogeneous_point_plane, classify_plane_aabb3_report, classify_point_convex_planes3,
    classify_point_convex_polygon2, classify_point_line, classify_point_line_batch,
    classify_point_ring_even_odd, classify_point_ring_even_odd_report,
    classify_ray_triangle3_intersection, classify_ray_triangle3_intersection_batch,
    classify_ray_triangle3_intersection_report, classify_segment_triangle3_intersection,
    classify_segment_triangle3_intersection_batch, classify_segment_triangle3_intersection_report,
    classify_segment3_intersection, classify_segment3_intersection_batch,
    classify_sphere3_intersection, classify_triangle_triangle3, classify_triangle3_degeneracy,
    compare_point_line3_distance_squared, compare_point_plane_distance_squared,
    compare_point_segment3_distance_squared, incircle2 as incircle2d, incircle2_evidence,
    incircle2_with_evidence as incircle2d_with_evidence, insphere3 as insphere3d,
    insphere3_evidence, insphere3_with_evidence as insphere3d_with_evidence,
    intersect_segment_with_oriented_plane, intersect_three_planes, intersect_two_planes,
    orient2 as orient2d, orient2_batch as orient2d_batch, projected_line_parameter3,
    projected_segment_parameter3,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

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
/// Rational inputs ensure fuzzing exercises exact predicate packages and
/// retained-evidence reuse, not primitive-float filters.
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

    let abc = orient2d(&a, &b, &c, APPROX);
    let bca = orient2d(&b, &c, &a, APPROX);
    let bac = orient2d(&b, &a, &c, APPROX);

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
    let batch = orient2d_batch(&batch_cases, APPROX);
    assert_eq!(batch[0].value(), orient2d(&a, &b, &c, APPROX).value());
    assert_eq!(batch[1].value(), orient2d(&b, &a, &c, APPROX).value());

    let line_side = classify_point_line(&a, &b, &c, APPROX).value();
    if let Some(sign) = orient2d(&a, &b, &c, APPROX).value() {
        assert_eq!(line_side, Some(LineSide::from(sign)));
    }

    let ring = [a.clone(), b.clone(), c.clone(), d.clone()];
    let reversed_ring = [d.clone(), c.clone(), b.clone(), a.clone()];
    let ring_location = classify_point_ring_even_odd(&ring, &a, APPROX).value();
    assert_eq!(
        classify_point_ring_even_odd(&reversed_ring, &a, APPROX).value(),
        ring_location,
        "even-odd point/ring classification must be invariant under ring reversal"
    );
    if let Some(report) = classify_point_ring_even_odd_report(&ring, &a, APPROX).value() {
        assert_eq!(
            Some(report.location),
            ring_location,
            "even-odd report relation must match scalar classifier"
        );
        report
            .validate_against_sources(&ring, &a, APPROX)
            .expect("even-odd ring report must replay against exact sources");
    }

    let orientation = hyperlimit::line2_orientation(&a, &b);
    assert_eq!(
        hyperlimit::classify_point_line_with_orientation(&a, &b, &c, &orientation, APPROX).value(),
        line_side
    );

    let line_batch_cases = [
        (a.clone(), b.clone(), c.clone()),
        (a.clone(), b.clone(), d.clone()),
    ];
    let line_batch = classify_point_line_batch(&line_batch_cases, APPROX);
    assert_eq!(
        line_batch[0].value(),
        classify_point_line(&a, &b, &c, APPROX).value()
    );
    assert_eq!(
        line_batch[1].value(),
        classify_point_line(&a, &b, &d, APPROX).value()
    );

    // Any input site lies exactly on its own circumcircle. Degenerate fixed
    // triples may make the circle predicate zero for broader reasons, but the
    // boundary-site law must always hold when the predicate decides.
    assert_decided_zero(incircle2d(&a, &b, &c, &a, APPROX));
    assert_decided_zero(incircle2d(&a, &b, &c, &b, APPROX));
    assert_decided_zero(incircle2d(&a, &b, &c, &c, APPROX));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &p, APPROX));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &q, APPROX));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &r, APPROX));
    assert_decided_zero(insphere3d(&p, &q, &r, &s, &s, APPROX));

    let incircle_evidence = incircle2_evidence(&a, &b, &c);
    assert_eq!(
        incircle2d_with_evidence(&a, &b, &c, &d, &incircle_evidence, APPROX).value(),
        incircle2d(&a, &b, &c, &d, APPROX).value(),
        "retained in-circle evidence must agree with the scalar predicate"
    );
    assert!(
        incircle_evidence
            .coefficient_facts()
            .coefficient_exact
            .all_exact_rational,
        "rational fuzz sites must produce exact rational lifted-circle coefficients"
    );
    assert_eq!(
        incircle_evidence
            .coefficient_facts()
            .coefficient_unknown_zero_count(),
        0,
        "rational lifted-circle coefficients should have decidable zero status"
    );

    let insphere_evidence = insphere3_evidence(&p, &q, &r, &s);
    assert_eq!(
        insphere3d_with_evidence(&p, &q, &r, &s, &t, &insphere_evidence, APPROX).value(),
        insphere3d(&p, &q, &r, &s, &t, APPROX).value(),
        "retained in-sphere evidence must agree with the scalar predicate"
    );
    assert!(
        insphere_evidence
            .coefficient_facts()
            .coefficient_exact
            .all_exact_rational,
        "rational fuzz sites must produce exact rational lifted-sphere coefficients"
    );
    assert_eq!(
        insphere_evidence
            .coefficient_facts()
            .coefficient_unknown_zero_count(),
        0,
        "rational lifted-sphere coefficients should have decidable zero status"
    );

    let interval = certified_interval_sign(&a.x, &b.x, APPROX);
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
    let ball = certified_ball_sign(&a.x, &radius, APPROX);
    let lower = a.x.clone() - radius.clone();
    let upper = a.x.clone() + radius;
    assert_eq!(
        ball.and_then(PredicateOutcome::value),
        certified_interval_sign(&lower, &upper, APPROX).and_then(PredicateOutcome::value),
        "certified ball signs must agree with their exact interval enclosure"
    );

    let strict = hyperlimit::orient2(&a, &b, &c, PredicatePolicy::STRICT);
    if let Some(sign) = orient2d(&a, &b, &c, APPROX).value() {
        // The explicit strict policy and convenience entry point must preserve
        // the same exact-rational orientation decision.
        assert_eq!(strict.value(), Some(sign));
    }

    let common_a = common_scale_point3(input.p.x_num, input.p.y_num, input.p.z_num);
    let common_b = common_scale_point3(input.q.x_num, input.q.y_num, input.q.z_num);
    let common_c = common_scale_point3(input.r.x_num, input.r.y_num, input.r.z_num);
    let common_d = common_scale_point3(input.s.x_num, input.s.y_num, input.s.z_num);
    let common = hyperlimit::orient3(&common_a, &common_b, &common_c, &common_d, APPROX);
    let swapped = hyperlimit::orient3(&common_b, &common_a, &common_c, &common_d, APPROX);
    if let (Some(sign), Some(swapped)) = (common.value(), swapped.value()) {
        // These generated points all use one unreduced prime denominator, so
        // they cover the common-scale rational-vector regime before scalar
        // expansion. The public invariant remains purely
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
            classify_homogeneous_point_plane(&homogeneous, plane, APPROX).value(),
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

    let segment_relation = classify_segment3_intersection(&p, &q, &r, &s, APPROX).value();
    let segment_batch_cases = [(p.clone(), q.clone(), r.clone(), s.clone())];
    assert_eq!(
        classify_segment3_intersection_batch(&segment_batch_cases, APPROX)[0].value(),
        segment_relation,
        "3D segment batch relation must match scalar relation"
    );
    assert_eq!(
        segment_relation,
        classify_segment3_intersection(&r, &s, &p, &q, APPROX).value(),
        "3D segment intersection classification must be symmetric under segment exchange"
    );
    assert_eq!(
        segment_relation,
        classify_segment3_intersection(&q, &p, &r, &s, APPROX).value(),
        "3D segment intersection classification must be invariant under endpoint reversal"
    );

    let zero = Real::from(0);
    assert_eq!(
        compare_point_line3_distance_squared(&p, &p, &q, &zero, APPROX).value(),
        Some(core::cmp::Ordering::Equal),
        "a source endpoint has zero squared distance to its generated line"
    );
    assert_eq!(
        compare_point_segment3_distance_squared(&p, &p, &q, &zero, APPROX).value(),
        Some(core::cmp::Ordering::Equal),
        "a source endpoint has zero squared distance to its generated segment"
    );
    assert_eq!(
        compare_point_plane_distance_squared(&p, &z_plane, &zero, APPROX).value(),
        Some(core::cmp::Ordering::Equal),
        "a coordinate-plane source point has zero squared distance to its plane"
    );
    assert_eq!(
        classify_sphere3_intersection(&p, &zero, &p, &zero, APPROX).value(),
        Some(SphereIntersection::Touching),
        "equal zero-radius spheres touch exactly at their shared center"
    );
    assert_eq!(
        classify_aabb3_sphere_intersection(&p, &p, &p, &zero, APPROX).value(),
        Some(AabbSphereIntersection::Touching),
        "zero-volume AABB and zero-radius sphere touch exactly at their shared point"
    );
    if let Some(report) = classify_plane_aabb3_report(&z_plane, &p, &p, APPROX).value() {
        assert_eq!(
            report.validate_against_sources(&z_plane, &p, &p, APPROX,),
            Ok(()),
            "point-sized AABB plane report must replay exact support extrema"
        );
        assert_eq!(
            report.relation,
            hyperlimit::PlaneAabbRelation::Intersecting,
            "point-sized AABB on its coordinate plane must intersect"
        );
    }

    let ray_direction = Point3::new(&q.x - &p.x, &q.y - &p.y, &q.z - &p.z);
    let segment_triangle =
        classify_segment_triangle3_intersection(&p, &q, &p, &r, &s, APPROX).value();
    let ray_triangle =
        classify_ray_triangle3_intersection(&p, &ray_direction, &p, &r, &s, APPROX).value();
    let segment_triangle_batch = [(p.clone(), q.clone(), p.clone(), r.clone(), s.clone())];
    assert_eq!(
        classify_segment_triangle3_intersection_batch(&segment_triangle_batch, APPROX)[0].value(),
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
        classify_ray_triangle3_intersection_batch(&ray_triangle_batch, APPROX)[0].value(),
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
    if let Some(report) =
        classify_segment_triangle3_intersection_report(&p, &q, &p, &r, &s, APPROX).value()
    {
        assert_eq!(
            Some(report.relation),
            segment_triangle,
            "segment/triangle report relation must match scalar classifier"
        );
        report
            .validate_against_sources(&p, &q, &p, &r, &s, APPROX)
            .expect("segment/triangle report must replay against exact sources");
        if report.relation.intersects()
            && report.relation != hyperlimit::SegmentTriangleIntersection::Coplanar
        {
            assert!(
                report.has_candidate_point(),
                "non-coplanar segment/triangle contacts must retain a candidate point"
            );
        }
    }
    if let Some(report) =
        classify_ray_triangle3_intersection_report(&p, &ray_direction, &p, &r, &s, APPROX).value()
    {
        assert_eq!(
            Some(report.relation),
            ray_triangle,
            "ray/triangle report relation must match scalar classifier"
        );
        report
            .validate_against_sources(&p, &ray_direction, &p, &r, &s, APPROX)
            .expect("ray/triangle report must replay against exact sources");
        if report.relation.intersects()
            && report.relation != hyperlimit::RayTriangleIntersection::Coplanar
        {
            assert!(
                report.has_candidate_point(),
                "non-coplanar ray/triangle contacts must retain a candidate point"
            );
        }
    }

    let triangle_degeneracy = classify_triangle3_degeneracy(&p, &q, &r, APPROX);
    assert!(
        matches!(
            triangle_degeneracy,
            PredicateOutcome::Decided {
                value: TriangleDegeneracy::NonDegenerate | TriangleDegeneracy::Degenerate,
                ..
            }
        ),
        "rational 3D triangle degeneracy should be exactly decided"
    );

    let segment_plane = intersect_segment_with_oriented_plane(&p, &q, &r, &s, &t, APPROX);
    segment_plane
        .validate(APPROX)
        .expect("segment/plane event must be internally coherent");
    segment_plane
        .validate_against_sources(&p, &q, &r, &s, &t, APPROX)
        .expect("segment/plane event must replay against its source points");
    if segment_plane.relation == SegmentPlaneRelation::ProperCrossing {
        assert!(
            segment_plane.point.is_some() && segment_plane.parameter_ratio.is_some(),
            "proper segment/plane crossings must retain exact construction data"
        );
    }

    let lifted = [
        Point3::new(a.x.clone(), a.y.clone(), 0.into()),
        Point3::new(b.x.clone(), b.y.clone(), 0.into()),
        Point3::new(c.x.clone(), c.y.clone(), 0.into()),
        Point3::new(d.x.clone(), d.y.clone(), 0.into()),
        Point3::new(&a.x + &Real::from(1), a.y.clone(), 0.into()),
        Point3::new(a.x.clone(), &a.y + &Real::from(1), 0.into()),
    ];
    let coplanar = classify_coplanar_triangles(&lifted, [0, 1, 2], [3, 4, 5], APPROX);
    coplanar
        .validate_against_sources(&lifted, [0, 1, 2], [3, 4, 5], APPROX)
        .expect("coplanar classifier must validate and replay");
    if let Some(tri_tri) = classify_triangle_triangle3(
        &lifted[0], &lifted[1], &lifted[2], &lifted[3], &lifted[4], &lifted[5], APPROX,
    )
    .value()
    {
        tri_tri
            .validate_against_triangles(
                [&lifted[0], &lifted[1], &lifted[2]],
                [&lifted[3], &lifted[4], &lifted[5]],
                APPROX,
            )
            .expect("triangle/triangle report must replay against exact sources");
        let swapped = classify_triangle_triangle3(
            &lifted[3], &lifted[4], &lifted[5], &lifted[0], &lifted[1], &lifted[2], APPROX,
        )
        .value()
        .expect("swapped exact triangle pair should decide");
        assert_eq!(
            tri_tri.relation, swapped.relation,
            "triangle/triangle relation must be symmetric under pair exchange"
        );
    }

    let exact_half = (Real::from(1) / &Real::from(2)).expect("half is rational");
    assert_eq!(
        projected_segment_parameter3(
            &Point3::new(2.into(), 0.into(), 0.into()),
            &Point3::new(0.into(), 0.into(), 0.into()),
            &Point3::new(4.into(), 0.into(), 0.into()),
            CoplanarProjection::Xy,
            APPROX
        ),
        Some(exact_half.clone()),
        "projected segment parameter should preserve exact affine ratios"
    );
    assert_eq!(
        projected_line_parameter3(
            &Point3::new(0.into(), (-2).into(), 0.into()),
            &Point3::new(0.into(), 2.into(), 0.into()),
            &Point3::new((-1).into(), 0.into(), 0.into()),
            &Point3::new(1.into(), 0.into(), 0.into()),
            CoplanarProjection::Xy,
            APPROX
        ),
        Some(exact_half),
        "projected line crossing parameter should preserve determinant ratios"
    );

    let unit_x_from_a = Point2::new(&a.x + &Real::from(1), a.y.clone());
    assert_eq!(
        classify_circle_line2(&a, &zero, &a, &unit_x_from_a, APPROX).value(),
        Some(hyperlimit::CircleLineRelation::Tangent),
        "zero-radius circle centered on a nondegenerate line has one boundary contact"
    );
    let circle_line_batch = [(a.clone(), zero.clone(), a.clone(), unit_x_from_a.clone())];
    assert_eq!(
        classify_circle_line2_batch(&circle_line_batch, APPROX)[0].value(),
        classify_circle_line2(&a, &zero, &a, &unit_x_from_a, APPROX).value(),
        "circle/line batch relation must match scalar relation"
    );
    assert_eq!(
        classify_circle_segment2(&a, &zero, &a, &a, APPROX).value(),
        Some(hyperlimit::CircleSegmentRelation::Tangent),
        "zero-radius circle and degenerate segment at the center touch exactly once"
    );
    let circle_segment_batch = [(a.clone(), zero.clone(), a.clone(), a.clone())];
    assert_eq!(
        classify_circle_segment2_batch(&circle_segment_batch, APPROX)[0].value(),
        classify_circle_segment2(&a, &zero, &a, &a, APPROX).value(),
        "circle/segment batch relation must match scalar relation"
    );

    let unit_square = vec![
        Point2::new(0.into(), 0.into()),
        Point2::new(1.into(), 0.into()),
        Point2::new(1.into(), 1.into()),
        Point2::new(0.into(), 1.into()),
    ];
    assert_eq!(
        classify_point_convex_polygon2(&unit_square, &Point2::new(0.into(), 0.into()), APPROX)
            .value(),
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
        classify_point_convex_planes3(
            &unit_cube_planes,
            &Point3::new(0.into(), 0.into(), 0.into()),
            APPROX
        )
        .value(),
        Some(hyperlimit::ConvexPointLocation::Boundary),
        "convex plane composition must retain exact boundary points"
    );

    let dop_axes = [
        Point3::new(1.into(), 0.into(), 0.into()),
        Point3::new(0.into(), 1.into(), 0.into()),
        Point3::new(0.into(), 0.into(), 1.into()),
        Point3::new(1.into(), 1.into(), 1.into()),
    ];
    let dop_points = [p.clone(), q.clone(), r.clone(), s.clone(), t.clone()];
    if let Some(dop) = SupportDop3::from_points(&dop_axes, &dop_points, APPROX).value() {
        for point in &dop_points {
            assert!(
                dop.classify_point(point, APPROX)
                    .value()
                    .is_some_and(|location| location.is_inside_or_boundary()),
                "support k-DOP built from exact points must contain every source witness"
            );
        }
        for slab in dop.slabs() {
            assert!(
                slab.min_witness.is_some() && slab.max_witness.is_some(),
                "support slabs should retain source witnesses"
            );
            let min_witness = &dop_points[slab.min_witness.expect("checked min witness")];
            let max_witness = &dop_points[slab.max_witness.expect("checked max witness")];
            assert_eq!(
                slab.project_point(min_witness),
                slab.min,
                "min support witness projection must replay exactly"
            );
            assert_eq!(
                slab.project_point(max_witness),
                slab.max,
                "max support witness projection must replay exactly"
            );
        }
    }

    let unit_dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(
            Point3::new(1.into(), 0.into(), 0.into()),
            0.into(),
            1.into(),
        ),
        SupportSlab3::new(
            Point3::new(0.into(), 1.into(), 0.into()),
            0.into(),
            1.into(),
        ),
        SupportSlab3::new(
            Point3::new(0.into(), 0.into(), 1.into()),
            0.into(),
            1.into(),
        ),
    ]);
    assert_eq!(
        unit_dop
            .classify_aabb3(
                &Point3::new(1.into(), 0.into(), 0.into()),
                &Point3::new(2.into(), 1.into(), 1.into()),
                APPROX,
            )
            .value(),
        Some(SupportDopRelation::BoundaryTouch),
        "AABB sharing a support plane must be a boundary touch, not separated"
    );
    if let Some(report) = unit_dop
        .classify_aabb3_report(
            &Point3::new(1.into(), 0.into(), 0.into()),
            &Point3::new(2.into(), 1.into(), 1.into()),
            APPROX,
        )
        .value()
    {
        assert_eq!(
            report.relation,
            SupportDopRelation::BoundaryTouch,
            "report relation must match the coarse support-DOP/AABB classifier"
        );
        assert!(
            report
                .validate_against_sources(
                    &unit_dop,
                    &Point3::new(1.into(), 0.into(), 0.into()),
                    &Point3::new(2.into(), 1.into(), 1.into()),
                    APPROX,
                )
                .is_ok(),
            "support-DOP/AABB report evidence must replay from exact sources"
        );
    }
    assert_eq!(
        unit_dop
            .classify_aabb3(
                &Point3::new(2.into(), 0.into(), 0.into()),
                &Point3::new(3.into(), 1.into(), 1.into()),
                APPROX,
            )
            .value(),
        Some(SupportDopRelation::Separated),
        "a separating support axis must produce an exact separated relation"
    );
    if let Some(report) = unit_dop
        .classify_aabb3_report(
            &Point3::new(2.into(), 0.into(), 0.into()),
            &Point3::new(3.into(), 1.into(), 1.into()),
            APPROX,
        )
        .value()
    {
        assert_eq!(report.terminal_slab, Some(0));
        assert!(
            report
                .validate_against_sources(
                    &unit_dop,
                    &Point3::new(2.into(), 0.into(), 0.into()),
                    &Point3::new(3.into(), 1.into(), 1.into()),
                    APPROX,
                )
                .is_ok(),
            "separating support-DOP/AABB report must replay its terminal slab"
        );
    }
    let unit_plane = Plane3::new(Point3::new(1.into(), 0.into(), 0.into()), (-1).into());
    if let Some(report) = unit_dop.classify_plane3_report(&unit_plane, APPROX).value() {
        assert_eq!(
            report.relation,
            SupportDopPlaneRelation::Intersecting,
            "unit support DOP must touch the x=1 query plane"
        );
        assert!(
            report
                .validate_against_sources(&unit_dop, &unit_plane, APPROX,)
                .is_ok(),
            "support-DOP/plane feasibility evidence must replay from exact sources"
        );
    }
    let outside_plane = Plane3::new(Point3::new(1.into(), 0.into(), 0.into()), (-2).into());
    if let Some(report) = unit_dop
        .classify_plane3_report(&outside_plane, APPROX)
        .value()
    {
        assert_eq!(
            report.relation,
            SupportDopPlaneRelation::Below,
            "unit support DOP must lie below the x=2 query plane"
        );
        assert!(
            report
                .validate_against_sources(&unit_dop, &outside_plane, APPROX,)
                .is_ok(),
            "one-sided support-DOP/plane evidence must replay"
        );
    }

    let fixed_point_halfspaces = vec![
        Plane3::new(Point3::new(1.into(), 0.into(), 0.into()), -&p.x),
        Plane3::new(Point3::new((-1).into(), 0.into(), 0.into()), p.x.clone()),
        Plane3::new(Point3::new(0.into(), 1.into(), 0.into()), -&p.y),
        Plane3::new(Point3::new(0.into(), (-1).into(), 0.into()), p.y.clone()),
        Plane3::new(Point3::new(0.into(), 0.into(), 1.into()), -&p.z),
        Plane3::new(Point3::new(0.into(), 0.into(), (-1).into()), p.z.clone()),
    ];
    if let Some(feasibility) =
        classify_halfspace_feasibility3(&fixed_point_halfspaces, APPROX).value()
    {
        assert!(
            feasibility.is_feasible(),
            "coordinate halfspaces that pin a generated point must be feasible"
        );
        assert_eq!(
            feasibility
                .validate_against_planes(&fixed_point_halfspaces, APPROX,)
                .value(),
            Some(true),
            "halfspace feasibility witness must replay through point-plane predicates"
        );
    }
    let impossible_halfspaces = vec![
        Plane3::new(Point3::new(1.into(), 0.into(), 0.into()), 1.into()),
        Plane3::new(Point3::new((-1).into(), 0.into(), 0.into()), 0.into()),
    ];
    if let Some(report) = classify_halfspace_feasibility3(&impossible_halfspaces, APPROX).value() {
        assert_eq!(
            report.status,
            hyperlimit::HalfspaceFeasibility::Infeasible,
            "opposed exact halfspaces x <= -1 and x >= 0 must be infeasible"
        );
        assert!(
            report.infeasibility_certificate.is_some(),
            "opposed exact halfspaces should retain a Farkas certificate"
        );
        assert_eq!(
            report
                .validate_against_planes(&impossible_halfspaces, APPROX,)
                .value(),
            Some(true),
            "halfspace infeasibility certificate must replay exactly"
        );
    }
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
