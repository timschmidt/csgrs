#[cfg(feature = "triangulation")]
use hypercurve::triangulate_finite_rings;
use hypercurve::{
    BooleanOp, BulgeVertex2, CircularArc2, Contour2, Curve2, CurveContext, CurveError, CurvePath2,
    CurveRegion2, LineSeg2, NurbsCurve2, OffsetCap, OffsetCornerStyle2, Point2, QuadraticBezier2,
    Real, Similarity2, StraightSkeletonStage2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(xmin, ymin), r(0)),
        BulgeVertex2::new(p(xmax, ymin), r(0)),
        BulgeVertex2::new(p(xmax, ymax), r(0)),
        BulgeVertex2::new(p(xmin, ymax), r(0)),
    ])
    .expect("trace rectangle is valid")
}

fn star(vertex_count: usize, center: (f64, f64), radii: (f64, f64), rotation: f64) -> Contour2 {
    let vertices = (0..vertex_count)
        .map(|index| {
            let angle = rotation + std::f64::consts::TAU * index as f64 / vertex_count as f64;
            let radius = if index % 2 == 0 { radii.0 } else { radii.1 };
            let point = Point2::new(
                Real::try_from(center.0 + radius * angle.cos()).expect("finite star x"),
                Real::try_from(center.1 + radius * angle.sin()).expect("finite star y"),
            );
            BulgeVertex2::new(point, Real::zero())
        })
        .collect::<Vec<_>>();
    Contour2::from_bulge_vertices(&vertices).expect("trace star is valid")
}

fn trace<T, E: std::fmt::Display>(name: &str, workload: impl FnOnce() -> Result<T, E>) {
    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(workload)
        .unwrap_or_else(|error| panic!("{name} trace workload must remain certified: {error}"));
    std::hint::black_box(result);
    let snapshot = hyperreal::dispatch_trace::take_trace();
    let correlation = snapshot.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "{name} did not emit an exact-computation path trace"
    );
    println!(
        "{name}: correlation={correlation:?}, rational={:?}",
        snapshot.rational
    );
    for summary in snapshot.operation_summaries() {
        println!(
            "  {}/{}/{}",
            summary.layer, summary.operation, summary.count
        );
    }
}

fn main() {
    let policy = CurveContext::STRICT;
    let horizontal = LineSeg2::try_new(p(-4, 0), p(4, 0)).expect("line is valid");
    let vertical = LineSeg2::try_new(p(0, -4), p(0, 4)).expect("line is valid");
    trace("line_line_intersection", || {
        horizontal.intersect_line(&vertical, &policy)
    });

    let first_arc =
        CircularArc2::try_from_center(p(4, 0), p(-4, 0), p(0, 0), false).expect("arc is valid");
    let second_arc =
        CircularArc2::try_from_center(p(8, 0), p(0, 0), p(4, 0), false).expect("arc is valid");
    trace("arc_arc_intersection", || {
        first_arc.intersect_arc(&second_arc, &policy)
    });

    let quadratic = QuadraticBezier2::new(p(-3, 0), p(0, 6), p(3, 0));
    trace("quadratic_bezier_evaluation", || {
        Ok::<_, CurveError>(quadratic.point_at((r(1) / r(3))?))
    });

    trace("exact_similarity_transform", || {
        let transform = Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(5), r(7))?;
        Ok::<_, CurveError>(transform.transform_point(&p(3, 4)))
    });

    trace("finite_ring_import", || {
        let points = (0..64)
            .map(|index| {
                let angle = std::f64::consts::TAU * index as f64 / 64.0;
                [angle.cos(), angle.sin()]
            })
            .collect::<Vec<_>>();
        Contour2::from_finite_ring(&points)
    });

    trace("nurbs_global_interpolation", || {
        Ok::<_, CurveError>(
            NurbsCurve2::interpolate_uniform(
                2,
                vec![p(0, 0), p(2, 2), p(4, 0)],
                &CurveContext::STRICT,
            )
            .expect("trace interpolation remains exact")
            .into_value(),
        )
    });

    let open_path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).expect("line is valid")),
        Curve2::from(LineSeg2::try_new(p(4, 0), p(4, 3)).expect("line is valid")),
    ])
    .expect("path is connected");
    trace("curve_region_path_stroke", || {
        CurveRegion2::stroke_path(
            &open_path,
            r(1),
            &OffsetCornerStyle2::Round,
            OffsetCap::Round,
            &policy,
        )
    });

    let concave = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), r(0)),
        BulgeVertex2::new(p(30, 0), r(0)),
        BulgeVertex2::new(p(30, 24), r(0)),
        BulgeVertex2::new(p(20, 24), r(0)),
        BulgeVertex2::new(p(20, 7), r(0)),
        BulgeVertex2::new(p(17, 11), r(0)),
        BulgeVertex2::new(p(17, 24), r(0)),
        BulgeVertex2::new(p(0, 24), r(0)),
    ])
    .expect("trace concave contour is valid");
    trace("straight_skeleton", || {
        let evidence = concave.straight_skeleton(&policy)?;
        assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
        Ok::<_, CurveError>(evidence)
    });

    let first =
        CurveRegion2::try_from_native_material_contours(vec![rectangle(0, 0, 4, 4)], &policy)
            .expect("trace region is valid")
            .into_value();
    let second =
        CurveRegion2::try_from_native_material_contours(vec![rectangle(2, -1, 6, 3)], &policy)
            .expect("trace region is valid")
            .into_value();
    trace("region_boolean", || {
        first.boolean_region(&second, BooleanOp::Union, &policy)
    });

    let first_star = CurveRegion2::try_from_native_material_contours(
        vec![star(64, (0.0, 0.0), (100.0, 72.0), 0.0)],
        &policy,
    )
    .expect("trace star is valid")
    .into_value();
    let second_star = CurveRegion2::try_from_native_material_contours(
        vec![star(
            64,
            (18.0, 7.0),
            (96.0, 68.0),
            std::f64::consts::PI / 64.0,
        )],
        &policy,
    )
    .expect("trace star is valid")
    .into_value();
    trace("star64_region_boolean", || {
        first_star.boolean_region(&second_star, BooleanOp::Intersection, &policy)
    });

    trace("region_containment", || {
        first.classify_point(&p(1, 1), &policy)
    });

    #[cfg(feature = "triangulation")]
    trace("finite_ring_triangulation", || {
        let material = [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]];
        let hole = [[2.0, 2.0], [2.0, 4.0], [6.0, 4.0], [6.0, 2.0]];
        triangulate_finite_rings(&material, &[&hole], &policy)
    });
}
