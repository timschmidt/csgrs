#[cfg(feature = "triangulation")]
use hypercurve::triangulate_finite_rings;
use hypercurve::{
    BooleanOp, BulgeVertex2, CircularArc2, Classification, Contour2, CurvePolicy, CurveResult,
    CurveString2, FillRule, LineSeg2, NurbsCurve2, Point2, QuadraticBezier2, Real, Region2,
    Segment2, Similarity2,
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

fn trace<T>(name: &str, workload: impl FnOnce() -> CurveResult<T>) {
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
    println!("{name}: correlation={correlation:?}");
    for summary in snapshot.operation_summaries() {
        println!(
            "  {}/{}/{}",
            summary.layer, summary.operation, summary.count
        );
    }
}

fn main() {
    let policy = CurvePolicy::certified();
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
        Ok(quadratic.point_at((r(1) / r(3))?))
    });

    trace("exact_similarity_transform", || {
        let transform = Similarity2::try_from_real_affine(r(0), r(-1), r(1), r(0), r(5), r(7))?;
        Ok(transform.transform_point(&p(3, 4)))
    });

    trace("finite_ring_import", || {
        let points = (0..64)
            .map(|index| {
                let angle = std::f64::consts::TAU * index as f64 / 64.0;
                [angle.cos(), angle.sin()]
            })
            .collect::<Vec<_>>();
        Contour2::import_finite_ring(&points)
    });

    trace("nurbs_global_interpolation", || {
        Ok(
            NurbsCurve2::interpolate_uniform(2, vec![p(0, 0), p(2, 2), p(4, 0)])
                .expect("trace interpolation remains exact"),
        )
    });

    let open_path = CurveString2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(0, 0), p(4, 0)).expect("line is valid")),
        Segment2::Line(LineSeg2::try_new(p(4, 0), p(4, 3)).expect("line is valid")),
    ])
    .expect("path is connected");
    trace("curve_string_offset", || {
        let offset = open_path.offset_left_checked(r(1), &policy)?;
        assert!(matches!(offset, Classification::Decided(_)));
        Ok(offset)
    });

    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    trace("region_boolean", || {
        let result = first.boolean_region_with_report(
            &second,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy,
        )?;
        assert!(result.region().is_some());
        Ok(result)
    });

    let prepared = first.prepare_topology_queries(&policy);
    trace("prepared_region_containment", || {
        Ok(prepared.classify_point(&p(1, 1), &policy))
    });

    #[cfg(feature = "triangulation")]
    trace("finite_ring_triangulation", || {
        let material = [[0.0, 0.0], [8.0, 0.0], [8.0, 6.0], [0.0, 6.0]];
        let hole = [[2.0, 2.0], [2.0, 4.0], [6.0, 4.0], [6.0, 2.0]];
        triangulate_finite_rings(&material, &[&hole])
    });
}
