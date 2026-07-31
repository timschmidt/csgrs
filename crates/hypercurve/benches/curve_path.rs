use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveBoundaryInteriorSide2,
    CurveContext, CurvePath2, LineSeg2, Point2, QuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("benchmark denominator is nonzero")
}

fn rectangle(x0: i32, y0: i32, x1: i32, y1: i32) -> CurvePath2 {
    let points = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
    CurvePath2::try_new(
        (0..4)
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(points[index].clone(), points[(index + 1) % 4].clone())
                        .expect("benchmark rectangle edges are nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark rectangle is connected")
}

fn full_circle(center_x: i32) -> Curve2 {
    Curve2::from(
        CircularArc2::try_from_center(
            p(center_x + 1, 0),
            p(center_x + 1, 0),
            p(center_x, 0),
            false,
        )
        .expect("benchmark circle is valid"),
    )
}

fn closed_under_cubic(curve: CubicBezier2, lower_y: i32) -> CurvePath2 {
    let start = curve.start().clone();
    let end = curve.end().clone();
    let lower_end = Point2::new(end.x().clone(), r(lower_y));
    let lower_start = Point2::new(start.x().clone(), r(lower_y));
    CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(LineSeg2::try_new(end, lower_end.clone()).expect("benchmark side is nonzero")),
        Curve2::from(
            LineSeg2::try_new(lower_end, lower_start.clone()).expect("benchmark base is nonzero"),
        ),
        Curve2::from(LineSeg2::try_new(lower_start, start).expect("benchmark side is nonzero")),
    ])
    .expect("benchmark curved path is connected")
}

fn main() {
    let policy = CurveContext::STRICT;
    let first = rectangle(0, 0, 2, 2);
    let second = rectangle(1, -1, 3, 1);

    let promotion_iterations = 20_000_u32;
    let promotion_cache_iterations = 2_000_000_u32;
    first
        .native_bezier_fragments(&policy)
        .expect("path promotion is exact");
    let started = Instant::now();
    let mut promotion_checksum = 0_usize;
    for _ in 0..promotion_cache_iterations {
        promotion_checksum = promotion_checksum.wrapping_add(black_box(
            black_box(&first)
                .native_bezier_fragments(&policy)
                .expect("cached path promotion remains exact")
                .into_value()
                .len(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_native_promotion: {promotion_cache_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={promotion_checksum}",
        elapsed / promotion_cache_iterations
    );

    first
        .bezier_boundary_loop(&policy)
        .expect("benchmark boundary materialization is exact");
    let boundary_cache_iterations = 2_000_000_u32;
    let started = Instant::now();
    let mut boundary_checksum = 0_usize;
    for _ in 0..boundary_cache_iterations {
        boundary_checksum ^= black_box(
            first
                .bezier_boundary_loop(&policy)
                .expect("cached path boundary remains exact")
                .into_value()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_boundary_loop: {boundary_cache_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={boundary_checksum}",
        elapsed / boundary_cache_iterations
    );

    let boundary_build_iterations = 20_000_u32;
    let rectangle_curves = first.curves().to_vec();
    let started = Instant::now();
    let mut boundary_build_checksum = 0_usize;
    for _ in 0..boundary_build_iterations {
        let path = CurvePath2::try_new(rectangle_curves.clone())
            .expect("fresh benchmark path is connected");
        boundary_build_checksum ^= black_box(
            path.bezier_boundary_loop(&policy)
                .expect("fresh benchmark boundary is exact")
                .into_value()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_immediate_boundary_loop: {boundary_build_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={boundary_build_checksum}",
        elapsed / boundary_build_iterations
    );

    let query = p(1, 1);
    let classification_iterations = 50_000_u32;
    let started = Instant::now();
    let mut classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let classification = first
            .classify_point(black_box(&query), &policy)
            .expect("benchmark path classification is exact")
            .into_value();
        let Classification::Decided(location) = classification else {
            panic!("benchmark path classification became uncertain");
        };
        classification_checksum ^= black_box(location as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_point_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={classification_checksum}",
        elapsed / classification_iterations
    );

    let native = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0)));
    native
        .native_bezier_fragments(&policy)
        .expect("benchmark native curve promotes exactly");
    let started = Instant::now();
    let mut full_trim_checksum = 0_usize;
    for _ in 0..promotion_iterations {
        let trimmed = native
            .subcurve(r(0), r(1), &policy)
            .expect("full-domain trim is exact")
            .into_value();
        full_trim_checksum ^= black_box(
            trimmed
                .native_bezier_fragments(&policy)
                .unwrap()
                .into_value()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_cached_full_domain_subcurve: {promotion_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={full_trim_checksum}",
        elapsed / promotion_iterations
    );

    let native_split_iterations = 5_000_u32;
    let started = Instant::now();
    let mut native_split_checksum = 0_usize;
    for _ in 0..native_split_iterations {
        let (left, right) = native
            .split_at(q(1, 2), &policy)
            .expect("native benchmark split is exact")
            .into_value();
        native_split_checksum ^= black_box(left.family() as usize ^ right.family() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "curve_native_exact_split: {native_split_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={native_split_checksum}",
        elapsed / native_split_iterations
    );

    let spline = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(3), r(4)],
        vec![r(0), r(1), r(2), r(3), r(4), r(5), r(6)],
    )
    .expect("benchmark NURBS is valid");
    let spline_split_iterations = 1_000_u32;
    let started = Instant::now();
    let mut spline_split_checksum = 0_usize;
    for _ in 0..spline_split_iterations {
        let (left, right) = spline
            .split_at(r(3), &policy)
            .expect("spline benchmark split is exact")
            .into_value();
        spline_split_checksum ^= black_box(
            left.native_bezier_fragments(&policy)
                .unwrap()
                .into_value()
                .len()
                + right
                    .native_bezier_fragments(&policy)
                    .unwrap()
                    .into_value()
                    .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_nurbs_exact_split: {spline_split_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={spline_split_checksum}",
        elapsed / spline_split_iterations
    );

    let replay_iterations = 20_000_u32;
    let immediate_iterations = 2_000_u32;
    let started = Instant::now();
    let mut evidence_checksum = 0_usize;
    for _ in 0..immediate_iterations {
        let evidence = first
            .intersect_path(&second, &policy)
            .expect("benchmark path evidence is complete")
            .into_value();
        evidence_checksum ^= black_box(
            evidence.candidate_curve_pair_count()
                + evidence.contacts().len()
                + evidence.overlaps().len()
                + evidence.blockers().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_immediate_intersection_evidence: {immediate_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={evidence_checksum}",
        elapsed / immediate_iterations
    );

    let started = Instant::now();
    let mut topology_checksum = 0_usize;
    for _ in 0..immediate_iterations {
        let topology = first
            .intersection_topology(&second, &policy)
            .expect("benchmark path topology is complete")
            .into_value();
        topology_checksum ^= black_box(
            topology.first().len()
                + topology.second().len()
                + topology.arrangement_graph_view().unwrap().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_immediate_intersection_topology: {immediate_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={topology_checksum}",
        elapsed / immediate_iterations
    );

    let partial_first = rectangle(0, 0, 2, 4);
    let partial_second = rectangle(2, 1, 4, 3);
    let boolean_iterations = 500_u32;
    let started = Instant::now();
    let mut boolean_checksum = 0_usize;
    for _ in 0..boolean_iterations {
        let selections = partial_first
            .boolean_selections(
                &partial_second,
                CurveBoundaryInteriorSide2::Left,
                CurveBoundaryInteriorSide2::Left,
                &policy,
            )
            .expect("all immediate partial-overlap Booleans are exact")
            .into_value();
        boolean_checksum ^= black_box(
            selections.union().kept_fragment_count()
                + selections.intersection().kept_fragment_count()
                + selections.difference().kept_fragment_count()
                + selections.xor().kept_fragment_count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_immediate_boolean_batch: {boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={boolean_checksum}",
        elapsed / boolean_iterations
    );

    let partial_union = partial_first
        .boolean_selection(
            &partial_second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .expect("partial-overlap union selection is exact")
        .into_value();
    partial_union
        .region_view()
        .expect("partial-overlap union region is exact");
    let started = Instant::now();
    let mut partial_checksum = 0_usize;
    for _ in 0..replay_iterations {
        let region = partial_union
            .region_view()
            .expect("cached partial-overlap union remains exact");
        partial_checksum ^=
            black_box(partial_union.kept_fragment_count() + region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_partial_line_overlap_union: {replay_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={partial_checksum}",
        elapsed / replay_iterations
    );

    let partial_arc_first = CurvePath2::try_new(vec![
        Curve2::from(
            CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false)
                .expect("benchmark semicircle is valid"),
        ),
        Curve2::from(LineSeg2::try_new(p(-5, 0), p(5, 0)).expect("benchmark diameter is nonzero")),
    ])
    .expect("benchmark semicircle path is connected");
    let partial_arc_second = CurvePath2::try_new(vec![
        Curve2::from(
            CircularArc2::try_from_center(p(4, 3), p(0, 5), p(0, 0), false)
                .expect("benchmark subarc is valid"),
        ),
        Curve2::from(
            LineSeg2::try_new(p(0, 5), p(4, 3)).expect("benchmark subarc chord is nonzero"),
        ),
    ])
    .expect("benchmark circular-segment path is connected");
    let partial_arc_union = partial_arc_first
        .boolean_selection(
            &partial_arc_second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .expect("partial-arc union selection is exact")
        .into_value();
    partial_arc_union
        .region_view()
        .expect("partial-arc union region is exact");
    let started = Instant::now();
    let mut partial_arc_checksum = 0_usize;
    for _ in 0..replay_iterations {
        let region = partial_arc_union
            .region_view()
            .expect("cached partial-arc union remains exact");
        partial_arc_checksum ^=
            black_box(partial_arc_union.kept_fragment_count() + region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_partial_arc_overlap_union: {replay_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={partial_arc_checksum}",
        elapsed / replay_iterations
    );

    let nonlinear_source = CubicBezier2::new(p(0, 0), p(1, 3), p(3, 3), p(4, 0));
    let nonlinear_first = closed_under_cubic(
        nonlinear_source
            .subcurve_between_exact(&r(0), &q(3, 4), &policy)
            .expect("benchmark cubic subcurve is exact"),
        -5,
    );
    let nonlinear_second = closed_under_cubic(
        nonlinear_source
            .subcurve_between_exact(&q(3, 8), &q(7, 8), &policy)
            .expect("benchmark cubic subcurve is exact"),
        -6,
    );
    let nonlinear_union = nonlinear_first
        .boolean_selection(
            &nonlinear_second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Right,
            CurveBoundaryInteriorSide2::Right,
            &policy,
        )
        .expect("partial nonlinear-overlap union selection is exact")
        .into_value();
    nonlinear_union
        .region_view()
        .expect("partial nonlinear-overlap union region is exact");
    let started = Instant::now();
    let mut nonlinear_checksum = 0_usize;
    for _ in 0..replay_iterations {
        let region = nonlinear_union
            .region_view()
            .expect("cached partial nonlinear-overlap union remains exact");
        nonlinear_checksum ^=
            black_box(nonlinear_union.kept_fragment_count() + region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_partial_nonlinear_overlap_union: {replay_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={nonlinear_checksum}",
        elapsed / replay_iterations
    );

    let lineage_source = Curve2::from(CubicBezier2::new(p(0, 0), p(1, 3), p(3, 3), p(4, 0)));
    let lineage_first = lineage_source
        .subcurve(r(0), q(3, 4), &policy)
        .expect("benchmark source trim is exact")
        .into_value();
    let lineage_second = lineage_source
        .subcurve(q(1, 4), r(1), &policy)
        .expect("benchmark source trim is exact")
        .into_value();
    let lineage_iterations = 5_000_u32;
    let started = Instant::now();
    let mut lineage_checksum = 0_usize;
    for _ in 0..lineage_iterations {
        lineage_checksum ^= black_box(
            lineage_first
                .intersect_curve(&lineage_second, &policy)
                .expect("lineage evidence is exact")
                .into_value()
                .overlaps()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_pair_immediate_lineage_partial_nonlinear_overlap: {lineage_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={lineage_checksum}",
        elapsed / lineage_iterations
    );

    let first_circle = full_circle(0);
    let second_circle = full_circle(1);
    let native_iterations = 500_u32;
    let started = Instant::now();
    let mut native_checksum = 0_usize;
    for _ in 0..native_iterations {
        let evidence = first_circle
            .intersect_curve(&second_circle, &policy)
            .expect("native circle evidence is complete")
            .into_value();
        native_checksum ^= black_box(evidence.contacts().len() + evidence.span_pair_count());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_pair_native_circle_dispatch: {native_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={native_checksum}",
        elapsed / native_iterations
    );

    let first_circle_path =
        CurvePath2::try_new(vec![first_circle]).expect("benchmark circle path is connected");
    let second_circle_path =
        CurvePath2::try_new(vec![second_circle]).expect("benchmark circle path is connected");
    let circle_union = first_circle_path
        .boolean_selection(
            &second_circle_path,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .expect("benchmark circle union is exact")
        .into_value();
    circle_union
        .region_view()
        .expect("benchmark circle region is retained");
    let started = Instant::now();
    let mut circle_checksum = 0_usize;
    for _ in 0..replay_iterations {
        let region = circle_union
            .region_view()
            .expect("cached circle region remains exact");
        circle_checksum ^= black_box(region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_cached_circle_union_region: {replay_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={circle_checksum}",
        elapsed / replay_iterations
    );
}
