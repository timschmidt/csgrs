use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ArcDirection, BeadFillAxis, BezierParameter, CardinalPoint, CardinalRotation, CircularArc,
    ConstructionStamp, CubicBezier, ExplicitCircularArc, FeedPathElement, HigherOrderBezier,
    LinePathSegment, LookaheadFeedSchedule, MeanderKeepout, MeanderObstacle,
    MeanderPlacementCandidate, NetId, OffsetSide, PathProvenance, PathSourceFormat,
    PcbBoardOutline, PcbCardinalRectPad, PcbCircularBoardOutline, PcbCircularPad,
    PcbConvexBoardOutline, PcbConvexPad, PcbObroundPad, PcbOrientedRectPad,
    PcbOrthogonalBoardOutline, PcbOrthogonalPad, PcbRectPad, PcbRoundedRectPad, PcbTrace,
    PcbViaStack, QuadraticBezier, RationalQuadraticBezier, RectangularPocket, SourceLengthUnit,
    SpecctraGridKeepoutRecord, SpecctraGridKeepoutShape, SpecctraGridTraceRecord,
    SpecctraGridViaRecord, SpecctraLayerAlias, SpecctraNetAlias, SweptLineSegment, TangentSpan,
    TraceLayer, ViaDrillIntent, arrange_cubic_beziers, arrange_explicit_arcs,
    arrange_line_segments, arrange_line_segments_with_explicit_arcs,
    arrange_line_segments_with_quadratic_beziers,
    arrange_line_segments_with_rational_quadratic_beziers, arrange_quadratic_beziers,
    arrange_rational_quadratic_beziers, build_alternating_detour_meander, build_g1_join_problem,
    build_keepout_aware_detour_meander, build_length_match_problem, build_multi_detour_meander,
    build_nonuniform_detour_meander, build_obstacle_aware_detour_meander,
    build_oriented_tangent_alignment_problem, build_rectangular_bead_plan,
    build_rectangular_pocket_link_graph, build_rectangular_pocket_plan,
    build_rectangular_serpentine_infill_graph, build_rectangular_support_plan,
    build_single_detour_meander, build_tangent_alignment_problem,
    certify_acceleration_limited_feed_time, certify_acceleration_limited_feed_time_for_path,
    certify_constant_feed_time, certify_constant_feed_time_for_path,
    certify_corner_lookahead_limits, certify_differential_pair_skew, certify_g1_chain,
    certify_g1_join_candidate, certify_length_extension, certify_lookahead_feed_schedule,
    certify_symmetric_jerk_limited_feed_time, certify_symmetric_jerk_limited_feed_time_for_path,
    certify_tangent_alignment_candidate, check_cardinal_rect_pad_board_clearance,
    check_circular_pad_board_clearance, check_circular_pad_circular_board_clearance,
    check_convex_pad_board_clearance, check_obround_pad_board_clearance,
    check_oriented_rect_pad_board_clearance, check_orthogonal_pad_board_clearance,
    check_rect_pad_board_clearance, check_rounded_rect_pad_board_clearance,
    check_trace_board_clearance, check_trace_cardinal_rect_pad_clearance,
    check_trace_circular_board_clearance, check_trace_clearance,
    check_trace_convex_board_clearance, check_trace_convex_pad_clearance,
    check_trace_obround_pad_clearance, check_trace_oriented_rect_pad_clearance,
    check_trace_orthogonal_board_clearance, check_trace_orthogonal_pad_clearance,
    check_trace_pad_clearance, check_trace_rect_pad_clearance,
    check_trace_rounded_rect_pad_clearance, check_trace_via_clearance,
    check_trace_via_drill_clearance, check_via_drill_board_clearance,
    classify_meander_candidate_slots, classify_meander_placement_slots,
    classify_meander_placement_slots_with_keepouts, classify_tangent_alignment,
    classify_tangent_chain, classify_tangent_join, import_specctra_trace_record,
    import_specctra_via_record, intersect_axis_aligned_line_quadratic_bezier,
    intersect_axis_aligned_line_rational_quadratic_bezier, intersect_rectangular_regions,
    offset_axis_aligned_segment, offset_cardinal_arc, offset_cubic_bezier_sample,
    offset_explicit_arc, offset_higher_order_bezier_sample, offset_quadratic_bezier_sample,
    parse_specctra_grid_route_records, parse_specctra_grid_trace_records,
    serialize_specctra_grid_keepout_records, serialize_specctra_grid_route_records,
    serialize_specctra_grid_trace_records, serialize_specctra_grid_via_records,
    specctra_grid_keepout_record, specctra_grid_trace_record, specctra_grid_via_record,
    subtract_rectangular_region,
};
use hyperreal::{Rational, Real};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn rq(numerator: i64, denominator: i64) -> Real {
    Real::new(Rational::new(numerator) / Rational::new(denominator))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn pq(x_num: i64, x_den: i64, y_num: i64, y_den: i64) -> Point2 {
    Point2::new(rq(x_num, x_den), rq(y_num, y_den))
}

fn trace(net: u32, start: Point2, end: Point2) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(0),
        SweptLineSegment::new(LinePathSegment::new(start, end), r(2)).unwrap(),
    )
}

fn path_predicates(c: &mut Criterion) {
    let provenance = PathProvenance::fixed_grid_with_unit(
        PathSourceFormat::Gerber,
        1_000_000,
        SourceLengthUnit::Millimeter,
    )
    .unwrap();
    c.bench_function("source_grid_token_exact_lift", |b| {
        b.iter(|| provenance.real_from_units(123_456))
    });
    let stamp = ConstructionStamp::new(17, 29);
    let stamped = provenance.with_construction(stamp);
    c.bench_function("construction_freshness_check", |b| {
        b.iter(|| stamped.is_fresh_for(stamp))
    });
    let tangent_segment = LinePathSegment::new(p(0, 0), p(1000, 250));
    c.bench_function("line_segment_exact_tangent", |b| {
        b.iter(|| tangent_segment.start_tangent())
    });
    c.bench_function("tangent_alignment_exact_predicate", |b| {
        b.iter(|| classify_tangent_alignment(&p(3, 4), &p(6, 8), PredicatePolicy::default()))
    });
    c.bench_function("tangent_join_exact_predicate", |b| {
        b.iter(|| {
            classify_tangent_join(
                &p(10, 20),
                &p(3, 4),
                &p(10, 20),
                &p(6, 8),
                PredicatePolicy::default(),
            )
        })
    });
    let tangent_chain = vec![
        TangentSpan {
            start: p(0, 0),
            start_tangent: p(3, 4),
            end: p(10, 20),
            end_tangent: p(3, 4),
        },
        TangentSpan {
            start: p(10, 20),
            start_tangent: p(6, 8),
            end: p(30, 40),
            end_tangent: p(6, 8),
        },
    ];
    c.bench_function("tangent_chain_exact_predicate", |b| {
        b.iter(|| classify_tangent_chain(&tangent_chain, PredicatePolicy::default()))
    });
    c.bench_function("g1_chain_hypersolve_certification", |b| {
        b.iter(|| certify_g1_chain(&tangent_chain))
    });
    let line_arrangement_segments = vec![
        LinePathSegment::new(p(0, 0), p(1000, 0)),
        LinePathSegment::new(p(500, -250), p(500, 250)),
        LinePathSegment::new(p(250, 0), p(750, 0)),
        LinePathSegment::new(p(1000, 0), p(1200, 200)),
    ];
    c.bench_function("line_arrangement_exact_cleanup", |b| {
        b.iter(|| arrange_line_segments(&line_arrangement_segments, PredicatePolicy::default()))
    });
    let line_arc_lines = vec![
        LinePathSegment::new(p(-600, 0), p(600, 0)),
        LinePathSegment::new(p(0, -600), p(0, 600)),
        LinePathSegment::new(p(-500, 500), p(500, 500)),
    ];
    let line_arc_arcs = vec![
        ExplicitCircularArc::new(p(0, 0), r(500), p(500, 0), p(500, 0), ArcDirection::Ccw).unwrap(),
    ];
    c.bench_function("line_arc_arrangement_axis_cleanup", |b| {
        b.iter(|| {
            arrange_line_segments_with_explicit_arcs(
                &line_arc_lines,
                &line_arc_arcs,
                PredicatePolicy::default(),
            )
        })
    });
    let arc_arrangement_arcs = vec![
        ExplicitCircularArc::new(
            p(-300, 0),
            r(500),
            p(-300, -500),
            p(-300, 500),
            ArcDirection::Ccw,
        )
        .unwrap(),
        ExplicitCircularArc::new(
            p(300, 0),
            r(500),
            p(300, 500),
            p(300, -500),
            ArcDirection::Ccw,
        )
        .unwrap(),
        ExplicitCircularArc::new(p(0, 0), r(500), p(500, 0), p(-500, 0), ArcDirection::Ccw)
            .unwrap(),
    ];
    c.bench_function("explicit_arc_arrangement_split_cleanup", |b| {
        b.iter(|| arrange_explicit_arcs(&arc_arrangement_arcs, PredicatePolicy::default()))
    });
    let tangent_span_arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let tangent_span_curve = CubicBezier::new(p(-3, 4), p(-7, 1), p(-9, 1), p(-13, 4));
    let tangent_span_conic = RationalQuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0), r(2)).unwrap();
    c.bench_function("tangent_span_from_exact_primitives", |b| {
        b.iter(|| {
            (
                TangentSpan::from_line_segment(&tangent_segment),
                TangentSpan::from_explicit_arc(&tangent_span_arc),
                TangentSpan::from_cubic_bezier(&tangent_span_curve),
                TangentSpan::from_rational_quadratic_bezier(&tangent_span_conic),
            )
        })
    });
    let tangent_model = build_tangent_alignment_problem(p(3, 4), p(6, 8));
    c.bench_function("tangent_alignment_hypersolve_certification", |b| {
        b.iter(|| certify_tangent_alignment_candidate(&tangent_model))
    });
    let oriented_tangent_model = build_oriented_tangent_alignment_problem(p(3, 4), p(6, 8));
    c.bench_function("oriented_tangent_alignment_hypersolve_certification", |b| {
        b.iter(|| certify_tangent_alignment_candidate(&oriented_tangent_model))
    });
    let g1_join_model = build_g1_join_problem(p(10, 20), p(3, 4), p(10, 20), p(6, 8));
    c.bench_function("g1_join_hypersolve_certification", |b| {
        b.iter(|| certify_g1_join_candidate(&g1_join_model))
    });

    let bezier = QuadraticBezier::new(p(0, 0), p(500, 200), p(1000, 0));
    let half = BezierParameter::new(1, 2).unwrap();
    c.bench_function("quadratic_bezier_exact_eval", |b| {
        b.iter(|| bezier.eval(half))
    });
    c.bench_function("quadratic_bezier_exact_hodograph", |b| {
        b.iter(|| bezier.derivative(half))
    });
    c.bench_function("quadratic_bezier_exact_speed_squared", |b| {
        b.iter(|| bezier.speed_squared(half))
    });
    let line_quadratic_line = LinePathSegment::new(p(0, 0), p(1000, 0));
    c.bench_function("line_quadratic_bezier_exact_events", |b| {
        b.iter(|| {
            intersect_axis_aligned_line_quadratic_bezier(
                &line_quadratic_line,
                &bezier,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("line_quadratic_bezier_arrangement_cleanup", |b| {
        b.iter(|| {
            arrange_line_segments_with_quadratic_beziers(
                std::slice::from_ref(&line_quadratic_line),
                std::slice::from_ref(&bezier),
                PredicatePolicy::default(),
            )
        })
    });
    let line_quadratic_overlap_line = LinePathSegment::new(p(250, 0), p(750, 0));
    let line_quadratic_overlap_curve = QuadraticBezier::new(p(0, 0), p(500, 0), p(1000, 0));
    c.bench_function("line_quadratic_bezier_overlap_promotion", |b| {
        b.iter(|| {
            arrange_line_segments_with_quadratic_beziers(
                std::slice::from_ref(&line_quadratic_overlap_line),
                std::slice::from_ref(&line_quadratic_overlap_curve),
                PredicatePolicy::default(),
            )
        })
    });
    let bezier_events = vec![vec![
        BezierParameter::new(1, 4).unwrap(),
        BezierParameter::new(1, 2).unwrap(),
        BezierParameter::new(3, 4).unwrap(),
    ]];
    c.bench_function("quadratic_bezier_arrangement_split_cleanup", |b| {
        b.iter(|| {
            arrange_quadratic_beziers(
                std::slice::from_ref(&bezier),
                &bezier_events,
                PredicatePolicy::default(),
            )
        })
    });
    let conic = RationalQuadraticBezier::new(p(0, 0), p(500, 200), p(1000, 0), r(2)).unwrap();
    c.bench_function("rational_quadratic_bezier_exact_eval", |b| {
        b.iter(|| conic.eval(half))
    });
    c.bench_function("rational_quadratic_bezier_exact_hodograph", |b| {
        b.iter(|| conic.derivative(half))
    });
    c.bench_function("rational_quadratic_bezier_exact_speed_squared", |b| {
        b.iter(|| conic.speed_squared(half))
    });
    c.bench_function("rational_quadratic_bezier_arrangement_split_cleanup", |b| {
        b.iter(|| {
            arrange_rational_quadratic_beziers(
                std::slice::from_ref(&conic),
                &bezier_events,
                PredicatePolicy::default(),
            )
        })
    });
    let line_conic = RationalQuadraticBezier::new(p(0, 0), p(500, 1000), p(1000, 0), r(1)).unwrap();
    let line_conic_line = LinePathSegment::new(p(0, 375), p(1000, 375));
    c.bench_function("line_rational_quadratic_bezier_exact_events", |b| {
        b.iter(|| {
            intersect_axis_aligned_line_rational_quadratic_bezier(
                &line_conic_line,
                &line_conic,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("line_rational_quadratic_bezier_arrangement_cleanup", |b| {
        b.iter(|| {
            arrange_line_segments_with_rational_quadratic_beziers(
                std::slice::from_ref(&line_conic_line),
                std::slice::from_ref(&line_conic),
                PredicatePolicy::default(),
            )
        })
    });
    let line_conic_overlap =
        RationalQuadraticBezier::new(p(0, 0), p(500, 0), p(1000, 0), r(2)).unwrap();
    let line_conic_overlap_line = LinePathSegment::new(pq(3500, 11, 0, 1), pq(7500, 11, 0, 1));
    c.bench_function("line_rational_quadratic_bezier_overlap_promotion", |b| {
        b.iter(|| {
            arrange_line_segments_with_rational_quadratic_beziers(
                std::slice::from_ref(&line_conic_overlap_line),
                std::slice::from_ref(&line_conic_overlap),
                PredicatePolicy::default(),
            )
        })
    });
    let cubic = CubicBezier::new(p(0, 0), p(300, 300), p(700, 300), p(1000, 0));
    c.bench_function("cubic_bezier_exact_eval", |b| b.iter(|| cubic.eval(half)));
    c.bench_function("cubic_bezier_exact_hodograph", |b| {
        b.iter(|| cubic.derivative(half))
    });
    c.bench_function("cubic_bezier_exact_speed_squared", |b| {
        b.iter(|| cubic.speed_squared(half))
    });
    c.bench_function("cubic_bezier_arrangement_split_cleanup", |b| {
        b.iter(|| {
            arrange_cubic_beziers(
                std::slice::from_ref(&cubic),
                &bezier_events,
                PredicatePolicy::default(),
            )
        })
    });
    let quintic = HigherOrderBezier::quintic(
        p(0, 0),
        p(200, 100),
        p(400, 200),
        p(600, 200),
        p(800, 100),
        p(1000, 0),
    );
    c.bench_function("higher_order_bezier_exact_eval", |b| {
        b.iter(|| quintic.eval(half))
    });
    c.bench_function("higher_order_bezier_exact_hodograph", |b| {
        b.iter(|| quintic.derivative(half))
    });
    c.bench_function("higher_order_bezier_exact_speed_squared", |b| {
        b.iter(|| quintic.speed_squared(half))
    });
    c.bench_function("quadratic_bezier_offset_sample", |b| {
        b.iter(|| {
            offset_quadratic_bezier_sample(
                &bezier,
                half,
                r(25),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("cubic_bezier_offset_sample", |b| {
        b.iter(|| {
            offset_cubic_bezier_sample(
                &cubic,
                half,
                r(25),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("higher_order_bezier_offset_sample", |b| {
        b.iter(|| {
            offset_higher_order_bezier_sample(
                &quintic,
                half,
                r(25),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("explicit_circular_arc_exact_construction", |b| {
        b.iter(|| ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw))
    });
    let explicit_arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_sweep_class_fact", |b| {
        b.iter(|| explicit_arc.facts().sweep_class)
    });
    c.bench_function("explicit_circular_arc_point_membership", |b| {
        b.iter(|| explicit_arc.classify_point(&p(0, 5), PredicatePolicy::default()))
    });
    let explicit_arc_line = LinePathSegment::new(p(-10, 4), p(10, 4));
    c.bench_function("explicit_circular_arc_axis_line_intersection", |b| {
        b.iter(|| {
            explicit_arc
                .intersect_axis_aligned_segment(&explicit_arc_line, PredicatePolicy::default())
        })
    });
    let explicit_arc_subset =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, 5), p(-3, 4), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_same_circle_overlap", |b| {
        b.iter(|| {
            explicit_arc
                .classify_same_circle_overlap(&explicit_arc_subset, PredicatePolicy::default())
        })
    });
    let external_tangent_arc =
        ExplicitCircularArc::new(p(10, 0), r(5), p(15, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_circle_relation", |b| {
        b.iter(|| {
            explicit_arc.classify_circle_relation(&external_tangent_arc, PredicatePolicy::default())
        })
    });
    let tangent_membership_arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let tangent_membership_other =
        ExplicitCircularArc::new(p(10, 0), r(5), p(5, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_tangent_intersection", |b| {
        b.iter(|| {
            tangent_membership_arc.classify_tangent_intersection(
                &tangent_membership_other,
                PredicatePolicy::default(),
            )
        })
    });
    let secant_intersection_arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();
    let secant_intersection_other =
        ExplicitCircularArc::new(p(6, 0), r(5), p(11, 0), p(11, 0), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_secant_intersection", |b| {
        b.iter(|| {
            secant_intersection_arc
                .intersect_arc(&secant_intersection_other, PredicatePolicy::default())
        })
    });
    c.bench_function("explicit_circular_arc_arrangement_dispatch", |b| {
        b.iter(|| {
            secant_intersection_arc
                .arrange_with(&secant_intersection_other, PredicatePolicy::default())
        })
    });
    c.bench_function("explicit_circular_arc_exact_tangents", |b| {
        b.iter(|| (explicit_arc.start_tangent(), explicit_arc.end_tangent()))
    });
    let explicit_half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, -4), ArcDirection::Ccw).unwrap();
    c.bench_function("explicit_circular_arc_certified_length", |b| {
        b.iter(|| explicit_half.certified_sweep_length())
    });
    c.bench_function("explicit_circular_arc_analytic_minor_length", |b| {
        b.iter(|| explicit_arc.certified_sweep_length())
    });
    c.bench_function("explicit_circular_arc_offset_exact", |b| {
        b.iter(|| {
            offset_explicit_arc(
                &explicit_arc,
                r(5),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });

    let first = trace(1, p(0, 0), p(1000, 0));
    let second = trace(2, p(0, 6), p(1000, 6));
    c.bench_function("axis_aligned_trace_clearance_exact", |b| {
        b.iter(|| check_trace_clearance(&first, &second, &r(3), PredicatePolicy::default()))
    });

    let crossing = trace(2, p(500, -100), p(500, 100));
    c.bench_function("trace_no_short_exact_segment_predicate", |b| {
        b.iter(|| check_trace_clearance(&first, &crossing, &r(1), PredicatePolicy::default()))
    });

    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(500, 6), r(2)).unwrap();
    c.bench_function("trace_pad_clearance_exact", |b| {
        b.iter(|| check_trace_pad_clearance(&first, &pad, &r(3), PredicatePolicy::default()))
    });

    let via = PcbViaStack::new(NetId(2), TraceLayer(0), TraceLayer(2), p(500, 6), r(2)).unwrap();
    c.bench_function("trace_via_clearance_exact", |b| {
        b.iter(|| check_trace_via_clearance(&first, &via, &r(3), PredicatePolicy::default()))
    });
    c.bench_function("via_layer_transition_classification", |b| {
        b.iter(|| via.classify_layer_transition(4))
    });
    let overlapping_via =
        PcbViaStack::new(NetId(3), TraceLayer(1), TraceLayer(3), p(520, 6), r(2)).unwrap();
    c.bench_function("via_layer_span_relation", |b| {
        b.iter(|| via.classify_layer_span_with(&overlapping_via))
    });
    let drilled_via = PcbViaStack::with_drill(
        NetId(2),
        TraceLayer(0),
        TraceLayer(2),
        p(500, 6),
        r(10),
        r(2),
    )
    .unwrap();
    c.bench_function("trace_via_drill_clearance_exact", |b| {
        b.iter(|| {
            check_trace_via_drill_clearance(&first, &drilled_via, &r(3), PredicatePolicy::default())
        })
    });
    c.bench_function("via_drill_policy_classification", |b| {
        b.iter(|| drilled_via.classify_drill_policy(&r(3), PredicatePolicy::default()))
    });

    let rect = PcbRectPad::new(NetId(2), TraceLayer(0), p(500, 6), r(10), r(2)).unwrap();
    c.bench_function("trace_rect_pad_clearance_exact", |b| {
        b.iter(|| check_trace_rect_pad_clearance(&first, &rect, &r(3), PredicatePolicy::default()))
    });

    let cardinal_rect = PcbCardinalRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(500, 6),
        r(10),
        r(2),
        CardinalRotation::Deg90,
    )
    .unwrap();
    c.bench_function("trace_cardinal_rect_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_cardinal_rect_pad_clearance(
                &first,
                &cardinal_rect,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let rounded_rect =
        PcbRoundedRectPad::new(NetId(2), TraceLayer(0), p(500, 8), r(10), r(4), r(2)).unwrap();
    c.bench_function("trace_rounded_rect_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_rounded_rect_pad_clearance(
                &first,
                &rounded_rect,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let oriented_rect = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(500, 6),
        r(10),
        r(4),
        Point2::new(rq(3, 5), rq(4, 5)),
        PredicatePolicy::default(),
    )
    .unwrap();
    let oriented_trace = PcbTrace::new(
        NetId(1),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(pq(2442, 5, 11, 5), pq(2502, 5, 91, 5)),
            r(2),
        )
        .unwrap(),
    );
    c.bench_function("trace_oriented_rect_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_oriented_rect_pad_clearance(
                &oriented_trace,
                &oriented_rect,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let obround_pad = PcbObroundPad::new(
        NetId(2),
        TraceLayer(0),
        LinePathSegment::new(p(480, 0), p(520, 30)),
        r(8),
    )
    .unwrap();
    c.bench_function("trace_obround_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_obround_pad_clearance(
                &oriented_trace,
                &obround_pad,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let convex_pad = PcbConvexPad::new(
        NetId(2),
        TraceLayer(0),
        vec![p(500, 12), p(516, 0), p(500, -12), p(484, 0)],
    )
    .unwrap();
    c.bench_function("trace_convex_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_convex_pad_clearance(
                &oriented_trace,
                &convex_pad,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let orthogonal_pad = PcbOrthogonalPad::new(
        NetId(2),
        TraceLayer(0),
        vec![
            p(480, -20),
            p(520, -20),
            p(520, 0),
            p(500, 0),
            p(500, 20),
            p(480, 20),
        ],
    )
    .unwrap();
    c.bench_function("trace_orthogonal_pad_clearance_exact", |b| {
        b.iter(|| {
            check_trace_orthogonal_pad_clearance(
                &oriented_trace,
                &orthogonal_pad,
                &r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let board = PcbBoardOutline::new(p(-100, -100), p(1100, 100)).unwrap();
    c.bench_function("trace_board_edge_clearance_exact", |b| {
        b.iter(|| check_trace_board_clearance(&first, &board, &r(25), PredicatePolicy::default()))
    });
    let convex_board = PcbConvexBoardOutline::new(vec![
        p(-100, -100),
        p(1100, -100),
        p(1200, 100),
        p(-100, 100),
    ])
    .unwrap();
    c.bench_function("trace_convex_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_trace_convex_board_clearance(
                &first,
                &convex_board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    let orthogonal_board = PcbOrthogonalBoardOutline::new(vec![
        p(-100, -100),
        p(1100, -100),
        p(1100, 100),
        p(700, 100),
        p(700, 40),
        p(300, 40),
        p(300, 100),
        p(-100, 100),
    ])
    .unwrap();
    c.bench_function("trace_orthogonal_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_trace_orthogonal_board_clearance(
                &first,
                &orthogonal_board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    let circular_board = PcbCircularBoardOutline::new(p(500, 0), r(600)).unwrap();
    c.bench_function("trace_circular_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_trace_circular_board_clearance(
                &oriented_trace,
                &circular_board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("via_drill_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_via_drill_board_clearance(
                &drilled_via,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("circular_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_circular_pad_board_clearance(&pad, &board, &r(25), PredicatePolicy::default())
        })
    });
    c.bench_function("circular_pad_circular_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_circular_pad_circular_board_clearance(
                &pad,
                &circular_board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("rect_pad_board_edge_clearance_exact", |b| {
        b.iter(|| check_rect_pad_board_clearance(&rect, &board, &r(25), PredicatePolicy::default()))
    });
    c.bench_function("cardinal_rect_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_cardinal_rect_pad_board_clearance(
                &cardinal_rect,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("rounded_rect_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_rounded_rect_pad_board_clearance(
                &rounded_rect,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("oriented_rect_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_oriented_rect_pad_board_clearance(
                &oriented_rect,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("obround_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_obround_pad_board_clearance(
                &obround_pad,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("convex_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_convex_pad_board_clearance(
                &convex_pad,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("orthogonal_pad_board_edge_clearance_exact", |b| {
        b.iter(|| {
            check_orthogonal_pad_board_clearance(
                &orthogonal_pad,
                &board,
                &r(25),
                PredicatePolicy::default(),
            )
        })
    });

    let model = build_length_match_problem(r(1000), r(1250), r(250));
    c.bench_function("length_match_hypersolve_certification", |b| {
        b.iter(|| certify_length_extension(&model))
    });

    let tune_source = LinePathSegment::new(p(0, 0), p(1000, 0));
    let first_pair_route = vec![
        LinePathSegment::new(p(0, 0), p(600, 0)),
        LinePathSegment::new(p(600, 0), p(600, 120)),
    ];
    let second_pair_route = vec![LinePathSegment::new(p(0, 20), p(700, 20))];
    c.bench_function("differential_pair_skew_certification", |b| {
        b.iter(|| {
            certify_differential_pair_skew(
                &first_pair_route,
                &second_pair_route,
                r(20),
                PredicatePolicy::default(),
            )
        })
    });
    let feed_route = vec![
        LinePathSegment::new(p(0, 0), p(500, 0)),
        LinePathSegment::new(p(500, 0), p(500, 250)),
    ];
    c.bench_function("constant_feed_time_certification", |b| {
        b.iter(|| certify_constant_feed_time(&feed_route, r(250), r(3), PredicatePolicy::default()))
    });
    let mixed_radius = (r(10) / Real::pi()).unwrap();
    let mixed_feed_route = vec![
        FeedPathElement::Line(LinePathSegment::new(p(0, 0), p(740, 0))),
        FeedPathElement::ExplicitArc(
            ExplicitCircularArc::new(
                p(0, 0),
                mixed_radius.clone(),
                Point2::new(mixed_radius.clone(), r(0)),
                Point2::new(-mixed_radius, r(0)),
                ArcDirection::Ccw,
            )
            .unwrap(),
        ),
    ];
    c.bench_function("mixed_path_constant_feed_time_certification", |b| {
        b.iter(|| {
            certify_constant_feed_time_for_path(
                &mixed_feed_route,
                r(250),
                r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let acceleration_triangular_route = vec![LinePathSegment::new(p(0, 0), p(9, 0))];
    c.bench_function("acceleration_limited_feed_time_triangular", |b| {
        b.iter(|| {
            certify_acceleration_limited_feed_time(
                &acceleration_triangular_route,
                r(10),
                r(4),
                r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let mixed_accel_radius = (r(4) / Real::pi()).unwrap();
    let mixed_acceleration_route = vec![
        FeedPathElement::Line(LinePathSegment::new(p(0, 0), p(5, 0))),
        FeedPathElement::ExplicitArc(
            ExplicitCircularArc::new(
                p(0, 0),
                mixed_accel_radius.clone(),
                Point2::new(mixed_accel_radius.clone(), r(0)),
                Point2::new(-mixed_accel_radius, r(0)),
                ArcDirection::Ccw,
            )
            .unwrap(),
        ),
    ];
    c.bench_function("mixed_path_acceleration_limited_feed_time", |b| {
        b.iter(|| {
            certify_acceleration_limited_feed_time_for_path(
                &mixed_acceleration_route,
                r(10),
                r(4),
                r(3),
                PredicatePolicy::default(),
            )
        })
    });
    let acceleration_feed_route = vec![LinePathSegment::new(p(0, 0), p(1500, 0))];
    c.bench_function("acceleration_limited_feed_time_trapezoidal", |b| {
        b.iter(|| {
            certify_acceleration_limited_feed_time(
                &acceleration_feed_route,
                r(100),
                r(10),
                r(25),
                PredicatePolicy::default(),
            )
        })
    });
    let jerk_feed_route = vec![LinePathSegment::new(p(0, 0), p(4000, 0))];
    c.bench_function("symmetric_jerk_limited_feed_time", |b| {
        b.iter(|| {
            certify_symmetric_jerk_limited_feed_time(
                &jerk_feed_route,
                r(400),
                r(100),
                r(16),
                r(20),
                PredicatePolicy::default(),
            )
        })
    });
    let mixed_jerk_radius = (r(10) / Real::pi()).unwrap();
    let mixed_jerk_route = vec![
        FeedPathElement::Line(LinePathSegment::new(p(0, 0), p(3990, 0))),
        FeedPathElement::ExplicitArc(
            ExplicitCircularArc::new(
                p(0, 0),
                mixed_jerk_radius.clone(),
                Point2::new(mixed_jerk_radius.clone(), r(0)),
                Point2::new(-mixed_jerk_radius, r(0)),
                ArcDirection::Ccw,
            )
            .unwrap(),
        ),
    ];
    c.bench_function("mixed_path_symmetric_jerk_limited_feed_time", |b| {
        b.iter(|| {
            certify_symmetric_jerk_limited_feed_time_for_path(
                &mixed_jerk_route,
                r(400),
                r(100),
                r(16),
                r(20),
                PredicatePolicy::default(),
            )
        })
    });
    let corner_lookahead_spans = vec![
        TangentSpan::from_line_segment(&LinePathSegment::new(p(0, 0), p(100, 0))),
        TangentSpan::from_line_segment(&LinePathSegment::new(p(100, 0), p(100, 100))),
        TangentSpan::from_line_segment(&LinePathSegment::new(p(100, 100), p(150, 100))),
    ];
    c.bench_function("corner_lookahead_feed_limit_certification", |b| {
        b.iter(|| {
            certify_corner_lookahead_limits(
                &corner_lookahead_spans,
                r(10),
                r(20),
                r(25),
                r(4),
                PredicatePolicy::default(),
            )
        })
    });
    let lookahead_route = vec![
        FeedPathElement::Line(LinePathSegment::new(p(0, 0), p(100, 0))),
        FeedPathElement::Line(LinePathSegment::new(p(100, 0), p(100, 100))),
        FeedPathElement::Line(LinePathSegment::new(p(100, 100), p(150, 100))),
    ];
    let lookahead_schedule = LookaheadFeedSchedule {
        entry_feed: r(0),
        corner_feeds: vec![r(10), r(10)],
        corner_radii: vec![r(4), r(4)],
        exit_feed: r(0),
    };
    c.bench_function("lookahead_feed_schedule_certification", |b| {
        b.iter(|| {
            certify_lookahead_feed_schedule(
                &lookahead_route,
                &corner_lookahead_spans,
                &lookahead_schedule,
                r(20),
                r(25),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("single_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_single_detour_meander(
                &tune_source,
                r(250),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("multi_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_multi_detour_meander(
                &tune_source,
                r(250),
                4,
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("alternating_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_alternating_detour_meander(
                &tune_source,
                r(250),
                4,
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("nonuniform_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_nonuniform_detour_meander(
                &tune_source,
                vec![r(25), r(75), r(50)],
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    let meander_obstacles = vec![MeanderObstacle {
        min: p(-10, 20),
        max: p(300, 40),
    }];
    c.bench_function("obstacle_aware_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_obstacle_aware_detour_meander(
                &tune_source,
                r(250),
                4,
                OffsetSide::Left,
                meander_obstacles.clone(),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("meander_placement_slot_classification", |b| {
        b.iter(|| {
            classify_meander_placement_slots(
                &tune_source,
                r(25),
                4,
                OffsetSide::Left,
                meander_obstacles.clone(),
                PredicatePolicy::default(),
            )
        })
    });
    let meander_keepouts = vec![MeanderKeepout::Circular {
        center: p(100, 25),
        radius: r(20),
    }];
    c.bench_function("keepout_aware_detour_meander_exact_build", |b| {
        b.iter(|| {
            build_keepout_aware_detour_meander(
                &tune_source,
                r(250),
                4,
                OffsetSide::Left,
                meander_keepouts.clone(),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("meander_keepout_slot_classification", |b| {
        b.iter(|| {
            classify_meander_placement_slots_with_keepouts(
                &tune_source,
                r(25),
                4,
                OffsetSide::Left,
                meander_keepouts.clone(),
                PredicatePolicy::default(),
            )
        })
    });
    let arbitrary_meander_candidates = vec![
        MeanderPlacementCandidate {
            base: LinePathSegment::new(p(0, 0), p(150, 0)),
            amplitude: r(10),
        },
        MeanderPlacementCandidate {
            base: LinePathSegment::new(p(175, 0), p(450, 0)),
            amplitude: r(35),
        },
        MeanderPlacementCandidate {
            base: LinePathSegment::new(p(500, 0), p(1000, 0)),
            amplitude: r(20),
        },
    ];
    c.bench_function("meander_candidate_slot_classification", |b| {
        b.iter(|| {
            classify_meander_candidate_slots(
                arbitrary_meander_candidates.clone(),
                OffsetSide::Left,
                meander_obstacles.clone(),
                PredicatePolicy::default(),
            )
        })
    });

    let route_record = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: NetId(3),
        layer: TraceLayer(1),
        start_x: 0,
        start_y: 0,
        end_x: 1000,
        end_y: 0,
        width: 8,
        grid_denominator: 10,
    })
    .unwrap();
    c.bench_function("specctra_trace_record_exact_import", |b| {
        b.iter(|| import_specctra_trace_record(&route_record))
    });
    let via_record = specctra_grid_via_record(SpecctraGridViaRecord {
        net: NetId(3),
        start_layer: TraceLayer(0),
        end_layer: TraceLayer(3),
        x: 1000,
        y: 0,
        land_diameter: 24,
        drill_diameter: 10,
        drill_intent: ViaDrillIntent::Plated,
        grid_denominator: 10,
    })
    .unwrap();
    c.bench_function("specctra_via_record_exact_import", |b| {
        b.iter(|| import_specctra_via_record(&via_record))
    });

    let route_text = serialize_specctra_grid_trace_records(&[SpecctraGridTraceRecord {
        net: NetId(3),
        layer: TraceLayer(1),
        start_x: 0,
        start_y: 0,
        end_x: 1000,
        end_y: 0,
        width: 8,
        grid_denominator: 10,
    }]);
    c.bench_function("specctra_grid_route_text_parse", |b| {
        b.iter(|| parse_specctra_grid_trace_records(&route_text))
    });
    let via_record_text = SpecctraGridViaRecord {
        net: NetId(3),
        start_layer: TraceLayer(0),
        end_layer: TraceLayer(3),
        x: 1000,
        y: 0,
        land_diameter: 24,
        drill_diameter: 10,
        drill_intent: ViaDrillIntent::Plated,
        grid_denominator: 10,
    };
    let via_text = serialize_specctra_grid_via_records(&[via_record_text]);
    c.bench_function("specctra_grid_via_text_parse", |b| {
        b.iter(|| parse_specctra_grid_route_records(&via_text))
    });
    let mixed_text = serialize_specctra_grid_route_records(&hyperpath::SpecctraGridRouteRecords {
        net_aliases: vec![SpecctraNetAlias {
            net: NetId(3),
            name: "CLK_P".to_owned(),
        }],
        layer_aliases: vec![SpecctraLayerAlias {
            layer: TraceLayer(1),
            name: "F_Cu".to_owned(),
        }],
        traces: vec![SpecctraGridTraceRecord {
            net: NetId(3),
            layer: TraceLayer(1),
            start_x: 0,
            start_y: 0,
            end_x: 1000,
            end_y: 0,
            width: 8,
            grid_denominator: 10,
        }],
        vias: vec![via_record_text],
        keepouts: vec![SpecctraGridKeepoutRecord {
            layer: Some(TraceLayer(1)),
            shape: SpecctraGridKeepoutShape::Polygon {
                vertices: vec![
                    (400, 100),
                    (700, 100),
                    (700, 200),
                    (550, 200),
                    (550, 350),
                    (400, 350),
                ],
            },
            grid_denominator: 10,
        }],
    });
    c.bench_function("specctra_grid_mixed_route_text_parse", |b| {
        b.iter(|| parse_specctra_grid_route_records(&mixed_text))
    });
    let keepout_text = serialize_specctra_grid_keepout_records(&[SpecctraGridKeepoutRecord {
        layer: Some(TraceLayer(1)),
        shape: SpecctraGridKeepoutShape::Rect {
            min_x: -100,
            min_y: -50,
            max_x: 100,
            max_y: 50,
        },
        grid_denominator: 10,
    }]);
    c.bench_function("specctra_grid_keepout_text_parse", |b| {
        b.iter(|| parse_specctra_grid_route_records(&keepout_text))
    });
    let exact_keepout = SpecctraGridKeepoutRecord {
        layer: None,
        shape: SpecctraGridKeepoutShape::Circle {
            x: 100,
            y: 200,
            radius: 50,
        },
        grid_denominator: 10,
    };
    c.bench_function("specctra_grid_keepout_exact_lift", |b| {
        b.iter(|| specctra_grid_keepout_record(exact_keepout.clone()))
    });
    let exact_polygon_keepout = SpecctraGridKeepoutRecord {
        layer: Some(TraceLayer(1)),
        shape: SpecctraGridKeepoutShape::Polygon {
            vertices: vec![(0, 0), (60, 0), (60, 20), (20, 20), (20, 60), (0, 60)],
        },
        grid_denominator: 10,
    };
    c.bench_function("specctra_grid_polygon_keepout_exact_lift", |b| {
        b.iter(|| specctra_grid_keepout_record(exact_polygon_keepout.clone()))
    });
    let envelope_path_text = concat!(
        "(session \"bench board\"",
        " (metadata (ignored yes))",
        " (routes",
        "  (net 3 \"CLK P\")",
        "  (layer 1 \"F.Cu signal\")",
        "  (wire (net 3) (path 1 8 0 0 1000 0 1000 500 1500 500) (grid 10))",
        "  (via (net 3) (layers 0 3) (at 1000 0) (land 24) (drill 10) (intent plated) (grid 10))))",
    );
    c.bench_function("specctra_envelope_path_route_text_parse", |b| {
        b.iter(|| parse_specctra_grid_route_records(envelope_path_text))
    });

    let offset_source = LinePathSegment::new(p(0, 0), p(1000, 0));
    c.bench_function("axis_aligned_line_offset_exact", |b| {
        b.iter(|| {
            offset_axis_aligned_segment(
                &offset_source,
                r(25),
                OffsetSide::Left,
                PredicatePolicy::default(),
            )
        })
    });
    let pocket = RectangularPocket::new(p(0, 0), p(10_000, 6_000)).unwrap();
    c.bench_function("rectangular_pocket_offset_ring_schedule", |b| {
        b.iter(|| {
            build_rectangular_pocket_plan(
                pocket.clone(),
                r(125),
                r(250),
                128,
                PredicatePolicy::default(),
            )
        })
    });
    let pocket_plan = build_rectangular_pocket_plan(
        pocket.clone(),
        r(125),
        r(250),
        24,
        PredicatePolicy::default(),
    )
    .unwrap();
    c.bench_function("rectangular_pocket_link_graph", |b| {
        b.iter(|| {
            build_rectangular_pocket_link_graph(pocket_plan.clone(), PredicatePolicy::default())
        })
    });
    c.bench_function("rectangular_additive_bead_schedule", |b| {
        b.iter(|| {
            build_rectangular_bead_plan(
                pocket.clone(),
                BeadFillAxis::Horizontal,
                r(400),
                r(350),
                256,
                PredicatePolicy::default(),
            )
        })
    });
    let bead_plan = build_rectangular_bead_plan(
        pocket.clone(),
        BeadFillAxis::Horizontal,
        r(400),
        r(350),
        256,
        PredicatePolicy::default(),
    )
    .unwrap();
    c.bench_function("rectangular_serpentine_infill_graph", |b| {
        b.iter(|| {
            build_rectangular_serpentine_infill_graph(bead_plan.clone(), PredicatePolicy::default())
        })
    });
    let support_overhang = RectangularPocket::new(p(1_000, 1_000), p(3_000, 2_000)).unwrap();
    let support_base = RectangularPocket::new(p(0, 0), p(10_000, 6_000)).unwrap();
    c.bench_function("rectangular_support_footprint_plan", |b| {
        b.iter(|| {
            build_rectangular_support_plan(
                support_overhang.clone(),
                support_base.clone(),
                r(125),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("rectangular_region_intersection", |b| {
        b.iter(|| {
            intersect_rectangular_regions(
                support_base.clone(),
                support_overhang.clone(),
                PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("rectangular_region_subtraction", |b| {
        b.iter(|| {
            subtract_rectangular_region(
                support_base.clone(),
                support_overhang.clone(),
                PredicatePolicy::default(),
            )
        })
    });

    let arc = CircularArc::cardinal(
        p(0, 0),
        r(100),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
    )
    .unwrap();
    c.bench_function("cardinal_arc_exact_tangents", |b| {
        b.iter(|| (arc.start_tangent(), arc.end_tangent()))
    });
    c.bench_function("cardinal_arc_offset_exact", |b| {
        b.iter(|| offset_cardinal_arc(&arc, r(10), OffsetSide::Left, PredicatePolicy::default()))
    });
}

criterion_group!(benches, path_predicates);
criterion_main!(benches);
