use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ArcDirection, ArcOffsetError, Axis, BeadFillAxis, BeadPlanError, BezierOffsetError,
    BezierParameter, BezierParameterError, BoardContourError, BoardContourOrientation,
    CardinalPoint, CardinalRotation, CircularArc, CircularArcError, ClearanceStatus,
    ConstructionStamp, CubicBezier, DrillBoardClearanceReport, ExplicitArcArrangementClass,
    ExplicitArcIntersectionClass, ExplicitArcOverlapClass, ExplicitArcPointClassification,
    ExplicitArcSweepClass, ExplicitArcTangentClass, ExplicitCircleRelationClass,
    ExplicitCircularArc, HigherOrderBezier, HigherOrderBezierError, InfillGraphError,
    LineArcArrangementEventClass, LineArrangementError, LineArrangementEventClass,
    LineExplicitArcIntersectionClass, LineOffsetError, LinePathSegment,
    LineQuadraticBezierIntersectionClass, MeanderError, MeanderObstacle, MeanderPlacementCandidate,
    NetId, OffsetSide, PathProvenance, PathSourceFormat, PcbBoardOutline, PcbCardinalRectPad,
    PcbCircularBoardOutline, PcbCircularPad, PcbConvexBoardOutline, PcbConvexPad, PcbObroundPad,
    PcbOrientedRectPad, PcbOrthogonalBoardOutline, PcbRectPad, PcbRoundedRectPad, PcbTrace,
    PcbViaStack, PocketPlanError, PocketPlanStopReason, QuadraticBezier, RationalQuadraticBezier,
    RationalQuadraticBezierError, RectangularPocket, RectangularRegionRelation,
    RouteCertificationError, SegmentParameterOrder, SourceLengthUnit, SpecctraGridTraceRecord,
    SpecctraGridViaRecord, SpecctraImportError, SpecctraLayerAlias, SpecctraNetAlias,
    SpecctraParseError, SupportFootprintStatus, SupportPlanError, SweptLineSegment,
    TangentAlignment, TangentJoinClass, TangentJoinReport, TangentSpan, TraceLayer,
    ViaAnnularRingReport, ViaDrillIntent, ViaDrillPolicyClass, ViaLayerSpanRelation,
    ViaLayerTransitionClass, arrange_cubic_beziers, arrange_explicit_arcs, arrange_line_segments,
    arrange_line_segments_with_explicit_arcs, arrange_line_segments_with_quadratic_beziers,
    arrange_quadratic_beziers, arrange_rational_quadratic_beziers,
    build_alternating_detour_meander, build_g1_join_problem, build_length_match_problem,
    build_multi_detour_meander, build_nonuniform_detour_meander,
    build_obstacle_aware_detour_meander, build_oriented_tangent_alignment_problem,
    build_rectangular_bead_plan, build_rectangular_pocket_plan,
    build_rectangular_serpentine_infill_graph, build_rectangular_support_plan,
    build_single_detour_meander, build_tangent_alignment_problem, certify_constant_feed_time,
    certify_differential_pair_skew, certify_g1_chain, certify_g1_join_candidate,
    certify_length_extension, certify_tangent_alignment_candidate,
    check_cardinal_rect_pad_board_clearance, check_circular_pad_board_clearance,
    check_circular_pad_circular_board_clearance, check_convex_pad_board_clearance,
    check_obround_pad_board_clearance, check_oriented_rect_pad_board_clearance,
    check_rect_pad_board_clearance, check_rounded_rect_pad_board_clearance,
    check_trace_board_clearance, check_trace_cardinal_rect_pad_clearance,
    check_trace_circular_board_clearance, check_trace_clearance,
    check_trace_convex_board_clearance, check_trace_convex_pad_clearance,
    check_trace_obround_pad_clearance, check_trace_oriented_rect_pad_clearance,
    check_trace_orthogonal_board_clearance, check_trace_pad_clearance,
    check_trace_rect_pad_clearance, check_trace_rounded_rect_pad_clearance,
    check_trace_via_clearance, check_trace_via_drill_clearance, check_via_drill_board_clearance,
    classify_meander_candidate_slots, classify_meander_placement_slots, classify_tangent_alignment,
    classify_tangent_chain, classify_tangent_join, export_specctra_trace_record,
    import_specctra_text_route, import_specctra_trace_record, import_specctra_via_record,
    intersect_axis_aligned_line_quadratic_bezier, intersect_rectangular_regions,
    offset_axis_aligned_segment, offset_cardinal_arc, offset_cubic_bezier_sample,
    offset_explicit_arc, offset_higher_order_bezier_sample, offset_quadratic_bezier_sample,
    parse_specctra_grid_route_records, parse_specctra_grid_trace_records,
    serialize_specctra_grid_route_records, serialize_specctra_grid_trace_records,
    serialize_specctra_grid_via_records, specctra_grid_trace_record, specctra_grid_via_record,
    subtract_rectangular_region, tangent_cross, tangent_dot, tangent_norm_squared,
};
use hyperreal::{Rational, Real};
use proptest::prelude::*;

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

fn trace(net: u32, layer: u16, start: Point2, end: Point2, width: i64) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(layer),
        SweptLineSegment::new(LinePathSegment::new(start, end), r(width)).unwrap(),
    )
}

#[test]
fn line_segment_caches_axis_and_exact_length_facts() {
    let segment = LinePathSegment::new(p(0, 4), p(9, 4));

    assert_eq!(segment.facts().axis_aligned, Some(Axis::X));
    assert_eq!(segment.facts().known_degenerate, Some(false));
    assert!(segment.facts().endpoint_exact.all_exact_rational);
    assert_eq!(segment.axis_length(PredicatePolicy::default()), Some(r(9)));
    assert_eq!(segment.length_squared(), r(81));
    assert_eq!(segment.direction_vector(), p(9, 0));
    assert_eq!(segment.start_tangent(), p(9, 0));
    assert_eq!(segment.end_tangent(), p(9, 0));
}

#[test]
fn line_segment_retains_source_grid_provenance_and_prepared_bounds() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::Gerber, 1_000_000).unwrap();
    let segment = LinePathSegment::with_provenance(p(9, -2), p(3, 4), provenance);
    let bounds = segment.prepared_bounds();

    assert_eq!(segment.provenance(), provenance);
    assert_eq!(segment.bounds_min(), &p(3, -2));
    assert_eq!(segment.bounds_max(), &p(9, 4));
    assert_eq!(bounds.min(), segment.bounds_min());
    assert_eq!(bounds.max(), segment.bounds_max());
    assert!(bounds.contains_point(&p(6, 0)).value().unwrap());
    assert!(!bounds.contains_point(&p(10, 0)).value().unwrap());
}

#[test]
fn provenance_lifts_fixed_grid_tokens_exactly_and_tracks_units() {
    let millimeter = PathProvenance::fixed_grid_with_unit(
        PathSourceFormat::Gerber,
        1_000,
        SourceLengthUnit::Millimeter,
    )
    .unwrap();
    let inch = PathProvenance::fixed_grid_with_unit(
        PathSourceFormat::Gerber,
        1_000,
        SourceLengthUnit::Inch,
    )
    .unwrap();
    let unspecified = PathProvenance::fixed_grid(PathSourceFormat::Gerber, 1_000).unwrap();

    assert_eq!(
        millimeter.real_from_units(125),
        Some(Real::new(Rational::fraction(1, 8).unwrap()))
    );
    assert!(PathProvenance::native().real_from_units(1).is_none());
    assert!(millimeter.shares_grid_with(millimeter));
    assert!(!millimeter.shares_grid_with(inch));
    assert!(!millimeter.shares_grid_with(unspecified));
}

#[test]
fn provenance_construction_stamps_detect_stale_path_facts() {
    let stamp = ConstructionStamp::new(42, 7);
    let fresh = PathProvenance::fixed_grid(PathSourceFormat::KiCad, 1_000_000)
        .unwrap()
        .with_construction(stamp);
    let stale = fresh.with_construction(stamp.next_revision());
    let segment = LinePathSegment::with_provenance(p(0, 0), p(1, 0), fresh);
    let arc = CircularArc::cardinal_with_provenance(
        p(0, 0),
        r(5),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
        fresh,
    )
    .unwrap();

    assert!(fresh.is_fresh_for(stamp));
    assert!(!fresh.is_fresh_for(stamp.next_revision()));
    assert!(!fresh.shares_construction_with(stale));
    assert_eq!(segment.provenance(), fresh);
    assert_eq!(arc.provenance(), fresh);
}

#[test]
fn segment_parameter_order_respects_reversed_direction() {
    let segment = LinePathSegment::new(p(10, 0), p(0, 0));

    assert_eq!(
        segment.compare_points_along(&p(8, 0), &p(2, 0), PredicatePolicy::default()),
        SegmentParameterOrder::Before
    );
    assert_eq!(
        segment.compare_points_along(&p(2, 0), &p(8, 0), PredicatePolicy::default()),
        SegmentParameterOrder::After
    );
}

#[test]
fn line_arrangement_splits_crossings_touches_and_overlaps() {
    let horizontal = LinePathSegment::new(p(0, 0), p(10, 0));
    let vertical = LinePathSegment::new(p(5, -5), p(5, 5));
    let endpoint_touch = LinePathSegment::new(p(10, 0), p(12, 2));
    let overlap = LinePathSegment::new(p(3, 0), p(8, 0));

    let report = arrange_line_segments(
        &[horizontal, vertical, endpoint_touch, overlap],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.events.len(), 6);
    assert_eq!(
        report.events[0].class,
        LineArrangementEventClass::ProperCrossing
    );
    assert_eq!(report.events[0].point.as_ref().unwrap(), &p(5, 0));
    assert_eq!(
        report.events[1].class,
        LineArrangementEventClass::EndpointTouch
    );
    assert_eq!(report.events[1].point.as_ref().unwrap(), &p(10, 0));
    assert_eq!(
        report.events[2].class,
        LineArrangementEventClass::CollinearOverlap
    );
    assert_eq!(report.events[2].overlap.as_ref().unwrap().start(), &p(3, 0));
    assert_eq!(report.events[2].overlap.as_ref().unwrap().end(), &p(8, 0));

    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(0, 0), p(3, 0), p(5, 0), p(8, 0), p(10, 0)]
    );
    assert!(report.fragments.iter().any(|fragment| {
        fragment.source_segment == 0
            && fragment.segment.start() == &p(3, 0)
            && fragment.segment.end() == &p(5, 0)
    }));
    assert!(report.fragments.iter().any(|fragment| {
        fragment.source_segment == 1
            && fragment.segment.start() == &p(5, -5)
            && fragment.segment.end() == &p(5, 0)
    }));
}

#[test]
fn line_arrangement_rejects_degenerate_segment() {
    assert_eq!(
        arrange_line_segments(
            &[LinePathSegment::new(p(1, 1), p(1, 1))],
            PredicatePolicy::default(),
        )
        .unwrap_err(),
        LineArrangementError::DegenerateSegment { segment: 0 }
    );
}

#[test]
fn line_arrangement_handles_rational_proper_crossing() {
    let shallow = LinePathSegment::new(p(0, 0), p(6, 2));
    let steep = LinePathSegment::new(p(0, 2), p(6, 0));

    let report = arrange_line_segments(&[shallow, steep], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        LineArrangementEventClass::ProperCrossing
    );
    assert_eq!(report.events[0].point.as_ref().unwrap(), &p(3, 1));
    assert_eq!(report.breakpoints[0][1].point, p(3, 1));
    assert_eq!(report.breakpoints[0][1].parameter_numerator, r(20));
    assert_eq!(report.breakpoints[0][1].parameter_denominator, r(40));
    assert_eq!(report.fragments.len(), 4);
}

#[test]
fn line_arc_arrangement_splits_axis_lines_at_arc_events() {
    let line = LinePathSegment::new(p(-6, 0), p(6, 0));
    let tangent = LinePathSegment::new(p(-5, 5), p(5, 5));
    let arc = ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();

    let report = arrange_line_segments_with_explicit_arcs(
        &[line, tangent],
        &[arc],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.events.len(), 2);
    assert_eq!(report.events[0].class, LineArcArrangementEventClass::Secant);
    assert_eq!(report.events[0].points, vec![p(5, 0), p(-5, 0)]);
    assert_eq!(
        report.events[1].class,
        LineArcArrangementEventClass::Tangent
    );
    assert_eq!(report.events[1].points, vec![p(0, 5)]);
    assert_eq!(
        report.line_breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(-6, 0), p(-5, 0), p(5, 0), p(6, 0)]
    );
    assert_eq!(
        report.line_breakpoints[1]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(-5, 5), p(0, 5), p(5, 5)]
    );
    assert_eq!(report.line_fragments.len(), 5);
}

#[test]
fn line_arc_arrangement_reports_unknown_for_non_axis_line() {
    let diagonal = LinePathSegment::new(p(-5, -5), p(5, 5));
    let arc = ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();

    let report =
        arrange_line_segments_with_explicit_arcs(&[diagonal], &[arc], PredicatePolicy::default())
            .unwrap();

    assert_eq!(
        report.events[0].class,
        LineArcArrangementEventClass::Unknown
    );
    assert_eq!(report.line_breakpoints[0].len(), 2);
    assert_eq!(report.line_fragments.len(), 1);
}

#[test]
fn explicit_arc_arrangement_splits_different_circle_secants() {
    let left =
        ExplicitCircularArc::new(p(-3, 0), r(5), p(-3, -5), p(-3, 5), ArcDirection::Ccw).unwrap();
    let right =
        ExplicitCircularArc::new(p(3, 0), r(5), p(3, 5), p(3, -5), ArcDirection::Ccw).unwrap();

    let report = arrange_explicit_arcs(&[left, right], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::DifferentCircleTwoPoints
    );
    assert_eq!(report.events[0].points, vec![p(0, 4), p(0, -4)]);
    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(-3, -5), p(0, -4), p(0, 4), p(-3, 5)]
    );
    assert_eq!(report.breakpoints[1].len(), 4);
    assert_eq!(report.fragments.len(), 6);
    assert_eq!(report.fragments[0].arc.start(), &p(-3, -5));
    assert_eq!(report.fragments[0].arc.end(), &p(0, -4));
}

#[test]
fn explicit_arc_arrangement_promotes_same_circle_overlap_boundaries() {
    let top_half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(-5, 0), ArcDirection::Ccw).unwrap();
    let top_left =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, 5), p(-5, 0), ArcDirection::Ccw).unwrap();

    let report = arrange_explicit_arcs(&[top_half, top_left], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::SameCircleFirstCoversSecond
    );
    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(5, 0), p(0, 5), p(-5, 0)]
    );
    assert_eq!(
        report.breakpoints[1]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(0, 5), p(-5, 0)]
    );
    assert_eq!(report.fragments.len(), 3);
}

#[test]
fn explicit_arc_arrangement_uses_full_circle_start_as_branch_cut() {
    let full =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();
    let top_half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(-5, 0), ArcDirection::Ccw).unwrap();

    let report = arrange_explicit_arcs(&[full, top_half], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::SameCircleFirstCoversSecond
    );
    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(5, 0), p(-5, 0)]
    );
    assert_eq!(report.fragments.len(), 3);
    assert_eq!(report.fragments[1].arc.start(), &p(-5, 0));
    assert_eq!(report.fragments[1].arc.end(), &p(5, 0));
}

#[test]
fn explicit_arc_arrangement_orders_full_circle_secant_points_from_branch() {
    let full_left =
        ExplicitCircularArc::new(p(-3, 0), r(5), p(-3, -5), p(-3, -5), ArcDirection::Ccw).unwrap();
    let right =
        ExplicitCircularArc::new(p(3, 0), r(5), p(3, 5), p(3, -5), ArcDirection::Ccw).unwrap();

    let report = arrange_explicit_arcs(&[full_left, right], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::DifferentCircleTwoPoints
    );
    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.point.clone())
            .collect::<Vec<_>>(),
        vec![p(-3, -5), p(0, -4), p(0, 4)]
    );
    assert_eq!(report.fragments.len(), 6);
}

#[test]
fn quadratic_bezier_arrangement_splits_at_rational_events() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0));
    let half = BezierParameter::new(1, 2).unwrap();

    let report =
        arrange_quadratic_beziers(&[curve], &[vec![half]], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.breakpoints[0]
            .iter()
            .map(|breakpoint| breakpoint.parameter)
            .collect::<Vec<_>>(),
        vec![
            BezierParameter::new(0, 1).unwrap(),
            half,
            BezierParameter::new(1, 1).unwrap()
        ]
    );
    assert_eq!(report.fragments.len(), 2);
    assert_eq!(report.fragments[0].curve.start(), &p(0, 0));
    assert_eq!(report.fragments[0].curve.control(), &p(2, 4));
    assert_eq!(report.fragments[0].curve.end(), &p(4, 4));
    assert_eq!(report.fragments[1].curve.start(), &p(4, 4));
    assert_eq!(report.fragments[1].curve.end(), &p(8, 0));
}

#[test]
fn cubic_bezier_arrangement_sorts_and_dedups_parameters() {
    let curve = CubicBezier::new(p(0, 0), p(3, 9), p(6, 9), p(9, 0));
    let one_third = BezierParameter::new(1, 3).unwrap();
    let two_thirds = BezierParameter::new(2, 3).unwrap();

    let report = arrange_cubic_beziers(
        &[curve.clone()],
        &[vec![two_thirds, one_third, one_third]],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.breakpoints[0].len(), 4);
    assert_eq!(report.fragments.len(), 3);
    assert_eq!(report.fragments[0].curve.start(), curve.start());
    assert_eq!(report.fragments[0].curve.end(), &curve.eval(one_third));
    assert_eq!(report.fragments[1].curve.start(), &curve.eval(one_third));
    assert_eq!(report.fragments[1].curve.end(), &curve.eval(two_thirds));
    assert_eq!(report.fragments[2].curve.end(), curve.end());
}

#[test]
fn rational_quadratic_bezier_arrangement_emits_homogeneous_fragments() {
    let curve = RationalQuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0), r(2)).unwrap();
    let half = BezierParameter::new(1, 2).unwrap();

    let report =
        arrange_rational_quadratic_beziers(&[curve], &[vec![half]], PredicatePolicy::default())
            .unwrap();

    assert_eq!(report.breakpoints[0].len(), 3);
    assert_eq!(report.fragments.len(), 2);
    assert_eq!(report.fragments[0].start_control.x, r(0));
    assert_eq!(report.fragments[0].start_control.y, r(0));
    assert_eq!(report.fragments[0].start_control.w, r(1));
    assert_eq!(report.fragments[0].end_control.x, r(3));
    assert_eq!(report.fragments[0].end_control.y, r(4));
    assert_eq!(report.fragments[0].end_control.w, rq(3, 2));
    assert_eq!(
        report.fragments[1].start_control,
        report.fragments[0].end_control
    );
}

#[test]
fn line_quadratic_bezier_intersection_finds_endpoint_events() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(0, 0), p(8, 0));

    let report =
        intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

    assert_eq!(
        report.class,
        LineQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(report.intersections.len(), 2);
    assert_eq!(report.intersections[0].parameter, r(0));
    assert_eq!(report.intersections[0].point, p(0, 0));
    assert_eq!(report.intersections[1].parameter, r(1));
    assert_eq!(report.intersections[1].point, p(8, 0));
}

#[test]
fn line_quadratic_bezier_intersection_classifies_tangent_event() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(0, 2), p(8, 2));

    let report =
        intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

    assert_eq!(report.class, LineQuadraticBezierIntersectionClass::Tangent);
    assert_eq!(report.intersections.len(), 1);
    assert_eq!(report.intersections[0].parameter, rq(1, 2));
    assert_eq!(report.intersections[0].point, p(4, 2));
}

#[test]
fn line_quadratic_bezier_intersection_clips_to_segment_bounds() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(-1, 0), p(1, 0));

    let report =
        intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

    assert_eq!(report.class, LineQuadraticBezierIntersectionClass::OnePoint);
    assert_eq!(report.intersections.len(), 1);
    assert_eq!(report.intersections[0].parameter, r(0));
    assert_eq!(report.intersections[0].point, p(0, 0));
}

#[test]
fn line_quadratic_bezier_intersection_returns_unknown_for_overlap_support() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 0), p(8, 0));
    let line = LinePathSegment::new(p(2, 0), p(6, 0));

    let report =
        intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

    assert_eq!(report.class, LineQuadraticBezierIntersectionClass::Unknown);
    assert!(report.intersections.is_empty());
}

#[test]
fn line_quadratic_bezier_intersection_returns_unknown_for_non_axis_line() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(0, 0), p(8, 1));

    let report =
        intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

    assert_eq!(report.class, LineQuadraticBezierIntersectionClass::Unknown);
    assert!(report.intersections.is_empty());
}

#[test]
fn line_quadratic_bezier_arrangement_splits_certified_secant_roots() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0));
    let line = LinePathSegment::new(p(0, 3), p(8, 3));

    let report =
        arrange_line_segments_with_quadratic_beziers(&[line], &[curve], PredicatePolicy::default())
            .unwrap();

    assert_eq!(
        report.events[0].class,
        LineQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(report.line_breakpoints[0].len(), 4);
    assert_eq!(report.bezier_breakpoints[0].len(), 4);
    assert_eq!(report.line_fragments.len(), 3);
    assert_eq!(report.bezier_fragments.len(), 3);
    assert_eq!(report.line_breakpoints[0][0].point, p(0, 3));
    assert_eq!(report.line_breakpoints[0][1].point, p(2, 3));
    assert_eq!(report.line_breakpoints[0][2].point, p(6, 3));
    assert_eq!(report.line_breakpoints[0][3].point, p(8, 3));
    assert_eq!(report.bezier_breakpoints[0][0].parameter, r(0));
    assert_eq!(report.bezier_breakpoints[0][1].parameter, rq(1, 4));
    assert_eq!(report.bezier_breakpoints[0][2].parameter, rq(3, 4));
    assert_eq!(report.bezier_breakpoints[0][3].parameter, r(1));
    assert_eq!(report.bezier_fragments[0].curve.end(), &p(2, 3));
    assert_eq!(report.bezier_fragments[1].curve.start(), &p(2, 3));
    assert_eq!(report.bezier_fragments[1].curve.end(), &p(6, 3));
    assert_eq!(report.bezier_fragments[2].curve.start(), &p(6, 3));
}

#[test]
fn line_quadratic_bezier_arrangement_records_unrepresented_secant_unknown() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(0, 1), p(8, 1));

    let report =
        arrange_line_segments_with_quadratic_beziers(&[line], &[curve], PredicatePolicy::default())
            .unwrap();

    assert_eq!(
        report.events[0].class,
        LineQuadraticBezierIntersectionClass::Unknown
    );
    assert_eq!(report.line_breakpoints[0].len(), 2);
    assert_eq!(report.bezier_breakpoints[0].len(), 2);
}

#[test]
fn line_quadratic_bezier_arrangement_splits_tangent_curve_once() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0));
    let line = LinePathSegment::new(p(0, 2), p(8, 2));

    let report =
        arrange_line_segments_with_quadratic_beziers(&[line], &[curve], PredicatePolicy::default())
            .unwrap();

    assert_eq!(
        report.events[0].class,
        LineQuadraticBezierIntersectionClass::Tangent
    );
    assert_eq!(report.line_breakpoints[0].len(), 3);
    assert_eq!(report.bezier_breakpoints[0].len(), 3);
    assert_eq!(report.bezier_breakpoints[0][1].parameter, rq(1, 2));
    assert_eq!(report.bezier_fragments.len(), 2);
    assert_eq!(report.bezier_fragments[0].curve.end(), &p(4, 2));
    assert_eq!(report.bezier_fragments[1].curve.start(), &p(4, 2));
}

#[test]
fn line_quadratic_bezier_arrangement_keeps_overlap_unknown_unsplit() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 0), p(8, 0));
    let line = LinePathSegment::new(p(2, 0), p(6, 0));

    let report =
        arrange_line_segments_with_quadratic_beziers(&[line], &[curve], PredicatePolicy::default())
            .unwrap();

    assert_eq!(
        report.events[0].class,
        LineQuadraticBezierIntersectionClass::Unknown
    );
    assert_eq!(report.line_breakpoints[0].len(), 2);
    assert_eq!(report.bezier_breakpoints[0].len(), 2);
    assert_eq!(report.line_fragments.len(), 1);
    assert_eq!(report.bezier_fragments.len(), 1);
}

#[test]
fn line_quadratic_bezier_arrangement_rejects_degenerate_line_order() {
    let curve = QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0));
    let line = LinePathSegment::new(p(2, 3), p(2, 3));

    let err =
        arrange_line_segments_with_quadratic_beziers(&[line], &[curve], PredicatePolicy::default())
            .unwrap_err();

    assert_eq!(
        err,
        hyperpath::LineQuadraticBezierArrangementError::DegenerateLine { line: 0 }
    );
}

#[test]
fn tangent_alignment_classifies_exact_vector_relations() {
    let east = p(4, 0);
    let east_scaled = p(8, 0);
    let west = p(-2, 0);
    let north = p(0, 3);
    let zero = p(0, 0);

    assert_eq!(tangent_cross(&east, &north), r(12));
    assert_eq!(tangent_dot(&east, &west), r(-8));
    assert_eq!(tangent_norm_squared(&north), r(9));
    assert_eq!(
        classify_tangent_alignment(&east, &east_scaled, PredicatePolicy::default()),
        TangentAlignment::SameDirection
    );
    assert_eq!(
        classify_tangent_alignment(&east, &west, PredicatePolicy::default()),
        TangentAlignment::OppositeDirection
    );
    assert_eq!(
        classify_tangent_alignment(&east, &north, PredicatePolicy::default()),
        TangentAlignment::NotParallel
    );
    assert_eq!(
        classify_tangent_alignment(&east, &zero, PredicatePolicy::default()),
        TangentAlignment::Degenerate
    );
}

#[test]
fn tangent_alignment_accepts_arc_and_bezier_hodographs() {
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let bezier = CubicBezier::new(p(3, 4), p(-1, 7), p(1, 7), p(-3, 4));

    assert_eq!(
        classify_tangent_alignment(
            &arc.start_tangent(),
            &bezier.derivative(BezierParameter::new(0, 1).unwrap()),
            PredicatePolicy::default(),
        ),
        TangentAlignment::SameDirection
    );
    assert_eq!(
        classify_tangent_alignment(
            &arc.end_tangent(),
            &bezier.derivative(BezierParameter::new(1, 1).unwrap()),
            PredicatePolicy::default(),
        ),
        TangentAlignment::SameDirection
    );
}

#[test]
fn tangent_join_classifies_endpoint_and_g1_continuity() {
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let bezier = CubicBezier::new(p(-3, 4), p(-7, 1), p(-9, 1), p(-13, 4));

    assert_eq!(
        classify_tangent_join(
            arc.end(),
            &arc.end_tangent(),
            bezier.start(),
            &bezier.derivative(BezierParameter::new(0, 1).unwrap()),
            PredicatePolicy::default(),
        ),
        TangentJoinReport {
            class: TangentJoinClass::G1Continuous,
            endpoints_equal: Some(true),
            alignment: Some(TangentAlignment::SameDirection),
        }
    );
}

#[test]
fn tangent_join_accepts_line_to_arc_continuity() {
    let line = LinePathSegment::new(p(7, 1), p(3, 4));
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();

    assert_eq!(
        classify_tangent_join(
            line.end(),
            &line.end_tangent(),
            arc.start(),
            &arc.start_tangent(),
            PredicatePolicy::default(),
        ),
        TangentJoinReport {
            class: TangentJoinClass::G1Continuous,
            endpoints_equal: Some(true),
            alignment: Some(TangentAlignment::SameDirection),
        }
    );
}

#[test]
fn tangent_join_reports_mismatch_corner_and_degenerate_cases() {
    assert_eq!(
        classify_tangent_join(
            &p(0, 0),
            &p(1, 0),
            &p(1, 0),
            &p(1, 0),
            PredicatePolicy::default(),
        )
        .class,
        TangentJoinClass::EndpointMismatch
    );
    assert_eq!(
        classify_tangent_join(
            &p(0, 0),
            &p(1, 0),
            &p(0, 0),
            &p(0, 1),
            PredicatePolicy::default(),
        )
        .class,
        TangentJoinClass::Corner
    );
    assert_eq!(
        classify_tangent_join(
            &p(0, 0),
            &p(0, 0),
            &p(0, 0),
            &p(1, 0),
            PredicatePolicy::default(),
        )
        .class,
        TangentJoinClass::DegenerateTangent
    );
}

#[test]
fn tangent_alignment_problem_certifies_exact_cross_residual() {
    let satisfied = build_tangent_alignment_problem(p(3, 4), p(6, 8));
    let violated = build_tangent_alignment_problem(p(3, 4), p(0, 5));

    assert!(
        certify_tangent_alignment_candidate(&satisfied).all_satisfied(),
        "parallel tangent residual should certify"
    );
    assert!(
        certify_tangent_alignment_candidate(&violated).has_certified_violation(),
        "nonparallel tangent residual should violate"
    );
}

#[test]
fn oriented_tangent_alignment_problem_rejects_opposite_direction() {
    let same = build_oriented_tangent_alignment_problem(p(3, 4), p(6, 8));
    let opposite = build_oriented_tangent_alignment_problem(p(3, 4), p(-6, -8));

    assert!(certify_tangent_alignment_candidate(&same).all_satisfied());
    assert!(certify_tangent_alignment_candidate(&opposite).has_certified_violation());
}

#[test]
fn g1_join_problem_certifies_endpoint_and_oriented_tangent() {
    let satisfied = build_g1_join_problem(p(3, 4), p(-4, 3), p(3, 4), p(-8, 6));
    let endpoint_mismatch = build_g1_join_problem(p(3, 4), p(-4, 3), p(3, 5), p(-8, 6));
    let tangent_reversed = build_g1_join_problem(p(3, 4), p(-4, 3), p(3, 4), p(8, -6));

    assert!(certify_g1_join_candidate(&satisfied).all_satisfied());
    assert!(certify_g1_join_candidate(&endpoint_mismatch).has_certified_violation());
    assert!(certify_g1_join_candidate(&tangent_reversed).has_certified_violation());
}

#[test]
fn tangent_chain_reports_all_g1_and_first_bad_join() {
    let line = LinePathSegment::new(p(7, 1), p(3, 4));
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let cubic = CubicBezier::new(p(-3, 4), p(-7, 1), p(-9, 1), p(-13, 4));
    let spans = vec![
        TangentSpan::from_line_segment(&line),
        TangentSpan::from_explicit_arc(&arc),
        TangentSpan::from_cubic_bezier(&cubic),
    ];

    let report = classify_tangent_chain(&spans, PredicatePolicy::default());
    assert_eq!(report.joins.len(), 2);
    assert!(report.all_g1_continuous());
    assert_eq!(report.first_non_g1_join(), None);

    let mut broken = spans;
    broken[2].start_tangent = p(0, 1);
    let broken_report = classify_tangent_chain(&broken, PredicatePolicy::default());
    assert!(!broken_report.all_g1_continuous());
    assert_eq!(broken_report.first_non_g1_join(), Some(1));
    assert_eq!(broken_report.joins[1].class, TangentJoinClass::Corner);
}

#[test]
fn g1_chain_certification_replays_every_adjacent_join() {
    let line = LinePathSegment::new(p(7, 1), p(3, 4));
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let cubic = CubicBezier::new(p(-3, 4), p(-7, 1), p(-9, 1), p(-13, 4));
    let spans = vec![
        TangentSpan::from_line_segment(&line),
        TangentSpan::from_explicit_arc(&arc),
        TangentSpan::from_cubic_bezier(&cubic),
    ];

    let report = certify_g1_chain(&spans);
    assert_eq!(report.joins.len(), 2);
    assert!(report.all_certified());
    assert_eq!(report.first_uncertified_join(), None);

    let mut broken = spans;
    broken[2].start = p(-3, 5);
    let broken_report = certify_g1_chain(&broken);
    assert!(!broken_report.all_certified());
    assert_eq!(broken_report.first_uncertified_join(), Some(1));
    assert!(broken_report.joins[1].has_certified_violation());
}

#[test]
fn tangent_span_constructors_retain_primitive_endpoint_hodographs() {
    let line = LinePathSegment::new(p(0, 0), p(3, 4));
    let line_span = TangentSpan::from_line_segment(&line);
    assert_eq!(line_span.start, p(0, 0));
    assert_eq!(line_span.start_tangent, p(3, 4));
    assert_eq!(line_span.end, p(3, 4));
    assert_eq!(line_span.end_tangent, p(3, 4));

    let cardinal = CircularArc::cardinal(
        p(0, 0),
        r(5),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
    )
    .unwrap();
    let cardinal_span = TangentSpan::from_cardinal_arc(&cardinal);
    assert_eq!(cardinal_span.start, p(5, 0));
    assert_eq!(cardinal_span.start_tangent, p(0, 5));
    assert_eq!(cardinal_span.end, p(0, 5));
    assert_eq!(cardinal_span.end_tangent, p(-5, 0));

    let quadratic = QuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0));
    let quadratic_span = TangentSpan::from_quadratic_bezier(&quadratic);
    assert_eq!(quadratic_span.start, p(0, 0));
    assert_eq!(quadratic_span.start_tangent, p(4, 8));
    assert_eq!(quadratic_span.end, p(4, 0));
    assert_eq!(quadratic_span.end_tangent, p(4, -8));

    let cubic = CubicBezier::new(p(0, 0), p(2, 4), p(6, 4), p(8, 0));
    let cubic_span = TangentSpan::from_cubic_bezier(&cubic);
    assert_eq!(cubic_span.start, p(0, 0));
    assert_eq!(cubic_span.start_tangent, p(6, 12));
    assert_eq!(cubic_span.end, p(8, 0));
    assert_eq!(cubic_span.end_tangent, p(6, -12));

    let conic = RationalQuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0), r(2)).unwrap();
    let conic_span = TangentSpan::from_rational_quadratic_bezier(&conic).unwrap();
    assert_eq!(conic_span.start, p(0, 0));
    assert_eq!(conic_span.start_tangent, p(8, 16));
    assert_eq!(conic_span.end, p(4, 0));
    assert_eq!(conic_span.end_tangent, p(8, -16));
}

#[test]
fn quadratic_bezier_evaluates_exact_rational_parameters() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let curve = QuadraticBezier::with_provenance(p(0, 0), p(2, 4), p(4, 0), provenance);

    assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()), p(0, 0));
    assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()), p(4, 0));
    assert_eq!(curve.eval(BezierParameter::new(1, 2).unwrap()), p(2, 2));
    assert_eq!(curve.facts().chord_length_squared, r(16));
    assert!(curve.facts().control_exact.all_exact_rational);
    assert!(!curve.facts().known_degenerate);
    assert_eq!(curve.provenance(), provenance);
}

#[test]
fn quadratic_bezier_evaluates_exact_hodograph() {
    let curve = QuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0));

    assert_eq!(
        curve.derivative(BezierParameter::new(0, 1).unwrap()),
        p(4, 8)
    );
    assert_eq!(
        curve.derivative(BezierParameter::new(1, 2).unwrap()),
        p(4, 0)
    );
    assert_eq!(
        curve.derivative(BezierParameter::new(1, 1).unwrap()),
        p(4, -8)
    );
    assert_eq!(
        curve.speed_squared(BezierParameter::new(1, 2).unwrap()),
        r(16)
    );
}

#[test]
fn quadratic_bezier_rejects_invalid_parameters_and_detects_degenerate_curve() {
    let curve = QuadraticBezier::new(p(1, 1), p(1, 1), p(1, 1));

    assert!(curve.facts().known_degenerate);
    assert_eq!(
        BezierParameter::new(1, 0).unwrap_err(),
        BezierParameterError::ZeroDenominator
    );
    assert_eq!(
        BezierParameter::new(-1, 2).unwrap_err(),
        BezierParameterError::OutOfRange
    );
    assert_eq!(
        BezierParameter::new(3, 2).unwrap_err(),
        BezierParameterError::OutOfRange
    );
}

#[test]
fn rational_quadratic_bezier_evaluates_exact_conic_parameters() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let curve =
        RationalQuadraticBezier::with_provenance(p(0, 0), p(2, 4), p(4, 0), r(2), provenance)
            .unwrap();

    assert_eq!(
        curve.eval(BezierParameter::new(0, 1).unwrap()).unwrap(),
        p(0, 0)
    );
    assert_eq!(
        curve.eval(BezierParameter::new(1, 1).unwrap()).unwrap(),
        p(4, 0)
    );
    assert_eq!(
        curve.eval(BezierParameter::new(1, 2).unwrap()).unwrap(),
        Point2::new(r(2), Real::new(Rational::fraction(8, 3).unwrap()))
    );
    assert_eq!(curve.facts().chord_length_squared, r(16));
    assert!(curve.facts().exact.all_exact_rational);
    assert_eq!(curve.provenance(), provenance);
}

#[test]
fn rational_quadratic_bezier_evaluates_exact_hodograph() {
    let curve = RationalQuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0), r(2)).unwrap();

    assert_eq!(
        curve
            .derivative(BezierParameter::new(0, 1).unwrap())
            .unwrap(),
        p(8, 16)
    );
    assert_eq!(
        curve
            .derivative(BezierParameter::new(1, 2).unwrap())
            .unwrap(),
        Point2::new(Real::new(Rational::fraction(8, 3).unwrap()), Real::zero())
    );
    assert_eq!(
        curve
            .derivative(BezierParameter::new(1, 1).unwrap())
            .unwrap(),
        p(8, -16)
    );
    assert_eq!(
        curve
            .speed_squared(BezierParameter::new(1, 2).unwrap())
            .unwrap(),
        Real::new(Rational::fraction(64, 9).unwrap())
    );
}

#[test]
fn rational_quadratic_bezier_rejects_negative_weight() {
    let error = RationalQuadraticBezier::new(p(0, 0), p(1, 1), p(2, 0), r(-1))
        .expect_err("negative rational Bezier weight must be rejected");

    assert_eq!(error, RationalQuadraticBezierError::NegativeWeight);
}

#[test]
fn cubic_bezier_evaluates_exact_rational_parameters() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let curve = CubicBezier::with_provenance(p(0, 0), p(3, 6), p(6, 6), p(9, 0), provenance);

    assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()), p(0, 0));
    assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()), p(9, 0));
    assert_eq!(
        curve.eval(BezierParameter::new(1, 2).unwrap()),
        Point2::new(
            Real::new(Rational::fraction(9, 2).unwrap()),
            Real::new(Rational::fraction(9, 2).unwrap())
        )
    );
    assert_eq!(curve.facts().chord_length_squared, r(81));
    assert!(curve.facts().control_exact.all_exact_rational);
    assert!(!curve.facts().known_degenerate);
    assert_eq!(curve.provenance(), provenance);
}

#[test]
fn cubic_bezier_evaluates_exact_hodograph() {
    let curve = CubicBezier::new(p(0, 0), p(3, 6), p(6, 6), p(9, 0));

    assert_eq!(
        curve.derivative(BezierParameter::new(0, 1).unwrap()),
        p(9, 18)
    );
    assert_eq!(
        curve.derivative(BezierParameter::new(1, 2).unwrap()),
        p(9, 0)
    );
    assert_eq!(
        curve.derivative(BezierParameter::new(1, 1).unwrap()),
        p(9, -18)
    );
    assert_eq!(
        curve.speed_squared(BezierParameter::new(1, 2).unwrap()),
        r(81)
    );
}

#[test]
fn cubic_bezier_detects_degenerate_curve() {
    let curve = CubicBezier::new(p(1, 1), p(1, 1), p(1, 1), p(1, 1));

    assert!(curve.facts().known_degenerate);
}

#[test]
fn higher_order_bezier_evaluates_quartic_and_quintic_exactly() {
    let quartic = HigherOrderBezier::quartic(p(0, 0), p(4, 0), p(8, 0), p(12, 0), p(16, 0));
    assert_eq!(quartic.facts().degree, 4);
    assert_eq!(quartic.eval(BezierParameter::new(1, 2).unwrap()), p(8, 0));
    assert_eq!(
        quartic.derivative(BezierParameter::new(1, 2).unwrap()),
        p(16, 0)
    );
    assert_eq!(
        quartic.speed_squared(BezierParameter::new(1, 2).unwrap()),
        r(256)
    );

    let quintic = HigherOrderBezier::quintic(p(0, 0), p(2, 0), p(4, 0), p(6, 0), p(8, 0), p(10, 0));
    assert_eq!(quintic.facts().degree, 5);
    assert_eq!(quintic.eval(BezierParameter::new(1, 2).unwrap()), p(5, 0));
    assert_eq!(
        quintic.derivative(BezierParameter::new(1, 2).unwrap()),
        p(10, 0)
    );
}

#[test]
fn higher_order_bezier_rejects_unsupported_degrees_and_detects_degenerate_curve() {
    assert_eq!(
        HigherOrderBezier::with_provenance(
            vec![p(0, 0), p(1, 1), p(2, 0)],
            PathProvenance::native()
        )
        .unwrap_err(),
        HigherOrderBezierError::UnsupportedDegree
    );
    let degenerate = HigherOrderBezier::quartic(p(1, 1), p(1, 1), p(1, 1), p(1, 1), p(1, 1));
    assert!(degenerate.facts().known_degenerate);
    assert!(degenerate.facts().control_exact.all_exact_rational);
}

#[test]
fn cardinal_arc_preserves_exact_radius_endpoints_and_length() {
    let arc = CircularArc::cardinal(
        p(2, 3),
        r(5),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
    )
    .unwrap();

    assert_eq!(arc.start(), p(7, 3));
    assert_eq!(arc.end(), p(2, 8));
    assert_eq!(arc.facts().radius_squared, r(25));
    assert_eq!(arc.facts().quarter_turns, 1);
    assert_eq!(arc.chord_length_squared(), r(50));
    assert_eq!(
        arc.exact_length(),
        r(5) * Real::pi() * Real::new(Rational::fraction(1, 2).unwrap())
    );
    assert_eq!(arc.start_tangent(), p(0, 5));
    assert_eq!(arc.end_tangent(), p(-5, 0));
}

#[test]
fn cardinal_arc_tangents_respect_clockwise_direction() {
    let arc = CircularArc::cardinal(
        p(2, 3),
        r(5),
        CardinalPoint::East,
        CardinalPoint::South,
        ArcDirection::Cw,
    )
    .unwrap();

    assert_eq!(arc.start_tangent(), p(0, -5));
    assert_eq!(arc.end_tangent(), p(-5, 0));
}

#[test]
fn cardinal_arc_rejects_invalid_radius() {
    assert_eq!(
        CircularArc::cardinal(
            p(0, 0),
            r(0),
            CardinalPoint::East,
            CardinalPoint::North,
            ArcDirection::Ccw,
        )
        .unwrap_err(),
        CircularArcError::DegenerateRadius
    );
    assert_eq!(
        CircularArc::cardinal(
            p(0, 0),
            r(-1),
            CardinalPoint::East,
            CardinalPoint::North,
            ArcDirection::Ccw,
        )
        .unwrap_err(),
        CircularArcError::NegativeRadius
    );
}

#[test]
fn explicit_circular_arc_preserves_non_cardinal_endpoints_exactly() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let arc = ExplicitCircularArc::with_provenance(
        p(0, 0),
        r(5),
        p(3, 4),
        p(-3, 4),
        ArcDirection::Ccw,
        provenance,
    )
    .unwrap();

    assert_eq!(arc.center(), &p(0, 0));
    assert_eq!(arc.radius(), &r(5));
    assert_eq!(arc.start(), &p(3, 4));
    assert_eq!(arc.end(), &p(-3, 4));
    assert_eq!(arc.direction(), ArcDirection::Ccw);
    assert_eq!(arc.facts().radius_squared, r(25));
    assert_eq!(arc.chord_length_squared(), r(36));
    assert_eq!(arc.facts().radial_dot, r(7));
    assert_eq!(arc.facts().radial_cross, r(24));
    assert_eq!(
        arc.facts().sweep_class,
        ExplicitArcSweepClass::LessThanHalfTurn
    );
    assert!(!arc.facts().known_full_circle);
    assert!(arc.facts().exact.all_exact_rational);
    assert_eq!(arc.provenance(), provenance);
    assert_eq!(arc.start_tangent(), p(-4, 3));
    assert_eq!(arc.end_tangent(), p(-4, -3));
}

#[test]
fn explicit_circular_arc_tangents_respect_clockwise_direction() {
    let arc = ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Cw).unwrap();

    assert_eq!(arc.start_tangent(), p(4, -3));
    assert_eq!(arc.end_tangent(), p(4, 3));
}

#[test]
fn explicit_circular_arc_classifies_half_full_and_major_sweeps_exactly() {
    let half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, -4), ArcDirection::Ccw).unwrap();
    let quarter =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let major =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Cw).unwrap();
    let full = ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(3, 4), ArcDirection::Cw).unwrap();

    assert_eq!(half.facts().radial_cross, Real::zero());
    assert_eq!(half.facts().sweep_class, ExplicitArcSweepClass::HalfTurn);
    assert_eq!(half.certified_sweep_length(), Some(r(5) * Real::pi()));
    assert_eq!(
        quarter.certified_sweep_length(),
        Some(r(5) * Real::pi() * Real::new(Rational::fraction(1, 2).unwrap()))
    );
    assert_eq!(
        major.facts().sweep_class,
        ExplicitArcSweepClass::GreaterThanHalfTurn
    );
    assert!(major.certified_sweep_length().is_some());
    assert_eq!(full.facts().sweep_class, ExplicitArcSweepClass::FullCircle);
    assert_eq!(full.certified_sweep_length(), Some(r(10) * Real::pi()));
}

#[test]
fn explicit_circular_arc_classifies_point_membership_without_angles() {
    let minor =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    assert_eq!(
        minor.classify_point(&p(0, 5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
    assert_eq!(
        minor.classify_point(&p(0, -5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnCircleOutsideSweep
    );
    assert_eq!(
        minor.classify_point(&p(5, 0), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnCircleOutsideSweep
    );
    assert_eq!(
        minor.classify_point(&p(2, 2), PredicatePolicy::default()),
        ExplicitArcPointClassification::OffCircle
    );
    assert_eq!(
        minor.classify_point(minor.start(), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
    assert_eq!(
        minor.classify_point(minor.end(), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );

    let half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(-5, 0), ArcDirection::Ccw).unwrap();
    assert_eq!(
        half.classify_point(&p(0, 5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
    assert_eq!(
        half.classify_point(&p(0, -5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnCircleOutsideSweep
    );

    let major =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Cw).unwrap();
    assert_eq!(
        major.classify_point(&p(0, 5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnCircleOutsideSweep
    );
    assert_eq!(
        major.classify_point(&p(0, -5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
    assert_eq!(
        major.classify_point(major.start(), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
    assert_eq!(
        major.classify_point(major.end(), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );

    let full = ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(3, 4), ArcDirection::Cw).unwrap();
    assert_eq!(
        full.classify_point(&p(0, 5), PredicatePolicy::default()),
        ExplicitArcPointClassification::OnArc
    );
}

#[test]
fn explicit_circular_arc_intersects_axis_aligned_segments_exactly() {
    let minor =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();

    let chord = LinePathSegment::new(p(-10, 4), p(10, 4));
    let chord_report = minor.intersect_axis_aligned_segment(&chord, PredicatePolicy::default());
    assert_eq!(chord_report.class, LineExplicitArcIntersectionClass::Secant);
    assert_eq!(chord_report.points, vec![p(3, 4), p(-3, 4)]);

    let tangent = LinePathSegment::new(p(-10, 5), p(10, 5));
    let tangent_report = minor.intersect_axis_aligned_segment(&tangent, PredicatePolicy::default());
    assert_eq!(
        tangent_report.class,
        LineExplicitArcIntersectionClass::Tangent
    );
    assert_eq!(tangent_report.points, vec![p(0, 5)]);

    let off_sweep = LinePathSegment::new(p(-10, -5), p(10, -5));
    let off_sweep_report =
        minor.intersect_axis_aligned_segment(&off_sweep, PredicatePolicy::default());
    assert_eq!(
        off_sweep_report.class,
        LineExplicitArcIntersectionClass::Disjoint
    );
    assert!(off_sweep_report.points.is_empty());

    let outside_circle = LinePathSegment::new(p(-10, 6), p(10, 6));
    let outside_report =
        minor.intersect_axis_aligned_segment(&outside_circle, PredicatePolicy::default());
    assert_eq!(
        outside_report.class,
        LineExplicitArcIntersectionClass::Disjoint
    );
    assert!(outside_report.points.is_empty());

    let clipped = LinePathSegment::new(p(2, 4), p(10, 4));
    let clipped_report = minor.intersect_axis_aligned_segment(&clipped, PredicatePolicy::default());
    assert_eq!(
        clipped_report.class,
        LineExplicitArcIntersectionClass::Tangent
    );
    assert_eq!(clipped_report.points, vec![p(3, 4)]);

    let diagonal = LinePathSegment::new(p(-10, -10), p(10, 10));
    let diagonal_report =
        minor.intersect_axis_aligned_segment(&diagonal, PredicatePolicy::default());
    assert_eq!(
        diagonal_report.class,
        LineExplicitArcIntersectionClass::Unknown
    );
}

#[test]
fn explicit_circular_arc_classifies_same_circle_overlap() {
    let top_half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(-5, 0), ArcDirection::Ccw).unwrap();
    let same =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(-5, 0), ArcDirection::Ccw).unwrap();
    let subset =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();
    let left_half =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, 5), p(0, -5), ArcDirection::Ccw).unwrap();
    let lower_left =
        ExplicitCircularArc::new(p(0, 0), r(5), p(-5, 0), p(0, -5), ArcDirection::Ccw).unwrap();
    let lower_right =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, -5), p(5, 0), ArcDirection::Ccw).unwrap();
    let bottom_minor =
        ExplicitCircularArc::new(p(0, 0), r(5), p(-3, -4), p(3, -4), ArcDirection::Ccw).unwrap();
    let full =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(3, 4), ArcDirection::Ccw).unwrap();
    let other_circle =
        ExplicitCircularArc::new(p(10, 0), r(5), p(13, 4), p(7, 4), ArcDirection::Ccw).unwrap();

    let equal = top_half.classify_same_circle_overlap(&same, PredicatePolicy::default());
    assert_eq!(equal.class, ExplicitArcOverlapClass::Equal);
    assert_eq!(equal.shared_endpoints, vec![p(5, 0), p(-5, 0)]);

    let covers = top_half.classify_same_circle_overlap(&subset, PredicatePolicy::default());
    assert_eq!(covers.class, ExplicitArcOverlapClass::FirstCoversSecond);
    assert!(covers.shared_endpoints.is_empty());

    let covered = subset.classify_same_circle_overlap(&top_half, PredicatePolicy::default());
    assert_eq!(covered.class, ExplicitArcOverlapClass::SecondCoversFirst);

    let overlap = top_half.classify_same_circle_overlap(&left_half, PredicatePolicy::default());
    assert_eq!(overlap.class, ExplicitArcOverlapClass::Overlap);
    assert!(overlap.shared_endpoints.is_empty());

    let touch = top_half.classify_same_circle_overlap(&lower_left, PredicatePolicy::default());
    assert_eq!(touch.class, ExplicitArcOverlapClass::EndpointTouch);
    assert_eq!(touch.shared_endpoints, vec![p(-5, 0)]);

    let disjoint = top_half.classify_same_circle_overlap(&lower_right, PredicatePolicy::default());
    assert_eq!(disjoint.class, ExplicitArcOverlapClass::EndpointTouch);
    assert_eq!(disjoint.shared_endpoints, vec![p(5, 0)]);

    let disjoint = top_half.classify_same_circle_overlap(&bottom_minor, PredicatePolicy::default());
    assert_eq!(disjoint.class, ExplicitArcOverlapClass::Disjoint);
    assert!(disjoint.shared_endpoints.is_empty());

    let full_cover = full.classify_same_circle_overlap(&top_half, PredicatePolicy::default());
    assert_eq!(full_cover.class, ExplicitArcOverlapClass::FirstCoversSecond);

    let different =
        top_half.classify_same_circle_overlap(&other_circle, PredicatePolicy::default());
    assert_eq!(different.class, ExplicitArcOverlapClass::DifferentCircle);
}

#[test]
fn explicit_circular_arc_classifies_retained_circle_relation() {
    let base =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let same =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, 5), p(-5, 0), ArcDirection::Ccw).unwrap();
    let separate =
        ExplicitCircularArc::new(p(20, 0), r(5), p(25, 0), p(20, 5), ArcDirection::Ccw).unwrap();
    let external =
        ExplicitCircularArc::new(p(10, 0), r(5), p(15, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let secant =
        ExplicitCircularArc::new(p(6, 0), r(5), p(11, 0), p(6, 5), ArcDirection::Ccw).unwrap();
    let internal =
        ExplicitCircularArc::new(p(3, 0), r(2), p(5, 0), p(3, 2), ArcDirection::Ccw).unwrap();
    let contained =
        ExplicitCircularArc::new(p(1, 0), r(2), p(3, 0), p(1, 2), ArcDirection::Ccw).unwrap();

    let same_report = base.classify_circle_relation(&same, PredicatePolicy::default());
    assert_eq!(same_report.class, ExplicitCircleRelationClass::SameCircle);
    assert_eq!(same_report.center_distance_squared, r(0));
    assert_eq!(same_report.radius_sum_squared, r(100));
    assert_eq!(same_report.radius_difference_squared, r(0));

    assert_eq!(
        base.classify_circle_relation(&separate, PredicatePolicy::default())
            .class,
        ExplicitCircleRelationClass::Separate
    );
    let external_report = base.classify_circle_relation(&external, PredicatePolicy::default());
    assert_eq!(
        external_report.class,
        ExplicitCircleRelationClass::ExternallyTangent
    );
    assert_eq!(external_report.tangent_point, Some(p(5, 0)));
    assert_eq!(
        base.classify_circle_relation(&secant, PredicatePolicy::default())
            .class,
        ExplicitCircleRelationClass::Secant
    );
    let internal_report = base.classify_circle_relation(&internal, PredicatePolicy::default());
    assert_eq!(
        internal_report.class,
        ExplicitCircleRelationClass::InternallyTangent
    );
    assert_eq!(internal_report.tangent_point, Some(p(5, 0)));
    assert_eq!(
        base.classify_circle_relation(&contained, PredicatePolicy::default())
            .class,
        ExplicitCircleRelationClass::Contained
    );
}

#[test]
fn explicit_circular_arc_classifies_tangent_intersections_by_sweep_membership() {
    let base =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let tangent_on_both =
        ExplicitCircularArc::new(p(10, 0), r(5), p(5, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let tangent_outside_sweep =
        ExplicitCircularArc::new(p(10, 0), r(5), p(15, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let secant =
        ExplicitCircularArc::new(p(6, 0), r(5), p(11, 0), p(6, 5), ArcDirection::Ccw).unwrap();
    let same =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();

    let on_both = base.classify_tangent_intersection(&tangent_on_both, PredicatePolicy::default());
    assert_eq!(on_both.class, ExplicitArcTangentClass::TangentOnBoth);
    assert_eq!(
        on_both.circle_relation,
        ExplicitCircleRelationClass::ExternallyTangent
    );
    assert_eq!(on_both.tangent_point, Some(p(5, 0)));

    let outside =
        base.classify_tangent_intersection(&tangent_outside_sweep, PredicatePolicy::default());
    assert_eq!(
        outside.class,
        ExplicitArcTangentClass::CircleTangentOutsideArcSweep
    );
    assert_eq!(outside.tangent_point, Some(p(5, 0)));

    let secant_report = base.classify_tangent_intersection(&secant, PredicatePolicy::default());
    assert_eq!(
        secant_report.class,
        ExplicitArcTangentClass::NotCircleTangent
    );
    assert_eq!(
        secant_report.circle_relation,
        ExplicitCircleRelationClass::Secant
    );

    let same_report = base.classify_tangent_intersection(&same, PredicatePolicy::default());
    assert_eq!(same_report.class, ExplicitArcTangentClass::NotCircleTangent);
    assert_eq!(
        same_report.circle_relation,
        ExplicitCircleRelationClass::SameCircle
    );
}

#[test]
fn explicit_circular_arc_intersects_different_circle_arcs_exactly() {
    let full_left =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();
    let full_right =
        ExplicitCircularArc::new(p(6, 0), r(5), p(11, 0), p(11, 0), ArcDirection::Ccw).unwrap();
    let two_points = full_left.intersect_arc(&full_right, PredicatePolicy::default());
    assert_eq!(two_points.class, ExplicitArcIntersectionClass::TwoPoints);
    assert_eq!(
        two_points.circle_relation,
        ExplicitCircleRelationClass::Secant
    );
    assert_eq!(two_points.points, vec![p(3, 4), p(3, -4)]);

    let top_quarter =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let one_point = top_quarter.intersect_arc(&full_right, PredicatePolicy::default());
    assert_eq!(one_point.class, ExplicitArcIntersectionClass::OnePoint);
    assert_eq!(one_point.points, vec![p(3, 4)]);

    let external_tangent =
        ExplicitCircularArc::new(p(10, 0), r(5), p(5, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let tangent = top_quarter.intersect_arc(&external_tangent, PredicatePolicy::default());
    assert_eq!(tangent.class, ExplicitArcIntersectionClass::OnePoint);
    assert_eq!(
        tangent.circle_relation,
        ExplicitCircleRelationClass::ExternallyTangent
    );
    assert_eq!(tangent.points, vec![p(5, 0)]);

    let tangent_outside =
        ExplicitCircularArc::new(p(10, 0), r(5), p(15, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let outside = top_quarter.intersect_arc(&tangent_outside, PredicatePolicy::default());
    assert_eq!(
        outside.class,
        ExplicitArcIntersectionClass::CircleIntersectionsOutsideArcSweeps
    );
    assert!(outside.points.is_empty());

    let separate =
        ExplicitCircularArc::new(p(20, 0), r(5), p(25, 0), p(25, 0), ArcDirection::Ccw).unwrap();
    let disjoint = full_left.intersect_arc(&separate, PredicatePolicy::default());
    assert_eq!(disjoint.class, ExplicitArcIntersectionClass::Disjoint);

    let same = full_left.intersect_arc(&top_quarter, PredicatePolicy::default());
    assert_eq!(same.class, ExplicitArcIntersectionClass::SameCircle);
}

#[test]
fn explicit_circular_arc_schedules_arrangement_predicates() {
    let full_left =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();
    let full_right =
        ExplicitCircularArc::new(p(6, 0), r(5), p(11, 0), p(11, 0), ArcDirection::Ccw).unwrap();
    let two_points = full_left.arrange_with(&full_right, PredicatePolicy::default());
    assert_eq!(
        two_points.class,
        ExplicitArcArrangementClass::DifferentCircleTwoPoints
    );
    assert!(two_points.overlap.is_none());
    assert_eq!(
        two_points.intersection.unwrap().class,
        ExplicitArcIntersectionClass::TwoPoints
    );

    let top_quarter =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(0, 5), ArcDirection::Ccw).unwrap();
    let same_circle = full_left.arrange_with(&top_quarter, PredicatePolicy::default());
    assert_eq!(
        same_circle.class,
        ExplicitArcArrangementClass::SameCircleFirstCoversSecond
    );
    assert!(same_circle.intersection.is_none());
    assert_eq!(
        same_circle.overlap.unwrap().class,
        ExplicitArcOverlapClass::FirstCoversSecond
    );

    let disjoint_same_circle =
        ExplicitCircularArc::new(p(0, 0), r(5), p(0, -5), p(-5, 0), ArcDirection::Cw).unwrap();
    let disjoint = top_quarter.arrange_with(&disjoint_same_circle, PredicatePolicy::default());
    assert_eq!(
        disjoint.class,
        ExplicitArcArrangementClass::SameCircleDisjoint
    );

    let tangent_outside =
        ExplicitCircularArc::new(p(10, 0), r(5), p(15, 0), p(10, 5), ArcDirection::Ccw).unwrap();
    let outside = top_quarter.arrange_with(&tangent_outside, PredicatePolicy::default());
    assert_eq!(
        outside.class,
        ExplicitArcArrangementClass::DifferentCircleOutsideArcSweeps
    );
}

#[test]
fn explicit_circular_arc_rejects_off_circle_endpoints_and_marks_full_circle() {
    assert_eq!(
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(4, 4), ArcDirection::Cw).unwrap_err(),
        CircularArcError::EndPointOffCircle
    );
    assert_eq!(
        ExplicitCircularArc::new(p(0, 0), r(5), p(4, 4), p(3, 4), ArcDirection::Cw).unwrap_err(),
        CircularArcError::StartPointOffCircle
    );

    let full = ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(3, 4), ArcDirection::Cw).unwrap();
    assert!(full.facts().known_full_circle);
    assert_eq!(full.chord_length_squared(), Real::zero());
}

#[test]
fn cardinal_arc_offset_updates_radius_exactly() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let arc = CircularArc::cardinal_with_provenance(
        p(0, 0),
        r(10),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
        provenance,
    )
    .unwrap();

    let outward =
        offset_cardinal_arc(&arc, r(3), OffsetSide::Left, PredicatePolicy::default()).unwrap();
    let inward =
        offset_cardinal_arc(&arc, r(3), OffsetSide::Right, PredicatePolicy::default()).unwrap();

    assert_eq!(outward.arc.radius(), &r(13));
    assert_eq!(outward.arc.start(), p(13, 0));
    assert_eq!(inward.arc.radius(), &r(7));
    assert_eq!(inward.arc.end(), p(0, 7));
    assert_eq!(outward.arc.provenance(), provenance);
}

#[test]
fn explicit_circular_arc_offset_scales_endpoints_exactly() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let arc = ExplicitCircularArc::with_provenance(
        p(0, 0),
        r(5),
        p(3, 4),
        p(-3, 4),
        ArcDirection::Ccw,
        provenance,
    )
    .unwrap();

    let outward =
        offset_explicit_arc(&arc, r(5), OffsetSide::Left, PredicatePolicy::default()).unwrap();
    let inward =
        offset_explicit_arc(&arc, r(2), OffsetSide::Right, PredicatePolicy::default()).unwrap();

    assert_eq!(outward.arc.radius(), &r(10));
    assert_eq!(outward.arc.start(), &p(6, 8));
    assert_eq!(outward.arc.end(), &p(-6, 8));
    assert_eq!(outward.arc.facts().radius_squared, r(100));
    assert_eq!(outward.arc.provenance(), provenance);
    assert_eq!(inward.arc.radius(), &r(3));
    assert_eq!(
        inward.arc.start(),
        &Point2::new(
            Real::new(Rational::fraction(9, 5).unwrap()),
            Real::new(Rational::fraction(12, 5).unwrap())
        )
    );
}

#[test]
fn cardinal_arc_offset_rejects_negative_distance_and_radius_collapse() {
    let arc = CircularArc::cardinal(
        p(0, 0),
        r(5),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
    )
    .unwrap();

    assert_eq!(
        offset_cardinal_arc(&arc, r(-1), OffsetSide::Left, PredicatePolicy::default()).unwrap_err(),
        ArcOffsetError::NegativeDistance
    );
    assert_eq!(
        offset_cardinal_arc(&arc, r(5), OffsetSide::Right, PredicatePolicy::default()).unwrap_err(),
        ArcOffsetError::RadiusWouldCollapse
    );
    assert_eq!(
        offset_cardinal_arc(&arc, r(6), OffsetSide::Right, PredicatePolicy::default()).unwrap_err(),
        ArcOffsetError::RadiusWouldCollapse
    );
}

#[test]
fn explicit_circular_arc_offset_rejects_negative_distance_and_radius_collapse() {
    let arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(3, 4), p(-3, 4), ArcDirection::Ccw).unwrap();

    assert_eq!(
        offset_explicit_arc(&arc, r(-1), OffsetSide::Left, PredicatePolicy::default()).unwrap_err(),
        ArcOffsetError::NegativeDistance
    );
    assert_eq!(
        offset_explicit_arc(&arc, r(5), OffsetSide::Right, PredicatePolicy::default()).unwrap_err(),
        ArcOffsetError::RadiusWouldCollapse
    );
}

#[test]
fn pcb_clearance_certifies_same_layer_parallel_gap() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 2);
    let second = trace(2, 0, p(0, 5), p(10, 5), 2);

    let clear = check_trace_clearance(&first, &second, &r(3), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.axis_gap, Some(r(5)));

    let violation = check_trace_clearance(&first, &second, &r(4), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_clearance_reports_no_short_before_spacing() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 1);
    let second = trace(2, 0, p(5, -5), p(5, 5), 1);

    let report = check_trace_clearance(&first, &second, &r(1), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_clearance_ignores_same_net_and_different_layer_pairs() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 1);
    let same_net = trace(1, 0, p(5, -5), p(5, 5), 1);
    let other_layer = trace(2, 1, p(5, -5), p(5, 5), 1);

    assert_eq!(
        check_trace_clearance(&first, &same_net, &r(1), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
    assert_eq!(
        check_trace_clearance(&first, &other_layer, &r(1), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
}

#[test]
fn pcb_trace_pad_clearance_certifies_round_pad_gap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 5), r(2)).unwrap();

    let clear = check_trace_pad_clearance(&trace, &pad, &r(3), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);

    let violation = check_trace_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_pad_clearance_reports_copper_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 1), r(2)).unwrap();

    let report = check_trace_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_pad_rejects_negative_diameter() {
    let error = PcbCircularPad::new(NetId(1), TraceLayer(0), p(0, 0), r(-1))
        .expect_err("negative pad diameter must be rejected");
    assert_eq!(error, "pad diameter must be nonnegative");
}

#[test]
fn pcb_trace_via_clearance_respects_layer_span() {
    let trace = trace(1, 1, p(0, 0), p(10, 0), 2);
    let via = PcbViaStack::new(NetId(2), TraceLayer(0), TraceLayer(2), p(5, 5), r(2)).unwrap();
    let off_layer =
        PcbViaStack::new(NetId(2), TraceLayer(2), TraceLayer(3), p(5, 5), r(2)).unwrap();

    assert_eq!(
        check_trace_via_clearance(&trace, &via, &r(3), PredicatePolicy::default()).status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_via_clearance(&trace, &off_layer, &r(3), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
}

#[test]
fn pcb_trace_via_drill_clearance_uses_exact_hole_keepout() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let via = PcbViaStack::with_drill(NetId(2), TraceLayer(0), TraceLayer(2), p(5, 6), r(10), r(2))
        .unwrap();
    let no_drill =
        PcbViaStack::new(NetId(2), TraceLayer(0), TraceLayer(2), p(5, 6), r(10)).unwrap();
    let off_layer =
        PcbViaStack::with_drill(NetId(2), TraceLayer(1), TraceLayer(2), p(5, 6), r(10), r(2))
            .unwrap();

    assert_eq!(
        check_trace_via_drill_clearance(&trace, &via, &r(4), PredicatePolicy::default()).status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_via_drill_clearance(&trace, &via, &r(5), PredicatePolicy::default()).status,
        ClearanceStatus::ClearanceViolation
    );
    assert_eq!(
        check_trace_via_drill_clearance(&trace, &no_drill, &r(1), PredicatePolicy::default())
            .status,
        ClearanceStatus::Unknown
    );
    assert_eq!(
        check_trace_via_drill_clearance(&trace, &off_layer, &r(1), PredicatePolicy::default())
            .status,
        ClearanceStatus::NotApplicable
    );
}

#[test]
fn pcb_trace_via_drill_clearance_reports_drill_cutting_copper() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let via = PcbViaStack::with_drill(NetId(2), TraceLayer(0), TraceLayer(2), p(5, 1), r(10), r(2))
        .unwrap();

    assert_eq!(
        check_trace_via_drill_clearance(&trace, &via, &r(0), PredicatePolicy::default()).status,
        ClearanceStatus::NoShortViolation
    );
}

#[test]
fn pcb_via_rejects_reversed_layer_span() {
    let error = PcbViaStack::new(NetId(1), TraceLayer(3), TraceLayer(2), p(0, 0), r(1))
        .expect_err("reversed via layer span must be rejected");
    assert_eq!(error, "via start layer must not be above end layer");
}

#[test]
fn pcb_via_classifies_layer_transitions_against_board_stackup() {
    let through = PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(3), p(0, 0), r(10)).unwrap();
    let blind = PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(1), p(0, 0), r(10)).unwrap();
    let buried = PcbViaStack::new(NetId(1), TraceLayer(1), TraceLayer(2), p(0, 0), r(10)).unwrap();
    let land = PcbViaStack::new(NetId(1), TraceLayer(2), TraceLayer(2), p(0, 0), r(10)).unwrap();
    let outside = PcbViaStack::new(NetId(1), TraceLayer(2), TraceLayer(4), p(0, 0), r(10)).unwrap();

    let through_report = through.classify_layer_transition(4).unwrap();
    assert_eq!(through_report.class, ViaLayerTransitionClass::ThroughVia);
    assert_eq!(through_report.spanned_layers, 4);
    assert_eq!(through_report.start_layer, TraceLayer(0));
    assert_eq!(through_report.end_layer, TraceLayer(3));

    assert_eq!(
        blind.classify_layer_transition(4).unwrap().class,
        ViaLayerTransitionClass::BlindVia
    );
    assert_eq!(
        buried.classify_layer_transition(4).unwrap().class,
        ViaLayerTransitionClass::BuriedVia
    );
    assert_eq!(
        land.classify_layer_transition(4).unwrap().class,
        ViaLayerTransitionClass::SingleLayerLand
    );
    assert_eq!(
        through.classify_layer_transition(0).unwrap_err(),
        "board layer count must be positive"
    );
    assert_eq!(
        outside.classify_layer_transition(4).unwrap_err(),
        "via layer span exceeds board layer count"
    );
}

#[test]
fn pcb_via_classifies_layer_span_relations_exactly() {
    let first = PcbViaStack::new(NetId(1), TraceLayer(1), TraceLayer(3), p(0, 0), r(10)).unwrap();
    let overlap = PcbViaStack::new(NetId(2), TraceLayer(2), TraceLayer(4), p(1, 0), r(10)).unwrap();
    let touching =
        PcbViaStack::new(NetId(2), TraceLayer(3), TraceLayer(5), p(1, 0), r(10)).unwrap();
    let adjacent =
        PcbViaStack::new(NetId(2), TraceLayer(4), TraceLayer(6), p(1, 0), r(10)).unwrap();
    let disjoint =
        PcbViaStack::new(NetId(2), TraceLayer(5), TraceLayer(6), p(1, 0), r(10)).unwrap();

    let overlap_report = first.classify_layer_span_with(&overlap);
    assert_eq!(
        overlap_report.relation,
        ViaLayerSpanRelation::OverlappingLayers
    );
    assert_eq!(overlap_report.overlap_start, Some(TraceLayer(2)));
    assert_eq!(overlap_report.overlap_end, Some(TraceLayer(3)));
    assert_eq!(overlap_report.shared_layers, 2);

    let touching_report = first.classify_layer_span_with(&touching);
    assert_eq!(
        touching_report.relation,
        ViaLayerSpanRelation::TouchingLayer
    );
    assert_eq!(touching_report.overlap_start, Some(TraceLayer(3)));
    assert_eq!(touching_report.overlap_end, Some(TraceLayer(3)));
    assert_eq!(touching_report.shared_layers, 1);

    let adjacent_report = first.classify_layer_span_with(&adjacent);
    assert_eq!(
        adjacent_report.relation,
        ViaLayerSpanRelation::AdjacentBelow
    );
    assert_eq!(adjacent_report.shared_layers, 0);
    assert_eq!(
        adjacent.classify_layer_span_with(&first).relation,
        ViaLayerSpanRelation::AdjacentAbove
    );
    assert_eq!(
        first.classify_layer_span_with(&disjoint).relation,
        ViaLayerSpanRelation::DisjointBelow
    );
    assert_eq!(
        disjoint.classify_layer_span_with(&first).relation,
        ViaLayerSpanRelation::DisjointAbove
    );
}

#[test]
fn pcb_via_annular_ring_certifies_fabrication_requirement() {
    let via = PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(2), p(0, 0), r(10), r(4))
        .unwrap();

    assert_eq!(
        via.certify_annular_ring(&r(3), PredicatePolicy::default()),
        ViaAnnularRingReport::Certified
    );
    assert_eq!(
        via.certify_annular_ring(&r(4), PredicatePolicy::default()),
        ViaAnnularRingReport::Violation
    );
}

#[test]
fn pcb_via_annular_ring_reports_missing_and_invalid_inputs() {
    let without_drill =
        PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(1), p(0, 0), r(10)).unwrap();
    let negative_drill = PcbViaStack::with_drill(
        NetId(1),
        TraceLayer(0),
        TraceLayer(1),
        p(0, 0),
        r(10),
        r(-1),
    )
    .expect_err("negative drill diameter must be rejected");

    assert_eq!(
        without_drill.certify_annular_ring(&r(1), PredicatePolicy::default()),
        ViaAnnularRingReport::UnknownNoDrill
    );
    assert_eq!(
        without_drill.certify_annular_ring(&r(-1), PredicatePolicy::default()),
        ViaAnnularRingReport::UnknownNoDrill
    );
    assert_eq!(negative_drill, "via drill diameter must be nonnegative");
}

#[test]
fn pcb_via_drill_policy_separates_plated_nonplated_and_missing_intent() {
    let missing = PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(1), p(0, 0), r(10)).unwrap();
    let plated =
        PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(1), p(0, 0), r(10), r(4))
            .unwrap();
    let non_plated = PcbViaStack::with_drill_intent(
        NetId(1),
        TraceLayer(0),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        ViaDrillIntent::NonPlated,
    )
    .unwrap();
    let unspecified = PcbViaStack::with_drill_intent(
        NetId(1),
        TraceLayer(0),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        ViaDrillIntent::Unspecified,
    )
    .unwrap();

    let missing_report = missing.classify_drill_policy(&r(3), PredicatePolicy::default());
    assert_eq!(missing_report.class, ViaDrillPolicyClass::MissingDrill);
    assert_eq!(missing_report.drill_diameter, None);
    assert_eq!(missing_report.annular_ring, None);

    let plated_report = plated.classify_drill_policy(&r(3), PredicatePolicy::default());
    assert_eq!(plated_report.class, ViaDrillPolicyClass::PlatedCopperVia);
    assert_eq!(plated_report.intent, ViaDrillIntent::Plated);
    assert_eq!(plated_report.drill_diameter, Some(r(4)));
    assert_eq!(
        plated_report.annular_ring,
        Some(ViaAnnularRingReport::Certified)
    );

    let non_plated_report = non_plated.classify_drill_policy(&r(3), PredicatePolicy::default());
    assert_eq!(
        non_plated_report.class,
        ViaDrillPolicyClass::NonPlatedMechanicalHole
    );
    assert_eq!(non_plated_report.intent, ViaDrillIntent::NonPlated);
    assert_eq!(non_plated_report.annular_ring, None);

    let unspecified_report = unspecified.classify_drill_policy(&r(3), PredicatePolicy::default());
    assert_eq!(
        unspecified_report.class,
        ViaDrillPolicyClass::UnspecifiedDrilledHole
    );
    assert_eq!(unspecified_report.intent, ViaDrillIntent::Unspecified);
    assert_eq!(unspecified_report.annular_ring, None);
}

#[test]
fn pcb_trace_rect_pad_clearance_certifies_non_circular_pad_gap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRectPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4), r(2)).unwrap();

    let clear = check_trace_rect_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);

    let violation = check_trace_rect_pad_clearance(&trace, &pad, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_rect_pad_clearance_reports_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRectPad::new(NetId(2), TraceLayer(0), p(5, 1), r(4), r(2)).unwrap();

    let report = check_trace_rect_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_trace_rounded_rect_pad_clearance_certifies_corner_radius_gap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRoundedRectPad::new(NetId(2), TraceLayer(0), p(5, 8), r(6), r(4), r(1)).unwrap();

    let tangent =
        check_trace_rounded_rect_pad_clearance(&trace, &pad, &r(5), PredicatePolicy::default());
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);

    let violation =
        check_trace_rounded_rect_pad_clearance(&trace, &pad, &r(6), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_rounded_rect_pad_clearance_reports_corner_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRoundedRectPad::new(NetId(2), TraceLayer(0), p(5, 2), r(6), r(4), r(1)).unwrap();

    let report =
        check_trace_rounded_rect_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_rounded_rect_pad_zero_radius_matches_rect_pad_predicate() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let rect = PcbRectPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4), r(2)).unwrap();
    let rounded =
        PcbRoundedRectPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4), r(2), r(0)).unwrap();

    let rect_report =
        check_trace_rect_pad_clearance(&trace, &rect, &r(4), PredicatePolicy::default());
    let rounded_report =
        check_trace_rounded_rect_pad_clearance(&trace, &rounded, &r(4), PredicatePolicy::default());
    assert_eq!(rounded_report.status, rect_report.status);
}

#[test]
fn pcb_rounded_square_with_half_radius_matches_circular_pad_predicate() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let circular = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4)).unwrap();
    let rounded =
        PcbRoundedRectPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4), r(4), r(2)).unwrap();

    let circular_report =
        check_trace_pad_clearance(&trace, &circular, &r(3), PredicatePolicy::default());
    let rounded_report =
        check_trace_rounded_rect_pad_clearance(&trace, &rounded, &r(3), PredicatePolicy::default());
    assert_eq!(rounded_report.status, circular_report.status);
}

#[test]
fn pcb_rounded_rect_pad_rejects_invalid_radius() {
    let negative = PcbRoundedRectPad::new(NetId(1), TraceLayer(0), p(0, 0), r(4), r(2), r(-1))
        .expect_err("negative rounded rectangular pad radius must be rejected");
    assert_eq!(
        negative,
        "rounded rect pad corner radius must be nonnegative"
    );

    let too_large = PcbRoundedRectPad::new(NetId(1), TraceLayer(0), p(0, 0), r(4), r(2), r(2))
        .expect_err("radius larger than half of a pad extent must be rejected");
    assert_eq!(
        too_large,
        "rounded rect pad corner radius must not exceed half extent"
    );
}

#[test]
fn pcb_trace_obround_pad_clearance_certifies_general_spine_gap() {
    let trace = trace(1, 0, p(0, 8), p(10, 8), 2);
    let pad = PcbObroundPad::new(
        NetId(2),
        TraceLayer(0),
        LinePathSegment::new(p(0, 0), p(10, 0)),
        r(2),
    )
    .unwrap();

    let tangent =
        check_trace_obround_pad_clearance(&trace, &pad, &r(6), PredicatePolicy::default());
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);

    let violation =
        check_trace_obround_pad_clearance(&trace, &pad, &r(7), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_obround_pad_clearance_reports_diagonal_spine_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbObroundPad::new(
        NetId(2),
        TraceLayer(0),
        LinePathSegment::new(p(5, -5), p(5, 5)),
        r(2),
    )
    .unwrap();

    let report = check_trace_obround_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_degenerate_obround_pad_matches_circular_pad_predicate() {
    let trace = trace(1, 0, p(0, 6), p(10, 6), 2);
    let circular = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 0), r(4)).unwrap();
    let obround = PcbObroundPad::new(
        NetId(2),
        TraceLayer(0),
        LinePathSegment::new(p(5, 0), p(5, 0)),
        r(4),
    )
    .unwrap();

    assert_eq!(obround.facts().degenerate_spine, Some(true));
    assert_eq!(
        check_trace_obround_pad_clearance(&trace, &obround, &r(3), PredicatePolicy::default())
            .status,
        check_trace_pad_clearance(&trace, &circular, &r(3), PredicatePolicy::default()).status
    );
}

#[test]
fn pcb_obround_pad_rejects_negative_diameter() {
    let error = PcbObroundPad::new(
        NetId(2),
        TraceLayer(0),
        LinePathSegment::new(p(0, 0), p(1, 0)),
        r(-1),
    )
    .expect_err("negative obround pad diameter must be rejected");
    assert_eq!(error, "obround pad diameter must be nonnegative");
}

#[test]
fn pcb_convex_pad_validates_strict_convexity() {
    let diamond = PcbConvexPad::new(
        NetId(2),
        TraceLayer(0),
        vec![p(0, 5), p(5, 0), p(0, -5), p(-5, 0)],
    )
    .unwrap();
    assert_eq!(diamond.orientation(), BoardContourOrientation::Clockwise);
    assert_eq!(diamond.vertices().len(), 4);

    let too_few = PcbConvexPad::new(NetId(2), TraceLayer(0), vec![p(0, 0), p(1, 0)])
        .expect_err("two vertices cannot define a convex pad");
    assert_eq!(too_few, BoardContourError::TooFewVertices);

    let collinear = PcbConvexPad::new(NetId(2), TraceLayer(0), vec![p(0, 0), p(1, 0), p(2, 0)])
        .expect_err("zero-area pad must be rejected");
    assert_eq!(collinear, BoardContourError::DegenerateArea);

    let nonconvex = PcbConvexPad::new(
        NetId(2),
        TraceLayer(0),
        vec![p(0, 0), p(4, 0), p(1, 1), p(0, 4)],
    )
    .expect_err("concave pad must be rejected");
    assert_eq!(nonconvex, BoardContourError::NonConvex);
}

#[test]
fn pcb_trace_convex_pad_clearance_certifies_polygon_gap() {
    let trace = trace(1, 0, p(-2, 10), p(2, 10), 2);
    let pad = PcbConvexPad::new(
        NetId(2),
        TraceLayer(0),
        vec![p(0, 5), p(5, 0), p(0, -5), p(-5, 0)],
    )
    .unwrap();

    let tangent = check_trace_convex_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);

    let violation =
        check_trace_convex_pad_clearance(&trace, &pad, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_convex_pad_clearance_reports_overlap_and_not_applicable() {
    let crossing_trace = trace(1, 0, p(-10, 0), p(10, 0), 2);
    let same_net = trace(2, 0, p(-10, 0), p(10, 0), 2);
    let pad = PcbConvexPad::new(
        NetId(2),
        TraceLayer(0),
        vec![p(0, 5), p(5, 0), p(0, -5), p(-5, 0)],
    )
    .unwrap();

    let report =
        check_trace_convex_pad_clearance(&crossing_trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);

    let skipped =
        check_trace_convex_pad_clearance(&same_net, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(skipped.status, ClearanceStatus::NotApplicable);
}

#[test]
fn pcb_rect_pad_rejects_negative_extent() {
    let error = PcbRectPad::new(NetId(1), TraceLayer(0), p(0, 0), r(-1), r(1))
        .expect_err("negative rectangular pad width must be rejected");
    assert_eq!(error, "rect pad width must be nonnegative");
}

#[test]
fn pcb_cardinal_rect_pad_swaps_effective_extents_exactly() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::KiCad, 1_000_000).unwrap();
    let pad = PcbCardinalRectPad::with_provenance(
        NetId(2),
        TraceLayer(0),
        p(5, 6),
        r(8),
        r(2),
        CardinalRotation::Deg90,
        provenance,
    )
    .unwrap();
    let effective = pad.effective_rect().unwrap();

    assert_eq!(effective.width(), &r(2));
    assert_eq!(effective.height(), &r(8));
    assert_eq!(effective.provenance(), provenance);
    assert_eq!(pad.rotation(), CardinalRotation::Deg90);
}

#[test]
fn pcb_trace_cardinal_rect_pad_clearance_uses_rotated_extents() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let wide_horizontal = PcbCardinalRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(5, 8),
        r(8),
        r(2),
        CardinalRotation::Deg0,
    )
    .unwrap();
    let wide_vertical = PcbCardinalRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(5, 8),
        r(8),
        r(2),
        CardinalRotation::Deg90,
    )
    .unwrap();

    assert_eq!(
        check_trace_cardinal_rect_pad_clearance(
            &trace,
            &wide_horizontal,
            &r(4),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_cardinal_rect_pad_clearance(
            &trace,
            &wide_vertical,
            &r(4),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
}

#[test]
fn pcb_cardinal_rect_pad_rejects_negative_extent() {
    let error = PcbCardinalRectPad::new(
        NetId(1),
        TraceLayer(0),
        p(0, 0),
        r(1),
        r(-1),
        CardinalRotation::Deg0,
    )
    .expect_err("negative cardinal rectangular pad height must be rejected");

    assert_eq!(error, "cardinal rect pad height must be nonnegative");
}

#[test]
fn pcb_oriented_rect_pad_accepts_exact_pythagorean_axis() {
    let pad = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        Point2::new(rq(3, 5), rq(4, 5)),
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(pad.local_x(), &Point2::new(rq(3, 5), rq(4, 5)));
    assert_eq!(pad.local_y(), Point2::new(rq(-4, 5), rq(3, 5)));
    assert_eq!(pad.facts().local_x_length_squared, r(1));
}

#[test]
fn pcb_oriented_rect_pad_rejects_non_unit_axis_and_negative_extent() {
    let non_unit = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        Point2::new(r(1), r(1)),
        PredicatePolicy::default(),
    )
    .expect_err("non-unit orientation must not be normalized silently");
    assert_eq!(
        non_unit,
        "oriented rect pad local x axis must be exact unit length"
    );

    let negative = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(0, 0),
        r(-10),
        r(4),
        Point2::new(r(1), r(0)),
        PredicatePolicy::default(),
    )
    .expect_err("negative oriented pad width must be rejected");
    assert_eq!(negative, "oriented rect pad width must be nonnegative");
}

#[test]
fn pcb_trace_oriented_rect_pad_clearance_certifies_rotated_gap() {
    let pad = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        Point2::new(rq(3, 5), rq(4, 5)),
        PredicatePolicy::default(),
    )
    .unwrap();
    let trace = PcbTrace::new(
        NetId(1),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(pq(-58, 5, -19, 5), pq(2, 5, 61, 5)),
            r(2),
        )
        .unwrap(),
    );

    let tangent =
        check_trace_oriented_rect_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);

    let violation =
        check_trace_oriented_rect_pad_clearance(&trace, &pad, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_oriented_rect_pad_clearance_reports_rotated_overlap() {
    let pad = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        Point2::new(rq(3, 5), rq(4, 5)),
        PredicatePolicy::default(),
    )
    .unwrap();
    let trace = PcbTrace::new(
        NetId(1),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(pq(-38, 5, -34, 5), pq(22, 5, 46, 5)),
            r(2),
        )
        .unwrap(),
    );

    let report =
        check_trace_oriented_rect_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_board_outline_rejects_reversed_bounds_and_retains_provenance() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::KiCad, 1_000_000).unwrap();
    let board = PcbBoardOutline::with_provenance(p(0, 0), p(20, 10), provenance).unwrap();

    assert_eq!(board.min(), &p(0, 0));
    assert_eq!(board.max(), &p(20, 10));
    assert_eq!(board.provenance(), provenance);
    assert!(board.exact_facts().all_exact_rational);
    assert_eq!(
        PcbBoardOutline::new(p(10, 0), p(0, 10)).unwrap_err(),
        "board outline x bounds must be ordered"
    );
    assert_eq!(
        PcbBoardOutline::new(p(0, 10), p(10, 0)).unwrap_err(),
        "board outline y bounds must be ordered"
    );
}

#[test]
fn pcb_convex_board_outline_validates_orientation_and_convexity() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::KiCad, 1_000_000).unwrap();
    let board = PcbConvexBoardOutline::with_provenance(
        vec![p(0, 0), p(20, 0), p(25, 10), p(0, 10)],
        provenance,
    )
    .unwrap();

    assert_eq!(board.vertices().len(), 4);
    assert_eq!(
        board.orientation(),
        BoardContourOrientation::CounterClockwise
    );
    assert_eq!(board.provenance(), provenance);
    assert!(board.exact_facts().all_exact_rational);
    assert_eq!(
        PcbConvexBoardOutline::new(vec![p(0, 0), p(1, 0)]).unwrap_err(),
        BoardContourError::TooFewVertices
    );
    assert_eq!(
        PcbConvexBoardOutline::new(vec![p(0, 0), p(1, 0), p(2, 0)]).unwrap_err(),
        BoardContourError::DegenerateArea
    );
    assert_eq!(
        PcbConvexBoardOutline::new(vec![p(0, 0), p(2, 0), p(1, 1), p(2, 2), p(0, 2)]).unwrap_err(),
        BoardContourError::NonConvex
    );
}

#[test]
fn pcb_orthogonal_board_outline_validates_nonconvex_simple_contours() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::KiCad, 1_000_000).unwrap();
    let board = PcbOrthogonalBoardOutline::with_provenance(
        vec![
            p(0, 0),
            p(20, 0),
            p(20, 10),
            p(12, 10),
            p(12, 4),
            p(8, 4),
            p(8, 10),
            p(0, 10),
        ],
        provenance,
    )
    .unwrap();

    assert_eq!(board.vertices().len(), 8);
    assert_eq!(
        board.orientation(),
        BoardContourOrientation::CounterClockwise
    );
    assert_eq!(board.provenance(), provenance);
    assert!(board.exact_facts().all_exact_rational);
    assert_eq!(
        PcbOrthogonalBoardOutline::new(vec![p(0, 0), p(2, 0), p(3, 1), p(0, 1)]).unwrap_err(),
        BoardContourError::NonOrthogonal
    );
    assert_eq!(
        PcbOrthogonalBoardOutline::new(vec![
            p(0, 0),
            p(4, 0),
            p(4, 4),
            p(2, 4),
            p(2, -1),
            p(0, -1),
        ])
        .unwrap_err(),
        BoardContourError::SelfIntersecting
    );
}

#[test]
fn pcb_trace_convex_board_clearance_certifies_slanted_edge_gap() {
    let board = PcbConvexBoardOutline::new(vec![p(0, 0), p(20, 0), p(25, 10), p(0, 10)]).unwrap();
    let centered = trace(1, 0, p(5, 5), p(10, 5), 2);
    let near_bottom = trace(1, 0, p(5, 1), p(10, 1), 2);
    let outside_slant = trace(1, 0, p(24, 9), p(25, 9), 0);

    assert_eq!(
        check_trace_convex_board_clearance(&centered, &board, &r(1), PredicatePolicy::default())
            .status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_convex_board_clearance(&near_bottom, &board, &r(1), PredicatePolicy::default())
            .status,
        ClearanceStatus::ClearanceViolation
    );
    assert_eq!(
        check_trace_convex_board_clearance(
            &outside_slant,
            &board,
            &r(0),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
}

#[test]
fn pcb_trace_orthogonal_board_clearance_handles_nonconvex_notches_exactly() {
    let board = PcbOrthogonalBoardOutline::new(vec![
        p(0, 0),
        p(20, 0),
        p(20, 10),
        p(12, 10),
        p(12, 4),
        p(8, 4),
        p(8, 10),
        p(0, 10),
    ])
    .unwrap();
    let clear = trace(1, 0, p(2, 2), p(18, 2), 2);
    let near_notch = trace(1, 0, p(6, 5), p(7, 5), 2);
    let crossing_notch = trace(1, 0, p(6, 5), p(14, 5), 0);
    let outside_notch = trace(1, 0, p(9, 6), p(11, 6), 0);

    assert_eq!(
        check_trace_orthogonal_board_clearance(&clear, &board, &r(1), PredicatePolicy::default())
            .status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_orthogonal_board_clearance(
            &near_notch,
            &board,
            &r(1),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
    assert_eq!(
        check_trace_orthogonal_board_clearance(
            &crossing_notch,
            &board,
            &r(0),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
    assert_eq!(
        check_trace_orthogonal_board_clearance(
            &outside_notch,
            &board,
            &r(0),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
}

#[test]
fn pcb_trace_board_clearance_certifies_inside_gap_and_edge_violation() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let centered = trace(1, 0, p(3, 5), p(17, 5), 2);
    let near_edge = trace(1, 0, p(1, 5), p(17, 5), 2);
    let outside = trace(1, 0, p(-1, 5), p(17, 5), 2);

    let clear = check_trace_board_clearance(&centered, &board, &r(2), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.axis_gap, Some(r(3)));

    let violation =
        check_trace_board_clearance(&near_edge, &board, &r(1), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.axis_gap, Some(r(1)));

    let outside_report =
        check_trace_board_clearance(&outside, &board, &r(0), PredicatePolicy::default());
    assert_eq!(outside_report.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(outside_report.axis_gap, Some(r(-1)));
}

#[test]
fn pcb_circular_board_outline_rejects_negative_radius() {
    let error = PcbCircularBoardOutline::new(p(0, 0), r(-1))
        .expect_err("negative circular board radius must be rejected");
    assert_eq!(error, "circular board radius must be nonnegative");
}

#[test]
fn pcb_trace_circular_board_clearance_certifies_diagonal_endpoint_radius() {
    let board = PcbCircularBoardOutline::new(p(0, 0), r(10)).unwrap();
    let trace = trace(1, 0, p(-3, -4), p(3, 4), 2);

    let tangent =
        check_trace_circular_board_clearance(&trace, &board, &r(4), PredicatePolicy::default());
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);

    let violation =
        check_trace_circular_board_clearance(&trace, &board, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_circular_board_clearance_rejects_impossible_allowance_before_squaring() {
    let board = PcbCircularBoardOutline::new(p(0, 0), r(3)).unwrap();
    let trace = trace(1, 0, p(0, 0), p(0, 0), 8);

    let report =
        check_trace_circular_board_clearance(&trace, &board, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_circular_pad_circular_board_clearance_uses_exact_center_radius_sum() {
    let board = PcbCircularBoardOutline::new(p(0, 0), r(10)).unwrap();
    let pad = PcbCircularPad::new(NetId(1), TraceLayer(0), p(3, 4), r(4)).unwrap();

    let tangent = check_circular_pad_circular_board_clearance(
        &pad,
        &board,
        &r(3),
        PredicatePolicy::default(),
    );
    assert_eq!(tangent.status, ClearanceStatus::CertifiedClear);
    assert_eq!(tangent.copper_gap, None);

    let violation = check_circular_pad_circular_board_clearance(
        &pad,
        &board,
        &r(4),
        PredicatePolicy::default(),
    );
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_via_drill_board_clearance_certifies_edge_gap_and_missing_drill() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let centered =
        PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(2), p(10, 5), r(8), r(2))
            .unwrap();
    let near_edge =
        PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(2), p(2, 5), r(8), r(2))
            .unwrap();
    let no_drill =
        PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(2), p(10, 5), r(8)).unwrap();

    assert_eq!(
        check_via_drill_board_clearance(&centered, &board, &r(3), PredicatePolicy::default()),
        DrillBoardClearanceReport {
            status: ClearanceStatus::CertifiedClear,
            axis_gap: Some(r(5)),
            missing_drill: false,
        }
    );
    assert_eq!(
        check_via_drill_board_clearance(&near_edge, &board, &r(2), PredicatePolicy::default()),
        DrillBoardClearanceReport {
            status: ClearanceStatus::ClearanceViolation,
            axis_gap: Some(r(2)),
            missing_drill: false,
        }
    );
    assert_eq!(
        check_via_drill_board_clearance(&no_drill, &board, &r(1), PredicatePolicy::default()),
        DrillBoardClearanceReport {
            status: ClearanceStatus::Unknown,
            axis_gap: None,
            missing_drill: true,
        }
    );
}

#[test]
fn pcb_via_drill_board_clearance_reports_outside_board() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let outside =
        PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(2), p(-1, 5), r(8), r(2))
            .unwrap();

    let report =
        check_via_drill_board_clearance(&outside, &board, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(report.axis_gap, Some(r(-1)));
    assert!(!report.missing_drill);
}

#[test]
fn pcb_circular_pad_board_clearance_certifies_edge_gap() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let centered = PcbCircularPad::new(NetId(1), TraceLayer(0), p(10, 5), r(4)).unwrap();
    let near_edge = PcbCircularPad::new(NetId(1), TraceLayer(0), p(3, 5), r(4)).unwrap();
    let outside = PcbCircularPad::new(NetId(1), TraceLayer(0), p(1, 5), r(4)).unwrap();

    let clear =
        check_circular_pad_board_clearance(&centered, &board, &r(3), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(r(3)));

    let violation =
        check_circular_pad_board_clearance(&near_edge, &board, &r(2), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(r(1)));

    let outside_report =
        check_circular_pad_board_clearance(&outside, &board, &r(0), PredicatePolicy::default());
    assert_eq!(outside_report.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(outside_report.copper_gap, Some(r(-1)));
}

#[test]
fn pcb_rect_pad_board_clearance_uses_copper_edges() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let centered = PcbRectPad::new(NetId(1), TraceLayer(0), p(10, 5), r(4), r(2)).unwrap();
    let near_edge = PcbRectPad::new(NetId(1), TraceLayer(0), p(3, 5), r(4), r(2)).unwrap();

    let clear =
        check_rect_pad_board_clearance(&centered, &board, &r(4), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(r(4)));

    let violation =
        check_rect_pad_board_clearance(&near_edge, &board, &r(2), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(r(1)));
}

#[test]
fn pcb_rounded_rect_pad_board_clearance_uses_outer_copper_bounds() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let centered =
        PcbRoundedRectPad::new(NetId(1), TraceLayer(0), p(10, 5), r(4), r(2), r(1)).unwrap();
    let near_edge =
        PcbRoundedRectPad::new(NetId(1), TraceLayer(0), p(3, 5), r(4), r(2), r(1)).unwrap();

    let clear = check_rounded_rect_pad_board_clearance(
        &centered,
        &board,
        &r(4),
        PredicatePolicy::default(),
    );
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(r(4)));

    let violation = check_rounded_rect_pad_board_clearance(
        &near_edge,
        &board,
        &r(2),
        PredicatePolicy::default(),
    );
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(r(1)));
}

#[test]
fn pcb_obround_pad_board_clearance_uses_spine_extrema_plus_radius() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let pad = PcbObroundPad::new(
        NetId(1),
        TraceLayer(0),
        LinePathSegment::new(p(4, 5), p(16, 5)),
        r(4),
    )
    .unwrap();

    let clear = check_obround_pad_board_clearance(&pad, &board, &r(2), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(r(2)));

    let violation =
        check_obround_pad_board_clearance(&pad, &board, &r(3), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(r(2)));
}

#[test]
fn pcb_convex_pad_board_clearance_uses_vertex_extrema() {
    let board = PcbBoardOutline::new(p(-10, -10), p(10, 10)).unwrap();
    let pad = PcbConvexPad::new(
        NetId(1),
        TraceLayer(0),
        vec![p(0, 5), p(5, 0), p(0, -5), p(-5, 0)],
    )
    .unwrap();

    let clear = check_convex_pad_board_clearance(&pad, &board, &r(5), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(r(5)));

    let violation =
        check_convex_pad_board_clearance(&pad, &board, &r(6), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(r(5)));
}

#[test]
fn pcb_cardinal_rect_pad_board_clearance_uses_rotated_extents() {
    let board = PcbBoardOutline::new(p(0, 0), p(20, 10)).unwrap();
    let horizontal = PcbCardinalRectPad::new(
        NetId(1),
        TraceLayer(0),
        p(10, 3),
        r(8),
        r(2),
        CardinalRotation::Deg0,
    )
    .unwrap();
    let vertical = PcbCardinalRectPad::new(
        NetId(1),
        TraceLayer(0),
        p(10, 3),
        r(8),
        r(2),
        CardinalRotation::Deg90,
    )
    .unwrap();

    assert_eq!(
        check_cardinal_rect_pad_board_clearance(
            &horizontal,
            &board,
            &r(2),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_cardinal_rect_pad_board_clearance(
            &vertical,
            &board,
            &r(0),
            PredicatePolicy::default()
        )
        .status,
        ClearanceStatus::ClearanceViolation
    );
}

#[test]
fn pcb_oriented_rect_pad_board_clearance_uses_rotated_corner_extrema() {
    let board = PcbBoardOutline::new(p(-10, -10), p(10, 10)).unwrap();
    let pad = PcbOrientedRectPad::new(
        NetId(1),
        TraceLayer(0),
        p(0, 0),
        r(10),
        r(4),
        Point2::new(rq(3, 5), rq(4, 5)),
        PredicatePolicy::default(),
    )
    .unwrap();

    let clear =
        check_oriented_rect_pad_board_clearance(&pad, &board, &r(4), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.copper_gap, Some(rq(24, 5)));

    let violation =
        check_oriented_rect_pad_board_clearance(&pad, &board, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
    assert_eq!(violation.copper_gap, Some(rq(24, 5)));
}

#[test]
fn swept_segment_rejects_negative_width() {
    let error = SweptLineSegment::new(LinePathSegment::new(p(0, 0), p(1, 0)), r(-1))
        .expect_err("negative trace/cutter width must be rejected");
    assert_eq!(error, "swept path width must be nonnegative");
}

#[test]
fn axis_aligned_line_offset_preserves_exact_distance_and_provenance() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::GCode, 1_000).unwrap();
    let segment = LinePathSegment::with_provenance(p(0, 0), p(10, 0), provenance);

    let left =
        offset_axis_aligned_segment(&segment, r(3), OffsetSide::Left, PredicatePolicy::default())
            .unwrap();
    let right = offset_axis_aligned_segment(
        &segment,
        r(3),
        OffsetSide::Right,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(left.segment.start(), &p(0, 3));
    assert_eq!(left.segment.end(), &p(10, 3));
    assert_eq!(right.segment.start(), &p(0, -3));
    assert_eq!(right.segment.end(), &p(10, -3));
    assert_eq!(left.segment.provenance(), provenance);
    assert_eq!(left.distance, r(3));
}

#[test]
fn axis_aligned_line_offset_respects_reversed_and_vertical_direction() {
    let reversed = LinePathSegment::new(p(10, 0), p(0, 0));
    let vertical = LinePathSegment::new(p(0, 0), p(0, 10));

    let reversed_left = offset_axis_aligned_segment(
        &reversed,
        r(2),
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();
    let vertical_left = offset_axis_aligned_segment(
        &vertical,
        r(2),
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(reversed_left.segment.start(), &p(10, -2));
    assert_eq!(reversed_left.segment.end(), &p(0, -2));
    assert_eq!(vertical_left.segment.start(), &p(-2, 0));
    assert_eq!(vertical_left.segment.end(), &p(-2, 10));
}

#[test]
fn line_offset_rejects_invalid_candidates() {
    let diagonal = LinePathSegment::new(p(0, 0), p(1, 1));
    let degenerate = LinePathSegment::new(p(0, 0), p(0, 0));
    let horizontal = LinePathSegment::new(p(0, 0), p(1, 0));

    assert_eq!(
        offset_axis_aligned_segment(
            &diagonal,
            r(1),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        LineOffsetError::NotAxisAligned
    );
    assert_eq!(
        offset_axis_aligned_segment(
            &degenerate,
            r(1),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        LineOffsetError::UnknownDirection
    );
    assert_eq!(
        offset_axis_aligned_segment(
            &horizontal,
            r(-1),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        LineOffsetError::NegativeDistance
    );
}

#[test]
fn bezier_offset_samples_retain_exact_normal_facts() {
    let quadratic = QuadraticBezier::new(p(0, 0), p(5, 0), p(10, 0));
    let sample = offset_quadratic_bezier_sample(
        &quadratic,
        BezierParameter::new(1, 2).unwrap(),
        r(3),
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(sample.point, p(5, 0));
    assert_eq!(sample.tangent, p(10, 0));
    assert_eq!(sample.normal, p(0, 10));
    assert_eq!(sample.speed_squared, r(100));
    assert_eq!(sample.offset_distance_squared, r(9));
    assert_eq!(sample.offset_point, Some(p(5, 3)));

    let cubic = CubicBezier::new(p(0, 0), p(3, 0), p(6, 0), p(9, 0));
    let cubic_sample = offset_cubic_bezier_sample(
        &cubic,
        BezierParameter::new(1, 2).unwrap(),
        r(2),
        OffsetSide::Right,
        PredicatePolicy::default(),
    )
    .unwrap();
    let nine_halves = Real::new(Rational::fraction(9, 2).unwrap());
    assert_eq!(cubic_sample.point, Point2::new(nine_halves.clone(), r(0)));
    assert_eq!(cubic_sample.normal, p(0, -9));
    assert_eq!(
        cubic_sample.offset_point,
        Some(Point2::new(nine_halves, r(-2)))
    );
}

#[test]
fn bezier_offset_samples_reject_invalid_inputs() {
    let degenerate = QuadraticBezier::new(p(1, 1), p(1, 1), p(1, 1));
    assert_eq!(
        offset_quadratic_bezier_sample(
            &degenerate,
            BezierParameter::new(1, 2).unwrap(),
            r(1),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        BezierOffsetError::DegenerateTangent
    );

    let line = HigherOrderBezier::quartic(p(0, 0), p(1, 0), p(2, 0), p(3, 0), p(4, 0));
    assert_eq!(
        offset_higher_order_bezier_sample(
            &line,
            BezierParameter::new(1, 2).unwrap(),
            r(-1),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        BezierOffsetError::NegativeDistance
    );
}

#[test]
fn rectangular_pocket_plan_schedules_exact_inset_rings() {
    let pocket = RectangularPocket::new(p(0, 0), p(20, 12)).unwrap();
    let plan =
        build_rectangular_pocket_plan(pocket.clone(), r(2), r(3), 8, PredicatePolicy::default())
            .unwrap();

    assert_eq!(plan.pocket, pocket);
    assert_eq!(plan.rings.len(), 2);
    assert_eq!(plan.stop_reason, PocketPlanStopReason::GeometryExhausted);
    assert_eq!(plan.rings[0].index, 0);
    assert_eq!(plan.rings[0].inset, r(2));
    assert_eq!(plan.rings[0].min, p(2, 2));
    assert_eq!(plan.rings[0].max, p(18, 10));
    assert_eq!(plan.rings[1].index, 1);
    assert_eq!(plan.rings[1].inset, r(5));
    assert_eq!(plan.rings[1].min, p(5, 5));
    assert_eq!(plan.rings[1].max, p(15, 7));
    assert_eq!(plan.pocket.width(), r(20));
    assert_eq!(plan.pocket.height(), r(12));
    assert!(plan.pocket.exact_facts().all_exact_rational);
}

#[test]
fn rectangular_pocket_plan_rejects_invalid_inputs_and_respects_ring_limit() {
    assert_eq!(
        RectangularPocket::new(p(10, 0), p(0, 10)).unwrap_err(),
        PocketPlanError::UnorderedBounds
    );
    let pocket = RectangularPocket::new(p(0, 0), p(100, 100)).unwrap();
    assert_eq!(
        build_rectangular_pocket_plan(pocket.clone(), r(-1), r(1), 1, PredicatePolicy::default())
            .unwrap_err(),
        PocketPlanError::NegativeToolRadius
    );
    assert_eq!(
        build_rectangular_pocket_plan(pocket.clone(), r(1), r(0), 1, PredicatePolicy::default())
            .unwrap_err(),
        PocketPlanError::NonPositiveStepover
    );
    assert_eq!(
        build_rectangular_pocket_plan(pocket.clone(), r(1), r(1), 0, PredicatePolicy::default())
            .unwrap_err(),
        PocketPlanError::ZeroMaxRings
    );
    let limited =
        build_rectangular_pocket_plan(pocket, r(1), r(1), 2, PredicatePolicy::default()).unwrap();
    assert_eq!(limited.rings.len(), 2);
    assert_eq!(limited.stop_reason, PocketPlanStopReason::MaxRingsReached);
}

#[test]
fn rectangular_bead_plan_schedules_exact_centerlines() {
    let region = RectangularPocket::new(p(0, 0), p(10, 6)).unwrap();
    let plan = build_rectangular_bead_plan(
        region.clone(),
        BeadFillAxis::Horizontal,
        r(2),
        r(2),
        8,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(plan.region, region);
    assert_eq!(plan.beads.len(), 3);
    assert_eq!(plan.stop_reason, PocketPlanStopReason::GeometryExhausted);
    assert_eq!(plan.beads[0].index, 0);
    assert_eq!(plan.beads[0].pitch_position, r(1));
    assert_eq!(plan.beads[0].segment.start(), &p(0, 1));
    assert_eq!(plan.beads[0].segment.end(), &p(10, 1));
    assert_eq!(plan.beads[1].pitch_position, r(3));
    assert_eq!(plan.beads[2].pitch_position, r(5));

    let vertical = build_rectangular_bead_plan(
        RectangularPocket::new(p(0, 0), p(10, 6)).unwrap(),
        BeadFillAxis::Vertical,
        r(2),
        r(4),
        8,
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(vertical.beads.len(), 3);
    assert_eq!(vertical.beads[0].segment.start(), &p(1, 0));
    assert_eq!(vertical.beads[0].segment.end(), &p(1, 6));
}

#[test]
fn rectangular_bead_plan_rejects_invalid_inputs_and_respects_limit() {
    let region = RectangularPocket::new(p(0, 0), p(10, 10)).unwrap();
    assert_eq!(
        build_rectangular_bead_plan(
            region.clone(),
            BeadFillAxis::Horizontal,
            r(0),
            r(1),
            1,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        BeadPlanError::NonPositiveBeadWidth
    );
    assert_eq!(
        build_rectangular_bead_plan(
            region.clone(),
            BeadFillAxis::Horizontal,
            r(1),
            r(0),
            1,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        BeadPlanError::NonPositiveSpacing
    );
    assert_eq!(
        build_rectangular_bead_plan(
            region.clone(),
            BeadFillAxis::Horizontal,
            r(1),
            r(1),
            0,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        BeadPlanError::ZeroMaxBeads
    );
    let limited = build_rectangular_bead_plan(
        region,
        BeadFillAxis::Horizontal,
        r(2),
        r(1),
        2,
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(limited.beads.len(), 2);
    assert_eq!(limited.stop_reason, PocketPlanStopReason::MaxRingsReached);
}

#[test]
fn rectangular_serpentine_infill_graph_links_exact_bead_endpoints() {
    let plan = build_rectangular_bead_plan(
        RectangularPocket::new(p(0, 0), p(10, 6)).unwrap(),
        BeadFillAxis::Horizontal,
        r(2),
        r(2),
        8,
        PredicatePolicy::default(),
    )
    .unwrap();
    let graph = build_rectangular_serpentine_infill_graph(plan.clone(), PredicatePolicy::default())
        .unwrap();

    assert_eq!(graph.plan, plan);
    assert_eq!(graph.deposition_segments.len(), 3);
    assert_eq!(graph.links.len(), 2);
    assert_eq!(graph.deposition_segments[0].start(), &p(0, 1));
    assert_eq!(graph.deposition_segments[0].end(), &p(10, 1));
    assert_eq!(graph.deposition_segments[1].start(), &p(10, 3));
    assert_eq!(graph.deposition_segments[1].end(), &p(0, 3));
    assert_eq!(graph.deposition_segments[2].start(), &p(0, 5));
    assert_eq!(graph.deposition_segments[2].end(), &p(10, 5));
    assert_eq!(graph.links[0].from_bead, 0);
    assert_eq!(graph.links[0].to_bead, 1);
    assert_eq!(graph.links[0].connector.start(), &p(10, 1));
    assert_eq!(graph.links[0].connector.end(), &p(10, 3));
    assert_eq!(graph.links[1].connector.start(), &p(0, 3));
    assert_eq!(graph.links[1].connector.end(), &p(0, 5));
}

#[test]
fn rectangular_serpentine_infill_graph_rejects_empty_bead_plans() {
    let plan = build_rectangular_bead_plan(
        RectangularPocket::new(p(0, 0), p(10, 1)).unwrap(),
        BeadFillAxis::Horizontal,
        r(2),
        r(2),
        8,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert!(plan.beads.is_empty());
    assert_eq!(
        build_rectangular_serpentine_infill_graph(plan, PredicatePolicy::default()).unwrap_err(),
        InfillGraphError::EmptyBeadPlan
    );
}

#[test]
fn rectangular_support_plan_expands_and_classifies_exact_footprints() {
    let overhang = RectangularPocket::new(p(4, 4), p(6, 6)).unwrap();
    let base = RectangularPocket::new(p(0, 0), p(10, 10)).unwrap();
    let plan = build_rectangular_support_plan(
        overhang.clone(),
        base.clone(),
        r(1),
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(plan.overhang, overhang);
    assert_eq!(plan.base, base);
    assert_eq!(plan.xy_margin, r(1));
    assert_eq!(plan.footprint.min(), &p(3, 3));
    assert_eq!(plan.footprint.max(), &p(7, 7));
    assert_eq!(plan.status, SupportFootprintStatus::ContainedInBase);

    let outside = build_rectangular_support_plan(
        RectangularPocket::new(p(1, 1), p(3, 3)).unwrap(),
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        r(2),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(outside.footprint.min(), &p(-1, -1));
    assert_eq!(outside.status, SupportFootprintStatus::OutsideBase);
}

#[test]
fn rectangular_support_plan_rejects_negative_margin() {
    assert_eq!(
        build_rectangular_support_plan(
            RectangularPocket::new(p(4, 4), p(6, 6)).unwrap(),
            RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
            r(-1),
            PredicatePolicy::default()
        )
        .unwrap_err(),
        SupportPlanError::NegativeMargin
    );
}

#[test]
fn rectangular_region_intersection_classifies_disjoint_touching_and_overlap() {
    let overlap = intersect_rectangular_regions(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(4, 3), p(12, 8)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(overlap.relation, RectangularRegionRelation::AreaOverlap);
    assert_eq!(overlap.intersection.as_ref().unwrap().min(), &p(4, 3));
    assert_eq!(overlap.intersection.as_ref().unwrap().max(), &p(10, 8));

    let touching = intersect_rectangular_regions(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(10, 2), p(12, 8)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(touching.relation, RectangularRegionRelation::Touching);
    assert_eq!(touching.intersection.as_ref().unwrap().min(), &p(10, 2));
    assert_eq!(touching.intersection.as_ref().unwrap().max(), &p(10, 8));

    let disjoint = intersect_rectangular_regions(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(11, 2), p(12, 8)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(disjoint.relation, RectangularRegionRelation::Disjoint);
    assert!(disjoint.intersection.is_none());
}

#[test]
fn rectangular_region_difference_emits_positive_area_remainder_pieces() {
    let difference = subtract_rectangular_region(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(3, 4), p(7, 8)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(difference.relation, RectangularRegionRelation::AreaOverlap);
    assert_eq!(difference.intersection.as_ref().unwrap().min(), &p(3, 4));
    assert_eq!(difference.intersection.as_ref().unwrap().max(), &p(7, 8));
    assert_eq!(difference.remainder.len(), 4);
    assert_eq!(difference.remainder[0].min(), &p(0, 0));
    assert_eq!(difference.remainder[0].max(), &p(3, 10));
    assert_eq!(difference.remainder[1].min(), &p(7, 0));
    assert_eq!(difference.remainder[1].max(), &p(10, 10));
    assert_eq!(difference.remainder[2].min(), &p(3, 0));
    assert_eq!(difference.remainder[2].max(), &p(7, 4));
    assert_eq!(difference.remainder[3].min(), &p(3, 8));
    assert_eq!(difference.remainder[3].max(), &p(7, 10));

    let covered = subtract_rectangular_region(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(-1, -1), p(11, 11)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(covered.relation, RectangularRegionRelation::AreaOverlap);
    assert!(covered.remainder.is_empty());

    let touching = subtract_rectangular_region(
        RectangularPocket::new(p(0, 0), p(10, 10)).unwrap(),
        RectangularPocket::new(p(10, 0), p(12, 10)).unwrap(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(touching.relation, RectangularRegionRelation::Touching);
    assert_eq!(touching.remainder.len(), 1);
    assert_eq!(touching.remainder[0].min(), &p(0, 0));
    assert_eq!(touching.remainder[0].max(), &p(10, 10));
}

#[test]
fn length_match_problem_certifies_exact_extension_candidate() {
    let model = build_length_match_problem(r(100), r(125), r(25));
    let report = certify_length_extension(&model);

    assert!(report.all_satisfied());
    assert_eq!(model.extra_length_symbol.0, 0);
}

#[test]
fn length_match_problem_reports_wrong_extension_as_violation() {
    let model = build_length_match_problem(r(100), r(125), r(20));
    let report = certify_length_extension(&model);

    assert!(report.has_certified_violation());
}

#[test]
fn differential_pair_skew_replays_exact_axis_lengths() {
    let first = vec![
        LinePathSegment::new(p(0, 0), p(10, 0)),
        LinePathSegment::new(p(10, 0), p(10, 5)),
    ];
    let second = vec![LinePathSegment::new(p(0, 2), p(12, 2))];
    let report =
        certify_differential_pair_skew(&first, &second, r(3), PredicatePolicy::default()).unwrap();

    assert_eq!(report.first_length, r(15));
    assert_eq!(report.second_length, r(12));
    assert_eq!(report.actual_skew, r(3));
    assert_eq!(report.target_skew, r(3));
    assert!(report.certification.all_satisfied());

    let wrong =
        certify_differential_pair_skew(&first, &second, r(4), PredicatePolicy::default()).unwrap();
    assert!(wrong.certification.has_certified_violation());
}

#[test]
fn differential_pair_skew_rejects_empty_and_unsupported_routes() {
    let axis = vec![LinePathSegment::new(p(0, 0), p(10, 0))];
    let diagonal = vec![LinePathSegment::new(p(0, 0), p(3, 4))];

    assert_eq!(
        certify_differential_pair_skew(&[], &axis, Real::zero(), PredicatePolicy::default())
            .unwrap_err(),
        RouteCertificationError::EmptyRoute
    );
    assert_eq!(
        certify_differential_pair_skew(&axis, &diagonal, Real::zero(), PredicatePolicy::default())
            .unwrap_err(),
        RouteCertificationError::UnsupportedRouteGeometry
    );
}

#[test]
fn constant_feed_time_replays_exact_axis_length() {
    let route = vec![
        LinePathSegment::new(p(0, 0), p(10, 0)),
        LinePathSegment::new(p(10, 0), p(10, 5)),
    ];
    let report =
        certify_constant_feed_time(&route, r(5), r(3), PredicatePolicy::default()).unwrap();

    assert_eq!(report.path_length, r(15));
    assert_eq!(report.feed_rate, r(5));
    assert_eq!(report.target_time, r(3));
    assert!(report.certification.all_satisfied());

    let wrong = certify_constant_feed_time(&route, r(5), r(4), PredicatePolicy::default()).unwrap();
    assert!(wrong.certification.has_certified_violation());
}

#[test]
fn constant_feed_time_rejects_invalid_inputs() {
    let route = vec![LinePathSegment::new(p(0, 0), p(10, 0))];
    let diagonal = vec![LinePathSegment::new(p(0, 0), p(3, 4))];

    assert_eq!(
        certify_constant_feed_time(&[], r(1), r(1), PredicatePolicy::default()).unwrap_err(),
        RouteCertificationError::EmptyRoute
    );
    assert_eq!(
        certify_constant_feed_time(&route, r(-1), r(1), PredicatePolicy::default()).unwrap_err(),
        RouteCertificationError::NegativeFeedRate
    );
    assert_eq!(
        certify_constant_feed_time(&route, Real::zero(), r(1), PredicatePolicy::default())
            .unwrap_err(),
        RouteCertificationError::ZeroFeedRate
    );
    assert_eq!(
        certify_constant_feed_time(&route, r(1), r(-1), PredicatePolicy::default()).unwrap_err(),
        RouteCertificationError::NegativeTime
    );
    assert_eq!(
        certify_constant_feed_time(&diagonal, r(1), r(5), PredicatePolicy::default()).unwrap_err(),
        RouteCertificationError::UnsupportedRouteGeometry
    );
}

#[test]
fn single_detour_meander_adds_exact_length_and_certifies_target() {
    let source = LinePathSegment::new(p(0, 0), p(10, 0));
    let meander =
        build_single_detour_meander(&source, r(6), OffsetSide::Left, PredicatePolicy::default())
            .unwrap();

    assert_eq!(meander.amplitude, r(3));
    assert_eq!(meander.segments.len(), 3);
    assert_eq!(meander.segments[0].start(), &p(0, 0));
    assert_eq!(meander.segments[0].end(), &p(0, 3));
    assert_eq!(meander.segments[1].start(), &p(0, 3));
    assert_eq!(meander.segments[1].end(), &p(10, 3));
    assert_eq!(meander.segments[2].start(), &p(10, 3));
    assert_eq!(meander.segments[2].end(), &p(10, 0));
    assert_eq!(
        meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(16)
    );
    assert!(
        meander
            .certify_target_length(r(16), PredicatePolicy::default())
            .unwrap()
            .all_satisfied()
    );
}

#[test]
fn single_detour_meander_handles_zero_and_rejects_invalid_inputs() {
    let source = LinePathSegment::new(p(0, 0), p(10, 0));
    let diagonal = LinePathSegment::new(p(0, 0), p(1, 1));

    let zero =
        build_single_detour_meander(&source, r(0), OffsetSide::Left, PredicatePolicy::default())
            .unwrap();
    assert_eq!(zero.segments, vec![source.clone()]);
    assert_eq!(
        zero.exact_axis_length(PredicatePolicy::default()).unwrap(),
        r(10)
    );
    assert_eq!(
        build_single_detour_meander(&source, r(-1), OffsetSide::Left, PredicatePolicy::default())
            .unwrap_err(),
        MeanderError::NegativeExtraLength
    );
    assert_eq!(
        build_single_detour_meander(
            &diagonal,
            r(2),
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::UnsupportedSourceGeometry
    );
}

#[test]
fn multi_detour_meander_splits_source_and_certifies_exact_length() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let meander = build_multi_detour_meander(
        &source,
        r(12),
        3,
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(meander.bump_count, 3);
    assert_eq!(meander.amplitude, r(2));
    assert_eq!(meander.segments.len(), 9);
    assert_eq!(meander.segments[0].start(), &p(0, 0));
    assert_eq!(meander.segments[0].end(), &p(0, 2));
    assert_eq!(meander.segments[1].start(), &p(0, 2));
    assert_eq!(meander.segments[1].end(), &p(4, 2));
    assert_eq!(meander.segments[2].end(), &p(4, 0));
    assert_eq!(meander.segments[7].start(), &p(8, 2));
    assert_eq!(meander.segments[7].end(), &p(12, 2));
    assert_eq!(
        meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(24)
    );
    assert!(
        meander
            .certify_target_length(r(24), PredicatePolicy::default())
            .unwrap()
            .all_satisfied()
    );
}

#[test]
fn multi_detour_meander_handles_vertical_reversed_and_rejects_bad_bumps() {
    let vertical = LinePathSegment::new(p(0, 10), p(0, 0));
    let meander = build_multi_detour_meander(
        &vertical,
        r(8),
        2,
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(meander.amplitude, r(2));
    assert_eq!(meander.segments.len(), 6);
    assert_eq!(meander.segments[0].start(), &p(0, 10));
    assert_eq!(meander.segments[0].end(), &p(2, 10));
    assert_eq!(meander.segments[1].end(), &p(2, 5));
    assert_eq!(meander.segments[5].end(), &p(0, 0));
    assert_eq!(
        meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(18)
    );
    assert_eq!(
        build_multi_detour_meander(
            &vertical,
            r(8),
            0,
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::ZeroBumps
    );
    assert_eq!(
        build_multi_detour_meander(
            &vertical,
            r(-1),
            2,
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::NegativeExtraLength
    );
}

#[test]
fn alternating_detour_meander_flips_sides_and_certifies_length() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let meander = build_alternating_detour_meander(
        &source,
        r(12),
        3,
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(meander.bump_count, 3);
    assert_eq!(meander.amplitude, r(2));
    assert_eq!(meander.segments.len(), 9);
    assert_eq!(meander.segments[0].end(), &p(0, 2));
    assert_eq!(meander.segments[1].end(), &p(4, 2));
    assert_eq!(meander.segments[3].end(), &p(4, -2));
    assert_eq!(meander.segments[4].end(), &p(8, -2));
    assert_eq!(meander.segments[6].end(), &p(8, 2));
    assert_eq!(meander.segments[7].end(), &p(12, 2));
    assert_eq!(
        meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(24)
    );
    assert!(
        meander
            .certify_target_length(r(24), PredicatePolicy::default())
            .unwrap()
            .all_satisfied()
    );
}

#[test]
fn nonuniform_detour_meander_retains_amplitudes_and_certifies_length() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let meander = build_nonuniform_detour_meander(
        &source,
        vec![r(1), r(3), r(2)],
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(meander.amplitudes, vec![r(1), r(3), r(2)]);
    assert_eq!(meander.extra_length, r(12));
    assert_eq!(meander.segments.len(), 9);
    assert_eq!(meander.segments[0].end(), &p(0, 1));
    assert_eq!(meander.segments[1].end(), &p(4, 1));
    assert_eq!(meander.segments[3].end(), &p(4, 3));
    assert_eq!(meander.segments[4].end(), &p(8, 3));
    assert_eq!(meander.segments[6].end(), &p(8, 2));
    assert_eq!(meander.segments[7].end(), &p(12, 2));
    assert_eq!(
        meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(24)
    );
    assert!(
        meander
            .certify_target_length(r(24), PredicatePolicy::default())
            .unwrap()
            .all_satisfied()
    );
}

#[test]
fn nonuniform_detour_meander_rejects_empty_and_negative_amplitudes() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));

    assert_eq!(
        build_nonuniform_detour_meander(
            &source,
            vec![],
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::ZeroBumps
    );
    assert_eq!(
        build_nonuniform_detour_meander(
            &source,
            vec![r(1), r(-1)],
            OffsetSide::Left,
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::NegativeAmplitude
    );
    let zero = build_nonuniform_detour_meander(
        &source,
        vec![r(0), r(0)],
        OffsetSide::Left,
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(zero.segments, vec![source]);
    assert_eq!(zero.extra_length, Real::zero());
}

#[test]
fn obstacle_aware_detour_meander_selects_clear_side_and_certifies_length() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let obstacle = MeanderObstacle {
        min: p(-1, 1),
        max: p(2, 3),
    };
    let routed = build_obstacle_aware_detour_meander(
        &source,
        r(12),
        3,
        OffsetSide::Left,
        vec![obstacle.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(
        routed.selected_sides,
        vec![OffsetSide::Right, OffsetSide::Left, OffsetSide::Left]
    );
    assert_eq!(routed.obstacles, vec![obstacle]);
    assert_eq!(routed.meander.bump_count, 3);
    assert_eq!(routed.meander.amplitude, r(2));
    assert_eq!(routed.meander.segments[0].end(), &p(0, -2));
    assert_eq!(routed.meander.segments[1].end(), &p(4, -2));
    assert_eq!(routed.meander.segments[3].end(), &p(4, 2));
    assert_eq!(
        routed
            .meander
            .exact_axis_length(PredicatePolicy::default())
            .unwrap(),
        r(24)
    );
    assert!(
        routed
            .meander
            .certify_target_length(r(24), PredicatePolicy::default())
            .unwrap()
            .all_satisfied()
    );
}

#[test]
fn meander_placement_slots_report_exact_side_blockage() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let obstacle = MeanderObstacle {
        min: p(-1, 1),
        max: p(3, 3),
    };
    let report = classify_meander_placement_slots(
        &source,
        r(2),
        3,
        OffsetSide::Left,
        vec![obstacle.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.source, source);
    assert_eq!(report.amplitude, r(2));
    assert_eq!(report.obstacles, vec![obstacle]);
    assert_eq!(report.slots.len(), 3);
    assert_eq!(report.slots[0].index, 0);
    assert_eq!(report.slots[0].base.start(), &p(0, 0));
    assert_eq!(report.slots[0].base.end(), &p(4, 0));
    assert_eq!(report.slots[0].amplitude, r(2));
    assert!(report.slots[0].preferred_blocked);
    assert!(!report.slots[0].opposite_blocked);
    assert_eq!(report.slots[0].selected_side, Some(OffsetSide::Right));
    assert_eq!(report.slots[1].selected_side, Some(OffsetSide::Left));
    assert_eq!(report.slots[2].selected_side, Some(OffsetSide::Left));
}

#[test]
fn meander_candidate_slots_accept_arbitrary_windows_and_amplitudes() {
    let first = MeanderPlacementCandidate {
        base: LinePathSegment::new(p(0, 0), p(3, 0)),
        amplitude: r(1),
    };
    let second = MeanderPlacementCandidate {
        base: LinePathSegment::new(p(5, 0), p(11, 0)),
        amplitude: r(3),
    };
    let obstacle = MeanderObstacle {
        min: p(4, 2),
        max: p(12, 4),
    };
    let report = classify_meander_candidate_slots(
        vec![first.clone(), second.clone()],
        OffsetSide::Left,
        vec![obstacle.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.obstacles, vec![obstacle]);
    assert_eq!(report.slots.len(), 2);
    assert_eq!(report.slots[0].base, first.base);
    assert_eq!(report.slots[0].amplitude, r(1));
    assert_eq!(report.slots[0].selected_side, Some(OffsetSide::Left));
    assert_eq!(report.slots[1].base, second.base);
    assert_eq!(report.slots[1].amplitude, r(3));
    assert!(report.slots[1].preferred_blocked);
    assert!(!report.slots[1].opposite_blocked);
    assert_eq!(report.slots[1].selected_side, Some(OffsetSide::Right));
}

#[test]
fn meander_candidate_slots_reject_invalid_candidate_geometry() {
    let diagonal = MeanderPlacementCandidate {
        base: LinePathSegment::new(p(0, 0), p(3, 2)),
        amplitude: r(1),
    };
    let negative = MeanderPlacementCandidate {
        base: LinePathSegment::new(p(0, 0), p(3, 0)),
        amplitude: r(-1),
    };

    assert_eq!(
        classify_meander_candidate_slots(
            vec![diagonal],
            OffsetSide::Left,
            Vec::new(),
            PredicatePolicy::default(),
        )
        .unwrap_err(),
        MeanderError::UnsupportedSourceGeometry
    );
    assert_eq!(
        classify_meander_candidate_slots(
            vec![negative],
            OffsetSide::Left,
            Vec::new(),
            PredicatePolicy::default(),
        )
        .unwrap_err(),
        MeanderError::NegativeAmplitude
    );
    assert_eq!(
        classify_meander_candidate_slots(
            Vec::new(),
            OffsetSide::Left,
            Vec::new(),
            PredicatePolicy::default(),
        )
        .unwrap_err(),
        MeanderError::ZeroBumps
    );
}

#[test]
fn meander_placement_slots_report_conflicted_windows_without_committing_route() {
    let source = LinePathSegment::new(p(0, 0), p(4, 0));
    let above = MeanderObstacle {
        min: p(-1, 1),
        max: p(5, 3),
    };
    let below = MeanderObstacle {
        min: p(-1, -3),
        max: p(5, -1),
    };
    let report = classify_meander_placement_slots(
        &source,
        r(2),
        1,
        OffsetSide::Left,
        vec![above, below],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.slots.len(), 1);
    assert!(report.slots[0].preferred_blocked);
    assert!(report.slots[0].opposite_blocked);
    assert_eq!(report.slots[0].selected_side, None);
}

#[test]
fn obstacle_aware_detour_meander_rejects_blocked_or_invalid_keepouts() {
    let source = LinePathSegment::new(p(0, 0), p(12, 0));
    let above = MeanderObstacle {
        min: p(-1, 1),
        max: p(5, 3),
    };
    let below = MeanderObstacle {
        min: p(-1, -3),
        max: p(5, -1),
    };
    assert_eq!(
        build_obstacle_aware_detour_meander(
            &source,
            r(12),
            3,
            OffsetSide::Left,
            vec![above, below],
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::ObstacleConflict
    );

    let invalid = MeanderObstacle {
        min: p(2, 0),
        max: p(1, 1),
    };
    assert_eq!(
        build_obstacle_aware_detour_meander(
            &source,
            r(12),
            3,
            OffsetSide::Left,
            vec![invalid],
            PredicatePolicy::default()
        )
        .unwrap_err(),
        MeanderError::InvalidObstacleBounds
    );
}

#[test]
fn specctra_grid_trace_import_preserves_exact_source_grid() {
    let record = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: NetId(7),
        layer: TraceLayer(3),
        start_x: 0,
        start_y: 10,
        end_x: 50,
        end_y: 10,
        width: 6,
        grid_denominator: 10,
    })
    .unwrap();
    let trace = import_specctra_trace_record(&record).unwrap();
    let exported = export_specctra_trace_record(&trace);

    assert_eq!(trace.net(), NetId(7));
    assert_eq!(trace.layer(), TraceLayer(3));
    assert_eq!(
        trace.provenance(),
        PathProvenance::fixed_grid(PathSourceFormat::Specctra, 10).unwrap()
    );
    assert_eq!(exported, record);
    assert_eq!(trace.swept().centerline().start(), &Point2::new(r(0), r(1)));
    assert_eq!(trace.swept().centerline().end(), &Point2::new(r(5), r(1)));
    assert_eq!(
        trace.swept().width(),
        &Real::new(Rational::fraction(3, 5).unwrap())
    );
}

#[test]
fn specctra_route_import_rejects_invalid_grid_and_negative_width() {
    let invalid_grid = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: NetId(1),
        layer: TraceLayer(0),
        start_x: 0,
        start_y: 0,
        end_x: 1,
        end_y: 0,
        width: 1,
        grid_denominator: 0,
    })
    .expect_err("zero source grid must be rejected");
    let negative_width = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: NetId(1),
        layer: TraceLayer(0),
        start_x: 0,
        start_y: 0,
        end_x: 1,
        end_y: 0,
        width: -1,
        grid_denominator: 1,
    })
    .and_then(|record| import_specctra_trace_record(&record))
    .expect_err("negative trace width must be rejected");

    assert_eq!(invalid_grid, SpecctraImportError::InvalidGrid);
    assert_eq!(negative_width, SpecctraImportError::NegativeWidth);
}

#[test]
fn specctra_grid_via_import_preserves_exact_source_grid() {
    let record = specctra_grid_via_record(SpecctraGridViaRecord {
        net: NetId(7),
        start_layer: TraceLayer(1),
        end_layer: TraceLayer(4),
        x: 25,
        y: -50,
        land_diameter: 12,
        drill_diameter: 6,
        drill_intent: ViaDrillIntent::NonPlated,
        grid_denominator: 10,
    })
    .unwrap();
    let via = import_specctra_via_record(&record).unwrap();

    assert_eq!(via.net(), NetId(7));
    assert_eq!(via.start_layer(), TraceLayer(1));
    assert_eq!(via.end_layer(), TraceLayer(4));
    assert_eq!(
        via.center(),
        &Point2::new(r(2) + Real::new(Rational::fraction(1, 2).unwrap()), r(-5))
    );
    assert_eq!(
        via.land_diameter(),
        &Real::new(Rational::fraction(6, 5).unwrap())
    );
    assert_eq!(
        via.drill_diameter().unwrap(),
        &Real::new(Rational::fraction(3, 5).unwrap())
    );
    assert_eq!(record.drill_intent, ViaDrillIntent::NonPlated);
    assert_eq!(via.drill_intent(), ViaDrillIntent::NonPlated);
}

#[test]
fn specctra_grid_via_import_rejects_invalid_geometry() {
    let reversed = specctra_grid_via_record(SpecctraGridViaRecord {
        net: NetId(1),
        start_layer: TraceLayer(3),
        end_layer: TraceLayer(1),
        x: 0,
        y: 0,
        land_diameter: 10,
        drill_diameter: 4,
        drill_intent: ViaDrillIntent::Plated,
        grid_denominator: 1,
    })
    .and_then(|record| import_specctra_via_record(&record))
    .unwrap_err();
    let negative_drill = specctra_grid_via_record(SpecctraGridViaRecord {
        net: NetId(1),
        start_layer: TraceLayer(0),
        end_layer: TraceLayer(1),
        x: 0,
        y: 0,
        land_diameter: 10,
        drill_diameter: -4,
        drill_intent: ViaDrillIntent::Plated,
        grid_denominator: 1,
    })
    .and_then(|record| import_specctra_via_record(&record))
    .unwrap_err();

    assert_eq!(reversed, SpecctraImportError::ReversedLayerSpan);
    assert_eq!(negative_drill, SpecctraImportError::NegativeDiameter);
}

#[test]
fn specctra_grid_route_text_round_trips_canonical_records() {
    let records = vec![
        SpecctraGridTraceRecord {
            net: NetId(7),
            layer: TraceLayer(3),
            start_x: 0,
            start_y: 10,
            end_x: 50,
            end_y: 10,
            width: 6,
            grid_denominator: 10,
        },
        SpecctraGridTraceRecord {
            net: NetId(8),
            layer: TraceLayer(4),
            start_x: -2,
            start_y: 3,
            end_x: 9,
            end_y: -11,
            width: 1,
            grid_denominator: 100,
        },
    ];

    let text = serialize_specctra_grid_trace_records(&records);
    let parsed = parse_specctra_grid_trace_records(&text).unwrap();
    let route = import_specctra_text_route(&text).unwrap();

    assert_eq!(parsed, records);
    assert_eq!(route.traces().len(), 2);
    assert_eq!(route.traces()[0].net(), NetId(7));
    assert_eq!(route.traces()[1].layer(), TraceLayer(4));
}

#[test]
fn specctra_grid_route_text_round_trips_vias_and_wires() {
    let wire = SpecctraGridTraceRecord {
        net: NetId(7),
        layer: TraceLayer(3),
        start_x: 0,
        start_y: 10,
        end_x: 50,
        end_y: 10,
        width: 6,
        grid_denominator: 10,
    };
    let via = SpecctraGridViaRecord {
        net: NetId(7),
        start_layer: TraceLayer(0),
        end_layer: TraceLayer(3),
        x: 50,
        y: 10,
        land_diameter: 12,
        drill_diameter: 6,
        drill_intent: ViaDrillIntent::Plated,
        grid_denominator: 10,
    };
    let text = serialize_specctra_grid_route_records(&hyperpath::SpecctraGridRouteRecords {
        net_aliases: Vec::new(),
        layer_aliases: Vec::new(),
        traces: vec![wire],
        vias: vec![via],
    });
    let parsed = parse_specctra_grid_route_records(&text).unwrap();
    let route = import_specctra_text_route(&text).unwrap();

    assert_eq!(parsed.traces, vec![wire]);
    assert_eq!(parsed.vias, vec![via]);
    assert_eq!(route.traces().len(), 1);
    assert_eq!(route.vias().len(), 1);
    assert_eq!(route.vias()[0].start_layer(), TraceLayer(0));
    assert_eq!(route.vias()[0].end_layer(), TraceLayer(3));
    assert_eq!(route.vias()[0].drill_intent(), ViaDrillIntent::Plated);
}

#[test]
fn specctra_grid_route_text_round_trips_net_aliases() {
    let alias = SpecctraNetAlias {
        net: NetId(7),
        name: "USB_DP".to_owned(),
    };
    let layer_alias = SpecctraLayerAlias {
        layer: TraceLayer(3),
        name: "F_Cu".to_owned(),
    };
    let wire = SpecctraGridTraceRecord {
        net: NetId(7),
        layer: TraceLayer(3),
        start_x: 0,
        start_y: 10,
        end_x: 50,
        end_y: 10,
        width: 6,
        grid_denominator: 10,
    };
    let text = serialize_specctra_grid_route_records(&hyperpath::SpecctraGridRouteRecords {
        net_aliases: vec![alias.clone()],
        layer_aliases: vec![layer_alias.clone()],
        traces: vec![wire],
        vias: Vec::new(),
    });
    let parsed = parse_specctra_grid_route_records(&text).unwrap();
    let route = import_specctra_text_route(&text).unwrap();

    assert_eq!(parsed.net_aliases, vec![alias]);
    assert_eq!(parsed.layer_aliases, vec![layer_alias]);
    assert_eq!(parsed.traces, vec![wire]);
    assert_eq!(route.traces()[0].net(), NetId(7));
}

#[test]
fn specctra_grid_route_text_parses_quoted_aliases_comments_and_envelopes() {
    let input = r#"
        (session "demo board"
          ; unrelated DSN/SES metadata must not affect retained route objects
          (placement (component U1 (at 10 20)))
          (autoroute
            (routes
              (net 7 "USB DP")
              (layer 3 "F.Cu signal")
              (wire (net 7) (layer 3) (start 0 10) (end 50 10) (width 6) (grid 10))
              (via (net 7) (layers 0 3) (at 50 10) (land 12) (drill 6)
                   (intent nonplated) (grid 10)))))"#;

    let parsed = parse_specctra_grid_route_records(input).unwrap();
    let route = import_specctra_text_route(input).unwrap();
    let serialized = serialize_specctra_grid_route_records(&parsed);
    let reparsed = parse_specctra_grid_route_records(&serialized).unwrap();

    assert_eq!(
        parsed.net_aliases,
        vec![SpecctraNetAlias {
            net: NetId(7),
            name: "USB DP".to_owned()
        }]
    );
    assert_eq!(
        parsed.layer_aliases,
        vec![SpecctraLayerAlias {
            layer: TraceLayer(3),
            name: "F.Cu signal".to_owned()
        }]
    );
    assert_eq!(parsed.traces.len(), 1);
    assert_eq!(parsed.vias.len(), 1);
    assert_eq!(parsed.vias[0].drill_intent, ViaDrillIntent::NonPlated);
    assert_eq!(route.traces().len(), 1);
    assert_eq!(route.vias().len(), 1);
    assert_eq!(reparsed, parsed);
}

#[test]
fn specctra_path_wire_lowers_to_exact_consecutive_segments() {
    let text = "(pcb \"board\" (library ignored) (routes \
        (wire (net 3) (path 2 8 0 0 10 0 10 10 -5 10) (grid 4))))";
    let parsed = parse_specctra_grid_route_records(text).unwrap();
    let route = import_specctra_text_route(text).unwrap();

    assert_eq!(
        parsed.traces,
        vec![
            SpecctraGridTraceRecord {
                net: NetId(3),
                layer: TraceLayer(2),
                start_x: 0,
                start_y: 0,
                end_x: 10,
                end_y: 0,
                width: 8,
                grid_denominator: 4,
            },
            SpecctraGridTraceRecord {
                net: NetId(3),
                layer: TraceLayer(2),
                start_x: 10,
                start_y: 0,
                end_x: 10,
                end_y: 10,
                width: 8,
                grid_denominator: 4,
            },
            SpecctraGridTraceRecord {
                net: NetId(3),
                layer: TraceLayer(2),
                start_x: 10,
                start_y: 10,
                end_x: -5,
                end_y: 10,
                width: 8,
                grid_denominator: 4,
            },
        ]
    );
    assert_eq!(route.traces().len(), 3);
    assert_eq!(route.traces()[0].swept().width(), &r(2));
}

#[test]
fn specctra_grid_route_text_rejects_malformed_and_invalid_routes() {
    assert_eq!(
        parse_specctra_grid_trace_records("(routes (wire (net 1)))").unwrap_err(),
        SpecctraParseError::InvalidSyntax
    );
    assert_eq!(
        parse_specctra_grid_trace_records(
            "(routes (wire (net 1) (layer 0) (start 0 0) (end 1 0) (width 1) (grid 0)))"
        )
        .unwrap_err(),
        SpecctraParseError::InvalidGrid
    );
    assert_eq!(
        import_specctra_text_route(
            "(routes (wire (net 1) (layer 0) (start 0 0) (end 1 0) (width -1) (grid 1)))"
        )
        .unwrap_err(),
        SpecctraParseError::NegativeWidth
    );
    assert_eq!(
        import_specctra_text_route(
            "(routes (via (net 1) (layers 2 0) (at 0 0) (land 10) (drill 4) (grid 1)))"
        )
        .unwrap_err(),
        SpecctraParseError::ReversedLayerSpan
    );
    assert_eq!(
        import_specctra_text_route(
            "(routes (via (net 1) (layers 0 1) (at 0 0) (land 10) (drill -4) (grid 1)))"
        )
        .unwrap_err(),
        SpecctraParseError::NegativeDiameter
    );
    assert_eq!(
        parse_specctra_grid_route_records(
            "(routes (via (net 1) (layers 0 1) (at 0 0) (land 10) (drill 4) (grid 1)))"
        )
        .unwrap()
        .vias[0]
            .drill_intent,
        ViaDrillIntent::Unspecified
    );
    assert_eq!(
        parse_specctra_grid_route_records(
            "(routes (via (net 1) (layers 0 1) (at 0 0) (land 10) (drill 4) (intent mystery) (grid 1)))"
        )
        .unwrap_err(),
        SpecctraParseError::InvalidDrillIntent
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (net 1 A) (net 1 B))").unwrap_err(),
        SpecctraParseError::InvalidNetAlias
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (net 1 A) (net 2 A))").unwrap_err(),
        SpecctraParseError::InvalidNetAlias
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (layer 1 F_Cu) (layer 1 B_Cu))").unwrap_err(),
        SpecctraParseError::InvalidLayerAlias
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (layer 1 F_Cu) (layer 2 F_Cu))").unwrap_err(),
        SpecctraParseError::InvalidLayerAlias
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (net 1 \"unterminated))").unwrap_err(),
        SpecctraParseError::InvalidSyntax
    );
    assert_eq!(
        parse_specctra_grid_route_records("(routes (wire (net 1) (path 0 1 0 0 1) (grid 1)))")
            .unwrap_err(),
        SpecctraParseError::InvalidSyntax
    );
    assert_eq!(
        parse_specctra_grid_route_records(
            "(routes (wire (net 1) (path 0 1 0 0 1 1) (layer 0) (grid 1)))"
        )
        .unwrap_err(),
        SpecctraParseError::InvalidSyntax
    );
    assert_eq!(
        parse_specctra_grid_route_records("(session \"empty\" (library ignored))").unwrap_err(),
        SpecctraParseError::InvalidSyntax
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn horizontal_parallel_gap_matches_integer_coordinate_difference(
        y0 in -100_i16..=100,
        gap in 0_i16..=100,
        width in 0_i16..=10,
    ) {
        let first = trace(1, 0, p(0, i64::from(y0)), p(10, i64::from(y0)), i64::from(width));
        let second_y = i64::from(y0) + i64::from(gap);
        let second = trace(2, 0, p(0, second_y), p(10, second_y), i64::from(width));
        let report = check_trace_clearance(&first, &second, &r(0), PredicatePolicy::default());
        if gap == 0 {
            prop_assert_eq!(report.status, ClearanceStatus::NoShortViolation);
        } else {
            prop_assert_eq!(report.axis_gap, Some(r(i64::from(gap))));
        }
    }

    #[test]
    fn trace_pad_clearance_handles_generated_axis_gaps(
        gap in 0_i16..=100,
        trace_width in 0_i16..=10,
        pad_diameter in 0_i16..=10,
    ) {
        let trace = trace(1, 0, p(0, 0), p(20, 0), i64::from(trace_width));
        let pad = PcbCircularPad::new(
            NetId(2),
            TraceLayer(0),
            p(10, i64::from(gap)),
            r(i64::from(pad_diameter)),
        ).unwrap();
        let report = check_trace_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());

        let doubled_gap = i64::from(gap) * 2;
        let overlap = i64::from(trace_width) + i64::from(pad_diameter);
        if doubled_gap <= overlap {
            prop_assert_eq!(report.status, ClearanceStatus::NoShortViolation);
        } else {
            prop_assert_eq!(report.status, ClearanceStatus::CertifiedClear);
        }
    }

    #[test]
    fn trace_via_drill_clearance_handles_generated_axis_gaps(
        gap in 0_i16..=100,
        trace_width in 0_i16..=10,
        drill_diameter in 0_i16..=10,
    ) {
        let trace = trace(1, 0, p(0, 0), p(20, 0), i64::from(trace_width));
        let via = PcbViaStack::with_drill(
            NetId(2),
            TraceLayer(0),
            TraceLayer(2),
            p(10, i64::from(gap)),
            r(20),
            r(i64::from(drill_diameter)),
        ).unwrap();
        let report = check_trace_via_drill_clearance(&trace, &via, &r(0), PredicatePolicy::default());

        let doubled_gap = i64::from(gap) * 2;
        let overlap = i64::from(trace_width) + i64::from(drill_diameter);
        if doubled_gap <= overlap {
            prop_assert_eq!(report.status, ClearanceStatus::NoShortViolation);
        } else {
            prop_assert_eq!(report.status, ClearanceStatus::CertifiedClear);
        }
    }

    #[test]
    fn via_layer_transition_generated_spans_stay_in_board_stackup(
        board_layers in 1_u16..=16,
        start in 0_u16..=15,
        end in 0_u16..=15,
    ) {
        prop_assume!(start <= end);
        prop_assume!(end < board_layers);
        let via = PcbViaStack::new(
            NetId(1),
            TraceLayer(start),
            TraceLayer(end),
            p(0, 0),
            r(10),
        ).unwrap();
        let report = via.classify_layer_transition(board_layers).unwrap();

        prop_assert_eq!(report.start_layer, TraceLayer(start));
        prop_assert_eq!(report.end_layer, TraceLayer(end));
        prop_assert_eq!(report.spanned_layers, end - start + 1);
        if start == end {
            prop_assert_eq!(report.class, ViaLayerTransitionClass::SingleLayerLand);
        } else if start == 0 && end == board_layers - 1 {
            prop_assert_eq!(report.class, ViaLayerTransitionClass::ThroughVia);
        } else if start == 0 || end == board_layers - 1 {
            prop_assert_eq!(report.class, ViaLayerTransitionClass::BlindVia);
        } else {
            prop_assert_eq!(report.class, ViaLayerTransitionClass::BuriedVia);
        }
    }

    #[test]
    fn via_layer_span_relation_generated_overlap_count_matches_interval_math(
        a_start in 0_u16..=16,
        a_end in 0_u16..=16,
        b_start in 0_u16..=16,
        b_end in 0_u16..=16,
    ) {
        prop_assume!(a_start <= a_end);
        prop_assume!(b_start <= b_end);
        let first = PcbViaStack::new(
            NetId(1),
            TraceLayer(a_start),
            TraceLayer(a_end),
            p(0, 0),
            r(10),
        ).unwrap();
        let second = PcbViaStack::new(
            NetId(2),
            TraceLayer(b_start),
            TraceLayer(b_end),
            p(1, 0),
            r(10),
        ).unwrap();
        let report = first.classify_layer_span_with(&second);
        let overlap_start = a_start.max(b_start);
        let overlap_end = a_end.min(b_end);

        if overlap_start <= overlap_end {
            let shared = overlap_end - overlap_start + 1;
            prop_assert_eq!(report.overlap_start, Some(TraceLayer(overlap_start)));
            prop_assert_eq!(report.overlap_end, Some(TraceLayer(overlap_end)));
            prop_assert_eq!(report.shared_layers, shared);
            if shared == 1 {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::TouchingLayer);
            } else {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::OverlappingLayers);
            }
        } else {
            prop_assert_eq!(report.shared_layers, 0);
            prop_assert_eq!(report.overlap_start, None);
            prop_assert_eq!(report.overlap_end, None);
            if a_end.checked_add(1) == Some(b_start) {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::AdjacentBelow);
            } else if b_end.checked_add(1) == Some(a_start) {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::AdjacentAbove);
            } else if a_end < b_start {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::DisjointBelow);
            } else {
                prop_assert_eq!(report.relation, ViaLayerSpanRelation::DisjointAbove);
            }
        }
    }

    #[test]
    fn via_drill_policy_generated_plated_ring_matches_annular_requirement(
        land in 0_i16..=64,
        drill in 0_i16..=64,
        minimum in 0_i16..=32,
    ) {
        let via = PcbViaStack::with_drill(
            NetId(1),
            TraceLayer(0),
            TraceLayer(1),
            p(0, 0),
            r(i64::from(land)),
            r(i64::from(drill)),
        ).unwrap();
        let report = via.classify_drill_policy(&r(i64::from(minimum)), PredicatePolicy::default());
        let expected = if i64::from(land) >= i64::from(drill) + 2 * i64::from(minimum) {
            ViaAnnularRingReport::Certified
        } else {
            ViaAnnularRingReport::Violation
        };

        prop_assert_eq!(report.class, ViaDrillPolicyClass::PlatedCopperVia);
        prop_assert_eq!(report.intent, ViaDrillIntent::Plated);
        prop_assert_eq!(report.drill_diameter, Some(r(i64::from(drill))));
        prop_assert_eq!(report.annular_ring, Some(expected));
    }

    #[test]
    fn specctra_grid_route_text_round_trips_generated_integer_records(
        net in 0_u32..=128,
        layer in 0_u16..=16,
        start_x in -1_000_i16..=1_000,
        start_y in -1_000_i16..=1_000,
        end_x in -1_000_i16..=1_000,
        end_y in -1_000_i16..=1_000,
        width in 0_i16..=64,
        grid_denominator in 1_u64..=10_000,
    ) {
        let records = vec![SpecctraGridTraceRecord {
            net: NetId(net),
            layer: TraceLayer(layer),
            start_x: i64::from(start_x),
            start_y: i64::from(start_y),
            end_x: i64::from(end_x),
            end_y: i64::from(end_y),
            width: i64::from(width),
            grid_denominator,
        }];

        let text = serialize_specctra_grid_trace_records(&records);
        prop_assert_eq!(parse_specctra_grid_trace_records(&text).unwrap(), records);
        prop_assert_eq!(import_specctra_text_route(&text).unwrap().traces().len(), 1);
    }

    #[test]
    fn specctra_path_wire_generated_points_lower_to_segment_count(
        net in 0_u32..=64,
        layer in 0_u16..=16,
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
        width in 0_i16..=64,
        grid_denominator in 1_u64..=10_000,
    ) {
        let text = format!(
            "(session generated (routes (wire (net {net}) (path {layer} {width} {x0} {y0} {x1} {y1} {x2} {y2}) (grid {grid_denominator}))))"
        );
        let parsed = parse_specctra_grid_route_records(&text).unwrap();
        let route = import_specctra_text_route(&text).unwrap();

        prop_assert_eq!(parsed.traces.len(), 2);
        prop_assert_eq!(route.traces().len(), 2);
        prop_assert_eq!(parsed.traces[0].end_x, i64::from(x1));
        prop_assert_eq!(parsed.traces[0].end_y, i64::from(y1));
        prop_assert_eq!(parsed.traces[1].start_x, i64::from(x1));
        prop_assert_eq!(parsed.traces[1].start_y, i64::from(y1));
    }

    #[test]
    fn specctra_grid_via_text_round_trips_generated_integer_records(
        net in 0_u32..=32,
        start_layer in 0_u16..=7,
        span in 0_u16..=7,
        x in -100_i16..=100,
        y in -100_i16..=100,
        land in 0_i16..=64,
        drill in 0_i16..=64,
        grid_denominator in 1_u64..=10_000,
    ) {
        let end_layer = start_layer.saturating_add(span);
        let records = vec![SpecctraGridViaRecord {
            net: NetId(net),
            start_layer: TraceLayer(start_layer),
            end_layer: TraceLayer(end_layer),
            x: i64::from(x),
            y: i64::from(y),
            land_diameter: i64::from(land),
            drill_diameter: i64::from(drill),
            drill_intent: ViaDrillIntent::Plated,
            grid_denominator,
        }];

        let text = serialize_specctra_grid_via_records(&records);
        let parsed = parse_specctra_grid_route_records(&text).unwrap();
        let route = import_specctra_text_route(&text).unwrap();

        prop_assert_eq!(parsed.vias, records);
        prop_assert!(parsed.traces.is_empty());
        prop_assert_eq!(route.vias().len(), 1);
        prop_assert_eq!(route.vias()[0].start_layer(), TraceLayer(start_layer));
        prop_assert_eq!(route.vias()[0].end_layer(), TraceLayer(end_layer));
        prop_assert_eq!(route.vias()[0].drill_intent(), ViaDrillIntent::Plated);
    }

    #[test]
    fn specctra_grid_net_aliases_round_trip_generated_atoms(
        net in 0_u32..=128,
        suffix in 0_u16..=999,
    ) {
        let alias = SpecctraNetAlias {
            net: NetId(net),
            name: format!("NET_{}", suffix),
        };
        let text = serialize_specctra_grid_route_records(&hyperpath::SpecctraGridRouteRecords {
            net_aliases: vec![alias.clone()],
            layer_aliases: Vec::new(),
            traces: Vec::new(),
            vias: Vec::new(),
        });
        let parsed = parse_specctra_grid_route_records(&text).unwrap();

        prop_assert_eq!(parsed.net_aliases, vec![alias]);
        prop_assert!(parsed.traces.is_empty());
        prop_assert!(parsed.vias.is_empty());
    }

    #[test]
    fn specctra_grid_layer_aliases_round_trip_generated_atoms(
        layer in 0_u16..=128,
        suffix in 0_u16..=999,
    ) {
        let alias = SpecctraLayerAlias {
            layer: TraceLayer(layer),
            name: format!("LAYER_{}", suffix),
        };
        let text = serialize_specctra_grid_route_records(&hyperpath::SpecctraGridRouteRecords {
            net_aliases: Vec::new(),
            layer_aliases: vec![alias.clone()],
            traces: Vec::new(),
            vias: Vec::new(),
        });
        let parsed = parse_specctra_grid_route_records(&text).unwrap();

        prop_assert_eq!(parsed.layer_aliases, vec![alias]);
        prop_assert!(parsed.net_aliases.is_empty());
        prop_assert!(parsed.traces.is_empty());
        prop_assert!(parsed.vias.is_empty());
    }

    #[test]
    fn source_grid_lifts_generated_integer_tokens_exactly(
        units in -10_000_i16..=10_000,
        denominator in 1_u64..=10_000,
    ) {
        let provenance = PathProvenance::fixed_grid_with_unit(
            PathSourceFormat::Other,
            denominator,
            SourceLengthUnit::BoardUnit,
        ).unwrap();

        prop_assert_eq!(
            provenance.real_from_units(i64::from(units)),
            Some(Real::new(Rational::fraction(i64::from(units), denominator).unwrap()))
        );
    }

    #[test]
    fn generated_construction_stamps_detect_exact_revision_freshness(
        id in 0_u64..=1_000,
        revision in 0_u64..=1_000,
    ) {
        let stamp = ConstructionStamp::new(id, revision);
        let provenance = PathProvenance::native().with_construction(stamp);

        prop_assert!(provenance.is_fresh_for(stamp));
        prop_assert!(!provenance.is_fresh_for(stamp.next_revision()));
        prop_assert!(provenance.shares_construction_with(PathProvenance::native().with_construction(stamp)));
    }

    #[test]
    fn quadratic_bezier_generated_endpoints_evaluate_exactly(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control = p(i64::from(x1), i64::from(y1));
        let end = p(i64::from(x2), i64::from(y2));
        let curve = QuadraticBezier::new(start.clone(), control, end.clone());

        prop_assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()), start);
        prop_assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()), end);
    }

    #[test]
    fn quadratic_bezier_generated_endpoint_hodographs_are_exact(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control = p(i64::from(x1), i64::from(y1));
        let end = p(i64::from(x2), i64::from(y2));
        let curve = QuadraticBezier::new(start.clone(), control.clone(), end.clone());

        prop_assert_eq!(
            curve.derivative(BezierParameter::new(0, 1).unwrap()),
            Point2::new(
                (control.x.clone() - start.x.clone()) * r(2),
                (control.y.clone() - start.y.clone()) * r(2),
            )
        );
        prop_assert_eq!(
            curve.derivative(BezierParameter::new(1, 1).unwrap()),
            Point2::new(
                (end.x.clone() - control.x.clone()) * r(2),
                (end.y.clone() - control.y.clone()) * r(2),
            )
        );
    }

    #[test]
    fn rational_quadratic_bezier_generated_endpoints_evaluate_exactly(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
        weight in 0_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control = p(i64::from(x1), i64::from(y1));
        let end = p(i64::from(x2), i64::from(y2));
        let curve = RationalQuadraticBezier::new(start.clone(), control, end.clone(), r(i64::from(weight))).unwrap();

        prop_assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()).unwrap(), start);
        prop_assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()).unwrap(), end);
    }

    #[test]
    fn rational_quadratic_bezier_generated_endpoint_hodographs_are_exact(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
        weight in 1_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control = p(i64::from(x1), i64::from(y1));
        let end = p(i64::from(x2), i64::from(y2));
        let weight = r(i64::from(weight));
        let curve = RationalQuadraticBezier::new(
            start.clone(),
            control.clone(),
            end.clone(),
            weight.clone(),
        ).unwrap();

        prop_assert_eq!(
            curve.derivative(BezierParameter::new(0, 1).unwrap()).unwrap(),
            Point2::new(
                (control.x.clone() - start.x.clone()) * r(2) * weight.clone(),
                (control.y.clone() - start.y.clone()) * r(2) * weight.clone(),
            )
        );
        prop_assert_eq!(
            curve.derivative(BezierParameter::new(1, 1).unwrap()).unwrap(),
            Point2::new(
                (end.x.clone() - control.x.clone()) * r(2) * weight.clone(),
                (end.y.clone() - control.y.clone()) * r(2) * weight,
            )
        );
    }

    #[test]
    fn cubic_bezier_generated_endpoints_evaluate_exactly(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
        x3 in -100_i16..=100,
        y3 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control0 = p(i64::from(x1), i64::from(y1));
        let control1 = p(i64::from(x2), i64::from(y2));
        let end = p(i64::from(x3), i64::from(y3));
        let curve = CubicBezier::new(start.clone(), control0, control1, end.clone());

        prop_assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()), start);
        prop_assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()), end);
    }

    #[test]
    fn cubic_bezier_generated_endpoint_hodographs_are_exact(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
        x2 in -100_i16..=100,
        y2 in -100_i16..=100,
        x3 in -100_i16..=100,
        y3 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control0 = p(i64::from(x1), i64::from(y1));
        let control1 = p(i64::from(x2), i64::from(y2));
        let end = p(i64::from(x3), i64::from(y3));
        let curve = CubicBezier::new(start.clone(), control0.clone(), control1.clone(), end.clone());

        prop_assert_eq!(
            curve.derivative(BezierParameter::new(0, 1).unwrap()),
            Point2::new(
                (control0.x.clone() - start.x.clone()) * r(3),
                (control0.y.clone() - start.y.clone()) * r(3),
            )
        );
        prop_assert_eq!(
            curve.derivative(BezierParameter::new(1, 1).unwrap()),
            Point2::new(
                (end.x.clone() - control1.x.clone()) * r(3),
                (end.y.clone() - control1.y.clone()) * r(3),
            )
        );
    }

    #[test]
    fn higher_order_bezier_generated_endpoints_and_hodographs_are_exact(
        x0 in -50_i16..=50,
        y0 in -50_i16..=50,
        x1 in -50_i16..=50,
        y1 in -50_i16..=50,
        x2 in -50_i16..=50,
        y2 in -50_i16..=50,
        x3 in -50_i16..=50,
        y3 in -50_i16..=50,
        x4 in -50_i16..=50,
        y4 in -50_i16..=50,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let control0 = p(i64::from(x1), i64::from(y1));
        let control1 = p(i64::from(x2), i64::from(y2));
        let control2 = p(i64::from(x3), i64::from(y3));
        let end = p(i64::from(x4), i64::from(y4));
        let curve = HigherOrderBezier::quartic(
            start.clone(),
            control0.clone(),
            control1,
            control2,
            end.clone(),
        );

        prop_assert_eq!(curve.eval(BezierParameter::new(0, 1).unwrap()), start.clone());
        prop_assert_eq!(curve.eval(BezierParameter::new(1, 1).unwrap()), end);
        prop_assert_eq!(
            curve.derivative(BezierParameter::new(0, 1).unwrap()),
            Point2::new(
                (control0.x.clone() - start.x.clone()) * r(4),
                (control0.y.clone() - start.y.clone()) * r(4),
            )
        );
    }

    #[test]
    fn bezier_offset_generated_linear_quadratics_have_exact_unit_normal_witness(
        step in 1_i16..=50,
        distance in 0_i16..=50,
    ) {
        let step = i64::from(step);
        let distance = i64::from(distance);
        let curve = QuadraticBezier::new(p(0, 0), p(step, 0), p(2 * step, 0));
        let sample = offset_quadratic_bezier_sample(
            &curve,
            BezierParameter::new(1, 2).unwrap(),
            r(distance),
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(sample.tangent, p(2 * step, 0));
        prop_assert_eq!(sample.normal, p(0, 2 * step));
        prop_assert_eq!(sample.speed_squared, r(4 * step * step));
        prop_assert_eq!(sample.offset_point, Some(p(step, distance)));
    }

    #[test]
    fn rectangular_pocket_plan_generated_square_ring_count_matches_exact_half_width(
        size in 2_i16..=100,
        stepover in 1_i16..=20,
    ) {
        let size = i64::from(size);
        let stepover = i64::from(stepover);
        let pocket = RectangularPocket::new(p(0, 0), p(size, size)).unwrap();
        let plan = build_rectangular_pocket_plan(
            pocket,
            r(0),
            r(stepover),
            128,
            PredicatePolicy::default(),
        ).unwrap();
        let expected = (size / (2 * stepover) + 1) as usize;

        prop_assert_eq!(plan.rings.len(), expected);
        prop_assert_eq!(plan.stop_reason, PocketPlanStopReason::GeometryExhausted);
        for ring in &plan.rings {
            prop_assert_eq!(ring.min.x.clone(), ring.inset.clone());
            prop_assert_eq!(ring.min.y.clone(), ring.inset.clone());
            prop_assert_eq!(ring.max.x.clone(), r(size) - ring.inset.clone());
            prop_assert_eq!(ring.max.y.clone(), r(size) - ring.inset.clone());
        }
    }

    #[test]
    fn rectangular_bead_plan_generated_horizontal_count_matches_exact_pitch(
        height in 2_i16..=100,
        spacing in 1_i16..=20,
    ) {
        let height = i64::from(height);
        let spacing = i64::from(spacing);
        let region = RectangularPocket::new(p(0, 0), p(10, height)).unwrap();
        let plan = build_rectangular_bead_plan(
            region,
            BeadFillAxis::Horizontal,
            r(2),
            r(spacing),
            128,
            PredicatePolicy::default(),
        ).unwrap();
        let expected = if height < 2 {
            0
        } else {
            ((height - 2) / spacing + 1) as usize
        };

        prop_assert_eq!(plan.beads.len(), expected);
        prop_assert_eq!(plan.stop_reason, PocketPlanStopReason::GeometryExhausted);
        for bead in &plan.beads {
            prop_assert_eq!(bead.segment.start().x.clone(), r(0));
            prop_assert_eq!(bead.segment.end().x.clone(), r(10));
            prop_assert_eq!(bead.segment.start().y.clone(), bead.pitch_position.clone());
            prop_assert_eq!(bead.segment.end().y.clone(), bead.pitch_position.clone());
        }
    }

    #[test]
    fn rectangular_serpentine_infill_graph_generated_links_are_endpoint_continuous(
        height in 2_i16..=100,
        spacing in 1_i16..=20,
    ) {
        let height = i64::from(height);
        let spacing = i64::from(spacing);
        let plan = build_rectangular_bead_plan(
            RectangularPocket::new(p(0, 0), p(10, height)).unwrap(),
            BeadFillAxis::Horizontal,
            r(2),
            r(spacing),
            128,
            PredicatePolicy::default(),
        ).unwrap();
        prop_assume!(!plan.beads.is_empty());
        let expected_links = plan.beads.len().saturating_sub(1);
        let graph = build_rectangular_serpentine_infill_graph(
            plan,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(graph.links.len(), expected_links);
        prop_assert_eq!(graph.deposition_segments.len(), graph.plan.beads.len());
        for (index, link) in graph.links.iter().enumerate() {
            prop_assert_eq!(link.from_bead, index);
            prop_assert_eq!(link.to_bead, index + 1);
            prop_assert_eq!(
                link.connector.start(),
                graph.deposition_segments[index].end()
            );
            prop_assert_eq!(
                link.connector.end(),
                graph.deposition_segments[index + 1].start()
            );
        }
    }

    #[test]
    fn rectangular_support_plan_generated_margin_matches_exact_footprint(
        x0 in 0_i16..=40,
        y0 in 0_i16..=40,
        width in 0_i16..=20,
        height in 0_i16..=20,
        margin in 0_i16..=10,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let width = i64::from(width);
        let height = i64::from(height);
        let margin = i64::from(margin);
        let overhang = RectangularPocket::new(
            p(x0, y0),
            p(x0 + width, y0 + height),
        ).unwrap();
        let base = RectangularPocket::new(p(-20, -20), p(80, 80)).unwrap();
        let plan = build_rectangular_support_plan(
            overhang,
            base,
            r(margin),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(plan.footprint.min(), &p(x0 - margin, y0 - margin));
        prop_assert_eq!(plan.footprint.max(), &p(x0 + width + margin, y0 + height + margin));
        prop_assert_eq!(plan.status, SupportFootprintStatus::ContainedInBase);
    }

    #[test]
    fn rectangular_region_set_algebra_generated_inner_cut_removes_exact_area(
        x0 in 0_i16..=20,
        y0 in 0_i16..=20,
        width in 1_i16..=20,
        height in 1_i16..=20,
        inset in 0_i16..=5,
    ) {
        let x0 = i64::from(x0);
        let y0 = i64::from(y0);
        let width = i64::from(width);
        let height = i64::from(height);
        let inset = i64::from(inset);
        let subject = RectangularPocket::new(
            p(x0, y0),
            p(x0 + width + 2 * inset, y0 + height + 2 * inset),
        ).unwrap();
        let cutter = RectangularPocket::new(
            p(x0 + inset, y0 + inset),
            p(x0 + inset + width, y0 + inset + height),
        ).unwrap();
        let report = subtract_rectangular_region(
            subject,
            cutter,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.relation, RectangularRegionRelation::AreaOverlap);
        prop_assert_eq!(
            report.intersection.as_ref().unwrap().min(),
            &p(x0 + inset, y0 + inset)
        );
        prop_assert_eq!(
            report.intersection.as_ref().unwrap().max(),
            &p(x0 + inset + width, y0 + inset + height)
        );
        for piece in &report.remainder {
            prop_assert_ne!(&piece.min().x, &piece.max().x);
            prop_assert_ne!(&piece.min().y, &piece.max().y);
        }
    }

    #[test]
    fn horizontal_offset_preserves_generated_integer_axis_length(
        x0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y in -100_i16..=100,
        distance in 0_i16..=100,
    ) {
        prop_assume!(x0 != x1);
        let segment = LinePathSegment::new(
            p(i64::from(x0), i64::from(y)),
            p(i64::from(x1), i64::from(y)),
        );
        let offset = offset_axis_aligned_segment(
            &segment,
            r(i64::from(distance)),
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            offset.segment.axis_length(PredicatePolicy::default()),
            segment.axis_length(PredicatePolicy::default())
        );
        prop_assert_eq!(
            offset.segment.start().x.clone() - segment.start().x.clone(),
            Real::zero()
        );
        prop_assert_eq!(
            offset.segment.end().x.clone() - segment.end().x.clone(),
            Real::zero()
        );
    }

    #[test]
    fn single_detour_meander_generated_length_is_baseline_plus_extra(
        length in 1_i16..=200,
        extra in 0_i16..=200,
    ) {
        let source = LinePathSegment::new(p(0, 0), p(i64::from(length), 0));
        let meander = build_single_detour_meander(
            &source,
            r(i64::from(extra)),
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            meander.exact_axis_length(PredicatePolicy::default()).unwrap(),
            r(i64::from(length) + i64::from(extra))
        );
        prop_assert!(
            meander
                .certify_target_length(r(i64::from(length) + i64::from(extra)), PredicatePolicy::default())
                .unwrap()
                .all_satisfied()
        );
    }

    #[test]
    fn differential_pair_skew_generated_axis_routes_certify_exact_difference(
        first_horizontal in 1_i16..=200,
        first_vertical in 1_i16..=200,
        second_horizontal in 1_i16..=200,
    ) {
        let first = vec![
            LinePathSegment::new(p(0, 0), p(i64::from(first_horizontal), 0)),
            LinePathSegment::new(
                p(i64::from(first_horizontal), 0),
                p(i64::from(first_horizontal), i64::from(first_vertical)),
            ),
        ];
        let second = vec![LinePathSegment::new(
            p(0, 10),
            p(i64::from(second_horizontal), 10),
        )];
        let expected = i64::from(first_horizontal) + i64::from(first_vertical)
            - i64::from(second_horizontal);
        let report = certify_differential_pair_skew(
            &first,
            &second,
            r(expected),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.first_length, r(i64::from(first_horizontal) + i64::from(first_vertical)));
        prop_assert_eq!(report.second_length, r(i64::from(second_horizontal)));
        prop_assert_eq!(report.actual_skew, r(expected));
        prop_assert!(report.certification.all_satisfied());
    }

    #[test]
    fn constant_feed_time_generated_axis_routes_certify_exact_product(
        feed_rate in 1_i16..=20,
        time in 1_i16..=20,
    ) {
        let path_length = i64::from(feed_rate) * i64::from(time);
        let route = vec![LinePathSegment::new(p(0, 0), p(path_length, 0))];
        let report = certify_constant_feed_time(
            &route,
            r(i64::from(feed_rate)),
            r(i64::from(time)),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.path_length, r(path_length));
        prop_assert!(report.certification.all_satisfied());
    }

    #[test]
    fn multi_detour_meander_generated_length_is_baseline_plus_extra(
        length in 1_i16..=200,
        extra in 0_i16..=200,
        bump_count in 1_u64..=8,
    ) {
        let source = LinePathSegment::new(p(0, 0), p(i64::from(length), 0));
        let meander = build_multi_detour_meander(
            &source,
            r(i64::from(extra)),
            bump_count,
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            meander.exact_axis_length(PredicatePolicy::default()).unwrap(),
            r(i64::from(length) + i64::from(extra))
        );
        prop_assert!(
            meander
                .certify_target_length(r(i64::from(length) + i64::from(extra)), PredicatePolicy::default())
                .unwrap()
                .all_satisfied()
        );
        if extra == 0 {
            prop_assert_eq!(meander.segments.len(), 1);
        } else {
            prop_assert_eq!(meander.segments.len(), bump_count as usize * 3);
        }
    }

    #[test]
    fn alternating_detour_meander_generated_length_is_baseline_plus_extra(
        length in 1_i16..=200,
        extra in 0_i16..=200,
        bump_count in 1_u64..=8,
        starts_left in any::<bool>(),
    ) {
        let source = LinePathSegment::new(p(0, 0), p(i64::from(length), 0));
        let first_side = if starts_left {
            OffsetSide::Left
        } else {
            OffsetSide::Right
        };
        let meander = build_alternating_detour_meander(
            &source,
            r(i64::from(extra)),
            bump_count,
            first_side,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            meander.exact_axis_length(PredicatePolicy::default()).unwrap(),
            r(i64::from(length) + i64::from(extra))
        );
        prop_assert!(
            meander
                .certify_target_length(r(i64::from(length) + i64::from(extra)), PredicatePolicy::default())
                .unwrap()
                .all_satisfied()
        );
        if extra == 0 {
            prop_assert_eq!(meander.segments.len(), 1);
        } else {
            prop_assert_eq!(meander.segments.len(), bump_count as usize * 3);
        }
    }

    #[test]
    fn nonuniform_detour_meander_generated_length_is_baseline_plus_amplitudes(
        length in 1_i16..=200,
        a in 0_i16..=50,
        b in 0_i16..=50,
        c in 0_i16..=50,
    ) {
        let source = LinePathSegment::new(p(0, 0), p(i64::from(length), 0));
        let amplitudes = vec![r(i64::from(a)), r(i64::from(b)), r(i64::from(c))];
        let expected_extra = i64::from(a + b + c) * 2;
        let meander = build_nonuniform_detour_meander(
            &source,
            amplitudes,
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(&meander.extra_length, &r(expected_extra));
        prop_assert_eq!(
            meander.exact_axis_length(PredicatePolicy::default()).unwrap(),
            r(i64::from(length) + expected_extra)
        );
        prop_assert!(
            meander
                .certify_target_length(r(i64::from(length) + expected_extra), PredicatePolicy::default())
                .unwrap()
                .all_satisfied()
        );
    }

    #[test]
    fn obstacle_aware_detour_meander_generated_empty_obstacles_keeps_preferred_side(
        length in 3_i16..=200,
        extra in 1_i16..=200,
        bump_count in 1_u64..=8,
    ) {
        let source = LinePathSegment::new(p(0, 0), p(i64::from(length), 0));
        let routed = build_obstacle_aware_detour_meander(
            &source,
            r(i64::from(extra)),
            bump_count,
            OffsetSide::Left,
            vec![],
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            routed.selected_sides,
            vec![OffsetSide::Left; bump_count as usize]
        );
        prop_assert_eq!(
            routed.meander.exact_axis_length(PredicatePolicy::default()).unwrap(),
            r(i64::from(length) + i64::from(extra))
        );
        prop_assert!(
            routed
                .meander
                .certify_target_length(r(i64::from(length) + i64::from(extra)), PredicatePolicy::default())
                .unwrap()
                .all_satisfied()
        );
    }

    #[test]
    fn meander_placement_slots_generated_empty_obstacles_select_preferred_side(
        amplitude in 0_i16..=50,
        bump_count in 1_u64..=8,
    ) {
        let source = LinePathSegment::new(p(0, 0), p(80, 0));
        let report = classify_meander_placement_slots(
            &source,
            r(i64::from(amplitude)),
            bump_count,
            OffsetSide::Left,
            Vec::new(),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.slots.len(), bump_count as usize);
        for (index, slot) in report.slots.iter().enumerate() {
            prop_assert_eq!(slot.index, index as u64);
            prop_assert_eq!(&slot.amplitude, &r(i64::from(amplitude)));
            prop_assert!(!slot.preferred_blocked);
            prop_assert!(!slot.opposite_blocked);
            prop_assert_eq!(slot.selected_side, Some(OffsetSide::Left));
        }
    }

    #[test]
    fn meander_candidate_slots_generated_empty_obstacles_keep_amplitudes(
        length_a in 1_i16..=80,
        gap in 0_i16..=20,
        length_b in 1_i16..=80,
        amplitude_a in 0_i16..=30,
        amplitude_b in 0_i16..=30,
    ) {
        let first = MeanderPlacementCandidate {
            base: LinePathSegment::new(p(0, 0), p(i64::from(length_a), 0)),
            amplitude: r(i64::from(amplitude_a)),
        };
        let second_start = i64::from(length_a + gap);
        let second = MeanderPlacementCandidate {
            base: LinePathSegment::new(
                p(second_start, 0),
                p(second_start + i64::from(length_b), 0),
            ),
            amplitude: r(i64::from(amplitude_b)),
        };
        let report = classify_meander_candidate_slots(
            vec![first.clone(), second.clone()],
            OffsetSide::Right,
            Vec::new(),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.slots.len(), 2);
        prop_assert_eq!(&report.slots[0].base, &first.base);
        prop_assert_eq!(&report.slots[0].amplitude, &first.amplitude);
        prop_assert_eq!(report.slots[0].selected_side, Some(OffsetSide::Right));
        prop_assert_eq!(&report.slots[1].base, &second.base);
        prop_assert_eq!(&report.slots[1].amplitude, &second.amplitude);
        prop_assert_eq!(report.slots[1].selected_side, Some(OffsetSide::Right));
    }

    #[test]
    fn cardinal_arc_outward_offset_preserves_generated_quarter_turns(
        cx in -100_i16..=100,
        cy in -100_i16..=100,
        radius in 1_i16..=100,
        distance in 0_i16..=100,
    ) {
        let arc = CircularArc::cardinal(
            p(i64::from(cx), i64::from(cy)),
            r(i64::from(radius)),
            CardinalPoint::East,
            CardinalPoint::North,
            ArcDirection::Ccw,
        ).unwrap();
        let offset = offset_cardinal_arc(
            &arc,
            r(i64::from(distance)),
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(offset.arc.facts().quarter_turns, arc.facts().quarter_turns);
        prop_assert_eq!(offset.arc.radius(), &r(i64::from(radius) + i64::from(distance)));
        prop_assert_eq!(offset.arc.center(), arc.center());
    }

    #[test]
    fn explicit_circular_arc_accepts_generated_pythagorean_endpoints(
        scale in 1_i16..=50,
        cx in -100_i16..=100,
        cy in -100_i16..=100,
    ) {
        let scale = i64::from(scale);
        let center = p(i64::from(cx), i64::from(cy));
        let start = p(i64::from(cx) + 3 * scale, i64::from(cy) + 4 * scale);
        let end = p(i64::from(cx) - 4 * scale, i64::from(cy) + 3 * scale);
        let arc = ExplicitCircularArc::new(
            center,
            r(5 * scale),
            start.clone(),
            end.clone(),
            ArcDirection::Ccw,
        ).unwrap();

        prop_assert_eq!(arc.start(), &start);
        prop_assert_eq!(arc.end(), &end);
        prop_assert_eq!(&arc.facts().radius_squared, &r(25 * scale * scale));
        prop_assert_eq!(arc.chord_length_squared(), r(50 * scale * scale));
        prop_assert_eq!(arc.facts().sweep_class, ExplicitArcSweepClass::LessThanHalfTurn);
        prop_assert_eq!(
            arc.certified_sweep_length(),
            Some(r(5 * scale) * Real::pi() * Real::new(Rational::fraction(1, 2).unwrap()))
        );
    }

    #[test]
    fn explicit_circular_arc_generated_full_circle_classifies_cardinal_points_on_arc(
        scale in 1_i16..=50,
        cx in -100_i16..=100,
        cy in -100_i16..=100,
        clockwise in any::<bool>(),
    ) {
        let scale = i64::from(scale);
        let center = p(i64::from(cx), i64::from(cy));
        let start = p(i64::from(cx) + 3 * scale, i64::from(cy) + 4 * scale);
        let direction = if clockwise { ArcDirection::Cw } else { ArcDirection::Ccw };
        let arc = ExplicitCircularArc::new(
            center.clone(),
            r(5 * scale),
            start.clone(),
            start,
            direction,
        ).unwrap();
        let north = p(i64::from(cx), i64::from(cy) + 5 * scale);
        let east = p(i64::from(cx) + 5 * scale, i64::from(cy));

        prop_assert_eq!(
            arc.classify_point(&north, PredicatePolicy::default()),
            ExplicitArcPointClassification::OnArc
        );
        prop_assert_eq!(
            arc.classify_point(&east, PredicatePolicy::default()),
            ExplicitArcPointClassification::OnArc
        );
        prop_assert_eq!(
            arc.classify_point(&center, PredicatePolicy::default()),
            ExplicitArcPointClassification::OffCircle
        );
    }

    #[test]
    fn explicit_circular_arc_generated_full_circle_horizontal_line_intersects_twice(
        scale in 1_i16..=50,
    ) {
        let scale = i64::from(scale);
        let arc = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(3 * scale, 4 * scale),
            p(3 * scale, 4 * scale),
            ArcDirection::Ccw,
        ).unwrap();
        let line = LinePathSegment::new(p(-10 * scale, 4 * scale), p(10 * scale, 4 * scale));
        let report = arc.intersect_axis_aligned_segment(&line, PredicatePolicy::default());

        prop_assert_eq!(report.class, LineExplicitArcIntersectionClass::Secant);
        prop_assert_eq!(report.points, vec![p(3 * scale, 4 * scale), p(-3 * scale, 4 * scale)]);
    }

    #[test]
    fn explicit_circular_arc_generated_full_circle_covers_minor_arc(
        scale in 1_i16..=50,
        clockwise in any::<bool>(),
    ) {
        let scale = i64::from(scale);
        let direction = if clockwise { ArcDirection::Cw } else { ArcDirection::Ccw };
        let full = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(3 * scale, 4 * scale),
            p(3 * scale, 4 * scale),
            direction,
        ).unwrap();
        let minor = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(3 * scale, 4 * scale),
            p(-3 * scale, 4 * scale),
            ArcDirection::Ccw,
        ).unwrap();
        let report = full.classify_same_circle_overlap(&minor, PredicatePolicy::default());

        prop_assert_eq!(report.class, ExplicitArcOverlapClass::FirstCoversSecond);
    }

    #[test]
    fn explicit_circular_arc_generated_external_tangent_relation(
        radius in 1_i16..=50,
    ) {
        let radius = i64::from(radius);
        let left = ExplicitCircularArc::new(
            p(0, 0),
            r(radius),
            p(radius, 0),
            p(0, radius),
            ArcDirection::Ccw,
        ).unwrap();
        let right = ExplicitCircularArc::new(
            p(2 * radius, 0),
            r(radius),
            p(radius, 0),
            p(2 * radius, radius),
            ArcDirection::Ccw,
        ).unwrap();
        let report = left.classify_circle_relation(&right, PredicatePolicy::default());

        prop_assert_eq!(report.class, ExplicitCircleRelationClass::ExternallyTangent);
        prop_assert_eq!(report.center_distance_squared, r(4 * radius * radius));
        prop_assert_eq!(report.radius_sum_squared, r(4 * radius * radius));
        prop_assert_eq!(report.tangent_point, Some(p(radius, 0)));
        let arc_report = left.classify_tangent_intersection(&right, PredicatePolicy::default());
        prop_assert_eq!(arc_report.class, ExplicitArcTangentClass::TangentOnBoth);
        prop_assert_eq!(arc_report.tangent_point, Some(p(radius, 0)));
    }

    #[test]
    fn explicit_circular_arc_generated_secant_intersections_are_exact(
        scale in 1_i16..=50,
    ) {
        let scale = i64::from(scale);
        let left = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(5 * scale, 0),
            p(5 * scale, 0),
            ArcDirection::Ccw,
        ).unwrap();
        let right = ExplicitCircularArc::new(
            p(6 * scale, 0),
            r(5 * scale),
            p(11 * scale, 0),
            p(11 * scale, 0),
            ArcDirection::Ccw,
        ).unwrap();
        let report = left.intersect_arc(&right, PredicatePolicy::default());

        prop_assert_eq!(report.class, ExplicitArcIntersectionClass::TwoPoints);
        prop_assert_eq!(report.circle_relation, ExplicitCircleRelationClass::Secant);
        prop_assert_eq!(report.points, vec![p(3 * scale, 4 * scale), p(3 * scale, -4 * scale)]);
        let arrangement = left.arrange_with(&right, PredicatePolicy::default());
        prop_assert_eq!(
            arrangement.class,
            ExplicitArcArrangementClass::DifferentCircleTwoPoints
        );
    }

    #[test]
    fn explicit_circular_arc_generated_tangents_are_perpendicular_to_radii(
        scale in 1_i16..=50,
        clockwise in any::<bool>(),
    ) {
        let scale = i64::from(scale);
        let direction = if clockwise { ArcDirection::Cw } else { ArcDirection::Ccw };
        let arc = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(3 * scale, 4 * scale),
            p(-4 * scale, 3 * scale),
            direction,
        ).unwrap();
        let start_tangent = arc.start_tangent();
        let end_tangent = arc.end_tangent();
        let start_dot = arc.start().x.clone() * start_tangent.x
            + arc.start().y.clone() * start_tangent.y;
        let end_dot = arc.end().x.clone() * end_tangent.x
            + arc.end().y.clone() * end_tangent.y;

        prop_assert_eq!(start_dot, Real::zero());
        prop_assert_eq!(end_dot, Real::zero());
    }

    #[test]
    fn explicit_circular_arc_outward_offset_preserves_generated_pythagorean_shape(
        scale in 1_i16..=50,
        distance in 0_i16..=50,
    ) {
        let scale = i64::from(scale);
        let distance = i64::from(distance);
        let arc = ExplicitCircularArc::new(
            p(0, 0),
            r(5 * scale),
            p(3 * scale, 4 * scale),
            p(-4 * scale, 3 * scale),
            ArcDirection::Ccw,
        ).unwrap();
        let offset = offset_explicit_arc(
            &arc,
            r(5 * distance),
            OffsetSide::Left,
            PredicatePolicy::default(),
        ).unwrap();
        let expected_scale = scale + distance;

        prop_assert_eq!(offset.arc.radius(), &r(5 * expected_scale));
        prop_assert_eq!(offset.arc.start(), &p(3 * expected_scale, 4 * expected_scale));
        prop_assert_eq!(offset.arc.end(), &p(-4 * expected_scale, 3 * expected_scale));
        prop_assert_eq!(&offset.arc.facts().radius_squared, &r(25 * expected_scale * expected_scale));
    }

    #[test]
    fn cardinal_rect_pad_effective_extents_match_rotation(
        width in 0_i16..=100,
        height in 0_i16..=100,
        rotated in any::<bool>(),
    ) {
        let rotation = if rotated {
            CardinalRotation::Deg90
        } else {
            CardinalRotation::Deg0
        };
        let pad = PcbCardinalRectPad::new(
            NetId(1),
            TraceLayer(0),
            p(0, 0),
            r(i64::from(width)),
            r(i64::from(height)),
            rotation,
        ).unwrap();
        let effective = pad.effective_rect().unwrap();

        if rotated {
            prop_assert_eq!(effective.width(), &r(i64::from(height)));
            prop_assert_eq!(effective.height(), &r(i64::from(width)));
        } else {
            prop_assert_eq!(effective.width(), &r(i64::from(width)));
            prop_assert_eq!(effective.height(), &r(i64::from(height)));
        }
    }

    #[test]
    fn board_clearance_accepts_generated_interior_axis_traces(
        x0 in 20_i16..=80,
        x1 in 20_i16..=80,
        y in 20_i16..=80,
        width in 0_i16..=10,
        clearance in 0_i16..=10,
    ) {
        prop_assume!(x0 != x1);
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let trace = trace(
            1,
            0,
            p(i64::from(x0), i64::from(y)),
            p(i64::from(x1), i64::from(y)),
            i64::from(width),
        );

        prop_assert_eq!(
            check_trace_board_clearance(
                &trace,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn convex_board_clearance_accepts_generated_interior_axis_traces(
        x0 in 20_i16..=70,
        x1 in 20_i16..=70,
        y in 20_i16..=70,
        width in 0_i16..=8,
        clearance in 0_i16..=8,
    ) {
        prop_assume!(x0 != x1);
        let board = PcbConvexBoardOutline::new(vec![
            p(0, 0),
            p(100, 0),
            p(100, 100),
            p(0, 100),
        ]).unwrap();
        let trace = trace(
            1,
            0,
            p(i64::from(x0), i64::from(y)),
            p(i64::from(x1), i64::from(y)),
            i64::from(width),
        );

        prop_assert_eq!(
            check_trace_convex_board_clearance(
                &trace,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn orthogonal_board_clearance_accepts_generated_lower_lobe_traces(
        x0 in 20_i16..=80,
        x1 in 20_i16..=80,
        y in 20_i16..=25,
        width in 0_i16..=8,
        clearance in 0_i16..=8,
    ) {
        prop_assume!(x0 != x1);
        let board = PcbOrthogonalBoardOutline::new(vec![
            p(0, 0),
            p(100, 0),
            p(100, 100),
            p(60, 100),
            p(60, 40),
            p(40, 40),
            p(40, 100),
            p(0, 100),
        ]).unwrap();
        let trace = trace(
            1,
            0,
            p(i64::from(x0), i64::from(y)),
            p(i64::from(x1), i64::from(y)),
            i64::from(width),
        );

        prop_assert_eq!(
            check_trace_orthogonal_board_clearance(
                &trace,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn circular_board_clearance_accepts_generated_interior_diagonal_traces(
        x0 in -20_i16..=20,
        y0 in -20_i16..=20,
        x1 in -20_i16..=20,
        y1 in -20_i16..=20,
        width in 0_i16..=10,
        clearance in 0_i16..=10,
    ) {
        let board = PcbCircularBoardOutline::new(p(0, 0), r(50)).unwrap();
        let trace = trace(
            1,
            0,
            p(i64::from(x0), i64::from(y0)),
            p(i64::from(x1), i64::from(y1)),
            i64::from(width),
        );

        prop_assert_eq!(
            check_trace_circular_board_clearance(
                &trace,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn via_drill_board_clearance_accepts_generated_interior_drills(
        x in 20_i16..=80,
        y in 20_i16..=80,
        drill in 0_i16..=10,
        clearance in 0_i16..=10,
    ) {
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let via = PcbViaStack::with_drill(
            NetId(1),
            TraceLayer(0),
            TraceLayer(2),
            p(i64::from(x), i64::from(y)),
            r(20),
            r(i64::from(drill)),
        ).unwrap();

        prop_assert_eq!(
            check_via_drill_board_clearance(
                &via,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn circular_pad_board_clearance_accepts_generated_interior_pads(
        x in 30_i16..=70,
        y in 30_i16..=70,
        diameter in 0_i16..=20,
        clearance in 0_i16..=10,
    ) {
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let pad = PcbCircularPad::new(
            NetId(1),
            TraceLayer(0),
            p(i64::from(x), i64::from(y)),
            r(i64::from(diameter)),
        ).unwrap();

        prop_assert_eq!(
            check_circular_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn circular_pad_circular_board_clearance_accepts_generated_interior_pads(
        x in -20_i16..=20,
        y in -20_i16..=20,
        diameter in 0_i16..=20,
        clearance in 0_i16..=10,
    ) {
        let board = PcbCircularBoardOutline::new(p(0, 0), r(50)).unwrap();
        let pad = PcbCircularPad::new(
            NetId(1),
            TraceLayer(0),
            p(i64::from(x), i64::from(y)),
            r(i64::from(diameter)),
        ).unwrap();

        prop_assert_eq!(
            check_circular_pad_circular_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn rect_pad_board_clearance_accepts_generated_interior_pads(
        x in 30_i16..=70,
        y in 30_i16..=70,
        width in 0_i16..=20,
        height in 0_i16..=20,
        clearance in 0_i16..=10,
    ) {
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let pad = PcbRectPad::new(
            NetId(1),
            TraceLayer(0),
            p(i64::from(x), i64::from(y)),
            r(i64::from(width)),
            r(i64::from(height)),
        ).unwrap();

        prop_assert_eq!(
            check_rect_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn rounded_rect_pad_board_clearance_accepts_generated_interior_pads(
        x in 30_i16..=70,
        y in 30_i16..=70,
        width in 0_i16..=20,
        height in 0_i16..=20,
        radius in 0_i16..=10,
        clearance in 0_i16..=10,
    ) {
        prop_assume!(i32::from(radius) * 2 <= i32::from(width));
        prop_assume!(i32::from(radius) * 2 <= i32::from(height));
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let pad = PcbRoundedRectPad::new(
            NetId(1),
            TraceLayer(0),
            p(i64::from(x), i64::from(y)),
            r(i64::from(width)),
            r(i64::from(height)),
            r(i64::from(radius)),
        ).unwrap();

        prop_assert_eq!(
            check_rounded_rect_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn rounded_rect_zero_radius_matches_rect_for_generated_trace_gaps(
        pad_y in 5_i16..=40,
        width in 0_i16..=30,
        height in 0_i16..=30,
        clearance in 0_i16..=20,
    ) {
        let trace = trace(1, 0, p(0, 0), p(40, 0), 2);
        let rect = PcbRectPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(width)),
            r(i64::from(height)),
        ).unwrap();
        let rounded = PcbRoundedRectPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(width)),
            r(i64::from(height)),
            r(0),
        ).unwrap();

        prop_assert_eq!(
            check_trace_rounded_rect_pad_clearance(
                &trace,
                &rounded,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            check_trace_rect_pad_clearance(
                &trace,
                &rect,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status
        );
    }

    #[test]
    fn degenerate_obround_matches_circular_pad_for_generated_axis_trace_gaps(
        pad_y in 5_i16..=40,
        diameter in 0_i16..=30,
        clearance in 0_i16..=20,
    ) {
        let trace = trace(1, 0, p(0, 0), p(40, 0), 2);
        let circular = PcbCircularPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(diameter)),
        ).unwrap();
        let obround = PcbObroundPad::new(
            NetId(2),
            TraceLayer(0),
            LinePathSegment::new(p(20, i64::from(pad_y)), p(20, i64::from(pad_y))),
            r(i64::from(diameter)),
        ).unwrap();

        prop_assert_eq!(
            check_trace_obround_pad_clearance(
                &trace,
                &obround,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            check_trace_pad_clearance(
                &trace,
                &circular,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status
        );
    }

    #[test]
    fn obround_pad_board_clearance_accepts_generated_interior_axis_spines(
        x0 in 30_i16..=70,
        x1 in 30_i16..=70,
        y in 30_i16..=70,
        diameter in 0_i16..=20,
        clearance in 0_i16..=10,
    ) {
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let pad = PcbObroundPad::new(
            NetId(1),
            TraceLayer(0),
            LinePathSegment::new(
                p(i64::from(x0), i64::from(y)),
                p(i64::from(x1), i64::from(y)),
            ),
            r(i64::from(diameter)),
        ).unwrap();

        prop_assert_eq!(
            check_obround_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn convex_rect_pad_matches_rect_for_generated_trace_gaps(
        pad_y in 5_i16..=40,
        width in 0_i16..=30,
        height in 0_i16..=30,
        clearance in 0_i16..=20,
    ) {
        prop_assume!(width > 0);
        prop_assume!(height > 0);
        prop_assume!(width % 2 == 0);
        prop_assume!(height % 2 == 0);
        let trace = trace(1, 0, p(0, 0), p(40, 0), 2);
        let rect = PcbRectPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(width)),
            r(i64::from(height)),
        ).unwrap();
        let half_width = i64::from(width) / 2;
        let half_height = i64::from(height) / 2;
        prop_assume!(half_width > 0);
        prop_assume!(half_height > 0);
        let convex = PcbConvexPad::new(
            NetId(2),
            TraceLayer(0),
            vec![
                p(20 - half_width, i64::from(pad_y) - half_height),
                p(20 + half_width, i64::from(pad_y) - half_height),
                p(20 + half_width, i64::from(pad_y) + half_height),
                p(20 - half_width, i64::from(pad_y) + half_height),
            ],
        ).unwrap();

        prop_assert_eq!(
            check_trace_convex_pad_clearance(
                &trace,
                &convex,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            check_trace_rect_pad_clearance(
                &trace,
                &rect,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status
        );
    }

    #[test]
    fn convex_pad_board_clearance_accepts_generated_interior_diamonds(
        x in 30_i16..=70,
        y in 30_i16..=70,
        radius in 1_i16..=20,
        clearance in 0_i16..=10,
    ) {
        let board = PcbBoardOutline::new(p(0, 0), p(100, 100)).unwrap();
        let x = i64::from(x);
        let y = i64::from(y);
        let radius = i64::from(radius);
        let pad = PcbConvexPad::new(
            NetId(1),
            TraceLayer(0),
            vec![
                p(x, y + radius),
                p(x + radius, y),
                p(x, y - radius),
                p(x - radius, y),
            ],
        ).unwrap();

        prop_assert_eq!(
            check_convex_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn oriented_rect_axis_aligned_matches_rect_for_generated_trace_gaps(
        pad_y in 5_i16..=40,
        width in 0_i16..=30,
        height in 0_i16..=30,
        clearance in 0_i16..=20,
    ) {
        let trace = trace(1, 0, p(0, 0), p(40, 0), 2);
        let rect = PcbRectPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(width)),
            r(i64::from(height)),
        ).unwrap();
        let oriented = PcbOrientedRectPad::new(
            NetId(2),
            TraceLayer(0),
            p(20, i64::from(pad_y)),
            r(i64::from(width)),
            r(i64::from(height)),
            Point2::new(r(1), r(0)),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            check_trace_oriented_rect_pad_clearance(
                &trace,
                &oriented,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            check_trace_rect_pad_clearance(
                &trace,
                &rect,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status
        );
    }

    #[test]
    fn oriented_rect_board_clearance_accepts_generated_pythagorean_interior_pads(
        x in -20_i16..=20,
        y in -20_i16..=20,
        width in 0_i16..=20,
        height in 0_i16..=20,
        clearance in 0_i16..=20,
    ) {
        let board = PcbBoardOutline::new(p(-100, -100), p(100, 100)).unwrap();
        let pad = PcbOrientedRectPad::new(
            NetId(1),
            TraceLayer(0),
            p(i64::from(x), i64::from(y)),
            r(i64::from(width)),
            r(i64::from(height)),
            Point2::new(rq(3, 5), rq(4, 5)),
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(
            check_oriented_rect_pad_board_clearance(
                &pad,
                &board,
                &r(i64::from(clearance)),
                PredicatePolicy::default(),
            ).status,
            ClearanceStatus::CertifiedClear
        );
    }

    #[test]
    fn tangent_alignment_handles_generated_scaled_vectors(
        x in -50_i16..=50,
        y in -50_i16..=50,
        a in 1_i16..=20,
        b in 1_i16..=20,
    ) {
        prop_assume!(x != 0 || y != 0);
        let base = p(i64::from(x), i64::from(y));
        let same = Point2::new(base.x.clone() * r(i64::from(a)), base.y.clone() * r(i64::from(a)));
        let opposite = Point2::new(-base.x.clone() * r(i64::from(b)), -base.y.clone() * r(i64::from(b)));

        prop_assert_eq!(
            classify_tangent_alignment(&base, &same, PredicatePolicy::default()),
            TangentAlignment::SameDirection
        );
        prop_assert_eq!(
            classify_tangent_alignment(&base, &opposite, PredicatePolicy::default()),
            TangentAlignment::OppositeDirection
        );
    }

    #[test]
    fn tangent_join_accepts_generated_same_endpoint_scaled_vectors(
        x in -50_i16..=50,
        y in -50_i16..=50,
        tx in -50_i16..=50,
        ty in -50_i16..=50,
        scale in 1_i16..=20,
    ) {
        prop_assume!(tx != 0 || ty != 0);
        let endpoint = p(i64::from(x), i64::from(y));
        let tangent = p(i64::from(tx), i64::from(ty));
        let scaled = Point2::new(
            tangent.x.clone() * r(i64::from(scale)),
            tangent.y.clone() * r(i64::from(scale)),
        );
        let report = classify_tangent_join(
            &endpoint,
            &tangent,
            &endpoint,
            &scaled,
            PredicatePolicy::default(),
        );

        prop_assert_eq!(report.class, TangentJoinClass::G1Continuous);
        prop_assert_eq!(report.endpoints_equal, Some(true));
        prop_assert_eq!(report.alignment, Some(TangentAlignment::SameDirection));
    }

    #[test]
    fn line_segment_generated_tangent_matches_endpoint_displacement(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let end = p(i64::from(x1), i64::from(y1));
        let segment = LinePathSegment::new(start.clone(), end.clone());
        let expected = Point2::new(end.x - start.x, end.y - start.y);

        prop_assert_eq!(segment.direction_vector(), expected.clone());
        prop_assert_eq!(segment.start_tangent(), expected.clone());
        prop_assert_eq!(segment.end_tangent(), expected);
    }

    #[test]
    fn line_arrangement_generated_axis_crossings_split_both_segments(
        x in -50_i16..=50,
        y in -50_i16..=50,
        horizontal_half in 1_i16..=50,
        vertical_half in 1_i16..=50,
    ) {
        let x = i64::from(x);
        let y = i64::from(y);
        let horizontal_half = i64::from(horizontal_half);
        let vertical_half = i64::from(vertical_half);
        let horizontal = LinePathSegment::new(
            p(x - horizontal_half, y),
            p(x + horizontal_half, y),
        );
        let vertical = LinePathSegment::new(
            p(x, y - vertical_half),
            p(x, y + vertical_half),
        );

        let report = arrange_line_segments(&[horizontal, vertical], PredicatePolicy::default()).unwrap();

        prop_assert_eq!(report.events[0].class, LineArrangementEventClass::ProperCrossing);
        prop_assert_eq!(report.events[0].point.as_ref().unwrap(), &p(x, y));
        prop_assert_eq!(report.breakpoints[0].len(), 3);
        prop_assert_eq!(report.breakpoints[1].len(), 3);
        prop_assert_eq!(report.fragments.len(), 4);
    }

    #[test]
    fn line_arrangement_generated_collinear_overlap_splits_union_endpoints(
        a in -100_i16..=80,
        len_a in 4_i16..=80,
        offset in 1_i16..=20,
        len_b in 2_i16..=80,
    ) {
        prop_assume!(offset < len_a);
        let a = i64::from(a);
        let len_a = i64::from(len_a);
        let offset = i64::from(offset);
        let len_b = i64::from(len_b);
        let first = LinePathSegment::new(p(a, 7), p(a + len_a, 7));
        let second = LinePathSegment::new(p(a + offset, 7), p(a + offset + len_b, 7));

        let report = arrange_line_segments(&[first, second], PredicatePolicy::default()).unwrap();

        prop_assert!(matches!(
            report.events[0].class,
            LineArrangementEventClass::CollinearOverlap | LineArrangementEventClass::EndpointTouch
        ));
        if report.events[0].class == LineArrangementEventClass::CollinearOverlap {
            let overlap = report.events[0].overlap.as_ref().unwrap();
            prop_assert_eq!(overlap.start(), &p(a + offset, 7));
            prop_assert!(report.breakpoints[0].iter().any(|breakpoint| breakpoint.point == p(a + offset, 7)));
        }
    }

    #[test]
    fn line_arc_arrangement_generated_full_circle_axis_secants_split_line(
        cx in -50_i16..=50,
        cy in -50_i16..=50,
        radius in 1_i16..=40,
        pad in 1_i16..=40,
    ) {
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let radius = i64::from(radius);
        let pad = i64::from(pad);
        let line = LinePathSegment::new(
            p(cx - radius - pad, cy),
            p(cx + radius + pad, cy),
        );
        let arc = ExplicitCircularArc::new(
            p(cx, cy),
            r(radius),
            p(cx + radius, cy),
            p(cx + radius, cy),
            ArcDirection::Ccw,
        )
        .unwrap();

        let report = arrange_line_segments_with_explicit_arcs(&[line], &[arc], PredicatePolicy::default()).unwrap();

        prop_assert_eq!(report.events[0].class, LineArcArrangementEventClass::Secant);
        prop_assert_eq!(report.events[0].points.len(), 2);
        prop_assert_eq!(report.line_breakpoints[0].len(), 4);
        prop_assert_eq!(report.line_fragments.len(), 3);
        prop_assert!(report.line_breakpoints[0].iter().any(|breakpoint| breakpoint.point == p(cx - radius, cy)));
        prop_assert!(report.line_breakpoints[0].iter().any(|breakpoint| breakpoint.point == p(cx + radius, cy)));
    }

    #[test]
    fn quadratic_bezier_arrangement_generated_split_preserves_endpoints(
        x0 in -50_i16..=50,
        y0 in -50_i16..=50,
        x1 in -50_i16..=50,
        y1 in -50_i16..=50,
        x2 in -50_i16..=50,
        y2 in -50_i16..=50,
        numerator in 1_i64..=9,
    ) {
        let parameter = BezierParameter::new(numerator, 10).unwrap();
        let curve = QuadraticBezier::new(
            p(i64::from(x0), i64::from(y0)),
            p(i64::from(x1), i64::from(y1)),
            p(i64::from(x2), i64::from(y2)),
        );
        let report = arrange_quadratic_beziers(&[curve.clone()], &[vec![parameter]], PredicatePolicy::default()).unwrap();

        prop_assert_eq!(report.fragments.len(), 2);
        prop_assert_eq!(report.fragments[0].curve.start(), curve.start());
        prop_assert_eq!(report.fragments[0].curve.end(), &curve.eval(parameter));
        prop_assert_eq!(report.fragments[1].curve.start(), &curve.eval(parameter));
        prop_assert_eq!(report.fragments[1].curve.end(), curve.end());
    }

    #[test]
    fn line_quadratic_bezier_generated_endpoint_events_survive_scale(
        x0 in -20_i16..=20,
        width in 1_i16..=40,
        lift in 1_i16..=40,
    ) {
        let start_x = i64::from(x0);
        let width = i64::from(width);
        let lift = i64::from(lift);
        let curve = QuadraticBezier::new(
            p(start_x, 0),
            p(start_x + width, lift),
            p(start_x + 2 * width, 0),
        );
        let line = LinePathSegment::new(p(start_x, 0), p(start_x + 2 * width, 0));

        let report =
            intersect_axis_aligned_line_quadratic_bezier(&line, &curve, PredicatePolicy::default());

        prop_assert_eq!(report.class, LineQuadraticBezierIntersectionClass::TwoPoints);
        prop_assert_eq!(report.intersections.len(), 2);
        prop_assert_eq!(report.intersections[0].parameter.clone(), r(0));
        prop_assert_eq!(report.intersections[0].point.clone(), curve.start().clone());
        prop_assert_eq!(report.intersections[1].parameter.clone(), r(1));
        prop_assert_eq!(report.intersections[1].point.clone(), curve.end().clone());
    }

    #[test]
    fn line_quadratic_bezier_arrangement_generated_tangencies_split_once(
        x0 in -20_i16..=20,
        width in 1_i16..=40,
        lift in 1_i16..=40,
    ) {
        let start_x = i64::from(x0);
        let width = i64::from(width);
        let lift = i64::from(lift);
        let curve = QuadraticBezier::new(
            p(start_x, 0),
            p(start_x + width, lift),
            p(start_x + 2 * width, 0),
        );
        let line = LinePathSegment::new(
            pq(start_x * 2, 2, lift, 2),
            pq((start_x + 2 * width) * 2, 2, lift, 2),
        );

        let report = arrange_line_segments_with_quadratic_beziers(
            &[line],
            &[curve],
            PredicatePolicy::default(),
        ).unwrap();

        prop_assert_eq!(report.events[0].class, LineQuadraticBezierIntersectionClass::Tangent);
        prop_assert_eq!(report.line_breakpoints[0].len(), 3);
        prop_assert_eq!(report.bezier_breakpoints[0].len(), 3);
        prop_assert_eq!(report.bezier_breakpoints[0][1].parameter.clone(), rq(1, 2));
        prop_assert_eq!(report.bezier_fragments.len(), 2);
        prop_assert_eq!(report.bezier_fragments[0].curve.end(), report.bezier_fragments[1].curve.start());
    }

    #[test]
    fn cubic_bezier_arrangement_generated_split_preserves_endpoints(
        x0 in -20_i16..=20,
        y0 in -20_i16..=20,
        x1 in -20_i16..=20,
        y1 in -20_i16..=20,
        x2 in -20_i16..=20,
        y2 in -20_i16..=20,
        x3 in -20_i16..=20,
        y3 in -20_i16..=20,
        numerator in 1_i64..=9,
    ) {
        let parameter = BezierParameter::new(numerator, 10).unwrap();
        let curve = CubicBezier::new(
            p(i64::from(x0), i64::from(y0)),
            p(i64::from(x1), i64::from(y1)),
            p(i64::from(x2), i64::from(y2)),
            p(i64::from(x3), i64::from(y3)),
        );
        let report = arrange_cubic_beziers(&[curve.clone()], &[vec![parameter]], PredicatePolicy::default()).unwrap();

        prop_assert_eq!(report.fragments.len(), 2);
        prop_assert_eq!(report.fragments[0].curve.start(), curve.start());
        prop_assert_eq!(report.fragments[0].curve.end(), &curve.eval(parameter));
        prop_assert_eq!(report.fragments[1].curve.start(), &curve.eval(parameter));
        prop_assert_eq!(report.fragments[1].curve.end(), curve.end());
    }

    #[test]
    fn tangent_span_from_generated_line_retains_exact_displacement(
        x0 in -100_i16..=100,
        y0 in -100_i16..=100,
        x1 in -100_i16..=100,
        y1 in -100_i16..=100,
    ) {
        let start = p(i64::from(x0), i64::from(y0));
        let end = p(i64::from(x1), i64::from(y1));
        let segment = LinePathSegment::new(start.clone(), end.clone());
        let span = TangentSpan::from_line_segment(&segment);
        let expected = Point2::new(end.x.clone() - start.x.clone(), end.y.clone() - start.y.clone());

        prop_assert_eq!(span.start, start);
        prop_assert_eq!(span.start_tangent, expected.clone());
        prop_assert_eq!(span.end, end);
        prop_assert_eq!(span.end_tangent, expected);
    }

    #[test]
    fn tangent_alignment_problem_certifies_generated_parallel_vectors(
        x in -20_i16..=20,
        y in -20_i16..=20,
        scale in 1_i16..=20,
    ) {
        prop_assume!(x != 0 || y != 0);
        let candidate = p(i64::from(x), i64::from(y));
        let target = Point2::new(
            candidate.x.clone() * r(i64::from(scale)),
            candidate.y.clone() * r(i64::from(scale)),
        );
        let model = build_tangent_alignment_problem(candidate, target);

        prop_assert!(certify_tangent_alignment_candidate(&model).all_satisfied());
    }

    #[test]
    fn oriented_tangent_alignment_problem_rejects_generated_opposites(
        x in -20_i16..=20,
        y in -20_i16..=20,
        scale in 1_i16..=20,
    ) {
        prop_assume!(x != 0 || y != 0);
        let candidate = p(i64::from(x), i64::from(y));
        let target = Point2::new(
            -candidate.x.clone() * r(i64::from(scale)),
            -candidate.y.clone() * r(i64::from(scale)),
        );
        let model = build_oriented_tangent_alignment_problem(candidate, target);

        prop_assert!(certify_tangent_alignment_candidate(&model).has_certified_violation());
    }

    #[test]
    fn g1_join_problem_certifies_generated_same_endpoint_scaled_tangent(
        x in -20_i16..=20,
        y in -20_i16..=20,
        tx in -20_i16..=20,
        ty in -20_i16..=20,
        scale in 1_i16..=20,
    ) {
        prop_assume!(tx != 0 || ty != 0);
        let endpoint = p(i64::from(x), i64::from(y));
        let tangent = p(i64::from(tx), i64::from(ty));
        let target_tangent = Point2::new(
            tangent.x.clone() * r(i64::from(scale)),
            tangent.y.clone() * r(i64::from(scale)),
        );
        let model = build_g1_join_problem(endpoint.clone(), tangent, endpoint, target_tangent);

        prop_assert!(certify_g1_join_candidate(&model).all_satisfied());
    }

    #[test]
    fn tangent_chain_accepts_generated_two_span_g1_join(
        x in -20_i16..=20,
        y in -20_i16..=20,
        tx in -20_i16..=20,
        ty in -20_i16..=20,
        scale in 1_i16..=20,
    ) {
        prop_assume!(tx != 0 || ty != 0);
        let endpoint = p(i64::from(x), i64::from(y));
        let tangent = p(i64::from(tx), i64::from(ty));
        let scaled = Point2::new(
            tangent.x.clone() * r(i64::from(scale)),
            tangent.y.clone() * r(i64::from(scale)),
        );
        let spans = vec![
            TangentSpan {
                start: p(0, 0),
                start_tangent: tangent.clone(),
                end: endpoint.clone(),
                end_tangent: tangent.clone(),
            },
            TangentSpan {
                start: endpoint,
                start_tangent: scaled.clone(),
                end: p(1, 1),
                end_tangent: scaled,
            },
        ];
        let report = classify_tangent_chain(&spans, PredicatePolicy::default());

        prop_assert!(report.all_g1_continuous());
        prop_assert_eq!(report.first_non_g1_join(), None);
    }

    #[test]
    fn g1_chain_certifies_generated_two_span_g1_join(
        x in -20_i16..=20,
        y in -20_i16..=20,
        tx in -20_i16..=20,
        ty in -20_i16..=20,
        scale in 1_i16..=20,
    ) {
        prop_assume!(tx != 0 || ty != 0);
        let endpoint = p(i64::from(x), i64::from(y));
        let tangent = p(i64::from(tx), i64::from(ty));
        let scaled = Point2::new(
            tangent.x.clone() * r(i64::from(scale)),
            tangent.y.clone() * r(i64::from(scale)),
        );
        let spans = vec![
            TangentSpan {
                start: p(0, 0),
                start_tangent: tangent.clone(),
                end: endpoint.clone(),
                end_tangent: tangent,
            },
            TangentSpan {
                start: endpoint,
                start_tangent: scaled.clone(),
                end: p(1, 1),
                end_tangent: scaled,
            },
        ];
        let report = certify_g1_chain(&spans);

        prop_assert!(report.all_certified());
        prop_assert_eq!(report.first_uncertified_join(), None);
    }
}
