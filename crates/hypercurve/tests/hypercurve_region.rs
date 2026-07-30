use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    FillRule, FiniteProjectionOptions, LineArcRegion2, Real, RegionArrangement2,
    RegionBoundaryContourRole2, RegionLineSegmentRegionBuildStage2, RegionPointLocation,
    RegionView2, Segment2, SegmentKind, SegmentKindCounts, UncertaintyReason,
    finite_polyline_vertex_centroid, finite_ring_signed_area, try_finite_polyline_vertex_centroid,
    try_finite_ring_signed_area,
};
use proptest::prelude::*;

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(0))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmax, ymin),
        vertex(xmax, ymax),
        vertex(xmin, ymax),
    ])
    .unwrap()
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> hypercurve::LineSeg2 {
    hypercurve::LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap()
}

fn arc_bulge(start_x: i32, start_y: i32, end_x: i32, end_y: i32, bulge: i32) -> CircularArc2 {
    CircularArc2::from_bulge(p(start_x, start_y), p(end_x, end_y), s(bulge)).unwrap()
}

fn reversed_segment(segment: &Segment2) -> Segment2 {
    segment.reversed()
}

fn reversed_rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmin, ymax),
        vertex(xmax, ymax),
        vertex(xmax, ymin),
    ])
    .unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::STRICT
}

fn evaluate_unordered_line_segments(
    segments: Vec<hypercurve::LineSeg2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<RegionArrangement2> {
    LineArcRegion2::arrange_unordered_line_segments(segments, fill_rule, policy)
}

fn evaluate_borrowed_unordered_line_segments(
    segments: &[hypercurve::LineSeg2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<RegionArrangement2> {
    LineArcRegion2::arrange_unordered_line_segments_borrowed(segments, fill_rule, policy)
}

fn evaluate_unordered_segments(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<RegionArrangement2> {
    LineArcRegion2::arrange_unordered_segments(segments, fill_rule, policy)
}

fn evaluate_borrowed_unordered_segments(
    segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<RegionArrangement2> {
    LineArcRegion2::arrange_unordered_segments_borrowed(segments, fill_rule, policy)
}

#[test]
fn empty_region_classifies_everything_outside() {
    let region = LineArcRegion2::empty();

    assert!(region.is_empty());
    assert_eq!(
        region.signed_depth(&p(0, 0), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(0, 0), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn material_contour_classifies_inside_outside_and_boundary() {
    let region = LineArcRegion2::from_material_contours(vec![rectangle(0, 0, 10, 10)]);

    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(11, 1), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        region.classify_point(&p(10, 5), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn region_aabb_miss_has_zero_depth_without_boundary_work() {
    let region = LineArcRegion2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 20, 30, 30)],
        vec![rectangle(3, 3, 7, 7)],
    );

    assert_eq!(
        region.signed_depth(&p(100, 100), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(100, 100), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn sparse_region_classification_keeps_only_relevant_contour_depth() {
    let region = LineArcRegion2::from_material_contours(vec![
        rectangle(0, 0, 4, 4),
        rectangle(20, 20, 24, 24),
        rectangle(40, 40, 44, 44),
    ]);

    assert_eq!(
        region.signed_depth(&p(21, 21), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(21, 21), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(20, 22), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn hole_bin_subtracts_from_material_depth() {
    let region = LineArcRegion2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

    assert_eq!(
        region.signed_depth(&p(1, 1), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.signed_depth(&p(5, 5), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn boundary_contour_nesting_assigns_disjoint_nested_roles() {
    let region = match LineArcRegion2::from_boundary_contours(
        vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        &policy(),
    )
    .unwrap()
    {
        Classification::Decided(region) => region,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    };

    assert_eq!(region.material_contours().len(), 1);
    assert_eq!(region.hole_contours().len(), 1);
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}
#[test]
fn borrowed_boundary_contours_convenience_returns_decided_region() {
    let contours = vec![rectangle(0, 0, 5, 5), rectangle(1, 1, 3, 3)];
    let region =
        match LineArcRegion2::from_boundary_contours_borrowed(&contours, &policy()).unwrap() {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
        };

    assert_eq!(contours.len(), 2);
    assert_eq!(region.material_contours().len(), 1);
    assert_eq!(region.hole_contours().len(), 1);
    assert_eq!(
        region.classify_point(&p(4, 4), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(2, 2), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn boundary_contour_nesting_rejects_crossing_or_touching_loops() {
    assert_eq!(
        LineArcRegion2::from_boundary_contours(
            vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        LineArcRegion2::from_boundary_contours(
            vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn unordered_line_segments_build_region_with_source_provenance() {
    let built = evaluate_unordered_line_segments(
        vec![
            line(0, 0, 4, 0),
            line(0, 4, 4, 4),
            line(0, 0, 0, 4),
            line(4, 0, 4, 4),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert!(built.status().unwrap().is_native_exact());
    assert!(built.region().is_some());
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(built.split_intersection_event_count(), Some(4));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(built.output_ring_count(), Some(1));
    assert_eq!(built.output_boundary_segment_count(), Some(4));
    let sources = built.source_evidence().unwrap();
    assert_eq!(sources.len(), 4);
    assert_eq!(sources[0].source_segment_index(), 0);
}

#[test]
fn unordered_line_segments_retain_disconnected_boundary_blocker() {
    let built = evaluate_unordered_line_segments(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert!(built.status().unwrap().is_retained_evidence());
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(1));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_line_segments_split_crossings_before_boundary_blocker() {
    let built = evaluate_unordered_line_segments(
        vec![line(0, 0, 4, 4), line(0, 4, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_intersection_points().unwrap(), &[p(2, 2)]);
    let events = built.split_intersection_evidence().unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].first_source_param(), &q(1, 2));
    assert_eq!(events[0].second_source_param(), &q(1, 2));
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_line_segments_retain_overlap_source_pair_blocker() {
    let built = evaluate_unordered_line_segments(
        vec![line(0, 0, 4, 0), line(2, 0, 6, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert_eq!(built.split_overlap_relation_count(), Some(1));
    assert_eq!(built.split_blocker_first_source_segment_index(), Some(0));
    assert_eq!(built.split_blocker_second_source_segment_index(), Some(1));
    assert_eq!(
        built.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn borrowed_unordered_line_segments_evaluate_retained_arrangement() {
    let segments = vec![
        line(0, 0, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
        line(4, 0, 4, 4),
    ];

    let built =
        evaluate_borrowed_unordered_line_segments(&segments, FillRule::NonZero, &policy()).unwrap();

    assert!(built.region().is_some());
    assert_eq!(segments.len(), 4);
    assert!(built.status().unwrap().is_native_exact());
    assert_eq!(built.source_segment_count(), 4);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(built.arranged_source_evidence_count(), Some(4));
    assert_eq!(built.source_evidence_count(), Some(4));
}
#[test]
fn unordered_segments_return_retained_evidence() {
    let lines = vec![
        line(0, 0, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
        line(4, 0, 4, 4),
    ];
    let result =
        evaluate_borrowed_unordered_line_segments(&lines, FillRule::NonZero, &policy()).unwrap();
    let classification = result.region_classification();
    let evidence = &result;
    assert!(evidence.status().unwrap().is_native_exact());
    assert_eq!(evidence.source_segment_count(), 4);
    assert!(evidence.source_line_segments().is_some());
    match classification {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected decided line region, got {reason:?}"),
    }

    let result =
        evaluate_unordered_line_segments(lines.clone(), FillRule::NonZero, &policy()).unwrap();
    let classification = result.region_classification();
    let evidence = &result;
    assert!(evidence.status().unwrap().is_native_exact());
    assert_eq!(evidence.source_segment_count(), 4);
    assert!(matches!(classification, Classification::Decided(_)));

    let disconnected = vec![line(0, 0, 1, 0), line(3, 0, 4, 0)];
    let result =
        evaluate_borrowed_unordered_line_segments(&disconnected, FillRule::NonZero, &policy())
            .unwrap();
    let classification = result.region_classification();
    let evidence = &result;
    assert_eq!(
        classification,
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(evidence.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(evidence.materialized_region(), Some(false));

    let native_segments = lines.into_iter().map(Segment2::Line).collect::<Vec<_>>();
    let result =
        evaluate_borrowed_unordered_segments(&native_segments, FillRule::NonZero, &policy())
            .unwrap();
    let classification = result.region_classification();
    let evidence = &result;
    assert!(evidence.status().unwrap().is_native_exact());
    assert_eq!(evidence.source_segment_count(), 4);
    assert!(evidence.source_line_segments().is_none());
    assert!(matches!(classification, Classification::Decided(_)));

    let result =
        evaluate_unordered_segments(native_segments, FillRule::NonZero, &policy()).unwrap();
    let classification = result.region_classification();
    let evidence = &result;
    assert!(evidence.status().unwrap().is_native_exact());
    assert!(matches!(classification, Classification::Decided(_)));
}
#[test]
fn unordered_native_segments_build_line_arc_region_with_source_provenance() {
    let built = evaluate_unordered_segments(
        vec![
            Segment2::Line(line(4, 0, 0, 0)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.status().unwrap().is_native_exact());
    assert!(built.region().is_some());
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(built.split_intersection_event_count(), Some(2));
    assert_eq!(
        built.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    let arranged = built.arranged_source_evidence().unwrap();
    assert_eq!(arranged.len(), 2);
    assert_eq!(arranged[0].source_segment_kind(), SegmentKind::Line);
    assert_eq!(arranged[1].source_segment_kind(), SegmentKind::Arc);
}

#[test]
fn borrowed_unordered_native_segments_evaluate_retained_arrangement() {
    let segments = vec![
        Segment2::Line(line(4, 0, 0, 0)),
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
    ];

    let built =
        evaluate_borrowed_unordered_segments(&segments, FillRule::NonZero, &policy()).unwrap();

    assert!(built.region().is_some());
    assert_eq!(segments.len(), 2);
    assert!(built.status().unwrap().is_native_exact());
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(built.arranged_segment_count(), Some(2));
    assert_eq!(built.arranged_source_evidence_count(), Some(2));
    assert_eq!(built.source_evidence_count(), Some(2));
}

#[test]
fn region_arrangement_builds_line_region_with_immediate_evidence() {
    let lines = vec![
        line(4, 0, 0, 0),
        line(4, 4, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
    ];
    let result = LineArcRegion2::arrange_unordered_line_segments_borrowed(
        &lines,
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert!(result.region().is_some());
    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.source_line_segments(), Some(lines.as_slice()));
    assert_eq!(
        result.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(result.source_endpoint_count(), 8);
    assert_eq!(result.split_schedule_candidate_pair_count(), 6);
    assert_eq!(result.split_candidate_pair_count(), Some(6));
    assert_eq!(result.split_intersection_event_count(), Some(4));
    assert_eq!(result.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(result.output_ring_count(), Some(1));
    assert_eq!(result.output_boundary_segment_count(), Some(4));
    assert_eq!(result.output_contour_count(), Some(1));
}

#[test]
fn region_arrangement_retains_output_role_containment() {
    let lines = vec![
        line(0, 0, 10, 0),
        line(10, 0, 10, 10),
        line(10, 10, 0, 10),
        line(0, 10, 0, 0),
        line(3, 3, 7, 3),
        line(7, 3, 7, 7),
        line(7, 7, 3, 7),
        line(3, 7, 3, 3),
    ];
    let result = LineArcRegion2::arrange_unordered_line_segments_borrowed(
        &lines,
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.output_ring_count(), Some(2));
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(1));
    assert_eq!(result.role_evidence_count(), Some(2));
    assert_eq!(result.role_containment_bucket_count(), Some(1));
    assert_eq!(result.role_containment_ref_count(), Some(1));
    assert_eq!(result.role_uncontained_assignment_ref_count(), Some(1));
    let roles = result.role_evidence().unwrap();
    assert_eq!(roles[0].role(), RegionBoundaryContourRole2::Material);
    assert_eq!(roles[1].role(), RegionBoundaryContourRole2::Hole);
}
#[test]
fn region_arrangement_retains_overlap_blocker() {
    let segments = vec![
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
    ];
    let result =
        LineArcRegion2::arrange_unordered_segments(segments, FillRule::NonZero, &policy()).unwrap();

    assert!(result.region().is_none());
    assert!(result.evaluated_output());
    assert_eq!(result.materialized_region(), Some(false));
    assert_eq!(
        result.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(result.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(result.split_overlap_relation_count(), Some(1));
    assert_eq!(result.endpoint_graph_endpoint_count(), None);
    assert_eq!(result.output_ring_count(), None);
}

#[test]
fn unordered_native_segments_return_decided_region() {
    let result = LineArcRegion2::arrange_unordered_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(4, 0, 0, 0)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    let Some(region) = result.region() else {
        panic!("line-arc native region should materialize");
    };
    assert_eq!(
        region.classify_point(&p(2, -1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unordered_native_segments_retain_arc_overlap_boundary_blocker() {
    let built = evaluate_unordered_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 2 }
    );
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(built.split_overlap_relation_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(0));
    assert_eq!(
        built.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        built.split_blocker_second_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(built.endpoint_graph_endpoint_count(), None);
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_native_segments_split_line_arc_crossing_before_boundary_blocker() {
    let built = evaluate_unordered_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(2, -3, 2, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_intersection_points().unwrap(), &[p(2, -2)]);
    let events = built.split_intersection_evidence().unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].first_source_segment_kind(), SegmentKind::Arc);
    assert_eq!(events[0].second_source_segment_kind(), SegmentKind::Line);
    assert_eq!(events[0].point(), &p(2, -2));
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_native_segments_split_arc_arc_crossing_before_boundary_blocker() {
    let built = evaluate_unordered_segments(
        vec![
            Segment2::Arc(
                CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap(),
            ),
            Segment2::Arc(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap()),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(built.region().is_none());
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_intersection_points().unwrap(), &[p(4, 3)]);
    let events = built.split_intersection_evidence().unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].first_source_segment_kind(), SegmentKind::Arc);
    assert_eq!(events[0].second_source_segment_kind(), SegmentKind::Arc);
    assert_eq!(events[0].point(), &p(4, 3));
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn hole_boundary_is_explicit() {
    let region = LineArcRegion2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

    assert_eq!(
        region.signed_depth(&p(3, 5), &policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        region.classify_point(&p(3, 5), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn material_island_inside_hole_adds_depth_back() {
    let region = LineArcRegion2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );

    assert_eq!(
        region.signed_depth(&p(1, 1), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.signed_depth(&p(3, 3), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        region.classify_point(&p(3, 3), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        region.signed_depth(&p(5, 5), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn contour_profiles_group_holes_with_containing_material() {
    let left = rectangle(0, 0, 10, 10);
    let right = rectangle(20, 0, 30, 10);
    let left_hole = rectangle(2, 2, 4, 4);
    let right_hole = rectangle(22, 2, 24, 4);
    let region = LineArcRegion2::new(
        vec![left.clone(), right.clone()],
        vec![left_hole.clone(), right_hole.clone()],
    );

    let profiles = region.contour_profiles(&policy());
    let Classification::Decided(profiles) = profiles else {
        panic!("profile ownership should be decided: {profiles:?}");
    };

    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes.len() == 1));
    assert_eq!(profiles[0].material, &left);
    assert_eq!(profiles[0].holes[0], &left_hole);
    assert_eq!(profiles[1].material, &right);
    assert_eq!(profiles[1].holes[0], &right_hole);
}

#[test]
fn contour_profiles_reject_holes_without_material_owner() {
    let region = LineArcRegion2::new(Vec::new(), vec![rectangle(2, 2, 4, 4)]);

    assert_eq!(
        region.contour_profiles(&policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn contour_projection_closes_finite_ring_without_owning_topology() {
    let contour = rectangle(0, 0, 10, 10);
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();

    let ring = contour.project_to_finite_ring(&options).unwrap();

    assert!(ring.is_closed());
    assert_eq!(ring.arc_chord_error(), 0.01);
    assert_eq!(ring.points().first(), ring.points().last());
    assert_eq!(ring.points().len(), 5);
    assert_eq!(ring.signed_ring_area(), 100.0);
    assert_eq!(ring.try_signed_ring_area().unwrap(), 100.0);
    assert_eq!(finite_ring_signed_area(ring.points()), 100.0);
    assert_eq!(ring.vertex_centroid(), Some([5.0, 5.0]));
    assert_eq!(ring.try_vertex_centroid().unwrap(), Some([5.0, 5.0]));
}

#[test]
fn finite_projection_checked_measurements_reject_nonfinite_or_overflow() {
    assert_eq!(
        try_finite_ring_signed_area(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert_eq!(
        try_finite_polyline_vertex_centroid(&[[0.0, 0.0], [f64::INFINITY, 1.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert_eq!(
        try_finite_ring_signed_area(&[[1.0e308, 0.0], [0.0, 1.0e308], [0.0, 0.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert_eq!(
        try_finite_polyline_vertex_centroid(&[[1.0e308, 0.0], [1.0e308, 0.0], [0.0, 0.0]])
            .unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );

    assert!(finite_ring_signed_area(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]).is_nan());
    assert!(finite_polyline_vertex_centroid(&[[0.0, 0.0], [f64::INFINITY, 1.0]]).is_some());
}

#[test]
fn curve_string_projection_subdivides_arc_and_keeps_exact_endpoints() {
    use hypercurve::{LineSeg2, Point2};

    let start = Point2::new(Real::from(1_i8), Real::from(0_i8));
    let end = Point2::new(Real::from(-1_i8), Real::from(0_i8));
    let center = Point2::new(Real::from(0_i8), Real::from(0_i8));
    let arc = CircularArc2::try_from_center(start, end.clone(), center, false).unwrap();
    let tail = LineSeg2::try_new(end, Point2::new(Real::from(-2_i8), Real::from(0_i8))).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc), Segment2::Line(tail)]).unwrap();

    let polyline = curve
        .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.05).unwrap())
        .unwrap();

    assert!(!polyline.is_closed());
    assert!(polyline.points().len() > 3);
    assert_eq!(polyline.arc_chord_error(), 0.05);
    assert_eq!(polyline.points().first(), Some(&[1.0, 0.0]));
    assert_eq!(polyline.points().last(), Some(&[-2.0, 0.0]));
}

#[test]
fn curve_string_projection_rejects_nonfinite_arc_samples() {
    use hypercurve::Point2;

    let huge = Real::try_from(1.1e308).unwrap();
    let start = Point2::new(Real::zero(), Real::zero());
    let end = Point2::new(huge.clone(), huge.clone());
    let center = Point2::new(huge, Real::zero());
    let arc = CircularArc2::try_from_center(start, end, center, false).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();

    assert_eq!(
        curve
            .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.01).unwrap())
            .unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
}
#[test]
fn region_projection_preserves_material_hole_bins() {
    let outer = rectangle(0, 0, 10, 10);
    let island = rectangle(4, 4, 6, 6);
    let hole = rectangle(2, 2, 8, 8);
    let region = LineArcRegion2::new(vec![outer.clone(), island.clone()], vec![hole.clone()]);
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();

    let projection = region.project_to_finite_region(&options).unwrap();

    assert_eq!(projection.material_rings().len(), 2);
    assert_eq!(projection.hole_rings().len(), 1);
    assert!(
        projection
            .material_rings()
            .iter()
            .chain(projection.hole_rings())
            .all(|ring| ring.is_closed())
    );

    let material = [outer, island];
    let holes = [hole];
    let view = RegionView2::new(&material, &holes);
    let view_projection = view.project_to_finite_region(&options).unwrap();
    assert_eq!(view_projection, projection);
}

#[test]
fn finite_profile_projection_preserves_exact_hole_ownership() {
    let left = rectangle(0, 0, 10, 10);
    let right = rectangle(20, 0, 30, 10);
    let left_hole = rectangle(2, 2, 4, 4);
    let right_hole = rectangle(22, 2, 24, 4);
    let region = LineArcRegion2::new(vec![left, right], vec![left_hole, right_hole]);
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();

    let profiles = region
        .project_to_finite_profiles(&options, &policy())
        .unwrap();
    let Classification::Decided(profiles) = profiles else {
        panic!("finite profile ownership should be decided: {profiles:?}");
    };

    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().len() == 1));
    assert_eq!(profiles[0].material().points()[0], [0.0, 0.0]);
    assert_eq!(profiles[0].holes()[0].points()[0], [2.0, 2.0]);
    assert_eq!(profiles[1].material().points()[0], [20.0, 0.0]);
    assert_eq!(profiles[1].holes()[0].points()[0], [22.0, 2.0]);
    assert_eq!(profiles[0].projected_filled_area(), 96.0);
    assert_eq!(profiles[1].projected_filled_area(), 96.0);
    assert_eq!(profiles[0].try_projected_filled_area().unwrap(), 96.0);
    assert_eq!(profiles[1].try_projected_filled_area().unwrap(), 96.0);
}

#[test]
fn finite_profile_projection_keeps_orphan_hole_uncertainty() {
    let region = LineArcRegion2::new(Vec::new(), vec![rectangle(2, 2, 4, 4)]);
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();

    assert_eq!(
        region
            .project_to_finite_profiles(&options, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn similarity_transform_preserves_arc_segment_without_flattening() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();
    let transform =
        hypercurve::Similarity2::try_from_f64_affine(0.0, -1.0, 1.0, 0.0, 3.0, -2.0, 1e-9).unwrap();

    let transformed = curve.transform_similarity(&transform).unwrap();

    let [Segment2::Arc(transformed_arc)] = transformed.segments() else {
        panic!("similarity transform should preserve arc segment type");
    };
    assert_eq!(
        transformed_arc.start(),
        &hypercurve::Point2::from_values(3, -1)
    );
    assert_eq!(
        transformed_arc.end(),
        &hypercurve::Point2::from_values(2, -2)
    );
    assert_eq!(
        transformed_arc.center(),
        &hypercurve::Point2::from_values(3, -2)
    );
    assert!(!transformed_arc.is_clockwise());
}

#[test]
fn similarity_reflection_flips_arc_orientation_and_rejects_shear() {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(1, 0), Real::from(1_i8)),
        BulgeVertex2::new(p(-1, 0), Real::zero()),
    ])
    .unwrap();
    let reflection =
        hypercurve::Similarity2::try_from_f64_affine(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1e-9).unwrap();

    let transformed = contour.transform_similarity(&reflection).unwrap();
    let Segment2::Arc(arc) = &transformed.segments()[0] else {
        panic!("reflected bulge contour should retain arc segment");
    };
    assert!(arc.is_clockwise());
    assert!(reflection.reverses_orientation());

    assert_eq!(
        hypercurve::Similarity2::try_from_f64_affine(1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1e-9),
        Err(CurveError::InvalidSimilarityTransform)
    );
}

#[test]
fn region_filled_area_uses_roles_instead_of_contour_orientation() {
    let outer = reversed_rectangle(0, 0, 10, 10);
    let hole = rectangle(3, 3, 7, 7);
    let region = LineArcRegion2::new(vec![outer.clone()], vec![hole.clone()]);

    assert_eq!(
        region.filled_area(&policy()).unwrap(),
        Classification::Decided(Some(Real::from(84_i8)))
    );

    let material = [outer];
    let holes = [hole];
    let view = RegionView2::new(&material, &holes);
    assert_eq!(
        view.filled_area(&policy()).unwrap(),
        Classification::Decided(Some(Real::from(84_i8)))
    );
}

#[test]
fn region_filled_area_counts_nested_material_back_into_holes() {
    let region = LineArcRegion2::new(
        vec![rectangle(0, 0, 10, 10), reversed_rectangle(4, 4, 6, 6)],
        vec![reversed_rectangle(2, 2, 8, 8)],
    );

    assert_eq!(
        region.filled_area(&policy()).unwrap(),
        Classification::Decided(Some(Real::from(68_i8)))
    );
}

#[test]
fn region_filled_area_is_exact_for_center_defined_circle() {
    let top = CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let bottom = CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap();
    let contour = Contour2::try_new(vec![Segment2::Arc(top), Segment2::Arc(bottom)]).unwrap();
    let region = LineArcRegion2::from_material_contours(vec![contour]);

    assert_eq!(
        region.filled_area(&policy()).unwrap(),
        Classification::Decided(Some(Real::pi()))
    );
}

#[test]
fn borrowed_region_view_matches_owned_region() {
    let outer = rectangle(0, 0, 10, 10);
    let island = rectangle(4, 4, 6, 6);
    let hole = rectangle(2, 2, 8, 8);
    let material = [outer.clone(), island.clone()];
    let holes = [hole.clone()];
    let view = RegionView2::new(&material, &holes);
    let owned = LineArcRegion2::new(vec![outer, island], vec![hole]);

    assert_eq!(view.material_contours().len(), 2);
    assert_eq!(view.hole_contours().len(), 1);
    for point in [p(1, 1), p(3, 3), p(5, 5), p(11, 1)] {
        assert_eq!(
            view.classify_point(&point, &policy()),
            owned.classify_point(&point, &policy())
        );
        assert_eq!(
            view.signed_depth(&point, &policy()),
            owned.signed_depth(&point, &policy())
        );
    }
}

proptest! {
    #[test]
    fn generated_unordered_line_rectangles_build_native_regions(
        xmin in -50_i32..50,
        ymin in -50_i32..50,
        width in 2_i32..80,
        height in 2_i32..80,
        order_variant in 0_usize..4,
        reverse_mask in 0_u8..16,
    ) {
        let xmax = xmin + width;
        let ymax = ymin + height;
        let mut lines = vec![
            line(xmin, ymin, xmax, ymin),
            line(xmax, ymin, xmax, ymax),
            line(xmax, ymax, xmin, ymax),
            line(xmin, ymax, xmin, ymin),
        ];
        for (index, line) in lines.iter_mut().enumerate() {
            if reverse_mask & (1 << index) != 0 {
                *line = line.reversed();
            }
        }
        match order_variant {
            0 => {}
            1 => lines.swap(0, 2),
            2 => lines.rotate_left(1),
            _ => lines.reverse(),
        }

        let built = evaluate_unordered_line_segments(
            lines,
            FillRule::NonZero,
            &policy(),
        ).unwrap();

        prop_assert!(built.status().unwrap().is_native_exact());
        prop_assert_eq!(built.source_segment_count(), 4);
        prop_assert_eq!(built.arranged_segment_count(), Some(4));
        prop_assert_eq!(built.split_candidate_pair_count(), Some(6));
        prop_assert_eq!(built.split_skipped_aabb_pair_count(), Some(2));
        prop_assert_eq!(built.split_tested_pair_count(), Some(4));
        prop_assert_eq!(built.split_intersection_event_count(), Some(4));
        let split_points = built.split_intersection_points().unwrap();
        prop_assert_eq!(split_points.len(), 4);
        prop_assert!(split_points.contains(&p(xmin, ymin)));
        prop_assert!(split_points.contains(&p(xmax, ymin)));
        prop_assert!(split_points.contains(&p(xmax, ymax)));
        prop_assert!(split_points.contains(&p(xmin, ymax)));
        prop_assert_eq!(built.endpoint_graph_endpoint_count(), Some(8));
        prop_assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(4));
        prop_assert_eq!(
            built.endpoint_graph_structural_singleton_bucket_count(),
            Some(0)
        );
        prop_assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(2));
        prop_assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(0));
        prop_assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(0));
        prop_assert_eq!(built.output_ring_count(), Some(1));
        prop_assert_eq!(built.output_boundary_segment_count(), Some(4));
        prop_assert_eq!(built.blocker(), None);

        let region = built.region().expect("generated rectangle should materialize");
        prop_assert_eq!(
            region.classify_point(&p(xmin + 1, ymin + 1), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_unordered_line_arc_semicircles_build_native_regions(
        xmin in -50_i32..50,
        ymin in -50_i32..50,
        width in 4_i32..80,
        bulge_sign in any::<bool>(),
        order_variant in 0_usize..2,
        reverse_mask in 0_u8..4,
    ) {
        let xmax = xmin + width;
        let bulge = if bulge_sign { 1 } else { -1 };
        let inside_y = if bulge_sign { ymin - 1 } else { ymin + 1 };
        let mut segments = vec![
            Segment2::Line(line(xmax, ymin, xmin, ymin)),
            Segment2::Arc(arc_bulge(xmin, ymin, xmax, ymin, bulge)),
        ];
        for (index, segment) in segments.iter_mut().enumerate() {
            if reverse_mask & (1 << index) != 0 {
                *segment = reversed_segment(segment);
            }
        }
        if order_variant == 1 {
            segments.swap(0, 1);
        }

        let built = evaluate_unordered_segments(
            segments,
            FillRule::NonZero,
            &policy(),
        ).unwrap();

        prop_assert!(built.status().unwrap().is_native_exact());
        prop_assert_eq!(built.source_segment_count(), 2);
        prop_assert_eq!(built.arranged_segment_count(), Some(2));
        prop_assert_eq!(built.split_candidate_pair_count(), Some(1));
        prop_assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
        prop_assert_eq!(built.split_tested_pair_count(), Some(1));
        prop_assert_eq!(built.split_intersection_event_count(), Some(2));
        let split_points = built.split_intersection_points().unwrap();
        prop_assert_eq!(split_points.len(), 2);
        prop_assert!(split_points.contains(&p(xmin, ymin)));
        prop_assert!(split_points.contains(&p(xmax, ymin)));
        prop_assert_eq!(built.split_output_segment_count(), Some(2));
        prop_assert_eq!(built.split_blocker_first_source_segment_index(), None);
        prop_assert_eq!(built.split_blocker_second_source_segment_index(), None);
        prop_assert_eq!(built.endpoint_graph_endpoint_count(), Some(4));
        prop_assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(2));
        prop_assert_eq!(
            built.endpoint_graph_structural_singleton_bucket_count(),
            Some(0)
        );
        prop_assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(0));
        prop_assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(0));
        prop_assert_eq!(built.endpoint_graph_blocker_arranged_segment_index(), None);
        prop_assert_eq!(built.endpoint_graph_blocker_endpoint(), None);
        prop_assert_eq!(
            built.attempted_endpoint_connection_count(),
            Some(
                built.exact_endpoint_connection_count().unwrap()
                    + built.disconnected_endpoint_connection_count().unwrap()
                    + built.unresolved_endpoint_connection_count().unwrap()
            )
        );
        prop_assert!(built.exact_endpoint_connection_count().unwrap() >= 2);
        prop_assert_eq!(built.unresolved_endpoint_connection_count(), Some(0));
        prop_assert!(built.reversed_source_segment_count().unwrap() <= 1);
        let arranged_sources = built.arranged_source_evidence().unwrap();
        prop_assert_eq!(arranged_sources.len(), 2);
        prop_assert!(arranged_sources
            .iter()
            .all(|source| source.status().is_native_exact()));
        let source_evidence = built.source_evidence().unwrap();
        prop_assert_eq!(source_evidence.len(), 2);
        prop_assert!(source_evidence
            .iter()
            .all(|source| source.status().is_native_exact()));
        prop_assert_eq!(built.output_ring_count(), Some(1));
        prop_assert_eq!(built.output_boundary_segment_count(), Some(2));
        prop_assert_eq!(built.blocker(), None);

        let region = built.region().expect("generated line-arc region should materialize");
        prop_assert_eq!(
            region.classify_point(&p(xmin + width / 2, inside_y), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_rectangle_hole_filled_area_uses_role_not_orientation(
        width in 3_i32..80,
        height in 3_i32..80,
        hole_width in 1_i32..20,
        hole_height in 1_i32..20,
    ) {
        let hole_width = hole_width.min(width - 2);
        let hole_height = hole_height.min(height - 2);
        let region = LineArcRegion2::new(
            vec![reversed_rectangle(0, 0, width, height)],
            vec![reversed_rectangle(1, 1, 1 + hole_width, 1 + hole_height)],
        );
        let expected = Real::from(width * height - hole_width * hole_height);

        prop_assert_eq!(
            region.filled_area(&policy()).unwrap(),
            Classification::Decided(Some(expected.clone()))
        );

    }
}

#[test]
fn batched_region_classifier_matches_scalar_classification() {
    let region = LineArcRegion2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    let policy = policy();
    let facts = hypercurve::LineArcRegion2::structural_facts(&region, &policy);
    assert!(facts.has_decided_region_box);
    assert_eq!(facts.material_contour_count, 2);
    assert_eq!(facts.hole_contour_count, 1);
    assert_eq!(
        facts.segment_kinds,
        SegmentKindCounts { lines: 12, arcs: 0 }
    );
    let points = [p(1, 1), p(3, 3), p(5, 5), p(11, 1), p(100, 100)];
    let batched = hypercurve::LineArcRegion2::classify_points(&region, &points, &policy);
    assert_eq!(
        batched,
        points
            .iter()
            .map(|point| region.classify_point(point, &policy))
            .collect::<Vec<_>>()
    );
}

#[test]
fn batched_region_view_preserves_boundary_hits() {
    let material = [rectangle(0, 0, 4, 4), rectangle(20, 20, 24, 24)];
    let holes: [Contour2; 0] = [];
    let view = RegionView2::new(&material, &holes);
    let policy = policy();
    let batched = hypercurve::RegionView2::classify_points(
        &view,
        &[p(20, 22), p(21, 21), p(100, 100)],
        &policy,
    );
    assert_eq!(
        batched,
        vec![
            Classification::Decided(RegionPointLocation::Boundary),
            Classification::Decided(RegionPointLocation::Inside),
            Classification::Decided(RegionPointLocation::Outside),
        ]
    );
}
