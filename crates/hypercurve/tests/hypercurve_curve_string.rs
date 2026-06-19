use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    CurveStringChamferInputPath2, CurveStringChamferStage2, CurveStringConnectSource2,
    CurveStringConnectStage2, CurveStringCurveTrimQueryPath2, CurveStringCurveTrimStage2,
    CurveStringDeduplicateStage2, CurveStringEndpoint2,
    CurveStringEndpointConnectionPredicatePath2, CurveStringEndpointConnectionStatus2,
    CurveStringExtendStage2, CurveStringFilletInputPath2, CurveStringFilletStage2,
    CurveStringIntersectionPredicatePath2, CurveStringIntersectionQueryPath2,
    CurveStringLineMergeStage2, CurveStringLinkKind2, CurveStringLinkSourceInput2,
    CurveStringLinkStage2, CurveStringOrderedLinkStage2, CurveStringPreparedCacheFreshness2,
    CurveStringRegionTrimBoundaryPredicatePath2, CurveStringRegionTrimQueryPath2,
    CurveStringRegionTrimStage2, CurveStringTrimInputPath2, CurveStringTrimPoint2,
    IntersectionKind, LineArcIntersection, LineArcOrder, LineSeg2, Point2, Real, Region2,
    RegionContourRole, RegionPointLocation, Segment2, SegmentIntersection, SegmentKind,
    SegmentKindCounts, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn rectangle_region(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Region2 {
    Region2::from_material_contours(vec![
        Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(p(xmin, ymin), s(0)),
            BulgeVertex2::new(p(xmax, ymin), s(0)),
            BulgeVertex2::new(p(xmax, ymax), s(0)),
            BulgeVertex2::new(p(xmin, ymax), s(0)),
        ])
        .unwrap(),
    ])
}

fn assert_line(segment: &Segment2, start: Point2, end: Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn sparse_zigzag(segment_count: i32) -> CurveString2 {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(Segment2::Line(
            LineSeg2::try_new(previous, next.clone()).unwrap(),
        ));
        previous = next;
    }
    CurveString2::try_new(segments).unwrap()
}

#[test]
fn curve_string_and_contour_reject_forged_zero_length_segments() {
    let zero = Segment2::Line(LineSeg2::new_unchecked(p(0, 0), p(0, 0)));

    assert_eq!(
        CurveString2::try_new(vec![zero.clone()]).unwrap_err(),
        CurveError::ZeroLengthLine
    );
    assert_eq!(
        Contour2::try_new(vec![
            line_segment(0, 0, 1, 0),
            line_segment(1, 0, 0, 1),
            zero,
        ])
        .unwrap_err(),
        CurveError::ZeroLengthLine
    );
}

#[test]
fn curve_string_endpoint_report_certifies_exact_connection() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    let report = first
        .endpoint_connection_report(
            &second,
            CurveStringEndpoint2::End,
            CurveStringEndpoint2::Start,
            &policy(),
        )
        .unwrap();

    assert_eq!(report.first_endpoint(), CurveStringEndpoint2::End);
    assert_eq!(report.second_endpoint(), CurveStringEndpoint2::Start);
    assert_eq!(
        report.status(),
        CurveStringEndpointConnectionStatus2::NativeExact
    );
    assert!(report.topology_status().is_native_exact());
    assert_eq!(
        report.predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceZero
    );
    assert_eq!(report.distance_squared(), &s(0));
}

#[test]
fn curve_string_intersection_report_counts_aabb_skips() {
    let first =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 10, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, -1, 1, 1)]).unwrap();

    let intersections = first
        .intersect_curve_string_with_report(&second, &policy())
        .unwrap();
    let report = intersections.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.query_path(),
        CurveStringIntersectionQueryPath2::Direct
    );
    assert_eq!(
        report.predicate_path(),
        CurveStringIntersectionPredicatePath2::ExactSegmentPredicates
    );
    assert_eq!(report.first_segment_count(), 2);
    assert_eq!(report.second_segment_count(), 1);
    assert_eq!(
        report.first_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(
        report.second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(report.first_decided_segment_box_count(), 2);
    assert_eq!(report.second_decided_segment_box_count(), 1);
    assert_eq!(report.first_undecided_segment_box_count(), 0);
    assert_eq!(report.second_undecided_segment_box_count(), 0);
    assert_eq!(report.candidate_pair_count(), 2);
    assert_eq!(report.skipped_aabb_pair_count(), 1);
    assert_eq!(report.tested_pair_count(), 1);
    assert_eq!(report.intersection_count(), 1);
    assert_eq!(report.point_relation_count(), 1);
    assert_eq!(report.overlap_relation_count(), 0);
    assert_eq!(report.uncertain_relation_count(), 0);
    assert_eq!(report.prepared_cache_report(), None);
    assert_eq!(report.blocker(), None);
    assert_eq!(intersections.intersections().len(), 1);
    assert_eq!(intersections.intersections()[0].a_segment_index(), 0);
    assert_eq!(intersections.intersections()[0].b_segment_index(), 0);
    assert_eq!(
        intersections.intersections()[0].a_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        intersections.intersections()[0].a_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        intersections.intersections()[0].a_segment_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_start_point(),
        &p(1, -1)
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_end_point(),
        &p(1, 1)
    );
}

#[test]
fn curve_string_intersection_report_names_aabb_only_predicate_path() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap();

    let intersections = first
        .intersect_curve_string_with_report(&second, &policy())
        .unwrap();
    let report = intersections.report();

    assert_eq!(
        report.predicate_path(),
        CurveStringIntersectionPredicatePath2::AabbOnly
    );
    assert_eq!(report.candidate_pair_count(), 1);
    assert_eq!(
        report.first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        report.second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(report.skipped_aabb_pair_count(), 1);
    assert_eq!(report.tested_pair_count(), 0);
    assert_eq!(report.intersection_count(), 0);
    assert_eq!(report.point_relation_count(), 0);
    assert_eq!(report.overlap_relation_count(), 0);
    assert_eq!(report.uncertain_relation_count(), 0);
    assert!(intersections.intersections().is_empty());
}

#[test]
fn curve_string_intersection_report_counts_overlap_relations() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(2, 0, 6, 0)]).unwrap();

    let intersections = first
        .intersect_curve_string_with_report(&second, &policy())
        .unwrap();
    let report = intersections.report();

    assert_eq!(report.intersection_count(), 1);
    assert_eq!(report.point_relation_count(), 0);
    assert_eq!(report.overlap_relation_count(), 1);
    assert_eq!(report.uncertain_relation_count(), 0);
    assert_eq!(intersections.intersections().len(), 1);
    assert_eq!(
        intersections.intersections()[0].a_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        intersections.intersections()[0].a_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        intersections.intersections()[0].a_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_start_point(),
        &p(2, 0)
    );
    assert_eq!(
        intersections.intersections()[0].b_segment_end_point(),
        &p(6, 0)
    );
    let SegmentIntersection::LineLine(hypercurve::LineLineIntersection::Overlap { .. }) =
        intersections.intersections()[0].relation()
    else {
        panic!("expected line-line overlap relation");
    };
}

#[test]
fn prepared_curve_string_intersection_report_matches_plain_events() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let cutter = CurveString2::try_new(vec![line_segment(5, -1, 5, 1)]).unwrap();
    let policy = policy();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);

    let prepared = prepared_curve
        .intersect_prepared_curve_string_with_report(&prepared_cutter, &policy)
        .unwrap();
    let plain = curve.intersect_curve_string(&cutter, &policy).unwrap();

    assert!(prepared.report().status().is_native_exact());
    assert_eq!(
        prepared.report().query_path(),
        CurveStringIntersectionQueryPath2::Prepared
    );
    assert_eq!(
        prepared.report().predicate_path(),
        CurveStringIntersectionPredicatePath2::ExactSegmentPredicates
    );
    assert_eq!(prepared.report().candidate_pair_count(), 1);
    assert_eq!(
        prepared.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        prepared.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(prepared.report().first_decided_segment_box_count(), 1);
    assert_eq!(prepared.report().second_decided_segment_box_count(), 1);
    assert_eq!(prepared.report().first_undecided_segment_box_count(), 0);
    assert_eq!(prepared.report().second_undecided_segment_box_count(), 0);
    let prepared_cache = prepared.report().prepared_cache_report().unwrap();
    assert_eq!(
        prepared_cache.first().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(
        prepared_cache.second().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(prepared_cache.first().prepared_segment_count(), 1);
    assert_eq!(prepared_cache.second().prepared_segment_count(), 1);
    assert_eq!(
        prepared_cache.first().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        prepared_cache.second().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(prepared_cache.first().decided_segment_box_count(), 1);
    assert_eq!(prepared_cache.second().decided_segment_box_count(), 1);
    assert_eq!(prepared_cache.first().undecided_segment_box_count(), 0);
    assert_eq!(prepared_cache.second().undecided_segment_box_count(), 0);
    assert!(prepared_cache.first().curve_box_decided());
    assert!(prepared_cache.second().curve_box_decided());
    assert_eq!(prepared.report().tested_pair_count(), 1);
    assert_eq!(prepared.report().intersection_count(), plain.len());
    assert_eq!(prepared.report().point_relation_count(), 1);
    assert_eq!(prepared.report().overlap_relation_count(), 0);
    assert_eq!(prepared.report().uncertain_relation_count(), 0);
    assert_eq!(
        prepared.intersections()[0].a_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        prepared.intersections()[0].b_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(prepared.intersections(), plain.as_slice());
}

#[test]
fn prepared_curve_string_reports_cached_segment_box_counts() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 2, 3),
        line_segment(2, 3, 5, 3),
    ])
    .unwrap();
    let prepared = curve.prepare_topology_queries(&policy());

    assert_eq!(prepared.prepared_segment_count(), 3);
    assert_eq!(
        prepared.prepared_segment_count(),
        prepared.segment_boxes().len()
    );
    assert_eq!(
        prepared.prepared_segment_count(),
        prepared.prepared_segments().len()
    );
    assert!(
        prepared
            .prepared_segments()
            .iter()
            .all(|segment| segment.segment_kind() == SegmentKind::Line)
    );
    assert_eq!(
        prepared.prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 3, arcs: 0 }
    );
    assert_eq!(prepared.decided_segment_box_count(), 3);
    assert_eq!(prepared.undecided_segment_box_count(), 0);
    assert!(prepared.curve_box().is_some());
}

#[test]
fn prepared_contour_reports_cached_segment_box_counts() {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), s(0)),
        BulgeVertex2::new(p(4, 0), s(0)),
        BulgeVertex2::new(p(4, 3), s(0)),
        BulgeVertex2::new(p(0, 3), s(0)),
    ])
    .unwrap();
    let prepared = contour.prepare_topology_queries(&policy());

    assert_eq!(prepared.prepared_segment_count(), 4);
    assert_eq!(
        prepared.prepared_segment_count(),
        prepared.segment_boxes().len()
    );
    assert_eq!(
        prepared.prepared_segment_count(),
        prepared.prepared_segments().len()
    );
    assert!(
        prepared
            .prepared_segments()
            .iter()
            .all(|segment| segment.segment_kind() == SegmentKind::Line)
    );
    assert_eq!(
        prepared.prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(prepared.decided_segment_box_count(), 4);
    assert_eq!(prepared.undecided_segment_box_count(), 0);
    assert!(prepared.contour_box().is_some());
}

#[test]
fn curve_string_merge_adjacent_collinear_lines_reports_source_runs() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 5, 0),
        line_segment(5, 0, 5, 2),
        line_segment(5, 2, 5, 6),
    ])
    .unwrap();

    let merged = curve.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(
        merged.report().stage(),
        CurveStringLineMergeStage2::SegmentMaterialization
    );
    assert_eq!(merged.report().source_segment_count(), 4);
    assert_eq!(merged.report().adjacent_pair_count(), 3);
    assert_eq!(merged.report().merged_pair_count(), 2);
    assert_eq!(merged.report().preserved_pair_count(), 1);
    assert_eq!(merged.report().output_segment_count(), Some(2));
    assert_eq!(
        merged.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        merged.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    assert_eq!(merged.report().spans().len(), 2);
    assert_eq!(merged.report().spans()[0].source_start_segment_index(), 0);
    assert_eq!(merged.report().spans()[0].source_end_segment_index(), 1);
    assert_eq!(merged.report().spans()[0].source_segment_indices(), &[0, 1]);
    assert_eq!(
        merged.report().spans()[0].source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(merged.report().spans()[0].output_segment_index(), 0);
    assert_eq!(
        merged.report().spans()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(merged.report().spans()[0].output_start_point(), &p(0, 0));
    assert_eq!(merged.report().spans()[0].output_end_point(), &p(5, 0));
    assert_eq!(merged.report().spans()[1].source_start_segment_index(), 2);
    assert_eq!(merged.report().spans()[1].source_end_segment_index(), 3);
    assert_eq!(merged.report().spans()[1].source_segment_indices(), &[2, 3]);
    assert_eq!(
        merged.report().spans()[1].source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(merged.report().spans()[1].output_segment_index(), 1);
    assert_eq!(
        merged.report().spans()[1].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(merged.report().spans()[1].output_start_point(), &p(5, 0));
    assert_eq!(merged.report().spans()[1].output_end_point(), &p(5, 6));

    let curve = merged
        .curve_string()
        .expect("certified same-direction line runs should materialize");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(5, 0));
    assert_line(&curve.segments()[1], p(5, 0), p(5, 6));
}

#[test]
fn curve_string_merge_adjacent_collinear_lines_preserves_corners() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 3)]).unwrap();

    let merged = curve.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(merged.report().adjacent_pair_count(), 1);
    assert_eq!(merged.report().merged_pair_count(), 0);
    assert_eq!(merged.report().preserved_pair_count(), 1);
    assert_eq!(merged.report().output_segment_count(), Some(2));
    assert_eq!(merged.report().spans().len(), 2);
    assert_eq!(merged.report().spans()[0].source_segment_indices(), &[0]);
    assert_eq!(merged.report().spans()[1].source_segment_indices(), &[1]);
    let curve = merged
        .curve_string()
        .expect("certified corner preservation should materialize");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(2, 0));
    assert_line(&curve.segments()[1], p(2, 0), p(2, 3));
}

#[test]
fn curve_string_line_merge_span_reports_preserve_mixed_segment_kinds() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        Segment2::Arc(CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap()),
    ])
    .unwrap();

    let merged = curve.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(merged.report().merged_pair_count(), 0);
    assert_eq!(merged.report().preserved_pair_count(), 1);
    assert_eq!(merged.report().spans().len(), 2);
    assert_eq!(
        merged.report().spans()[0].source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        merged.report().spans()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        merged.report().spans()[1].source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(
        merged.report().spans()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        merged.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
}

#[test]
fn curve_string_merge_adjacent_collinear_lines_preserves_reversal() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 1, 0)]).unwrap();

    let merged = curve.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(merged.report().adjacent_pair_count(), 1);
    assert_eq!(merged.report().merged_pair_count(), 0);
    assert_eq!(merged.report().preserved_pair_count(), 1);
    assert_eq!(merged.report().output_segment_count(), Some(2));
    assert_eq!(merged.report().spans().len(), 2);
    assert_eq!(merged.report().spans()[0].source_segment_indices(), &[0]);
    assert_eq!(merged.report().spans()[1].source_segment_indices(), &[1]);
    let curve = merged
        .curve_string()
        .expect("certified reversal preservation should materialize");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(2, 0));
    assert_line(&curve.segments()[1], p(2, 0), p(1, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_reports_removed_pairs() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 2, 0),
        line_segment(2, 0, 1, 0),
        line_segment(1, 0, 3, 0),
    ])
    .unwrap();

    let deduped = curve.remove_adjacent_reversed_duplicates().unwrap();

    assert!(deduped.report().status().is_native_exact());
    assert_eq!(
        deduped.report().stage(),
        CurveStringDeduplicateStage2::SegmentMaterialization
    );
    assert_eq!(deduped.report().source_segment_count(), 4);
    assert_eq!(deduped.report().output_segment_count(), Some(2));
    assert_eq!(
        deduped.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        deduped.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    assert_eq!(deduped.report().retained_source_segment_indices(), &[0, 3]);
    assert_eq!(deduped.report().removed_pairs().len(), 1);
    assert_eq!(
        deduped.report().removed_pairs()[0].first_source_segment_index(),
        1
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_source_segment_index(),
        2
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].first_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].first_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].first_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_start_point(),
        &p(2, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_end_point(),
        &p(1, 0)
    );
    assert!(
        deduped.report().removed_pairs()[0]
            .status()
            .is_native_exact()
    );
    assert_eq!(deduped.report().retained_segments().len(), 2);
    assert_eq!(
        deduped.report().retained_segments()[0].source_segment_index(),
        0
    );
    assert_eq!(
        deduped.report().retained_segments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        deduped.report().retained_segments()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        deduped.report().retained_segments()[0].output_segment_index(),
        0
    );
    assert_eq!(
        deduped.report().retained_segments()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[0].source_segment_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[0].output_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[0].output_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[1].source_segment_index(),
        3
    );
    assert_eq!(
        deduped.report().retained_segments()[1].output_segment_index(),
        1
    );
    assert_eq!(
        deduped.report().retained_segments()[1].source_segment_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[1].source_segment_end_point(),
        &p(3, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().retained_segments()[1].output_end_point(),
        &p(3, 0)
    );
    let curve = deduped
        .curve_string()
        .expect("partial exact duplicate removal should materialize");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(1, 0));
    assert_line(&curve.segments()[1], p(1, 0), p(3, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_reports_mixed_segment_kinds() {
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap());
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        arc.clone(),
        arc.reversed(),
        line_segment(1, 0, 0, 0),
        line_segment(0, 0, -1, 0),
    ])
    .unwrap();

    let deduped = curve.remove_adjacent_reversed_duplicates().unwrap();
    let report = deduped.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        CurveStringDeduplicateStage2::SegmentMaterialization
    );
    assert_eq!(report.source_segment_count(), 5);
    assert_eq!(report.output_segment_count(), Some(1));
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 3, arcs: 2 }
    );
    assert_eq!(
        report.output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 0 })
    );
    assert_eq!(report.retained_source_segment_indices(), &[4]);
    assert_eq!(report.removed_pairs().len(), 2);
    assert_eq!(report.removed_pairs()[0].first_source_segment_index(), 1);
    assert_eq!(report.removed_pairs()[0].second_source_segment_index(), 2);
    assert_eq!(
        report.removed_pairs()[0].first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.removed_pairs()[0].second_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(report.removed_pairs()[1].first_source_segment_index(), 0);
    assert_eq!(report.removed_pairs()[1].second_source_segment_index(), 3);
    assert_eq!(
        report.removed_pairs()[1].first_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.removed_pairs()[1].second_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(report.retained_segments().len(), 1);
    assert_eq!(report.retained_segments()[0].source_segment_index(), 4);
    assert_eq!(
        report.retained_segments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.retained_segments()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert!(
        report
            .removed_pairs()
            .iter()
            .all(|pair| pair.status().is_native_exact())
    );
    assert!(
        report
            .retained_segments()
            .iter()
            .all(|segment| segment.status().is_native_exact())
    );

    let curve = deduped
        .curve_string()
        .expect("mixed duplicate removal should retain the final line");
    assert_eq!(curve.len(), 1);
    assert_line(&curve.segments()[0], p(0, 0), p(-1, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_reports_empty_output_blocker() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 2, 0),
        line_segment(2, 0, 1, 0),
        line_segment(1, 0, 0, 0),
    ])
    .unwrap();

    let deduped = curve.remove_adjacent_reversed_duplicates().unwrap();

    assert!(deduped.curve_string().is_none());
    assert!(deduped.report().status().is_retained_evidence());
    assert_eq!(
        deduped.report().stage(),
        CurveStringDeduplicateStage2::PairCancellation
    );
    assert_eq!(deduped.report().source_segment_count(), 4);
    assert_eq!(deduped.report().output_segment_count(), None);
    assert_eq!(
        deduped.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(deduped.report().output_segment_kind_counts(), None);
    assert!(
        deduped
            .report()
            .retained_source_segment_indices()
            .is_empty()
    );
    assert!(deduped.report().retained_segments().is_empty());
    assert_eq!(deduped.report().removed_pairs().len(), 2);
    assert_eq!(
        deduped.report().removed_pairs()[0].first_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].first_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_start_point(),
        &p(2, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[0].second_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[1].first_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[1].first_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[1].second_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        deduped.report().removed_pairs()[1].second_end_point(),
        &p(0, 0)
    );
    assert_eq!(
        deduped.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_keeps_partial_backtrack() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 3, 0), line_segment(3, 0, 1, 0)]).unwrap();

    let deduped = curve.remove_adjacent_reversed_duplicates().unwrap();

    assert!(deduped.report().status().is_native_exact());
    assert_eq!(deduped.report().removed_pairs().len(), 0);
    assert_eq!(deduped.report().output_segment_count(), Some(2));
    assert_eq!(deduped.report().retained_source_segment_indices(), &[0, 1]);
    let curve = deduped
        .curve_string()
        .expect("nonduplicate partial backtrack should remain materialized");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(3, 0));
    assert_line(&curve.segments()[1], p(3, 0), p(1, 0));
}

#[test]
fn curve_string_link_materializes_unique_end_start_connection() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    let linked = match first.link_connected_endpoints(&second, &policy()).unwrap() {
        Classification::Decided(Some(linked)) => linked,
        other => panic!("expected decided linked curve string, got {other:?}"),
    };
    let reported = first
        .link_connected_endpoints_with_report(&second, &policy())
        .unwrap();

    assert_eq!(
        linked.report().kind(),
        CurveStringLinkKind2::FirstEndToSecondStart
    );
    assert!(reported.linked_curve_string().is_some());
    assert_eq!(
        reported.report().stage(),
        CurveStringLinkStage2::SegmentMaterialization
    );
    assert_eq!(
        reported.report().selected_kind(),
        Some(CurveStringLinkKind2::FirstEndToSecondStart)
    );
    assert_eq!(
        reported
            .report()
            .selected_endpoint_report()
            .unwrap()
            .status(),
        CurveStringEndpointConnectionStatus2::NativeExact
    );
    assert_eq!(
        reported
            .report()
            .selected_endpoint_report()
            .unwrap()
            .predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceZero
    );
    assert_eq!(reported.report().first_segment_count(), 1);
    assert_eq!(reported.report().second_segment_count(), 1);
    assert_eq!(reported.report().endpoint_pair_count(), 4);
    assert_eq!(reported.report().exact_endpoint_pair_count(), 1);
    assert_eq!(reported.report().disconnected_endpoint_pair_count(), 3);
    assert_eq!(reported.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(reported.report().output_segment_count(), Some(2));
    assert_eq!(reported.report().output_segments().len(), 2);
    assert!(reported.report().status().is_native_exact());
    assert_eq!(reported.report().blocker(), None);
    assert_eq!(linked.report().endpoint_report().first_point(), &p(1, 0));
    assert_eq!(linked.report().endpoint_report().second_point(), &p(1, 0));
    assert_eq!(
        linked.report().endpoint_report().predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceZero
    );
    assert_eq!(linked.report().first_segment_count(), 1);
    assert_eq!(linked.report().second_segment_count(), 1);
    assert_eq!(linked.report().endpoint_pair_count(), 4);
    assert_eq!(linked.report().exact_endpoint_pair_count(), 1);
    assert_eq!(linked.report().disconnected_endpoint_pair_count(), 3);
    assert_eq!(linked.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(linked.report().output_segment_count(), Some(2));
    assert_eq!(linked.report().output_segments().len(), 2);
    assert_eq!(
        linked.report().output_segments()[0].source_input(),
        CurveStringLinkSourceInput2::First
    );
    assert_eq!(
        linked.report().output_segments()[0].source_segment_index(),
        0
    );
    assert_eq!(
        linked.report().output_segments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        linked.report().output_segments()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert!(!linked.report().output_segments()[0].reversed());
    assert_eq!(
        linked.report().output_segments()[0].output_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        linked.report().output_segments()[0].output_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].source_input(),
        CurveStringLinkSourceInput2::Second
    );
    assert_eq!(
        linked.report().output_segments()[1].source_segment_index(),
        0
    );
    assert!(!linked.report().output_segments()[1].reversed());
    assert_eq!(
        linked.report().output_segments()[1].source_segment_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].source_segment_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].output_end_point(),
        &p(2, 0)
    );
    assert!(linked.report().status().is_native_exact());
    assert_eq!(
        linked.report().stage(),
        CurveStringLinkStage2::SegmentMaterialization
    );
    assert_eq!(linked.curve_string().len(), 2);
    assert_eq!(linked.curve_string().start(), Some(&p(0, 0)));
    assert_eq!(linked.curve_string().end(), Some(&p(2, 0)));
}

#[test]
fn curve_string_link_output_reports_preserve_mixed_segment_kinds() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let linked = match first.link_connected_endpoints(&second, &policy()).unwrap() {
        Classification::Decided(Some(linked)) => linked,
        other => panic!("expected decided mixed linked curve string, got {other:?}"),
    };

    assert!(linked.report().status().is_native_exact());
    assert_eq!(
        linked.report().kind(),
        CurveStringLinkKind2::FirstEndToSecondStart
    );
    assert_eq!(
        linked.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        linked.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(
        linked.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(linked.report().output_segments().len(), 2);
    assert_eq!(
        linked.report().output_segments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        linked.report().output_segments()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        linked.report().output_segments()[1].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        linked.report().output_segments()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert!(!linked.report().output_segments()[1].reversed());
    assert_eq!(
        linked.report().output_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].output_end_point(),
        &p(3, 0)
    );
}

#[test]
fn curve_string_link_reverses_second_curve_when_endpoints_match_end_to_end() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(2, 0, 1, 0)]).unwrap();

    let linked = match first.link_connected_endpoints(&second, &policy()).unwrap() {
        Classification::Decided(Some(linked)) => linked,
        other => panic!("expected decided linked curve string, got {other:?}"),
    };

    assert_eq!(
        linked.report().kind(),
        CurveStringLinkKind2::FirstEndToSecondEnd
    );
    assert_eq!(linked.report().endpoint_pair_count(), 4);
    assert_eq!(linked.report().exact_endpoint_pair_count(), 1);
    assert_eq!(linked.report().disconnected_endpoint_pair_count(), 3);
    assert_eq!(linked.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(linked.report().output_segment_count(), Some(2));
    assert_eq!(linked.report().output_segments().len(), 2);
    assert_eq!(
        linked.report().output_segments()[1].source_input(),
        CurveStringLinkSourceInput2::Second
    );
    assert_eq!(
        linked.report().output_segments()[1].source_segment_index(),
        0
    );
    assert!(linked.report().output_segments()[1].reversed());
    assert_eq!(
        linked.report().output_segments()[1].source_segment_start_point(),
        &p(2, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].source_segment_end_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        linked.report().output_segments()[1].output_end_point(),
        &p(2, 0)
    );
    assert_eq!(linked.curve_string().start(), Some(&p(0, 0)));
    assert_eq!(linked.curve_string().end(), Some(&p(2, 0)));
    assert_eq!(linked.curve_string().segments()[1].start(), &p(1, 0));
    assert_eq!(linked.curve_string().segments()[1].end(), &p(2, 0));
}

#[test]
fn curve_string_link_returns_none_for_certified_disconnected_inputs() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap();

    let disconnected = first
        .endpoint_connection_report(
            &second,
            CurveStringEndpoint2::End,
            CurveStringEndpoint2::Start,
            &policy(),
        )
        .unwrap();

    assert_eq!(
        disconnected.status(),
        CurveStringEndpointConnectionStatus2::Disconnected
    );
    assert_eq!(
        disconnected.predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceNonzero
    );
    assert_eq!(disconnected.first_point(), &p(1, 0));
    assert_eq!(disconnected.second_point(), &p(3, 0));
    assert_eq!(
        first.link_connected_endpoints(&second, &policy()).unwrap(),
        Classification::Decided(None)
    );
    let reported = first
        .link_connected_endpoints_with_report(&second, &policy())
        .unwrap();
    assert!(reported.linked_curve_string().is_none());
    assert_eq!(
        reported.report().stage(),
        CurveStringLinkStage2::EndpointSelection
    );
    assert_eq!(reported.report().selected_kind(), None);
    assert!(reported.report().selected_endpoint_report().is_none());
    assert_eq!(reported.report().endpoint_pair_count(), 4);
    assert_eq!(reported.report().exact_endpoint_pair_count(), 0);
    assert_eq!(reported.report().disconnected_endpoint_pair_count(), 4);
    assert_eq!(reported.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(reported.report().output_segment_count(), None);
    assert_eq!(
        reported.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        reported.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(reported.report().output_segment_kind_counts(), None);
    assert!(reported.report().output_segments().is_empty());
    assert!(reported.report().status().is_retained_evidence());
    assert_eq!(
        reported.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_ordered_link_materializes_multistep_chain() {
    let linked = CurveString2::link_ordered_connected_endpoints(
        vec![
            CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
            CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap(),
            CurveString2::try_new(vec![line_segment(2, 0, 3, 0)]).unwrap(),
        ],
        &policy(),
    )
    .unwrap();

    assert!(linked.report().status().is_native_exact());
    assert_eq!(
        linked.report().stage(),
        CurveStringOrderedLinkStage2::ChainMaterialization
    );
    assert_eq!(linked.report().source_curve_string_count(), 3);
    assert_eq!(linked.report().attempted_link_step_count(), 2);
    assert_eq!(linked.report().materialized_link_step_count(), 2);
    assert_eq!(linked.report().blocked_link_step_count(), 0);
    assert_eq!(linked.report().output_segment_count(), Some(3));
    assert_eq!(
        linked.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 3, arcs: 0 }
    );
    assert_eq!(
        linked.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 3, arcs: 0 })
    );
    assert_eq!(linked.report().output_source_indices(), &[0, 1, 2]);
    assert_eq!(linked.report().steps().len(), 2);
    assert_eq!(
        linked.report().steps()[0].accumulated_source_indices(),
        &[0]
    );
    assert_eq!(linked.report().steps()[0].next_source_index(), 1);
    assert_eq!(
        linked.report().steps()[0].link_report().unwrap().kind(),
        CurveStringLinkKind2::FirstEndToSecondStart
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_attempt_report()
            .unwrap()
            .selected_kind(),
        Some(CurveStringLinkKind2::FirstEndToSecondStart)
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_attempt_report()
            .unwrap()
            .blocker(),
        None
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_attempt_report()
            .unwrap()
            .output_segment_count(),
        Some(2)
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .output_segment_count(),
        Some(2)
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .endpoint_pair_count(),
        4
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .exact_endpoint_pair_count(),
        1
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .output_segments()[0]
            .source_input(),
        CurveStringLinkSourceInput2::First
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .output_segments()[1]
            .source_input(),
        CurveStringLinkSourceInput2::Second
    );
    assert_eq!(
        linked.report().steps()[1].accumulated_source_indices(),
        &[0, 1]
    );
    assert_eq!(linked.report().steps()[1].next_source_index(), 2);

    let curve = linked
        .curve_string()
        .expect("ordered exact links should materialize");
    assert_eq!(curve.len(), 3);
    assert_line(&curve.segments()[0], p(0, 0), p(1, 0));
    assert_line(&curve.segments()[2], p(2, 0), p(3, 0));
}

#[test]
fn borrowed_curve_string_ordered_link_materializes_multistep_chain() {
    let curve_strings = vec![
        CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(2, 0, 3, 0)]).unwrap(),
    ];

    let linked =
        CurveString2::link_ordered_connected_endpoints_borrowed(&curve_strings, &policy()).unwrap();

    assert_eq!(curve_strings.len(), 3);
    assert!(linked.report().status().is_native_exact());
    assert_eq!(
        linked.report().stage(),
        CurveStringOrderedLinkStage2::ChainMaterialization
    );
    assert_eq!(linked.report().source_curve_string_count(), 3);
    assert_eq!(linked.report().attempted_link_step_count(), 2);
    assert_eq!(linked.report().materialized_link_step_count(), 2);
    assert_eq!(linked.report().blocked_link_step_count(), 0);
    assert_eq!(linked.report().output_segment_count(), Some(3));
    assert_eq!(linked.report().output_source_indices(), &[0, 1, 2]);
    assert_eq!(linked.report().steps().len(), 2);
    let curve = linked
        .curve_string()
        .expect("borrowed ordered exact links should materialize");
    assert_eq!(curve.len(), 3);
    assert_line(&curve.segments()[0], p(0, 0), p(1, 0));
    assert_line(&curve.segments()[2], p(2, 0), p(3, 0));
}

#[test]
fn curve_string_ordered_link_reports_reversed_accumulated_sources() {
    let linked = CurveString2::link_ordered_connected_endpoints(
        vec![
            CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap(),
            CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
        ],
        &policy(),
    )
    .unwrap();

    assert!(linked.report().status().is_native_exact());
    assert_eq!(linked.report().attempted_link_step_count(), 1);
    assert_eq!(linked.report().materialized_link_step_count(), 1);
    assert_eq!(linked.report().blocked_link_step_count(), 0);
    assert_eq!(linked.report().output_source_indices(), &[1, 0]);
    assert_eq!(
        linked.report().steps()[0].accumulated_source_indices(),
        &[0]
    );
    assert_eq!(linked.report().steps()[0].next_source_index(), 1);
    assert_eq!(
        linked.report().steps()[0].link_report().unwrap().kind(),
        CurveStringLinkKind2::FirstStartToSecondEnd
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_attempt_report()
            .unwrap()
            .selected_kind(),
        Some(CurveStringLinkKind2::FirstStartToSecondEnd)
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .endpoint_pair_count(),
        4
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .exact_endpoint_pair_count(),
        1
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .output_segments()[0]
            .source_input(),
        CurveStringLinkSourceInput2::Second
    );
    assert_eq!(
        linked.report().steps()[0]
            .link_report()
            .unwrap()
            .output_segments()[1]
            .source_input(),
        CurveStringLinkSourceInput2::First
    );
    let curve = linked
        .curve_string()
        .expect("reordered exact endpoint link should materialize");
    assert_eq!(curve.len(), 2);
    assert_line(&curve.segments()[0], p(0, 0), p(1, 0));
    assert_line(&curve.segments()[1], p(1, 0), p(2, 0));
}

#[test]
fn curve_string_ordered_link_reports_disconnected_step() {
    let linked = CurveString2::link_ordered_connected_endpoints(
        vec![
            CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
            CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap(),
        ],
        &policy(),
    )
    .unwrap();

    assert!(linked.curve_string().is_none());
    assert!(linked.report().status().is_retained_evidence());
    assert_eq!(
        linked.report().stage(),
        CurveStringOrderedLinkStage2::StepLinking
    );
    assert_eq!(linked.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(linked.report().attempted_link_step_count(), 1);
    assert_eq!(linked.report().materialized_link_step_count(), 0);
    assert_eq!(linked.report().blocked_link_step_count(), 1);
    assert_eq!(linked.report().output_segment_count(), None);
    assert_eq!(
        linked.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(linked.report().output_segment_kind_counts(), None);
    assert_eq!(linked.report().output_source_indices(), &[0]);
    assert_eq!(linked.report().steps().len(), 1);
    assert_eq!(
        linked.report().steps()[0].status(),
        hypercurve::RetainedTopologyStatus::Unsupported
    );
    assert_eq!(
        linked.report().steps()[0].blocker(),
        Some(UncertaintyReason::Boundary)
    );
    let step_attempt = linked.report().steps()[0].link_attempt_report().unwrap();
    assert_eq!(
        step_attempt.stage(),
        CurveStringLinkStage2::EndpointSelection
    );
    assert_eq!(step_attempt.selected_kind(), None);
    assert_eq!(step_attempt.endpoint_pair_count(), 4);
    assert_eq!(step_attempt.exact_endpoint_pair_count(), 0);
    assert_eq!(step_attempt.disconnected_endpoint_pair_count(), 4);
    assert_eq!(step_attempt.unresolved_endpoint_pair_count(), 0);
    assert_eq!(step_attempt.output_segment_count(), None);
    assert_eq!(
        step_attempt.first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        step_attempt.second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(step_attempt.output_segment_kind_counts(), None);
    assert_eq!(step_attempt.blocker(), Some(UncertaintyReason::Boundary));
    assert!(linked.report().steps()[0].link_report().is_none());
}

#[test]
fn borrowed_curve_string_ordered_link_reports_disconnected_step() {
    let curve_strings = vec![
        CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap(),
    ];

    let linked =
        CurveString2::link_ordered_connected_endpoints_borrowed(&curve_strings, &policy()).unwrap();

    assert_eq!(curve_strings.len(), 2);
    assert!(linked.curve_string().is_none());
    assert!(linked.report().status().is_retained_evidence());
    assert_eq!(
        linked.report().stage(),
        CurveStringOrderedLinkStage2::StepLinking
    );
    assert_eq!(linked.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(linked.report().attempted_link_step_count(), 1);
    assert_eq!(linked.report().materialized_link_step_count(), 0);
    assert_eq!(linked.report().blocked_link_step_count(), 1);
    assert_eq!(linked.report().output_segment_count(), None);
    assert_eq!(linked.report().output_source_indices(), &[0]);
    assert_eq!(linked.report().steps().len(), 1);
    assert_eq!(
        linked.report().steps()[0].blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert!(linked.report().steps()[0].link_report().is_none());
    assert_eq!(
        linked.report().steps()[0]
            .link_attempt_report()
            .unwrap()
            .endpoint_pair_count(),
        4
    );
}

#[test]
fn curve_string_connect_end_to_start_inserts_exact_line() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(3, 1, 4, 1)]).unwrap();

    let connected = first
        .connect_end_to_start_with_line_with_report(&second, &policy())
        .unwrap();

    assert!(connected.report().status().is_native_exact());
    assert_eq!(
        connected.report().stage(),
        CurveStringConnectStage2::ConnectorMaterialization
    );
    assert!(connected.report().blocker().is_none());
    assert_eq!(
        connected.report().endpoint_report().status(),
        CurveStringEndpointConnectionStatus2::Disconnected
    );
    assert_eq!(
        connected.report().endpoint_report().predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceNonzero
    );
    assert_eq!(connected.report().endpoint_report().first_point(), &p(1, 0));
    assert_eq!(
        connected.report().endpoint_report().second_point(),
        &p(3, 1)
    );
    assert_eq!(connected.report().first_segment_count(), 1);
    assert_eq!(connected.report().second_segment_count(), 1);
    assert_eq!(
        connected.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        connected.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(connected.report().endpoint_pair_count(), 1);
    assert_eq!(connected.report().endpoint_reports().len(), 1);
    assert_eq!(
        connected.report().endpoint_reports()[0].status(),
        CurveStringEndpointConnectionStatus2::Disconnected
    );
    assert_eq!(
        connected.report().endpoint_reports()[0].predicate_path(),
        CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceNonzero
    );
    assert_eq!(connected.report().exact_endpoint_pair_count(), 0);
    assert_eq!(connected.report().disconnected_endpoint_pair_count(), 1);
    assert_eq!(connected.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(
        connected.report().kind(),
        Some(CurveStringLinkKind2::FirstEndToSecondStart)
    );
    assert_eq!(connected.report().connector_segment_index(), Some(1));
    assert_eq!(connected.report().connector_start_point(), Some(&p(1, 0)));
    assert_eq!(connected.report().connector_end_point(), Some(&p(3, 1)));
    assert_eq!(connected.report().output_segment_count(), Some(3));
    assert_eq!(
        connected.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 3, arcs: 0 })
    );
    assert_eq!(connected.report().output_segments().len(), 3);
    assert_eq!(
        connected.report().output_segments()[0].source(),
        CurveStringConnectSource2::First
    );
    assert_eq!(
        connected.report().output_segments()[0].source_segment_index(),
        Some(0)
    );
    assert_eq!(
        connected.report().output_segments()[0].source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        connected.report().output_segments()[0].source_segment_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        connected.report().output_segments()[0].source_segment_end_point(),
        Some(&p(1, 0))
    );
    assert_eq!(
        connected.report().output_segments()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        connected.report().output_segments()[1].source(),
        CurveStringConnectSource2::Connector
    );
    assert_eq!(
        connected.report().output_segments()[1].source_segment_index(),
        None
    );
    assert_eq!(
        connected.report().output_segments()[1].source_segment_kind(),
        None
    );
    assert_eq!(
        connected.report().output_segments()[1].source_segment_start_point(),
        None
    );
    assert_eq!(
        connected.report().output_segments()[1].source_segment_end_point(),
        None
    );
    assert_eq!(
        connected.report().output_segments()[1].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        connected.report().output_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        connected.report().output_segments()[1].output_end_point(),
        &p(3, 1)
    );
    assert_eq!(
        connected.report().output_segments()[2].source(),
        CurveStringConnectSource2::Second
    );
    assert_eq!(
        connected.report().output_segments()[2].source_segment_start_point(),
        Some(&p(3, 1))
    );
    assert_eq!(
        connected.report().output_segments()[2].source_segment_end_point(),
        Some(&p(4, 1))
    );
    let curve = connected
        .curve_string()
        .expect("distinct endpoints should get connector");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.start(), Some(&p(0, 0)));
    assert_eq!(curve.end(), Some(&p(4, 1)));
    assert_eq!(curve.segments()[1].start(), &p(1, 0));
    assert_eq!(curve.segments()[1].end(), &p(3, 1));
}

#[test]
fn curve_string_connect_output_reports_preserve_mixed_segment_kinds() {
    let first = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let second = CurveString2::try_new(vec![line_segment(4, 0, 5, 0)]).unwrap();

    let connected = first
        .connect_end_to_start_with_line(&second, &policy())
        .unwrap();
    let report = connected.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.first_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(
        report.second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        report.output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 1 })
    );
    assert_eq!(report.output_segments().len(), 3);
    assert_eq!(
        report.output_segments()[0].source(),
        CurveStringConnectSource2::First
    );
    assert_eq!(
        report.output_segments()[0].source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        report.output_segments()[0].output_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.output_segments()[1].source(),
        CurveStringConnectSource2::Connector
    );
    assert_eq!(report.output_segments()[1].source_segment_kind(), None);
    assert_eq!(
        report.output_segments()[1].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.output_segments()[2].source(),
        CurveStringConnectSource2::Second
    );
    assert_eq!(
        report.output_segments()[2].source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        report.output_segments()[2].output_segment_kind(),
        SegmentKind::Line
    );

    let curve = connected
        .curve_string()
        .expect("mixed connector should materialize");
    assert_eq!(curve.len(), 3);
    assert!(matches!(curve.segments()[0], Segment2::Arc(_)));
    assert_line(&curve.segments()[1], p(2, 0), p(4, 0));
    assert_line(&curve.segments()[2], p(4, 0), p(5, 0));
}

#[test]
fn curve_string_connect_selected_endpoints_orients_inputs() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap();

    let connected = first
        .connect_endpoints_with_line_with_report(
            &second,
            CurveStringLinkKind2::FirstStartToSecondEnd,
            &policy(),
        )
        .unwrap();

    assert!(connected.report().status().is_native_exact());
    assert_eq!(connected.report().endpoint_pair_count(), 1);
    assert_eq!(connected.report().endpoint_reports().len(), 1);
    assert_eq!(connected.report().exact_endpoint_pair_count(), 0);
    assert_eq!(connected.report().disconnected_endpoint_pair_count(), 1);
    assert_eq!(connected.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(
        connected.report().kind(),
        Some(CurveStringLinkKind2::FirstStartToSecondEnd)
    );
    assert_eq!(connected.report().connector_segment_index(), Some(1));
    assert_eq!(connected.report().connector_start_point(), Some(&p(4, 0)));
    assert_eq!(connected.report().connector_end_point(), Some(&p(0, 0)));
    assert_eq!(connected.report().output_segment_count(), Some(3));
    assert_eq!(connected.report().output_segments().len(), 3);
    assert_eq!(
        connected.report().output_segments()[0].source(),
        CurveStringConnectSource2::Second
    );
    assert_eq!(
        connected.report().output_segments()[0].output_start_point(),
        &p(3, 0)
    );
    assert_eq!(
        connected.report().output_segments()[0].output_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        connected.report().output_segments()[1].source(),
        CurveStringConnectSource2::Connector
    );
    assert_eq!(
        connected.report().output_segments()[1].output_start_point(),
        &p(4, 0)
    );
    assert_eq!(
        connected.report().output_segments()[1].output_end_point(),
        &p(0, 0)
    );
    assert_eq!(
        connected.report().output_segments()[2].source(),
        CurveStringConnectSource2::First
    );
    let curve = connected
        .curve_string()
        .expect("selected endpoint connector should materialize");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.start(), Some(&p(3, 0)));
    assert_eq!(curve.segments()[1].start(), &p(4, 0));
    assert_eq!(curve.segments()[1].end(), &p(0, 0));
    assert_eq!(curve.end(), Some(&p(1, 0)));
}

#[test]
fn curve_string_connect_nearest_endpoints_selects_unique_pair() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(4, 0, 3, 0)]).unwrap();

    let connected = first
        .connect_nearest_endpoints_with_line_with_report(&second, &policy())
        .unwrap();

    assert!(connected.report().status().is_native_exact());
    assert_eq!(connected.report().endpoint_pair_count(), 4);
    assert_eq!(connected.report().endpoint_reports().len(), 4);
    assert_eq!(connected.report().exact_endpoint_pair_count(), 0);
    assert_eq!(connected.report().disconnected_endpoint_pair_count(), 4);
    assert_eq!(connected.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(
        connected.report().kind(),
        Some(CurveStringLinkKind2::FirstEndToSecondEnd)
    );
    assert_eq!(connected.report().connector_segment_index(), Some(1));
    assert_eq!(connected.report().connector_start_point(), Some(&p(1, 0)));
    assert_eq!(connected.report().connector_end_point(), Some(&p(3, 0)));
    assert_eq!(connected.report().output_segment_count(), Some(3));
    assert_eq!(connected.report().output_segments().len(), 3);
    assert_eq!(
        connected.report().output_segments()[0].source(),
        CurveStringConnectSource2::First
    );
    assert_eq!(
        connected.report().output_segments()[1].source(),
        CurveStringConnectSource2::Connector
    );
    assert_eq!(
        connected.report().output_segments()[1].output_start_point(),
        &p(1, 0)
    );
    assert_eq!(
        connected.report().output_segments()[1].output_end_point(),
        &p(3, 0)
    );
    assert_eq!(
        connected.report().output_segments()[2].source(),
        CurveStringConnectSource2::Second
    );
    assert!(connected.report().output_segments()[2].reversed());
    assert_eq!(
        connected.report().output_segments()[2].source_segment_start_point(),
        Some(&p(4, 0))
    );
    assert_eq!(
        connected.report().output_segments()[2].source_segment_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        connected.report().output_segments()[2].output_start_point(),
        &p(3, 0)
    );
    assert_eq!(
        connected.report().output_segments()[2].output_end_point(),
        &p(4, 0)
    );
    let curve = connected
        .curve_string()
        .expect("nearest endpoint connector should materialize");
    assert_eq!(curve.start(), Some(&p(0, 0)));
    assert_eq!(curve.segments()[1].start(), &p(1, 0));
    assert_eq!(curve.segments()[1].end(), &p(3, 0));
    assert_eq!(curve.end(), Some(&p(4, 0)));
}

#[test]
fn curve_string_connect_nearest_endpoints_reports_tie_boundary() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 2, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 3, 1, 5)]).unwrap();

    let connected = first
        .connect_nearest_endpoints_with_line(&second, &policy())
        .unwrap();

    assert!(connected.curve_string().is_none());
    assert!(connected.report().status().is_retained_evidence());
    assert_eq!(
        connected.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(connected.report().endpoint_pair_count(), 4);
    assert_eq!(connected.report().endpoint_reports().len(), 4);
    assert_eq!(connected.report().exact_endpoint_pair_count(), 0);
    assert_eq!(connected.report().disconnected_endpoint_pair_count(), 4);
    assert_eq!(connected.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(connected.report().connector_segment_index(), None);
    assert_eq!(connected.report().connector_start_point(), None);
    assert_eq!(connected.report().connector_end_point(), None);
    assert_eq!(connected.report().output_segment_count(), None);
    assert_eq!(
        connected.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        connected.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(connected.report().output_segment_kind_counts(), None);
    assert!(connected.report().output_segments().is_empty());
}

#[test]
fn curve_string_connect_end_to_start_blocks_already_connected_endpoints() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    let connected = first
        .connect_end_to_start_with_line(&second, &policy())
        .unwrap();

    assert!(connected.curve_string().is_none());
    assert!(connected.report().status().is_retained_evidence());
    assert_eq!(
        connected.report().stage(),
        CurveStringConnectStage2::EndpointSelection
    );
    assert_eq!(connected.report().endpoint_pair_count(), 1);
    assert_eq!(connected.report().endpoint_reports().len(), 1);
    assert_eq!(connected.report().exact_endpoint_pair_count(), 1);
    assert_eq!(connected.report().disconnected_endpoint_pair_count(), 0);
    assert_eq!(connected.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(connected.report().connector_segment_index(), None);
    assert_eq!(connected.report().connector_start_point(), None);
    assert_eq!(connected.report().connector_end_point(), None);
    assert_eq!(connected.report().output_segment_count(), None);
    assert_eq!(
        connected.report().first_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        connected.report().second_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(connected.report().output_segment_kind_counts(), None);
    assert_eq!(
        connected.report().endpoint_report().status(),
        CurveStringEndpointConnectionStatus2::NativeExact
    );
    assert_eq!(
        connected.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_connect_rejects_empty_unchecked_input() {
    let empty = CurveString2::new_unchecked(Vec::new());
    let nonempty = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();

    assert_eq!(
        empty
            .connect_end_to_start_with_line(&nonempty, &policy())
            .unwrap_err(),
        CurveError::EmptyCurveString
    );
}

#[test]
fn curve_string_link_rejects_multiple_exact_endpoint_pairings() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 0, 0)]).unwrap();

    assert_eq!(
        first.link_connected_endpoints(&second, &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let reported = first
        .link_connected_endpoints_with_report(&second, &policy())
        .unwrap();
    assert!(reported.linked_curve_string().is_none());
    assert_eq!(
        reported.report().stage(),
        CurveStringLinkStage2::EndpointSelection
    );
    assert_eq!(reported.report().selected_kind(), None);
    assert!(reported.report().selected_endpoint_report().is_none());
    assert_eq!(reported.report().endpoint_pair_count(), 4);
    assert_eq!(reported.report().exact_endpoint_pair_count(), 2);
    assert_eq!(reported.report().disconnected_endpoint_pair_count(), 2);
    assert_eq!(reported.report().unresolved_endpoint_pair_count(), 0);
    assert_eq!(reported.report().output_segment_count(), None);
    assert!(reported.report().output_segments().is_empty());
    assert!(reported.report().status().is_retained_evidence());
    assert_eq!(
        reported.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_endpoint_report_rejects_empty_unchecked_input() {
    let empty = CurveString2::new_unchecked(Vec::new());
    let nonempty = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();

    assert_eq!(
        empty
            .endpoint_connection_report(
                &nonempty,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::Start,
                &policy(),
            )
            .unwrap_err(),
        CurveError::EmptyCurveString
    );
}

#[test]
fn curve_string_extend_line_end_to_exact_target() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 2, 0)]).unwrap();

    let extended = curve
        .extend_line_endpoint_to_point_with_report(CurveStringEndpoint2::End, p(5, 0), &policy())
        .unwrap();

    assert!(extended.report().status().is_native_exact());
    assert_eq!(
        extended.report().stage(),
        CurveStringExtendStage2::SegmentMaterialization
    );
    assert_eq!(extended.report().endpoint(), CurveStringEndpoint2::End);
    assert_eq!(extended.report().source_segment_index(), 0);
    assert_eq!(extended.report().source_segment_kind(), SegmentKind::Line);
    assert_eq!(extended.report().output_segment_index(), Some(0));
    assert_eq!(
        extended.report().output_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        extended.report().output_segment_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(extended.report().output_segment_end_point(), Some(&p(5, 0)));
    assert_eq!(extended.report().source_segment_start_point(), &p(0, 0));
    assert_eq!(extended.report().source_segment_end_point(), &p(2, 0));
    assert_eq!(extended.report().source_endpoint_point(), &p(2, 0));
    assert_eq!(extended.report().target_point(), &p(5, 0));
    assert_eq!(extended.report().source_param(), Some(&q(5, 2)));
    assert_eq!(extended.report().source_segment_count(), 1);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(extended.report().output_segment_count(), Some(1));
    assert_eq!(
        extended.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 0 })
    );
    assert!(extended.report().blocker().is_none());
    let curve = extended
        .curve_string()
        .expect("line extension should materialize");
    assert_eq!(curve.start(), Some(&p(0, 0)));
    assert_eq!(curve.end(), Some(&p(5, 0)));
}

#[test]
fn curve_string_extend_line_start_to_exact_target() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 2)]).unwrap();

    let extended = curve
        .extend_line_endpoint_to_point(CurveStringEndpoint2::Start, p(-3, 0), &policy())
        .unwrap();

    assert!(extended.report().status().is_native_exact());
    assert_eq!(extended.report().endpoint(), CurveStringEndpoint2::Start);
    assert_eq!(extended.report().source_segment_index(), 0);
    assert_eq!(extended.report().output_segment_index(), Some(0));
    assert_eq!(
        extended.report().output_segment_start_point(),
        Some(&p(-3, 0))
    );
    assert_eq!(extended.report().output_segment_end_point(), Some(&p(2, 0)));
    assert_eq!(extended.report().source_endpoint_point(), &p(0, 0));
    assert_eq!(extended.report().target_point(), &p(-3, 0));
    assert_eq!(extended.report().source_param(), Some(&q(-3, 2)));
    assert_eq!(extended.report().source_segment_count(), 2);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(extended.report().output_segment_count(), Some(2));
    assert_eq!(
        extended.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    let curve = extended
        .curve_string()
        .expect("start line extension should materialize");
    assert_eq!(curve.len(), 2);
    assert_eq!(curve.start(), Some(&p(-3, 0)));
    assert_eq!(curve.end(), Some(&p(2, 2)));
}

#[test]
fn curve_string_extend_line_reports_interior_target_boundary() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let extended = curve
        .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(1, 0), &policy())
        .unwrap();

    assert!(extended.curve_string().is_none());
    assert!(extended.report().status().is_retained_evidence());
    assert_eq!(
        extended.report().stage(),
        CurveStringExtendStage2::TargetValidation
    );
    assert_eq!(extended.report().source_endpoint_point(), &p(4, 0));
    assert_eq!(extended.report().target_point(), &p(1, 0));
    assert_eq!(extended.report().output_segment_index(), None);
    assert_eq!(extended.report().output_segment_start_point(), None);
    assert_eq!(extended.report().output_segment_end_point(), None);
    assert_eq!(extended.report().source_segment_start_point(), &p(0, 0));
    assert_eq!(extended.report().source_segment_end_point(), &p(4, 0));
    assert_eq!(extended.report().source_param(), Some(&q(1, 4)));
    assert_eq!(extended.report().source_segment_count(), 1);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(extended.report().output_segment_count(), None);
    assert_eq!(extended.report().output_segment_kind_counts(), None);
    assert_eq!(
        extended.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_extend_line_reports_off_support_boundary() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let extended = curve
        .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(5, 1), &policy())
        .unwrap();

    assert!(extended.curve_string().is_none());
    assert!(extended.report().status().is_retained_evidence());
    assert_eq!(extended.report().source_endpoint_point(), &p(4, 0));
    assert_eq!(extended.report().target_point(), &p(5, 1));
    assert_eq!(extended.report().source_segment_kind(), SegmentKind::Line);
    assert_eq!(extended.report().output_segment_index(), None);
    assert_eq!(extended.report().output_segment_kind(), None);
    assert_eq!(extended.report().output_segment_start_point(), None);
    assert_eq!(extended.report().output_segment_end_point(), None);
    assert_eq!(extended.report().source_param(), None);
    assert_eq!(extended.report().source_segment_count(), 1);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(extended.report().output_segment_count(), None);
    assert_eq!(extended.report().output_segment_kind_counts(), None);
    assert_eq!(
        extended.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_extend_arc_endpoint_to_same_circle_target() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap(),
    )])
    .unwrap();

    let extended = curve
        .extend_endpoint_to_point_with_report(CurveStringEndpoint2::End, p(-1, 0), &policy())
        .unwrap();

    assert!(extended.report().status().is_native_exact());
    assert_eq!(
        extended.report().stage(),
        CurveStringExtendStage2::SegmentMaterialization
    );
    assert_eq!(extended.report().endpoint(), CurveStringEndpoint2::End);
    assert_eq!(extended.report().source_segment_index(), 0);
    assert_eq!(extended.report().source_segment_kind(), SegmentKind::Arc);
    assert_eq!(extended.report().output_segment_index(), Some(0));
    assert_eq!(
        extended.report().output_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        extended.report().output_segment_start_point(),
        Some(&p(1, 0))
    );
    assert_eq!(
        extended.report().output_segment_end_point(),
        Some(&p(-1, 0))
    );
    assert_eq!(extended.report().source_segment_start_point(), &p(1, 0));
    assert_eq!(extended.report().source_segment_end_point(), &p(0, 1));
    assert_eq!(extended.report().source_endpoint_point(), &p(0, 1));
    assert_eq!(extended.report().target_point(), &p(-1, 0));
    assert_eq!(extended.report().source_param(), None);
    assert_eq!(extended.report().source_segment_count(), 1);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(extended.report().output_segment_count(), Some(1));
    assert_eq!(
        extended.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 0, arcs: 1 })
    );
    let curve = extended
        .curve_string()
        .expect("same-circle arc extension should materialize");
    let Segment2::Arc(arc) = &curve.segments()[0] else {
        panic!("expected extended arc");
    };
    assert_eq!(arc.start(), &p(1, 0));
    assert_eq!(arc.end(), &p(-1, 0));
    assert_eq!(arc.center(), &p(0, 0));
    assert_eq!(arc.radius_squared(), s(1));
    assert!(!arc.is_clockwise());
}

#[test]
fn curve_string_extend_arc_endpoint_reports_off_circle_boundary() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let extended = curve
        .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(3, 0), &policy())
        .unwrap();

    assert!(extended.curve_string().is_none());
    assert!(extended.report().status().is_retained_evidence());
    assert_eq!(
        extended.report().stage(),
        CurveStringExtendStage2::TargetValidation
    );
    assert_eq!(extended.report().source_endpoint_point(), &p(2, 0));
    assert_eq!(extended.report().target_point(), &p(3, 0));
    assert_eq!(extended.report().output_segment_index(), None);
    assert_eq!(extended.report().output_segment_start_point(), None);
    assert_eq!(extended.report().output_segment_end_point(), None);
    assert_eq!(
        extended.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(extended.report().output_segment_kind_counts(), None);
    assert_eq!(
        extended.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_extend_arc_endpoint_blocks_existing_arc_point() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let extended = curve
        .extend_endpoint_to_point(CurveStringEndpoint2::End, p(1, -1), &policy())
        .unwrap();

    assert!(extended.curve_string().is_none());
    assert!(extended.report().status().is_retained_evidence());
    assert_eq!(extended.report().source_endpoint_point(), &p(2, 0));
    assert_eq!(extended.report().target_point(), &p(1, -1));
    assert_eq!(extended.report().output_segment_index(), None);
    assert_eq!(extended.report().output_segment_start_point(), None);
    assert_eq!(extended.report().output_segment_end_point(), None);
    assert_eq!(
        extended.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_chamfer_line_line_vertex_materializes_exact_segments() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let chamfer = curve
        .chamfer_line_line_vertex_by_parameters_with_report(1, q(3, 4), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().input_path(),
        CurveStringChamferInputPath2::Parameters
    );
    assert_eq!(
        chamfer.report().stage(),
        CurveStringChamferStage2::SegmentMaterialization
    );
    assert_eq!(chamfer.report().previous_segment_index(), 0);
    assert_eq!(chamfer.report().next_segment_index(), 1);
    assert_eq!(chamfer.report().previous_segment_start_point(), &p(0, 0));
    assert_eq!(chamfer.report().previous_segment_end_point(), &p(4, 0));
    assert_eq!(chamfer.report().next_segment_start_point(), &p(4, 0));
    assert_eq!(chamfer.report().next_segment_end_point(), &p(4, 4));
    assert_eq!(chamfer.report().previous_trim().param(), &q(3, 4));
    assert_eq!(chamfer.report().next_trim().param(), &q(1, 4));
    assert_eq!(chamfer.report().previous_cut_point(), Some(&p(3, 0)));
    assert_eq!(chamfer.report().next_cut_point(), Some(&p(4, 1)));
    assert_eq!(chamfer.report().chamfer_segment_index(), Some(1));
    assert_eq!(
        chamfer.report().chamfer_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        chamfer.report().chamfer_segment_start_point(),
        Some(&p(3, 0))
    );
    assert_eq!(chamfer.report().chamfer_segment_end_point(), Some(&p(4, 1)));
    assert_eq!(chamfer.report().source_segment_count(), 2);
    assert_eq!(
        chamfer.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(chamfer.report().output_segment_count(), Some(3));
    assert_eq!(
        chamfer.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 3, arcs: 0 })
    );
    assert_eq!(chamfer.report().trim_segment_report_count(), 2);
    assert_eq!(chamfer.report().segment_reports().len(), 2);
    assert_eq!(
        chamfer.report().segment_reports()[0].source_range().start(),
        &s(0)
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].range_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].range_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].output_segment_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        chamfer.report().segment_reports()[0].output_segment_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].source_range().end(),
        &s(1)
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].range_start_point(),
        Some(&p(4, 1))
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].range_end_point(),
        Some(&p(4, 4))
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].output_segment_index(),
        Some(2)
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].output_segment_start_point(),
        Some(&p(4, 1))
    );
    assert_eq!(
        chamfer.report().segment_reports()[1].output_segment_end_point(),
        Some(&p(4, 4))
    );

    let curve = chamfer
        .curve_string()
        .expect("line-line chamfer should materialize");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.segments()[0].start(), &p(0, 0));
    assert_eq!(curve.segments()[0].end(), &p(3, 0));
    assert_eq!(curve.segments()[1].start(), &p(3, 0));
    assert_eq!(curve.segments()[1].end(), &p(4, 1));
    assert_eq!(curve.segments()[2].start(), &p(4, 1));
    assert_eq!(curve.segments()[2].end(), &p(4, 4));
}

#[test]
fn curve_string_chamfer_line_line_vertex_by_points_materializes_exact_segments() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let chamfer = curve
        .chamfer_line_line_vertex_by_points_with_report(1, &p(3, 0), &p(4, 1), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().input_path(),
        CurveStringChamferInputPath2::Points
    );
    assert_eq!(chamfer.report().previous_trim().param(), &q(3, 4));
    assert_eq!(chamfer.report().next_trim().param(), &q(1, 4));
    assert_eq!(chamfer.report().previous_cut_point(), Some(&p(3, 0)));
    assert_eq!(chamfer.report().next_cut_point(), Some(&p(4, 1)));
    assert_eq!(
        chamfer.report().chamfer_segment_start_point(),
        Some(&p(3, 0))
    );
    assert_eq!(chamfer.report().chamfer_segment_end_point(), Some(&p(4, 1)));
    assert_eq!(chamfer.report().output_segment_count(), Some(3));
    assert_eq!(
        chamfer.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 3, arcs: 0 })
    );
    let curve = chamfer
        .curve_string()
        .expect("point-bearing line-line chamfer should materialize");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.segments()[0].end(), &p(3, 0));
    assert_eq!(curve.segments()[1].start(), &p(3, 0));
    assert_eq!(curve.segments()[1].end(), &p(4, 1));
    assert_eq!(curve.segments()[2].start(), &p(4, 1));
}

#[test]
fn curve_string_chamfer_line_line_vertex_by_points_reports_off_segment_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let chamfer = curve
        .chamfer_line_line_vertex_by_points(1, &p(5, 0), &p(4, 1), &policy())
        .unwrap();

    assert!(chamfer.curve_string().is_none());
    assert!(chamfer.report().status().is_retained_evidence());
    assert_eq!(
        chamfer.report().stage(),
        CurveStringChamferStage2::InputValidation
    );
    assert_eq!(
        chamfer.report().input_path(),
        CurveStringChamferInputPath2::Points
    );
    assert_eq!(
        chamfer.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(chamfer.report().previous_segment_start_point(), &p(0, 0));
    assert_eq!(chamfer.report().previous_segment_end_point(), &p(4, 0));
    assert_eq!(chamfer.report().next_segment_start_point(), &p(4, 0));
    assert_eq!(chamfer.report().next_segment_end_point(), &p(4, 4));
    assert_eq!(chamfer.report().previous_cut_point(), None);
    assert_eq!(chamfer.report().next_cut_point(), None);
    assert_eq!(chamfer.report().chamfer_segment_start_point(), None);
    assert_eq!(chamfer.report().chamfer_segment_end_point(), None);
    assert_eq!(chamfer.report().output_segment_count(), None);
    assert_eq!(
        chamfer.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(chamfer.report().output_segment_kind_counts(), None);
}

#[test]
fn curve_string_chamfer_line_line_vertex_reports_boundary_parameters() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let chamfer = curve
        .chamfer_line_line_vertex_by_parameters(1, s(1), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.curve_string().is_none());
    assert!(chamfer.report().status().is_retained_evidence());
    assert_eq!(
        chamfer.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(chamfer.report().chamfer_segment_index(), None);
    assert_eq!(chamfer.report().chamfer_segment_kind(), None);
    assert_eq!(chamfer.report().chamfer_segment_start_point(), None);
    assert_eq!(chamfer.report().chamfer_segment_end_point(), None);
    assert_eq!(chamfer.report().previous_cut_point(), None);
    assert_eq!(chamfer.report().next_cut_point(), None);
    assert_eq!(chamfer.report().output_segment_count(), None);
}

#[test]
fn curve_string_chamfer_arc_neighbor_reports_unsupported() {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap()),
        line_segment(2, 0, 2, 2),
    ])
    .unwrap();

    let chamfer = curve
        .chamfer_line_line_vertex_by_parameters(1, q(1, 2), q(1, 2), &policy())
        .unwrap();

    assert!(chamfer.curve_string().is_none());
    assert!(chamfer.report().status().is_retained_evidence());
    assert_eq!(
        chamfer.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(chamfer.report().previous_segment_start_point(), &p(0, 0));
    assert_eq!(chamfer.report().previous_segment_end_point(), &p(2, 0));
    assert_eq!(chamfer.report().next_segment_start_point(), &p(2, 0));
    assert_eq!(chamfer.report().next_segment_end_point(), &p(2, 2));
    assert_eq!(chamfer.report().previous_cut_point(), None);
    assert_eq!(chamfer.report().next_cut_point(), None);
    assert_eq!(chamfer.report().chamfer_segment_start_point(), None);
    assert_eq!(chamfer.report().chamfer_segment_end_point(), None);
    assert_eq!(chamfer.report().output_segment_count(), None);
}

#[test]
fn curve_string_fillet_line_line_vertex_materializes_exact_arc() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let fillet = curve
        .fillet_line_line_vertex_by_points_with_report(
            1,
            &p(3, 0),
            &p(4, 1),
            &p(3, 1),
            false,
            &policy(),
        )
        .unwrap();

    assert!(fillet.report().status().is_native_exact());
    assert_eq!(
        fillet.report().input_path(),
        CurveStringFilletInputPath2::Points
    );
    assert_eq!(
        fillet.report().stage(),
        CurveStringFilletStage2::ArcMaterialization
    );
    assert_eq!(fillet.report().previous_segment_index(), 0);
    assert_eq!(fillet.report().next_segment_index(), 1);
    assert_eq!(fillet.report().previous_segment_start_point(), &p(0, 0));
    assert_eq!(fillet.report().previous_segment_end_point(), &p(4, 0));
    assert_eq!(fillet.report().next_segment_start_point(), &p(4, 0));
    assert_eq!(fillet.report().next_segment_end_point(), &p(4, 4));
    assert_eq!(fillet.report().previous_trim().param(), &q(3, 4));
    assert_eq!(fillet.report().next_trim().param(), &q(1, 4));
    assert_eq!(fillet.report().previous_tangent_point(), Some(&p(3, 0)));
    assert_eq!(fillet.report().next_tangent_point(), Some(&p(4, 1)));
    assert_eq!(fillet.report().center(), Some(&p(3, 1)));
    assert_eq!(fillet.report().radius_squared(), Some(&s(1)));
    assert_eq!(fillet.report().fillet_segment_index(), Some(1));
    assert_eq!(
        fillet.report().fillet_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(fillet.report().fillet_segment_start_point(), Some(&p(3, 0)));
    assert_eq!(fillet.report().fillet_segment_end_point(), Some(&p(4, 1)));
    assert_eq!(fillet.report().source_segment_count(), 2);
    assert_eq!(
        fillet.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(fillet.report().output_segment_count(), Some(3));
    assert_eq!(
        fillet.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 1 })
    );
    assert_eq!(fillet.report().segment_reports().len(), 2);
    assert_eq!(
        fillet.report().segment_reports()[0].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        fillet.report().segment_reports()[0].output_segment_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        fillet.report().segment_reports()[0].output_segment_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        fillet.report().segment_reports()[1].output_segment_index(),
        Some(2)
    );
    assert_eq!(
        fillet.report().segment_reports()[1].output_segment_start_point(),
        Some(&p(4, 1))
    );
    assert_eq!(
        fillet.report().segment_reports()[1].output_segment_end_point(),
        Some(&p(4, 4))
    );
    assert_eq!(fillet.report().trim_segment_report_count(), 2);
    assert_eq!(fillet.report().segment_reports().len(), 2);
    assert_eq!(
        fillet.report().segment_reports()[0].source_range().start(),
        &s(0)
    );
    assert_eq!(
        fillet.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
    );
    assert_eq!(
        fillet.report().segment_reports()[0].range_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        fillet.report().segment_reports()[0].range_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        fillet.report().segment_reports()[1].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        fillet.report().segment_reports()[1].source_range().end(),
        &s(1)
    );
    assert_eq!(
        fillet.report().segment_reports()[1].range_start_point(),
        Some(&p(4, 1))
    );
    assert_eq!(
        fillet.report().segment_reports()[1].range_end_point(),
        Some(&p(4, 4))
    );

    let curve = fillet
        .curve_string()
        .expect("line-line fillet should materialize");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.segments()[0].start(), &p(0, 0));
    assert_eq!(curve.segments()[0].end(), &p(3, 0));
    let Segment2::Arc(arc) = &curve.segments()[1] else {
        panic!("fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(3, 0));
    assert_eq!(arc.end(), &p(4, 1));
    assert_eq!(arc.center(), &p(3, 1));
    assert_eq!(arc.radius_squared_ref(), &s(1));
    assert!(!arc.is_clockwise());
    assert_eq!(curve.segments()[2].start(), &p(4, 1));
    assert_eq!(curve.segments()[2].end(), &p(4, 4));
}

#[test]
fn curve_string_fillet_line_line_vertex_by_parameters_materializes_exact_arc() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let fillet = curve
        .fillet_line_line_vertex_by_parameters_with_report(
            1,
            q(3, 4),
            q(1, 4),
            &p(3, 1),
            false,
            &policy(),
        )
        .unwrap();

    assert!(fillet.report().status().is_native_exact());
    assert_eq!(
        fillet.report().input_path(),
        CurveStringFilletInputPath2::Parameters
    );
    assert_eq!(fillet.report().previous_trim().param(), &q(3, 4));
    assert_eq!(fillet.report().next_trim().param(), &q(1, 4));
    assert_eq!(fillet.report().previous_tangent_point(), Some(&p(3, 0)));
    assert_eq!(fillet.report().next_tangent_point(), Some(&p(4, 1)));
    assert_eq!(fillet.report().center(), Some(&p(3, 1)));
    assert_eq!(fillet.report().radius_squared(), Some(&s(1)));
    assert_eq!(fillet.report().fillet_segment_index(), Some(1));
    assert_eq!(fillet.report().fillet_segment_start_point(), Some(&p(3, 0)));
    assert_eq!(fillet.report().fillet_segment_end_point(), Some(&p(4, 1)));
    assert_eq!(fillet.report().output_segment_count(), Some(3));
    assert_eq!(
        fillet.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 1 })
    );

    let curve = fillet
        .curve_string()
        .expect("parameter line-line fillet should materialize");
    assert_eq!(curve.len(), 3);
    assert_eq!(curve.segments()[0].end(), &p(3, 0));
    let Segment2::Arc(arc) = &curve.segments()[1] else {
        panic!("fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(3, 0));
    assert_eq!(arc.end(), &p(4, 1));
    assert_eq!(arc.center(), &p(3, 1));
    assert!(!arc.is_clockwise());
    assert_eq!(curve.segments()[2].start(), &p(4, 1));
}

#[test]
fn curve_string_fillet_reports_radius_mismatch_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let fillet = curve
        .fillet_line_line_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 2), false, &policy())
        .unwrap();

    assert!(fillet.curve_string().is_none());
    assert!(fillet.report().status().is_retained_evidence());
    assert_eq!(
        fillet.report().stage(),
        CurveStringFilletStage2::RadiusAndTangencyValidation
    );
    assert_eq!(fillet.report().previous_segment_start_point(), &p(0, 0));
    assert_eq!(fillet.report().previous_segment_end_point(), &p(4, 0));
    assert_eq!(fillet.report().next_segment_start_point(), &p(4, 0));
    assert_eq!(fillet.report().next_segment_end_point(), &p(4, 4));
    assert_eq!(fillet.report().center(), Some(&p(3, 2)));
    assert_eq!(fillet.report().previous_tangent_point(), None);
    assert_eq!(fillet.report().next_tangent_point(), None);
    assert_eq!(fillet.report().fillet_segment_start_point(), None);
    assert_eq!(fillet.report().fillet_segment_end_point(), None);
    assert_eq!(
        fillet.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(fillet.report().output_segment_kind_counts(), None);
    assert_eq!(fillet.report().blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn curve_string_fillet_reports_wrong_orientation_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let fillet = curve
        .fillet_line_line_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), true, &policy())
        .unwrap();

    assert!(fillet.curve_string().is_none());
    assert!(fillet.report().status().is_retained_evidence());
    assert_eq!(fillet.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(fillet.report().fillet_segment_index(), None);
    assert_eq!(fillet.report().fillet_segment_kind(), None);
    assert_eq!(fillet.report().fillet_segment_start_point(), None);
    assert_eq!(fillet.report().fillet_segment_end_point(), None);
    assert_eq!(fillet.report().previous_tangent_point(), None);
    assert_eq!(fillet.report().next_tangent_point(), None);
    assert_eq!(fillet.report().output_segment_count(), None);
}

#[test]
fn curve_string_fillet_reports_boundary_parameters() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    let fillet = curve
        .fillet_line_line_vertex_by_points(1, &p(4, 0), &p(4, 1), &p(3, 1), false, &policy())
        .unwrap();

    assert!(fillet.curve_string().is_none());
    assert!(fillet.report().status().is_retained_evidence());
    assert_eq!(fillet.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(fillet.report().fillet_segment_index(), None);
    assert_eq!(fillet.report().fillet_segment_start_point(), None);
    assert_eq!(fillet.report().fillet_segment_end_point(), None);
    assert_eq!(fillet.report().previous_tangent_point(), None);
    assert_eq!(fillet.report().next_tangent_point(), None);
    assert_eq!(fillet.report().output_segment_count(), None);
}

#[test]
fn curve_string_trim_materializes_exact_line_subsegment_with_report() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let trim = curve
        .trim_between_parameters_with_report(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(
        trim.report().input_path(),
        CurveStringTrimInputPath2::Parameters
    );
    assert_eq!(trim.report().source_segment_count(), 1);
    assert_eq!(
        trim.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(trim.report().output_segment_count(), Some(1));
    assert_eq!(
        trim.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 0 })
    );
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert_eq!(trim.report().segment_reports()[0].source_segment_index(), 0);
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_start_point(),
        Some(&p(1, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
    );
    assert_eq!(trim.report().segment_reports()[0].range_start_point(), None);
    assert_eq!(trim.report().segment_reports()[0].range_end_point(), None);
    let trimmed = trim.curve_string().expect("line trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(1, 0)));
    assert_eq!(trimmed.end(), Some(&p(3, 0)));
}

#[test]
fn curve_string_trim_materializes_across_line_segments_with_source_ranges() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 4),
        line_segment(4, 4, 8, 4),
    ])
    .unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 2)),
            CurveStringTrimPoint2::new(2, q(1, 2)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().output_segment_count(), Some(3));
    assert_eq!(
        trim.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 3, arcs: 0 }
    );
    assert_eq!(
        trim.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 3, arcs: 0 })
    );
    let reports = trim.report().segment_reports();
    assert_eq!(reports.len(), 3);
    assert_eq!(reports[0].source_range().start(), &q(1, 2));
    assert_eq!(reports[0].source_range().end(), &s(1));
    assert_eq!(reports[1].source_range().start(), &s(0));
    assert_eq!(reports[1].source_range().end(), &s(1));
    assert_eq!(reports[2].source_range().start(), &s(0));
    assert_eq!(reports[2].source_range().end(), &q(1, 2));

    let trimmed = trim
        .curve_string()
        .expect("line-chain trim should materialize");
    assert_eq!(trimmed.len(), 3);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(6, 4)));
}

#[test]
fn curve_string_trim_preserves_whole_arc_segment() {
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let curve = CurveString2::try_new(vec![arc.clone()]).unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, s(0)),
            CurveStringTrimPoint2::new(0, s(1)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().output_segment_count(), Some(1));
    assert_eq!(
        trim.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(
        trim.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 0, arcs: 1 })
    );
    assert_eq!(trim.curve_string().unwrap().segments(), &[arc]);
}

#[test]
fn curve_string_trim_reports_partial_arc_as_unsupported_without_materializing() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy(),
        )
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().input_path(),
        CurveStringTrimInputPath2::Parameters
    );
    assert_eq!(
        trim.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(trim.report().output_segment_count(), None);
    assert_eq!(
        trim.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(trim.report().output_segment_kind_counts(), None);
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert!(
        trim.report().segment_reports()[0]
            .status()
            .is_retained_evidence()
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_kind(),
        None
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_index(),
        None
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_start_point(),
        None
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_end_point(),
        None
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
    );
    assert_eq!(trim.report().segment_reports()[0].range_start_point(), None);
    assert_eq!(trim.report().segment_reports()[0].range_end_point(), None);
}

#[test]
fn curve_string_trim_rejects_reversed_and_out_of_domain_ranges() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    assert_eq!(
        curve
            .trim_between_parameters(
                CurveStringTrimPoint2::new(0, q(3, 4)),
                CurveStringTrimPoint2::new(0, q(1, 4)),
                &policy(),
            )
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
    assert_eq!(
        curve
            .trim_between_parameters(
                CurveStringTrimPoint2::new(0, s(-1)),
                CurveStringTrimPoint2::new(0, q(1, 4)),
                &policy(),
            )
            .unwrap_err(),
        CurveError::InvalidCurveParameter
    );
}

#[test]
fn curve_string_trim_between_points_materializes_line_subsegment() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let trim = curve
        .trim_between_points_with_report(&p(1, 0), &p(3, 0), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(
        trim.report().input_path(),
        CurveStringTrimInputPath2::Points
    );
    assert_eq!(trim.report().start().segment_index(), 0);
    assert_eq!(trim.report().start().param(), &q(1, 4));
    assert_eq!(trim.report().end().segment_index(), 0);
    assert_eq!(trim.report().end().param(), &q(3, 4));
    assert_eq!(trim.report().output_segment_count(), Some(1));
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert_eq!(
        trim.report().segment_reports()[0].range_start_point(),
        Some(&p(1, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].range_end_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_start_point(),
        Some(&p(1, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_end_point(),
        Some(&p(3, 0))
    );
    let trimmed = trim.curve_string().expect("point trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(1, 0)));
    assert_eq!(trimmed.end(), Some(&p(3, 0)));
}

#[test]
fn curve_string_trim_between_points_materializes_partial_arc() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let trim = curve
        .trim_between_points(&p(0, 0), &p(1, -1), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(
        trim.report().input_path(),
        CurveStringTrimInputPath2::Points
    );
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert_eq!(trim.report().output_segment_count(), Some(1));
    assert_eq!(
        trim.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 1 }
    );
    assert_eq!(
        trim.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 0, arcs: 1 })
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].output_segment_end_point(),
        Some(&p(1, -1))
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_segment_end_point(),
        &p(2, 0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &s(0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(1, 2)
    );
    assert_eq!(
        trim.report().segment_reports()[0].range_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        trim.report().segment_reports()[0].range_end_point(),
        Some(&p(1, -1))
    );
    let trimmed = trim
        .curve_string()
        .expect("point-bearing arc trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(0, 0)));
    assert_eq!(trimmed.end(), Some(&p(1, -1)));
    let Segment2::Arc(arc) = &trimmed.segments()[0] else {
        panic!("partial point trim should preserve arc topology");
    };
    assert_eq!(arc.center(), &p(1, 0));
    assert_eq!(arc.radius_squared(), s(1));
}

#[test]
fn curve_string_trim_between_points_accepts_shared_vertex_once() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 2)]).unwrap();

    let trim = curve
        .trim_between_points(&p(2, 0), &p(2, 2), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().start().segment_index(), 1);
    assert_eq!(trim.report().start().param(), &s(0));
    let trimmed = trim
        .curve_string()
        .expect("shared vertex trim should materialize");
    assert_eq!(trimmed.len(), 1);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(2, 2)));
}

#[test]
fn curve_string_trim_between_points_reports_repeated_nonadjacent_point_boundary() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 0, 0),
        line_segment(0, 0, 0, 1),
    ])
    .unwrap();

    let trim = curve
        .trim_between_points(&p(0, 0), &p(0, 1), &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().input_path(),
        CurveStringTrimInputPath2::Points
    );
    assert_eq!(trim.report().output_segment_count(), None);
    assert_eq!(trim.report().blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn curve_string_trim_between_curve_intersections_materializes_line_window() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections_with_report(&start_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(
        trim.report().query_path(),
        CurveStringCurveTrimQueryPath2::Direct
    );
    assert_eq!(
        trim.report().stage(),
        CurveStringCurveTrimStage2::RangeMaterialization
    );
    assert_eq!(
        trim.report().start_intersection_report().query_path(),
        CurveStringIntersectionQueryPath2::Direct
    );
    assert_eq!(
        trim.report().end_intersection_report().query_path(),
        CurveStringIntersectionQueryPath2::Direct
    );
    assert_eq!(
        trim.report()
            .start_intersection_report()
            .candidate_pair_count(),
        1
    );
    assert_eq!(
        trim.report()
            .start_intersection_report()
            .tested_pair_count(),
        1
    );
    assert_eq!(
        trim.report()
            .start_intersection_report()
            .intersection_count(),
        1
    );
    assert_eq!(
        trim.report().end_intersection_report().intersection_count(),
        1
    );
    assert!(trim.report().blocker().is_none());
    assert_eq!(trim.report().start_hits().len(), 1);
    assert_eq!(trim.report().end_hits().len(), 1);
    assert_eq!(trim.report().start_hits()[0].source_segment_index(), 0);
    assert_eq!(trim.report().start_hits()[0].cutter_segment_index(), 0);
    assert_eq!(
        trim.report().start_hits()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trim.report().start_hits()[0].cutter_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trim.report().start_hits()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        trim.report().start_hits()[0].source_segment_end_point(),
        &p(10, 0)
    );
    assert_eq!(
        trim.report().start_hits()[0].cutter_segment_start_point(),
        &p(2, -1)
    );
    assert_eq!(
        trim.report().start_hits()[0].cutter_segment_end_point(),
        &p(2, 1)
    );
    assert_eq!(trim.report().start_hits()[0].source_param(), &q(1, 5));
    assert_eq!(trim.report().start_hits()[0].cutter_param(), &q(1, 2));
    assert_eq!(trim.report().start_hits()[0].point(), &p(2, 0));
    assert_eq!(
        trim.report().start_hits()[0].kind(),
        IntersectionKind::Crossing
    );
    assert_eq!(trim.report().end_hits()[0].source_param(), &q(4, 5));
    assert_eq!(
        trim.report().end_hits()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trim.report().end_hits()[0].cutter_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trim.report().end_hits()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        trim.report().end_hits()[0].source_segment_end_point(),
        &p(10, 0)
    );
    assert_eq!(
        trim.report().end_hits()[0].cutter_segment_start_point(),
        &p(8, -1)
    );
    assert_eq!(
        trim.report().end_hits()[0].cutter_segment_end_point(),
        &p(8, 1)
    );
    assert_eq!(trim.report().end_hits()[0].cutter_param(), &q(1, 2));
    assert_eq!(trim.report().end_hits()[0].point(), &p(8, 0));

    let trim_report = trim
        .report()
        .trim_report()
        .expect("curve trim should retain point trim report");
    assert_eq!(trim_report.input_path(), CurveStringTrimInputPath2::Points);
    assert_eq!(trim_report.start().param(), &q(1, 5));
    assert_eq!(trim_report.end().param(), &q(4, 5));
    assert_eq!(trim_report.output_segment_count(), Some(1));
    let trimmed = trim
        .curve_string()
        .expect("curve-intersection trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(8, 0)));
}

#[test]
fn prepared_curve_string_trim_between_curve_intersections_reuses_cached_evidence() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();
    let policy = policy();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_start = start_cutter.prepare_topology_queries(&policy);
    let prepared_end = end_cutter.prepare_topology_queries(&policy);

    let direct = curve
        .trim_between_curve_intersections(&start_cutter, &end_cutter, &policy)
        .unwrap();
    let prepared = prepared_curve
        .trim_between_prepared_curve_intersections(&prepared_start, &prepared_end, &policy)
        .unwrap();

    assert!(prepared.report().status().is_native_exact());
    assert_eq!(
        prepared.report().query_path(),
        CurveStringCurveTrimQueryPath2::Prepared
    );
    assert_eq!(
        prepared.report().stage(),
        CurveStringCurveTrimStage2::RangeMaterialization
    );
    assert_eq!(
        prepared.report().start_intersection_report().query_path(),
        CurveStringIntersectionQueryPath2::Prepared
    );
    assert_eq!(
        prepared.report().end_intersection_report().query_path(),
        CurveStringIntersectionQueryPath2::Prepared
    );
    assert_eq!(direct.report().start_prepared_cache_report(), None);
    assert_eq!(direct.report().end_prepared_cache_report(), None);
    let start_cache = prepared.report().start_prepared_cache_report().unwrap();
    assert_eq!(
        start_cache.first().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(
        start_cache.second().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(start_cache.first().prepared_segment_count(), 1);
    assert_eq!(start_cache.second().prepared_segment_count(), 1);
    assert_eq!(
        start_cache.first().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        start_cache.second().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(start_cache.first().decided_segment_box_count(), 1);
    assert_eq!(start_cache.second().decided_segment_box_count(), 1);
    assert!(start_cache.first().curve_box_decided());
    assert!(start_cache.second().curve_box_decided());
    let end_cache = prepared.report().end_prepared_cache_report().unwrap();
    assert_eq!(
        end_cache.first().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(
        end_cache.second().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(end_cache.first().prepared_segment_count(), 1);
    assert_eq!(end_cache.second().prepared_segment_count(), 1);
    assert_eq!(
        end_cache.first().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        end_cache.second().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(end_cache.first().decided_segment_box_count(), 1);
    assert_eq!(end_cache.second().decided_segment_box_count(), 1);
    assert!(end_cache.first().curve_box_decided());
    assert!(end_cache.second().curve_box_decided());
    assert_eq!(
        prepared
            .report()
            .start_intersection_report()
            .intersection_count(),
        direct
            .report()
            .start_intersection_report()
            .intersection_count()
    );
    assert_eq!(
        prepared
            .report()
            .end_intersection_report()
            .intersection_count(),
        direct
            .report()
            .end_intersection_report()
            .intersection_count()
    );
    assert_eq!(prepared.report().start_hits(), direct.report().start_hits());
    assert_eq!(prepared.report().end_hits(), direct.report().end_hits());
    assert_eq!(
        prepared.report().trim_report(),
        direct.report().trim_report()
    );
    assert_eq!(
        prepared
            .report()
            .trim_report()
            .unwrap()
            .output_segment_count(),
        Some(1)
    );
    assert_eq!(
        prepared.curve_string().unwrap().segments(),
        direct.curve_string().unwrap().segments()
    );
}

#[test]
fn curve_string_trim_between_curve_intersections_reports_ambiguous_cutter_hits() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let ambiguous_cutter = CurveString2::try_new(vec![
        line_segment(2, -1, 2, 1),
        line_segment(2, 1, 8, 1),
        line_segment(8, 1, 8, -1),
    ])
    .unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(9, -1, 9, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections(&ambiguous_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().stage(),
        CurveStringCurveTrimStage2::HitSelection
    );
    assert_eq!(trim.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(trim.report().start_hits().len(), 2);
    assert_eq!(trim.report().end_hits().len(), 1);
    assert!(
        trim.report()
            .start_hits()
            .iter()
            .all(|hit| hit.source_segment_kind() == SegmentKind::Line
                && hit.cutter_segment_kind() == SegmentKind::Line)
    );
    assert_eq!(
        trim.report().start_intersection_report().query_path(),
        CurveStringIntersectionQueryPath2::Direct
    );
    assert_eq!(
        trim.report()
            .start_intersection_report()
            .intersection_count(),
        2
    );
    assert_eq!(
        trim.report().end_intersection_report().intersection_count(),
        1
    );
    assert!(trim.report().trim_report().is_none());
}

#[test]
fn curve_string_trim_between_curve_intersections_reports_overlap_blocker() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let overlapping_cutter = CurveString2::try_new(vec![line_segment(2, 0, 4, 0)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections(&overlapping_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert!(trim.report().start_hits().is_empty());
    assert_eq!(trim.report().end_hits().len(), 1);
    assert!(trim.report().trim_report().is_none());
}

#[test]
fn curve_string_trim_inside_region_materializes_inside_window() {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 6, 1)]).unwrap();
    let region = rectangle_region(0, 0, 4, 4);

    let trimmed = curve
        .trim_inside_region_with_report(&region, &policy())
        .unwrap();

    assert!(trimmed.report().status().is_native_exact());
    assert_eq!(
        trimmed.report().query_path(),
        CurveStringRegionTrimQueryPath2::Direct
    );
    assert_eq!(
        trimmed.report().stage(),
        CurveStringRegionTrimStage2::OutputMaterialization
    );
    assert_eq!(trimmed.report().source_segment_count(), 1);
    assert_eq!(
        trimmed.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(trimmed.report().region_material_contour_count(), 1);
    assert_eq!(trimmed.report().region_hole_contour_count(), 0);
    assert_eq!(trimmed.report().region_material_segment_count(), 4);
    assert_eq!(trimmed.report().region_hole_segment_count(), 0);
    assert_eq!(
        trimmed.report().region_material_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        trimmed.report().region_hole_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 0 }
    );
    assert_eq!(trimmed.report().output_curve_string_count(), Some(1));
    assert_eq!(trimmed.report().output_segment_count(), Some(1));
    assert_eq!(
        trimmed.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 0 })
    );
    assert_eq!(trimmed.report().interval_candidate_count(), 3);
    assert_eq!(trimmed.report().interval_classification_count(), 3);
    assert_eq!(trimmed.report().boundary_hit_count(), 2);
    assert_eq!(trimmed.report().boundary_point_relation_count(), 2);
    assert_eq!(trimmed.report().boundary_overlap_relation_count(), 0);
    assert_eq!(trimmed.report().boundary_uncertain_relation_count(), 0);
    assert_eq!(trimmed.report().boundary_hits().len(), 2);
    assert_eq!(
        trimmed.report().boundary_hits()[0].region_contour_role(),
        RegionContourRole::Material
    );
    let left_hit = trimmed
        .report()
        .boundary_hits()
        .iter()
        .find(|hit| hit.point() == &p(0, 1))
        .expect("left boundary hit is retained");
    assert_eq!(left_hit.source_segment_kind(), SegmentKind::Line);
    assert_eq!(left_hit.source_segment_start_point(), &p(-2, 1));
    assert_eq!(left_hit.source_segment_end_point(), &p(6, 1));
    assert_eq!(left_hit.source_param(), &q(1, 4));
    assert_eq!(left_hit.region_segment_index(), 3);
    assert_eq!(left_hit.region_segment_kind(), SegmentKind::Line);
    assert_eq!(left_hit.region_segment_start_point(), &p(0, 4));
    assert_eq!(left_hit.region_segment_end_point(), &p(0, 0));
    assert_eq!(left_hit.region_param(), &q(3, 4));
    let right_hit = trimmed
        .report()
        .boundary_hits()
        .iter()
        .find(|hit| hit.point() == &p(4, 1))
        .expect("right boundary hit is retained");
    assert_eq!(right_hit.source_segment_kind(), SegmentKind::Line);
    assert_eq!(right_hit.source_segment_start_point(), &p(-2, 1));
    assert_eq!(right_hit.source_segment_end_point(), &p(6, 1));
    assert_eq!(right_hit.source_param(), &q(3, 4));
    assert_eq!(right_hit.region_segment_index(), 1);
    assert_eq!(right_hit.region_segment_kind(), SegmentKind::Line);
    assert_eq!(right_hit.region_segment_start_point(), &p(4, 0));
    assert_eq!(right_hit.region_segment_end_point(), &p(4, 4));
    assert_eq!(right_hit.region_param(), &q(1, 4));
    assert_eq!(trimmed.report().interval_reports().len(), 3);
    assert_eq!(
        trimmed.report().interval_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trimmed.report().interval_reports()[0].source_segment_start_point(),
        &p(-2, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[0].source_segment_end_point(),
        &p(6, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[0].output_segment_kind(),
        None
    );
    assert_eq!(
        trimmed.report().interval_reports()[0].output_segment_start_point(),
        None
    );
    assert_eq!(
        trimmed.report().interval_reports()[0].output_segment_end_point(),
        None
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].range_start_point(),
        &p(0, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].source_segment_start_point(),
        &p(-2, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].source_segment_end_point(),
        &p(6, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].range_end_point(),
        &p(4, 1)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].location(),
        Some(RegionPointLocation::Inside)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].output_curve_string_index(),
        Some(0)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].output_segment_index(),
        Some(0)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].output_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].output_segment_start_point(),
        Some(&p(0, 1))
    );
    assert_eq!(
        trimmed.report().interval_reports()[1].output_segment_end_point(),
        Some(&p(4, 1))
    );

    assert_eq!(trimmed.curve_strings().len(), 1);
    assert_eq!(trimmed.curve_strings()[0].len(), 1);
    assert_line(&trimmed.curve_strings()[0].segments()[0], p(0, 1), p(4, 1));
}

#[test]
fn prepared_curve_string_trim_inside_region_matches_direct_result() {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)]).unwrap();
    let first = rectangle_region(0, 0, 2, 2);
    let second = rectangle_region(4, 0, 6, 2);
    let region = Region2::from_material_contours(vec![
        first.material_contours()[0].clone(),
        second.material_contours()[0].clone(),
    ]);
    let policy = policy();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_region = region.prepare_topology_queries(&policy);

    let direct = curve.trim_inside_region(&region, &policy).unwrap();
    let prepared = prepared_curve
        .trim_inside_prepared_region(&prepared_region, &policy)
        .unwrap();

    assert!(prepared.report().status().is_native_exact());
    assert_eq!(
        prepared.report().query_path(),
        CurveStringRegionTrimQueryPath2::Prepared
    );
    assert_eq!(
        prepared.report().stage(),
        CurveStringRegionTrimStage2::OutputMaterialization
    );
    assert_eq!(direct.report().prepared_cache_report(), None);
    let prepared_cache = prepared.report().prepared_cache_report().unwrap();
    assert_eq!(
        prepared_cache.source().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(prepared_cache.source().prepared_segment_count(), 1);
    assert_eq!(
        prepared_cache.source().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(prepared_cache.source().decided_segment_box_count(), 1);
    assert_eq!(prepared_cache.source().undecided_segment_box_count(), 0);
    assert!(prepared_cache.source().curve_box_decided());
    assert_eq!(
        prepared_cache.region().freshness(),
        CurveStringPreparedCacheFreshness2::BorrowedCurrentSource
    );
    assert_eq!(prepared_cache.region().prepared_contour_count(), 2);
    assert_eq!(prepared_cache.region().prepared_material_segment_count(), 8);
    assert_eq!(
        prepared_cache
            .region()
            .prepared_material_segment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(prepared_cache.region().prepared_hole_segment_count(), 0);
    assert_eq!(
        prepared_cache.region().prepared_hole_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 0 }
    );
    assert_eq!(prepared_cache.region().prepared_segment_count(), 8);
    assert_eq!(
        prepared_cache.region().prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(prepared_cache.region().decided_segment_box_count(), 8);
    assert_eq!(prepared_cache.region().undecided_segment_box_count(), 0);
    assert!(prepared_cache.region().region_box_decided());
    assert_eq!(
        direct.report().boundary_predicate_path(),
        CurveStringRegionTrimBoundaryPredicatePath2::AabbFilteredExactSegmentIntersections
    );
    assert_eq!(
        prepared.report().boundary_predicate_path(),
        CurveStringRegionTrimBoundaryPredicatePath2::AabbFilteredExactSegmentIntersections
    );
    assert_eq!(direct.report().boundary_candidate_pair_count(), 8);
    assert_eq!(direct.report().boundary_skipped_aabb_pair_count(), 4);
    assert_eq!(direct.report().boundary_tested_pair_count(), 4);
    assert_eq!(prepared.report().boundary_candidate_pair_count(), 8);
    assert_eq!(prepared.report().boundary_skipped_aabb_pair_count(), 4);
    assert_eq!(prepared.report().boundary_tested_pair_count(), 4);
    assert_eq!(prepared.report().region_material_segment_count(), 8);
    assert_eq!(prepared.report().region_hole_segment_count(), 0);
    assert_eq!(
        prepared.report().region_material_segment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(
        prepared.report().region_hole_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 0 }
    );
    assert_eq!(prepared.report().output_segment_count(), Some(2));
    assert_eq!(
        prepared.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    assert_eq!(direct.report().interval_candidate_count(), 5);
    assert_eq!(direct.report().interval_classification_count(), 5);
    assert_eq!(prepared.report().interval_candidate_count(), 5);
    assert_eq!(prepared.report().interval_classification_count(), 5);
    assert_eq!(
        prepared.report().boundary_hit_count(),
        direct.report().boundary_hit_count()
    );
    assert_eq!(
        prepared.report().boundary_point_relation_count(),
        direct.report().boundary_point_relation_count()
    );
    assert_eq!(
        prepared.report().boundary_overlap_relation_count(),
        direct.report().boundary_overlap_relation_count()
    );
    assert_eq!(
        prepared.report().boundary_uncertain_relation_count(),
        direct.report().boundary_uncertain_relation_count()
    );
    assert_eq!(
        prepared.report().boundary_hit_count(),
        prepared.report().boundary_hits().len()
    );
    assert_eq!(
        prepared.report().output_segment_count(),
        direct.report().output_segment_count()
    );
    assert_eq!(
        prepared.report().output_segment_kind_counts(),
        direct.report().output_segment_kind_counts()
    );
    assert_eq!(prepared.curve_strings(), direct.curve_strings());
    assert_eq!(
        prepared.report().boundary_hits(),
        direct.report().boundary_hits()
    );
    assert!(
        prepared
            .report()
            .boundary_hits()
            .iter()
            .all(|hit| hit.source_segment_kind() == SegmentKind::Line
                && hit.region_segment_kind() == SegmentKind::Line)
    );
    assert_eq!(
        prepared.report().interval_reports(),
        direct.report().interval_reports()
    );
}

#[test]
fn curve_string_trim_inside_region_splits_disconnected_inside_windows() {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)]).unwrap();
    let first = rectangle_region(0, 0, 2, 2);
    let second = rectangle_region(4, 0, 6, 2);
    let region = Region2::from_material_contours(vec![
        first.material_contours()[0].clone(),
        second.material_contours()[0].clone(),
    ]);

    let trimmed = curve.trim_inside_region(&region, &policy()).unwrap();

    assert!(trimmed.report().status().is_native_exact());
    assert_eq!(trimmed.report().output_curve_string_count(), Some(2));
    assert_eq!(trimmed.report().output_segment_count(), Some(2));
    assert_eq!(trimmed.report().region_material_segment_count(), 8);
    assert_eq!(trimmed.report().region_hole_segment_count(), 0);
    assert_eq!(
        trimmed.report().boundary_predicate_path(),
        CurveStringRegionTrimBoundaryPredicatePath2::AabbFilteredExactSegmentIntersections
    );
    assert_eq!(trimmed.report().boundary_candidate_pair_count(), 8);
    assert_eq!(trimmed.report().boundary_skipped_aabb_pair_count(), 4);
    assert_eq!(trimmed.report().boundary_tested_pair_count(), 4);
    assert_eq!(trimmed.report().interval_candidate_count(), 5);
    assert_eq!(trimmed.report().interval_classification_count(), 5);
    let output_intervals: Vec<_> = trimmed
        .report()
        .interval_reports()
        .iter()
        .filter(|interval| interval.location() == Some(RegionPointLocation::Inside))
        .collect();
    assert_eq!(output_intervals.len(), 2);
    assert_eq!(output_intervals[0].output_curve_string_index(), Some(0));
    assert_eq!(output_intervals[0].output_segment_index(), Some(0));
    assert_eq!(
        output_intervals[0].output_segment_start_point(),
        Some(&p(0, 1))
    );
    assert_eq!(
        output_intervals[0].output_segment_end_point(),
        Some(&p(2, 1))
    );
    assert_eq!(output_intervals[1].output_curve_string_index(), Some(1));
    assert_eq!(output_intervals[1].output_segment_index(), Some(0));
    assert_eq!(
        output_intervals[1].output_segment_start_point(),
        Some(&p(4, 1))
    );
    assert_eq!(
        output_intervals[1].output_segment_end_point(),
        Some(&p(6, 1))
    );
    assert_eq!(trimmed.curve_strings().len(), 2);
    assert_line(&trimmed.curve_strings()[0].segments()[0], p(0, 1), p(2, 1));
    assert_line(&trimmed.curve_strings()[1].segments()[0], p(4, 1), p(6, 1));
}

#[test]
fn curve_string_trim_inside_region_respects_holes() {
    let material = rectangle_region(0, 0, 10, 4).material_contours()[0].clone();
    let hole = rectangle_region(4, 0, 6, 4).material_contours()[0].clone();
    let region = Region2::new(vec![material], vec![hole]);
    let curve = CurveString2::try_new(vec![line_segment(1, 2, 9, 2)]).unwrap();

    let trimmed = curve.trim_inside_region(&region, &policy()).unwrap();

    assert!(trimmed.report().status().is_native_exact());
    assert_eq!(trimmed.report().boundary_candidate_pair_count(), 8);
    assert_eq!(trimmed.report().boundary_skipped_aabb_pair_count(), 6);
    assert_eq!(trimmed.report().boundary_tested_pair_count(), 2);
    assert_eq!(trimmed.report().region_hole_contour_count(), 1);
    assert_eq!(trimmed.report().region_material_segment_count(), 4);
    assert_eq!(trimmed.report().region_hole_segment_count(), 4);
    assert_eq!(
        trimmed.report().region_material_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        trimmed.report().region_hole_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(trimmed.report().output_curve_string_count(), Some(2));
    assert_eq!(trimmed.report().output_segment_count(), Some(2));
    assert_eq!(
        trimmed.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    assert_eq!(trimmed.report().interval_candidate_count(), 3);
    assert_eq!(trimmed.report().interval_classification_count(), 3);
    assert_eq!(trimmed.report().boundary_hit_count(), 2);
    assert_eq!(
        trimmed
            .report()
            .boundary_hits()
            .iter()
            .filter(|hit| hit.region_contour_role() == RegionContourRole::Hole)
            .count(),
        2
    );
    assert_line(&trimmed.curve_strings()[0].segments()[0], p(1, 2), p(4, 2));
    assert_line(&trimmed.curve_strings()[1].segments()[0], p(6, 2), p(9, 2));
}

#[test]
fn curve_string_trim_inside_region_reports_boundary_overlap_blocker() {
    let region = rectangle_region(0, 0, 4, 4);
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let trimmed = curve.trim_inside_region(&region, &policy()).unwrap();

    assert!(trimmed.curve_strings().is_empty());
    assert!(trimmed.report().status().is_retained_evidence());
    assert_eq!(
        trimmed.report().stage(),
        CurveStringRegionTrimStage2::BoundaryCollection
    );
    assert_eq!(
        trimmed.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(trimmed.report().region_material_segment_count(), 4);
    assert_eq!(trimmed.report().region_hole_segment_count(), 0);
    assert_eq!(
        trimmed.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 0 }
    );
    assert_eq!(
        trimmed.report().region_material_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(trimmed.report().output_segment_kind_counts(), None);
    assert_eq!(trimmed.report().interval_candidate_count(), 0);
    assert_eq!(trimmed.report().interval_classification_count(), 0);
    assert_eq!(trimmed.report().boundary_hit_count(), 0);
    assert_eq!(trimmed.report().boundary_point_relation_count(), 0);
    assert_eq!(trimmed.report().boundary_overlap_relation_count(), 1);
    assert_eq!(trimmed.report().boundary_uncertain_relation_count(), 0);
    assert_eq!(trimmed.report().output_curve_string_count(), None);
    assert_eq!(trimmed.report().output_segment_count(), None);
}

#[test]
fn prepared_curve_string_intersections_match_plain_sparse_scan() {
    let curve = sparse_zigzag(80);
    let cutter = CurveString2::try_new(vec![line_segment(121, -2, 121, 3)]).unwrap();
    let policy = policy();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);

    assert_eq!(prepared_curve.curve_string(), &curve);
    assert!(prepared_curve.curve_box().is_some());
    assert_eq!(prepared_curve.segment_boxes().len(), curve.segments().len());

    let plain_events = curve.intersect_curve_string(&cutter, &policy).unwrap();
    let prepared_events = prepared_curve
        .intersect_prepared_curve_string(&prepared_cutter, &policy)
        .unwrap();
    let mixed_events = prepared_curve
        .intersect_curve_string(&cutter, &policy)
        .unwrap();

    assert_eq!(prepared_events, plain_events);
    assert_eq!(mixed_events, plain_events);
    assert_eq!(prepared_events.len(), 1);
}

#[test]
fn prepared_curve_string_intersections_preserve_line_arc_hits() {
    let line_curve = CurveString2::try_new(vec![line_segment(1, -2, 1, 2)]).unwrap();
    let arc_curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let policy = policy();
    let prepared_line = line_curve.prepare_topology_queries(&policy);
    let prepared_arc = arc_curve.prepare_topology_queries(&policy);

    let plain_events = line_curve
        .intersect_curve_string(&arc_curve, &policy)
        .unwrap();
    let prepared_events = prepared_line
        .intersect_prepared_curve_string(&prepared_arc, &policy)
        .unwrap();

    assert_eq!(prepared_events, plain_events);
    assert_eq!(prepared_events.len(), 1);
    let SegmentIntersection::LineArc {
        order,
        result: LineArcIntersection::Point(hit),
    } = &prepared_events[0].relation
    else {
        panic!("expected prepared line-arc point event");
    };
    assert_eq!(*order, LineArcOrder::LineThenArc);
    assert_eq!(hit.point, p(1, -1));
}

#[test]
fn prepared_segment_pair_intersection_matches_plain_segment_relation() {
    let line = Segment2::Line(LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap());
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let prepared_line = hypercurve::PreparedSegment2::from_segment(&line);
    let prepared_arc = hypercurve::PreparedSegment2::from_segment(&arc);
    let policy = policy();

    let plain = line.intersect_segment(&arc, &policy).unwrap();
    let prepared = prepared_line
        .intersect_prepared_segment(&prepared_arc, &policy)
        .unwrap();

    assert_eq!(prepared, plain);
    let SegmentIntersection::LineArc {
        order: LineArcOrder::LineThenArc,
        result: LineArcIntersection::Point(hit),
    } = prepared
    else {
        panic!("expected prepared line-arc pair to preserve point relation");
    };
    assert_eq!(hit.point, p(1, -1));
}

#[test]
fn prepared_curve_string_intersections_skip_decided_disjoint_boxes() {
    let first = CurveString2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), s(0)),
        BulgeVertex2::new(p(2, 0), s(0)),
    ])
    .unwrap();
    let second = CurveString2::from_bulge_vertices(&[
        BulgeVertex2::new(p(10, 10), s(0)),
        BulgeVertex2::new(p(12, 10), s(0)),
    ])
    .unwrap();
    let policy = policy();
    let prepared_first = first.prepare_topology_queries(&policy);
    let prepared_second = second.prepare_topology_queries(&policy);

    assert!(
        prepared_first
            .intersect_prepared_curve_string(&prepared_second, &policy)
            .unwrap()
            .is_empty()
    );
}
