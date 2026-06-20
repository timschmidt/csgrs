use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    ExactCurveArrangementArrangedEndpointDegree2, ExactCurveArrangementAttempt2,
    ExactCurveArrangementRequest2, ExactCurveArrangementResult2,
    ExactCurveArrangementSourceAabbStatus2, ExactCurveArrangementSourceEndpoint2,
    ExactCurveArrangementSplitCandidateAabbStatus2, ExactCurveArrangementSplitRelationClass2,
    FillRule, FiniteProjectionOptions, Real, Region2, RegionBoundaryContourBuildPredicatePath2,
    RegionBoundaryContourBuildStage2, RegionBoundaryContourRole2,
    RegionLineSegmentArrangedEndpoint2, RegionLineSegmentEndpointGraphPredicatePath2,
    RegionLineSegmentRegionBuildStage2, RegionLineSegmentRingAssemblyPredicatePath2,
    RegionLineSegmentSplitPredicatePath2, RegionPointLocation, RegionView2, RetainedTopologyStatus,
    Segment2, SegmentKind, SegmentKindCounts, UncertaintyReason, finite_polyline_vertex_centroid,
    finite_ring_signed_area, try_finite_polyline_vertex_centroid, try_finite_ring_signed_area,
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
    CurvePolicy::certified()
}

fn evaluate_unordered_line_segments(
    segments: Vec<hypercurve::LineSeg2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<ExactCurveArrangementResult2> {
    ExactCurveArrangementAttempt2::new(ExactCurveArrangementRequest2::from_unordered_line_segments(
        segments, fill_rule,
    ))
    .evaluate_owned(policy)
}

fn evaluate_borrowed_unordered_line_segments(
    segments: &[hypercurve::LineSeg2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<ExactCurveArrangementResult2> {
    ExactCurveArrangementAttempt2::new(
        ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(segments, fill_rule),
    )
    .evaluate_owned(policy)
}

fn evaluate_unordered_segments(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<ExactCurveArrangementResult2> {
    ExactCurveArrangementAttempt2::new(ExactCurveArrangementRequest2::from_unordered_segments(
        segments, fill_rule,
    ))
    .evaluate_owned(policy)
}

fn evaluate_borrowed_unordered_segments(
    segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> hypercurve::CurveResult<ExactCurveArrangementResult2> {
    ExactCurveArrangementAttempt2::new(
        ExactCurveArrangementRequest2::from_borrowed_unordered_segments(segments, fill_rule),
    )
    .evaluate_owned(policy)
}

#[test]
fn empty_region_classifies_everything_outside() {
    let region = Region2::empty();

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
    let region = Region2::from_material_contours(vec![rectangle(0, 0, 10, 10)]);

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
    let region = Region2::new(
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
    let region = Region2::from_material_contours(vec![
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
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

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
    let region = match Region2::from_boundary_contours(
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
fn boundary_contour_region_report_assigns_material_and_hole_roles() {
    let built = Region2::from_boundary_contours_with_report(
        vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        &policy(),
    )
    .unwrap();

    assert!(built.status().is_native_exact());
    assert_eq!(
        built.stage(),
        RegionBoundaryContourBuildStage2::RoleAssignment
    );
    assert_eq!(
        built.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(built.source_contour_count(), 2);
    assert_eq!(built.source_segment_count(), 8);
    assert_eq!(built.validation_candidate_pair_count(), 1);
    assert_eq!(built.validation_tested_pair_count(), 1);
    assert_eq!(built.validation_intersection_event_count(), 0);
    assert_eq!(built.nesting_classification_count(), 2);
    assert_eq!(built.blocker_first_contour_index(), None);
    assert_eq!(built.blocker_second_contour_index(), None);
    assert_eq!(built.output_contour_count(), Some(2));
    assert_eq!(built.output_segment_count(), Some(8));
    assert_eq!(built.material_contour_count(), Some(1));
    assert_eq!(built.hole_contour_count(), Some(1));
    assert_eq!(built.material_segment_count(), Some(4));
    assert_eq!(built.hole_segment_count(), Some(4));
    assert_eq!(built.blocker(), None);
    assert_eq!(built.role_reports().len(), 2);

    let outer = &built.role_reports()[0];
    assert_eq!(outer.source_contour_index(), 0);
    assert_eq!(outer.source_segment_count(), 4);
    assert_eq!(outer.source_fill_rule(), FillRule::NonZero);
    assert_eq!(outer.nesting_sample_point(), &p(0, 0));
    assert!(outer.containing_contour_indices().is_empty());
    assert_eq!(outer.nesting_depth(), 0);
    assert_eq!(outer.role(), RegionBoundaryContourRole2::Material);
    assert_eq!(outer.output_role_index(), 0);
    assert!(outer.status().is_native_exact());

    let hole = &built.role_reports()[1];
    assert_eq!(hole.source_contour_index(), 1);
    assert_eq!(hole.source_segment_count(), 4);
    assert_eq!(hole.source_fill_rule(), FillRule::NonZero);
    assert_eq!(hole.nesting_sample_point(), &p(3, 3));
    assert_eq!(hole.containing_contour_indices(), &[0]);
    assert_eq!(hole.nesting_depth(), 1);
    assert_eq!(hole.role(), RegionBoundaryContourRole2::Hole);
    assert_eq!(hole.output_role_index(), 0);
    assert!(hole.status().is_native_exact());

    let region = built.region().unwrap();
    assert_eq!(region.material_contours().len(), 1);
    assert_eq!(region.hole_contours().len(), 1);
    assert_eq!(
        built.region_classification(),
        Classification::Decided(region)
    );
    assert_eq!(
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );

    let owned_report = built.clone().into_report();
    assert_eq!(&owned_report, built.report());
    let (owned_region, owned_parts_report) = built.clone().into_parts();
    assert_eq!(owned_region.as_ref(), built.region());
    assert_eq!(&owned_parts_report, built.report());
    assert_eq!(
        built.clone().into_region_classification(),
        Classification::Decided(region.clone())
    );
}

#[test]
fn borrowed_boundary_contours_build_region_with_report() {
    let contours = vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)];
    let built = Region2::from_boundary_contours_borrowed_with_report(&contours, &policy()).unwrap();

    assert!(built.status().is_native_exact());
    assert_eq!(
        built.stage(),
        RegionBoundaryContourBuildStage2::RoleAssignment
    );
    assert_eq!(
        built.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(built.source_contour_count(), 2);
    assert_eq!(built.source_segment_count(), 8);
    assert_eq!(built.output_contour_count(), Some(2));
    assert_eq!(built.output_segment_count(), Some(8));
    assert_eq!(built.material_contour_count(), Some(1));
    assert_eq!(built.hole_contour_count(), Some(1));
    assert_eq!(built.blocker(), None);
    assert_eq!(built.role_reports().len(), 2);
    assert_eq!(
        built.role_reports()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        built.role_reports()[1].role(),
        RegionBoundaryContourRole2::Hole
    );

    assert_eq!(contours.len(), 2);
    let region = built.region().unwrap();
    assert_eq!(region.material_contours().len(), 1);
    assert_eq!(region.hole_contours().len(), 1);
}

#[test]
fn borrowed_boundary_contours_convenience_returns_decided_region() {
    let contours = vec![rectangle(0, 0, 5, 5), rectangle(1, 1, 3, 3)];
    let region = match Region2::from_boundary_contours_borrowed(&contours, &policy()).unwrap() {
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
        Region2::from_boundary_contours(
            vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        Region2::from_boundary_contours(
            vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn boundary_contour_region_report_blocks_crossing_roles() {
    let built = Region2::from_boundary_contours_with_report(
        vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
        &policy(),
    )
    .unwrap();

    assert!(built.region().is_none());
    assert!(built.status().is_retained_evidence());
    assert_eq!(
        built.stage(),
        RegionBoundaryContourBuildStage2::NestingValidation
    );
    assert_eq!(
        built.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(built.source_contour_count(), 2);
    assert_eq!(built.source_segment_count(), 8);
    assert_eq!(built.validation_candidate_pair_count(), 1);
    assert_eq!(built.validation_tested_pair_count(), 1);
    assert_eq!(built.validation_intersection_event_count(), 2);
    assert_eq!(built.nesting_classification_count(), 0);
    assert_eq!(built.blocker_first_contour_index(), Some(0));
    assert_eq!(built.blocker_second_contour_index(), Some(1));
    assert_eq!(built.output_contour_count(), None);
    assert_eq!(built.output_segment_count(), None);
    assert_eq!(built.material_contour_count(), None);
    assert_eq!(built.hole_contour_count(), None);
    assert_eq!(built.material_segment_count(), None);
    assert_eq!(built.hole_segment_count(), None);
    assert!(built.role_reports().is_empty());
}

#[test]
fn boundary_contour_region_report_blocks_touching_roles_with_source_pair() {
    let built = Region2::from_boundary_contours_with_report(
        vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
        &policy(),
    )
    .unwrap();

    assert!(built.region().is_none());
    assert!(built.status().is_retained_evidence());
    assert_eq!(
        built.stage(),
        RegionBoundaryContourBuildStage2::NestingValidation
    );
    assert_eq!(
        built.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(built.validation_candidate_pair_count(), 1);
    assert_eq!(built.validation_tested_pair_count(), 1);
    assert_eq!(built.validation_intersection_event_count(), 7);
    assert_eq!(built.nesting_classification_count(), 0);
    assert_eq!(built.blocker_first_contour_index(), Some(0));
    assert_eq!(built.blocker_second_contour_index(), Some(1));
    assert_eq!(built.output_contour_count(), None);
    assert!(built.role_reports().is_empty());
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
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RegionRoleAssignment)
    );
    assert_eq!(built.source_segment_count(), 4);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(
        built.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(built.split_candidate_pair_count(), Some(6));
    assert_eq!(
        built.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(
        built.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        built.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(2));
    assert_eq!(built.split_tested_pair_count(), Some(4));
    assert_eq!(built.split_intersection_event_count(), Some(4));
    assert_eq!(built.split_point_relation_count(), Some(4));
    assert_eq!(built.split_overlap_relation_count(), Some(0));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    let split_points = built.split_intersection_points().unwrap();
    let split_reports = built.split_intersection_reports().unwrap();
    assert_eq!(split_points.len(), 4);
    assert_eq!(split_reports.len(), 4);
    assert!(split_points.contains(&p(0, 0)));
    assert!(split_points.contains(&p(4, 0)));
    assert!(split_points.contains(&p(0, 4)));
    assert!(split_points.contains(&p(4, 4)));
    assert_eq!(built.split_output_segment_count(), Some(4));
    assert_eq!(built.split_blocker_first_source_segment_index(), None);
    assert_eq!(built.split_blocker_first_source_segment_kind(), None);
    assert_eq!(built.split_blocker_first_source_start_point(), None);
    assert_eq!(built.split_blocker_first_source_end_point(), None);
    assert_eq!(built.split_blocker_second_source_segment_index(), None);
    assert_eq!(built.split_blocker_second_source_segment_kind(), None);
    assert_eq!(built.split_blocker_second_source_start_point(), None);
    assert_eq!(built.split_blocker_second_source_end_point(), None);
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(4));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(2));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(built.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(built.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(built.endpoint_graph_blocker_point(), None);
    assert_eq!(built.reversed_source_segment_count(), Some(2));
    assert_eq!(built.output_ring_count(), Some(1));
    assert_eq!(built.output_boundary_segment_count(), Some(4));
    assert_eq!(
        built.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    let arranged_sources = built.arranged_source_reports().unwrap();
    assert_eq!(built.arranged_source_report_count(), Some(4));
    assert_eq!(arranged_sources.len(), 4);
    assert_eq!(arranged_sources[0].source_segment_index(), 0);
    assert_eq!(arranged_sources[0].source_segment_kind(), SegmentKind::Line);
    assert_eq!(arranged_sources[0].source_segment_start_point(), &p(0, 0));
    assert_eq!(arranged_sources[0].source_segment_end_point(), &p(4, 0));
    assert_eq!(
        arranged_sources[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_sources[0].source_range(),
        &hypercurve::ParamRange::new(s(0), s(1))
    );
    let source_reports = built.source_reports().unwrap();
    assert_eq!(built.source_report_count(), Some(4));
    assert_eq!(source_reports.len(), 4);
    assert_eq!(source_reports[0].source_segment_index(), 0);
    assert_eq!(source_reports[0].source_segment_start_point(), &p(0, 0));
    assert_eq!(source_reports[0].source_segment_end_point(), &p(4, 0));
    assert_eq!(
        source_reports[0].source_range(),
        &hypercurve::ParamRange::new(s(0), s(1))
    );
    assert!(!source_reports[0].reversed());
    assert_eq!(source_reports[0].source_segment_kind(), SegmentKind::Line);
    assert_eq!(source_reports[0].output_segment_kind(), SegmentKind::Line);
    assert_eq!(source_reports[1].source_segment_index(), 3);
    assert!(!source_reports[1].reversed());
    assert_eq!(source_reports[2].source_segment_index(), 1);
    assert!(source_reports[2].reversed());
    assert_eq!(source_reports[2].source_segment_start_point(), &p(0, 4));
    assert_eq!(source_reports[2].source_segment_end_point(), &p(4, 4));

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(ring_cache.arranged_source_reports(), arranged_sources);
    assert_eq!(ring_cache.source_reports(), source_reports);

    let fragment_cache = ring_cache.arranged_fragment_cache();
    assert_eq!(fragment_cache.arranged_fragment_count(), 4);
    assert_eq!(fragment_cache.source_ref_count(), 4);
    assert_eq!(
        fragment_cache.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        fragment_cache.arranged_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(fragment_cache.max_source_ref_count(), 1);
    assert_eq!(fragment_cache.fragments().len(), 4);
    assert_eq!(fragment_cache.fragments()[0].arranged_segment_index(), 0);
    assert_eq!(
        fragment_cache.fragments()[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(fragment_cache.fragments()[0].output_start_point(), &p(0, 0));
    assert_eq!(fragment_cache.fragments()[0].output_end_point(), &p(4, 0));
    assert_eq!(fragment_cache.fragments()[0].source_refs().len(), 1);
    assert_eq!(
        fragment_cache.fragments()[0].source_refs()[0].arranged_source_report_index(),
        0
    );
    assert_eq!(
        fragment_cache.fragments()[0].source_refs()[0].source_segment_index(),
        0
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_kind_bucket_cache()
            .line_fragment_ref_count(),
        4
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_status_bucket_cache()
            .native_exact_ref_count(),
        4
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_source_range_cache()
            .full_source_range_ref_count(),
        4
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_source_range_cache()
            .partial_source_range_ref_count(),
        0
    );

    let output_ring_bucket_cache = ring_cache.output_ring_bucket_cache();
    assert_eq!(output_ring_bucket_cache.ring_count(), 1);
    assert_eq!(output_ring_bucket_cache.segment_ref_count(), 4);
    assert_eq!(output_ring_bucket_cache.max_ring_segment_count(), 4);
    assert_eq!(output_ring_bucket_cache.rings().len(), 1);
    assert_eq!(output_ring_bucket_cache.rings()[0].output_ring_index(), 0);
    assert_eq!(output_ring_bucket_cache.rings()[0].segments().len(), 4);
    assert_eq!(
        output_ring_bucket_cache.rings()[0].segments()[0].source_report_index(),
        0
    );
    assert_eq!(
        output_ring_bucket_cache.rings()[0].segments()[0].output_segment_index(),
        0
    );
    assert!(!output_ring_bucket_cache.rings()[0].segments()[0].reversed());

    let output_kind_bucket_cache = ring_cache.output_segment_kind_bucket_cache();
    assert_eq!(output_kind_bucket_cache.line_segment_ref_count(), 4);
    assert_eq!(output_kind_bucket_cache.arc_segment_ref_count(), 0);
    assert_eq!(output_kind_bucket_cache.max_bucket_size(), 4);

    let output_source_range_cache = ring_cache.output_segment_source_range_cache();
    assert_eq!(output_source_range_cache.output_segment_ref_count(), 4);
    assert_eq!(output_source_range_cache.full_source_range_ref_count(), 2);
    assert_eq!(
        output_source_range_cache.partial_source_range_ref_count(),
        2
    );
    assert_eq!(
        output_source_range_cache.ranges()[0].source_segment_index(),
        0
    );
    assert_eq!(
        output_source_range_cache.ranges()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), s(1))
    );

    let output_endpoint_cache = ring_cache.output_segment_endpoint_cache();
    assert_eq!(output_endpoint_cache.output_segment_ref_count(), 4);
    assert_eq!(output_endpoint_cache.output_endpoint_ref_count(), 8);
    assert_eq!(
        output_endpoint_cache.segments()[0].output_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        output_endpoint_cache.segments()[0].output_end_point(),
        &p(4, 0)
    );

    let output_continuity_cache = ring_cache.output_ring_continuity_cache();
    assert_eq!(output_continuity_cache.output_ring_ref_count(), 1);
    assert_eq!(output_continuity_cache.output_connection_ref_count(), 4);
    assert_eq!(output_continuity_cache.max_ring_connection_count(), 4);
    assert_eq!(
        output_continuity_cache.connections()[0].output_end_point(),
        output_continuity_cache.connections()[0].next_output_start_point()
    );

    let output_direction_bucket_cache = ring_cache.output_segment_direction_bucket_cache();
    assert_eq!(output_direction_bucket_cache.output_segment_ref_count(), 4);
    assert_eq!(output_direction_bucket_cache.forward_segment_ref_count(), 2);
    assert_eq!(
        output_direction_bucket_cache.reversed_segment_ref_count(),
        2
    );
    assert_eq!(output_direction_bucket_cache.max_bucket_size(), 2);
    assert_eq!(source_reports[3].source_segment_index(), 2);
    assert!(source_reports[3].reversed());
    assert!(built.exact_endpoint_connection_count().unwrap() >= 4);
    assert_eq!(built.unresolved_endpoint_connection_count(), Some(0));
    assert_eq!(built.blocker(), None);

    assert_eq!(
        built.boundary_build_validation_intersection_event_count(),
        Some(0)
    );
    assert_eq!(built.output_contour_count(), Some(1));
    assert_eq!(built.output_segment_count(), Some(4));
    assert_eq!(built.material_contour_count(), Some(1));
    assert_eq!(built.hole_contour_count(), Some(0));
    assert_eq!(built.material_segment_count(), Some(4));
    assert_eq!(built.hole_segment_count(), Some(0));
    let role_reports = built.role_reports().unwrap();
    assert_eq!(built.role_report_count(), Some(role_reports.len()));
    assert_eq!(role_reports.len(), 1);
    assert_eq!(role_reports[0].role(), RegionBoundaryContourRole2::Material);
    assert_eq!(role_reports[0].nesting_depth(), 0);

    let region = built.region().unwrap();
    assert_eq!(
        region.classify_point(&p(2, 2), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unordered_line_segments_report_disconnected_boundary_blocker() {
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
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(built.arranged_segment_count(), Some(2));
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(1));
    assert_eq!(built.split_tested_pair_count(), Some(0));
    assert_eq!(built.split_intersection_event_count(), Some(0));
    assert_eq!(built.split_point_relation_count(), Some(0));
    assert_eq!(built.split_overlap_relation_count(), Some(0));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    assert!(built.split_intersection_points().unwrap().is_empty());
    assert_eq!(built.split_output_segment_count(), Some(2));
    assert_eq!(
        built.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        built.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(4));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(1));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(
        built.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        built.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(built.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    assert_eq!(built.arranged_source_report_count(), Some(2));
    assert_eq!(built.output_ring_count(), None);
    assert_eq!(built.output_boundary_segment_count(), None);
    assert_eq!(built.output_contour_count(), None);
    assert_eq!(built.output_segment_count(), None);
    assert_eq!(built.material_contour_count(), None);
    assert_eq!(built.hole_contour_count(), None);
    assert_eq!(built.material_segment_count(), None);
    assert_eq!(built.hole_segment_count(), None);
    assert_eq!(built.role_report_count(), None);
    assert_eq!(built.role_reports(), None);
    assert_eq!(built.source_report_count(), Some(0));
    assert_eq!(built.boundary_build_stage(), None);

    let endpoint_graph_cache = built.endpoint_graph_cache().unwrap();
    assert_eq!(endpoint_graph_cache.endpoint_count(), 4);
    assert_eq!(endpoint_graph_cache.structural_bucket_count(), 4);
    assert_eq!(endpoint_graph_cache.structural_singleton_bucket_count(), 4);
    assert_eq!(endpoint_graph_cache.max_structural_bucket_size(), 1);
    assert_eq!(endpoint_graph_cache.dangling_endpoint_count(), 4);
    assert_eq!(endpoint_graph_cache.branch_endpoint_count(), 0);
    assert_eq!(
        endpoint_graph_cache.blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        endpoint_graph_cache.blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(endpoint_graph_cache.blocker_point(), Some(&p(0, 0)));
    assert_eq!(
        endpoint_graph_cache.endpoint_bucket_cache().bucket_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .start_endpoint_ref_count(),
        2
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .end_endpoint_ref_count(),
        2
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .dangling_structural_bucket_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .chain_structural_bucket_count(),
        0
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .branch_structural_bucket_count(),
        0
    );

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(
        ring_cache.predicate_path(),
        RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal
    );
    assert_eq!(ring_cache.output_ring_count(), None);
    assert_eq!(ring_cache.output_boundary_segment_count(), None);
    assert!(ring_cache.source_reports().is_empty());
    assert_eq!(ring_cache.output_ring_bucket_cache().ring_count(), 0);
    assert_eq!(
        ring_cache
            .output_segment_kind_bucket_cache()
            .output_segment_ref_count(),
        0
    );
    assert_eq!(
        ring_cache
            .output_segment_source_bucket_cache()
            .source_segment_bucket_count(),
        0
    );
    assert_eq!(
        ring_cache
            .output_ring_continuity_cache()
            .output_connection_ref_count(),
        0
    );
    assert_eq!(ring_cache.arranged_source_reports().len(), 2);
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_count(),
        2
    );
    assert_eq!(ring_cache.arranged_fragment_cache().source_ref_count(), 2);
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_status_bucket_cache()
            .native_exact_ref_count(),
        2
    );
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
    assert!(built.status().unwrap().is_retained_evidence());
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(
        built.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(
        built.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(1));
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_overlap_relation_count(), Some(0));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    let split_points = built.split_intersection_points().unwrap();
    let split_reports = built.split_intersection_reports().unwrap();
    assert_eq!(split_points, &[p(2, 2)]);
    assert_eq!(split_reports.len(), 1);
    let event: &hypercurve::RegionLineSegmentSplitIntersectionReport2 = &split_reports[0];
    assert_eq!(event.first_source_segment_index(), 0);
    assert_eq!(event.first_source_segment_kind(), SegmentKind::Line);
    assert_eq!(event.first_source_segment_start_point(), &p(0, 0));
    assert_eq!(event.first_source_segment_end_point(), &p(4, 4));
    assert_eq!(event.first_source_param(), &q(1, 2));
    assert_eq!(event.second_source_segment_index(), 1);
    assert_eq!(event.second_source_segment_kind(), SegmentKind::Line);
    assert_eq!(event.second_source_segment_start_point(), &p(0, 4));
    assert_eq!(event.second_source_segment_end_point(), &p(4, 0));
    assert_eq!(event.second_source_param(), &q(1, 2));
    assert_eq!(event.point(), &p(2, 2));
    assert_eq!(built.split_output_segment_count(), Some(4));
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        built.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        built.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(built.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    let arranged_sources = built.arranged_source_reports().unwrap();
    assert_eq!(built.arranged_source_report_count(), Some(4));
    assert_eq!(arranged_sources.len(), 4);
    assert_eq!(arranged_sources[0].source_segment_index(), 0);
    assert_eq!(
        arranged_sources[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 2))
    );
    assert_eq!(built.source_report_count(), Some(0));

    let split_cache = built.split_cache().unwrap();
    assert_eq!(split_cache.intersection_points(), &[p(2, 2)]);
    assert_eq!(split_cache.intersection_reports(), split_reports);
    assert_eq!(split_cache.output_segment_count(), Some(4));
    assert!(split_cache.blocker_cache().is_none());
    assert_eq!(
        split_cache.relation_bucket_cache().point_relation_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_bucket_cache()
            .intersection_event_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_parameter_cache()
            .source_parameter_ref_count(),
        2
    );

    let endpoint_graph_cache = built.endpoint_graph_cache().unwrap();
    assert_eq!(endpoint_graph_cache.endpoint_count(), 8);
    assert_eq!(endpoint_graph_cache.structural_bucket_count(), 5);
    assert_eq!(endpoint_graph_cache.structural_singleton_bucket_count(), 4);
    assert_eq!(endpoint_graph_cache.max_structural_bucket_size(), 4);
    assert_eq!(endpoint_graph_cache.dangling_endpoint_count(), 4);
    assert_eq!(endpoint_graph_cache.branch_endpoint_count(), 4);
    assert_eq!(
        endpoint_graph_cache.blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        endpoint_graph_cache.blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(endpoint_graph_cache.blocker_point(), Some(&p(0, 0)));
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .dangling_structural_bucket_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .chain_structural_bucket_count(),
        0
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .branch_structural_bucket_count(),
        1
    );

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(ring_cache.arranged_source_reports(), arranged_sources);
    assert!(ring_cache.source_reports().is_empty());
    assert_eq!(ring_cache.output_ring_count(), None);
    assert_eq!(ring_cache.output_ring_bucket_cache().ring_count(), 0);
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_count(),
        4
    );
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_kind_bucket_cache()
            .line_fragment_ref_count(),
        4
    );
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_source_range_cache()
            .partial_source_range_ref_count(),
        4
    );
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_line_segments_report_overlap_source_pair_blocker() {
    let built = evaluate_unordered_line_segments(
        vec![line(0, 0, 4, 0), line(2, 0, 6, 0)],
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
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(built.arranged_segment_count(), None);
    assert_eq!(built.arranged_segment_kind_counts(), None);
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(
        built.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(0));
    assert_eq!(built.split_point_relation_count(), Some(0));
    assert_eq!(built.split_overlap_relation_count(), Some(1));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    assert!(built.split_intersection_points().unwrap().is_empty());
    assert_eq!(built.split_output_segment_count(), None);
    assert_eq!(built.endpoint_graph_predicate_path(), None);
    assert_eq!(built.ring_assembly_predicate_path(), None);
    assert_eq!(built.split_blocker_first_source_segment_index(), Some(0));
    assert_eq!(
        built.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        built.split_blocker_first_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(built.split_blocker_first_source_end_point(), Some(&p(4, 0)));
    assert_eq!(built.split_blocker_second_source_segment_index(), Some(1));
    assert_eq!(
        built.split_blocker_second_source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        built.split_blocker_second_source_start_point(),
        Some(&p(2, 0))
    );
    assert_eq!(
        built.split_blocker_second_source_end_point(),
        Some(&p(6, 0))
    );
    assert_eq!(built.arranged_source_report_count(), None);
    assert_eq!(built.output_boundary_segment_kind_counts(), None);
    assert_eq!(built.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(built.endpoint_graph_blocker_endpoint(), None);

    let split_cache = built.split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(split_cache.candidate_pair_count(), 1);
    assert_eq!(split_cache.skipped_aabb_pair_count(), 0);
    assert_eq!(split_cache.tested_pair_count(), 1);
    assert_eq!(split_cache.intersection_event_count(), 0);
    assert!(split_cache.intersection_points().is_empty());
    assert!(split_cache.intersection_reports().is_empty());
    assert_eq!(split_cache.output_segment_count(), None);
    assert_eq!(split_cache.relation_bucket_cache().relation_count(), 1);
    assert_eq!(
        split_cache.relation_bucket_cache().overlap_relation_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_bucket_cache()
            .intersection_event_count(),
        0
    );
    assert_eq!(
        split_cache
            .intersection_parameter_cache()
            .source_parameter_ref_count(),
        0
    );
    let split_blocker_cache = split_cache.blocker_cache().unwrap();
    assert_eq!(split_blocker_cache.first_source_segment_index(), 0);
    assert_eq!(
        split_blocker_cache.first_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(split_blocker_cache.first_source_start_point(), &p(0, 0));
    assert_eq!(split_blocker_cache.first_source_end_point(), &p(4, 0));
    assert_eq!(split_blocker_cache.second_source_segment_index(), 1);
    assert_eq!(
        split_blocker_cache.second_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(split_blocker_cache.second_source_start_point(), &p(2, 0));
    assert_eq!(split_blocker_cache.second_source_end_point(), &p(6, 0));
    assert_eq!(
        split_blocker_cache.blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert!(built.endpoint_graph_cache().is_none());
    assert!(built.ring_assembly_cache().is_none());
    assert!(built.output_cache().is_some());
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
    assert_eq!(built.arranged_source_report_count(), Some(4));
    assert_eq!(built.source_report_count(), Some(4));
}

#[test]
fn exact_curve_arrangement_result_returns_region_classification() {
    let result = evaluate_unordered_line_segments(
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

    match result.region_classification() {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected decided region, got {reason:?}"),
    }

    match result.into_region_classification() {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected owned region, got {reason:?}"),
    }

    let retained_result = evaluate_unordered_line_segments(
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
    let (evaluation, owned_region) = retained_result.into_evaluation_and_region();
    assert!(
        evaluation
            .summary_cache()
            .status()
            .unwrap()
            .is_native_exact()
    );
    assert!(owned_region.is_some());

    let retained_result = evaluate_unordered_line_segments(
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
    let (evaluation, classification) = retained_result.into_evaluation_and_region_classification();
    assert!(
        evaluation
            .summary_cache()
            .status()
            .unwrap()
            .is_native_exact()
    );
    match classification {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected owned region, got {reason:?}"),
    }

    let retained_result = evaluate_unordered_line_segments(
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
    let arrangement_report = retained_result.arrangement_report();
    assert!(retained_result.region().is_some());
    assert_eq!(arrangement_report.materialized_region(), Some(true));
    assert!(arrangement_report.status().unwrap().is_native_exact());
    assert_eq!(
        arrangement_report.summary_cache(),
        retained_result.summary_cache()
    );

    let retained_result = evaluate_unordered_line_segments(
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
    let arrangement_report = retained_result.arrangement_report();
    assert_eq!(arrangement_report.materialized_region(), Some(true));
    assert!(arrangement_report.status().unwrap().is_native_exact());
    assert_eq!(
        arrangement_report.summary_cache(),
        retained_result.summary_cache()
    );
    match retained_result.region_classification() {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected owned region, got {reason:?}"),
    }

    let retained_result = evaluate_unordered_line_segments(
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
    let arrangement_report = retained_result.arrangement_report();
    assert!(retained_result.region().is_some());
    assert!(arrangement_report.status().unwrap().is_native_exact());
    assert_eq!(
        arrangement_report.summary_cache(),
        retained_result.summary_cache()
    );
}

#[test]
fn unordered_segments_convenience_returns_arrangement_report() {
    let lines = vec![
        line(0, 0, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
        line(4, 0, 4, 4),
    ];
    let (classification, report) =
        Region2::from_unordered_line_segments_borrowed_with_arrangement_report(
            &lines,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert!(report.status().unwrap().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert!(report.source_line_segments().is_some());
    match classification {
        Classification::Decided(region) => assert_eq!(
            region.classify_point(&p(2, 2), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        ),
        Classification::Uncertain(reason) => panic!("expected decided line region, got {reason:?}"),
    }

    let (classification, report) = Region2::from_unordered_line_segments_with_arrangement_report(
        lines.clone(),
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(report.status().unwrap().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert!(matches!(classification, Classification::Decided(_)));

    let disconnected = vec![line(0, 0, 1, 0), line(3, 0, 4, 0)];
    let (classification, report) =
        Region2::from_unordered_line_segments_borrowed_with_arrangement_report(
            &disconnected,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert_eq!(
        classification,
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(report.materialized_region(), Some(false));

    let native_segments = lines.into_iter().map(Segment2::Line).collect::<Vec<_>>();
    let (classification, report) =
        Region2::from_unordered_segments_borrowed_with_arrangement_report(
            &native_segments,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert!(report.status().unwrap().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert!(report.source_line_segments().is_none());
    assert!(matches!(classification, Classification::Decided(_)));

    let (classification, report) = Region2::from_unordered_segments_with_arrangement_report(
        native_segments,
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    assert!(report.status().unwrap().is_native_exact());
    assert!(matches!(classification, Classification::Decided(_)));
}

#[test]
fn exact_curve_arrangement_result_classification_preserves_blocker() {
    let result = evaluate_unordered_line_segments(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert_eq!(
        result.region_classification(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        result.into_region_classification(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let result = evaluate_unordered_line_segments(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let (evaluation, classification) = result.into_evaluation_and_region_classification();
    assert_eq!(
        evaluation.summary_cache().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(
        classification,
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let result = evaluate_unordered_line_segments(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let arrangement_report = result.arrangement_report();
    assert_eq!(
        arrangement_report.blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
    assert_eq!(
        result.region_classification(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let result = evaluate_unordered_line_segments(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let arrangement_report = result.arrangement_report();
    assert!(result.region().is_none());
    assert_eq!(
        arrangement_report.blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
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
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RegionRoleAssignment)
    );
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(built.arranged_segment_count(), Some(2));
    assert_eq!(
        built.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(
        built.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(2));
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_overlap_relation_count(), Some(0));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    let split_points = built.split_intersection_points().unwrap();
    let split_reports = built.split_intersection_reports().unwrap();
    assert_eq!(split_points.len(), 2);
    assert_eq!(split_reports.len(), 2);
    assert!(split_points.contains(&p(0, 0)));
    assert!(split_points.contains(&p(4, 0)));
    assert_eq!(built.split_output_segment_count(), Some(2));
    assert_eq!(
        built.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        built.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(built.split_blocker_first_source_segment_index(), None);
    assert_eq!(built.split_blocker_first_source_segment_kind(), None);
    assert_eq!(built.split_blocker_first_source_start_point(), None);
    assert_eq!(built.split_blocker_first_source_end_point(), None);
    assert_eq!(built.split_blocker_second_source_segment_index(), None);
    assert_eq!(built.split_blocker_second_source_segment_kind(), None);
    assert_eq!(built.split_blocker_second_source_start_point(), None);
    assert_eq!(built.split_blocker_second_source_end_point(), None);
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(2));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(2));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(built.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(built.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(built.endpoint_graph_blocker_point(), None);
    assert_eq!(built.reversed_source_segment_count(), Some(0));
    assert_eq!(built.output_ring_count(), Some(1));
    assert_eq!(built.output_boundary_segment_count(), Some(2));
    assert_eq!(
        built.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    let arranged_sources = built.arranged_source_reports().unwrap();
    assert_eq!(built.arranged_source_report_count(), Some(2));
    assert_eq!(arranged_sources.len(), 2);
    assert_eq!(arranged_sources[0].source_segment_index(), 0);
    assert_eq!(arranged_sources[1].source_segment_index(), 1);
    assert_eq!(arranged_sources[0].source_segment_kind(), SegmentKind::Line);
    assert_eq!(arranged_sources[0].source_segment_start_point(), &p(4, 0));
    assert_eq!(arranged_sources[0].source_segment_end_point(), &p(0, 0));
    assert_eq!(
        arranged_sources[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(arranged_sources[1].source_segment_kind(), SegmentKind::Arc);
    assert_eq!(arranged_sources[1].source_segment_start_point(), &p(0, 0));
    assert_eq!(arranged_sources[1].source_segment_end_point(), &p(4, 0));
    assert_eq!(
        arranged_sources[1].arranged_segment_kind(),
        SegmentKind::Arc
    );
    let source_reports = built.source_reports().unwrap();
    assert_eq!(built.source_report_count(), Some(2));
    assert_eq!(source_reports.len(), 2);
    assert_eq!(source_reports[0].source_segment_index(), 0);
    assert_eq!(source_reports[1].source_segment_index(), 1);
    assert_eq!(source_reports[0].source_segment_start_point(), &p(4, 0));
    assert_eq!(source_reports[0].source_segment_end_point(), &p(0, 0));
    assert_eq!(source_reports[0].source_segment_kind(), SegmentKind::Line);
    assert_eq!(source_reports[0].output_segment_kind(), SegmentKind::Line);
    assert_eq!(source_reports[1].source_segment_kind(), SegmentKind::Arc);
    assert_eq!(source_reports[1].source_segment_start_point(), &p(0, 0));
    assert_eq!(source_reports[1].source_segment_end_point(), &p(4, 0));
    assert_eq!(source_reports[1].output_segment_kind(), SegmentKind::Arc);

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(ring_cache.arranged_source_reports(), arranged_sources);
    assert_eq!(ring_cache.source_reports(), source_reports);

    let fragment_cache = ring_cache.arranged_fragment_cache();
    assert_eq!(fragment_cache.arranged_fragment_count(), 2);
    assert_eq!(fragment_cache.source_ref_count(), 2);
    assert_eq!(
        fragment_cache.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(
        fragment_cache.arranged_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_kind_bucket_cache()
            .line_fragment_ref_count(),
        1
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_kind_bucket_cache()
            .arc_fragment_ref_count(),
        1
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_status_bucket_cache()
            .native_exact_ref_count(),
        2
    );
    assert_eq!(
        fragment_cache
            .arranged_fragment_source_range_cache()
            .source_ref_count(),
        2
    );

    let output_ring_bucket_cache = ring_cache.output_ring_bucket_cache();
    assert_eq!(output_ring_bucket_cache.ring_count(), 1);
    assert_eq!(output_ring_bucket_cache.segment_ref_count(), 2);
    assert_eq!(output_ring_bucket_cache.max_ring_segment_count(), 2);

    let output_kind_bucket_cache = ring_cache.output_segment_kind_bucket_cache();
    assert_eq!(output_kind_bucket_cache.line_segment_ref_count(), 1);
    assert_eq!(output_kind_bucket_cache.arc_segment_ref_count(), 1);
    assert_eq!(output_kind_bucket_cache.max_bucket_size(), 1);

    let output_source_bucket_cache = ring_cache.output_segment_source_bucket_cache();
    assert_eq!(output_source_bucket_cache.source_segment_bucket_count(), 2);
    assert_eq!(output_source_bucket_cache.output_segment_ref_count(), 2);
    assert_eq!(output_source_bucket_cache.max_bucket_size(), 1);
    assert_eq!(
        output_source_bucket_cache.buckets()[0].source_segment_index(),
        0
    );
    assert_eq!(
        output_source_bucket_cache.buckets()[1].source_segment_index(),
        1
    );

    let output_status_bucket_cache = ring_cache.output_segment_status_bucket_cache();
    assert_eq!(output_status_bucket_cache.output_segment_ref_count(), 2);
    assert_eq!(output_status_bucket_cache.native_exact_ref_count(), 2);
    assert_eq!(output_status_bucket_cache.unresolved_ref_count(), 0);

    let output_direction_bucket_cache = ring_cache.output_segment_direction_bucket_cache();
    assert_eq!(output_direction_bucket_cache.output_segment_ref_count(), 2);
    assert_eq!(output_direction_bucket_cache.forward_segment_ref_count(), 2);
    assert_eq!(
        output_direction_bucket_cache.reversed_segment_ref_count(),
        0
    );

    let output_continuity_cache = ring_cache.output_ring_continuity_cache();
    assert_eq!(output_continuity_cache.output_ring_ref_count(), 1);
    assert_eq!(output_continuity_cache.output_connection_ref_count(), 2);
    assert_eq!(
        output_continuity_cache.connections()[0].output_end_point(),
        output_continuity_cache.connections()[0].next_output_start_point()
    );
    assert_eq!(built.blocker(), None);

    let region = built.region().unwrap();
    assert_eq!(
        region.classify_point(&p(2, -1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(2, 0), &policy()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
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
    assert_eq!(built.arranged_source_report_count(), Some(2));
    assert_eq!(built.source_report_count(), Some(2));
}

#[test]
fn exact_curve_arrangement_attempt_builds_line_region_with_line_specific_report() {
    let lines = vec![
        line(4, 0, 0, 0),
        line(4, 4, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
    ];
    let request = ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(
        &lines,
        FillRule::NonZero,
    );
    let attempt = ExactCurveArrangementAttempt2::new(request);
    let result = attempt.evaluate(&policy()).unwrap();

    assert_eq!(result.source_segment_count(), 4);
    assert_eq!(result.source_line_segments(), Some(lines.as_slice()));
    let (owned_source_segments, owned_source_line_segments, owned_fill_rule) =
        result.request().clone().into_parts();
    assert_eq!(owned_source_segments.as_slice(), result.source_segments());
    assert_eq!(
        owned_source_line_segments.as_deref(),
        Some(lines.as_slice())
    );
    assert_eq!(owned_fill_rule, result.fill_rule());
    assert_eq!(
        result.request().clone().into_source_line_segments(),
        Some(lines.clone())
    );
    assert_eq!(result.source_segments().len(), 4);
    assert!(
        result
            .source_segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
    assert_eq!(
        result.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(result.source_segment_aabbs().len(), 4);
    assert_eq!(result.decided_source_segment_aabb_count(), 4);
    assert_eq!(result.undecided_source_segment_aabb_count(), 0);
    assert_eq!(
        result.source_segment_aabbs()[0]
            .as_ref()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        Some((p(0, 0), p(4, 0)))
    );
    assert_eq!(
        result
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        Some((p(0, 0), p(4, 4)))
    );
    let source_segment_cache = result.source_segment_cache();
    assert_eq!(
        source_segment_cache.source_segment_count(),
        result.source_segment_count()
    );
    assert_eq!(
        source_segment_cache.source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(
        source_segment_cache.decided_source_segment_aabb_count(),
        result.decided_source_segment_aabb_count()
    );
    assert_eq!(
        source_segment_cache.undecided_source_segment_aabb_count(),
        result.undecided_source_segment_aabb_count()
    );
    assert_eq!(
        source_segment_cache
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        result
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone()))
    );
    let source_aabb_bucket_cache = source_segment_cache.source_aabb_bucket_cache();
    assert_eq!(source_aabb_bucket_cache.bucket_count(), 2);
    assert_eq!(source_aabb_bucket_cache.source_ref_count(), 4);
    assert_eq!(source_aabb_bucket_cache.decided_source_ref_count(), 4);
    assert_eq!(source_aabb_bucket_cache.undecided_source_ref_count(), 0);
    assert_eq!(source_aabb_bucket_cache.max_bucket_size(), 4);
    assert_eq!(source_aabb_bucket_cache.buckets().len(), 2);
    assert_eq!(
        source_aabb_bucket_cache.buckets()[0].aabb_status(),
        ExactCurveArrangementSourceAabbStatus2::Decided
    );
    assert_eq!(
        source_aabb_bucket_cache.buckets()[0].source_refs().len(),
        source_segment_cache.decided_source_segment_aabb_count()
    );
    assert_eq!(
        source_aabb_bucket_cache.buckets()[0].source_refs()[0].source_segment_index(),
        0
    );
    assert!(
        source_segment_cache.segments()
            [source_aabb_bucket_cache.buckets()[0].source_refs()[0].source_segment_index()]
        .source_aabb()
        .is_some()
    );
    assert_eq!(
        source_aabb_bucket_cache.buckets()[1].aabb_status(),
        ExactCurveArrangementSourceAabbStatus2::Undecided
    );
    assert!(
        source_aabb_bucket_cache.buckets()[1]
            .source_refs()
            .is_empty()
    );
    let source_segment_kind_bucket_cache = source_segment_cache.source_segment_kind_bucket_cache();
    assert_eq!(source_segment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        source_segment_kind_bucket_cache.source_segment_ref_count(),
        source_segment_cache.source_segment_count()
    );
    assert_eq!(
        source_segment_kind_bucket_cache.line_segment_ref_count(),
        result.source_segment_kind_counts().lines
    );
    assert_eq!(
        source_segment_kind_bucket_cache.arc_segment_ref_count(),
        result.source_segment_kind_counts().arcs
    );
    assert_eq!(source_segment_kind_bucket_cache.max_bucket_size(), 4);
    assert_eq!(source_segment_kind_bucket_cache.buckets().len(), 2);
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[0]
            .source_refs()
            .len(),
        result.source_segment_kind_counts().lines
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[0].source_refs()[0].source_segment_index(),
        0
    );
    assert_eq!(
        source_segment_cache.segments()
            [source_segment_kind_bucket_cache.buckets()[0].source_refs()[0].source_segment_index()]
        .source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[1].source_segment_kind(),
        SegmentKind::Arc
    );
    assert!(
        source_segment_kind_bucket_cache.buckets()[1]
            .source_refs()
            .is_empty()
    );
    assert_eq!(source_segment_cache.segments().len(), 4);
    let first_source_segment = &source_segment_cache.segments()[0];
    assert_eq!(first_source_segment.source_segment_index(), 0);
    assert_eq!(
        first_source_segment.source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(first_source_segment.source_start_point(), &p(4, 0));
    assert_eq!(first_source_segment.source_end_point(), &p(0, 0));
    assert_eq!(
        first_source_segment
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        Some((p(0, 0), p(4, 0)))
    );
    let source_endpoint_cache = result.source_endpoint_bucket_cache();
    assert_eq!(source_endpoint_cache.endpoint_count(), 8);
    assert_eq!(source_endpoint_cache.bucket_count(), 4);
    assert_eq!(source_endpoint_cache.singleton_bucket_count(), 0);
    assert_eq!(source_endpoint_cache.max_bucket_size(), 2);
    assert_eq!(source_endpoint_cache.buckets().len(), 4);
    let first_source_endpoint_bucket = &source_endpoint_cache.buckets()[0];
    assert_eq!(first_source_endpoint_bucket.point(), &p(4, 0));
    assert_eq!(first_source_endpoint_bucket.endpoints().len(), 2);
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[0].source_segment_index(),
        0
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[0].endpoint(),
        ExactCurveArrangementSourceEndpoint2::Start
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[1].source_segment_index(),
        1
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[1].endpoint(),
        ExactCurveArrangementSourceEndpoint2::End
    );
    let split_schedule_cache = result.split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.split_candidate_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.split_skipped_aabb_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.split_tested_pair_count().unwrap()
    );
    assert_eq!(split_schedule_cache.undecided_aabb_pair_count(), 0);
    assert_eq!(split_schedule_cache.candidate_pairs().len(), 6);
    let split_schedule_bucket_cache = split_schedule_cache.bucket_cache();
    assert_eq!(split_schedule_bucket_cache.bucket_count(), 3);
    assert_eq!(split_schedule_bucket_cache.candidate_ref_count(), 6);
    assert_eq!(split_schedule_bucket_cache.max_bucket_size(), 4);
    assert_eq!(split_schedule_bucket_cache.buckets().len(), 3);
    assert_eq!(
        split_schedule_bucket_cache.buckets()[0].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[0]
            .candidate_refs()
            .len(),
        split_schedule_cache.decided_disjoint_pair_count()
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[0].candidate_refs()[0].candidate_pair_index(),
        1
    );
    assert_eq!(
        split_schedule_cache.candidate_pairs()
            [split_schedule_bucket_cache.buckets()[0].candidate_refs()[0].candidate_pair_index()]
        .aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[1].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[1]
            .candidate_refs()
            .len(),
        split_schedule_cache.predicate_candidate_pair_count()
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[2].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::Undecided
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[2]
            .candidate_refs()
            .len(),
        split_schedule_cache.undecided_aabb_pair_count()
    );
    assert_eq!(
        (
            split_schedule_cache.candidate_pairs()[0].first_source_segment_index(),
            split_schedule_cache.candidate_pairs()[0].second_source_segment_index(),
            split_schedule_cache.candidate_pairs()[0].aabb_status(),
        ),
        (
            0,
            1,
            ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint,
        )
    );
    assert_eq!(
        (
            split_schedule_cache.candidate_pairs()[1].first_source_segment_index(),
            split_schedule_cache.candidate_pairs()[1].second_source_segment_index(),
            split_schedule_cache.candidate_pairs()[1].aabb_status(),
        ),
        (
            0,
            2,
            ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint,
        )
    );
    assert_eq!(
        result.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(result.split_candidate_pair_count(), Some(6));
    assert_eq!(result.split_skipped_aabb_pair_count(), Some(2));
    assert_eq!(result.split_tested_pair_count(), Some(4));
    assert_eq!(result.split_intersection_event_count(), Some(4));
    assert_eq!(result.split_intersection_points().unwrap().len(), 4);
    assert_eq!(result.split_intersection_reports().unwrap().len(), 4);
    assert_eq!(result.split_output_segment_count(), Some(4));
    assert_eq!(result.split_blocker_cache(), None);
    let split_relation_bucket_cache = result.split_relation_bucket_cache().unwrap();
    assert_eq!(split_relation_bucket_cache.bucket_count(), 3);
    assert_eq!(
        split_relation_bucket_cache.relation_count(),
        result.split_point_relation_count().unwrap()
            + result.split_overlap_relation_count().unwrap()
            + result.split_uncertain_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.point_relation_count(),
        result.split_point_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.overlap_relation_count(),
        result.split_overlap_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.uncertain_relation_count(),
        result.split_uncertain_relation_count().unwrap()
    );
    assert_eq!(split_relation_bucket_cache.buckets().len(), 3);
    assert_eq!(
        split_relation_bucket_cache.buckets()[0].relation(),
        ExactCurveArrangementSplitRelationClass2::Point
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[0].relation_count(),
        result.split_point_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[1].relation(),
        ExactCurveArrangementSplitRelationClass2::Overlap
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[1].relation_count(),
        result.split_overlap_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[2].relation(),
        ExactCurveArrangementSplitRelationClass2::Uncertain
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[2].relation_count(),
        result.split_uncertain_relation_count().unwrap()
    );
    let split_intersection_bucket_cache = result.split_intersection_bucket_cache().unwrap();
    assert_eq!(
        split_intersection_bucket_cache.intersection_event_count(),
        4
    );
    assert_eq!(split_intersection_bucket_cache.bucket_count(), 4);
    assert_eq!(split_intersection_bucket_cache.singleton_bucket_count(), 4);
    assert_eq!(split_intersection_bucket_cache.max_bucket_size(), 1);
    assert_eq!(split_intersection_bucket_cache.buckets().len(), 4);
    let first_split_intersection_bucket = &split_intersection_bucket_cache.buckets()[0];
    assert_eq!(first_split_intersection_bucket.point(), &p(4, 0));
    assert_eq!(first_split_intersection_bucket.intersections().len(), 1);
    assert_eq!(
        first_split_intersection_bucket.intersections()[0].intersection_report_index(),
        0
    );
    assert_eq!(
        first_split_intersection_bucket.point(),
        result.split_intersection_reports().unwrap()[0].point()
    );
    let split_intersection_parameter_cache = result.split_intersection_parameter_cache().unwrap();
    assert_eq!(
        split_intersection_parameter_cache.intersection_event_count(),
        result.split_intersection_reports().unwrap().len()
    );
    assert_eq!(
        split_intersection_parameter_cache.source_parameter_ref_count(),
        result.split_intersection_reports().unwrap().len() * 2
    );
    assert_eq!(
        split_intersection_parameter_cache.parameters().len(),
        result.split_intersection_reports().unwrap().len()
    );
    let first_split_parameter = &split_intersection_parameter_cache.parameters()[0];
    assert_eq!(first_split_parameter.intersection_report_index(), 0);
    assert_eq!(
        first_split_parameter.first_source_segment_index(),
        result.split_intersection_reports().unwrap()[0].first_source_segment_index()
    );
    assert_eq!(
        first_split_parameter.first_source_param(),
        result.split_intersection_reports().unwrap()[0].first_source_param()
    );
    assert_eq!(
        first_split_parameter.second_source_segment_index(),
        result.split_intersection_reports().unwrap()[0].second_source_segment_index()
    );
    assert_eq!(
        first_split_parameter.second_source_param(),
        result.split_intersection_reports().unwrap()[0].second_source_param()
    );
    assert_eq!(
        first_split_parameter.point(),
        result.split_intersection_reports().unwrap()[0].point()
    );
    assert_eq!(
        result.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(result.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(result.endpoint_graph_structural_bucket_count(), Some(4));
    assert_eq!(
        result.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(result.endpoint_graph_max_structural_bucket_size(), Some(2));
    let arranged_endpoint_bucket_cache = result.arranged_endpoint_bucket_cache().unwrap();
    assert_eq!(
        arranged_endpoint_bucket_cache.endpoint_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.bucket_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.singleton_bucket_count(),
        result
            .endpoint_graph_structural_singleton_bucket_count()
            .unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.max_bucket_size(),
        result.endpoint_graph_max_structural_bucket_size().unwrap()
    );
    assert_eq!(arranged_endpoint_bucket_cache.buckets().len(), 4);
    let first_arranged_endpoint_bucket = &arranged_endpoint_bucket_cache.buckets()[0];
    assert_eq!(first_arranged_endpoint_bucket.point(), &p(4, 0));
    assert_eq!(first_arranged_endpoint_bucket.endpoints().len(), 2);
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[0].arranged_segment_index(),
        0
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[1].arranged_segment_index(),
        1
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[1].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    let arranged_endpoint_side_bucket_cache = result.arranged_endpoint_side_bucket_cache().unwrap();
    assert_eq!(
        result.arranged_endpoint_side_bucket_count(),
        Some(arranged_endpoint_side_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_endpoint_side_ref_count(),
        Some(arranged_endpoint_side_bucket_cache.endpoint_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_start_ref_count(),
        Some(arranged_endpoint_side_bucket_cache.start_endpoint_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_end_ref_count(),
        Some(arranged_endpoint_side_bucket_cache.end_endpoint_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_side_max_bucket_size(),
        Some(arranged_endpoint_side_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_endpoint_side_bucket_cache.bucket_count(), 2);
    assert_eq!(
        arranged_endpoint_side_bucket_cache.endpoint_ref_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.start_endpoint_ref_count(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.end_endpoint_ref_count(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(arranged_endpoint_side_bucket_cache.max_bucket_size(), 4);
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0]
            .endpoints()
            .len(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoints()[0].arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[1].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[1]
            .endpoints()
            .len(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[1].endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    let arranged_endpoint_point_cache = result.arranged_endpoint_point_cache().unwrap();
    assert_eq!(
        result.arranged_endpoint_point_fragment_ref_count(),
        Some(arranged_endpoint_point_cache.arranged_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_point_ref_count(),
        Some(arranged_endpoint_point_cache.endpoint_ref_count())
    );
    assert_eq!(
        arranged_endpoint_point_cache.arranged_fragment_ref_count(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(
        arranged_endpoint_point_cache.endpoint_ref_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_point_cache.endpoints().len(),
        result.arranged_source_reports().unwrap().len()
    );
    let arranged_endpoint_point_ref = &arranged_endpoint_point_cache.endpoints()[0];
    assert_eq!(
        arranged_endpoint_point_ref.arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert_eq!(
        arranged_endpoint_point_ref.output_start_point(),
        result.arranged_source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        arranged_endpoint_point_ref.output_end_point(),
        result.arranged_source_reports().unwrap()[0].output_end_point()
    );
    let arranged_endpoint_degree_bucket_cache =
        result.arranged_endpoint_degree_bucket_cache().unwrap();
    assert_eq!(
        result.arranged_endpoint_degree_bucket_count(),
        Some(arranged_endpoint_degree_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_endpoint_degree_structural_bucket_ref_count(),
        Some(arranged_endpoint_degree_bucket_cache.structural_bucket_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_dangling_structural_bucket_count(),
        Some(arranged_endpoint_degree_bucket_cache.dangling_structural_bucket_count())
    );
    assert_eq!(
        result.arranged_endpoint_chain_structural_bucket_count(),
        Some(arranged_endpoint_degree_bucket_cache.chain_structural_bucket_count())
    );
    assert_eq!(
        result.arranged_endpoint_branch_structural_bucket_count(),
        Some(arranged_endpoint_degree_bucket_cache.branch_structural_bucket_count())
    );
    assert_eq!(
        result.arranged_endpoint_degree_max_bucket_size(),
        Some(arranged_endpoint_degree_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_endpoint_degree_bucket_cache.bucket_count(), 3);
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.structural_bucket_ref_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.dangling_structural_bucket_count(),
        0
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.chain_structural_bucket_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.branch_structural_bucket_count(),
        0
    );
    assert_eq!(arranged_endpoint_degree_bucket_cache.max_bucket_size(), 4);
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[0].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Dangling
    );
    assert!(
        arranged_endpoint_degree_bucket_cache.buckets()[0]
            .endpoint_buckets()
            .is_empty()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[1].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Chain
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[1]
            .endpoint_buckets()
            .len(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    let first_degree_ref =
        &arranged_endpoint_degree_bucket_cache.buckets()[1].endpoint_buckets()[0];
    assert_eq!(first_degree_ref.structural_bucket_index(), 0);
    assert_eq!(
        first_degree_ref.endpoint_ref_count(),
        arranged_endpoint_bucket_cache.buckets()[0]
            .endpoints()
            .len()
    );
    assert_eq!(
        first_degree_ref.point(),
        arranged_endpoint_bucket_cache.buckets()[0].point()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[2].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Branch
    );
    assert!(
        arranged_endpoint_degree_bucket_cache.buckets()[2]
            .endpoint_buckets()
            .is_empty()
    );
    assert_eq!(result.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(result.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(result.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(result.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(result.endpoint_graph_blocker_point(), None);
    assert_eq!(
        result.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(
        result.attempted_endpoint_connection_count(),
        Some(
            result.exact_endpoint_connection_count().unwrap()
                + result.disconnected_endpoint_connection_count().unwrap()
                + result.unresolved_endpoint_connection_count().unwrap()
        )
    );
    assert!(result.exact_endpoint_connection_count().unwrap() >= 4);
    assert_eq!(result.unresolved_endpoint_connection_count(), Some(0));
    assert_eq!(result.reversed_source_segment_count(), Some(0));
    assert_eq!(result.output_ring_count(), Some(1));
    assert_eq!(result.output_boundary_segment_count(), Some(4));
    assert_eq!(
        result.arranged_source_report_count(),
        Some(result.arranged_source_reports().unwrap().len())
    );
    let arranged_fragment_cache = result.arranged_fragment_cache().unwrap();
    assert_eq!(arranged_fragment_cache.arranged_fragment_count(), 4);
    assert_eq!(arranged_fragment_cache.source_ref_count(), 4);
    assert_eq!(
        arranged_fragment_cache.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        arranged_fragment_cache.arranged_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    let arranged_fragment_kind_bucket_cache =
        arranged_fragment_cache.arranged_fragment_kind_bucket_cache();
    assert_eq!(
        result.arranged_fragment_kind_bucket_count(),
        Some(arranged_fragment_kind_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_fragment_kind_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.arranged_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_line_fragment_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.line_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_arc_fragment_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.arc_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_kind_max_bucket_size(),
        Some(arranged_fragment_kind_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_fragment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        arranged_fragment_kind_bucket_cache.arranged_fragment_ref_count(),
        4
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.line_fragment_ref_count(),
        4
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.arc_fragment_ref_count(),
        0
    );
    assert_eq!(arranged_fragment_kind_bucket_cache.max_bucket_size(), 4);
    assert_eq!(arranged_fragment_kind_bucket_cache.buckets().len(), 2);
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[0]
            .fragment_refs()
            .len(),
        arranged_fragment_cache.arranged_segment_kind_counts().lines
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[0].fragment_refs()[0]
            .arranged_fragment_index(),
        0
    );
    assert_eq!(
        arranged_fragment_cache.fragments()[arranged_fragment_kind_bucket_cache.buckets()[0]
            .fragment_refs()[0]
            .arranged_fragment_index()]
        .arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[1].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert!(
        arranged_fragment_kind_bucket_cache.buckets()[1]
            .fragment_refs()
            .is_empty()
    );
    let arranged_fragment_status_bucket_cache =
        arranged_fragment_cache.arranged_fragment_status_bucket_cache();
    assert_eq!(
        result.arranged_fragment_status_bucket_count(),
        Some(arranged_fragment_status_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_fragment_status_source_ref_count(),
        Some(arranged_fragment_status_bucket_cache.source_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_native_exact_ref_count(),
        Some(arranged_fragment_status_bucket_cache.native_exact_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_certified_approximation_ref_count(),
        Some(arranged_fragment_status_bucket_cache.certified_approximation_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_display_or_export_ref_count(),
        Some(arranged_fragment_status_bucket_cache.display_or_export_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_imported_lossy_ref_count(),
        Some(arranged_fragment_status_bucket_cache.imported_lossy_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_unsupported_ref_count(),
        Some(arranged_fragment_status_bucket_cache.unsupported_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_unresolved_ref_count(),
        Some(arranged_fragment_status_bucket_cache.unresolved_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_status_max_bucket_size(),
        Some(arranged_fragment_status_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_fragment_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        arranged_fragment_status_bucket_cache.source_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.native_exact_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.display_or_export_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.imported_lossy_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.unsupported_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.unresolved_ref_count(),
        0
    );
    assert_eq!(arranged_fragment_status_bucket_cache.max_bucket_size(), 4);
    assert_eq!(arranged_fragment_status_bucket_cache.buckets().len(), 6);
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0]
            .source_refs()
            .len(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .arranged_fragment_index(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0].source_ref_index(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .arranged_source_report_index(),
        0
    );
    assert_eq!(
        arranged_fragment_cache.fragments()[arranged_fragment_status_bucket_cache.buckets()[0]
            .source_refs()[0]
            .arranged_fragment_index()]
        .source_refs()[arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .source_ref_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        arranged_fragment_status_bucket_cache.buckets()[1]
            .source_refs()
            .is_empty()
    );
    let arranged_fragment_source_range_cache =
        arranged_fragment_cache.arranged_fragment_source_range_cache();
    assert_eq!(
        result.arranged_fragment_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.source_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_full_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.full_source_range_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_partial_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.partial_source_range_ref_count())
    );
    assert_eq!(
        arranged_fragment_source_range_cache.source_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_source_range_cache.full_source_range_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_source_range_cache.partial_source_range_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_source_range_cache.ranges().len(),
        arranged_fragment_cache.source_ref_count()
    );
    let arranged_source_range_ref = &arranged_fragment_source_range_cache.ranges()[0];
    assert_eq!(arranged_source_range_ref.arranged_source_report_index(), 0);
    assert_eq!(
        arranged_source_range_ref.source_segment_index(),
        result.arranged_source_reports().unwrap()[0].source_segment_index()
    );
    assert_eq!(
        arranged_source_range_ref.source_range(),
        result.arranged_source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        arranged_source_range_ref.arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert!(arranged_source_range_ref.covers_full_source_range());
    assert_eq!(arranged_fragment_cache.max_source_ref_count(), 1);
    assert_eq!(arranged_fragment_cache.fragments().len(), 4);
    let arranged_fragment = &arranged_fragment_cache.fragments()[0];
    assert_eq!(arranged_fragment.arranged_segment_index(), 0);
    assert_eq!(arranged_fragment.arranged_segment_kind(), SegmentKind::Line);
    assert_eq!(
        arranged_fragment.output_start_point(),
        result.arranged_source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        arranged_fragment.output_end_point(),
        result.arranged_source_reports().unwrap()[0].output_end_point()
    );
    assert_eq!(arranged_fragment.source_refs().len(), 1);
    assert_eq!(
        arranged_fragment.source_refs()[0].arranged_source_report_index(),
        0
    );
    assert_eq!(arranged_fragment.source_refs()[0].source_segment_index(), 0);
    assert_eq!(
        arranged_fragment.source_refs()[0].source_range(),
        result.arranged_source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        arranged_fragment.source_refs()[0].status(),
        result.arranged_source_reports().unwrap()[0].status()
    );
    assert_eq!(
        result.source_report_count(),
        Some(result.source_reports().unwrap().len())
    );
    let output_ring_bucket_cache = result.output_ring_bucket_cache().unwrap();
    assert_eq!(
        result.output_ring_segment_ref_count(),
        Some(output_ring_bucket_cache.segment_ref_count())
    );
    assert_eq!(
        result.output_ring_max_segment_count(),
        Some(output_ring_bucket_cache.max_ring_segment_count())
    );
    assert_eq!(output_ring_bucket_cache.ring_count(), 1);
    assert_eq!(output_ring_bucket_cache.segment_ref_count(), 4);
    assert_eq!(output_ring_bucket_cache.max_ring_segment_count(), 4);
    assert_eq!(output_ring_bucket_cache.rings().len(), 1);
    let output_ring_bucket = &output_ring_bucket_cache.rings()[0];
    assert_eq!(output_ring_bucket.output_ring_index(), 0);
    assert_eq!(output_ring_bucket.segments().len(), 4);
    assert_eq!(output_ring_bucket.segments()[0].source_report_index(), 0);
    assert_eq!(output_ring_bucket.segments()[0].output_segment_index(), 0);
    assert_eq!(
        output_ring_bucket.segments()[0].reversed(),
        result.source_reports().unwrap()[0].reversed()
    );
    let output_segment_kind_bucket_cache = result.output_segment_kind_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_kind_bucket_count(),
        Some(output_segment_kind_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_kind_ref_count(),
        Some(output_segment_kind_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_line_segment_ref_count(),
        Some(output_segment_kind_bucket_cache.line_segment_ref_count())
    );
    assert_eq!(
        result.output_arc_segment_ref_count(),
        Some(output_segment_kind_bucket_cache.arc_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_kind_max_bucket_size(),
        Some(output_segment_kind_bucket_cache.max_bucket_size())
    );
    assert_eq!(output_segment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        output_segment_kind_bucket_cache.output_segment_ref_count(),
        4
    );
    assert_eq!(output_segment_kind_bucket_cache.line_segment_ref_count(), 4);
    assert_eq!(output_segment_kind_bucket_cache.arc_segment_ref_count(), 0);
    assert_eq!(output_segment_kind_bucket_cache.max_bucket_size(), 4);
    assert_eq!(output_segment_kind_bucket_cache.buckets().len(), 2);
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        result.output_boundary_segment_kind_counts().unwrap().lines
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].source_report_index(),
        0
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        result.source_reports().unwrap()
            [output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].source_report_index()]
        .output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert!(
        output_segment_kind_bucket_cache.buckets()[1]
            .segment_refs()
            .is_empty()
    );
    let output_segment_source_bucket_cache = result.output_segment_source_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_source_bucket_count(),
        Some(output_segment_source_bucket_cache.source_segment_bucket_count())
    );
    assert_eq!(
        result.output_segment_source_ref_count(),
        Some(output_segment_source_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_source_max_bucket_size(),
        Some(output_segment_source_bucket_cache.max_bucket_size())
    );
    assert_eq!(
        output_segment_source_bucket_cache.source_segment_bucket_count(),
        4
    );
    assert_eq!(
        output_segment_source_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(output_segment_source_bucket_cache.max_bucket_size(), 1);
    assert_eq!(output_segment_source_bucket_cache.buckets().len(), 4);
    assert_eq!(
        output_segment_source_bucket_cache.buckets()[0].source_segment_index(),
        0
    );
    assert_eq!(
        output_segment_source_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        1
    );
    let source_ref = &output_segment_source_bucket_cache.buckets()[0].segment_refs()[0];
    assert_eq!(
        source_ref.output_ring_index(),
        result.source_reports().unwrap()[source_ref.source_report_index()].output_ring_index()
    );
    assert_eq!(
        source_ref.output_segment_index(),
        result.source_reports().unwrap()[source_ref.source_report_index()].output_segment_index()
    );
    assert_eq!(
        result.source_reports().unwrap()[source_ref.source_report_index()].source_segment_index(),
        output_segment_source_bucket_cache.buckets()[0].source_segment_index()
    );
    let output_segment_source_range_cache = result.output_segment_source_range_cache().unwrap();
    assert_eq!(
        output_segment_source_range_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_source_range_cache.full_source_range_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_source_range_cache.partial_source_range_ref_count(),
        0
    );
    assert_eq!(
        output_segment_source_range_cache.ranges().len(),
        result.source_report_count().unwrap()
    );
    let source_range_ref = &output_segment_source_range_cache.ranges()[0];
    assert_eq!(source_range_ref.source_report_index(), 0);
    assert_eq!(
        source_range_ref.source_segment_index(),
        result.source_reports().unwrap()[0].source_segment_index()
    );
    assert_eq!(
        source_range_ref.source_range(),
        result.source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        source_range_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        source_range_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert!(source_range_ref.covers_full_source_range());
    let output_segment_endpoint_cache = result.output_segment_endpoint_cache().unwrap();
    assert_eq!(
        output_segment_endpoint_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_endpoint_cache.output_endpoint_ref_count(),
        result.source_report_count().unwrap() * 2
    );
    assert_eq!(
        output_segment_endpoint_cache.segments().len(),
        result.source_report_count().unwrap()
    );
    let endpoint_ref = &output_segment_endpoint_cache.segments()[0];
    assert_eq!(endpoint_ref.source_report_index(), 0);
    assert_eq!(
        endpoint_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        endpoint_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        endpoint_ref.output_start_point(),
        result.source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        endpoint_ref.output_end_point(),
        result.source_reports().unwrap()[0].output_end_point()
    );
    let output_ring_continuity_cache = result.output_ring_continuity_cache().unwrap();
    assert_eq!(
        output_ring_continuity_cache.output_ring_ref_count(),
        result.output_ring_bucket_cache().unwrap().ring_count()
    );
    assert_eq!(
        output_ring_continuity_cache.output_connection_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_ring_continuity_cache.max_ring_connection_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_ring_continuity_cache.connections().len(),
        result.source_report_count().unwrap()
    );
    let continuity_ref = &output_ring_continuity_cache.connections()[0];
    assert_eq!(continuity_ref.source_report_index(), 0);
    assert_eq!(continuity_ref.next_source_report_index(), 1);
    assert_eq!(
        continuity_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        continuity_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        continuity_ref.next_output_segment_index(),
        result.source_reports().unwrap()[1].output_segment_index()
    );
    assert_eq!(
        continuity_ref.output_end_point(),
        result.source_reports().unwrap()[0].output_end_point()
    );
    assert_eq!(
        continuity_ref.next_output_start_point(),
        result.source_reports().unwrap()[1].output_start_point()
    );
    assert_eq!(
        continuity_ref.output_end_point(),
        continuity_ref.next_output_start_point()
    );
    let closing_continuity_ref = output_ring_continuity_cache.connections().last().unwrap();
    assert_eq!(closing_continuity_ref.next_source_report_index(), 0);
    assert_eq!(
        closing_continuity_ref.output_end_point(),
        closing_continuity_ref.next_output_start_point()
    );
    let output_segment_status_bucket_cache = result.output_segment_status_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_status_bucket_count(),
        Some(output_segment_status_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_status_ref_count(),
        Some(output_segment_status_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_native_exact_segment_ref_count(),
        Some(output_segment_status_bucket_cache.native_exact_ref_count())
    );
    assert_eq!(
        result.output_certified_approximation_segment_ref_count(),
        Some(output_segment_status_bucket_cache.certified_approximation_ref_count())
    );
    assert_eq!(
        result.output_display_or_export_segment_ref_count(),
        Some(output_segment_status_bucket_cache.display_or_export_ref_count())
    );
    assert_eq!(
        result.output_imported_lossy_segment_ref_count(),
        Some(output_segment_status_bucket_cache.imported_lossy_ref_count())
    );
    assert_eq!(
        result.output_unsupported_segment_ref_count(),
        Some(output_segment_status_bucket_cache.unsupported_ref_count())
    );
    assert_eq!(
        result.output_unresolved_segment_ref_count(),
        Some(output_segment_status_bucket_cache.unresolved_ref_count())
    );
    assert_eq!(
        result.output_segment_status_max_bucket_size(),
        Some(output_segment_status_bucket_cache.max_bucket_size())
    );
    assert_eq!(output_segment_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        output_segment_status_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.native_exact_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.display_or_export_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.imported_lossy_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.unsupported_ref_count(),
        0
    );
    assert_eq!(output_segment_status_bucket_cache.unresolved_ref_count(), 0);
    assert_eq!(output_segment_status_bucket_cache.max_bucket_size(), 4);
    assert_eq!(output_segment_status_bucket_cache.buckets().len(), 6);
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].source_report_index(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        result.source_reports().unwrap()[output_segment_status_bucket_cache.buckets()[0]
            .segment_refs()[0]
            .source_report_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        output_segment_status_bucket_cache.buckets()[1]
            .segment_refs()
            .is_empty()
    );
    let output_segment_direction_bucket_cache =
        result.output_segment_direction_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_direction_bucket_count(),
        Some(output_segment_direction_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_direction_ref_count(),
        Some(output_segment_direction_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_forward_segment_ref_count(),
        Some(output_segment_direction_bucket_cache.forward_segment_ref_count())
    );
    assert_eq!(
        result.output_reversed_segment_ref_count(),
        Some(output_segment_direction_bucket_cache.reversed_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_direction_max_bucket_size(),
        Some(output_segment_direction_bucket_cache.max_bucket_size())
    );
    let reversed_source_segment_count = result.reversed_source_segment_count().unwrap();
    let forward_source_segment_count = result
        .source_report_count()
        .unwrap()
        .saturating_sub(reversed_source_segment_count);
    assert_eq!(output_segment_direction_bucket_cache.bucket_count(), 2);
    assert_eq!(
        output_segment_direction_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_direction_bucket_cache.forward_segment_ref_count(),
        forward_source_segment_count
    );
    assert_eq!(
        output_segment_direction_bucket_cache.reversed_segment_ref_count(),
        reversed_source_segment_count
    );
    assert_eq!(
        output_segment_direction_bucket_cache.max_bucket_size(),
        forward_source_segment_count.max(reversed_source_segment_count)
    );
    assert_eq!(output_segment_direction_bucket_cache.buckets().len(), 2);
    assert!(!output_segment_direction_bucket_cache.buckets()[0].reversed());
    assert_eq!(
        output_segment_direction_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        forward_source_segment_count
    );
    if forward_source_segment_count > 0 {
        let forward_ref = &output_segment_direction_bucket_cache.buckets()[0].segment_refs()[0];
        assert_eq!(
            forward_ref.output_ring_index(),
            result.source_reports().unwrap()[forward_ref.source_report_index()].output_ring_index()
        );
        assert_eq!(
            forward_ref.output_segment_index(),
            result.source_reports().unwrap()[forward_ref.source_report_index()]
                .output_segment_index()
        );
        assert!(!result.source_reports().unwrap()[forward_ref.source_report_index()].reversed());
    }
    assert!(output_segment_direction_bucket_cache.buckets()[1].reversed());
    assert_eq!(
        output_segment_direction_bucket_cache.buckets()[1]
            .segment_refs()
            .len(),
        reversed_source_segment_count
    );
    if reversed_source_segment_count > 0 {
        let reversed_ref = &output_segment_direction_bucket_cache.buckets()[1].segment_refs()[0];
        assert_eq!(
            reversed_ref.output_ring_index(),
            result.source_reports().unwrap()[reversed_ref.source_report_index()]
                .output_ring_index()
        );
        assert_eq!(
            reversed_ref.output_segment_index(),
            result.source_reports().unwrap()[reversed_ref.source_report_index()]
                .output_segment_index()
        );
        assert!(result.source_reports().unwrap()[reversed_ref.source_report_index()].reversed());
    }
    assert!(result.evaluated_output());
    assert_eq!(result.materialized_region(), Some(true));
    assert_eq!(
        result.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RegionRoleAssignment)
    );
    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.blocker(), None);
    assert_eq!(result.output_ring_count(), Some(1));
    assert_eq!(result.output_boundary_segment_count(), Some(4));
    assert_eq!(
        result.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(result.output_contour_count(), Some(1));
    assert_eq!(result.output_segment_count(), Some(4));
    let summary_cache = result.summary_cache();
    assert!(summary_cache.evaluated_output());
    assert_eq!(summary_cache.materialized_region(), Some(true));
    assert_eq!(summary_cache.stage(), result.stage());
    assert_eq!(summary_cache.status(), result.status());
    assert_eq!(summary_cache.blocker(), None);
    assert_eq!(
        summary_cache.output_ring_count(),
        result.output_ring_count()
    );
    assert_eq!(
        summary_cache.output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        summary_cache.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(summary_cache.output_contour_count(), Some(1));
    assert_eq!(summary_cache.output_segment_count(), Some(4));
    let evaluation = result.evaluation();
    assert_eq!(evaluation.summary_cache(), summary_cache);
    assert_eq!(evaluation.evaluated_output(), result.evaluated_output());
    assert_eq!(
        evaluation.materialized_region(),
        result.materialized_region()
    );
    assert_eq!(evaluation.stage(), result.stage());
    assert_eq!(evaluation.status(), result.status());
    assert_eq!(evaluation.blocker(), result.blocker());
    assert_eq!(evaluation.output_ring_count(), result.output_ring_count());
    assert_eq!(
        evaluation.output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        evaluation.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(
        evaluation.output_contour_count(),
        result.output_contour_count()
    );
    assert_eq!(
        evaluation.output_segment_count(),
        result.output_segment_count()
    );
    let arrangement_report = result.arrangement_report();
    assert_eq!(arrangement_report.fill_rule(), result.fill_rule());
    assert_eq!(
        arrangement_report.evaluated_output(),
        result.evaluated_output()
    );
    assert_eq!(
        arrangement_report.source_segments(),
        result.source_segments()
    );
    assert_eq!(
        arrangement_report.source_line_segments(),
        result.source_line_segments()
    );
    assert_eq!(
        arrangement_report.source_segment_cache(),
        result.source_segment_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_cache(),
        result.source_endpoint_bucket_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_count(),
        result.source_endpoint_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_count(),
        result.source_endpoint_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_singleton_bucket_count(),
        result.source_endpoint_singleton_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_max_bucket_size(),
        result.source_endpoint_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.split_schedule_cache(),
        result.split_schedule_cache()
    );
    assert_eq!(
        arrangement_report.split_schedule_candidate_pair_count(),
        result.split_schedule_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_decided_disjoint_pair_count(),
        result.split_schedule_decided_disjoint_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_predicate_candidate_pair_count(),
        result.split_schedule_predicate_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_undecided_aabb_pair_count(),
        result.split_schedule_undecided_aabb_pair_count()
    );
    assert_eq!(arrangement_report.split_cache(), result.split_cache());
    assert_eq!(
        arrangement_report.split_relation_bucket_cache(),
        result.split_relation_bucket_cache()
    );
    assert_eq!(
        arrangement_report.split_intersection_bucket_cache(),
        result.split_intersection_bucket_cache()
    );
    assert_eq!(
        arrangement_report.split_intersection_parameter_cache(),
        result.split_intersection_parameter_cache()
    );
    assert_eq!(
        arrangement_report.split_blocker_cache(),
        result.split_blocker_cache()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_cache(),
        result.endpoint_graph_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_bucket_cache(),
        result.arranged_endpoint_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_bucket_cache(),
        result.arranged_endpoint_side_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_bucket_count(),
        result.arranged_endpoint_side_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_ref_count(),
        result.arranged_endpoint_side_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_start_ref_count(),
        result.arranged_endpoint_start_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_end_ref_count(),
        result.arranged_endpoint_end_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_max_bucket_size(),
        result.arranged_endpoint_side_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_cache(),
        result.arranged_endpoint_point_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_fragment_ref_count(),
        result.arranged_endpoint_point_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_ref_count(),
        result.arranged_endpoint_point_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_bucket_cache(),
        result.arranged_endpoint_degree_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_bucket_count(),
        result.arranged_endpoint_degree_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_structural_bucket_ref_count(),
        result.arranged_endpoint_degree_structural_bucket_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_dangling_structural_bucket_count(),
        result.arranged_endpoint_dangling_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_chain_structural_bucket_count(),
        result.arranged_endpoint_chain_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_branch_structural_bucket_count(),
        result.arranged_endpoint_branch_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_max_bucket_size(),
        result.arranged_endpoint_degree_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.ring_assembly_cache(),
        result.ring_assembly_cache()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_cache(),
        result.arranged_fragment_cache()
    );
    assert_eq!(arrangement_report.output_cache(), result.output_cache());
    assert_eq!(
        arrangement_report.output_ring_bucket_cache(),
        result.output_ring_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_ring_segment_ref_count(),
        result.output_ring_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_ring_max_segment_count(),
        result.output_ring_max_segment_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_bucket_cache(),
        result.output_segment_kind_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_bucket_count(),
        result.output_segment_kind_bucket_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_ref_count(),
        result.output_segment_kind_ref_count()
    );
    assert_eq!(
        arrangement_report.output_line_segment_ref_count(),
        result.output_line_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_arc_segment_ref_count(),
        result.output_arc_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_max_bucket_size(),
        result.output_segment_kind_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.output_segment_source_bucket_cache(),
        result.output_segment_source_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_source_bucket_count(),
        result.output_segment_source_bucket_count()
    );
    assert_eq!(
        arrangement_report.output_segment_source_ref_count(),
        result.output_segment_source_ref_count()
    );
    assert_eq!(
        arrangement_report.output_segment_source_max_bucket_size(),
        result.output_segment_source_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.output_segment_source_range_cache(),
        result.output_segment_source_range_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_endpoint_cache(),
        result.output_segment_endpoint_cache()
    );
    assert_eq!(
        arrangement_report.output_ring_continuity_cache(),
        result.output_ring_continuity_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_status_bucket_cache(),
        result.output_segment_status_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_status_bucket_count(),
        result.output_segment_status_bucket_count()
    );
    assert_eq!(
        arrangement_report.output_segment_status_ref_count(),
        result.output_segment_status_ref_count()
    );
    assert_eq!(
        arrangement_report.output_native_exact_segment_ref_count(),
        result.output_native_exact_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_certified_approximation_segment_ref_count(),
        result.output_certified_approximation_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_display_or_export_segment_ref_count(),
        result.output_display_or_export_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_imported_lossy_segment_ref_count(),
        result.output_imported_lossy_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_unsupported_segment_ref_count(),
        result.output_unsupported_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_unresolved_segment_ref_count(),
        result.output_unresolved_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_segment_status_max_bucket_size(),
        result.output_segment_status_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.output_segment_direction_bucket_cache(),
        result.output_segment_direction_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_direction_bucket_count(),
        result.output_segment_direction_bucket_count()
    );
    assert_eq!(
        arrangement_report.output_segment_direction_ref_count(),
        result.output_segment_direction_ref_count()
    );
    assert_eq!(
        arrangement_report.output_forward_segment_ref_count(),
        result.output_forward_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_reversed_segment_ref_count(),
        result.output_reversed_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_segment_direction_max_bucket_size(),
        result.output_segment_direction_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.boundary_output_cache(),
        result.boundary_output_cache()
    );
    assert_eq!(
        arrangement_report.boundary_output_role_bucket_cache(),
        result.boundary_output_role_bucket_cache()
    );
    assert_eq!(arrangement_report.role_cache(), result.role_cache());
    assert_eq!(arrangement_report.role_buckets(), result.role_buckets());
    assert_eq!(
        arrangement_report.role_status_bucket_cache(),
        result.role_status_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_source_contour_bucket_cache(),
        result.role_source_contour_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_nesting_depth_bucket_cache(),
        result.role_nesting_depth_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_containment_bucket_cache(),
        result.role_containment_bucket_cache()
    );
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
    assert_eq!(
        arrangement_report.source_segment_count(),
        result.source_segment_count()
    );
    assert_eq!(
        arrangement_report.source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.source_segment_aabbs(),
        result.source_segment_aabbs()
    );
    assert_eq!(arrangement_report.source_aabb(), result.source_aabb());
    assert_eq!(
        arrangement_report.decided_source_segment_aabb_count(),
        result.decided_source_segment_aabb_count()
    );
    assert_eq!(
        arrangement_report.split_candidate_pair_count(),
        result.split_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_tested_pair_count(),
        result.split_tested_pair_count()
    );
    assert_eq!(
        arrangement_report.split_point_relation_count(),
        result.split_point_relation_count()
    );
    assert_eq!(
        arrangement_report.split_intersection_points(),
        result.split_intersection_points()
    );
    assert_eq!(
        arrangement_report.split_intersection_reports(),
        result.split_intersection_reports()
    );
    assert_eq!(
        arrangement_report.split_predicate_path(),
        result.split_predicate_path()
    );
    assert_eq!(
        arrangement_report.split_output_segment_count(),
        result.split_output_segment_count()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_index(),
        result.split_blocker_first_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_kind(),
        result.split_blocker_first_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_start_point(),
        result.split_blocker_first_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_end_point(),
        result.split_blocker_first_source_end_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_index(),
        result.split_blocker_second_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_kind(),
        result.split_blocker_second_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_start_point(),
        result.split_blocker_second_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_end_point(),
        result.split_blocker_second_source_end_point()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_predicate_path(),
        result.endpoint_graph_predicate_path()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_structural_bucket_count(),
        result.endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_structural_singleton_bucket_count(),
        result.endpoint_graph_structural_singleton_bucket_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_max_structural_bucket_size(),
        result.endpoint_graph_max_structural_bucket_size()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_dangling_endpoint_count(),
        result.endpoint_graph_dangling_endpoint_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_branch_endpoint_count(),
        result.endpoint_graph_branch_endpoint_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_arranged_segment_index(),
        result.endpoint_graph_blocker_arranged_segment_index()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_endpoint(),
        result.endpoint_graph_blocker_endpoint()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_point(),
        result.endpoint_graph_blocker_point()
    );
    assert_eq!(
        arrangement_report.ring_assembly_predicate_path(),
        result.ring_assembly_predicate_path()
    );
    assert_eq!(
        arrangement_report.attempted_endpoint_connection_count(),
        result.attempted_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.exact_endpoint_connection_count(),
        result.exact_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.disconnected_endpoint_connection_count(),
        result.disconnected_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.unresolved_endpoint_connection_count(),
        result.unresolved_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.reversed_source_segment_count(),
        result.reversed_source_segment_count()
    );
    assert_eq!(
        arrangement_report.arranged_segment_count(),
        result.arranged_segment_count()
    );
    assert_eq!(
        arrangement_report.arranged_segment_kind_counts(),
        result.arranged_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_bucket_count(),
        result.arranged_fragment_kind_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_ref_count(),
        result.arranged_fragment_kind_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_line_fragment_ref_count(),
        result.arranged_line_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_arc_fragment_ref_count(),
        result.arranged_arc_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_max_bucket_size(),
        result.arranged_fragment_kind_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_bucket_count(),
        result.arranged_fragment_status_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_source_ref_count(),
        result.arranged_fragment_status_source_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_native_exact_ref_count(),
        result.arranged_fragment_native_exact_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_certified_approximation_ref_count(),
        result.arranged_fragment_certified_approximation_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_display_or_export_ref_count(),
        result.arranged_fragment_display_or_export_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_imported_lossy_ref_count(),
        result.arranged_fragment_imported_lossy_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_unsupported_ref_count(),
        result.arranged_fragment_unsupported_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_unresolved_ref_count(),
        result.arranged_fragment_unresolved_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_max_bucket_size(),
        result.arranged_fragment_status_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_source_report_count(),
        result.arranged_source_report_count()
    );
    assert_eq!(
        arrangement_report.arranged_source_reports(),
        result.arranged_source_reports()
    );
    assert_eq!(
        arrangement_report.source_report_count(),
        result.source_report_count()
    );
    assert_eq!(arrangement_report.source_reports(), result.source_reports());
    assert_eq!(
        arrangement_report.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_counts(),
        result.output_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.material_contour_count(),
        result.material_contour_count()
    );
    assert_eq!(
        arrangement_report.hole_contour_count(),
        result.hole_contour_count()
    );
    assert_eq!(
        arrangement_report.material_segment_count(),
        result.material_segment_count()
    );
    assert_eq!(
        arrangement_report.hole_segment_count(),
        result.hole_segment_count()
    );
    assert_eq!(
        arrangement_report.role_report_count(),
        result.role_report_count()
    );
    assert_eq!(arrangement_report.role_reports(), result.role_reports());
    assert_eq!(
        arrangement_report.boundary_build_report(),
        result.boundary_build_report()
    );
    assert_eq!(
        arrangement_report.boundary_build_stage(),
        result.boundary_build_stage()
    );
    assert_eq!(
        arrangement_report.boundary_build_predicate_path(),
        result.boundary_build_predicate_path()
    );
    assert_eq!(
        arrangement_report.boundary_build_status(),
        result.boundary_build_status()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker(),
        result.boundary_build_blocker()
    );
    assert_eq!(
        arrangement_report.boundary_build_source_contour_count(),
        result.boundary_build_source_contour_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_source_segment_count(),
        result.boundary_build_source_segment_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_candidate_pair_count(),
        result.boundary_build_validation_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_tested_pair_count(),
        result.boundary_build_validation_tested_pair_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_intersection_event_count(),
        result.boundary_build_validation_intersection_event_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_nesting_classification_count(),
        result.boundary_build_nesting_classification_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_first_contour_index(),
        result.boundary_build_blocker_first_contour_index()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_second_contour_index(),
        result.boundary_build_blocker_second_contour_index()
    );
    assert_eq!(
        arrangement_report.materialized_region(),
        result.materialized_region()
    );
    assert_eq!(arrangement_report.stage(), result.stage());
    assert_eq!(arrangement_report.status(), result.status());
    assert_eq!(arrangement_report.blocker(), result.blocker());
    assert_eq!(
        result.boundary_build_stage(),
        Some(RegionBoundaryContourBuildStage2::RoleAssignment)
    );
    assert_eq!(
        result.boundary_build_status(),
        Some(RetainedTopologyStatus::NativeExact)
    );
    assert_eq!(result.boundary_build_blocker(), None);
    assert_eq!(
        result.boundary_build_validation_intersection_event_count(),
        Some(0)
    );
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(0));
    assert_eq!(result.output_contour_count(), Some(1));
    assert_eq!(result.output_segment_count(), Some(4));
    assert_eq!(
        result.output_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(0));
    assert_eq!(result.material_segment_count(), Some(4));
    assert_eq!(result.hole_segment_count(), Some(0));
    let boundary_role_bucket_cache = result.boundary_output_role_bucket_cache().unwrap();
    assert_eq!(boundary_role_bucket_cache.bucket_count(), 2);
    assert_eq!(boundary_role_bucket_cache.output_contour_count(), 1);
    assert_eq!(boundary_role_bucket_cache.output_segment_count(), 4);
    assert_eq!(boundary_role_bucket_cache.max_segment_count(), 4);
    assert_eq!(boundary_role_bucket_cache.buckets().len(), 2);
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_contour_count(),
        1
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_segment_count(),
        4
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_contour_count(),
        0
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_segment_count(),
        0
    );
    let role_reports = result.role_reports().unwrap();
    assert_eq!(result.role_report_count().unwrap(), role_reports.len());
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(0));
    assert_eq!(result.material_segment_count(), Some(4));
    assert_eq!(result.hole_segment_count(), Some(0));
    let role_status_bucket_cache = result.role_status_bucket_cache().unwrap();
    assert_eq!(role_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        role_status_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.native_exact_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(role_status_bucket_cache.display_or_export_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.imported_lossy_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.unsupported_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.unresolved_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.max_bucket_size(), 1);
    assert_eq!(role_status_bucket_cache.buckets().len(), 6);
    assert_eq!(
        role_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments().len(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].assignment_index(),
        0
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].role_report_index(),
        0
    );
    assert_eq!(
        result.role_buckets().unwrap()[0].assignments()
            [role_status_bucket_cache.buckets()[0].assignments()[0].assignment_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        role_status_bucket_cache.buckets()[1]
            .assignments()
            .is_empty()
    );
    let role_source_contour_bucket_cache = result.role_source_contour_bucket_cache().unwrap();
    assert_eq!(
        role_source_contour_bucket_cache.source_contour_bucket_count(),
        role_reports.len()
    );
    assert_eq!(
        role_source_contour_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(role_source_contour_bucket_cache.max_bucket_size(), 1);
    assert_eq!(role_source_contour_bucket_cache.buckets().len(), 1);
    assert_eq!(
        role_source_contour_bucket_cache.buckets()[0].source_contour_index(),
        role_reports[0].source_contour_index()
    );
    assert_eq!(
        role_source_contour_bucket_cache.buckets()[0]
            .assignments()
            .len(),
        1
    );
    let source_contour_assignment = &role_source_contour_bucket_cache.buckets()[0].assignments()[0];
    assert_eq!(
        source_contour_assignment.role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(source_contour_assignment.assignment_index(), 0);
    assert_eq!(source_contour_assignment.role_report_index(), 0);
    assert_eq!(source_contour_assignment.output_role_index(), 0);
    let role_nesting_depth_bucket_cache = result.role_nesting_depth_bucket_cache().unwrap();
    assert_eq!(
        role_nesting_depth_bucket_cache.nesting_depth_bucket_count(),
        1
    );
    assert_eq!(
        role_nesting_depth_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(role_nesting_depth_bucket_cache.max_bucket_size(), 1);
    assert_eq!(role_nesting_depth_bucket_cache.buckets().len(), 1);
    assert_eq!(
        role_nesting_depth_bucket_cache.buckets()[0].nesting_depth(),
        role_reports[0].nesting_depth()
    );
    assert_eq!(
        role_nesting_depth_bucket_cache.buckets()[0]
            .assignments()
            .len(),
        1
    );
    let nesting_depth_assignment = &role_nesting_depth_bucket_cache.buckets()[0].assignments()[0];
    assert_eq!(
        nesting_depth_assignment.role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(nesting_depth_assignment.assignment_index(), 0);
    assert_eq!(nesting_depth_assignment.role_report_index(), 0);
    assert_eq!(nesting_depth_assignment.source_contour_index(), 0);
    assert_eq!(nesting_depth_assignment.output_role_index(), 0);
    assert_eq!(result.role_buckets().unwrap().len(), 2);
    assert_eq!(
        result.role_buckets().unwrap()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(result.role_buckets().unwrap()[0].assignments().len(), 1);
    assert_eq!(
        result.role_buckets().unwrap()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert!(result.role_buckets().unwrap()[1].assignments().is_empty());
    let role_assignment = &result.role_buckets().unwrap()[0].assignments()[0];
    assert_eq!(role_assignment.role_report_index(), 0);
    assert_eq!(role_assignment.source_contour_index(), 0);
    assert_eq!(role_assignment.source_segment_count(), 4);
    assert_eq!(role_assignment.source_fill_rule(), FillRule::NonZero);
    assert_eq!(
        role_assignment.nesting_sample_point(),
        role_reports[0].nesting_sample_point()
    );
    assert_eq!(role_assignment.containing_contour_indices(), &[]);
    assert_eq!(role_assignment.nesting_depth(), 0);
    assert_eq!(role_assignment.output_role_index(), 0);
    assert_eq!(role_assignment.status(), role_reports[0].status());
    assert!(result.region().is_some());
    assert_eq!(
        result.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
}

#[test]
fn exact_curve_arrangement_attempt_retains_output_role_containment() {
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
    let request = ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(
        &lines,
        FillRule::NonZero,
    );
    let result = ExactCurveArrangementAttempt2::new(request)
        .evaluate_owned(&policy())
        .unwrap();

    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.output_ring_count(), Some(2));
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(1));
    assert_eq!(result.role_report_count(), Some(2));

    let boundary_role_bucket_cache = result.boundary_output_role_bucket_cache().unwrap();
    assert_eq!(boundary_role_bucket_cache.bucket_count(), 2);
    assert_eq!(boundary_role_bucket_cache.output_contour_count(), 2);
    assert_eq!(boundary_role_bucket_cache.output_segment_count(), 8);
    assert_eq!(boundary_role_bucket_cache.max_segment_count(), 4);
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_contour_count(),
        1
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_segment_count(),
        4
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_contour_count(),
        1
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_segment_count(),
        4
    );

    let containment_cache = result.role_containment_bucket_cache().unwrap();
    assert_eq!(containment_cache.containing_contour_bucket_count(), 1);
    assert_eq!(containment_cache.containment_ref_count(), 1);
    assert_eq!(containment_cache.uncontained_assignment_ref_count(), 1);
    assert_eq!(containment_cache.max_bucket_size(), 1);
    assert_eq!(containment_cache.buckets().len(), 1);
    let containment_bucket = &containment_cache.buckets()[0];
    assert_eq!(containment_bucket.containing_contour_index(), 0);
    assert_eq!(containment_bucket.containments().len(), 1);
    let containment = &containment_bucket.containments()[0];
    assert_eq!(containment.role(), RegionBoundaryContourRole2::Hole);
    assert_eq!(containment.assignment_index(), 0);
    assert_eq!(containment.role_report_index(), 1);
    assert_eq!(containment.source_contour_index(), 1);
    assert_eq!(containment.containing_contour_index(), 0);
    assert_eq!(containment.containing_contour_ref_index(), 0);
    assert_eq!(containment.output_role_index(), 0);
    assert_eq!(
        result.role_buckets().unwrap()[1].assignments()[containment.assignment_index()]
            .containing_contour_indices(),
        &[0]
    );

    let nesting_depth_cache = result.role_nesting_depth_bucket_cache().unwrap();
    assert_eq!(nesting_depth_cache.nesting_depth_bucket_count(), 2);
    assert_eq!(nesting_depth_cache.assignment_ref_count(), 2);
    assert_eq!(nesting_depth_cache.buckets()[0].nesting_depth(), 0);
    assert_eq!(nesting_depth_cache.buckets()[1].nesting_depth(), 1);
}

#[test]
fn exact_curve_arrangement_attempt_builds_native_region_with_retained_workspace() {
    let segments = vec![
        Segment2::Line(line(4, 0, 0, 0)),
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
    ];
    let request = ExactCurveArrangementRequest2::from_borrowed_unordered_segments(
        &segments,
        FillRule::NonZero,
    );
    let attempt = ExactCurveArrangementAttempt2::new(request);
    let result = attempt.evaluate(&policy()).unwrap();
    let owned_result = attempt.clone().evaluate_owned(&policy()).unwrap();
    assert_eq!(owned_result, result);

    assert_eq!(result.source_segment_count(), 2);
    assert_eq!(result.fill_rule(), FillRule::NonZero);
    assert_eq!(result.source_segments(), segments.as_slice());
    assert_eq!(result.request(), attempt.request());
    assert_eq!(result.evaluation().workspace(), result.workspace());
    assert_eq!(result.evaluation().workspace().request(), result.request());
    assert_eq!(result.evaluation().request(), result.request());
    assert_eq!(
        result.evaluation().source_segments(),
        result.source_segments()
    );
    assert_eq!(
        result.evaluation().source_line_segments(),
        result.source_line_segments()
    );
    assert_eq!(result.evaluation().fill_rule(), result.fill_rule());
    assert_eq!(
        result.evaluation().source_segment_count(),
        result.source_segment_count()
    );
    assert_eq!(
        result.evaluation().source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(
        result.evaluation().source_segment_aabbs(),
        result.source_segment_aabbs()
    );
    assert_eq!(result.evaluation().source_aabb(), result.source_aabb());
    assert_eq!(
        result.evaluation().decided_source_segment_aabb_count(),
        result.decided_source_segment_aabb_count()
    );
    assert_eq!(
        result.evaluation().undecided_source_segment_aabb_count(),
        result.undecided_source_segment_aabb_count()
    );
    assert_eq!(
        result.evaluation().source_segment_cache(),
        result.source_segment_cache()
    );
    assert_eq!(
        result.evaluation().source_aabb_bucket_cache(),
        result.source_aabb_bucket_cache()
    );
    assert_eq!(
        result.evaluation().source_segment_kind_bucket_cache(),
        result.source_segment_kind_bucket_cache()
    );
    assert_eq!(
        result.evaluation().source_endpoint_bucket_cache(),
        result.source_endpoint_bucket_cache()
    );
    assert_eq!(result.source_endpoint_count(), 4);
    assert_eq!(result.source_endpoint_bucket_count(), 2);
    assert_eq!(result.source_endpoint_singleton_bucket_count(), 0);
    assert_eq!(result.source_endpoint_max_bucket_size(), 2);
    assert_eq!(
        result.evaluation().source_endpoint_count(),
        result.source_endpoint_count()
    );
    assert_eq!(
        result.evaluation().source_endpoint_bucket_count(),
        result.source_endpoint_bucket_count()
    );
    assert_eq!(
        result.evaluation().source_endpoint_singleton_bucket_count(),
        result.source_endpoint_singleton_bucket_count()
    );
    assert_eq!(
        result.evaluation().source_endpoint_max_bucket_size(),
        result.source_endpoint_max_bucket_size()
    );
    assert_eq!(
        result.workspace().source_endpoint_bucket_count(),
        result.source_endpoint_bucket_count()
    );
    assert_eq!(
        result.evaluation().split_schedule_cache(),
        result.split_schedule_cache()
    );
    assert_eq!(result.split_schedule_candidate_pair_count(), 1);
    assert_eq!(result.split_schedule_decided_disjoint_pair_count(), 0);
    assert_eq!(result.split_schedule_predicate_candidate_pair_count(), 1);
    assert_eq!(result.split_schedule_undecided_aabb_pair_count(), 0);
    assert_eq!(
        result.evaluation().split_schedule_candidate_pair_count(),
        result.split_schedule_candidate_pair_count()
    );
    assert_eq!(
        result
            .evaluation()
            .split_schedule_decided_disjoint_pair_count(),
        result.split_schedule_decided_disjoint_pair_count()
    );
    assert_eq!(
        result
            .evaluation()
            .split_schedule_predicate_candidate_pair_count(),
        result.split_schedule_predicate_candidate_pair_count()
    );
    assert_eq!(
        result
            .evaluation()
            .split_schedule_undecided_aabb_pair_count(),
        result.split_schedule_undecided_aabb_pair_count()
    );
    assert_eq!(
        result.workspace().split_schedule_candidate_pair_count(),
        result.split_schedule_candidate_pair_count()
    );
    assert_eq!(result.evaluation().split_cache(), result.split_cache());
    assert_eq!(
        result.evaluation().split_predicate_path(),
        result.split_predicate_path()
    );
    assert_eq!(
        result.evaluation().split_candidate_pair_count(),
        result.split_candidate_pair_count()
    );
    assert_eq!(
        result.evaluation().split_skipped_aabb_pair_count(),
        result.split_skipped_aabb_pair_count()
    );
    assert_eq!(
        result.evaluation().split_tested_pair_count(),
        result.split_tested_pair_count()
    );
    assert_eq!(
        result.evaluation().split_intersection_event_count(),
        result.split_intersection_event_count()
    );
    assert_eq!(
        result.evaluation().split_point_relation_count(),
        result.split_point_relation_count()
    );
    assert_eq!(
        result.evaluation().split_overlap_relation_count(),
        result.split_overlap_relation_count()
    );
    assert_eq!(
        result.evaluation().split_uncertain_relation_count(),
        result.split_uncertain_relation_count()
    );
    assert_eq!(
        result.evaluation().split_intersection_points(),
        result.split_intersection_points()
    );
    assert_eq!(
        result.evaluation().split_intersection_reports(),
        result.split_intersection_reports()
    );
    assert_eq!(
        result.evaluation().split_relation_bucket_cache(),
        result.split_relation_bucket_cache()
    );
    assert_eq!(
        result.evaluation().split_intersection_bucket_cache(),
        result.split_intersection_bucket_cache()
    );
    assert_eq!(
        result.evaluation().split_intersection_parameter_cache(),
        result.split_intersection_parameter_cache()
    );
    assert_eq!(
        result.evaluation().split_blocker_cache(),
        result.split_blocker_cache()
    );
    assert_eq!(
        result
            .evaluation()
            .split_blocker_first_source_segment_index(),
        result.split_blocker_first_source_segment_index()
    );
    assert_eq!(
        result
            .evaluation()
            .split_blocker_first_source_segment_kind(),
        result.split_blocker_first_source_segment_kind()
    );
    assert_eq!(
        result.evaluation().split_blocker_first_source_start_point(),
        result.split_blocker_first_source_start_point()
    );
    assert_eq!(
        result.evaluation().split_blocker_first_source_end_point(),
        result.split_blocker_first_source_end_point()
    );
    assert_eq!(
        result
            .evaluation()
            .split_blocker_second_source_segment_index(),
        result.split_blocker_second_source_segment_index()
    );
    assert_eq!(
        result
            .evaluation()
            .split_blocker_second_source_segment_kind(),
        result.split_blocker_second_source_segment_kind()
    );
    assert_eq!(
        result
            .evaluation()
            .split_blocker_second_source_start_point(),
        result.split_blocker_second_source_start_point()
    );
    assert_eq!(
        result.evaluation().split_blocker_second_source_end_point(),
        result.split_blocker_second_source_end_point()
    );
    assert_eq!(
        result.evaluation().split_output_segment_count(),
        result.split_output_segment_count()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_cache(),
        result.endpoint_graph_cache()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_predicate_path(),
        result.endpoint_graph_predicate_path()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_endpoint_count(),
        result.endpoint_graph_endpoint_count()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_structural_bucket_count(),
        result.endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        result
            .evaluation()
            .endpoint_graph_structural_singleton_bucket_count(),
        result.endpoint_graph_structural_singleton_bucket_count()
    );
    assert_eq!(
        result
            .evaluation()
            .endpoint_graph_max_structural_bucket_size(),
        result.endpoint_graph_max_structural_bucket_size()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_bucket_cache(),
        result.arranged_endpoint_bucket_cache()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_side_bucket_cache(),
        result.arranged_endpoint_side_bucket_cache()
    );
    assert_eq!(result.arranged_endpoint_side_bucket_count(), Some(2));
    assert_eq!(
        result.arranged_endpoint_side_ref_count(),
        result.endpoint_graph_endpoint_count()
    );
    assert_eq!(result.arranged_endpoint_start_ref_count(), Some(2));
    assert_eq!(result.arranged_endpoint_end_ref_count(), Some(2));
    assert_eq!(result.arranged_endpoint_side_max_bucket_size(), Some(2));
    assert_eq!(
        result.evaluation().arranged_endpoint_side_bucket_count(),
        result.arranged_endpoint_side_bucket_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_side_ref_count(),
        result.arranged_endpoint_side_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_start_ref_count(),
        result.arranged_endpoint_start_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_end_ref_count(),
        result.arranged_endpoint_end_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_point_cache(),
        result.arranged_endpoint_point_cache()
    );
    assert_eq!(result.arranged_endpoint_point_fragment_ref_count(), Some(2));
    assert_eq!(
        result.arranged_endpoint_point_ref_count(),
        result.endpoint_graph_endpoint_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_endpoint_point_fragment_ref_count(),
        result.arranged_endpoint_point_fragment_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_point_ref_count(),
        result.arranged_endpoint_point_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_endpoint_degree_bucket_cache(),
        result.arranged_endpoint_degree_bucket_cache()
    );
    assert_eq!(result.arranged_endpoint_degree_bucket_count(), Some(3));
    assert_eq!(
        result.arranged_endpoint_degree_structural_bucket_ref_count(),
        result.endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        result.arranged_endpoint_dangling_structural_bucket_count(),
        Some(0)
    );
    assert_eq!(
        result.arranged_endpoint_chain_structural_bucket_count(),
        result.endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        result.arranged_endpoint_branch_structural_bucket_count(),
        Some(0)
    );
    assert_eq!(result.arranged_endpoint_degree_max_bucket_size(), Some(2));
    assert_eq!(
        result.evaluation().arranged_endpoint_degree_bucket_count(),
        result.arranged_endpoint_degree_bucket_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_endpoint_degree_structural_bucket_ref_count(),
        result.arranged_endpoint_degree_structural_bucket_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_endpoint_chain_structural_bucket_count(),
        result.arranged_endpoint_chain_structural_bucket_count()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_dangling_endpoint_count(),
        result.endpoint_graph_dangling_endpoint_count()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_branch_endpoint_count(),
        result.endpoint_graph_branch_endpoint_count()
    );
    assert_eq!(
        result
            .evaluation()
            .endpoint_graph_blocker_arranged_segment_index(),
        result.endpoint_graph_blocker_arranged_segment_index()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_blocker_endpoint(),
        result.endpoint_graph_blocker_endpoint()
    );
    assert_eq!(
        result.evaluation().endpoint_graph_blocker_point(),
        result.endpoint_graph_blocker_point()
    );
    assert_eq!(
        result.evaluation().ring_assembly_cache(),
        result.ring_assembly_cache()
    );
    assert_eq!(
        result.evaluation().ring_assembly_predicate_path(),
        result.ring_assembly_predicate_path()
    );
    assert_eq!(
        result.evaluation().attempted_endpoint_connection_count(),
        result.attempted_endpoint_connection_count()
    );
    assert_eq!(
        result.evaluation().exact_endpoint_connection_count(),
        result.exact_endpoint_connection_count()
    );
    assert_eq!(
        result.evaluation().disconnected_endpoint_connection_count(),
        result.disconnected_endpoint_connection_count()
    );
    assert_eq!(
        result.evaluation().unresolved_endpoint_connection_count(),
        result.unresolved_endpoint_connection_count()
    );
    assert_eq!(
        result.evaluation().reversed_source_segment_count(),
        result.reversed_source_segment_count()
    );
    assert_eq!(
        result.evaluation().arranged_source_reports(),
        result.arranged_source_reports()
    );
    assert_eq!(
        result.evaluation().arranged_source_report_count(),
        result.arranged_source_report_count()
    );
    assert_eq!(
        result.evaluation().source_reports(),
        result.source_reports()
    );
    assert_eq!(
        result.evaluation().source_report_count(),
        result.source_report_count()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_cache(),
        result.arranged_fragment_cache()
    );
    assert_eq!(
        result.evaluation().arranged_segment_count(),
        result.arranged_segment_count()
    );
    assert_eq!(
        result.evaluation().arranged_segment_kind_counts(),
        result.arranged_segment_kind_counts()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_kind_bucket_cache(),
        result.arranged_fragment_kind_bucket_cache()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_kind_bucket_count(),
        result.arranged_fragment_kind_bucket_count()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_kind_ref_count(),
        result.arranged_fragment_kind_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_line_fragment_ref_count(),
        result.arranged_line_fragment_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_arc_fragment_ref_count(),
        result.arranged_arc_fragment_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_kind_max_bucket_size(),
        result.arranged_fragment_kind_max_bucket_size()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_status_bucket_cache(),
        result.arranged_fragment_status_bucket_cache()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_status_bucket_count(),
        result.arranged_fragment_status_bucket_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_status_source_ref_count(),
        result.arranged_fragment_status_source_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_native_exact_ref_count(),
        result.arranged_fragment_native_exact_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_certified_approximation_ref_count(),
        result.arranged_fragment_certified_approximation_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_display_or_export_ref_count(),
        result.arranged_fragment_display_or_export_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_imported_lossy_ref_count(),
        result.arranged_fragment_imported_lossy_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_unsupported_ref_count(),
        result.arranged_fragment_unsupported_ref_count()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_unresolved_ref_count(),
        result.arranged_fragment_unresolved_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_status_max_bucket_size(),
        result.arranged_fragment_status_max_bucket_size()
    );
    assert_eq!(
        result.evaluation().arranged_fragment_source_range_cache(),
        result.arranged_fragment_source_range_cache()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_source_range_ref_count(),
        result.arranged_fragment_source_range_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_full_source_range_ref_count(),
        result.arranged_fragment_full_source_range_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .arranged_fragment_partial_source_range_ref_count(),
        result.arranged_fragment_partial_source_range_ref_count()
    );
    assert_eq!(
        result.evaluation().output_ring_bucket_cache(),
        result.output_ring_bucket_cache()
    );
    assert_eq!(
        result.evaluation().output_ring_segment_ref_count(),
        result.output_ring_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_ring_max_segment_count(),
        result.output_ring_max_segment_count()
    );
    assert_eq!(
        result.evaluation().output_segment_kind_bucket_cache(),
        result.output_segment_kind_bucket_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_kind_bucket_count(),
        result.output_segment_kind_bucket_count()
    );
    assert_eq!(
        result.evaluation().output_segment_kind_ref_count(),
        result.output_segment_kind_ref_count()
    );
    assert_eq!(
        result.evaluation().output_line_segment_ref_count(),
        result.output_line_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_arc_segment_ref_count(),
        result.output_arc_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_segment_kind_max_bucket_size(),
        result.output_segment_kind_max_bucket_size()
    );
    assert_eq!(
        result.evaluation().output_segment_source_bucket_cache(),
        result.output_segment_source_bucket_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_source_bucket_count(),
        result.output_segment_source_bucket_count()
    );
    assert_eq!(
        result.evaluation().output_segment_source_ref_count(),
        result.output_segment_source_ref_count()
    );
    assert_eq!(
        result.evaluation().output_segment_source_max_bucket_size(),
        result.output_segment_source_max_bucket_size()
    );
    assert_eq!(
        result.evaluation().output_segment_source_range_cache(),
        result.output_segment_source_range_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_endpoint_cache(),
        result.output_segment_endpoint_cache()
    );
    assert_eq!(
        result.evaluation().output_ring_continuity_cache(),
        result.output_ring_continuity_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_status_bucket_cache(),
        result.output_segment_status_bucket_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_status_bucket_count(),
        result.output_segment_status_bucket_count()
    );
    assert_eq!(
        result.evaluation().output_segment_status_ref_count(),
        result.output_segment_status_ref_count()
    );
    assert_eq!(
        result.evaluation().output_native_exact_segment_ref_count(),
        result.output_native_exact_segment_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .output_certified_approximation_segment_ref_count(),
        result.output_certified_approximation_segment_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .output_display_or_export_segment_ref_count(),
        result.output_display_or_export_segment_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .output_imported_lossy_segment_ref_count(),
        result.output_imported_lossy_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_unsupported_segment_ref_count(),
        result.output_unsupported_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_unresolved_segment_ref_count(),
        result.output_unresolved_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_segment_status_max_bucket_size(),
        result.output_segment_status_max_bucket_size()
    );
    assert_eq!(
        result.evaluation().output_segment_direction_bucket_cache(),
        result.output_segment_direction_bucket_cache()
    );
    assert_eq!(
        result.evaluation().output_segment_direction_bucket_count(),
        result.output_segment_direction_bucket_count()
    );
    assert_eq!(
        result.evaluation().output_segment_direction_ref_count(),
        result.output_segment_direction_ref_count()
    );
    assert_eq!(
        result.evaluation().output_forward_segment_ref_count(),
        result.output_forward_segment_ref_count()
    );
    assert_eq!(
        result.evaluation().output_reversed_segment_ref_count(),
        result.output_reversed_segment_ref_count()
    );
    assert_eq!(
        result
            .evaluation()
            .output_segment_direction_max_bucket_size(),
        result.output_segment_direction_max_bucket_size()
    );
    assert_eq!(result.evaluation().output_cache(), result.output_cache());
    assert_eq!(
        result.evaluation().boundary_build_report(),
        result.boundary_build_report()
    );
    assert_eq!(
        result.evaluation().boundary_build_stage(),
        result.boundary_build_stage()
    );
    assert_eq!(
        result.evaluation().boundary_build_predicate_path(),
        result.boundary_build_predicate_path()
    );
    assert_eq!(
        result.evaluation().boundary_build_status(),
        result.boundary_build_status()
    );
    assert_eq!(
        result.evaluation().boundary_build_blocker(),
        result.boundary_build_blocker()
    );
    assert_eq!(
        result.evaluation().boundary_build_source_contour_count(),
        result.boundary_build_source_contour_count()
    );
    assert_eq!(
        result.evaluation().boundary_build_source_segment_count(),
        result.boundary_build_source_segment_count()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_validation_candidate_pair_count(),
        result.boundary_build_validation_candidate_pair_count()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_validation_tested_pair_count(),
        result.boundary_build_validation_tested_pair_count()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_validation_intersection_event_count(),
        result.boundary_build_validation_intersection_event_count()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_nesting_classification_count(),
        result.boundary_build_nesting_classification_count()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_blocker_first_contour_index(),
        result.boundary_build_blocker_first_contour_index()
    );
    assert_eq!(
        result
            .evaluation()
            .boundary_build_blocker_second_contour_index(),
        result.boundary_build_blocker_second_contour_index()
    );
    assert_eq!(
        result.evaluation().boundary_output_cache(),
        result.boundary_output_cache()
    );
    assert_eq!(
        result.evaluation().boundary_output_role_bucket_cache(),
        result.boundary_output_role_bucket_cache()
    );
    assert_eq!(result.evaluation().role_cache(), result.role_cache());
    assert_eq!(
        result.evaluation().output_segment_kind_counts(),
        result.output_segment_kind_counts()
    );
    assert_eq!(
        result.evaluation().material_contour_count(),
        result.material_contour_count()
    );
    assert_eq!(
        result.evaluation().hole_contour_count(),
        result.hole_contour_count()
    );
    assert_eq!(
        result.evaluation().material_segment_count(),
        result.material_segment_count()
    );
    assert_eq!(
        result.evaluation().hole_segment_count(),
        result.hole_segment_count()
    );
    assert_eq!(
        result.evaluation().role_report_count(),
        result.role_report_count()
    );
    assert_eq!(
        result.evaluation().role_status_bucket_cache(),
        result.role_status_bucket_cache()
    );
    assert_eq!(
        result.evaluation().role_source_contour_bucket_cache(),
        result.role_source_contour_bucket_cache()
    );
    assert_eq!(
        result.evaluation().role_nesting_depth_bucket_cache(),
        result.role_nesting_depth_bucket_cache()
    );
    assert_eq!(
        result.evaluation().role_containment_bucket_cache(),
        result.role_containment_bucket_cache()
    );
    assert_eq!(result.evaluation().role_buckets(), result.role_buckets());
    assert_eq!(result.evaluation().role_reports(), result.role_reports());
    assert_eq!(result.evaluation().summary_cache(), result.summary_cache());
    assert_eq!(
        result.evaluation().arrangement_report(),
        result.arrangement_report()
    );
    let derived_region_build_result = result.derived_region_build_result();
    let derived_region_build_report = result.derived_region_build_report();
    assert_eq!(
        derived_region_build_result.report(),
        &derived_region_build_report
    );
    let owned_derived_report = derived_region_build_result.clone().into_report();
    assert_eq!(owned_derived_report, derived_region_build_report);
    let (owned_derived_region, owned_derived_parts_report) =
        derived_region_build_result.clone().into_parts();
    assert_eq!(owned_derived_region.as_ref(), result.region());
    assert_eq!(owned_derived_parts_report, derived_region_build_report);
    assert_eq!(
        derived_region_build_result.region_classification(),
        result.region_classification()
    );
    assert_eq!(
        derived_region_build_result
            .clone()
            .into_region_classification(),
        result.clone().into_region_classification()
    );
    assert_eq!(
        result.evaluation().derived_region_build_report(),
        derived_region_build_report
    );
    assert_eq!(
        result.arrangement_report().into_region_build_report(),
        derived_region_build_report
    );
    #[allow(deprecated)]
    {
        assert_eq!(result.region_build_result(), &derived_region_build_result);
        assert_eq!(result.report(), &derived_region_build_report);
    }
    let arrangement_report = result.arrangement_report();
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
    assert_eq!(
        arrangement_report.evaluated_output(),
        result.evaluated_output()
    );
    assert_eq!(
        arrangement_report.materialized_region(),
        result.materialized_region()
    );
    assert_eq!(arrangement_report.stage(), result.stage());
    assert_eq!(arrangement_report.status(), result.status());
    assert_eq!(arrangement_report.blocker(), result.blocker());
    assert_eq!(
        arrangement_report.output_ring_count(),
        result.output_ring_count()
    );
    assert_eq!(
        arrangement_report.output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        arrangement_report.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.output_contour_count(),
        result.output_contour_count()
    );
    assert_eq!(
        arrangement_report.output_segment_count(),
        result.output_segment_count()
    );
    assert_eq!(
        arrangement_report.region_build_report(),
        derived_region_build_report
    );
    assert_eq!(arrangement_report.workspace(), result.workspace());
    let (owned_report_region, owned_report) = result.clone().into_region_with_arrangement_report();
    assert_eq!(owned_report_region.as_ref(), result.region());
    assert_eq!(&owned_report, &arrangement_report);
    let (owned_parts_evaluation, owned_parts_region) = result.clone().into_parts();
    assert_eq!(&owned_parts_evaluation, result.evaluation());
    assert_eq!(owned_parts_region.as_ref(), result.region());
    let owned_report_request = owned_report.clone().into_request();
    assert_eq!(&owned_report_request, result.request());
    let (owned_report_classification, owned_classification_report) = result
        .clone()
        .into_region_classification_with_arrangement_report();
    assert_eq!(
        owned_report_classification,
        result.clone().into_region_classification()
    );
    assert_eq!(&owned_classification_report, &arrangement_report);
    let report_evaluation = arrangement_report.clone().into_evaluation();
    assert_eq!(&report_evaluation, result.evaluation());
    let (report_parts_workspace, report_parts_summary_cache) =
        arrangement_report.clone().into_parts();
    assert_eq!(&report_parts_workspace, result.workspace());
    assert_eq!(&report_parts_summary_cache, result.summary_cache());
    let owned_result_report = result.clone().into_arrangement_report();
    assert_eq!(&owned_result_report, &arrangement_report);
    let report_workspace = arrangement_report.into_workspace();
    assert_eq!(&report_workspace, result.workspace());
    let owned_attempt_request = attempt.clone().into_request();
    assert_eq!(&owned_attempt_request, result.request());
    let owned_evaluation = result.clone().into_evaluation();
    assert_eq!(&owned_evaluation, result.evaluation());
    let (owned_evaluation_workspace, owned_evaluation_summary_cache) =
        owned_evaluation.clone().into_parts();
    assert_eq!(&owned_evaluation_workspace, result.workspace());
    assert_eq!(&owned_evaluation_summary_cache, result.summary_cache());
    let owned_evaluation_report = owned_evaluation.clone().into_arrangement_report();
    assert_eq!(owned_evaluation_report, result.arrangement_report());
    let owned_evaluation_request = owned_evaluation.clone().into_request();
    assert_eq!(&owned_evaluation_request, result.request());
    let owned_result_request = result.clone().into_request();
    assert_eq!(&owned_result_request, result.request());
    let owned_result_workspace = result.clone().into_workspace();
    assert_eq!(&owned_result_workspace, result.workspace());
    let owned_workspace = owned_evaluation.clone().into_workspace();
    assert_eq!(&owned_workspace, result.workspace());
    let owned_workspace_request = owned_workspace.into_request();
    assert_eq!(&owned_workspace_request, result.request());
    let (owned_source_segments, owned_source_line_segments, owned_fill_rule) =
        owned_result_request.clone().into_parts();
    assert_eq!(owned_source_segments.as_slice(), result.source_segments());
    assert_eq!(owned_source_line_segments, None);
    assert_eq!(owned_fill_rule, result.fill_rule());
    assert_eq!(
        owned_result_request.into_source_segments().as_slice(),
        result.source_segments()
    );
    assert_eq!(
        result.workspace().source_segment_cache(),
        result.source_segment_cache()
    );
    assert_eq!(
        result.source_segment_cache().source_aabb_bucket_cache(),
        result.source_aabb_bucket_cache()
    );
    assert_eq!(
        result
            .source_segment_cache()
            .source_segment_kind_bucket_cache(),
        result.source_segment_kind_bucket_cache()
    );
    assert_eq!(
        result.workspace().source_endpoint_bucket_cache(),
        result.source_endpoint_bucket_cache()
    );
    assert_eq!(
        result.workspace().split_schedule_cache(),
        result.split_schedule_cache()
    );
    assert_eq!(result.workspace().split_cache(), result.split_cache());
    assert_eq!(
        result.workspace().endpoint_graph_cache(),
        result.endpoint_graph_cache()
    );
    assert_eq!(
        result.workspace().ring_assembly_cache(),
        result.ring_assembly_cache()
    );
    assert_eq!(
        result.workspace().arranged_source_reports(),
        result.arranged_source_reports()
    );
    assert_eq!(
        result.workspace().arranged_source_report_count(),
        result.arranged_source_report_count()
    );
    assert_eq!(result.workspace().source_reports(), result.source_reports());
    assert_eq!(
        result.workspace().source_report_count(),
        result.source_report_count()
    );
    assert_eq!(
        result.workspace().attempted_endpoint_connection_count(),
        result.attempted_endpoint_connection_count()
    );
    assert_eq!(
        result.workspace().exact_endpoint_connection_count(),
        result.exact_endpoint_connection_count()
    );
    assert_eq!(
        result.workspace().disconnected_endpoint_connection_count(),
        result.disconnected_endpoint_connection_count()
    );
    assert_eq!(
        result.workspace().unresolved_endpoint_connection_count(),
        result.unresolved_endpoint_connection_count()
    );
    assert_eq!(
        result.workspace().reversed_source_segment_count(),
        result.reversed_source_segment_count()
    );
    assert_eq!(
        result.workspace().arranged_segment_count(),
        result.arranged_segment_count()
    );
    assert_eq!(
        result.workspace().arranged_segment_kind_counts(),
        result.arranged_segment_kind_counts()
    );
    assert_eq!(
        result.workspace().output_ring_count(),
        result.output_ring_count()
    );
    assert_eq!(
        result.workspace().output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        result.workspace().output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(result.workspace().output_cache(), result.output_cache());
    assert_eq!(
        result.workspace().materialized_region(),
        result.materialized_region()
    );
    assert_eq!(result.workspace().stage(), result.stage());
    assert_eq!(result.workspace().status(), result.status());
    assert_eq!(result.workspace().blocker(), result.blocker());
    assert_eq!(
        result.workspace().boundary_build_report(),
        result.boundary_build_report()
    );
    assert_eq!(
        result.workspace().boundary_build_stage(),
        result.boundary_build_stage()
    );
    assert_eq!(
        result.workspace().boundary_build_predicate_path(),
        result.boundary_build_predicate_path()
    );
    assert_eq!(
        result.workspace().boundary_build_status(),
        result.boundary_build_status()
    );
    assert_eq!(
        result.workspace().boundary_build_blocker(),
        result.boundary_build_blocker()
    );
    assert_eq!(
        result.workspace().boundary_build_source_contour_count(),
        result.boundary_build_source_contour_count()
    );
    assert_eq!(
        result.workspace().boundary_build_source_segment_count(),
        result.boundary_build_source_segment_count()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_validation_candidate_pair_count(),
        result.boundary_build_validation_candidate_pair_count()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_validation_tested_pair_count(),
        result.boundary_build_validation_tested_pair_count()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_validation_intersection_event_count(),
        result.boundary_build_validation_intersection_event_count()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_nesting_classification_count(),
        result.boundary_build_nesting_classification_count()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_blocker_first_contour_index(),
        result.boundary_build_blocker_first_contour_index()
    );
    assert_eq!(
        result
            .workspace()
            .boundary_build_blocker_second_contour_index(),
        result.boundary_build_blocker_second_contour_index()
    );
    let arranged_fragment_cache = result.arranged_fragment_cache().unwrap();
    assert_eq!(
        result.workspace().arranged_fragment_cache().unwrap(),
        arranged_fragment_cache
    );
    assert_eq!(
        result.arranged_fragment_kind_bucket_cache().unwrap(),
        arranged_fragment_cache.arranged_fragment_kind_bucket_cache()
    );
    assert_eq!(
        result.arranged_fragment_status_bucket_cache().unwrap(),
        arranged_fragment_cache.arranged_fragment_status_bucket_cache()
    );
    assert_eq!(
        result.arranged_fragment_source_range_cache().unwrap(),
        arranged_fragment_cache.arranged_fragment_source_range_cache()
    );
    assert_eq!(
        result
            .workspace()
            .arranged_fragment_source_range_ref_count(),
        result.arranged_fragment_source_range_ref_count()
    );
    assert_eq!(
        result
            .workspace()
            .arranged_fragment_full_source_range_ref_count(),
        result.arranged_fragment_full_source_range_ref_count()
    );
    assert_eq!(
        result
            .workspace()
            .arranged_fragment_partial_source_range_ref_count(),
        result.arranged_fragment_partial_source_range_ref_count()
    );
    assert_eq!(
        result.workspace().output_ring_bucket_cache(),
        result.output_ring_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_ring_segment_ref_count(),
        result.output_ring_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_ring_max_segment_count(),
        result.output_ring_max_segment_count()
    );
    assert_eq!(
        result.workspace().output_segment_kind_bucket_cache(),
        result.output_segment_kind_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_segment_kind_bucket_count(),
        result.output_segment_kind_bucket_count()
    );
    assert_eq!(
        result.workspace().output_segment_kind_ref_count(),
        result.output_segment_kind_ref_count()
    );
    assert_eq!(
        result.workspace().output_line_segment_ref_count(),
        result.output_line_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_arc_segment_ref_count(),
        result.output_arc_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_segment_kind_max_bucket_size(),
        result.output_segment_kind_max_bucket_size()
    );
    assert_eq!(
        result.workspace().output_segment_source_bucket_cache(),
        result.output_segment_source_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_segment_source_bucket_count(),
        result.output_segment_source_bucket_count()
    );
    assert_eq!(
        result.workspace().output_segment_source_ref_count(),
        result.output_segment_source_ref_count()
    );
    assert_eq!(
        result.workspace().output_segment_source_max_bucket_size(),
        result.output_segment_source_max_bucket_size()
    );
    assert_eq!(
        result.workspace().output_segment_source_range_cache(),
        result.output_segment_source_range_cache()
    );
    assert_eq!(
        result.workspace().output_segment_endpoint_cache(),
        result.output_segment_endpoint_cache()
    );
    assert_eq!(
        result.workspace().output_ring_continuity_cache(),
        result.output_ring_continuity_cache()
    );
    assert_eq!(
        result.workspace().output_segment_status_bucket_cache(),
        result.output_segment_status_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_segment_status_bucket_count(),
        result.output_segment_status_bucket_count()
    );
    assert_eq!(
        result.workspace().output_segment_status_ref_count(),
        result.output_segment_status_ref_count()
    );
    assert_eq!(
        result.workspace().output_native_exact_segment_ref_count(),
        result.output_native_exact_segment_ref_count()
    );
    assert_eq!(
        result
            .workspace()
            .output_certified_approximation_segment_ref_count(),
        result.output_certified_approximation_segment_ref_count()
    );
    assert_eq!(
        result
            .workspace()
            .output_display_or_export_segment_ref_count(),
        result.output_display_or_export_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_imported_lossy_segment_ref_count(),
        result.output_imported_lossy_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_unsupported_segment_ref_count(),
        result.output_unsupported_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_unresolved_segment_ref_count(),
        result.output_unresolved_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_segment_status_max_bucket_size(),
        result.output_segment_status_max_bucket_size()
    );
    assert_eq!(
        result.workspace().output_segment_direction_bucket_cache(),
        result.output_segment_direction_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_segment_direction_bucket_count(),
        result.output_segment_direction_bucket_count()
    );
    assert_eq!(
        result.workspace().output_segment_direction_ref_count(),
        result.output_segment_direction_ref_count()
    );
    assert_eq!(
        result.workspace().output_forward_segment_ref_count(),
        result.output_forward_segment_ref_count()
    );
    assert_eq!(
        result.workspace().output_reversed_segment_ref_count(),
        result.output_reversed_segment_ref_count()
    );
    assert_eq!(
        result
            .workspace()
            .output_segment_direction_max_bucket_size(),
        result.output_segment_direction_max_bucket_size()
    );
    assert_eq!(
        result.workspace().boundary_output_cache(),
        result.boundary_output_cache()
    );
    assert_eq!(
        result.workspace().boundary_output_role_bucket_cache(),
        result.boundary_output_role_bucket_cache()
    );
    assert_eq!(
        result.workspace().output_contour_count(),
        result.output_contour_count()
    );
    assert_eq!(
        result.workspace().output_segment_count(),
        result.output_segment_count()
    );
    assert_eq!(
        result.workspace().output_segment_kind_counts(),
        result.output_segment_kind_counts()
    );
    assert_eq!(
        result.workspace().material_contour_count(),
        result.material_contour_count()
    );
    assert_eq!(
        result.workspace().hole_contour_count(),
        result.hole_contour_count()
    );
    assert_eq!(
        result.workspace().material_segment_count(),
        result.material_segment_count()
    );
    assert_eq!(
        result.workspace().hole_segment_count(),
        result.hole_segment_count()
    );
    assert_eq!(
        result.workspace().role_report_count(),
        result.role_report_count()
    );
    assert_eq!(result.workspace().role_cache(), result.role_cache());
    assert_eq!(
        result.workspace().role_status_bucket_cache(),
        result.role_status_bucket_cache()
    );
    assert_eq!(
        result.workspace().role_source_contour_bucket_cache(),
        result.role_source_contour_bucket_cache()
    );
    assert_eq!(
        result.workspace().role_nesting_depth_bucket_cache(),
        result.role_nesting_depth_bucket_cache()
    );
    assert_eq!(
        result.workspace().role_containment_bucket_cache(),
        result.role_containment_bucket_cache()
    );
    assert_eq!(
        result.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(result.source_segment_aabbs().len(), 2);
    assert_eq!(result.decided_source_segment_aabb_count(), 2);
    assert_eq!(result.undecided_source_segment_aabb_count(), 0);
    assert_eq!(
        result.source_aabb().map(|bbox| bbox.min().clone()),
        Some(p(0, -2))
    );
    let source_segment_cache = result.source_segment_cache();
    assert_eq!(
        source_segment_cache.source_segment_count(),
        result.source_segment_count()
    );
    assert_eq!(
        source_segment_cache.source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(source_segment_cache.decided_source_segment_aabb_count(), 2);
    assert_eq!(
        source_segment_cache.undecided_source_segment_aabb_count(),
        0
    );
    let source_aabb_bucket_cache = source_segment_cache.source_aabb_bucket_cache();
    assert_eq!(source_aabb_bucket_cache.bucket_count(), 2);
    assert_eq!(source_aabb_bucket_cache.source_ref_count(), 2);
    assert_eq!(source_aabb_bucket_cache.decided_source_ref_count(), 2);
    assert_eq!(source_aabb_bucket_cache.undecided_source_ref_count(), 0);
    assert_eq!(source_aabb_bucket_cache.max_bucket_size(), 2);
    assert_eq!(
        source_aabb_bucket_cache.buckets()[0].aabb_status(),
        ExactCurveArrangementSourceAabbStatus2::Decided
    );
    assert_eq!(source_aabb_bucket_cache.buckets()[0].source_refs().len(), 2);
    assert_eq!(
        source_aabb_bucket_cache.buckets()[1].aabb_status(),
        ExactCurveArrangementSourceAabbStatus2::Undecided
    );
    assert!(
        source_aabb_bucket_cache.buckets()[1]
            .source_refs()
            .is_empty()
    );
    let source_segment_kind_bucket_cache = source_segment_cache.source_segment_kind_bucket_cache();
    assert_eq!(source_segment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        source_segment_kind_bucket_cache.source_segment_ref_count(),
        source_segment_cache.source_segment_count()
    );
    assert_eq!(
        source_segment_kind_bucket_cache.line_segment_ref_count(),
        result.source_segment_kind_counts().lines
    );
    assert_eq!(
        source_segment_kind_bucket_cache.arc_segment_ref_count(),
        result.source_segment_kind_counts().arcs
    );
    assert_eq!(source_segment_kind_bucket_cache.max_bucket_size(), 1);
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[0].source_refs()[0].source_segment_index(),
        0
    );
    assert_eq!(
        source_segment_cache.segments()
            [source_segment_kind_bucket_cache.buckets()[0].source_refs()[0].source_segment_index()]
        .source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[1].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        source_segment_kind_bucket_cache.buckets()[1].source_refs()[0].source_segment_index(),
        1
    );
    assert_eq!(
        source_segment_cache.segments()
            [source_segment_kind_bucket_cache.buckets()[1].source_refs()[0].source_segment_index()]
        .source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(source_segment_cache.segments().len(), 2);
    let first_source_segment = &source_segment_cache.segments()[0];
    assert_eq!(first_source_segment.source_segment_index(), 0);
    assert_eq!(
        first_source_segment.source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(first_source_segment.source_start_point(), &p(4, 0));
    assert_eq!(first_source_segment.source_end_point(), &p(0, 0));
    let second_source_segment = &source_segment_cache.segments()[1];
    assert_eq!(second_source_segment.source_segment_index(), 1);
    assert_eq!(
        second_source_segment.source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(second_source_segment.source_start_point(), &p(0, 0));
    assert_eq!(second_source_segment.source_end_point(), &p(4, 0));
    let source_endpoint_cache = result.source_endpoint_bucket_cache();
    assert_eq!(source_endpoint_cache.endpoint_count(), 4);
    assert_eq!(source_endpoint_cache.bucket_count(), 2);
    assert_eq!(source_endpoint_cache.singleton_bucket_count(), 0);
    assert_eq!(source_endpoint_cache.max_bucket_size(), 2);
    let first_source_endpoint_bucket = &source_endpoint_cache.buckets()[0];
    assert_eq!(first_source_endpoint_bucket.point(), &p(4, 0));
    assert_eq!(first_source_endpoint_bucket.endpoints().len(), 2);
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[0].source_segment_index(),
        0
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[0].endpoint(),
        ExactCurveArrangementSourceEndpoint2::Start
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[1].source_segment_index(),
        1
    );
    assert_eq!(
        first_source_endpoint_bucket.endpoints()[1].endpoint(),
        ExactCurveArrangementSourceEndpoint2::End
    );
    let split_schedule_cache = result.split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.split_candidate_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.split_skipped_aabb_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.split_tested_pair_count().unwrap()
    );
    assert_eq!(split_schedule_cache.undecided_aabb_pair_count(), 0);
    assert_eq!(split_schedule_cache.candidate_pairs().len(), 1);
    let split_schedule_bucket_cache = split_schedule_cache.bucket_cache();
    assert_eq!(split_schedule_bucket_cache.bucket_count(), 3);
    assert_eq!(split_schedule_bucket_cache.candidate_ref_count(), 1);
    assert_eq!(split_schedule_bucket_cache.max_bucket_size(), 1);
    assert_eq!(split_schedule_bucket_cache.buckets().len(), 3);
    assert_eq!(
        split_schedule_bucket_cache.buckets()[0].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint
    );
    assert!(
        split_schedule_bucket_cache.buckets()[0]
            .candidate_refs()
            .is_empty()
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[1].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[1]
            .candidate_refs()
            .len(),
        1
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[1].candidate_refs()[0].candidate_pair_index(),
        0
    );
    assert_eq!(
        split_schedule_bucket_cache.buckets()[2].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::Undecided
    );
    assert!(
        split_schedule_bucket_cache.buckets()[2]
            .candidate_refs()
            .is_empty()
    );
    assert_eq!(
        (
            split_schedule_cache.candidate_pairs()[0].first_source_segment_index(),
            split_schedule_cache.candidate_pairs()[0].second_source_segment_index(),
            split_schedule_cache.candidate_pairs()[0].aabb_status(),
        ),
        (
            0,
            1,
            ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint,
        )
    );
    assert_eq!(
        result.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(result.split_candidate_pair_count(), Some(1));
    assert_eq!(result.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(result.split_tested_pair_count(), Some(1));
    assert_eq!(result.split_intersection_event_count(), Some(2));
    assert_eq!(result.split_intersection_reports().unwrap().len(), 2);
    assert_eq!(result.split_output_segment_count(), Some(2));
    assert_eq!(result.split_blocker_cache(), None);
    let split_relation_bucket_cache = result.split_relation_bucket_cache().unwrap();
    assert_eq!(split_relation_bucket_cache.bucket_count(), 3);
    assert_eq!(
        split_relation_bucket_cache.relation_count(),
        result.split_point_relation_count().unwrap()
            + result.split_overlap_relation_count().unwrap()
            + result.split_uncertain_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.point_relation_count(),
        result.split_point_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.overlap_relation_count(),
        result.split_overlap_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.uncertain_relation_count(),
        result.split_uncertain_relation_count().unwrap()
    );
    assert_eq!(split_relation_bucket_cache.buckets().len(), 3);
    assert_eq!(
        split_relation_bucket_cache.buckets()[0].relation(),
        ExactCurveArrangementSplitRelationClass2::Point
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[0].relation_count(),
        result.split_point_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[1].relation(),
        ExactCurveArrangementSplitRelationClass2::Overlap
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[1].relation_count(),
        result.split_overlap_relation_count().unwrap()
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[2].relation(),
        ExactCurveArrangementSplitRelationClass2::Uncertain
    );
    assert_eq!(
        split_relation_bucket_cache.buckets()[2].relation_count(),
        result.split_uncertain_relation_count().unwrap()
    );
    let split_intersection_bucket_cache = result.split_intersection_bucket_cache().unwrap();
    assert_eq!(
        split_intersection_bucket_cache.intersection_event_count(),
        result.split_intersection_event_count().unwrap()
    );
    assert_eq!(
        split_intersection_bucket_cache.bucket_count(),
        result.split_intersection_points().unwrap().len()
    );
    assert_eq!(split_intersection_bucket_cache.singleton_bucket_count(), 2);
    assert_eq!(split_intersection_bucket_cache.max_bucket_size(), 1);
    assert_eq!(split_intersection_bucket_cache.buckets().len(), 2);
    let first_split_intersection_bucket = &split_intersection_bucket_cache.buckets()[0];
    assert_eq!(first_split_intersection_bucket.intersections().len(), 1);
    assert_eq!(
        first_split_intersection_bucket.intersections()[0].intersection_report_index(),
        0
    );
    assert_eq!(
        first_split_intersection_bucket.point(),
        result.split_intersection_reports().unwrap()[0].point()
    );
    let split_intersection_parameter_cache = result.split_intersection_parameter_cache().unwrap();
    assert_eq!(
        split_intersection_parameter_cache.intersection_event_count(),
        result.split_intersection_reports().unwrap().len()
    );
    assert_eq!(
        split_intersection_parameter_cache.source_parameter_ref_count(),
        result.split_intersection_reports().unwrap().len() * 2
    );
    assert_eq!(
        split_intersection_parameter_cache.parameters().len(),
        result.split_intersection_reports().unwrap().len()
    );
    let first_split_parameter = &split_intersection_parameter_cache.parameters()[0];
    assert_eq!(first_split_parameter.intersection_report_index(), 0);
    assert_eq!(
        first_split_parameter.first_source_segment_index(),
        result.split_intersection_reports().unwrap()[0].first_source_segment_index()
    );
    assert_eq!(
        first_split_parameter.first_source_param(),
        result.split_intersection_reports().unwrap()[0].first_source_param()
    );
    assert_eq!(
        first_split_parameter.second_source_segment_index(),
        result.split_intersection_reports().unwrap()[0].second_source_segment_index()
    );
    assert_eq!(
        first_split_parameter.second_source_param(),
        result.split_intersection_reports().unwrap()[0].second_source_param()
    );
    assert_eq!(
        first_split_parameter.point(),
        result.split_intersection_reports().unwrap()[0].point()
    );
    assert_eq!(
        result.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(result.endpoint_graph_endpoint_count(), Some(4));
    assert_eq!(result.endpoint_graph_structural_bucket_count(), Some(2));
    assert_eq!(
        result.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(result.endpoint_graph_max_structural_bucket_size(), Some(2));
    let arranged_endpoint_bucket_cache = result.arranged_endpoint_bucket_cache().unwrap();
    assert_eq!(
        arranged_endpoint_bucket_cache.endpoint_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.bucket_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.singleton_bucket_count(),
        result
            .endpoint_graph_structural_singleton_bucket_count()
            .unwrap()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.max_bucket_size(),
        result.endpoint_graph_max_structural_bucket_size().unwrap()
    );
    assert_eq!(arranged_endpoint_bucket_cache.buckets().len(), 2);
    let first_arranged_endpoint_bucket = &arranged_endpoint_bucket_cache.buckets()[0];
    assert_eq!(first_arranged_endpoint_bucket.point(), &p(4, 0));
    assert_eq!(first_arranged_endpoint_bucket.endpoints().len(), 2);
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[0].arranged_segment_index(),
        0
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[1].arranged_segment_index(),
        1
    );
    assert_eq!(
        first_arranged_endpoint_bucket.endpoints()[1].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    let arranged_endpoint_side_bucket_cache = result.arranged_endpoint_side_bucket_cache().unwrap();
    assert_eq!(arranged_endpoint_side_bucket_cache.bucket_count(), 2);
    assert_eq!(
        arranged_endpoint_side_bucket_cache.endpoint_ref_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.start_endpoint_ref_count(),
        2
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.end_endpoint_ref_count(),
        2
    );
    assert_eq!(arranged_endpoint_side_bucket_cache.max_bucket_size(), 2);
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoints()[0].arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[0].endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::Start
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[1].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    assert_eq!(
        arranged_endpoint_side_bucket_cache.buckets()[1].endpoints()[0].endpoint(),
        RegionLineSegmentArrangedEndpoint2::End
    );
    let arranged_endpoint_point_cache = result.arranged_endpoint_point_cache().unwrap();
    assert_eq!(
        result.arranged_endpoint_point_fragment_ref_count(),
        Some(arranged_endpoint_point_cache.arranged_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_endpoint_point_ref_count(),
        Some(arranged_endpoint_point_cache.endpoint_ref_count())
    );
    assert_eq!(
        arranged_endpoint_point_cache.arranged_fragment_ref_count(),
        result.arranged_source_reports().unwrap().len()
    );
    assert_eq!(
        arranged_endpoint_point_cache.endpoint_ref_count(),
        result.endpoint_graph_endpoint_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_point_cache.endpoints().len(),
        result.arranged_source_reports().unwrap().len()
    );
    let arranged_endpoint_point_ref = &arranged_endpoint_point_cache.endpoints()[0];
    assert_eq!(
        arranged_endpoint_point_ref.arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert_eq!(
        arranged_endpoint_point_ref.output_start_point(),
        result.arranged_source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        arranged_endpoint_point_ref.output_end_point(),
        result.arranged_source_reports().unwrap()[0].output_end_point()
    );
    let arranged_endpoint_degree_bucket_cache =
        result.arranged_endpoint_degree_bucket_cache().unwrap();
    assert_eq!(arranged_endpoint_degree_bucket_cache.bucket_count(), 3);
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.structural_bucket_ref_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.dangling_structural_bucket_count(),
        0
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.chain_structural_bucket_count(),
        result.endpoint_graph_structural_bucket_count().unwrap()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.branch_structural_bucket_count(),
        0
    );
    assert_eq!(arranged_endpoint_degree_bucket_cache.max_bucket_size(), 2);
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[0].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Dangling
    );
    assert!(
        arranged_endpoint_degree_bucket_cache.buckets()[0]
            .endpoint_buckets()
            .is_empty()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[1].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Chain
    );
    let first_degree_ref =
        &arranged_endpoint_degree_bucket_cache.buckets()[1].endpoint_buckets()[0];
    assert_eq!(first_degree_ref.structural_bucket_index(), 0);
    assert_eq!(
        first_degree_ref.endpoint_ref_count(),
        arranged_endpoint_bucket_cache.buckets()[0]
            .endpoints()
            .len()
    );
    assert_eq!(
        first_degree_ref.point(),
        arranged_endpoint_bucket_cache.buckets()[0].point()
    );
    assert_eq!(
        arranged_endpoint_degree_bucket_cache.buckets()[2].degree(),
        ExactCurveArrangementArrangedEndpointDegree2::Branch
    );
    assert!(
        arranged_endpoint_degree_bucket_cache.buckets()[2]
            .endpoint_buckets()
            .is_empty()
    );
    assert_eq!(result.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(result.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(result.endpoint_graph_blocker_point(), None);
    assert_eq!(
        result.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(
        result.attempted_endpoint_connection_count(),
        Some(
            result.exact_endpoint_connection_count().unwrap()
                + result.disconnected_endpoint_connection_count().unwrap()
                + result.unresolved_endpoint_connection_count().unwrap()
        )
    );
    assert!(result.exact_endpoint_connection_count().unwrap() >= 2);
    assert_eq!(result.unresolved_endpoint_connection_count(), Some(0));
    assert_eq!(result.output_ring_count(), Some(1));
    assert_eq!(
        result.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(
        result.arranged_source_report_count(),
        Some(result.arranged_source_reports().unwrap().len())
    );
    let arranged_fragment_cache = result.arranged_fragment_cache().unwrap();
    assert_eq!(arranged_fragment_cache.arranged_fragment_count(), 2);
    assert_eq!(arranged_fragment_cache.source_ref_count(), 2);
    assert_eq!(
        arranged_fragment_cache.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(
        arranged_fragment_cache.arranged_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    let arranged_fragment_kind_bucket_cache =
        arranged_fragment_cache.arranged_fragment_kind_bucket_cache();
    assert_eq!(
        result.arranged_fragment_kind_bucket_count(),
        Some(arranged_fragment_kind_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_fragment_kind_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.arranged_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_line_fragment_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.line_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_arc_fragment_ref_count(),
        Some(arranged_fragment_kind_bucket_cache.arc_fragment_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_kind_max_bucket_size(),
        Some(arranged_fragment_kind_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_fragment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        arranged_fragment_kind_bucket_cache.arranged_fragment_ref_count(),
        2
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.line_fragment_ref_count(),
        1
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.arc_fragment_ref_count(),
        1
    );
    assert_eq!(arranged_fragment_kind_bucket_cache.max_bucket_size(), 1);
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_fragment_cache.fragments()[arranged_fragment_kind_bucket_cache.buckets()[0]
            .fragment_refs()[0]
            .arranged_fragment_index()]
        .arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_fragment_kind_bucket_cache.buckets()[1].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        arranged_fragment_cache.fragments()[arranged_fragment_kind_bucket_cache.buckets()[1]
            .fragment_refs()[0]
            .arranged_fragment_index()]
        .arranged_segment_kind(),
        SegmentKind::Arc
    );
    let arranged_fragment_status_bucket_cache =
        arranged_fragment_cache.arranged_fragment_status_bucket_cache();
    assert_eq!(
        result.arranged_fragment_status_bucket_count(),
        Some(arranged_fragment_status_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.arranged_fragment_status_source_ref_count(),
        Some(arranged_fragment_status_bucket_cache.source_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_native_exact_ref_count(),
        Some(arranged_fragment_status_bucket_cache.native_exact_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_certified_approximation_ref_count(),
        Some(arranged_fragment_status_bucket_cache.certified_approximation_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_display_or_export_ref_count(),
        Some(arranged_fragment_status_bucket_cache.display_or_export_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_imported_lossy_ref_count(),
        Some(arranged_fragment_status_bucket_cache.imported_lossy_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_unsupported_ref_count(),
        Some(arranged_fragment_status_bucket_cache.unsupported_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_unresolved_ref_count(),
        Some(arranged_fragment_status_bucket_cache.unresolved_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_status_max_bucket_size(),
        Some(arranged_fragment_status_bucket_cache.max_bucket_size())
    );
    assert_eq!(arranged_fragment_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        arranged_fragment_status_bucket_cache.source_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.native_exact_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.display_or_export_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.imported_lossy_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.unsupported_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.unresolved_ref_count(),
        0
    );
    assert_eq!(arranged_fragment_status_bucket_cache.max_bucket_size(), 2);
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0]
            .source_refs()
            .len(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .arranged_fragment_index(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0].source_ref_index(),
        0
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .arranged_source_report_index(),
        0
    );
    assert_eq!(
        arranged_fragment_cache.fragments()[arranged_fragment_status_bucket_cache.buckets()[0]
            .source_refs()[0]
            .arranged_fragment_index()]
        .source_refs()[arranged_fragment_status_bucket_cache.buckets()[0].source_refs()[0]
            .source_ref_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        arranged_fragment_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        arranged_fragment_status_bucket_cache.buckets()[1]
            .source_refs()
            .is_empty()
    );
    let arranged_fragment_source_range_cache =
        arranged_fragment_cache.arranged_fragment_source_range_cache();
    assert_eq!(
        result.arranged_fragment_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.source_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_full_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.full_source_range_ref_count())
    );
    assert_eq!(
        result.arranged_fragment_partial_source_range_ref_count(),
        Some(arranged_fragment_source_range_cache.partial_source_range_ref_count())
    );
    assert_eq!(
        arranged_fragment_source_range_cache.source_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_source_range_cache.full_source_range_ref_count(),
        arranged_fragment_cache.source_ref_count()
    );
    assert_eq!(
        arranged_fragment_source_range_cache.partial_source_range_ref_count(),
        0
    );
    assert_eq!(
        arranged_fragment_source_range_cache.ranges().len(),
        arranged_fragment_cache.source_ref_count()
    );
    let arranged_source_range_ref = &arranged_fragment_source_range_cache.ranges()[0];
    assert_eq!(arranged_source_range_ref.arranged_source_report_index(), 0);
    assert_eq!(
        arranged_source_range_ref.source_segment_index(),
        result.arranged_source_reports().unwrap()[0].source_segment_index()
    );
    assert_eq!(
        arranged_source_range_ref.source_range(),
        result.arranged_source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        arranged_source_range_ref.arranged_segment_index(),
        result.arranged_source_reports().unwrap()[0].arranged_segment_index()
    );
    assert!(arranged_source_range_ref.covers_full_source_range());
    assert_eq!(arranged_fragment_cache.max_source_ref_count(), 1);
    assert_eq!(arranged_fragment_cache.fragments().len(), 2);
    let arranged_fragment = &arranged_fragment_cache.fragments()[0];
    assert_eq!(arranged_fragment.arranged_segment_index(), 0);
    assert_eq!(arranged_fragment.arranged_segment_kind(), SegmentKind::Line);
    assert_eq!(
        arranged_fragment.output_start_point(),
        result.arranged_source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        arranged_fragment.output_end_point(),
        result.arranged_source_reports().unwrap()[0].output_end_point()
    );
    assert_eq!(arranged_fragment.source_refs().len(), 1);
    assert_eq!(
        arranged_fragment.source_refs()[0].arranged_source_report_index(),
        0
    );
    assert_eq!(arranged_fragment.source_refs()[0].source_segment_index(), 0);
    assert_eq!(
        arranged_fragment.source_refs()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        arranged_fragment.source_refs()[0].source_range(),
        result.arranged_source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        result.source_report_count(),
        Some(result.source_reports().unwrap().len())
    );
    let output_ring_bucket_cache = result.output_ring_bucket_cache().unwrap();
    assert_eq!(
        result.output_ring_segment_ref_count(),
        Some(output_ring_bucket_cache.segment_ref_count())
    );
    assert_eq!(
        result.output_ring_max_segment_count(),
        Some(output_ring_bucket_cache.max_ring_segment_count())
    );
    assert_eq!(output_ring_bucket_cache.ring_count(), 1);
    assert_eq!(output_ring_bucket_cache.segment_ref_count(), 2);
    assert_eq!(output_ring_bucket_cache.max_ring_segment_count(), 2);
    assert_eq!(output_ring_bucket_cache.rings().len(), 1);
    let output_ring_bucket = &output_ring_bucket_cache.rings()[0];
    assert_eq!(output_ring_bucket.output_ring_index(), 0);
    assert_eq!(output_ring_bucket.segments().len(), 2);
    assert_eq!(output_ring_bucket.segments()[0].source_report_index(), 0);
    assert_eq!(output_ring_bucket.segments()[0].output_segment_index(), 0);
    assert_eq!(
        output_ring_bucket.segments()[0].reversed(),
        result.source_reports().unwrap()[0].reversed()
    );
    let output_segment_kind_bucket_cache = result.output_segment_kind_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_kind_bucket_count(),
        Some(output_segment_kind_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_kind_ref_count(),
        Some(output_segment_kind_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_line_segment_ref_count(),
        Some(output_segment_kind_bucket_cache.line_segment_ref_count())
    );
    assert_eq!(
        result.output_arc_segment_ref_count(),
        Some(output_segment_kind_bucket_cache.arc_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_kind_max_bucket_size(),
        Some(output_segment_kind_bucket_cache.max_bucket_size())
    );
    assert_eq!(output_segment_kind_bucket_cache.bucket_count(), 2);
    assert_eq!(
        output_segment_kind_bucket_cache.output_segment_ref_count(),
        2
    );
    assert_eq!(output_segment_kind_bucket_cache.line_segment_ref_count(), 1);
    assert_eq!(output_segment_kind_bucket_cache.arc_segment_ref_count(), 1);
    assert_eq!(output_segment_kind_bucket_cache.max_bucket_size(), 1);
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        result.source_reports().unwrap()
            [output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].source_report_index()]
        .output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        result.source_reports().unwrap()
            [output_segment_kind_bucket_cache.buckets()[1].segment_refs()[0].source_report_index()]
        .output_segment_kind(),
        SegmentKind::Arc
    );
    let output_segment_source_bucket_cache = result.output_segment_source_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_source_bucket_count(),
        Some(output_segment_source_bucket_cache.source_segment_bucket_count())
    );
    assert_eq!(
        result.output_segment_source_ref_count(),
        Some(output_segment_source_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_source_max_bucket_size(),
        Some(output_segment_source_bucket_cache.max_bucket_size())
    );
    assert_eq!(
        output_segment_source_bucket_cache.source_segment_bucket_count(),
        2
    );
    assert_eq!(
        output_segment_source_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(output_segment_source_bucket_cache.max_bucket_size(), 1);
    assert_eq!(output_segment_source_bucket_cache.buckets().len(), 2);
    assert_eq!(
        output_segment_source_bucket_cache.buckets()[0].source_segment_index(),
        0
    );
    assert_eq!(
        output_segment_source_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        1
    );
    let source_ref = &output_segment_source_bucket_cache.buckets()[0].segment_refs()[0];
    assert_eq!(
        source_ref.output_ring_index(),
        result.source_reports().unwrap()[source_ref.source_report_index()].output_ring_index()
    );
    assert_eq!(
        source_ref.output_segment_index(),
        result.source_reports().unwrap()[source_ref.source_report_index()].output_segment_index()
    );
    assert_eq!(
        result.source_reports().unwrap()[source_ref.source_report_index()].source_segment_index(),
        output_segment_source_bucket_cache.buckets()[0].source_segment_index()
    );
    let output_segment_source_range_cache = result.output_segment_source_range_cache().unwrap();
    assert_eq!(
        output_segment_source_range_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_source_range_cache.full_source_range_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_source_range_cache.partial_source_range_ref_count(),
        0
    );
    assert_eq!(
        output_segment_source_range_cache.ranges().len(),
        result.source_report_count().unwrap()
    );
    let source_range_ref = &output_segment_source_range_cache.ranges()[0];
    assert_eq!(source_range_ref.source_report_index(), 0);
    assert_eq!(
        source_range_ref.source_segment_index(),
        result.source_reports().unwrap()[0].source_segment_index()
    );
    assert_eq!(
        source_range_ref.source_range(),
        result.source_reports().unwrap()[0].source_range()
    );
    assert_eq!(
        source_range_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        source_range_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert!(source_range_ref.covers_full_source_range());
    let output_segment_endpoint_cache = result.output_segment_endpoint_cache().unwrap();
    assert_eq!(
        output_segment_endpoint_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_endpoint_cache.output_endpoint_ref_count(),
        result.source_report_count().unwrap() * 2
    );
    assert_eq!(
        output_segment_endpoint_cache.segments().len(),
        result.source_report_count().unwrap()
    );
    let endpoint_ref = &output_segment_endpoint_cache.segments()[0];
    assert_eq!(endpoint_ref.source_report_index(), 0);
    assert_eq!(
        endpoint_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        endpoint_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        endpoint_ref.output_start_point(),
        result.source_reports().unwrap()[0].output_start_point()
    );
    assert_eq!(
        endpoint_ref.output_end_point(),
        result.source_reports().unwrap()[0].output_end_point()
    );
    let output_ring_continuity_cache = result.output_ring_continuity_cache().unwrap();
    assert_eq!(
        output_ring_continuity_cache.output_ring_ref_count(),
        result.output_ring_bucket_cache().unwrap().ring_count()
    );
    assert_eq!(
        output_ring_continuity_cache.output_connection_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_ring_continuity_cache.max_ring_connection_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_ring_continuity_cache.connections().len(),
        result.source_report_count().unwrap()
    );
    let continuity_ref = &output_ring_continuity_cache.connections()[0];
    assert_eq!(continuity_ref.source_report_index(), 0);
    assert_eq!(continuity_ref.next_source_report_index(), 1);
    assert_eq!(
        continuity_ref.output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        continuity_ref.output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        continuity_ref.next_output_segment_index(),
        result.source_reports().unwrap()[1].output_segment_index()
    );
    assert_eq!(
        continuity_ref.output_end_point(),
        result.source_reports().unwrap()[0].output_end_point()
    );
    assert_eq!(
        continuity_ref.next_output_start_point(),
        result.source_reports().unwrap()[1].output_start_point()
    );
    assert_eq!(
        continuity_ref.output_end_point(),
        continuity_ref.next_output_start_point()
    );
    let closing_continuity_ref = output_ring_continuity_cache.connections().last().unwrap();
    assert_eq!(closing_continuity_ref.next_source_report_index(), 0);
    assert_eq!(
        closing_continuity_ref.output_end_point(),
        closing_continuity_ref.next_output_start_point()
    );
    let output_segment_status_bucket_cache = result.output_segment_status_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_status_bucket_count(),
        Some(output_segment_status_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_status_ref_count(),
        Some(output_segment_status_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_native_exact_segment_ref_count(),
        Some(output_segment_status_bucket_cache.native_exact_ref_count())
    );
    assert_eq!(
        result.output_certified_approximation_segment_ref_count(),
        Some(output_segment_status_bucket_cache.certified_approximation_ref_count())
    );
    assert_eq!(
        result.output_display_or_export_segment_ref_count(),
        Some(output_segment_status_bucket_cache.display_or_export_ref_count())
    );
    assert_eq!(
        result.output_imported_lossy_segment_ref_count(),
        Some(output_segment_status_bucket_cache.imported_lossy_ref_count())
    );
    assert_eq!(
        result.output_unsupported_segment_ref_count(),
        Some(output_segment_status_bucket_cache.unsupported_ref_count())
    );
    assert_eq!(
        result.output_unresolved_segment_ref_count(),
        Some(output_segment_status_bucket_cache.unresolved_ref_count())
    );
    assert_eq!(
        result.output_segment_status_max_bucket_size(),
        Some(output_segment_status_bucket_cache.max_bucket_size())
    );
    assert_eq!(output_segment_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        output_segment_status_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.native_exact_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.display_or_export_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.imported_lossy_ref_count(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.unsupported_ref_count(),
        0
    );
    assert_eq!(output_segment_status_bucket_cache.unresolved_ref_count(), 0);
    assert_eq!(output_segment_status_bucket_cache.max_bucket_size(), 2);
    assert_eq!(output_segment_status_bucket_cache.buckets().len(), 6);
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].source_report_index(),
        0
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].output_ring_index(),
        result.source_reports().unwrap()[0].output_ring_index()
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[0].segment_refs()[0].output_segment_index(),
        result.source_reports().unwrap()[0].output_segment_index()
    );
    assert_eq!(
        result.source_reports().unwrap()[output_segment_status_bucket_cache.buckets()[0]
            .segment_refs()[0]
            .source_report_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        output_segment_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        output_segment_status_bucket_cache.buckets()[1]
            .segment_refs()
            .is_empty()
    );
    let output_segment_direction_bucket_cache =
        result.output_segment_direction_bucket_cache().unwrap();
    assert_eq!(
        result.output_segment_direction_bucket_count(),
        Some(output_segment_direction_bucket_cache.bucket_count())
    );
    assert_eq!(
        result.output_segment_direction_ref_count(),
        Some(output_segment_direction_bucket_cache.output_segment_ref_count())
    );
    assert_eq!(
        result.output_forward_segment_ref_count(),
        Some(output_segment_direction_bucket_cache.forward_segment_ref_count())
    );
    assert_eq!(
        result.output_reversed_segment_ref_count(),
        Some(output_segment_direction_bucket_cache.reversed_segment_ref_count())
    );
    assert_eq!(
        result.output_segment_direction_max_bucket_size(),
        Some(output_segment_direction_bucket_cache.max_bucket_size())
    );
    let reversed_source_segment_count = result.reversed_source_segment_count().unwrap();
    let forward_source_segment_count = result
        .source_report_count()
        .unwrap()
        .saturating_sub(reversed_source_segment_count);
    assert_eq!(output_segment_direction_bucket_cache.bucket_count(), 2);
    assert_eq!(
        output_segment_direction_bucket_cache.output_segment_ref_count(),
        result.source_report_count().unwrap()
    );
    assert_eq!(
        output_segment_direction_bucket_cache.forward_segment_ref_count(),
        forward_source_segment_count
    );
    assert_eq!(
        output_segment_direction_bucket_cache.reversed_segment_ref_count(),
        reversed_source_segment_count
    );
    assert_eq!(
        output_segment_direction_bucket_cache.max_bucket_size(),
        forward_source_segment_count.max(reversed_source_segment_count)
    );
    assert_eq!(output_segment_direction_bucket_cache.buckets().len(), 2);
    assert!(!output_segment_direction_bucket_cache.buckets()[0].reversed());
    assert_eq!(
        output_segment_direction_bucket_cache.buckets()[0]
            .segment_refs()
            .len(),
        forward_source_segment_count
    );
    if forward_source_segment_count > 0 {
        let forward_ref = &output_segment_direction_bucket_cache.buckets()[0].segment_refs()[0];
        assert_eq!(
            forward_ref.output_ring_index(),
            result.source_reports().unwrap()[forward_ref.source_report_index()].output_ring_index()
        );
        assert_eq!(
            forward_ref.output_segment_index(),
            result.source_reports().unwrap()[forward_ref.source_report_index()]
                .output_segment_index()
        );
        assert!(!result.source_reports().unwrap()[forward_ref.source_report_index()].reversed());
    }
    assert!(output_segment_direction_bucket_cache.buckets()[1].reversed());
    assert_eq!(
        output_segment_direction_bucket_cache.buckets()[1]
            .segment_refs()
            .len(),
        reversed_source_segment_count
    );
    if reversed_source_segment_count > 0 {
        let reversed_ref = &output_segment_direction_bucket_cache.buckets()[1].segment_refs()[0];
        assert_eq!(
            reversed_ref.output_ring_index(),
            result.source_reports().unwrap()[reversed_ref.source_report_index()]
                .output_ring_index()
        );
        assert_eq!(
            reversed_ref.output_segment_index(),
            result.source_reports().unwrap()[reversed_ref.source_report_index()]
                .output_segment_index()
        );
        assert!(result.source_reports().unwrap()[reversed_ref.source_report_index()].reversed());
    }
    assert!(result.evaluated_output());
    assert_eq!(result.materialized_region(), Some(true));
    assert_eq!(
        result.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RegionRoleAssignment)
    );
    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.blocker(), None);
    assert_eq!(result.output_ring_count(), Some(1));
    assert_eq!(result.output_boundary_segment_count(), Some(2));
    assert_eq!(
        result.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(result.output_contour_count(), Some(1));
    assert_eq!(result.output_segment_count(), Some(2));
    let summary_cache = result.summary_cache();
    assert!(summary_cache.evaluated_output());
    assert_eq!(summary_cache.materialized_region(), Some(true));
    assert_eq!(summary_cache.stage(), result.stage());
    assert_eq!(summary_cache.status(), result.status());
    assert_eq!(summary_cache.blocker(), None);
    assert_eq!(
        summary_cache.output_ring_count(),
        result.output_ring_count()
    );
    assert_eq!(
        summary_cache.output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        summary_cache.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(summary_cache.output_contour_count(), Some(1));
    assert_eq!(summary_cache.output_segment_count(), Some(2));
    let arrangement_report = result.arrangement_report();
    assert_eq!(arrangement_report.fill_rule(), result.fill_rule());
    assert_eq!(
        arrangement_report.evaluated_output(),
        result.evaluated_output()
    );
    assert_eq!(
        arrangement_report.source_segments(),
        result.source_segments()
    );
    assert_eq!(
        arrangement_report.source_line_segments(),
        result.source_line_segments()
    );
    assert_eq!(
        arrangement_report.source_segment_cache(),
        result.source_segment_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_cache(),
        result.source_endpoint_bucket_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_count(),
        result.source_endpoint_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_count(),
        result.source_endpoint_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_singleton_bucket_count(),
        result.source_endpoint_singleton_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_max_bucket_size(),
        result.source_endpoint_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.split_schedule_cache(),
        result.split_schedule_cache()
    );
    assert_eq!(
        arrangement_report.split_schedule_candidate_pair_count(),
        result.split_schedule_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_decided_disjoint_pair_count(),
        result.split_schedule_decided_disjoint_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_predicate_candidate_pair_count(),
        result.split_schedule_predicate_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_undecided_aabb_pair_count(),
        result.split_schedule_undecided_aabb_pair_count()
    );
    assert_eq!(arrangement_report.split_cache(), result.split_cache());
    assert_eq!(
        arrangement_report.endpoint_graph_cache(),
        result.endpoint_graph_cache()
    );
    assert_eq!(
        arrangement_report.ring_assembly_cache(),
        result.ring_assembly_cache()
    );
    assert_eq!(arrangement_report.output_cache(), result.output_cache());
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
    assert_eq!(
        arrangement_report.source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.split_intersection_event_count(),
        result.split_intersection_event_count()
    );
    assert_eq!(
        arrangement_report.split_point_relation_count(),
        result.split_point_relation_count()
    );
    assert_eq!(
        arrangement_report.split_predicate_path(),
        result.split_predicate_path()
    );
    assert_eq!(
        arrangement_report.split_output_segment_count(),
        result.split_output_segment_count()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_index(),
        result.split_blocker_first_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_kind(),
        result.split_blocker_first_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_start_point(),
        result.split_blocker_first_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_end_point(),
        result.split_blocker_first_source_end_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_index(),
        result.split_blocker_second_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_kind(),
        result.split_blocker_second_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_start_point(),
        result.split_blocker_second_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_end_point(),
        result.split_blocker_second_source_end_point()
    );
    assert_eq!(
        arrangement_report.output_ring_count(),
        result.output_ring_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_predicate_path(),
        result.endpoint_graph_predicate_path()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_structural_singleton_bucket_count(),
        result.endpoint_graph_structural_singleton_bucket_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_max_structural_bucket_size(),
        result.endpoint_graph_max_structural_bucket_size()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_dangling_endpoint_count(),
        result.endpoint_graph_dangling_endpoint_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_branch_endpoint_count(),
        result.endpoint_graph_branch_endpoint_count()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_arranged_segment_index(),
        result.endpoint_graph_blocker_arranged_segment_index()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_endpoint(),
        result.endpoint_graph_blocker_endpoint()
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_point(),
        result.endpoint_graph_blocker_point()
    );
    assert_eq!(
        arrangement_report.ring_assembly_predicate_path(),
        result.ring_assembly_predicate_path()
    );
    assert_eq!(
        arrangement_report.attempted_endpoint_connection_count(),
        result.attempted_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.exact_endpoint_connection_count(),
        result.exact_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.disconnected_endpoint_connection_count(),
        result.disconnected_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.unresolved_endpoint_connection_count(),
        result.unresolved_endpoint_connection_count()
    );
    assert_eq!(
        arrangement_report.reversed_source_segment_count(),
        result.reversed_source_segment_count()
    );
    assert_eq!(
        arrangement_report.arranged_segment_count(),
        result.arranged_segment_count()
    );
    assert_eq!(
        arrangement_report.arranged_segment_kind_counts(),
        result.arranged_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.arranged_source_report_count(),
        result.arranged_source_report_count()
    );
    assert_eq!(
        arrangement_report.arranged_source_reports(),
        result.arranged_source_reports()
    );
    assert_eq!(
        arrangement_report.source_report_count(),
        result.source_report_count()
    );
    assert_eq!(arrangement_report.source_reports(), result.source_reports());
    assert_eq!(
        arrangement_report.output_segment_count(),
        result.output_segment_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_counts(),
        result.output_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.material_contour_count(),
        result.material_contour_count()
    );
    assert_eq!(
        arrangement_report.hole_contour_count(),
        result.hole_contour_count()
    );
    assert_eq!(
        arrangement_report.material_segment_count(),
        result.material_segment_count()
    );
    assert_eq!(
        arrangement_report.hole_segment_count(),
        result.hole_segment_count()
    );
    assert_eq!(
        arrangement_report.role_report_count(),
        result.role_report_count()
    );
    assert_eq!(arrangement_report.role_reports(), result.role_reports());
    assert_eq!(
        arrangement_report.boundary_build_report(),
        result.boundary_build_report()
    );
    assert_eq!(
        arrangement_report.boundary_build_stage(),
        result.boundary_build_stage()
    );
    assert_eq!(
        arrangement_report.boundary_build_predicate_path(),
        result.boundary_build_predicate_path()
    );
    assert_eq!(
        arrangement_report.boundary_build_status(),
        result.boundary_build_status()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker(),
        result.boundary_build_blocker()
    );
    assert_eq!(
        arrangement_report.boundary_build_source_contour_count(),
        result.boundary_build_source_contour_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_source_segment_count(),
        result.boundary_build_source_segment_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_candidate_pair_count(),
        result.boundary_build_validation_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_tested_pair_count(),
        result.boundary_build_validation_tested_pair_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_intersection_event_count(),
        result.boundary_build_validation_intersection_event_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_nesting_classification_count(),
        result.boundary_build_nesting_classification_count()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_first_contour_index(),
        result.boundary_build_blocker_first_contour_index()
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_second_contour_index(),
        result.boundary_build_blocker_second_contour_index()
    );
    assert_eq!(
        result.boundary_build_stage(),
        Some(RegionBoundaryContourBuildStage2::RoleAssignment)
    );
    assert_eq!(
        result.boundary_build_status(),
        Some(RetainedTopologyStatus::NativeExact)
    );
    assert_eq!(result.boundary_build_blocker(), None);
    assert_eq!(
        result.boundary_build_validation_intersection_event_count(),
        Some(0)
    );
    assert_eq!(result.output_contour_count(), Some(1));
    assert_eq!(result.output_segment_count(), Some(2));
    assert_eq!(
        result.output_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(0));
    assert_eq!(result.material_segment_count(), Some(2));
    assert_eq!(result.hole_segment_count(), Some(0));
    let boundary_role_bucket_cache = result.boundary_output_role_bucket_cache().unwrap();
    assert_eq!(boundary_role_bucket_cache.bucket_count(), 2);
    assert_eq!(boundary_role_bucket_cache.output_contour_count(), 1);
    assert_eq!(boundary_role_bucket_cache.output_segment_count(), 2);
    assert_eq!(boundary_role_bucket_cache.max_segment_count(), 2);
    assert_eq!(boundary_role_bucket_cache.buckets().len(), 2);
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_contour_count(),
        1
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[0].output_segment_count(),
        2
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_contour_count(),
        0
    );
    assert_eq!(
        boundary_role_bucket_cache.buckets()[1].output_segment_count(),
        0
    );
    let role_reports = result.role_reports().unwrap();
    assert_eq!(result.role_report_count().unwrap(), role_reports.len());
    assert_eq!(result.material_contour_count(), Some(1));
    assert_eq!(result.hole_contour_count(), Some(0));
    assert_eq!(result.material_segment_count(), Some(2));
    assert_eq!(result.hole_segment_count(), Some(0));
    let role_status_bucket_cache = result.role_status_bucket_cache().unwrap();
    assert_eq!(role_status_bucket_cache.bucket_count(), 6);
    assert_eq!(
        role_status_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.native_exact_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.certified_approximation_ref_count(),
        0
    );
    assert_eq!(role_status_bucket_cache.display_or_export_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.imported_lossy_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.unsupported_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.unresolved_ref_count(), 0);
    assert_eq!(role_status_bucket_cache.max_bucket_size(), 1);
    assert_eq!(
        role_status_bucket_cache.buckets()[0].status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments().len(),
        result.role_report_count().unwrap()
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].assignment_index(),
        0
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[0].assignments()[0].role_report_index(),
        0
    );
    assert_eq!(
        result.role_buckets().unwrap()[0].assignments()
            [role_status_bucket_cache.buckets()[0].assignments()[0].assignment_index()]
        .status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(
        role_status_bucket_cache.buckets()[1].status(),
        RetainedTopologyStatus::CertifiedApproximation
    );
    assert!(
        role_status_bucket_cache.buckets()[1]
            .assignments()
            .is_empty()
    );
    let role_source_contour_bucket_cache = result.role_source_contour_bucket_cache().unwrap();
    assert_eq!(
        role_source_contour_bucket_cache.source_contour_bucket_count(),
        role_reports.len()
    );
    assert_eq!(
        role_source_contour_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(role_source_contour_bucket_cache.max_bucket_size(), 1);
    assert_eq!(role_source_contour_bucket_cache.buckets().len(), 1);
    assert_eq!(
        role_source_contour_bucket_cache.buckets()[0].source_contour_index(),
        role_reports[0].source_contour_index()
    );
    assert_eq!(
        role_source_contour_bucket_cache.buckets()[0]
            .assignments()
            .len(),
        1
    );
    let source_contour_assignment = &role_source_contour_bucket_cache.buckets()[0].assignments()[0];
    assert_eq!(
        source_contour_assignment.role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(source_contour_assignment.assignment_index(), 0);
    assert_eq!(source_contour_assignment.role_report_index(), 0);
    assert_eq!(source_contour_assignment.output_role_index(), 0);
    let role_nesting_depth_bucket_cache = result.role_nesting_depth_bucket_cache().unwrap();
    assert_eq!(
        role_nesting_depth_bucket_cache.nesting_depth_bucket_count(),
        1
    );
    assert_eq!(
        role_nesting_depth_bucket_cache.assignment_ref_count(),
        result.role_report_count().unwrap()
    );
    assert_eq!(role_nesting_depth_bucket_cache.max_bucket_size(), 1);
    assert_eq!(role_nesting_depth_bucket_cache.buckets().len(), 1);
    assert_eq!(
        role_nesting_depth_bucket_cache.buckets()[0].nesting_depth(),
        role_reports[0].nesting_depth()
    );
    assert_eq!(
        role_nesting_depth_bucket_cache.buckets()[0]
            .assignments()
            .len(),
        1
    );
    let nesting_depth_assignment = &role_nesting_depth_bucket_cache.buckets()[0].assignments()[0];
    assert_eq!(
        nesting_depth_assignment.role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(nesting_depth_assignment.assignment_index(), 0);
    assert_eq!(nesting_depth_assignment.role_report_index(), 0);
    assert_eq!(nesting_depth_assignment.source_contour_index(), 0);
    assert_eq!(nesting_depth_assignment.output_role_index(), 0);
    assert_eq!(result.role_buckets().unwrap().len(), 2);
    assert_eq!(
        result.role_buckets().unwrap()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(result.role_buckets().unwrap()[0].assignments().len(), 1);
    assert_eq!(
        result.role_buckets().unwrap()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert!(result.role_buckets().unwrap()[1].assignments().is_empty());
    let role_assignment = &result.role_buckets().unwrap()[0].assignments()[0];
    assert_eq!(role_assignment.role_report_index(), 0);
    assert_eq!(role_assignment.source_contour_index(), 0);
    assert_eq!(role_assignment.source_segment_count(), 2);
    assert_eq!(role_assignment.source_fill_rule(), FillRule::NonZero);
    assert_eq!(
        role_assignment.nesting_sample_point(),
        role_reports[0].nesting_sample_point()
    );
    assert_eq!(role_assignment.containing_contour_indices(), &[]);
    assert_eq!(role_assignment.nesting_depth(), 0);
    assert_eq!(role_assignment.output_role_index(), 0);
    assert_eq!(role_assignment.status(), role_reports[0].status());
    assert!(result.region().is_some());
    assert!(result.status().unwrap().is_native_exact());
}

#[test]
fn exact_curve_arrangement_attempt_retains_overlap_blocker() {
    let segments = vec![
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
    ];
    let result = ExactCurveArrangementAttempt2::new(
        ExactCurveArrangementRequest2::from_unordered_segments(segments.clone(), FillRule::NonZero),
    )
    .evaluate_owned(&policy())
    .unwrap();

    assert!(result.region().is_none());
    assert!(result.status().unwrap().is_retained_evidence());
    assert!(result.evaluated_output());
    assert_eq!(result.materialized_region(), Some(false));
    assert_eq!(
        result.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert!(result.status().unwrap().is_retained_evidence());
    assert_eq!(result.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(result.output_ring_count(), None);
    assert_eq!(result.output_boundary_segment_count(), None);
    assert_eq!(result.output_boundary_segment_kind_counts(), None);
    assert_eq!(result.output_contour_count(), None);
    assert_eq!(result.output_segment_count(), None);
    assert_eq!(
        result.stage().unwrap(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(result.blocker(), Some(UncertaintyReason::Boundary));
    let source_endpoint_cache = result.source_endpoint_bucket_cache();
    assert_eq!(source_endpoint_cache.endpoint_count(), 4);
    assert_eq!(source_endpoint_cache.bucket_count(), 2);
    assert_eq!(source_endpoint_cache.singleton_bucket_count(), 0);
    assert_eq!(source_endpoint_cache.max_bucket_size(), 2);
    let split_schedule_cache = result.split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.split_candidate_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.split_skipped_aabb_pair_count().unwrap()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.split_tested_pair_count().unwrap()
    );
    assert_eq!(split_schedule_cache.undecided_aabb_pair_count(), 0);
    assert_eq!(split_schedule_cache.candidate_pairs().len(), 1);
    assert_eq!(
        split_schedule_cache.candidate_pairs()[0].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint
    );
    assert_eq!(result.split_overlap_relation_count(), Some(1));
    assert_eq!(result.split_output_segment_count(), None);
    assert_eq!(
        result.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(result.split_intersection_event_count(), Some(0));
    assert!(result.split_intersection_points().unwrap().is_empty());
    let split_blocker_cache = result.split_blocker_cache().unwrap();
    assert_eq!(split_blocker_cache.first_source_segment_index(), 0);
    assert_eq!(
        split_blocker_cache.first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(split_blocker_cache.first_source_start_point(), &p(0, 0));
    assert_eq!(split_blocker_cache.first_source_end_point(), &p(4, 0));
    assert_eq!(split_blocker_cache.second_source_segment_index(), 1);
    assert_eq!(
        split_blocker_cache.second_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(split_blocker_cache.second_source_start_point(), &p(0, 0));
    assert_eq!(split_blocker_cache.second_source_end_point(), &p(4, 0));
    assert_eq!(split_blocker_cache.blocker(), result.blocker());
    let split_intersection_bucket_cache = result.split_intersection_bucket_cache().unwrap();
    assert_eq!(
        split_intersection_bucket_cache.intersection_event_count(),
        0
    );
    assert_eq!(split_intersection_bucket_cache.bucket_count(), 0);
    assert_eq!(split_intersection_bucket_cache.singleton_bucket_count(), 0);
    assert_eq!(split_intersection_bucket_cache.max_bucket_size(), 0);
    assert!(split_intersection_bucket_cache.buckets().is_empty());
    assert!(result.endpoint_graph_cache().is_none());
    assert!(result.ring_assembly_cache().is_none());
    assert_eq!(result.materialized_region(), Some(false));
    assert_eq!(
        result.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert!(result.status().unwrap().is_retained_evidence());
    assert_eq!(result.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(result.boundary_build_stage(), None);
    assert_eq!(result.boundary_output_cache(), None);
    assert_eq!(result.role_cache(), None);
    let summary_cache = result.summary_cache();
    assert!(summary_cache.evaluated_output());
    assert_eq!(summary_cache.materialized_region(), Some(false));
    assert_eq!(summary_cache.stage(), result.stage());
    assert_eq!(summary_cache.status(), result.status());
    assert_eq!(summary_cache.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(summary_cache.output_ring_count(), None);
    assert_eq!(summary_cache.output_boundary_segment_count(), None);
    assert_eq!(summary_cache.output_boundary_segment_kind_counts(), None);
    assert_eq!(summary_cache.output_contour_count(), None);
    assert_eq!(summary_cache.output_segment_count(), None);
    let evaluation = result.evaluation();
    assert_eq!(evaluation.summary_cache(), summary_cache);
    assert_eq!(evaluation.evaluated_output(), result.evaluated_output());
    assert_eq!(
        evaluation.materialized_region(),
        result.materialized_region()
    );
    assert_eq!(evaluation.stage(), result.stage());
    assert_eq!(evaluation.status(), result.status());
    assert_eq!(evaluation.blocker(), result.blocker());
    assert_eq!(evaluation.output_ring_count(), result.output_ring_count());
    assert_eq!(
        evaluation.output_boundary_segment_count(),
        result.output_boundary_segment_count()
    );
    assert_eq!(
        evaluation.output_boundary_segment_kind_counts(),
        result.output_boundary_segment_kind_counts()
    );
    assert_eq!(
        evaluation.output_contour_count(),
        result.output_contour_count()
    );
    assert_eq!(
        evaluation.output_segment_count(),
        result.output_segment_count()
    );

    let arrangement_report = result.arrangement_report();
    assert_eq!(arrangement_report.fill_rule(), result.fill_rule());
    assert_eq!(
        arrangement_report.evaluated_output(),
        result.evaluated_output()
    );
    assert_eq!(
        arrangement_report.source_aabb_bucket_cache(),
        result.source_aabb_bucket_cache()
    );
    assert_eq!(
        arrangement_report.source_segment_kind_bucket_cache(),
        result.source_segment_kind_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_bucket_cache(),
        result.arranged_fragment_kind_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_bucket_count(),
        result.arranged_fragment_kind_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_ref_count(),
        result.arranged_fragment_kind_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_line_fragment_ref_count(),
        result.arranged_line_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_arc_fragment_ref_count(),
        result.arranged_arc_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_kind_max_bucket_size(),
        result.arranged_fragment_kind_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_bucket_cache(),
        result.arranged_fragment_status_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_bucket_count(),
        result.arranged_fragment_status_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_source_ref_count(),
        result.arranged_fragment_status_source_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_native_exact_ref_count(),
        result.arranged_fragment_native_exact_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_certified_approximation_ref_count(),
        result.arranged_fragment_certified_approximation_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_display_or_export_ref_count(),
        result.arranged_fragment_display_or_export_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_imported_lossy_ref_count(),
        result.arranged_fragment_imported_lossy_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_unsupported_ref_count(),
        result.arranged_fragment_unsupported_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_unresolved_ref_count(),
        result.arranged_fragment_unresolved_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_status_max_bucket_size(),
        result.arranged_fragment_status_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_source_range_cache(),
        result.arranged_fragment_source_range_cache()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_source_range_ref_count(),
        result.arranged_fragment_source_range_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_full_source_range_ref_count(),
        result.arranged_fragment_full_source_range_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_partial_source_range_ref_count(),
        result.arranged_fragment_partial_source_range_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_fragment_cache(),
        result.arranged_fragment_cache()
    );
    assert_eq!(
        arrangement_report.split_relation_bucket_cache(),
        result.split_relation_bucket_cache()
    );
    assert_eq!(
        arrangement_report.split_intersection_bucket_cache(),
        result.split_intersection_bucket_cache()
    );
    assert_eq!(
        arrangement_report.split_intersection_parameter_cache(),
        result.split_intersection_parameter_cache()
    );
    assert_eq!(
        arrangement_report.split_blocker_cache(),
        result.split_blocker_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_bucket_cache(),
        result.arranged_endpoint_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_bucket_cache(),
        result.arranged_endpoint_side_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_bucket_count(),
        result.arranged_endpoint_side_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_ref_count(),
        result.arranged_endpoint_side_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_start_ref_count(),
        result.arranged_endpoint_start_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_end_ref_count(),
        result.arranged_endpoint_end_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_side_max_bucket_size(),
        result.arranged_endpoint_side_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_cache(),
        result.arranged_endpoint_point_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_fragment_ref_count(),
        result.arranged_endpoint_point_fragment_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_point_ref_count(),
        result.arranged_endpoint_point_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_bucket_cache(),
        result.arranged_endpoint_degree_bucket_cache()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_bucket_count(),
        result.arranged_endpoint_degree_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_structural_bucket_ref_count(),
        result.arranged_endpoint_degree_structural_bucket_ref_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_dangling_structural_bucket_count(),
        result.arranged_endpoint_dangling_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_chain_structural_bucket_count(),
        result.arranged_endpoint_chain_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_branch_structural_bucket_count(),
        result.arranged_endpoint_branch_structural_bucket_count()
    );
    assert_eq!(
        arrangement_report.arranged_endpoint_degree_max_bucket_size(),
        result.arranged_endpoint_degree_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.output_ring_bucket_cache(),
        result.output_ring_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_ring_segment_ref_count(),
        result.output_ring_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_ring_max_segment_count(),
        result.output_ring_max_segment_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_bucket_cache(),
        result.output_segment_kind_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_bucket_count(),
        result.output_segment_kind_bucket_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_ref_count(),
        result.output_segment_kind_ref_count()
    );
    assert_eq!(
        arrangement_report.output_line_segment_ref_count(),
        result.output_line_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_arc_segment_ref_count(),
        result.output_arc_segment_ref_count()
    );
    assert_eq!(
        arrangement_report.output_segment_kind_max_bucket_size(),
        result.output_segment_kind_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.output_segment_source_bucket_cache(),
        result.output_segment_source_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_source_range_cache(),
        result.output_segment_source_range_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_endpoint_cache(),
        result.output_segment_endpoint_cache()
    );
    assert_eq!(
        arrangement_report.output_ring_continuity_cache(),
        result.output_ring_continuity_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_status_bucket_cache(),
        result.output_segment_status_bucket_cache()
    );
    assert_eq!(
        arrangement_report.output_segment_direction_bucket_cache(),
        result.output_segment_direction_bucket_cache()
    );
    assert_eq!(
        arrangement_report.boundary_output_cache(),
        result.boundary_output_cache()
    );
    assert_eq!(
        arrangement_report.boundary_output_role_bucket_cache(),
        result.boundary_output_role_bucket_cache()
    );
    assert_eq!(arrangement_report.role_cache(), result.role_cache());
    assert_eq!(arrangement_report.role_buckets(), result.role_buckets());
    assert_eq!(
        arrangement_report.role_status_bucket_cache(),
        result.role_status_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_source_contour_bucket_cache(),
        result.role_source_contour_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_nesting_depth_bucket_cache(),
        result.role_nesting_depth_bucket_cache()
    );
    assert_eq!(
        arrangement_report.role_containment_bucket_cache(),
        result.role_containment_bucket_cache()
    );
    assert_eq!(
        arrangement_report.source_segment_count(),
        result.source_segment_count()
    );
    assert_eq!(
        arrangement_report.source_segment_kind_counts(),
        result.source_segment_kind_counts()
    );
    assert_eq!(
        arrangement_report.source_segment_aabbs(),
        result.source_segment_aabbs()
    );
    assert_eq!(arrangement_report.source_aabb(), result.source_aabb());
    assert_eq!(
        arrangement_report.decided_source_segment_aabb_count(),
        result.decided_source_segment_aabb_count()
    );
    assert_eq!(
        arrangement_report.undecided_source_segment_aabb_count(),
        result.undecided_source_segment_aabb_count()
    );
    assert_eq!(
        arrangement_report.source_segment_cache(),
        result.source_segment_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_cache(),
        result.source_endpoint_bucket_cache()
    );
    assert_eq!(
        arrangement_report.source_endpoint_count(),
        result.source_endpoint_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_bucket_count(),
        result.source_endpoint_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_singleton_bucket_count(),
        result.source_endpoint_singleton_bucket_count()
    );
    assert_eq!(
        arrangement_report.source_endpoint_max_bucket_size(),
        result.source_endpoint_max_bucket_size()
    );
    assert_eq!(
        arrangement_report.split_schedule_cache(),
        result.split_schedule_cache()
    );
    assert_eq!(
        arrangement_report.split_schedule_candidate_pair_count(),
        result.split_schedule_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_decided_disjoint_pair_count(),
        result.split_schedule_decided_disjoint_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_predicate_candidate_pair_count(),
        result.split_schedule_predicate_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_schedule_undecided_aabb_pair_count(),
        result.split_schedule_undecided_aabb_pair_count()
    );
    assert_eq!(arrangement_report.split_cache(), result.split_cache());
    assert_eq!(
        arrangement_report.endpoint_graph_cache(),
        result.endpoint_graph_cache()
    );
    assert_eq!(
        arrangement_report.ring_assembly_cache(),
        result.ring_assembly_cache()
    );
    assert_eq!(arrangement_report.output_cache(), result.output_cache());
    assert_eq!(arrangement_report.summary_cache(), result.summary_cache());
    assert_eq!(
        arrangement_report.split_candidate_pair_count(),
        result.split_candidate_pair_count()
    );
    assert_eq!(
        arrangement_report.split_skipped_aabb_pair_count(),
        result.split_skipped_aabb_pair_count()
    );
    assert_eq!(
        arrangement_report.split_tested_pair_count(),
        result.split_tested_pair_count()
    );
    assert_eq!(
        arrangement_report.split_intersection_event_count(),
        result.split_intersection_event_count()
    );
    assert_eq!(
        arrangement_report.split_point_relation_count(),
        result.split_point_relation_count()
    );
    assert_eq!(
        arrangement_report.split_overlap_relation_count(),
        result.split_overlap_relation_count()
    );
    assert_eq!(
        arrangement_report.split_uncertain_relation_count(),
        result.split_uncertain_relation_count()
    );
    assert_eq!(
        arrangement_report.split_intersection_points(),
        result.split_intersection_points()
    );
    assert_eq!(
        arrangement_report.split_intersection_reports(),
        result.split_intersection_reports()
    );
    assert_eq!(
        arrangement_report.split_predicate_path(),
        result.split_predicate_path()
    );
    assert_eq!(
        arrangement_report.split_output_segment_count(),
        result.split_output_segment_count()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_index(),
        result.split_blocker_first_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_segment_kind(),
        result.split_blocker_first_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_start_point(),
        result.split_blocker_first_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_first_source_end_point(),
        result.split_blocker_first_source_end_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_index(),
        result.split_blocker_second_source_segment_index()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_segment_kind(),
        result.split_blocker_second_source_segment_kind()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_start_point(),
        result.split_blocker_second_source_start_point()
    );
    assert_eq!(
        arrangement_report.split_blocker_second_source_end_point(),
        result.split_blocker_second_source_end_point()
    );
    assert_eq!(arrangement_report.endpoint_graph_predicate_path(), None);
    assert_eq!(arrangement_report.endpoint_graph_endpoint_count(), None);
    assert_eq!(
        arrangement_report.endpoint_graph_structural_bucket_count(),
        None
    );
    assert_eq!(
        arrangement_report.endpoint_graph_structural_singleton_bucket_count(),
        None
    );
    assert_eq!(
        arrangement_report.endpoint_graph_max_structural_bucket_size(),
        None
    );
    assert_eq!(
        arrangement_report.endpoint_graph_dangling_endpoint_count(),
        None
    );
    assert_eq!(
        arrangement_report.endpoint_graph_branch_endpoint_count(),
        None
    );
    assert_eq!(
        arrangement_report.endpoint_graph_blocker_arranged_segment_index(),
        None
    );
    assert_eq!(arrangement_report.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(arrangement_report.endpoint_graph_blocker_point(), None);
    assert_eq!(arrangement_report.ring_assembly_predicate_path(), None);
    assert_eq!(
        arrangement_report.attempted_endpoint_connection_count(),
        None
    );
    assert_eq!(arrangement_report.exact_endpoint_connection_count(), None);
    assert_eq!(
        arrangement_report.disconnected_endpoint_connection_count(),
        None
    );
    assert_eq!(
        arrangement_report.unresolved_endpoint_connection_count(),
        None
    );
    assert_eq!(arrangement_report.reversed_source_segment_count(), None);
    assert_eq!(arrangement_report.arranged_segment_count(), None);
    assert_eq!(arrangement_report.arranged_segment_kind_counts(), None);
    assert_eq!(
        arrangement_report.arranged_source_report_count(),
        result.arranged_source_report_count()
    );
    assert_eq!(
        arrangement_report.arranged_source_reports(),
        result.arranged_source_reports()
    );
    assert_eq!(
        arrangement_report.source_report_count(),
        result.source_report_count()
    );
    assert_eq!(arrangement_report.source_reports(), result.source_reports());
    assert_eq!(arrangement_report.output_ring_count(), None);
    assert_eq!(arrangement_report.output_boundary_segment_count(), None);
    assert_eq!(
        arrangement_report.output_boundary_segment_kind_counts(),
        None
    );
    assert_eq!(arrangement_report.output_contour_count(), None);
    assert_eq!(arrangement_report.output_segment_count(), None);
    assert_eq!(arrangement_report.output_segment_kind_counts(), None);
    assert_eq!(arrangement_report.material_contour_count(), None);
    assert_eq!(arrangement_report.hole_contour_count(), None);
    assert_eq!(arrangement_report.material_segment_count(), None);
    assert_eq!(arrangement_report.hole_segment_count(), None);
    assert_eq!(arrangement_report.role_report_count(), None);
    assert_eq!(arrangement_report.role_reports(), None);
    assert_eq!(arrangement_report.boundary_build_stage(), None);
    assert_eq!(arrangement_report.boundary_build_predicate_path(), None);
    assert_eq!(arrangement_report.boundary_build_status(), None);
    assert_eq!(arrangement_report.boundary_build_blocker(), None);
    assert_eq!(
        arrangement_report.boundary_build_source_contour_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_source_segment_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_candidate_pair_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_tested_pair_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_validation_intersection_event_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_nesting_classification_count(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_first_contour_index(),
        None
    );
    assert_eq!(
        arrangement_report.boundary_build_blocker_second_contour_index(),
        None
    );
    assert_eq!(arrangement_report.materialized_region(), Some(false));
    assert_eq!(
        arrangement_report.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(
        arrangement_report.status(),
        Some(RetainedTopologyStatus::Unsupported)
    );
    assert_eq!(
        arrangement_report.blocker(),
        Some(UncertaintyReason::Boundary)
    );
}

#[test]
fn unordered_native_segments_attempt_returns_decided_region() {
    let result =
        ExactCurveArrangementAttempt2::new(ExactCurveArrangementRequest2::from_unordered_segments(
            vec![
                Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
                Segment2::Line(line(4, 0, 0, 0)),
            ],
            FillRule::NonZero,
        ))
        .evaluate_owned(&policy())
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
fn unordered_native_segments_report_arc_overlap_boundary_blocker() {
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
    assert!(built.status().unwrap().is_retained_evidence());
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 2 }
    );
    assert_eq!(built.arranged_segment_count(), None);
    assert_eq!(built.arranged_segment_kind_counts(), None);
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(0));
    assert_eq!(built.split_point_relation_count(), Some(0));
    assert_eq!(built.split_overlap_relation_count(), Some(1));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    assert!(built.split_intersection_points().unwrap().is_empty());
    assert_eq!(built.split_output_segment_count(), None);
    assert_eq!(built.split_blocker_first_source_segment_index(), Some(0));
    assert_eq!(
        built.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        built.split_blocker_first_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(built.split_blocker_first_source_end_point(), Some(&p(4, 0)));
    assert_eq!(built.split_blocker_second_source_segment_index(), Some(1));
    assert_eq!(
        built.split_blocker_second_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        built.split_blocker_second_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        built.split_blocker_second_source_end_point(),
        Some(&p(4, 0))
    );
    assert_eq!(built.endpoint_graph_endpoint_count(), None);
    assert_eq!(built.endpoint_graph_structural_bucket_count(), None);
    assert_eq!(built.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(built.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(built.arranged_source_report_count(), None);
    assert_eq!(built.output_boundary_segment_kind_counts(), None);
    assert_eq!(built.source_report_count(), None);

    let split_cache = built.split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(split_cache.candidate_pair_count(), 1);
    assert_eq!(split_cache.skipped_aabb_pair_count(), 0);
    assert_eq!(split_cache.tested_pair_count(), 1);
    assert_eq!(split_cache.intersection_event_count(), 0);
    assert!(split_cache.intersection_points().is_empty());
    assert!(split_cache.intersection_reports().is_empty());
    assert_eq!(split_cache.output_segment_count(), None);
    assert_eq!(split_cache.relation_bucket_cache().relation_count(), 1);
    assert_eq!(
        split_cache.relation_bucket_cache().overlap_relation_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_bucket_cache()
            .intersection_event_count(),
        0
    );
    assert_eq!(
        split_cache
            .intersection_parameter_cache()
            .source_parameter_ref_count(),
        0
    );
    let split_blocker_cache = split_cache.blocker_cache().unwrap();
    assert_eq!(split_blocker_cache.first_source_segment_index(), 0);
    assert_eq!(
        split_blocker_cache.first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(split_blocker_cache.first_source_start_point(), &p(0, 0));
    assert_eq!(split_blocker_cache.first_source_end_point(), &p(4, 0));
    assert_eq!(split_blocker_cache.second_source_segment_index(), 1);
    assert_eq!(
        split_blocker_cache.second_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(split_blocker_cache.second_source_start_point(), &p(0, 0));
    assert_eq!(split_blocker_cache.second_source_end_point(), &p(4, 0));
    assert_eq!(
        split_blocker_cache.blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert!(built.endpoint_graph_cache().is_none());
    assert!(built.ring_assembly_cache().is_none());
    assert!(built.output_cache().is_some());
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
    assert!(built.status().unwrap().is_retained_evidence());
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(
        built.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 2 })
    );
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(1));
    assert_eq!(built.split_point_relation_count(), Some(1));
    assert_eq!(built.split_overlap_relation_count(), Some(0));
    assert_eq!(built.split_uncertain_relation_count(), Some(0));
    let split_points = built.split_intersection_points().unwrap();
    let split_reports = built.split_intersection_reports().unwrap();
    assert_eq!(split_points, &[p(2, -2)]);
    assert_eq!(split_reports.len(), 1);
    assert_eq!(split_reports[0].first_source_segment_index(), 0);
    assert_eq!(
        split_reports[0].first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        split_reports[0].first_source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(split_reports[0].first_source_segment_end_point(), &p(4, 0));
    assert_eq!(split_reports[0].second_source_segment_index(), 1);
    assert_eq!(
        split_reports[0].second_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        split_reports[0].second_source_segment_start_point(),
        &p(2, -3)
    );
    assert_eq!(split_reports[0].second_source_segment_end_point(), &p(2, 1));
    assert_eq!(split_reports[0].point(), &p(2, -2));
    assert_eq!(built.split_output_segment_count(), Some(4));
    assert_eq!(built.split_blocker_first_source_segment_kind(), None);
    assert_eq!(built.split_blocker_second_source_segment_kind(), None);
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        built.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        built.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(built.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    let arranged_sources = built.arranged_source_reports().unwrap();
    assert_eq!(built.arranged_source_report_count(), Some(4));
    assert_eq!(arranged_sources.len(), 4);
    assert_eq!(arranged_sources[0].source_segment_index(), 0);
    assert_eq!(arranged_sources[0].source_segment_kind(), SegmentKind::Arc);
    assert_eq!(arranged_sources[0].source_segment_start_point(), &p(0, 0));
    assert_eq!(arranged_sources[0].source_segment_end_point(), &p(4, 0));
    assert_eq!(
        arranged_sources[0].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        arranged_sources[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 2))
    );
    assert_eq!(arranged_sources[2].source_segment_index(), 1);
    assert_eq!(arranged_sources[2].source_segment_kind(), SegmentKind::Line);
    assert_eq!(arranged_sources[2].source_segment_start_point(), &p(2, -3));
    assert_eq!(arranged_sources[2].source_segment_end_point(), &p(2, 1));
    assert_eq!(built.source_report_count(), Some(0));

    let split_cache = built.split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(split_cache.candidate_pair_count(), 1);
    assert_eq!(split_cache.skipped_aabb_pair_count(), 0);
    assert_eq!(split_cache.tested_pair_count(), 1);
    assert_eq!(split_cache.intersection_event_count(), 1);
    assert_eq!(split_cache.point_relation_count(), 1);
    assert_eq!(split_cache.overlap_relation_count(), 0);
    assert_eq!(split_cache.uncertain_relation_count(), 0);
    assert_eq!(split_cache.intersection_points(), &[p(2, -2)]);
    assert_eq!(split_cache.intersection_reports(), split_reports);
    assert_eq!(split_cache.output_segment_count(), Some(4));
    assert!(split_cache.blocker_cache().is_none());
    assert_eq!(
        split_cache.relation_bucket_cache().point_relation_count(),
        1
    );
    assert_eq!(
        split_cache.relation_bucket_cache().overlap_relation_count(),
        0
    );
    assert_eq!(
        split_cache
            .intersection_bucket_cache()
            .intersection_event_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_parameter_cache()
            .source_parameter_ref_count(),
        2
    );

    let endpoint_graph_cache = built.endpoint_graph_cache().unwrap();
    assert_eq!(
        endpoint_graph_cache.predicate_path(),
        RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets
    );
    assert_eq!(endpoint_graph_cache.endpoint_count(), 8);
    assert_eq!(endpoint_graph_cache.structural_bucket_count(), 5);
    assert_eq!(endpoint_graph_cache.structural_singleton_bucket_count(), 4);
    assert_eq!(endpoint_graph_cache.max_structural_bucket_size(), 4);
    assert_eq!(endpoint_graph_cache.dangling_endpoint_count(), 4);
    assert_eq!(endpoint_graph_cache.branch_endpoint_count(), 4);
    assert_eq!(
        endpoint_graph_cache.blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        endpoint_graph_cache.blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(endpoint_graph_cache.blocker_point(), Some(&p(0, 0)));
    assert_eq!(
        endpoint_graph_cache.endpoint_bucket_cache().bucket_count(),
        5
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .start_endpoint_ref_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .end_endpoint_ref_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .dangling_structural_bucket_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .chain_structural_bucket_count(),
        0
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .branch_structural_bucket_count(),
        1
    );

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(
        ring_cache.predicate_path(),
        RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal
    );
    assert_eq!(ring_cache.output_ring_count(), None);
    assert_eq!(ring_cache.output_boundary_segment_count(), None);
    assert_eq!(ring_cache.arranged_source_reports(), arranged_sources);
    assert!(ring_cache.source_reports().is_empty());
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_count(),
        4
    );
    assert_eq!(ring_cache.arranged_fragment_cache().source_ref_count(), 4);
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_status_bucket_cache()
            .native_exact_ref_count(),
        4
    );
    assert!(built.output_cache().is_some());
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
    assert!(built.status().unwrap().is_retained_evidence());
    assert_eq!(
        built.stage(),
        Some(RegionLineSegmentRegionBuildStage2::RingAssembly)
    );
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(built.arranged_segment_count(), Some(4));
    assert_eq!(built.split_candidate_pair_count(), Some(1));
    assert_eq!(built.split_skipped_aabb_pair_count(), Some(0));
    assert_eq!(built.split_tested_pair_count(), Some(1));
    assert_eq!(built.split_intersection_event_count(), Some(1));
    assert_eq!(built.split_intersection_points().unwrap(), &[p(4, 3)]);
    let split_reports = built.split_intersection_reports().unwrap();
    assert_eq!(split_reports.len(), 1);
    assert_eq!(split_reports[0].first_source_segment_index(), 0);
    assert_eq!(
        split_reports[0].first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        split_reports[0].first_source_segment_start_point(),
        &p(5, 0)
    );
    assert_eq!(split_reports[0].first_source_segment_end_point(), &p(-5, 0));
    assert_eq!(split_reports[0].second_source_segment_index(), 1);
    assert_eq!(
        split_reports[0].second_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        split_reports[0].second_source_segment_start_point(),
        &p(3, 0)
    );
    assert_eq!(
        split_reports[0].second_source_segment_end_point(),
        &p(13, 0)
    );
    assert_eq!(split_reports[0].point(), &p(4, 3));
    assert_eq!(built.split_output_segment_count(), Some(4));
    assert_eq!(built.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(built.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        built.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(built.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(built.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(built.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        built.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        built.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(built.endpoint_graph_blocker_point(), Some(&p(5, 0)));
    let arranged_sources = built.arranged_source_reports().unwrap();
    assert_eq!(built.arranged_source_report_count(), Some(4));
    assert_eq!(arranged_sources.len(), 4);
    assert_eq!(arranged_sources[0].source_segment_index(), 0);
    assert_eq!(arranged_sources[0].source_segment_kind(), SegmentKind::Arc);
    assert_eq!(
        arranged_sources[0].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        arranged_sources[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 10))
    );
    assert_eq!(built.source_report_count(), Some(0));

    let split_cache = built.split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(split_cache.candidate_pair_count(), 1);
    assert_eq!(split_cache.skipped_aabb_pair_count(), 0);
    assert_eq!(split_cache.tested_pair_count(), 1);
    assert_eq!(split_cache.intersection_event_count(), 1);
    assert_eq!(split_cache.point_relation_count(), 1);
    assert_eq!(split_cache.overlap_relation_count(), 0);
    assert_eq!(split_cache.uncertain_relation_count(), 0);
    assert_eq!(split_cache.intersection_points(), &[p(4, 3)]);
    assert_eq!(split_cache.intersection_reports(), split_reports);
    assert_eq!(split_cache.output_segment_count(), Some(4));
    assert!(split_cache.blocker_cache().is_none());
    assert_eq!(
        split_cache.relation_bucket_cache().point_relation_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_bucket_cache()
            .intersection_event_count(),
        1
    );
    assert_eq!(
        split_cache
            .intersection_parameter_cache()
            .source_parameter_ref_count(),
        2
    );

    let endpoint_graph_cache = built.endpoint_graph_cache().unwrap();
    assert_eq!(
        endpoint_graph_cache.predicate_path(),
        RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets
    );
    assert_eq!(endpoint_graph_cache.endpoint_count(), 8);
    assert_eq!(endpoint_graph_cache.structural_bucket_count(), 5);
    assert_eq!(endpoint_graph_cache.structural_singleton_bucket_count(), 4);
    assert_eq!(endpoint_graph_cache.max_structural_bucket_size(), 4);
    assert_eq!(endpoint_graph_cache.dangling_endpoint_count(), 4);
    assert_eq!(endpoint_graph_cache.branch_endpoint_count(), 4);
    assert_eq!(
        endpoint_graph_cache.blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        endpoint_graph_cache.blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(endpoint_graph_cache.blocker_point(), Some(&p(5, 0)));
    assert_eq!(
        endpoint_graph_cache.endpoint_bucket_cache().bucket_count(),
        5
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .start_endpoint_ref_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_side_bucket_cache()
            .end_endpoint_ref_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .dangling_structural_bucket_count(),
        4
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .chain_structural_bucket_count(),
        0
    );
    assert_eq!(
        endpoint_graph_cache
            .endpoint_degree_bucket_cache()
            .branch_structural_bucket_count(),
        1
    );

    let ring_cache = built.ring_assembly_cache().unwrap();
    assert_eq!(
        ring_cache.predicate_path(),
        RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal
    );
    assert_eq!(ring_cache.output_ring_count(), None);
    assert_eq!(ring_cache.output_boundary_segment_count(), None);
    assert_eq!(ring_cache.arranged_source_reports(), arranged_sources);
    assert!(ring_cache.source_reports().is_empty());
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_count(),
        4
    );
    assert_eq!(ring_cache.arranged_fragment_cache().source_ref_count(), 4);
    assert_eq!(
        ring_cache
            .arranged_fragment_cache()
            .arranged_fragment_status_bucket_cache()
            .native_exact_ref_count(),
        4
    );
    assert!(built.output_cache().is_some());
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn hole_boundary_is_explicit() {
    let region = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);

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
    let region = Region2::new(
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
    let region = Region2::new(
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
    let region = Region2::new(Vec::new(), vec![rectangle(2, 2, 4, 4)]);

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
fn finite_line_string_import_promotes_boundary_f64_to_native_lines() {
    let curve =
        CurveString2::from_finite_line_string(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]]).unwrap();
    let iter_curve =
        CurveString2::from_finite_point_iter([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]]).unwrap();
    let import =
        CurveString2::import_finite_line_string_with_report(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
            .unwrap();

    assert_eq!(iter_curve, curve);
    assert_eq!(curve.len(), 2);
    assert_eq!(import.curve_string(), &curve);
    assert!(
        curve
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
    assert_eq!(
        CurveString2::from_finite_line_string(&[[0.0, 0.0], [f64::NAN, 1.0]]),
        Err(CurveError::NonFiniteReconstructionPoint)
    );
}

#[test]
fn finite_line_string_import_skips_duplicate_edges() {
    let import =
        CurveString2::import_finite_line_string_with_report(&[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]])
            .unwrap();

    assert_eq!(import.curve_string().len(), 1);
}

#[test]
fn finite_ring_import_accepts_repeated_closing_point_without_sample_ownership() {
    let contour =
        Contour2::from_finite_ring(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0], [0.0, 0.0]])
            .unwrap();
    let import = Contour2::import_finite_ring_with_report(&[
        [0.0, 0.0],
        [4.0, 0.0],
        [4.0, 3.0],
        [0.0, 3.0],
        [0.0, 0.0],
    ])
    .unwrap();

    assert_eq!(contour.len(), 4);
    assert_eq!(import.contour(), &contour);
    assert_eq!(
        contour.classify_point(&p(2, 1), &policy()),
        Classification::Decided(hypercurve::ContourPointLocation::Inside)
    );
}

#[test]
fn region_projection_preserves_material_hole_bins() {
    let outer = rectangle(0, 0, 10, 10);
    let island = rectangle(4, 4, 6, 6);
    let hole = rectangle(2, 2, 8, 8);
    let region = Region2::new(vec![outer.clone(), island.clone()], vec![hole.clone()]);
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
    let region = Region2::new(vec![left, right], vec![left_hole, right_hole]);
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
    let region = Region2::new(Vec::new(), vec![rectangle(2, 2, 4, 4)]);
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
    let region = Region2::new(vec![outer.clone()], vec![hole.clone()]);

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
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), reversed_rectangle(4, 4, 6, 6)],
        vec![reversed_rectangle(2, 2, 8, 8)],
    );

    assert_eq!(
        region.filled_area(&policy()).unwrap(),
        Classification::Decided(Some(Real::from(68_i8)))
    );
}

#[test]
fn region_filled_area_returns_none_for_unsupported_center_only_arc_area() {
    let top = CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let bottom = CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap();
    let contour = Contour2::try_new(vec![Segment2::Arc(top), Segment2::Arc(bottom)]).unwrap();
    let region = Region2::from_material_contours(vec![contour]);

    assert_eq!(
        region.filled_area(&policy()).unwrap(),
        Classification::Decided(None)
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
    let owned = Region2::new(vec![outer, island], vec![hole]);

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
        let arranged_sources = built.arranged_source_reports().unwrap();
        prop_assert_eq!(arranged_sources.len(), 2);
        prop_assert!(arranged_sources
            .iter()
            .all(|source| source.status().is_native_exact()));
        let source_reports = built.source_reports().unwrap();
        prop_assert_eq!(source_reports.len(), 2);
        prop_assert!(source_reports
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
        let region = Region2::new(
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
fn prepared_region_classifier_matches_owned_region() {
    let region = Region2::new(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    let policy = policy();
    let prepared = region.prepare_point_classifier(&policy);

    assert!(prepared.region_box().is_some());
    assert_eq!(prepared.material_contours().len(), 2);
    assert_eq!(prepared.hole_contours().len(), 1);
    assert_eq!(prepared.prepared_contour_count(), 3);
    assert_eq!(prepared.prepared_material_segment_count(), 8);
    assert_eq!(
        prepared.prepared_material_segment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(prepared.prepared_hole_segment_count(), 4);
    assert_eq!(
        prepared.prepared_hole_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(prepared.prepared_segment_count(), 12);
    assert_eq!(
        prepared.prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 12, arcs: 0 }
    );
    assert_eq!(
        prepared.prepared_segment_count(),
        prepared.prepared_material_segment_count() + prepared.prepared_hole_segment_count()
    );
    assert_eq!(prepared.decided_material_segment_box_count(), 8);
    assert_eq!(prepared.decided_hole_segment_box_count(), 4);
    assert_eq!(prepared.decided_segment_box_count(), 12);
    assert_eq!(
        prepared.decided_segment_box_count(),
        prepared.decided_material_segment_box_count() + prepared.decided_hole_segment_box_count()
    );
    assert_eq!(prepared.undecided_material_segment_box_count(), 0);
    assert_eq!(prepared.undecided_hole_segment_box_count(), 0);
    assert_eq!(prepared.undecided_segment_box_count(), 0);
    assert_eq!(
        prepared.undecided_segment_box_count(),
        prepared.undecided_material_segment_box_count()
            + prepared.undecided_hole_segment_box_count()
    );

    for point in [p(1, 1), p(3, 3), p(5, 5), p(11, 1), p(100, 100)] {
        assert_eq!(
            prepared.classify_point(&point, &policy),
            region.classify_point(&point, &policy)
        );
        assert_eq!(
            prepared.signed_depth(&point, &policy),
            region.signed_depth(&point, &policy)
        );
    }
}

#[test]
fn prepared_region_view_preserves_boundary_hits() {
    let material = [rectangle(0, 0, 4, 4), rectangle(20, 20, 24, 24)];
    let holes: [Contour2; 0] = [];
    let view = RegionView2::new(&material, &holes);
    let policy = policy();
    let prepared = view.prepare_point_classifier(&policy);

    assert_eq!(
        prepared.classify_point(&p(20, 22), &policy),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        prepared.classify_point(&p(21, 21), &policy),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        prepared.classify_point(&p(100, 100), &policy),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        prepared.signed_depth(&p(100, 100), &policy),
        Classification::Decided(0)
    );
}
