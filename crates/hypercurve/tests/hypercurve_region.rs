use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    ExactCurveArrangementAttempt2, ExactCurveArrangementRequest2,
    ExactCurveArrangementSourceAabbStatus2, ExactCurveArrangementSourceEndpoint2,
    ExactCurveArrangementSplitCandidateAabbStatus2, FillRule, FiniteProjectionOptions, Real,
    Region2, RegionBoundaryContourBuildPredicatePath2, RegionBoundaryContourBuildStage2,
    RegionBoundaryContourRole2, RegionLineSegmentArrangedEndpoint2,
    RegionLineSegmentEndpointGraphPredicatePath2, RegionLineSegmentRegionBuildStage2,
    RegionLineSegmentRingAssemblyPredicatePath2, RegionLineSegmentSplitPredicatePath2,
    RegionPointLocation, RegionView2, Segment2, SegmentKind, SegmentKindCounts, UncertaintyReason,
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
    CurvePolicy::certified()
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
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        RegionBoundaryContourBuildStage2::RoleAssignment
    );
    assert_eq!(
        report.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(report.source_contour_count(), 2);
    assert_eq!(report.source_segment_count(), 8);
    assert_eq!(report.validation_candidate_pair_count(), 1);
    assert_eq!(report.validation_tested_pair_count(), 1);
    assert_eq!(report.validation_intersection_event_count(), 0);
    assert_eq!(report.nesting_classification_count(), 2);
    assert_eq!(report.blocker_first_contour_index(), None);
    assert_eq!(report.blocker_second_contour_index(), None);
    assert_eq!(report.output_contour_count(), Some(2));
    assert_eq!(report.output_segment_count(), Some(8));
    assert_eq!(report.material_contour_count(), Some(1));
    assert_eq!(report.hole_contour_count(), Some(1));
    assert_eq!(report.material_segment_count(), Some(4));
    assert_eq!(report.hole_segment_count(), Some(4));
    assert_eq!(report.blocker(), None);
    assert_eq!(report.role_reports().len(), 2);

    let outer = &report.role_reports()[0];
    assert_eq!(outer.source_contour_index(), 0);
    assert_eq!(outer.source_segment_count(), 4);
    assert_eq!(outer.source_fill_rule(), FillRule::NonZero);
    assert_eq!(outer.nesting_sample_point(), &p(0, 0));
    assert!(outer.containing_contour_indices().is_empty());
    assert_eq!(outer.nesting_depth(), 0);
    assert_eq!(outer.role(), RegionBoundaryContourRole2::Material);
    assert_eq!(outer.output_role_index(), 0);
    assert!(outer.status().is_native_exact());

    let hole = &report.role_reports()[1];
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
        region.classify_point(&p(1, 1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn borrowed_boundary_contours_build_region_with_report() {
    let contours = vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)];
    let built = Region2::from_boundary_contours_borrowed_with_report(&contours, &policy()).unwrap();
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        RegionBoundaryContourBuildStage2::RoleAssignment
    );
    assert_eq!(
        report.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(report.source_contour_count(), 2);
    assert_eq!(report.source_segment_count(), 8);
    assert_eq!(report.output_contour_count(), Some(2));
    assert_eq!(report.output_segment_count(), Some(8));
    assert_eq!(report.material_contour_count(), Some(1));
    assert_eq!(report.hole_contour_count(), Some(1));
    assert_eq!(report.blocker(), None);
    assert_eq!(report.role_reports().len(), 2);
    assert_eq!(
        report.role_reports()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(
        report.role_reports()[1].role(),
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
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionBoundaryContourBuildStage2::NestingValidation
    );
    assert_eq!(
        report.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(report.source_contour_count(), 2);
    assert_eq!(report.source_segment_count(), 8);
    assert_eq!(report.validation_candidate_pair_count(), 1);
    assert_eq!(report.validation_tested_pair_count(), 1);
    assert_eq!(report.validation_intersection_event_count(), 2);
    assert_eq!(report.nesting_classification_count(), 0);
    assert_eq!(report.blocker_first_contour_index(), Some(0));
    assert_eq!(report.blocker_second_contour_index(), Some(1));
    assert_eq!(report.output_contour_count(), None);
    assert_eq!(report.output_segment_count(), None);
    assert_eq!(report.material_contour_count(), None);
    assert_eq!(report.hole_contour_count(), None);
    assert_eq!(report.material_segment_count(), None);
    assert_eq!(report.hole_segment_count(), None);
    assert!(report.role_reports().is_empty());
}

#[test]
fn boundary_contour_region_report_blocks_touching_roles_with_source_pair() {
    let built = Region2::from_boundary_contours_with_report(
        vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionBoundaryContourBuildStage2::NestingValidation
    );
    assert_eq!(
        report.predicate_path(),
        RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment
    );
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(report.validation_candidate_pair_count(), 1);
    assert_eq!(report.validation_tested_pair_count(), 1);
    assert_eq!(report.validation_intersection_event_count(), 7);
    assert_eq!(report.nesting_classification_count(), 0);
    assert_eq!(report.blocker_first_contour_index(), Some(0));
    assert_eq!(report.blocker_second_contour_index(), Some(1));
    assert_eq!(report.output_contour_count(), None);
    assert!(report.role_reports().is_empty());
}

#[test]
fn unordered_line_segments_build_region_with_source_provenance() {
    let built = Region2::from_unordered_line_segments_with_report(
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
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RegionRoleAssignment
    );
    assert_eq!(report.source_segment_count(), 4);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(report.arranged_segment_count(), Some(4));
    assert_eq!(
        report.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(report.split_candidate_pair_count(), 6);
    assert_eq!(
        report.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(
        report.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        report.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(report.split_skipped_aabb_pair_count(), 2);
    assert_eq!(report.split_tested_pair_count(), 4);
    assert_eq!(report.split_intersection_event_count(), 4);
    assert_eq!(report.split_point_relation_count(), 4);
    assert_eq!(report.split_overlap_relation_count(), 0);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert_eq!(report.split_intersection_points().len(), 4);
    assert_eq!(report.split_intersection_reports().len(), 4);
    assert!(report.split_intersection_points().contains(&p(0, 0)));
    assert!(report.split_intersection_points().contains(&p(4, 0)));
    assert!(report.split_intersection_points().contains(&p(0, 4)));
    assert!(report.split_intersection_points().contains(&p(4, 4)));
    assert_eq!(report.split_output_segment_count(), Some(4));
    assert_eq!(report.split_blocker_first_source_segment_index(), None);
    assert_eq!(report.split_blocker_first_source_segment_kind(), None);
    assert_eq!(report.split_blocker_first_source_start_point(), None);
    assert_eq!(report.split_blocker_first_source_end_point(), None);
    assert_eq!(report.split_blocker_second_source_segment_index(), None);
    assert_eq!(report.split_blocker_second_source_segment_kind(), None);
    assert_eq!(report.split_blocker_second_source_start_point(), None);
    assert_eq!(report.split_blocker_second_source_end_point(), None);
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(4));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(2));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(report.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(report.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(report.endpoint_graph_blocker_point(), None);
    assert_eq!(report.reversed_source_segment_count(), 2);
    assert_eq!(report.output_ring_count(), Some(1));
    assert_eq!(report.output_boundary_segment_count(), Some(4));
    assert_eq!(
        report.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(report.arranged_source_reports().len(), 4);
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_index(),
        0
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), s(1))
    );
    assert_eq!(report.source_reports().len(), 4);
    assert_eq!(report.source_reports()[0].source_segment_index(), 0);
    assert_eq!(
        report.source_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.source_reports()[0].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.source_reports()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), s(1))
    );
    assert!(!report.source_reports()[0].reversed());
    assert_eq!(
        report.source_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.source_reports()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(report.source_reports()[1].source_segment_index(), 3);
    assert!(!report.source_reports()[1].reversed());
    assert_eq!(report.source_reports()[2].source_segment_index(), 1);
    assert!(report.source_reports()[2].reversed());
    assert_eq!(
        report.source_reports()[2].source_segment_start_point(),
        &p(0, 4)
    );
    assert_eq!(
        report.source_reports()[2].source_segment_end_point(),
        &p(4, 4)
    );
    assert_eq!(report.source_reports()[3].source_segment_index(), 2);
    assert!(report.source_reports()[3].reversed());
    assert!(report.exact_endpoint_connection_count() >= 4);
    assert_eq!(report.unresolved_endpoint_connection_count(), 0);
    assert_eq!(report.blocker(), None);

    let boundary_report = report.boundary_build_report().unwrap();
    assert_eq!(boundary_report.validation_intersection_event_count(), 0);
    assert_eq!(boundary_report.material_contour_count(), Some(1));
    assert_eq!(boundary_report.hole_contour_count(), Some(0));

    let region = built.region().unwrap();
    assert_eq!(
        region.classify_point(&p(2, 2), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unordered_line_segments_report_disconnected_boundary_blocker() {
    let built = Region2::from_unordered_line_segments_with_report(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(report.arranged_segment_count(), Some(2));
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(report.split_skipped_aabb_pair_count(), 1);
    assert_eq!(report.split_tested_pair_count(), 0);
    assert_eq!(report.split_intersection_event_count(), 0);
    assert_eq!(report.split_point_relation_count(), 0);
    assert_eq!(report.split_overlap_relation_count(), 0);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert!(report.split_intersection_points().is_empty());
    assert_eq!(report.split_output_segment_count(), Some(2));
    assert_eq!(
        report.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        report.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(4));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(1));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(
        report.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        report.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(report.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    assert_eq!(report.arranged_source_reports().len(), 2);
    assert_eq!(report.output_ring_count(), None);
    assert_eq!(report.output_boundary_segment_count(), None);
    assert_eq!(report.source_reports().len(), 0);
    assert_eq!(report.boundary_build_report(), None);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_line_segments_split_crossings_before_boundary_blocker() {
    let built = Region2::from_unordered_line_segments_with_report(
        vec![line(0, 0, 4, 4), line(0, 4, 4, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(report.arranged_segment_count(), Some(4));
    assert_eq!(
        report.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 0 })
    );
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(
        report.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 1);
    assert_eq!(report.split_point_relation_count(), 1);
    assert_eq!(report.split_overlap_relation_count(), 0);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert_eq!(report.split_intersection_points(), &[p(2, 2)]);
    assert_eq!(report.split_intersection_reports().len(), 1);
    let event: &hypercurve::RegionLineSegmentSplitIntersectionReport2 =
        &report.split_intersection_reports()[0];
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
    assert_eq!(report.split_output_segment_count(), Some(4));
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        report.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        report.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(report.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    assert_eq!(report.arranged_source_reports().len(), 4);
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_index(),
        0
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 2))
    );
    assert_eq!(report.source_reports().len(), 0);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_line_segments_report_overlap_source_pair_blocker() {
    let built = Region2::from_unordered_line_segments_with_report(
        vec![line(0, 0, 4, 0), line(2, 0, 6, 0)],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(report.arranged_segment_count(), None);
    assert_eq!(report.arranged_segment_kind_counts(), None);
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(
        report.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 0);
    assert_eq!(report.split_point_relation_count(), 0);
    assert_eq!(report.split_overlap_relation_count(), 1);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert!(report.split_intersection_points().is_empty());
    assert_eq!(report.split_output_segment_count(), None);
    assert_eq!(report.endpoint_graph_predicate_path(), None);
    assert_eq!(report.ring_assembly_predicate_path(), None);
    assert_eq!(report.split_blocker_first_source_segment_index(), Some(0));
    assert_eq!(
        report.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        report.split_blocker_first_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        report.split_blocker_first_source_end_point(),
        Some(&p(4, 0))
    );
    assert_eq!(report.split_blocker_second_source_segment_index(), Some(1));
    assert_eq!(
        report.split_blocker_second_source_segment_kind(),
        Some(SegmentKind::Line)
    );
    assert_eq!(
        report.split_blocker_second_source_start_point(),
        Some(&p(2, 0))
    );
    assert_eq!(
        report.split_blocker_second_source_end_point(),
        Some(&p(6, 0))
    );
    assert_eq!(report.arranged_source_reports().len(), 0);
    assert_eq!(report.output_boundary_segment_kind_counts(), None);
    assert_eq!(report.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(report.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn borrowed_unordered_line_segments_build_region_with_report() {
    let segments = vec![
        line(0, 0, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
        line(4, 0, 4, 4),
    ];

    let built = Region2::from_unordered_line_segments_borrowed_with_report(
        &segments,
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_some());
    assert_eq!(segments.len(), 4);
    assert!(report.status().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(report.arranged_segment_count(), Some(4));
    assert_eq!(report.arranged_source_reports().len(), 4);
    assert_eq!(report.source_reports().len(), 4);
}

#[test]
fn unordered_native_segments_build_line_arc_region_with_source_provenance() {
    let built = Region2::from_unordered_segments_with_report(
        vec![
            Segment2::Line(line(4, 0, 0, 0)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RegionRoleAssignment
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(report.arranged_segment_count(), Some(2));
    assert_eq!(
        report.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(
        report.split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment)
    );
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 2);
    assert_eq!(report.split_point_relation_count(), 1);
    assert_eq!(report.split_overlap_relation_count(), 0);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert_eq!(report.split_intersection_points().len(), 2);
    assert_eq!(report.split_intersection_reports().len(), 2);
    assert!(report.split_intersection_points().contains(&p(0, 0)));
    assert!(report.split_intersection_points().contains(&p(4, 0)));
    assert_eq!(report.split_output_segment_count(), Some(2));
    assert_eq!(
        report.endpoint_graph_predicate_path(),
        Some(RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets)
    );
    assert_eq!(
        report.ring_assembly_predicate_path(),
        Some(RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal)
    );
    assert_eq!(report.split_blocker_first_source_segment_index(), None);
    assert_eq!(report.split_blocker_first_source_segment_kind(), None);
    assert_eq!(report.split_blocker_first_source_start_point(), None);
    assert_eq!(report.split_blocker_first_source_end_point(), None);
    assert_eq!(report.split_blocker_second_source_segment_index(), None);
    assert_eq!(report.split_blocker_second_source_segment_kind(), None);
    assert_eq!(report.split_blocker_second_source_start_point(), None);
    assert_eq!(report.split_blocker_second_source_end_point(), None);
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(2));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(0)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(2));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(0));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(0));
    assert_eq!(report.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(report.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(report.endpoint_graph_blocker_point(), None);
    assert_eq!(report.reversed_source_segment_count(), 0);
    assert_eq!(report.output_ring_count(), Some(1));
    assert_eq!(report.output_boundary_segment_count(), Some(2));
    assert_eq!(
        report.output_boundary_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 1, arcs: 1 })
    );
    assert_eq!(report.arranged_source_reports().len(), 2);
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_index(),
        0
    );
    assert_eq!(
        report.arranged_source_reports()[1].source_segment_index(),
        1
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_start_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_end_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].arranged_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.arranged_source_reports()[1].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.arranged_source_reports()[1].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[1].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[1].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(report.source_reports().len(), 2);
    assert_eq!(report.source_reports()[0].source_segment_index(), 0);
    assert_eq!(report.source_reports()[1].source_segment_index(), 1);
    assert_eq!(
        report.source_reports()[0].source_segment_start_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.source_reports()[0].source_segment_end_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.source_reports()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.source_reports()[0].output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.source_reports()[1].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.source_reports()[1].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.source_reports()[1].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.source_reports()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(report.blocker(), None);

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
fn borrowed_unordered_native_segments_build_line_arc_region_with_report() {
    let segments = vec![
        Segment2::Line(line(4, 0, 0, 0)),
        Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
    ];

    let built = Region2::from_unordered_segments_borrowed_with_report(
        &segments,
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_some());
    assert_eq!(segments.len(), 2);
    assert!(report.status().is_native_exact());
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(report.arranged_segment_count(), Some(2));
    assert_eq!(report.arranged_source_reports().len(), 2);
    assert_eq!(report.source_reports().len(), 2);
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
    let legacy = Region2::from_unordered_line_segments_with_report(
        lines.clone(),
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert_eq!(attempt.request().source_segment_count(), 4);
    assert_eq!(
        attempt.request().source_line_segments(),
        Some(lines.as_slice())
    );
    assert_eq!(attempt.request().source_segments().len(), 4);
    assert!(
        attempt
            .request()
            .source_segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
    assert_eq!(
        result.workspace().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(result.workspace().source_segment_aabbs().len(), 4);
    assert_eq!(result.workspace().decided_source_segment_aabb_count(), 4);
    assert_eq!(result.workspace().undecided_source_segment_aabb_count(), 0);
    assert_eq!(
        result.workspace().source_segment_aabbs()[0]
            .as_ref()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        Some((p(0, 0), p(4, 0)))
    );
    assert_eq!(
        result
            .workspace()
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        Some((p(0, 0), p(4, 4)))
    );
    let source_segment_cache = result.workspace().source_segment_cache();
    assert_eq!(
        source_segment_cache.source_segment_count(),
        attempt.request().source_segment_count()
    );
    assert_eq!(
        source_segment_cache.source_segment_kind_counts(),
        result.workspace().source_segment_kind_counts()
    );
    assert_eq!(
        source_segment_cache.decided_source_segment_aabb_count(),
        result.workspace().decided_source_segment_aabb_count()
    );
    assert_eq!(
        source_segment_cache.undecided_source_segment_aabb_count(),
        result.workspace().undecided_source_segment_aabb_count()
    );
    assert_eq!(
        source_segment_cache
            .source_aabb()
            .map(|bbox| (bbox.min().clone(), bbox.max().clone())),
        result
            .workspace()
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
    let source_endpoint_cache = result.workspace().source_endpoint_bucket_cache();
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
    let split_schedule_cache = result.workspace().split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.report().split_candidate_pair_count()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.report().split_skipped_aabb_pair_count()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.report().split_tested_pair_count()
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
    let split_cache = result.workspace().split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        result.report().split_predicate_path()
    );
    assert_eq!(
        split_cache.candidate_pair_count(),
        result.report().split_candidate_pair_count()
    );
    assert_eq!(
        split_cache.skipped_aabb_pair_count(),
        result.report().split_skipped_aabb_pair_count()
    );
    assert_eq!(
        split_cache.tested_pair_count(),
        result.report().split_tested_pair_count()
    );
    assert_eq!(
        split_cache.intersection_event_count(),
        result.report().split_intersection_event_count()
    );
    assert_eq!(
        split_cache.intersection_points(),
        result.report().split_intersection_points()
    );
    assert_eq!(
        split_cache.intersection_reports(),
        result.report().split_intersection_reports()
    );
    assert_eq!(split_cache.output_segment_count(), Some(4));
    assert_eq!(split_cache.blocker_cache(), None);
    let split_intersection_bucket_cache = split_cache.intersection_bucket_cache();
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
        result.report().split_intersection_reports()[0].point()
    );
    let endpoint_cache = result.workspace().endpoint_graph_cache().unwrap();
    assert_eq!(
        Some(endpoint_cache.predicate_path()),
        result.report().endpoint_graph_predicate_path()
    );
    assert_eq!(
        Some(endpoint_cache.endpoint_count()),
        result.report().endpoint_graph_endpoint_count()
    );
    assert_eq!(
        Some(endpoint_cache.structural_bucket_count()),
        result.report().endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        Some(endpoint_cache.structural_singleton_bucket_count()),
        result
            .report()
            .endpoint_graph_structural_singleton_bucket_count()
    );
    assert_eq!(
        Some(endpoint_cache.max_structural_bucket_size()),
        result.report().endpoint_graph_max_structural_bucket_size()
    );
    let arranged_endpoint_bucket_cache = endpoint_cache.endpoint_bucket_cache();
    assert_eq!(
        arranged_endpoint_bucket_cache.endpoint_count(),
        endpoint_cache.endpoint_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.bucket_count(),
        endpoint_cache.structural_bucket_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.singleton_bucket_count(),
        endpoint_cache.structural_singleton_bucket_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.max_bucket_size(),
        endpoint_cache.max_structural_bucket_size()
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
    assert_eq!(endpoint_cache.dangling_endpoint_count(), 0);
    assert_eq!(endpoint_cache.branch_endpoint_count(), 0);
    assert_eq!(endpoint_cache.blocker_arranged_segment_index(), None);
    assert_eq!(endpoint_cache.blocker_endpoint(), None);
    assert_eq!(endpoint_cache.blocker_point(), None);
    let ring_cache = result.workspace().ring_assembly_cache().unwrap();
    assert_eq!(
        Some(ring_cache.predicate_path()),
        result.report().ring_assembly_predicate_path()
    );
    assert_eq!(
        ring_cache.attempted_endpoint_connection_count(),
        result.report().attempted_endpoint_connection_count()
    );
    assert_eq!(
        ring_cache.exact_endpoint_connection_count(),
        result.report().exact_endpoint_connection_count()
    );
    assert_eq!(
        ring_cache.disconnected_endpoint_connection_count(),
        result.report().disconnected_endpoint_connection_count()
    );
    assert_eq!(
        ring_cache.unresolved_endpoint_connection_count(),
        result.report().unresolved_endpoint_connection_count()
    );
    assert_eq!(
        ring_cache.reversed_source_segment_count(),
        result.report().reversed_source_segment_count()
    );
    assert_eq!(ring_cache.output_ring_count(), Some(1));
    assert_eq!(ring_cache.output_boundary_segment_count(), Some(4));
    assert_eq!(
        ring_cache.arranged_source_reports(),
        result.report().arranged_source_reports()
    );
    let arranged_fragment_cache = ring_cache.arranged_fragment_cache();
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
    assert_eq!(arranged_fragment_cache.max_source_ref_count(), 1);
    assert_eq!(arranged_fragment_cache.fragments().len(), 4);
    let arranged_fragment = &arranged_fragment_cache.fragments()[0];
    assert_eq!(arranged_fragment.arranged_segment_index(), 0);
    assert_eq!(arranged_fragment.arranged_segment_kind(), SegmentKind::Line);
    assert_eq!(
        arranged_fragment.output_start_point(),
        result.report().arranged_source_reports()[0].output_start_point()
    );
    assert_eq!(
        arranged_fragment.output_end_point(),
        result.report().arranged_source_reports()[0].output_end_point()
    );
    assert_eq!(arranged_fragment.source_refs().len(), 1);
    assert_eq!(
        arranged_fragment.source_refs()[0].arranged_source_report_index(),
        0
    );
    assert_eq!(arranged_fragment.source_refs()[0].source_segment_index(), 0);
    assert_eq!(
        arranged_fragment.source_refs()[0].source_range(),
        result.report().arranged_source_reports()[0].source_range()
    );
    assert_eq!(
        arranged_fragment.source_refs()[0].status(),
        result.report().arranged_source_reports()[0].status()
    );
    assert_eq!(
        ring_cache.source_reports(),
        result.report().source_reports()
    );
    let output_ring_bucket_cache = ring_cache.output_ring_bucket_cache();
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
        result.report().source_reports()[0].reversed()
    );
    let output_segment_kind_bucket_cache = ring_cache.output_segment_kind_bucket_cache();
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
        result
            .report()
            .output_boundary_segment_kind_counts()
            .unwrap()
            .lines
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].source_report_index(),
        0
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].output_ring_index(),
        result.report().source_reports()[0].output_ring_index()
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].output_segment_index(),
        result.report().source_reports()[0].output_segment_index()
    );
    assert_eq!(
        result.report().source_reports()
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
    let output_cache = result.workspace().output_cache().unwrap();
    assert!(output_cache.materialized_region());
    assert_eq!(output_cache.status(), result.report().status());
    assert_eq!(output_cache.blocker(), None);
    assert_eq!(
        output_cache.boundary_build_report(),
        result.report().boundary_build_report()
    );
    let boundary_report = output_cache.boundary_build_report().unwrap();
    assert_eq!(boundary_report.material_contour_count(), Some(1));
    assert_eq!(boundary_report.hole_contour_count(), Some(0));
    let boundary_output_cache = output_cache.boundary_output_cache().unwrap();
    assert_eq!(boundary_output_cache.output_contour_count(), 1);
    assert_eq!(boundary_output_cache.output_segment_count(), 4);
    assert_eq!(
        boundary_output_cache.output_segment_kind_counts(),
        result
            .report()
            .output_boundary_segment_kind_counts()
            .unwrap()
    );
    assert_eq!(boundary_output_cache.material_contour_count(), 1);
    assert_eq!(boundary_output_cache.hole_contour_count(), 0);
    assert_eq!(boundary_output_cache.material_segment_count(), 4);
    assert_eq!(boundary_output_cache.hole_segment_count(), 0);
    let role_cache = output_cache.role_cache().unwrap();
    assert_eq!(
        role_cache.role_report_count(),
        boundary_report.role_reports().len()
    );
    assert_eq!(role_cache.material_contour_count(), 1);
    assert_eq!(role_cache.hole_contour_count(), 0);
    assert_eq!(role_cache.material_segment_count(), 4);
    assert_eq!(role_cache.hole_segment_count(), 0);
    assert_eq!(role_cache.buckets().len(), 2);
    assert_eq!(
        role_cache.buckets()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(role_cache.buckets()[0].assignments().len(), 1);
    assert_eq!(
        role_cache.buckets()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert!(role_cache.buckets()[1].assignments().is_empty());
    let role_assignment = &role_cache.buckets()[0].assignments()[0];
    assert_eq!(role_assignment.role_report_index(), 0);
    assert_eq!(role_assignment.source_contour_index(), 0);
    assert_eq!(role_assignment.source_segment_count(), 4);
    assert_eq!(role_assignment.source_fill_rule(), FillRule::NonZero);
    assert_eq!(
        role_assignment.nesting_sample_point(),
        boundary_report.role_reports()[0].nesting_sample_point()
    );
    assert_eq!(role_assignment.containing_contour_indices(), &[]);
    assert_eq!(role_assignment.nesting_depth(), 0);
    assert_eq!(role_assignment.output_role_index(), 0);
    assert_eq!(
        role_assignment.status(),
        boundary_report.role_reports()[0].status()
    );
    assert!(result.region().is_some());
    assert_eq!(
        result.report().split_predicate_path(),
        Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine)
    );
    assert_eq!(result.report(), legacy.report());
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
    let legacy = Region2::from_unordered_segments_with_report(
        segments.clone(),
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    assert_eq!(attempt.request().source_segment_count(), 2);
    assert_eq!(attempt.request().fill_rule(), FillRule::NonZero);
    assert_eq!(attempt.request().source_segments(), segments.as_slice());
    assert_eq!(result.workspace().request(), attempt.request());
    assert_eq!(
        result.workspace().source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(result.workspace().source_segment_aabbs().len(), 2);
    assert_eq!(result.workspace().decided_source_segment_aabb_count(), 2);
    assert_eq!(result.workspace().undecided_source_segment_aabb_count(), 0);
    assert_eq!(
        result
            .workspace()
            .source_aabb()
            .map(|bbox| bbox.min().clone()),
        Some(p(0, -2))
    );
    let source_segment_cache = result.workspace().source_segment_cache();
    assert_eq!(
        source_segment_cache.source_segment_count(),
        attempt.request().source_segment_count()
    );
    assert_eq!(
        source_segment_cache.source_segment_kind_counts(),
        result.workspace().source_segment_kind_counts()
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
    let source_endpoint_cache = result.workspace().source_endpoint_bucket_cache();
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
    let split_schedule_cache = result.workspace().split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.report().split_candidate_pair_count()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.report().split_skipped_aabb_pair_count()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.report().split_tested_pair_count()
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
    let split_cache = result.workspace().split_cache().unwrap();
    assert_eq!(
        split_cache.predicate_path(),
        result.report().split_predicate_path()
    );
    assert_eq!(
        split_cache.intersection_event_count(),
        result.report().split_intersection_event_count()
    );
    assert_eq!(
        split_cache.intersection_reports(),
        result.report().split_intersection_reports()
    );
    assert_eq!(
        split_cache.output_segment_count(),
        result.report().split_output_segment_count()
    );
    assert_eq!(split_cache.blocker_cache(), None);
    let split_intersection_bucket_cache = split_cache.intersection_bucket_cache();
    assert_eq!(
        split_intersection_bucket_cache.intersection_event_count(),
        result.report().split_intersection_event_count()
    );
    assert_eq!(
        split_intersection_bucket_cache.bucket_count(),
        result.report().split_intersection_points().len()
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
        result.report().split_intersection_reports()[0].point()
    );
    let endpoint_cache = result.workspace().endpoint_graph_cache().unwrap();
    assert_eq!(
        Some(endpoint_cache.predicate_path()),
        result.report().endpoint_graph_predicate_path()
    );
    assert_eq!(
        Some(endpoint_cache.endpoint_count()),
        result.report().endpoint_graph_endpoint_count()
    );
    assert_eq!(
        Some(endpoint_cache.structural_bucket_count()),
        result.report().endpoint_graph_structural_bucket_count()
    );
    assert_eq!(
        Some(endpoint_cache.max_structural_bucket_size()),
        result.report().endpoint_graph_max_structural_bucket_size()
    );
    let arranged_endpoint_bucket_cache = endpoint_cache.endpoint_bucket_cache();
    assert_eq!(
        arranged_endpoint_bucket_cache.endpoint_count(),
        endpoint_cache.endpoint_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.bucket_count(),
        endpoint_cache.structural_bucket_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.singleton_bucket_count(),
        endpoint_cache.structural_singleton_bucket_count()
    );
    assert_eq!(
        arranged_endpoint_bucket_cache.max_bucket_size(),
        endpoint_cache.max_structural_bucket_size()
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
    assert_eq!(
        Some(endpoint_cache.dangling_endpoint_count()),
        result.report().endpoint_graph_dangling_endpoint_count()
    );
    assert_eq!(
        Some(endpoint_cache.branch_endpoint_count()),
        result.report().endpoint_graph_branch_endpoint_count()
    );
    assert_eq!(endpoint_cache.blocker_point(), None);
    let ring_cache = result.workspace().ring_assembly_cache().unwrap();
    assert_eq!(
        Some(ring_cache.predicate_path()),
        result.report().ring_assembly_predicate_path()
    );
    assert_eq!(
        ring_cache.attempted_endpoint_connection_count(),
        result.report().attempted_endpoint_connection_count()
    );
    assert_eq!(
        ring_cache.output_ring_count(),
        result.report().output_ring_count()
    );
    assert_eq!(
        ring_cache.output_boundary_segment_kind_counts(),
        result.report().output_boundary_segment_kind_counts()
    );
    assert_eq!(
        ring_cache.arranged_source_reports(),
        result.report().arranged_source_reports()
    );
    let arranged_fragment_cache = ring_cache.arranged_fragment_cache();
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
    assert_eq!(arranged_fragment_cache.max_source_ref_count(), 1);
    assert_eq!(arranged_fragment_cache.fragments().len(), 2);
    let arranged_fragment = &arranged_fragment_cache.fragments()[0];
    assert_eq!(arranged_fragment.arranged_segment_index(), 0);
    assert_eq!(arranged_fragment.arranged_segment_kind(), SegmentKind::Line);
    assert_eq!(
        arranged_fragment.output_start_point(),
        result.report().arranged_source_reports()[0].output_start_point()
    );
    assert_eq!(
        arranged_fragment.output_end_point(),
        result.report().arranged_source_reports()[0].output_end_point()
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
        result.report().arranged_source_reports()[0].source_range()
    );
    assert_eq!(
        ring_cache.source_reports(),
        result.report().source_reports()
    );
    let output_ring_bucket_cache = ring_cache.output_ring_bucket_cache();
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
        result.report().source_reports()[0].reversed()
    );
    let output_segment_kind_bucket_cache = ring_cache.output_segment_kind_bucket_cache();
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
        result.report().source_reports()
            [output_segment_kind_bucket_cache.buckets()[0].segment_refs()[0].source_report_index()]
        .output_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        output_segment_kind_bucket_cache.buckets()[1].output_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        result.report().source_reports()
            [output_segment_kind_bucket_cache.buckets()[1].segment_refs()[0].source_report_index()]
        .output_segment_kind(),
        SegmentKind::Arc
    );
    let output_cache = result.workspace().output_cache().unwrap();
    assert!(output_cache.materialized_region());
    assert_eq!(output_cache.status(), result.report().status());
    assert_eq!(output_cache.blocker(), None);
    assert_eq!(
        output_cache.boundary_build_report(),
        result.report().boundary_build_report()
    );
    let boundary_report = output_cache.boundary_build_report().unwrap();
    let boundary_output_cache = output_cache.boundary_output_cache().unwrap();
    assert_eq!(boundary_output_cache.output_contour_count(), 1);
    assert_eq!(boundary_output_cache.output_segment_count(), 2);
    assert_eq!(
        boundary_output_cache.output_segment_kind_counts(),
        result
            .report()
            .output_boundary_segment_kind_counts()
            .unwrap()
    );
    assert_eq!(boundary_output_cache.material_contour_count(), 1);
    assert_eq!(boundary_output_cache.hole_contour_count(), 0);
    assert_eq!(boundary_output_cache.material_segment_count(), 2);
    assert_eq!(boundary_output_cache.hole_segment_count(), 0);
    let role_cache = output_cache.role_cache().unwrap();
    assert_eq!(
        role_cache.role_report_count(),
        boundary_report.role_reports().len()
    );
    assert_eq!(role_cache.material_contour_count(), 1);
    assert_eq!(role_cache.hole_contour_count(), 0);
    assert_eq!(role_cache.material_segment_count(), 2);
    assert_eq!(role_cache.hole_segment_count(), 0);
    assert_eq!(role_cache.buckets().len(), 2);
    assert_eq!(
        role_cache.buckets()[0].role(),
        RegionBoundaryContourRole2::Material
    );
    assert_eq!(role_cache.buckets()[0].assignments().len(), 1);
    assert_eq!(
        role_cache.buckets()[1].role(),
        RegionBoundaryContourRole2::Hole
    );
    assert!(role_cache.buckets()[1].assignments().is_empty());
    let role_assignment = &role_cache.buckets()[0].assignments()[0];
    assert_eq!(role_assignment.role_report_index(), 0);
    assert_eq!(role_assignment.source_contour_index(), 0);
    assert_eq!(role_assignment.source_segment_count(), 2);
    assert_eq!(role_assignment.source_fill_rule(), FillRule::NonZero);
    assert_eq!(
        role_assignment.nesting_sample_point(),
        boundary_report.role_reports()[0].nesting_sample_point()
    );
    assert_eq!(role_assignment.containing_contour_indices(), &[]);
    assert_eq!(role_assignment.nesting_depth(), 0);
    assert_eq!(role_assignment.output_role_index(), 0);
    assert_eq!(
        role_assignment.status(),
        boundary_report.role_reports()[0].status()
    );
    assert!(result.region().is_some());
    assert!(result.report().status().is_native_exact());
    assert_eq!(result.report(), legacy.report());
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
    .evaluate(&policy())
    .unwrap();
    let legacy =
        Region2::from_unordered_segments_with_report(segments, FillRule::NonZero, &policy())
            .unwrap();

    assert!(result.region().is_none());
    assert!(result.report().status().is_retained_evidence());
    assert_eq!(
        result.report().stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(result.report().blocker(), Some(UncertaintyReason::Boundary));
    let source_endpoint_cache = result.workspace().source_endpoint_bucket_cache();
    assert_eq!(source_endpoint_cache.endpoint_count(), 4);
    assert_eq!(source_endpoint_cache.bucket_count(), 2);
    assert_eq!(source_endpoint_cache.singleton_bucket_count(), 0);
    assert_eq!(source_endpoint_cache.max_bucket_size(), 2);
    let split_schedule_cache = result.workspace().split_schedule_cache();
    assert_eq!(
        split_schedule_cache.candidate_pair_count(),
        result.report().split_candidate_pair_count()
    );
    assert_eq!(
        split_schedule_cache.decided_disjoint_pair_count(),
        result.report().split_skipped_aabb_pair_count()
    );
    assert_eq!(
        split_schedule_cache.predicate_candidate_pair_count(),
        result.report().split_tested_pair_count()
    );
    assert_eq!(split_schedule_cache.undecided_aabb_pair_count(), 0);
    assert_eq!(split_schedule_cache.candidate_pairs().len(), 1);
    assert_eq!(
        split_schedule_cache.candidate_pairs()[0].aabb_status(),
        ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint
    );
    let split_cache = result.workspace().split_cache().unwrap();
    assert_eq!(split_cache.overlap_relation_count(), 1);
    assert_eq!(split_cache.output_segment_count(), None);
    assert_eq!(
        split_cache.intersection_points(),
        result.report().split_intersection_points()
    );
    let split_blocker_cache = split_cache.blocker_cache().unwrap();
    assert_eq!(
        split_blocker_cache.first_source_segment_index(),
        result
            .report()
            .split_blocker_first_source_segment_index()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.first_source_segment_kind(),
        result
            .report()
            .split_blocker_first_source_segment_kind()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.first_source_start_point(),
        result
            .report()
            .split_blocker_first_source_start_point()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.first_source_end_point(),
        result
            .report()
            .split_blocker_first_source_end_point()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.second_source_segment_index(),
        result
            .report()
            .split_blocker_second_source_segment_index()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.second_source_segment_kind(),
        result
            .report()
            .split_blocker_second_source_segment_kind()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.second_source_start_point(),
        result
            .report()
            .split_blocker_second_source_start_point()
            .unwrap()
    );
    assert_eq!(
        split_blocker_cache.second_source_end_point(),
        result
            .report()
            .split_blocker_second_source_end_point()
            .unwrap()
    );
    assert_eq!(split_blocker_cache.blocker(), result.report().blocker());
    let split_intersection_bucket_cache = split_cache.intersection_bucket_cache();
    assert_eq!(
        split_intersection_bucket_cache.intersection_event_count(),
        0
    );
    assert_eq!(split_intersection_bucket_cache.bucket_count(), 0);
    assert_eq!(split_intersection_bucket_cache.singleton_bucket_count(), 0);
    assert_eq!(split_intersection_bucket_cache.max_bucket_size(), 0);
    assert!(split_intersection_bucket_cache.buckets().is_empty());
    assert!(result.workspace().endpoint_graph_cache().is_none());
    assert!(result.workspace().ring_assembly_cache().is_none());
    let output_cache = result.workspace().output_cache().unwrap();
    assert!(!output_cache.materialized_region());
    assert_eq!(output_cache.status(), result.report().status());
    assert_eq!(output_cache.blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(output_cache.boundary_build_report(), None);
    assert_eq!(output_cache.boundary_output_cache(), None);
    assert_eq!(output_cache.role_cache(), None);
    assert_eq!(result.report(), legacy.report());
}

#[test]
fn unordered_native_segments_convenience_returns_decided_region() {
    let built = Region2::from_unordered_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(4, 0, 0, 0)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();

    let Classification::Decided(region) = built else {
        panic!("line-arc native region should materialize");
    };
    assert_eq!(
        region.classify_point(&p(2, -1), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unordered_native_segments_report_arc_overlap_boundary_blocker() {
    let built = Region2::from_unordered_segments_with_report(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 0, arcs: 2 }
    );
    assert_eq!(report.arranged_segment_count(), None);
    assert_eq!(report.arranged_segment_kind_counts(), None);
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 0);
    assert_eq!(report.split_point_relation_count(), 0);
    assert_eq!(report.split_overlap_relation_count(), 1);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert!(report.split_intersection_points().is_empty());
    assert_eq!(report.split_output_segment_count(), None);
    assert_eq!(report.split_blocker_first_source_segment_index(), Some(0));
    assert_eq!(
        report.split_blocker_first_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        report.split_blocker_first_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        report.split_blocker_first_source_end_point(),
        Some(&p(4, 0))
    );
    assert_eq!(report.split_blocker_second_source_segment_index(), Some(1));
    assert_eq!(
        report.split_blocker_second_source_segment_kind(),
        Some(SegmentKind::Arc)
    );
    assert_eq!(
        report.split_blocker_second_source_start_point(),
        Some(&p(0, 0))
    );
    assert_eq!(
        report.split_blocker_second_source_end_point(),
        Some(&p(4, 0))
    );
    assert_eq!(report.endpoint_graph_endpoint_count(), None);
    assert_eq!(report.endpoint_graph_structural_bucket_count(), None);
    assert_eq!(report.endpoint_graph_blocker_arranged_segment_index(), None);
    assert_eq!(report.endpoint_graph_blocker_endpoint(), None);
    assert_eq!(report.arranged_source_reports().len(), 0);
    assert_eq!(report.output_boundary_segment_kind_counts(), None);
    assert_eq!(report.source_reports().len(), 0);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_native_segments_split_line_arc_crossing_before_boundary_blocker() {
    let built = Region2::from_unordered_segments_with_report(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(2, -3, 2, 1)),
        ],
        FillRule::NonZero,
        &policy(),
    )
    .unwrap();
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 1, arcs: 1 }
    );
    assert_eq!(report.arranged_segment_count(), Some(4));
    assert_eq!(
        report.arranged_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 2 })
    );
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 1);
    assert_eq!(report.split_point_relation_count(), 1);
    assert_eq!(report.split_overlap_relation_count(), 0);
    assert_eq!(report.split_uncertain_relation_count(), 0);
    assert_eq!(report.split_intersection_points(), &[p(2, -2)]);
    assert_eq!(report.split_intersection_reports().len(), 1);
    assert_eq!(
        report.split_intersection_reports()[0].first_source_segment_index(),
        0
    );
    assert_eq!(
        report.split_intersection_reports()[0].first_source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.split_intersection_reports()[0].first_source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.split_intersection_reports()[0].first_source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.split_intersection_reports()[0].second_source_segment_index(),
        1
    );
    assert_eq!(
        report.split_intersection_reports()[0].second_source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.split_intersection_reports()[0].second_source_segment_start_point(),
        &p(2, -3)
    );
    assert_eq!(
        report.split_intersection_reports()[0].second_source_segment_end_point(),
        &p(2, 1)
    );
    assert_eq!(report.split_intersection_reports()[0].point(), &p(2, -2));
    assert_eq!(report.split_output_segment_count(), Some(4));
    assert_eq!(report.split_blocker_first_source_segment_kind(), None);
    assert_eq!(report.split_blocker_second_source_segment_kind(), None);
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        report.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        report.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(report.endpoint_graph_blocker_point(), Some(&p(0, 0)));
    assert_eq!(report.arranged_source_reports().len(), 4);
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_index(),
        0
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_end_point(),
        &p(4, 0)
    );
    assert_eq!(
        report.arranged_source_reports()[0].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 2))
    );
    assert_eq!(
        report.arranged_source_reports()[2].source_segment_index(),
        1
    );
    assert_eq!(
        report.arranged_source_reports()[2].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.arranged_source_reports()[2].source_segment_start_point(),
        &p(2, -3)
    );
    assert_eq!(
        report.arranged_source_reports()[2].source_segment_end_point(),
        &p(2, 1)
    );
    assert_eq!(report.source_reports().len(), 0);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_native_segments_split_arc_arc_crossing_before_boundary_blocker() {
    let built = Region2::from_unordered_segments_with_report(
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
    let report = built.report();

    assert!(built.region().is_none());
    assert!(report.status().is_retained_evidence());
    assert_eq!(
        report.stage(),
        RegionLineSegmentRegionBuildStage2::RingAssembly
    );
    assert_eq!(report.source_segment_count(), 2);
    assert_eq!(report.arranged_segment_count(), Some(4));
    assert_eq!(report.split_candidate_pair_count(), 1);
    assert_eq!(report.split_skipped_aabb_pair_count(), 0);
    assert_eq!(report.split_tested_pair_count(), 1);
    assert_eq!(report.split_intersection_event_count(), 1);
    assert_eq!(report.split_intersection_points(), &[p(4, 3)]);
    assert_eq!(report.split_output_segment_count(), Some(4));
    assert_eq!(report.endpoint_graph_endpoint_count(), Some(8));
    assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(5));
    assert_eq!(
        report.endpoint_graph_structural_singleton_bucket_count(),
        Some(4)
    );
    assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(4));
    assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(4));
    assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(4));
    assert_eq!(
        report.endpoint_graph_blocker_arranged_segment_index(),
        Some(0)
    );
    assert_eq!(
        report.endpoint_graph_blocker_endpoint(),
        Some(RegionLineSegmentArrangedEndpoint2::Start)
    );
    assert_eq!(report.endpoint_graph_blocker_point(), Some(&p(5, 0)));
    assert_eq!(report.arranged_source_reports().len(), 4);
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_index(),
        0
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.arranged_source_reports()[0].arranged_segment_kind(),
        SegmentKind::Arc
    );
    assert_eq!(
        report.arranged_source_reports()[0].source_range(),
        &hypercurve::ParamRange::new(s(0), q(1, 10))
    );
    assert_eq!(report.source_reports().len(), 0);
    assert_eq!(report.blocker(), Some(UncertaintyReason::Boundary));
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

        let built = Region2::from_unordered_line_segments_with_report(
            lines,
            FillRule::NonZero,
            &policy(),
        ).unwrap();
        let report = built.report();

        prop_assert!(report.status().is_native_exact());
        prop_assert_eq!(report.source_segment_count(), 4);
        prop_assert_eq!(report.arranged_segment_count(), Some(4));
        prop_assert_eq!(report.split_candidate_pair_count(), 6);
        prop_assert_eq!(report.split_skipped_aabb_pair_count(), 2);
        prop_assert_eq!(report.split_tested_pair_count(), 4);
        prop_assert_eq!(report.split_intersection_event_count(), 4);
        prop_assert_eq!(report.split_intersection_points().len(), 4);
        prop_assert!(report.split_intersection_points().contains(&p(xmin, ymin)));
        prop_assert!(report.split_intersection_points().contains(&p(xmax, ymin)));
        prop_assert!(report.split_intersection_points().contains(&p(xmax, ymax)));
        prop_assert!(report.split_intersection_points().contains(&p(xmin, ymax)));
        prop_assert_eq!(report.endpoint_graph_endpoint_count(), Some(8));
        prop_assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(4));
        prop_assert_eq!(
            report.endpoint_graph_structural_singleton_bucket_count(),
            Some(0)
        );
        prop_assert_eq!(report.endpoint_graph_max_structural_bucket_size(), Some(2));
        prop_assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(0));
        prop_assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(0));
        prop_assert_eq!(report.output_ring_count(), Some(1));
        prop_assert_eq!(report.output_boundary_segment_count(), Some(4));
        prop_assert_eq!(report.blocker(), None);

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

        let built = Region2::from_unordered_segments_with_report(
            segments,
            FillRule::NonZero,
            &policy(),
        ).unwrap();
        let report = built.report();

        prop_assert!(report.status().is_native_exact());
        prop_assert_eq!(report.source_segment_count(), 2);
        prop_assert_eq!(report.arranged_segment_count(), Some(2));
        prop_assert_eq!(report.split_candidate_pair_count(), 1);
        prop_assert_eq!(report.split_skipped_aabb_pair_count(), 0);
        prop_assert_eq!(report.split_tested_pair_count(), 1);
        prop_assert_eq!(report.split_intersection_event_count(), 2);
        prop_assert_eq!(report.split_intersection_points().len(), 2);
        prop_assert!(report.split_intersection_points().contains(&p(xmin, ymin)));
        prop_assert!(report.split_intersection_points().contains(&p(xmax, ymin)));
        prop_assert_eq!(report.split_output_segment_count(), Some(2));
        prop_assert_eq!(report.split_blocker_first_source_segment_index(), None);
        prop_assert_eq!(report.split_blocker_second_source_segment_index(), None);
        prop_assert_eq!(report.endpoint_graph_endpoint_count(), Some(4));
        prop_assert_eq!(report.endpoint_graph_structural_bucket_count(), Some(2));
        prop_assert_eq!(
            report.endpoint_graph_structural_singleton_bucket_count(),
            Some(0)
        );
        prop_assert_eq!(report.endpoint_graph_dangling_endpoint_count(), Some(0));
        prop_assert_eq!(report.endpoint_graph_branch_endpoint_count(), Some(0));
        prop_assert_eq!(report.endpoint_graph_blocker_arranged_segment_index(), None);
        prop_assert_eq!(report.endpoint_graph_blocker_endpoint(), None);
        prop_assert_eq!(
            report.attempted_endpoint_connection_count(),
            report.exact_endpoint_connection_count()
                + report.disconnected_endpoint_connection_count()
                + report.unresolved_endpoint_connection_count()
        );
        prop_assert!(report.exact_endpoint_connection_count() >= 2);
        prop_assert_eq!(report.unresolved_endpoint_connection_count(), 0);
        prop_assert!(report.reversed_source_segment_count() <= 1);
        prop_assert_eq!(report.arranged_source_reports().len(), 2);
        prop_assert!(report
            .arranged_source_reports()
            .iter()
            .all(|source| source.status().is_native_exact()));
        prop_assert_eq!(report.source_reports().len(), 2);
        prop_assert!(report
            .source_reports()
            .iter()
            .all(|source| source.status().is_native_exact()));
        prop_assert_eq!(report.output_ring_count(), Some(1));
        prop_assert_eq!(report.output_boundary_segment_count(), Some(2));
        prop_assert_eq!(report.blocker(), None);

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
