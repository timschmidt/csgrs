use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    FillRule, FiniteProjectionOptions, Real, Region2, RegionBoundaryContourBuildStage2,
    RegionBoundaryContourRole2, RegionPointLocation, RegionView2, Segment2, UncertaintyReason,
    finite_polyline_vertex_centroid, finite_ring_signed_area, try_finite_polyline_vertex_centroid,
    try_finite_ring_signed_area,
};
use proptest::prelude::*;

fn s(value: i32) -> Real {
    value.into()
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
        CurveString2::import_finite_line_string(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]]).unwrap();

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
        CurveString2::import_finite_line_string(&[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]]).unwrap();

    assert_eq!(import.curve_string().len(), 1);
}

#[test]
fn finite_ring_import_accepts_repeated_closing_point_without_sample_ownership() {
    let contour =
        Contour2::from_finite_ring(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0], [0.0, 0.0]])
            .unwrap();
    let import =
        Contour2::import_finite_ring(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0], [0.0, 0.0]])
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
    assert_eq!(prepared.prepared_hole_segment_count(), 4);
    assert_eq!(prepared.prepared_segment_count(), 12);
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
