use crate::CurveCertainty;
use crate::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveContext, CurveError, CurveRegion2,
    CurveRegionArrangement2, CurveRegionArrangementStage2, CurveString2, FillRule,
    FiniteProjectionOptions, Real, RegionPointLocation, Segment2, SegmentKindCounts,
    UncertaintyReason, finite_polyline_vertex_centroid, finite_ring_signed_area,
    try_finite_polyline_vertex_centroid, try_finite_ring_signed_area,
};
use proptest::prelude::*;

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> crate::Point2 {
    crate::Point2::new(s(x), s(y))
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

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> crate::LineSeg2 {
    crate::LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap()
}

fn arc_bulge(start_x: i32, start_y: i32, end_x: i32, end_y: i32, bulge: i32) -> CircularArc2 {
    CircularArc2::from_bulge(p(start_x, start_y), p(end_x, end_y), s(bulge)).unwrap()
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
    CurveRegion2::try_from_native_contours(material, holes, &policy())
        .unwrap()
        .into_value()
}

fn classify(region: &CurveRegion2, point: &crate::Point2) -> Classification<RegionPointLocation> {
    region
        .classify_point(point, &policy())
        .unwrap()
        .into_value()
}

fn depth(region: &CurveRegion2, point: &crate::Point2) -> Classification<i32> {
    region.signed_depth(point, &policy()).unwrap().into_value()
}

fn filled_area(region: &CurveRegion2) -> Classification<Option<Real>> {
    region.filled_area(&policy()).unwrap().into_value()
}

fn arrange_lines(segments: Vec<crate::LineSeg2>, fill_rule: FillRule) -> CurveRegionArrangement2 {
    CurveRegion2::arrange_unordered_segments(
        segments.into_iter().map(Segment2::Line).collect(),
        fill_rule,
        &policy(),
    )
    .unwrap()
    .into_value()
}

fn arrange_lines_borrowed(
    segments: &[crate::LineSeg2],
    fill_rule: FillRule,
) -> CurveRegionArrangement2 {
    let segments = segments
        .iter()
        .cloned()
        .map(Segment2::Line)
        .collect::<Vec<_>>();
    CurveRegion2::arrange_unordered_segments_borrowed(&segments, fill_rule, &policy())
        .unwrap()
        .into_value()
}

fn arrange_segments(segments: Vec<Segment2>, fill_rule: FillRule) -> CurveRegionArrangement2 {
    CurveRegion2::arrange_unordered_segments(segments, fill_rule, &policy())
        .unwrap()
        .into_value()
}

fn arrange_segments_borrowed(
    segments: &[Segment2],
    fill_rule: FillRule,
) -> CurveRegionArrangement2 {
    CurveRegion2::arrange_unordered_segments_borrowed(segments, fill_rule, &policy())
        .unwrap()
        .into_value()
}

#[test]
fn empty_region_classifies_everything_outside() {
    let region = CurveRegion2::empty();
    assert!(region.is_empty());
    assert_eq!(depth(&region, &p(0, 0)), Classification::Decided(0));
    assert_eq!(
        classify(&region, &p(0, 0)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn material_contour_classifies_inside_outside_and_boundary() {
    let region = region(vec![rectangle(0, 0, 10, 10)], Vec::new());
    assert_eq!(
        classify(&region, &p(1, 1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(&region, &p(11, 1)),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        classify(&region, &p(10, 5)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn sparse_region_classification_and_hole_depth_are_exact() {
    let region = region(
        vec![
            rectangle(0, 0, 10, 10),
            rectangle(20, 20, 24, 24),
            rectangle(40, 40, 44, 44),
        ],
        vec![rectangle(3, 3, 7, 7)],
    );
    assert_eq!(depth(&region, &p(21, 21)), Classification::Decided(1));
    assert_eq!(depth(&region, &p(5, 5)), Classification::Decided(0));
    assert_eq!(depth(&region, &p(100, 100)), Classification::Decided(0));
    assert_eq!(
        classify(&region, &p(20, 22)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        classify(&region, &p(5, 5)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn material_island_inside_hole_adds_depth_back() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    for (point, expected_depth, expected_location) in [
        (p(1, 1), 1, RegionPointLocation::Inside),
        (p(3, 3), 0, RegionPointLocation::Outside),
        (p(5, 5), 1, RegionPointLocation::Inside),
    ] {
        assert_eq!(
            depth(&region, &point),
            Classification::Decided(expected_depth)
        );
        assert_eq!(
            classify(&region, &point),
            Classification::Decided(expected_location)
        );
    }
    assert_eq!(
        depth(&region, &p(2, 5)),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        classify(&region, &p(2, 5)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn boundary_contour_nesting_assigns_disjoint_nested_roles() {
    let classified = CurveRegion2::try_from_native_boundary_contours(
        vec![rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let Classification::Decided(region) = classified else {
        panic!("nested contours should be decided: {classified:?}");
    };
    assert_eq!(
        region.loop_role_counts(&policy()).unwrap().into_value(),
        Classification::Decided((1, 1))
    );
    assert_eq!(
        classify(&region, &p(1, 1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(&region, &p(5, 5)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn borrowed_boundary_contours_use_the_same_authoritative_constructor() {
    let contours = vec![rectangle(0, 0, 5, 5), rectangle(1, 1, 3, 3)];
    let classified = CurveRegion2::try_from_native_boundary_contours_borrowed(&contours, &policy())
        .unwrap()
        .into_value();
    let Classification::Decided(region) = classified else {
        panic!("nested borrowed contours should be decided: {classified:?}");
    };
    assert_eq!(contours.len(), 2);
    assert_eq!(
        region.loop_role_counts(&policy()).unwrap().into_value(),
        Classification::Decided((1, 1))
    );
}

#[test]
fn boundary_contour_nesting_rejects_crossing_or_touching_loops() {
    for contours in [
        vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
        vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
    ] {
        assert_eq!(
            CurveRegion2::try_from_native_boundary_contours(contours, &policy())
                .unwrap()
                .into_value(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn unordered_lines_materialize_one_authoritative_region() {
    let built = arrange_lines(
        vec![
            line(0, 0, 4, 0),
            line(0, 4, 4, 4),
            line(0, 0, 0, 4),
            line(4, 0, 4, 4),
        ],
        FillRule::NonZero,
    );
    assert!(built.status().is_native_exact());
    assert_eq!(built.source_segment_count(), 4);
    assert_eq!(built.output_ring_count(), Some(1));
    assert_eq!(built.output_boundary_segment_count(), Some(4));
    let region = built.region().expect("rectangle should materialize");
    assert_eq!(
        classify(region, &p(2, 2)),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn borrowed_unordered_lines_and_segments_have_identical_semantics() {
    let lines = vec![
        line(0, 0, 4, 0),
        line(0, 4, 4, 4),
        line(0, 0, 0, 4),
        line(4, 0, 4, 4),
    ];
    let line_result = arrange_lines_borrowed(&lines, FillRule::NonZero);
    let segments = lines
        .iter()
        .cloned()
        .map(Segment2::Line)
        .collect::<Vec<_>>();
    let segment_result = arrange_segments_borrowed(&segments, FillRule::NonZero);
    assert_eq!(line_result.source_segment_count(), 4);
    assert_eq!(segment_result.source_segment_count(), 4);
    assert_eq!(line_result.stage(), segment_result.stage());
    assert_eq!(line_result.status(), segment_result.status());
    assert_eq!(line_result.blocker(), segment_result.blocker());
    assert_eq!(
        line_result.output_ring_count(),
        segment_result.output_ring_count()
    );
    assert_eq!(
        line_result.output_boundary_segment_count(),
        segment_result.output_boundary_segment_count()
    );
    assert_eq!(
        line_result.output_boundary_segment_kind_counts(),
        segment_result.output_boundary_segment_kind_counts()
    );
    assert_eq!(
        classify(line_result.region().unwrap(), &p(2, 2)),
        classify(segment_result.region().unwrap(), &p(2, 2))
    );
}

#[test]
fn unordered_open_lines_retain_a_boundary_blocker() {
    let built = arrange_lines(vec![line(0, 0, 1, 0), line(3, 0, 4, 0)], FillRule::NonZero);
    assert!(built.region().is_none());
    assert!(built.status().is_retained_evidence());
    assert_eq!(
        built.stage(),
        CurveRegionArrangementStage2::EndpointAssembly
    );
    assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn unordered_crossing_and_overlapping_lines_remain_explicit_blockers() {
    for lines in [
        vec![line(0, 0, 4, 4), line(0, 4, 4, 0)],
        vec![line(0, 0, 4, 0), line(2, 0, 6, 0)],
    ] {
        let built = arrange_lines(lines, FillRule::NonZero);
        assert!(built.region().is_none());
        assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
    }
}

#[test]
fn unordered_self_crossing_walk_uses_the_authoritative_curve_arrangement() {
    let source = vec![
        line(4, 4, 0, 4),
        line(4, 0, 0, 0),
        line(0, 4, 4, 0),
        line(0, 0, 4, 4),
    ];
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let built = arrange_lines(source.clone(), fill_rule);
        assert!(built.status().is_native_exact());
        assert_eq!(
            built.stage(),
            CurveRegionArrangementStage2::CurveArrangement
        );
        assert_eq!(built.output_ring_count(), Some(2));
        let region = built
            .region()
            .expect("the exact self-crossing walk should regularize");
        for (point, expected) in [
            (p(2, 3), RegionPointLocation::Inside),
            (p(2, 1), RegionPointLocation::Inside),
            (p(0, 2), RegionPointLocation::Outside),
        ] {
            assert_eq!(classify(region, &point), Classification::Decided(expected));
        }
    }
}

#[test]
fn unordered_crossing_walks_are_regularized_by_global_parity() {
    let source = vec![
        line(4, 4, 0, 4),
        line(0, 0, 4, 0),
        line(0, 4, 0, 0),
        line(4, 0, 4, 4),
        line(6, 3, 2, 3),
        line(2, -1, 6, -1),
        line(2, 3, 2, -1),
        line(6, -1, 6, 3),
    ];
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let built = arrange_lines(source.clone(), fill_rule);
        let region = built
            .region()
            .expect("crossing closed walks should reach unified face selection");
        for (point, expected) in [
            (p(1, 1), RegionPointLocation::Inside),
            (p(3, 1), RegionPointLocation::Outside),
            (p(5, 1), RegionPointLocation::Inside),
            (p(8, 1), RegionPointLocation::Outside),
        ] {
            assert_eq!(classify(region, &point), Classification::Decided(expected));
        }
    }
}

#[test]
fn unordered_single_full_circle_is_a_closed_walk() {
    let start = p(2, 0);
    let circle = CircularArc2::try_from_center(start.clone(), start, p(0, 0), false).unwrap();
    let built = arrange_segments(vec![Segment2::Arc(circle)], FillRule::NonZero);
    let region = built
        .region()
        .expect("a native full circle should regularize exactly");
    assert_eq!(built.output_ring_count(), Some(1));
    assert_eq!(
        classify(region, &p(0, 0)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(region, &p(3, 0)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unordered_line_arc_segments_materialize_without_exposing_native_ownership() {
    let built = arrange_segments(
        vec![
            Segment2::Line(line(4, 0, 0, 0)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
    );
    assert!(built.status().is_native_exact());
    assert_eq!(built.source_segment_count(), 2);
    assert_eq!(
        built.output_boundary_segment_kind_counts(),
        None,
        "the unified arrangement may retain the circular span as an exact conic"
    );
    let region = built.region().expect("semicircle should materialize");
    assert_eq!(
        classify(region, &p(2, -1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        region.structural_facts(&policy()).unwrap().into_value(),
        Classification::Uncertain(UncertaintyReason::Unsupported),
        "the exact conic result does not pretend to be a native line/arc carrier"
    );
}

#[test]
fn native_overlap_regularizes_empty_and_open_crossings_remain_blocked() {
    let coincident = arrange_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
    );
    assert!(coincident.status().is_native_exact());
    assert!(
        coincident
            .region()
            .expect("oppositely traversed coincident arcs have decided topology")
            .is_empty()
    );
    assert_eq!(coincident.output_ring_count(), Some(0));

    let cases = [
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(2, -3, 2, 1)),
        ],
        vec![
            Segment2::Arc(
                CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap(),
            ),
            Segment2::Arc(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap()),
        ],
    ];
    for segments in cases {
        let built = arrange_segments(segments, FillRule::NonZero);
        assert!(built.region().is_none());
        assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));
    }
}

#[test]
fn contour_profiles_group_holes_with_their_exact_material_owner() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 0, 30, 10)],
        vec![rectangle(2, 2, 4, 4), rectangle(22, 2, 24, 4)],
    );
    let profiles = region.boundary_profiles(&policy()).unwrap().into_value();
    let Classification::Decided(profiles) = profiles else {
        panic!("profile ownership should be decided: {profiles:?}");
    };
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().len() == 1));
    assert_eq!(profiles[0].material_loop_index(), 0);
    assert_eq!(profiles[0].hole_loop_indices(), &[2]);
    assert_eq!(profiles[1].material_loop_index(), 1);
    assert_eq!(profiles[1].hole_loop_indices(), &[3]);
}

#[test]
fn contour_profiles_reject_holes_without_a_material_owner() {
    let region = region(Vec::new(), vec![rectangle(2, 2, 4, 4)]);
    assert_eq!(
        region.boundary_profiles(&policy()).unwrap().into_value(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn contour_projection_closes_finite_ring_without_owning_topology() {
    let ring = rectangle(0, 0, 10, 10)
        .project_to_finite_ring(&FiniteProjectionOptions::try_new(0.01).unwrap())
        .unwrap();
    assert!(ring.is_closed());
    assert_eq!(ring.points().first(), ring.points().last());
    assert_eq!(ring.points().len(), 5);
    assert_eq!(ring.try_signed_ring_area().unwrap(), 100.0);
    assert_eq!(finite_ring_signed_area(ring.points()), 100.0);
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
    assert!(finite_ring_signed_area(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]).is_nan());
    assert!(finite_polyline_vertex_centroid(&[[0.0, 0.0], [f64::INFINITY, 1.0]]).is_some());
}

#[test]
fn curve_string_projection_subdivides_arcs_and_keeps_exact_endpoints() {
    let start = crate::Point2::new(Real::one(), Real::zero());
    let end = crate::Point2::new(-Real::one(), Real::zero());
    let center = crate::Point2::new(Real::zero(), Real::zero());
    let arc = CircularArc2::try_from_center(start, end.clone(), center, false).unwrap();
    let tail = crate::LineSeg2::try_new(end, p(-2, 0)).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc), Segment2::Line(tail)]).unwrap();
    let polyline = curve
        .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.05).unwrap())
        .unwrap();
    assert!(!polyline.is_closed());
    assert!(polyline.points().len() > 3);
    assert_eq!(polyline.points().first(), Some(&[1.0, 0.0]));
    assert_eq!(polyline.points().last(), Some(&[-2.0, 0.0]));
}

#[test]
fn curve_string_projection_rejects_nonfinite_arc_samples() {
    let huge = Real::try_from(1.1e308).unwrap();
    let arc = CircularArc2::try_from_center(
        crate::Point2::new(Real::zero(), Real::zero()),
        crate::Point2::new(huge.clone(), huge.clone()),
        crate::Point2::new(huge, Real::zero()),
        false,
    )
    .unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();
    assert_eq!(
        curve
            .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.01).unwrap())
            .unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
}

#[test]
fn unified_finite_profiles_preserve_material_hole_bins_and_ownership() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 0, 30, 10)],
        vec![rectangle(2, 2, 4, 4), rectangle(22, 2, 24, 4)],
    );
    let profiles = region
        .project_to_finite_profiles_exact(
            &FiniteProjectionOptions::try_new(0.01).unwrap(),
            &policy(),
        )
        .unwrap()
        .into_value();
    let Classification::Decided(profiles) = profiles else {
        panic!("finite profile ownership should be decided: {profiles:?}");
    };
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().len() == 1));
    assert_eq!(profiles[0].material().points()[0], [0.0, 0.0]);
    assert_eq!(profiles[0].holes()[0].points()[0], [2.0, 2.0]);
    assert_eq!(profiles[1].material().points()[0], [20.0, 0.0]);
    assert_eq!(profiles[1].holes()[0].points()[0], [22.0, 2.0]);
    assert_eq!(profiles[0].try_projected_filled_area().unwrap(), 96.0);
    assert_eq!(profiles[1].try_projected_filled_area().unwrap(), 96.0);
}

#[test]
fn similarity_transform_preserves_arcs_and_reflection_flips_orientation() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();
    let transform =
        crate::Similarity2::try_from_f64_affine(0.0, -1.0, 1.0, 0.0, 3.0, -2.0, 1e-9).unwrap();
    let transformed = curve.transform_similarity(&transform).unwrap();
    let [Segment2::Arc(transformed_arc)] = transformed.segments() else {
        panic!("similarity should preserve an arc");
    };
    assert_eq!(transformed_arc.start(), &crate::Point2::from_values(3, -1));
    assert_eq!(transformed_arc.end(), &crate::Point2::from_values(2, -2));
    assert!(!transformed_arc.is_clockwise());

    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(1, 0), Real::one()),
        BulgeVertex2::new(p(-1, 0), Real::zero()),
    ])
    .unwrap();
    let reflection =
        crate::Similarity2::try_from_f64_affine(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1e-9).unwrap();
    let transformed = contour.transform_similarity(&reflection).unwrap();
    let Segment2::Arc(arc) = &transformed.segments()[0] else {
        panic!("reflection should retain an arc");
    };
    assert!(arc.is_clockwise());
    assert!(reflection.reverses_orientation());
    assert_eq!(
        crate::Similarity2::try_from_f64_affine(1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1e-9),
        Err(CurveError::InvalidSimilarityTransform)
    );
}

#[test]
fn filled_area_uses_roles_not_orientation_and_counts_nested_islands() {
    let simple = region(
        vec![reversed_rectangle(0, 0, 10, 10)],
        vec![rectangle(3, 3, 7, 7)],
    );
    assert_eq!(
        filled_area(&simple),
        Classification::Decided(Some(Real::from(84_i8)))
    );
    let nested = region(
        vec![rectangle(0, 0, 10, 10), reversed_rectangle(4, 4, 6, 6)],
        vec![reversed_rectangle(2, 2, 8, 8)],
    );
    assert_eq!(
        filled_area(&nested),
        Classification::Decided(Some(Real::from(68_i8)))
    );
}

#[test]
fn filled_area_is_exact_for_center_defined_circle() {
    let top = CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let bottom = CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap();
    let contour = Contour2::try_new(vec![Segment2::Arc(top), Segment2::Arc(bottom)]).unwrap();
    assert_eq!(
        filled_area(&region(vec![contour], Vec::new())),
        Classification::Decided(Some(Real::pi()))
    );
}

#[test]
fn strict_and_approximate_512_share_the_unified_policy_terminal() {
    let region = region(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    for context in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            region
                .classify_point(&p(1, 1), &context)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            region.filled_area(&context).unwrap().into_value(),
            Classification::Decided(Some(Real::from(84_i8)))
        );
    }
}

#[test]
fn unordered_native_arrangement_obeys_the_approximate_512_terminal() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let first_sum = sine;
    let second_sum = first_sum.clone() + unresolved_zero;
    let lines = vec![
        crate::LineSeg2::try_new(
            crate::Point2::new(Real::zero(), Real::zero()),
            crate::Point2::new(first_sum.clone(), Real::zero()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(second_sum.clone(), Real::zero()),
            crate::Point2::new(first_sum.clone(), Real::one()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(second_sum, Real::one()),
            crate::Point2::new(Real::zero(), Real::one()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(Real::zero(), Real::one()),
            crate::Point2::new(Real::zero(), Real::zero()),
        )
        .unwrap(),
    ];

    let segments = lines.into_iter().map(Segment2::Line).collect::<Vec<_>>();
    let strict = CurveRegion2::arrange_unordered_segments_borrowed(
        &segments,
        FillRule::NonZero,
        &CurveContext::STRICT,
    )
    .unwrap();
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(strict.value.region().is_none());
    assert!(strict.value.status().is_retained_evidence());

    let approximate = CurveRegion2::arrange_unordered_segments_borrowed(
        &segments,
        FillRule::NonZero,
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(approximate.value.region().is_some());
    assert!(approximate.value.status().is_native_exact());

    let approximate_owned = CurveRegion2::arrange_unordered_segments(
        segments,
        FillRule::NonZero,
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(
        approximate_owned.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(approximate_owned.value.region().is_some());
}

proptest! {
    #[test]
    fn generated_unordered_line_rectangles_build_unified_regions(
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
        for (index, segment) in lines.iter_mut().enumerate() {
            if reverse_mask & (1 << index) != 0 {
                *segment = segment.reversed();
            }
        }
        match order_variant {
            0 => {}
            1 => lines.swap(0, 2),
            2 => lines.rotate_left(1),
            _ => lines.reverse(),
        }
        let built = arrange_lines(lines, FillRule::NonZero);
        prop_assert!(built.status().is_native_exact());
        prop_assert_eq!(built.source_segment_count(), 4);
        prop_assert_eq!(built.output_ring_count(), Some(1));
        prop_assert_eq!(built.output_boundary_segment_count(), Some(4));
        prop_assert_eq!(
            classify(built.region().unwrap(), &p(xmin + 1, ymin + 1)),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_unordered_line_arc_semicircles_build_unified_regions(
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
                *segment = segment.reversed();
            }
        }
        if order_variant == 1 {
            segments.swap(0, 1);
        }
        let built = arrange_segments(segments, FillRule::NonZero);
        prop_assert!(built.status().is_native_exact());
        prop_assert_eq!(built.source_segment_count(), 2);
        prop_assert_eq!(
            built.output_boundary_segment_kind_counts(),
            None
        );
        prop_assert_eq!(
            classify(
                built.region().expect("semicircle should materialize"),
                &p(xmin + width / 2, inside_y),
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_rectangle_hole_area_uses_roles_not_orientation(
        width in 3_i32..80,
        height in 3_i32..80,
        hole_width in 1_i32..20,
        hole_height in 1_i32..20,
    ) {
        let hole_width = hole_width.min(width - 2);
        let hole_height = hole_height.min(height - 2);
        let region = region(
            vec![reversed_rectangle(0, 0, width, height)],
            vec![reversed_rectangle(1, 1, 1 + hole_width, 1 + hole_height)],
        );
        prop_assert_eq!(
            filled_area(&region),
            Classification::Decided(Some(Real::from(
                width * height - hole_width * hole_height,
            )))
        );
    }
}

#[test]
fn batched_classifier_and_structural_facts_use_the_unified_surface() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    let Classification::Decided(facts) = region.structural_facts(&policy()).unwrap().into_value()
    else {
        panic!("native specialization should expose structural facts");
    };
    assert!(facts.has_decided_region_box);
    assert_eq!(facts.material_contour_count, 2);
    assert_eq!(facts.hole_contour_count, 1);
    assert_eq!(
        facts.segment_kinds,
        SegmentKindCounts { lines: 12, arcs: 0 }
    );

    let points = [p(1, 1), p(3, 3), p(5, 5), p(11, 1), p(100, 100), p(2, 5)];
    let batched = region
        .classify_points(&points, &policy())
        .unwrap()
        .into_value();
    assert_eq!(
        batched,
        points
            .iter()
            .map(|point| classify(&region, point))
            .collect::<Vec<_>>()
    );
}
