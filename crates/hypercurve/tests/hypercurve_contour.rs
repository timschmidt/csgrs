use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourChamferStage2, ContourClosureStage2,
    ContourFilletStage2, ContourLineMergeStage2, ContourPointLocation, CurveError, CurvePolicy,
    CurveString2, CurveStringChamferInputPath2, CurveStringFilletInputPath2, FillRule, Real,
    Region2, RegionPointLocation, Segment2, SegmentKind, SegmentKindCounts, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn assert_line(segment: &Segment2, start: hypercurve::Point2, end: hypercurve::Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap()
}

fn rotated_rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(0, 0, 0),
        vertex(4, 0, 0),
    ])
    .unwrap()
}

fn reversed_rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(0, 4, 0),
        vertex(4, 4, 0),
        vertex(4, 0, 0),
    ])
    .unwrap()
}

#[test]
fn contour_builds_closed_bulge_loop() {
    let contour = rectangle();

    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert!(
        contour
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
}

#[test]
fn contour_rejects_open_segment_chain() {
    let segments = vec![
        vertex(0, 0, 0).segment_to(&vertex(1, 0, 0)).unwrap(),
        vertex(1, 0, 0).segment_to(&vertex(2, 0, 0)).unwrap(),
    ];

    let err = Contour2::try_new(segments).expect_err("open chain is not a contour");
    assert_eq!(err, CurveError::DisconnectedCurveString);
}

#[test]
fn contour_closure_report_materializes_closed_curve_string() {
    let curve = CurveString2::try_new(vec![
        vertex(0, 0, 0).segment_to(&vertex(4, 0, 0)).unwrap(),
        vertex(4, 0, 0).segment_to(&vertex(4, 4, 0)).unwrap(),
        vertex(4, 4, 0).segment_to(&vertex(0, 4, 0)).unwrap(),
        vertex(0, 4, 0).segment_to(&vertex(0, 0, 0)).unwrap(),
    ])
    .unwrap();

    let closed = Contour2::from_curve_string_with_report(curve, FillRule::EvenOdd).unwrap();

    assert!(closed.report().status().is_native_exact());
    assert_eq!(
        closed.report().stage(),
        ContourClosureStage2::ContourMaterialization
    );
    assert_eq!(closed.report().source_segment_count(), 4);
    assert_eq!(closed.report().source_start_point(), &p(0, 0));
    assert_eq!(closed.report().source_end_point(), &p(0, 0));
    assert_eq!(closed.report().endpoint_distance_squared(), &s(0));
    assert_eq!(closed.report().fill_rule(), FillRule::EvenOdd);
    assert_eq!(closed.report().blocker(), None);
    let contour = closed.contour().unwrap();
    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::EvenOdd);
}

#[test]
fn contour_closure_report_blocks_certified_open_curve_string() {
    let curve = CurveString2::try_new(vec![
        vertex(0, 0, 0).segment_to(&vertex(1, 0, 0)).unwrap(),
        vertex(1, 0, 0).segment_to(&vertex(2, 0, 0)).unwrap(),
    ])
    .unwrap();

    let closed = Contour2::from_curve_string_with_report(curve, FillRule::NonZero).unwrap();

    assert!(closed.contour().is_none());
    assert!(closed.report().status().is_retained_evidence());
    assert_eq!(
        closed.report().stage(),
        ContourClosureStage2::EndpointValidation
    );
    assert_eq!(closed.report().source_segment_count(), 2);
    assert_eq!(closed.report().source_start_point(), &p(0, 0));
    assert_eq!(closed.report().source_end_point(), &p(2, 0));
    assert_eq!(closed.report().endpoint_distance_squared(), &s(4));
    assert_eq!(closed.report().fill_rule(), FillRule::NonZero);
    assert_eq!(closed.report().blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn closed_curve_string_contour_feeds_boundary_region_report() {
    let curve = CurveString2::try_new(vec![
        vertex(0, 0, 0).segment_to(&vertex(4, 0, 0)).unwrap(),
        vertex(4, 0, 0).segment_to(&vertex(4, 4, 0)).unwrap(),
        vertex(4, 4, 0).segment_to(&vertex(0, 4, 0)).unwrap(),
        vertex(0, 4, 0).segment_to(&vertex(0, 0, 0)).unwrap(),
    ])
    .unwrap();
    let contour = Contour2::from_curve_string_with_report(curve, FillRule::NonZero)
        .unwrap()
        .into_contour()
        .unwrap();

    let built = Region2::from_boundary_contours_with_report(vec![contour], &policy()).unwrap();

    assert!(built.report().status().is_native_exact());
    assert_eq!(built.report().source_contour_count(), 1);
    assert_eq!(built.report().material_contour_count(), Some(1));
    assert_eq!(built.report().hole_contour_count(), Some(0));
    assert_eq!(
        built.region().unwrap().classify_point(&p(2, 2), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn contour_merge_adjacent_collinear_lines_reports_source_runs() {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap();

    let merged = contour.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(
        merged.report().stage(),
        ContourLineMergeStage2::ContourMaterialization
    );
    assert_eq!(merged.report().source_segment_count(), 5);
    assert_eq!(merged.report().output_segment_count(), Some(4));
    assert_eq!(merged.report().adjacent_pair_count(), 5);
    assert_eq!(merged.report().merged_pair_count(), 1);
    assert_eq!(merged.report().preserved_pair_count(), 4);
    assert_eq!(merged.report().fill_rule(), FillRule::NonZero);
    assert_eq!(merged.report().spans().len(), 4);
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
    assert_eq!(merged.report().spans()[0].output_end_point(), &p(4, 0));
    let contour = merged
        .contour()
        .expect("certified contour line merge should materialize");
    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert_line(&contour.segments()[0], p(0, 0), p(4, 0));
}

#[test]
fn contour_line_merge_span_reports_preserve_mixed_segment_kinds() {
    let contour = Contour2::try_new(vec![
        Segment2::Line(hypercurve::LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
        Segment2::Arc(hypercurve::CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap()),
        Segment2::Line(hypercurve::LineSeg2::try_new(p(3, 0), p(0, 0)).unwrap()),
    ])
    .unwrap();

    let merged = contour.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(merged.report().merged_pair_count(), 0);
    assert_eq!(merged.report().spans().len(), 3);
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
}

#[test]
fn contour_merge_adjacent_collinear_lines_merges_wraparound_run() {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(2, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(0, 0, 0),
    ])
    .unwrap();

    let merged = contour.merge_adjacent_collinear_lines(&policy()).unwrap();

    assert!(merged.report().status().is_native_exact());
    assert_eq!(
        merged.report().stage(),
        ContourLineMergeStage2::ContourMaterialization
    );
    assert_eq!(merged.report().source_segment_count(), 5);
    assert_eq!(merged.report().output_segment_count(), Some(4));
    assert_eq!(merged.report().adjacent_pair_count(), 5);
    assert_eq!(merged.report().merged_pair_count(), 1);
    assert_eq!(merged.report().preserved_pair_count(), 4);
    assert_eq!(merged.report().spans()[0].source_segment_indices(), &[4, 0]);
    assert_eq!(merged.report().spans()[0].output_segment_index(), 0);
    assert_eq!(merged.report().spans()[0].output_start_point(), &p(0, 0));
    assert_eq!(merged.report().spans()[0].output_end_point(), &p(4, 0));
    let contour = merged
        .contour()
        .expect("certified wraparound contour line merge should materialize");
    assert_eq!(contour.len(), 4);
    assert_line(&contour.segments()[0], p(0, 0), p(4, 0));
    assert_line(&contour.segments()[1], p(4, 0), p(4, 4));
    assert_line(&contour.segments()[3], p(0, 4), p(0, 0));
}

#[test]
fn contour_chamfer_line_line_vertex_materializes_closed_contour() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_parameters_with_report(1, q(3, 4), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::ContourMaterialization
    );
    assert_eq!(chamfer.report().vertex_index(), 1);
    assert_eq!(
        chamfer.report().curve_string_report().input_path(),
        CurveStringChamferInputPath2::Parameters
    );
    assert_eq!(chamfer.report().source_segment_count(), 4);
    assert_eq!(
        chamfer.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(chamfer.report().output_segment_count(), Some(5));
    assert_eq!(
        chamfer.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 5, arcs: 0 })
    );
    assert_eq!(chamfer.report().previous_segment_index(), 0);
    assert_eq!(chamfer.report().next_segment_index(), 1);
    assert_eq!(chamfer.report().previous_trim().param(), &q(3, 4));
    assert_eq!(chamfer.report().next_trim().param(), &q(1, 4));
    assert_eq!(chamfer.report().previous_cut_point(), Some(&p(3, 0)));
    assert_eq!(chamfer.report().next_cut_point(), Some(&p(4, 1)));
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .chamfer_segment_start_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .chamfer_segment_end_point(),
        Some(&p(4, 1))
    );
    assert_eq!(chamfer.report().chamfer_segment_index(), Some(1));
    assert_eq!(
        chamfer.report().output_segment_count(),
        chamfer
            .report()
            .curve_string_report()
            .output_segment_count()
    );
    assert_eq!(chamfer.report().fill_rule(), FillRule::NonZero);
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .chamfer_segment_index(),
        Some(1)
    );
    let contour = chamfer
        .contour()
        .expect("line-line contour chamfer should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert_eq!(contour.segments()[0].start(), &p(0, 0));
    assert_eq!(contour.segments()[0].end(), &p(3, 0));
    assert_eq!(contour.segments()[1].start(), &p(3, 0));
    assert_eq!(contour.segments()[1].end(), &p(4, 1));
    assert_eq!(contour.segments()[4].end(), &p(0, 0));
}

#[test]
fn contour_chamfer_preserves_fill_rule() {
    let contour = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    let chamfer = contour
        .chamfer_line_line_vertex_by_parameters(1, q(3, 4), q(1, 4), &policy())
        .unwrap();

    assert_eq!(chamfer.report().fill_rule(), FillRule::EvenOdd);
    assert_eq!(chamfer.contour().unwrap().fill_rule(), FillRule::EvenOdd);
}

#[test]
fn contour_chamfer_reports_boundary_parameters() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_parameters(1, s(1), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.contour().is_none());
    assert!(chamfer.report().status().is_retained_evidence());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::CurveStringEdit
    );
    assert_eq!(
        chamfer.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .chamfer_segment_index(),
        None
    );
    assert_eq!(chamfer.report().output_segment_count(), None);
    assert_eq!(
        chamfer.report().output_segment_count(),
        chamfer
            .report()
            .curve_string_report()
            .output_segment_count()
    );
}

#[test]
fn contour_chamfer_line_line_vertex_by_points_materializes_closed_contour() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_points_with_report(1, &p(3, 0), &p(4, 1), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::ContourMaterialization
    );
    assert_eq!(chamfer.report().vertex_index(), 1);
    assert_eq!(
        chamfer.report().curve_string_report().input_path(),
        CurveStringChamferInputPath2::Points
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .previous_trim()
            .param(),
        &q(3, 4)
    );
    assert_eq!(
        chamfer.report().curve_string_report().next_trim().param(),
        &q(1, 4)
    );
    assert_eq!(chamfer.report().output_segment_count(), Some(5));
    let contour = chamfer
        .contour()
        .expect("point-bearing contour chamfer should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].end(), &p(3, 0));
    assert_eq!(contour.segments()[1].start(), &p(3, 0));
    assert_eq!(contour.segments()[1].end(), &p(4, 1));
    assert_eq!(contour.segments()[2].start(), &p(4, 1));
    assert_eq!(contour.segments()[4].end(), &p(0, 0));
}

#[test]
fn contour_chamfer_line_line_vertex_by_points_reports_off_segment_boundary() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_points(1, &p(5, 0), &p(4, 1), &policy())
        .unwrap();

    assert!(chamfer.contour().is_none());
    assert!(chamfer.report().status().is_retained_evidence());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::CurveStringEdit
    );
    assert_eq!(
        chamfer.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .chamfer_segment_index(),
        None
    );
    assert_eq!(chamfer.report().output_segment_count(), None);
}

#[test]
fn contour_chamfer_line_line_wraparound_vertex_materializes_closed_contour() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_parameters(0, q(3, 4), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::ContourMaterialization
    );
    assert_eq!(chamfer.report().vertex_index(), 0);
    assert_eq!(chamfer.report().source_segment_count(), 4);
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .previous_segment_index(),
        3
    );
    assert_eq!(
        chamfer.report().curve_string_report().next_segment_index(),
        0
    );
    assert_eq!(chamfer.report().output_segment_count(), Some(5));
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .previous_trim()
            .segment_index(),
        3
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .next_trim()
            .segment_index(),
        0
    );
    let segment_reports = chamfer.report().curve_string_report().segment_reports();
    assert_eq!(segment_reports.len(), 2);
    assert_eq!(segment_reports[0].source_segment_index(), 3);
    assert_eq!(segment_reports[1].source_segment_index(), 0);

    let contour = chamfer
        .contour()
        .expect("wraparound contour chamfer should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].start(), &p(0, 4));
    assert_eq!(contour.segments()[0].end(), &p(0, 1));
    assert_eq!(contour.segments()[1].start(), &p(0, 1));
    assert_eq!(contour.segments()[1].end(), &p(1, 0));
    assert_eq!(contour.segments()[2].start(), &p(1, 0));
    assert_eq!(contour.segments()[4].end(), &p(0, 4));
}

#[test]
fn contour_chamfer_line_line_wraparound_vertex_by_points_materializes_closed_contour() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_points(0, &p(0, 1), &p(1, 0), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(
        chamfer.report().stage(),
        ContourChamferStage2::ContourMaterialization
    );
    assert_eq!(chamfer.report().vertex_index(), 0);
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .previous_trim()
            .param(),
        &q(3, 4)
    );
    assert_eq!(
        chamfer.report().curve_string_report().next_trim().param(),
        &q(1, 4)
    );
    assert_eq!(
        chamfer
            .report()
            .curve_string_report()
            .previous_segment_index(),
        3
    );
    assert_eq!(
        chamfer.report().curve_string_report().next_segment_index(),
        0
    );

    let contour = chamfer
        .contour()
        .expect("point-bearing wraparound contour chamfer should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].end(), &p(0, 1));
    assert_eq!(contour.segments()[1].start(), &p(0, 1));
    assert_eq!(contour.segments()[1].end(), &p(1, 0));
    assert_eq!(contour.segments()[2].start(), &p(1, 0));
}

#[test]
fn contour_chamfer_rejects_out_of_range_vertex() {
    let contour = rectangle();

    assert_eq!(
        contour
            .chamfer_line_line_vertex_by_parameters(4, q(3, 4), q(1, 4), &policy())
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
}

#[test]
fn contour_fillet_line_line_vertex_materializes_closed_contour() {
    let contour = rectangle();

    let fillet = contour
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
        fillet.report().stage(),
        ContourFilletStage2::ContourMaterialization
    );
    assert_eq!(fillet.report().vertex_index(), 1);
    assert_eq!(
        fillet.report().curve_string_report().input_path(),
        CurveStringFilletInputPath2::Points
    );
    assert_eq!(fillet.report().source_segment_count(), 4);
    assert_eq!(fillet.report().output_segment_count(), Some(5));
    assert_eq!(fillet.report().previous_segment_index(), 0);
    assert_eq!(fillet.report().next_segment_index(), 1);
    assert_eq!(fillet.report().previous_trim().param(), &q(3, 4));
    assert_eq!(fillet.report().next_trim().param(), &q(1, 4));
    assert_eq!(fillet.report().previous_tangent_point(), Some(&p(3, 0)));
    assert_eq!(fillet.report().next_tangent_point(), Some(&p(4, 1)));
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .fillet_segment_start_point(),
        Some(&p(3, 0))
    );
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .fillet_segment_end_point(),
        Some(&p(4, 1))
    );
    assert_eq!(fillet.report().center(), Some(&p(3, 1)));
    assert_eq!(fillet.report().radius_squared(), Some(&s(1)));
    assert_eq!(fillet.report().fillet_segment_index(), Some(1));
    assert_eq!(
        fillet.report().output_segment_count(),
        fillet.report().curve_string_report().output_segment_count()
    );
    assert_eq!(fillet.report().fill_rule(), FillRule::NonZero);
    assert_eq!(
        fillet.report().curve_string_report().fillet_segment_index(),
        Some(1)
    );
    assert_eq!(fillet.report().output_segment_count(), Some(5));

    let contour = fillet
        .contour()
        .expect("line-line contour fillet should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert_eq!(contour.segments()[0].start(), &p(0, 0));
    assert_eq!(contour.segments()[0].end(), &p(3, 0));
    let Segment2::Arc(arc) = &contour.segments()[1] else {
        panic!("fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(3, 0));
    assert_eq!(arc.end(), &p(4, 1));
    assert_eq!(arc.center(), &p(3, 1));
    assert_eq!(arc.radius_squared_ref(), &s(1));
    assert!(!arc.is_clockwise());
    assert_eq!(contour.segments()[2].start(), &p(4, 1));
    assert_eq!(contour.segments()[4].end(), &p(0, 0));
}

#[test]
fn contour_fillet_line_line_vertex_by_parameters_materializes_closed_contour() {
    let contour = rectangle();

    let fillet = contour
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
        fillet.report().stage(),
        ContourFilletStage2::ContourMaterialization
    );
    assert_eq!(fillet.report().vertex_index(), 1);
    assert_eq!(
        fillet.report().curve_string_report().input_path(),
        CurveStringFilletInputPath2::Parameters
    );
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .previous_trim()
            .param(),
        &q(3, 4)
    );
    assert_eq!(
        fillet.report().curve_string_report().next_trim().param(),
        &q(1, 4)
    );
    assert_eq!(
        fillet.report().curve_string_report().fillet_segment_index(),
        Some(1)
    );
    assert_eq!(
        fillet.report().source_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );
    assert_eq!(
        fillet.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 4, arcs: 1 })
    );

    let contour = fillet
        .contour()
        .expect("parameter contour fillet should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].end(), &p(3, 0));
    let Segment2::Arc(arc) = &contour.segments()[1] else {
        panic!("fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(3, 0));
    assert_eq!(arc.end(), &p(4, 1));
    assert_eq!(arc.center(), &p(3, 1));
    assert_eq!(contour.segments()[2].start(), &p(4, 1));
}

#[test]
fn contour_fillet_preserves_fill_rule() {
    let contour = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    let fillet = contour
        .fillet_line_line_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), false, &policy())
        .unwrap();

    assert_eq!(fillet.report().fill_rule(), FillRule::EvenOdd);
    assert_eq!(fillet.contour().unwrap().fill_rule(), FillRule::EvenOdd);
}

#[test]
fn contour_fillet_reports_wrong_orientation_boundary() {
    let contour = rectangle();

    let fillet = contour
        .fillet_line_line_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), true, &policy())
        .unwrap();

    assert!(fillet.contour().is_none());
    assert!(fillet.report().status().is_retained_evidence());
    assert_eq!(
        fillet.report().stage(),
        ContourFilletStage2::CurveStringEdit
    );
    assert_eq!(fillet.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(
        fillet.report().curve_string_report().fillet_segment_index(),
        None
    );
    assert_eq!(fillet.report().output_segment_count(), None);
    assert_eq!(
        fillet.report().output_segment_count(),
        fillet.report().curve_string_report().output_segment_count()
    );
}

#[test]
fn contour_fillet_line_line_wraparound_vertex_materializes_closed_contour() {
    let contour = rectangle();

    let fillet = contour
        .fillet_line_line_vertex_by_points(0, &p(0, 1), &p(1, 0), &p(1, 1), false, &policy())
        .unwrap();

    assert!(fillet.report().status().is_native_exact());
    assert_eq!(
        fillet.report().stage(),
        ContourFilletStage2::ContourMaterialization
    );
    assert_eq!(fillet.report().vertex_index(), 0);
    assert_eq!(fillet.report().source_segment_count(), 4);
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .previous_segment_index(),
        3
    );
    assert_eq!(
        fillet.report().curve_string_report().next_segment_index(),
        0
    );
    assert_eq!(fillet.report().output_segment_count(), Some(5));
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .previous_trim()
            .segment_index(),
        3
    );
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .next_trim()
            .segment_index(),
        0
    );
    let segment_reports = fillet.report().curve_string_report().segment_reports();
    assert_eq!(segment_reports.len(), 2);
    assert_eq!(segment_reports[0].source_segment_index(), 3);
    assert_eq!(segment_reports[1].source_segment_index(), 0);

    let contour = fillet
        .contour()
        .expect("wraparound contour fillet should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].start(), &p(0, 4));
    assert_eq!(contour.segments()[0].end(), &p(0, 1));
    let Segment2::Arc(arc) = &contour.segments()[1] else {
        panic!("wraparound fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(0, 1));
    assert_eq!(arc.end(), &p(1, 0));
    assert_eq!(arc.center(), &p(1, 1));
    assert!(!arc.is_clockwise());
    assert_eq!(contour.segments()[2].start(), &p(1, 0));
    assert_eq!(contour.segments()[4].end(), &p(0, 4));
}

#[test]
fn contour_fillet_line_line_wraparound_vertex_by_parameters_materializes_closed_contour() {
    let contour = rectangle();

    let fillet = contour
        .fillet_line_line_vertex_by_parameters(0, q(3, 4), q(1, 4), &p(1, 1), false, &policy())
        .unwrap();

    assert!(fillet.report().status().is_native_exact());
    assert_eq!(
        fillet.report().stage(),
        ContourFilletStage2::ContourMaterialization
    );
    assert_eq!(fillet.report().vertex_index(), 0);
    assert_eq!(
        fillet
            .report()
            .curve_string_report()
            .previous_segment_index(),
        3
    );
    assert_eq!(
        fillet.report().curve_string_report().next_segment_index(),
        0
    );
    let segment_reports = fillet.report().curve_string_report().segment_reports();
    assert_eq!(segment_reports[0].source_segment_index(), 3);
    assert_eq!(segment_reports[1].source_segment_index(), 0);

    let contour = fillet
        .contour()
        .expect("parameter wraparound contour fillet should materialize");
    assert_eq!(contour.len(), 5);
    assert_eq!(contour.segments()[0].end(), &p(0, 1));
    let Segment2::Arc(arc) = &contour.segments()[1] else {
        panic!("wraparound fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(0, 1));
    assert_eq!(arc.end(), &p(1, 0));
    assert_eq!(arc.center(), &p(1, 1));
    assert_eq!(contour.segments()[2].start(), &p(1, 0));
}

#[test]
fn contour_fillet_rejects_out_of_range_vertex() {
    let contour = rectangle();

    assert_eq!(
        contour
            .fillet_line_line_vertex_by_points(4, &p(3, 0), &p(4, 1), &p(3, 1), false, &policy())
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
}

#[test]
fn rectangle_classifies_inside_outside_and_boundary() {
    let contour = rectangle();

    assert_eq!(
        contour.classify_point(&p(1, 1), &policy()),
        Classification::Decided(ContourPointLocation::Inside)
    );
    assert_eq!(
        contour.classify_point(&p(-1, 1), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );
    assert_eq!(
        contour.classify_point(&p(4, 2), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
    assert_eq!(
        contour.classify_point(&p(0, 0), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
}

#[test]
fn prepared_contour_classification_matches_plain_contour() {
    let contour = rectangle();
    let policy = policy();
    let prepared = contour.prepare_topology_queries(&policy);

    assert_eq!(prepared.contour(), &contour);
    assert!(prepared.contour_box().is_some());
    assert_eq!(prepared.segment_boxes().len(), contour.segments().len());
    assert_eq!(
        prepared.prepared_segment_kind_counts(),
        SegmentKindCounts { lines: 4, arcs: 0 }
    );

    for point in [p(1, 1), p(-1, 1), p(4, 2), p(0, 0), p(9, 2)] {
        assert_eq!(
            prepared.point_on_boundary(&point, &policy),
            contour.point_on_boundary(&point, &policy)
        );
        assert_eq!(
            prepared.winding_number(&point, &policy),
            contour.winding_number(&point, &policy)
        );
        assert_eq!(
            prepared.classify_point(&point, &policy),
            contour.classify_point(&point, &policy)
        );
    }
}

#[test]
fn contour_aabb_miss_classifies_outside_and_zero_winding() {
    let contour = rectangle();

    assert_eq!(
        contour.point_on_boundary(&p(9, 2), &policy()),
        Classification::Decided(false)
    );
    assert_eq!(
        contour.winding_number(&p(9, 2), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        contour.classify_point(&p(9, 2), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );
}

#[test]
fn contour_aabb_edge_hit_still_checks_boundary() {
    let contour = rectangle();

    assert_eq!(
        contour.point_on_boundary(&p(4, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        contour.classify_point(&p(4, 2), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
}

#[test]
fn rectangle_winding_is_positive_inside_and_boundary_is_explicit() {
    let contour = rectangle();

    assert_eq!(
        contour.winding_number(&p(2, 2), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        contour.winding_number(&p(4, 2), &policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn exact_boundary_equality_ignores_closed_start_and_direction() {
    let contour = rectangle();
    let rotated = rotated_rectangle();
    let reversed = reversed_rectangle();
    let even_odd = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();
    let different = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(5, 0, 0),
        vertex(5, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap();

    assert!(contour.has_same_exact_boundary(&rotated));
    assert!(contour.has_same_exact_boundary(&reversed));
    assert!(!contour.has_same_exact_boundary(&even_odd));
    assert!(!contour.has_same_exact_boundary(&different));
}

#[test]
fn even_odd_fill_uses_winding_parity() {
    let twice = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 1),
            vertex(2, 0, 1),
            vertex(0, 0, 1),
            vertex(2, 0, 1),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    assert_eq!(
        twice.winding_number(&p(1, 0), &policy()),
        Classification::Decided(2)
    );
    assert_eq!(
        twice.classify_point(&p(1, 0), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );

    let policy = policy();
    let prepared = twice.prepare_topology_queries(&policy);
    assert_eq!(
        prepared.winding_number(&p(1, 0), &policy),
        Classification::Decided(2)
    );
    assert_eq!(
        prepared.classify_point(&p(1, 0), &policy),
        Classification::Decided(ContourPointLocation::Outside)
    );
}

#[test]
fn circular_contour_winds_positive_semicircle_counter_clockwise() {
    let contour = Contour2::from_bulge_vertices(&[vertex(0, 0, 1), vertex(2, 0, 1)]).unwrap();

    assert_eq!(
        contour.winding_number(&p(1, 0), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        contour.classify_point(&p(3, 0), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );

    let reversed = Contour2::from_bulge_vertices(&[vertex(2, 0, -1), vertex(0, 0, -1)]).unwrap();
    assert_eq!(
        reversed.winding_number(&p(1, 0), &policy()),
        Classification::Decided(-1)
    );
}
