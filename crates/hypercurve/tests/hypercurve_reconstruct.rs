use hypercurve::{
    BulgeVertex2, Contour2, CurveError, CurveString2, FillRule, Point2,
    PolylineReconstructionOptions, Real, RetainedImportFormat2, RetainedImportRecord2,
    RetainedImportTopology2, RetainedSourceTolerance2, Segment2,
};

fn r(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn assert_close(actual: f64, expected: f64) {
    let tolerance = 1e-8_f64.max(expected.abs() * 1e-8);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {actual} to be within {tolerance} of {expected}"
    );
}

#[test]
fn reconstruction_merges_collinear_polyline_to_one_line() {
    let points = [p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0), p(3.0, 0.0)];

    let curve =
        CurveString2::reconstruct_from_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(curve.len(), 1);
    let Segment2::Line(line) = &curve.segments()[0] else {
        panic!("collinear samples should reconstruct as one line");
    };
    assert_eq!(line.start(), &points[0]);
    assert_eq!(line.end(), &points[3]);
}

#[test]
fn reconstruction_keeps_single_corner_as_two_lines_by_default() {
    let points = [p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0)];

    let curve =
        CurveString2::reconstruct_from_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(curve.len(), 2);
    assert!(
        curve
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
}

#[test]
fn reconstruction_promotes_consistent_semicircle_samples_to_arc() {
    let root_half = 0.5_f64.sqrt();
    let points = [
        p(0.0, 0.0),
        p(1.0 - root_half, -root_half),
        p(1.0, -1.0),
        p(1.0 + root_half, -root_half),
        p(2.0, 0.0),
    ];
    let options = PolylineReconstructionOptions {
        min_arc_points: 3,
        distance_tolerance: 1e-8,
        ..PolylineReconstructionOptions::default()
    };

    let curve = CurveString2::reconstruct_from_polyline(&points, options).unwrap();

    assert_eq!(curve.len(), 1);
    let Segment2::Arc(arc) = &curve.segments()[0] else {
        panic!("constant-curvature samples should reconstruct as one arc");
    };
    assert_eq!(arc.start(), &points[0]);
    assert_eq!(arc.end(), &points[4]);
    assert_close(arc.bulge().unwrap().to_f64_lossy().unwrap(), 1.0);
    assert_close(arc.center().x().to_f64_lossy().unwrap(), 1.0);
    assert_close(arc.center().y().to_f64_lossy().unwrap(), 0.0);
}

#[test]
fn reconstruction_splits_arcs_at_semicircle_boundary() {
    let points = [
        p(1.0, 0.0),
        p(0.0, -1.0),
        p(-1.0, 0.0),
        p(0.0, 1.0),
        p(1.0, 0.0),
    ];
    let options = PolylineReconstructionOptions {
        min_arc_points: 3,
        ..PolylineReconstructionOptions::default()
    };

    let curve = CurveString2::reconstruct_from_polyline(&points, options).unwrap();

    assert_eq!(curve.len(), 2);
    assert!(
        curve
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Arc(_)))
    );
}

#[test]
fn reconstruction_accepts_closed_polyline_without_repeated_first_point() {
    let points = [p(0.0, 0.0), p(4.0, 0.0), p(4.0, 3.0), p(0.0, 3.0)];

    let contour = Contour2::reconstruct_from_closed_polyline(
        &points,
        PolylineReconstructionOptions::default(),
    )
    .unwrap();

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
fn reconstruction_accepts_closed_polyline_with_repeated_first_point() {
    let points = [
        p(0.0, 0.0),
        p(4.0, 0.0),
        p(4.0, 3.0),
        p(0.0, 3.0),
        p(0.0, 0.0),
    ];

    let contour = Contour2::reconstruct_from_closed_polyline_with_fill_rule(
        &points,
        PolylineReconstructionOptions::default(),
        FillRule::EvenOdd,
    )
    .unwrap();

    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::EvenOdd);
}

#[test]
fn real_line_string_rejects_zero_length_source_edges_without_import_record() {
    let points = [[r(0.0), r(0.0)], [r(0.0), r(0.0)], [r(1.0), r(0.0)]];

    assert_eq!(
        CurveString2::from_real_line_string(&points).unwrap_err(),
        CurveError::ZeroLengthLine
    );
}

#[test]
fn reconstruction_removes_adjacent_duplicate_samples() {
    let points = [p(0.0, 0.0), p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0)];

    let vertices =
        BulgeVertex2::reconstruct_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(vertices.len(), 2);
    assert_eq!(vertices[0].point(), &points[0]);
    assert_eq!(vertices[1].point(), &points[3]);
}

#[test]
fn reconstruction_rejects_invalid_options() {
    let points = [p(0.0, 0.0), p(1.0, 0.0)];
    let options = PolylineReconstructionOptions {
        min_arc_points: 2,
        ..PolylineReconstructionOptions::default()
    };

    let err = CurveString2::reconstruct_from_polyline(&points, options)
        .expect_err("min_arc_points below three is invalid");
    assert_eq!(err, CurveError::InvalidReconstructionOptions);
}

#[test]
fn finite_line_string_import_preserves_step_tolerance_evidence() {
    let tolerance = RetainedSourceTolerance2::try_new(1.0e-5, 1.0e-8).unwrap();
    let import = CurveString2::import_finite_line_string_with_source_report(
        &[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
        RetainedImportFormat2::Step,
        42,
        Some(tolerance),
    )
    .unwrap();
    let record = import.report();

    assert_eq!(import.curve_string().len(), 1);
    assert_eq!(import.record(), record);
    assert_eq!(record.format(), RetainedImportFormat2::Step);
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::OpenLineString
    );
    assert_eq!(record.source_index(), 42);
    assert_eq!(record.source_version(), 0);
    assert_eq!(record.source_tolerance(), Some(tolerance));
    assert_eq!(record.input_point_count(), 3);
    assert_eq!(record.emitted_segment_count(), 1);
    assert_eq!(record.source_edge_count(), 2);
    assert_eq!(record.discarded_duplicate_count(), 1);
    assert!(record.has_discarded_duplicate_edges());
    assert!(record.has_source_tolerance());
    assert!(!record.has_zero_source_tolerance());
    assert!(record.topology_status().is_imported_lossy());
}

#[test]
fn finite_ring_import_preserves_dxf_handle_and_closure_evidence() {
    let tolerance = RetainedSourceTolerance2::try_new(0.0, 1.0e-7).unwrap();
    let import = Contour2::import_finite_ring_with_source_report(
        &[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 0.0]],
        FillRule::EvenOdd,
        RetainedImportFormat2::Dxf,
        0xabc,
        Some(tolerance),
    )
    .unwrap();
    let record = import.report();

    assert_eq!(import.contour().len(), 3);
    assert_eq!(import.record(), record);
    assert_eq!(import.contour().fill_rule(), FillRule::EvenOdd);
    assert_eq!(record.format(), RetainedImportFormat2::Dxf);
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::ClosedRing
    );
    assert_eq!(record.source_index(), 0xabc);
    assert_eq!(record.source_version(), 0);
    assert_eq!(record.source_tolerance().unwrap().relative(), 1.0e-7);
    assert!(record.has_source_tolerance());
    assert!(!record.has_zero_source_tolerance());
    assert_eq!(record.source_edge_count(), 4);
    assert_eq!(record.discarded_duplicate_count(), 1);
    assert!(record.has_discarded_duplicate_edges());
    assert!(record.topology_status().is_imported_lossy());
}

#[test]
fn finite_ring_import_accepts_unrepeated_closed_edge_accounting() {
    let import = Contour2::import_finite_ring_with_source(
        &[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]],
        FillRule::NonZero,
        RetainedImportFormat2::Dxf,
        0xdef,
        None,
    )
    .unwrap();
    let record = import.record();

    assert_eq!(import.contour().len(), 4);
    assert_eq!(record.input_point_count(), 4);
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::ClosedRing
    );
    assert_eq!(record.source_edge_count(), 4);
    assert_eq!(record.emitted_segment_count(), 4);
    assert_eq!(record.discarded_duplicate_count(), 0);
    assert!(!record.has_discarded_duplicate_edges());
    assert!(!record.has_source_tolerance());
    assert!(!record.has_zero_source_tolerance());
}

#[test]
fn finite_import_record_exposes_zero_tolerance_boundary_claim() {
    let tolerance = RetainedSourceTolerance2::try_new(0.0, 0.0).unwrap();
    let import = CurveString2::import_finite_line_string_with_source(
        &[[0.0, 0.0], [1.0, 0.0]],
        RetainedImportFormat2::Application,
        9,
        Some(tolerance),
    )
    .unwrap();

    assert!(import.record().has_source_tolerance());
    assert!(import.record().has_zero_source_tolerance());
    assert_eq!(import.record().source_edge_count(), 1);
    assert!(!import.record().has_discarded_duplicate_edges());
}

#[test]
fn finite_imports_preserve_source_versions() {
    let open = CurveString2::import_finite_line_string_with_source_version_report(
        &[[0.0, 0.0], [1.0, 0.0]],
        RetainedImportFormat2::Step,
        42,
        7,
        None,
    )
    .unwrap();
    assert_eq!(open.record().source_index(), 42);
    assert_eq!(open.record().source_version(), 7);
    assert_eq!(
        open.record().source_topology(),
        RetainedImportTopology2::OpenLineString
    );

    let closed = Contour2::import_finite_ring_with_source_version_report(
        &[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]],
        FillRule::NonZero,
        RetainedImportFormat2::Dxf,
        0xdef,
        11,
        None,
    )
    .unwrap();
    assert_eq!(closed.record().source_index(), 0xdef);
    assert_eq!(closed.record().source_version(), 11);
    assert_eq!(
        closed.record().source_topology(),
        RetainedImportTopology2::ClosedRing
    );
}

#[test]
fn finite_ring_import_discards_adjacent_duplicate_source_edges() {
    let import = Contour2::import_finite_ring_with_source(
        &[[0.0, 0.0], [0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 0.0]],
        FillRule::NonZero,
        RetainedImportFormat2::Dxf,
        0xbeef,
        None,
    )
    .unwrap();
    let record = import.record();

    assert_eq!(import.contour().len(), 3);
    assert_eq!(record.input_point_count(), 5);
    assert_eq!(record.emitted_segment_count(), 3);
    assert_eq!(record.discarded_duplicate_count(), 2);
    assert_eq!(
        record.source_topology(),
        RetainedImportTopology2::ClosedRing
    );
}

#[test]
fn finite_ring_import_rejects_all_duplicate_source_edges() {
    assert_eq!(
        Contour2::import_finite_ring(&[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]).unwrap_err(),
        CurveError::InsufficientVertices
    );
}

#[test]
fn source_tolerance_rejects_nonfinite_or_negative_values() {
    assert_eq!(
        RetainedSourceTolerance2::try_new(f64::NAN, 0.0).unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedSourceTolerance2::try_new(0.0, -1.0).unwrap_err(),
        CurveError::InvalidImportRecord
    );
}

#[test]
fn retained_import_record_rejects_inconsistent_counts() {
    assert_eq!(
        RetainedImportRecord2::try_new(RetainedImportFormat2::FinitePolyline, 0, None, 0, 0, 0)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new(RetainedImportFormat2::FinitePolyline, 0, None, 2, 0, 0)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new(RetainedImportFormat2::FinitePolyline, 0, None, 2, 2, 0)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    RetainedImportRecord2::try_new_closed_ring(RetainedImportFormat2::Dxf, 0, None, 3, 3, 0)
        .unwrap();
    assert_eq!(
        RetainedImportRecord2::try_new_closed_ring(RetainedImportFormat2::Dxf, 0, None, 3, 1, 2)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new(RetainedImportFormat2::FinitePolyline, 0, None, 3, 2, 2)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new(RetainedImportFormat2::FinitePolyline, 0, None, 5, 2, 0)
            .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new(
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
            usize::MAX,
            usize::MAX,
            1
        )
        .unwrap_err(),
        CurveError::InvalidImportRecord
    );
}

#[test]
fn retained_import_record_rejects_cross_topology_edge_evidence() {
    assert_eq!(
        RetainedImportRecord2::try_new_open_line_string(
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
            3,
            3,
            0
        )
        .unwrap_err(),
        CurveError::InvalidImportRecord
    );
    assert_eq!(
        RetainedImportRecord2::try_new_closed_ring(
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
            3,
            2,
            0
        )
        .unwrap_err(),
        CurveError::InvalidImportRecord
    );

    let open = RetainedImportRecord2::try_new_open_line_string(
        RetainedImportFormat2::FinitePolyline,
        7,
        None,
        3,
        1,
        1,
    )
    .unwrap();
    let closed = RetainedImportRecord2::try_new_closed_ring(
        RetainedImportFormat2::FinitePolyline,
        7,
        None,
        4,
        3,
        1,
    )
    .unwrap();

    assert_eq!(
        open.source_topology(),
        RetainedImportTopology2::OpenLineString
    );
    assert_eq!(
        closed.source_topology(),
        RetainedImportTopology2::ClosedRing
    );
    assert_eq!(open.source_version(), 0);
    assert_eq!(closed.source_version(), 0);

    let versioned = RetainedImportRecord2::try_new_open_line_string_with_source_version(
        RetainedImportFormat2::Application,
        9,
        17,
        None,
        3,
        2,
        0,
    )
    .unwrap();
    assert_eq!(versioned.source_index(), 9);
    assert_eq!(versioned.source_version(), 17);
}
