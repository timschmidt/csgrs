use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurveError, CurveString2, FillRule, Point2,
    PolylineReconstructionOptions, Real, Segment2, SegmentKind, SegmentKindCounts,
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
fn finite_ring_import_discards_adjacent_duplicate_source_edges() {
    let contour =
        Contour2::from_finite_ring(&[[0.0, 0.0], [0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 0.0]])
            .unwrap();

    assert_eq!(contour.len(), 3);
}

#[test]
fn finite_ring_import_counts_signed_zero_duplicates_at_the_input_boundary() {
    let contour = Contour2::from_finite_ring(&[
        [-0.0, 0.0],
        [0.0, -0.0],
        [4.0, 0.0],
        [4.0, 3.0],
        [-0.0, 0.0],
    ])
    .unwrap();

    assert_eq!(contour.len(), 3);
    let expected =
        Contour2::from_real_ring(&[[r(0.0), r(0.0)], [r(4.0), r(0.0)], [r(4.0), r(3.0)]]).unwrap();
    assert_eq!(contour, expected);
}

#[test]
fn finite_ring_import_rejects_nonfinite_points_even_when_adjacent() {
    assert_eq!(
        Contour2::from_finite_ring(&[
            [0.0, 0.0],
            [f64::INFINITY, 0.0],
            [f64::INFINITY, 0.0],
            [0.0, 1.0],
        ])
        .unwrap_err(),
        CurveError::NonFiniteReconstructionPoint
    );
}

#[test]
fn finite_ring_import_rejects_all_duplicate_source_edges() {
    assert_eq!(
        Contour2::from_finite_ring(&[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]).unwrap_err(),
        CurveError::InsufficientVertices
    );
}
