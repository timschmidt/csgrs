use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy,
    FillRule, Real, Segment2, UncertaintyReason,
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
fn contour_chamfer_line_line_vertex_materializes_closed_contour() {
    let contour = rectangle();

    let chamfer = contour
        .chamfer_line_line_vertex_by_parameters(1, q(3, 4), q(1, 4), &policy())
        .unwrap();

    assert!(chamfer.report().status().is_native_exact());
    assert_eq!(chamfer.report().vertex_index(), 1);
    assert_eq!(chamfer.report().source_segment_count(), 4);
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
}

#[test]
fn contour_chamfer_rejects_wraparound_vertex_for_now() {
    let contour = rectangle();

    assert_eq!(
        contour
            .chamfer_line_line_vertex_by_parameters(0, q(3, 4), q(1, 4), &policy())
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
