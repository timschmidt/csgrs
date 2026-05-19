use hypercurve::{
    BooleanBoundaryAuditStatus, BooleanBoundaryTraversalStatus, BooleanOp,
    BooleanRegionAuditStatus, BoundaryContourNestingStatus, BulgeVertex2, Classification, Contour2,
    ContourPointLocation, CurvePolicy, FillRule, Real, Region2, RegionPointLocation,
    Segment2, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn pf(x: f64, y: f64) -> hypercurve::Point2 {
    hypercurve::Point2::new(Real::try_from(x).unwrap(), Real::try_from(y).unwrap())
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn rectangle_rotated_start(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
    ])
}

fn rectangle_reversed(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmin, ymax, 0),
        vertex(xmax, ymax, 0),
        vertex(xmax, ymin, 0),
    ])
}

fn region(contours: Vec<Contour2>) -> Region2 {
    Region2::from_material_contours(contours)
}

fn donut(outer: Contour2, hole: Contour2) -> Region2 {
    Region2::new(vec![outer], vec![hole])
}

fn touching_material_bins() -> Region2 {
    Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)])
}

fn touching_material_bins_reordered() -> Region2 {
    Region2::from_material_contours(vec![rectangle(2, 0, 4, 2), rectangle(0, 0, 2, 2)])
}

fn touching_material_bins_rotated_and_reversed() -> Region2 {
    Region2::from_material_contours(vec![
        rectangle_reversed(2, 0, 4, 2),
        rectangle_rotated_start(0, 0, 2, 2),
    ])
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_contour_location(
    contour: &Contour2,
    point: hypercurve::Point2,
    expected: ContourPointLocation,
) {
    assert_eq!(
        contour.classify_point(&point, &policy()),
        Classification::Decided(expected)
    );
}

fn assert_point_finite(point: &hypercurve::Point2) {
    assert!(point.x().to_f64_approx().is_some_and(f64::is_finite));
    assert!(point.y().to_f64_approx().is_some_and(f64::is_finite));
}

fn assert_segment_finite(segment: &Segment2) {
    match segment {
        Segment2::Line(line) => {
            assert_point_finite(line.start());
            assert_point_finite(line.end());
        }
        Segment2::Arc(arc) => {
            assert_point_finite(arc.start());
            assert_point_finite(arc.end());
            assert_point_finite(arc.center());
            assert!(
                arc.radius_squared()
                    .to_f64_approx()
                    .is_some_and(f64::is_finite)
            );
        }
    }
}

fn assert_contour_finite(contour: &Contour2) {
    for segment in contour.segments() {
        assert_segment_finite(segment);
    }
}

fn assert_region_location(
    region: &Region2,
    point: hypercurve::Point2,
    expected: RegionPointLocation,
) {
    assert_eq!(
        region.classify_point(&point, &policy()),
        Classification::Decided(expected)
    );
}

#[test]
fn region_boolean_boundary_contours_union_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(5, 2), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(5, 4), ContourPointLocation::Outside);
}

#[test]
fn prepared_region_boolean_boundary_loops_match_plain_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);
    let policy = policy();
    let first_prepared = first.prepare_topology_queries(&policy);
    let second_prepared = second.prepare_topology_queries(&policy);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
    ] {
        assert_eq!(
            first_prepared
                .boolean_boundary_loops(&second_prepared, op, &policy)
                .unwrap(),
            first.boolean_boundary_loops(&second, op, &policy).unwrap()
        );
    }
}

#[test]
fn prepared_region_boolean_boundary_loops_regularizes_contact_degeneracies() {
    let shared_edge_first = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_second = region(vec![rectangle(2, -2, 6, 0)]);
    let point_touch_first = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_second = region(vec![rectangle(2, 2, 4, 4)]);
    let policy = policy();

    let Classification::Decided(shared_edge_loops) = shared_edge_first
        .prepare_topology_queries(&policy)
        .boolean_boundary_loops(
            &shared_edge_second.prepare_topology_queries(&policy),
            BooleanOp::Union,
            &policy,
        )
        .unwrap()
    else {
        panic!("shared-edge boundary-contact union should be decided");
    };
    assert_eq!(shared_edge_loops.len(), 1);

    let Classification::Decided(point_touch_loops) = point_touch_first
        .prepare_topology_queries(&policy)
        .boolean_boundary_loops(
            &point_touch_second.prepare_topology_queries(&policy),
            BooleanOp::Union,
            &policy,
        )
        .unwrap()
    else {
        panic!("point-touch boundary-contact union should be decided");
    };
    assert_eq!(point_touch_loops.len(), 2);

    let Classification::Decided(shared_edge_plain) = shared_edge_first
        .boolean_boundary_loops(&shared_edge_second, BooleanOp::Union, &policy)
        .unwrap()
    else {
        panic!("plain shared-edge union should be decided");
    };
    let Classification::Decided(point_touch_plain) = point_touch_first
        .boolean_boundary_loops(&point_touch_second, BooleanOp::Union, &policy)
        .unwrap()
    else {
        panic!("plain point-touch union should be decided");
    };
    assert_eq!(shared_edge_loops, shared_edge_plain);
    assert_eq!(point_touch_loops, point_touch_plain);
}

#[test]
fn boundary_traversal_report_explains_ready_and_unresolved_loop_stages() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);
    let shared_edge_first = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_second = region(vec![rectangle(2, -2, 6, 0)]);
    let point_touch_first = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(ready) = first
        .boolean_boundary_traversal_report(&second, BooleanOp::Union, &policy())
        .unwrap()
    else {
        panic!("expected decided traversal report");
    };
    assert_eq!(ready.status, BooleanBoundaryTraversalStatus::LoopsReady);
    assert!(ready.is_ready());
    assert_eq!(ready.blocker_reason, None);
    assert_eq!(ready.unresolved_boundary_count, 0);
    assert_eq!(ready.open_chain_count, 0);
    assert!(ready.loops.as_ref().is_some_and(|loops| !loops.is_empty()));

    let Classification::Decided(unresolved) = shared_edge_first
        .boolean_boundary_traversal_report(&shared_edge_second, BooleanOp::Union, &policy())
        .unwrap()
    else {
        panic!("expected decided traversal blocker report");
    };
    assert_eq!(
        unresolved.status,
        BooleanBoundaryTraversalStatus::UnresolvedBoundaries
    );
    assert!(!unresolved.is_ready());
    assert_eq!(unresolved.blocker_reason, Some(UncertaintyReason::Boundary));
    assert!(unresolved.unresolved_boundary_count > 0);
    assert!(unresolved.loops.is_none());

    let Classification::Decided(unsupported) = point_touch_first
        .boolean_boundary_traversal_report(&point_touch_second, BooleanOp::Union, &policy())
        .unwrap()
    else {
        panic!("expected decided unsupported traversal report");
    };
    assert_eq!(
        unsupported.status,
        BooleanBoundaryTraversalStatus::UnsupportedTraversal
    );
    assert_eq!(
        unsupported.blocker_reason,
        Some(UncertaintyReason::Unsupported)
    );
    assert!(!unsupported.is_ready());
    assert!(unsupported.loops.is_none());
}

#[test]
/// Verifies traversal blockers remain stable for boundary-contact degeneracies.
///
/// The test is intentionally anchored to the exact-geometric-computation principle
/// (C. K. Yap, *Towards Exact Geometric Computation*, 1997): shared-edge and
/// point-touch contacts are kept as explicit uncertainty/failure states in the
/// traversal report layer rather than being guessed away.
///
/// Degenerate shared-boundary treatment follows Foster, Hormann, and Popa's
/// boundary-contact model (2019) while preserving traversal blockers through
/// owned and prepared/mixed call paths.
fn prepared_boundary_traversal_reports_preserve_blockers_on_boundary_contacts() {
    let shared_edge_first = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_second = region(vec![rectangle(2, -2, 6, 0)]);
    let point_touch_first = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_second = region(vec![rectangle(2, 2, 4, 4)]);
    let policy = policy();
    let shared_first_prepared = shared_edge_first.prepare_topology_queries(&policy);
    let shared_second_prepared = shared_edge_second.prepare_topology_queries(&policy);
    let point_first_prepared = point_touch_first.prepare_topology_queries(&policy);
    let point_second_prepared = point_touch_second.prepare_topology_queries(&policy);

    let shared_plain = match shared_edge_first
        .boolean_boundary_traversal_report(&shared_edge_second, BooleanOp::Union, &policy)
        .unwrap()
    {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("shared-edge traversal report should be decided: {reason:?}")
        }
    };
    // Shared-edge contacts are an explicit unresolved-boundary case in the
    // Foster, Hormann, and Popa degeneracy model (2019), so traversal should
    // return a blocker reason rather than silently forcing an unsupported
    // topological guess.
    assert_eq!(shared_plain.status, BooleanBoundaryTraversalStatus::UnresolvedBoundaries);
    assert_eq!(shared_plain.blocker_reason, Some(UncertaintyReason::Boundary));
    assert!(shared_plain.unresolved_boundary_count > 0);
    assert!(shared_plain.loops.is_none());

    assert_eq!(
        shared_first_prepared
            .boolean_boundary_traversal_report(&shared_second_prepared, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(shared_plain.clone())
    );
    assert_eq!(
        shared_first_prepared
            .boolean_boundary_traversal_report_against_region(&shared_edge_second.as_view(), BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(shared_plain.clone())
    );
    assert_eq!(
        shared_edge_first
            .as_view()
            .boolean_boundary_traversal_report_against_prepared_region(&shared_second_prepared, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(shared_plain)
    );

    let point_plain = match point_touch_first
        .boolean_boundary_traversal_report(&point_touch_second, BooleanOp::Union, &policy)
        .unwrap()
    {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("point-touch traversal report should be decided: {reason:?}")
        }
    };
    // Point-touch degeneracies are currently unsupported by the raw loop
    // traversal kernel and are intentionally surfaced as blockers for a later
    // resolver path instead of collapsing into a fabricated boundary result.
    assert_eq!(
        point_plain.status,
        BooleanBoundaryTraversalStatus::UnsupportedTraversal
    );
    assert_eq!(
        point_plain.blocker_reason,
        Some(UncertaintyReason::Unsupported)
    );
    assert!(point_plain.loops.is_none());

    assert_eq!(
        point_first_prepared
            .boolean_boundary_traversal_report(&point_second_prepared, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(point_plain.clone())
    );
    assert_eq!(
        point_first_prepared
            .boolean_boundary_traversal_report_against_region(&point_touch_second.as_view(), BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(point_plain.clone())
    );
    assert_eq!(
        point_touch_first
            .as_view()
            .boolean_boundary_traversal_report_against_prepared_region(&point_second_prepared, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(point_plain)
    );
}

#[test]
fn prepared_boundary_traversal_reports_match_plain_audits() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);
    let policy = policy();
    let first_prepared = first.prepare_topology_queries(&policy);
    let second_prepared = second.prepare_topology_queries(&policy);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
    ] {
        let plain = first
            .boolean_boundary_traversal_report(&second, op, &policy)
            .unwrap();
        assert_eq!(
            first_prepared
                .boolean_boundary_traversal_report(&second_prepared, op, &policy)
                .unwrap(),
            plain
        );
        assert_eq!(
            first_prepared
                .boolean_boundary_traversal_report_against_region(&second.as_view(), op, &policy)
                .unwrap(),
            plain
        );
        assert_eq!(
            first
                .as_view()
                .boolean_boundary_traversal_report_against_prepared_region(
                    &second_prepared,
                    op,
                    &policy,
                )
                .unwrap(),
            plain
        );
    }
}

#[test]
fn prepared_region_boolean_boundary_contours_match_plain_results() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);
    let policy = policy();
    let first_prepared = first.prepare_topology_queries(&policy);
    let second_prepared = second.prepare_topology_queries(&policy);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        assert_eq!(
            first_prepared
                .boolean_boundary_contours(&second_prepared, op, FillRule::NonZero, &policy)
                .unwrap(),
            first
                .boolean_boundary_contours(&second, op, FillRule::NonZero, &policy)
                .unwrap()
        );
    }
}

#[test]
fn prepared_region_boolean_region_matches_plain_regularized_cases() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            region(vec![rectangle(0, 0, 2, 2)]),
            region(vec![rectangle(2, 2, 4, 4)]),
        ),
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -2, 6, 0)]),
        ),
        (
            donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8)),
            region(vec![rectangle(6, 2, 10, 10)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);
        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            assert_eq!(
                first_prepared
                    .boolean_region(&second_prepared, op, FillRule::NonZero, &policy)
                    .unwrap(),
                first
                    .boolean_region(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
        }
    }
}

#[test]
fn prepared_region_boolean_region_decides_boundary_touching_containment_identities() {
    let outer = region(vec![rectangle(0, 0, 6, 6)]);
    let inner_touching_edge = region(vec![rectangle(2, 0, 4, 2)]);
    let policy = policy();
    let outer_prepared = outer.prepare_topology_queries(&policy);
    let inner_prepared = inner_touching_edge.prepare_topology_queries(&policy);

    assert_eq!(
        outer_prepared
            .boolean_region(
                &inner_prepared,
                BooleanOp::Union,
                FillRule::NonZero,
                &policy
            )
            .unwrap(),
        outer
            .boolean_region(
                &inner_touching_edge,
                BooleanOp::Union,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
    );
    assert_eq!(
        outer_prepared
            .boolean_region(
                &inner_prepared,
                BooleanOp::Intersection,
                FillRule::NonZero,
                &policy,
            )
            .unwrap(),
        outer
            .boolean_region(
                &inner_touching_edge,
                BooleanOp::Intersection,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
    );
    assert_eq!(
        inner_prepared
            .boolean_region(
                &outer_prepared,
                BooleanOp::Difference,
                FillRule::NonZero,
                &policy,
            )
            .unwrap(),
        inner_touching_edge
            .boolean_region(&outer, BooleanOp::Difference, FillRule::NonZero, &policy)
            .unwrap()
    );
    assert_eq!(
        outer_prepared
            .boolean_region(
                &inner_prepared,
                BooleanOp::Difference,
                FillRule::NonZero,
                &policy,
            )
            .unwrap(),
        outer
            .boolean_region(
                &inner_touching_edge,
                BooleanOp::Difference,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
    );
}

#[test]
fn prepared_region_boolean_against_region_view_matches_prepared_and_plain() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            region(vec![rectangle(0, 0, 2, 2)]),
            region(vec![rectangle(2, 2, 4, 4)]),
        ),
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -2, 6, 0)]),
        ),
        (
            donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8)),
            region(vec![rectangle(6, 2, 10, 10)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);
        let first_view = first.as_view();
        let second_view = second.as_view();

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
        ] {
            assert_eq!(
                first_prepared
                    .boolean_boundary_loops_against_region(&second_view, op, &policy)
                    .unwrap(),
                first_prepared
                    .boolean_boundary_loops(&second_prepared, op, &policy)
                    .unwrap()
            );
            assert_eq!(
                first_view
                    .boolean_boundary_loops_against_prepared_region(&second_prepared, op, &policy)
                    .unwrap(),
                first.boolean_boundary_loops(&second, op, &policy).unwrap()
            );
        }

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            assert_eq!(
                first_prepared
                    .boolean_boundary_contours_against_region(
                        &second_view,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                first
                    .boolean_boundary_contours(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
            assert_eq!(
                first_view
                    .boolean_boundary_contours_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                first
                    .boolean_boundary_contours(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
            assert_eq!(
                first_prepared
                    .boolean_region_against_region(&second_view, op, FillRule::NonZero, &policy)
                    .unwrap(),
                first
                    .boolean_region(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
            assert_eq!(
                first_view
                    .boolean_region_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                first
                    .boolean_region(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
        }
    }
}

#[test]
fn prepared_region_boolean_reports_match_plain_audits() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            region(vec![rectangle(0, 0, 2, 2)]),
            region(vec![rectangle(2, 2, 4, 4)]),
        ),
        (
            donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8)),
            region(vec![rectangle(6, 2, 10, 10)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            let plain = first
                .boolean_region_report(&second, op, FillRule::NonZero, &policy)
                .unwrap();
            assert_eq!(
                first_prepared
                    .boolean_region_report(&second_prepared, op, FillRule::NonZero, &policy)
                    .unwrap(),
                plain
            );
            assert_eq!(
                first_prepared
                    .boolean_region_report_against_region(
                        &second.as_view(),
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first
                    .as_view()
                    .boolean_region_report_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
        }
    }
}

#[test]
fn prepared_region_boolean_pipeline_reports_match_plain_audits() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8)),
            region(vec![rectangle(6, 2, 10, 10)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            let plain = first
                .boolean_region_pipeline_report(&second, op, FillRule::NonZero, &policy)
                .unwrap();
            assert_eq!(
                first_prepared
                    .boolean_region_pipeline_report(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first_prepared
                    .boolean_region_pipeline_report_against_region(
                        &second.as_view(),
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first
                    .as_view()
                    .boolean_region_pipeline_report_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
        }
    }
}

#[test]
fn prepared_boundary_contour_reports_match_plain_audits() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            region(vec![rectangle(0, 0, 2, 2)]),
            region(vec![rectangle(2, 2, 4, 4)]),
        ),
        (
            region(vec![rectangle(47, -7, 60, -5)]),
            region(vec![rectangle(47, -5, 60, -3)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            let plain = first
                .boolean_boundary_contour_report(&second, op, FillRule::NonZero, &policy)
                .unwrap();
            assert_eq!(
                first_prepared
                    .boolean_boundary_contour_report(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first_prepared
                    .boolean_boundary_contour_report_against_region(
                        &second.as_view(),
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first
                    .as_view()
                    .boolean_boundary_contour_report_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
        }
    }
}

#[test]
fn prepared_boundary_loop_reports_match_plain_audits() {
    let cases = [
        (
            region(vec![rectangle(0, 0, 4, 4)]),
            region(vec![rectangle(2, -1, 6, 3)]),
        ),
        (
            donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8)),
            region(vec![rectangle(6, 2, 10, 10)]),
        ),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
        ] {
            let plain = first
                .boolean_boundary_loop_report(&second, op, FillRule::NonZero, &policy)
                .unwrap();
            assert_eq!(
                first_prepared
                    .boolean_boundary_loop_report(&second_prepared, op, FillRule::NonZero, &policy,)
                    .unwrap(),
                plain
            );
            assert_eq!(
                first_prepared
                    .boolean_boundary_loop_report_against_region(
                        &second.as_view(),
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
            assert_eq!(
                first
                    .as_view()
                    .boolean_boundary_loop_report_against_prepared_region(
                        &second_prepared,
                        op,
                        FillRule::NonZero,
                        &policy,
                    )
                    .unwrap(),
                plain
            );
        }
    }
}

#[test]
fn prepared_region_boolean_identity_fast_paths_match_plain() {
    let cases = [
        (touching_material_bins(), touching_material_bins()),
        (touching_material_bins(), touching_material_bins_reordered()),
        (
            touching_material_bins(),
            touching_material_bins_rotated_and_reversed(),
        ),
        (Region2::empty(), touching_material_bins()),
        (touching_material_bins(), Region2::empty()),
    ];
    let policy = policy();

    for (first, second) in cases {
        let first_prepared = first.prepare_topology_queries(&policy);
        let second_prepared = second.prepare_topology_queries(&policy);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            assert_eq!(
                first_prepared
                    .boolean_boundary_contours(&second_prepared, op, FillRule::NonZero, &policy)
                    .unwrap(),
                first
                    .boolean_boundary_contours(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
            assert_eq!(
                first_prepared
                    .boolean_region(&second_prepared, op, FillRule::NonZero, &policy)
                    .unwrap(),
                first
                    .boolean_region(&second, op, FillRule::NonZero, &policy)
                    .unwrap()
            );
        }
    }
}

#[test]
fn region_boolean_region_union_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(result) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 4), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_axis_aligned_rectangles_with_collinear_edge_overlap_are_regularized() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(1, 0, 3, 2)]);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let actual = first
            .boolean_region(&second, op, FillRule::NonZero, &policy())
            .unwrap();
        let Classification::Decided(result) = actual else {
            panic!("expected decided {op:?} for aligned overlapping rectangles, got {actual:?}");
        };

        match op {
            BooleanOp::Union => {
                assert_eq!(result.material_contours().len(), 1);
                assert_region_location(&result, p(0, 1), RegionPointLocation::Boundary);
                assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
                assert_region_location(&result, p(2, 1), RegionPointLocation::Inside);
                assert_region_location(&result, p(3, 1), RegionPointLocation::Boundary);
            }
            BooleanOp::Intersection => {
                assert_eq!(result.material_contours().len(), 1);
                assert_region_location(&result, p(1, 1), RegionPointLocation::Boundary);
                assert_region_location(&result, pf(1.5, 1.0), RegionPointLocation::Inside);
                assert_region_location(&result, pf(0.5, 1.0), RegionPointLocation::Outside);
            }
            BooleanOp::Difference => {
                assert_eq!(result.material_contours().len(), 1);
                assert_region_location(&result, pf(0.5, 1.0), RegionPointLocation::Inside);
                assert_region_location(&result, pf(1.5, 1.0), RegionPointLocation::Outside);
            }
            BooleanOp::Xor => {
                assert_eq!(result.material_contours().len(), 2);
                assert_region_location(&result, pf(0.5, 1.0), RegionPointLocation::Inside);
                assert_region_location(&result, pf(1.5, 1.0), RegionPointLocation::Outside);
                assert_region_location(&result, pf(2.5, 1.0), RegionPointLocation::Inside);
            }
        }
    }
}

#[test]
fn region_boolean_report_audits_regularized_union_output() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(report) = first
        .boolean_region_report(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided boolean region report");
    };

    assert_eq!(report.operation, BooleanOp::Union);
    assert_eq!(report.audit.status, BooleanRegionAuditStatus::Valid);
    assert!(report.audit.is_valid());
    assert_eq!(report.audit.material_contour_count, 1);
    assert_eq!(report.audit.hole_contour_count, 0);
    assert_region_location(&report.result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&report.result, p(5, 2), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_pipeline_report_audits_boundary_nesting_and_region() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(report) = first
        .boolean_region_pipeline_report(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided boolean pipeline report");
    };

    assert_eq!(report.operation, BooleanOp::Union);
    assert_eq!(
        report.boundary_audit.status,
        BooleanBoundaryAuditStatus::Valid
    );
    assert!(report.boundary_audit.is_valid());
    assert_eq!(report.boundary_audit.contour_count, 1);
    assert_eq!(
        report.nesting_audit.status,
        BoundaryContourNestingStatus::Valid
    );
    assert!(report.nesting_audit.is_valid());
    assert_eq!(report.nesting_audit.input_contour_count, 1);
    assert_eq!(report.nesting_audit.material_contour_count, 1);
    assert_eq!(report.nesting_audit.hole_contour_count, 0);
    assert_eq!(report.region_audit.status, BooleanRegionAuditStatus::Valid);
    assert!(report.region_audit.is_valid());
    assert_eq!(report.result.material_contours().len(), 1);
    assert_region_location(&report.result, p(5, 2), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contour_report_audits_regularized_union_output() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(report) = first
        .boolean_boundary_contour_report(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided boundary contour report");
    };

    assert_eq!(report.operation, BooleanOp::Union);
    assert_eq!(report.audit.status, BooleanBoundaryAuditStatus::Valid);
    assert!(report.audit.is_valid());
    assert_eq!(report.audit.contour_count, 1);
    assert_eq!(report.contours.len(), 1);
    assert_contour_location(&report.contours[0], p(1, 1), ContourPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_loop_report_audits_regularized_union_output() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(report) = first
        .boolean_boundary_loop_report(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided boundary loop report");
    };

    assert_eq!(report.operation, BooleanOp::Union);
    assert_eq!(report.audit.status, BooleanBoundaryAuditStatus::Valid);
    assert!(report.audit.is_valid());
    assert_eq!(report.audit.loop_count, 1);
    assert_eq!(report.loops.len(), 1);
    let contours = report.loops.to_contours(FillRule::NonZero).unwrap();
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Inside);
}

#[test]
fn region_boolean_report_marks_empty_regularized_results() {
    let rect = region(vec![rectangle(0, 0, 4, 4)]);

    let Classification::Decided(report) = rect
        .boolean_region_report(&rect, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty boolean report");
    };

    assert_eq!(report.audit.status, BooleanRegionAuditStatus::Empty);
    assert!(report.audit.is_valid());
    assert!(report.result.is_empty());
}

#[test]
fn region_boolean_boundary_contours_intersection_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided intersection boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(3, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Outside);
    assert_contour_location(&contours[0], p(5, 2), ContourPointLocation::Outside);
}

#[test]
fn region_boolean_region_difference_nested_rectangle_creates_hole() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(&inner, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided nested difference region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(3, 5), RegionPointLocation::Boundary);
}

#[test]
fn region_boolean_region_intersection_nested_rectangle_keeps_inner_material() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(
            &inner,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided nested intersection region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Inside);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_xor_nested_rectangle_creates_hole() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(&inner, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided nested xor region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_identical_rectangle_identities_are_decided() {
    let rect = region(vec![rectangle(0, 0, 4, 4)]);

    let Classification::Decided(union) = rect
        .boolean_region(&rect, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_region_location(&union, p(2, 2), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = rect
        .boolean_region(&rect, BooleanOp::Intersection, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self intersection");
    };
    assert_eq!(intersection.material_contours().len(), 1);
    assert_region_location(&intersection, p(2, 2), RegionPointLocation::Inside);

    let Classification::Decided(difference) = rect
        .boolean_region(&rect, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self difference");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = rect
        .boolean_region(&rect, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self xor");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_identical_donut_identities_are_decided() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(union) = ring
        .boolean_region(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_eq!(union.hole_contours().len(), 1);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = ring
        .boolean_region(&ring, BooleanOp::Intersection, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self intersection");
    };
    assert_eq!(intersection.material_contours().len(), 1);
    assert_eq!(intersection.hole_contours().len(), 1);
    assert_region_location(&intersection, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&intersection, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(difference) = ring
        .boolean_region(&ring, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self difference");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = ring
        .boolean_region(&ring, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self xor");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_empty_identities_preserve_donut_roles() {
    let empty = Region2::empty();
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(union_left) = empty
        .boolean_region(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty union donut");
    };
    assert_eq!(union_left.material_contours().len(), 1);
    assert_eq!(union_left.hole_contours().len(), 1);
    assert_region_location(&union_left, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_left, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(union_right) = ring
        .boolean_region(&empty, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union empty");
    };
    assert_eq!(union_right.material_contours().len(), 1);
    assert_eq!(union_right.hole_contours().len(), 1);
    assert_region_location(&union_right, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_right, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = ring
        .boolean_region(
            &empty,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection empty");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = ring
        .boolean_region(&empty, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference empty");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_eq!(difference.hole_contours().len(), 1);

    let Classification::Decided(empty_difference) = empty
        .boolean_region(&ring, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty difference donut");
    };
    assert!(empty_difference.is_empty());
}

#[test]
fn region_boolean_region_self_identity_preserves_touching_material_bins() {
    let touching = touching_material_bins();

    assert_eq!(
        Region2::from_boundary_contours(
            vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let Classification::Decided(union) = touching
        .boolean_region(&touching, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self union for explicit touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = touching
        .boolean_region(
            &touching,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided self intersection for explicit touching bins");
    };
    assert_eq!(intersection.material_contours().len(), 2);
    assert_eq!(intersection.hole_contours().len(), 0);

    let Classification::Decided(difference) = touching
        .boolean_region(
            &touching,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided self difference for explicit touching bins");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = touching
        .boolean_region(&touching, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self xor for explicit touching bins");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_identity_accepts_reordered_touching_material_bins() {
    let first = touching_material_bins();
    let second = touching_material_bins_reordered();

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union for reordered touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided intersection for reordered touching bins");
    };
    assert_eq!(intersection.material_contours().len(), 2);
    assert_eq!(intersection.hole_contours().len(), 0);

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided difference for reordered touching bins");
    };
    assert!(difference.is_empty());
}

#[test]
fn region_boolean_region_identity_accepts_rotated_and_reversed_bins() {
    let first = touching_material_bins();
    let second = touching_material_bins_rotated_and_reversed();

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union for rotated/reversed touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided xor for rotated/reversed touching bins");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_empty_identity_preserves_touching_material_bins() {
    let empty = Region2::empty();
    let touching = touching_material_bins();

    let Classification::Decided(union_left) = empty
        .boolean_region(&touching, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty union touching region");
    };
    assert_eq!(union_left.material_contours().len(), 2);
    assert_eq!(union_left.hole_contours().len(), 0);
    assert_region_location(&union_left, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_left, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(union_right) = touching
        .boolean_region(&empty, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided touching union empty");
    };
    assert_eq!(union_right.material_contours().len(), 2);
    assert_eq!(union_right.hole_contours().len(), 0);

    let Classification::Decided(xor_left) = empty
        .boolean_region(&touching, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty xor touching region");
    };
    assert_eq!(xor_left.material_contours().len(), 2);

    let Classification::Decided(difference) = touching
        .boolean_region(&empty, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided touching difference empty");
    };
    assert_eq!(difference.material_contours().len(), 2);

    let Classification::Decided(empty_difference) = empty
        .boolean_region(
            &touching,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided empty difference touching region");
    };
    assert!(empty_difference.is_empty());
}

#[test]
fn region_boolean_region_union_adds_island_inside_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(3, 3, 9, 9));
    let island = region(vec![rectangle(5, 5, 7, 7)]);

    let Classification::Decided(result) = ring
        .boolean_region(&island, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union island");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(3, 3), RegionPointLocation::Boundary);
    assert_region_location(&result, p(4, 4), RegionPointLocation::Outside);
    assert_region_location(&result, p(6, 6), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_difference_ignores_island_inside_hole() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));
    let island = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(result) = ring
        .boolean_region(&island, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference island-in-hole");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_intersection_with_island_inside_hole_is_empty() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));
    let island = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(result) = ring
        .boolean_region(
            &island,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection island-in-hole");
    };

    assert!(result.is_empty());
}

#[test]
fn region_boolean_region_hole_boundary_cutter_union_clips_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(&cutter, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Inside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Inside);
    assert_region_location(&result, p(13, 13), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_intersection_keeps_notched_material() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(
            &cutter,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Inside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Outside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_difference_merges_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(&cutter, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Outside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Outside);
    assert_region_location(&result, p(11, 11), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_xor_keeps_nested_island() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let xor_result = ring
        .boolean_region(&cutter, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap();
    let Classification::Decided(result) = xor_result else {
        panic!("expected decided donut xor hole-boundary cutter, got {xor_result:?}");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Inside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Outside);
    assert_region_location(&result, p(11, 11), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contours_hole_boundary_cutter_xor_are_decided() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(contours) = ring
        .boolean_boundary_contours(&cutter, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided xor boundary contours for hole-boundary cutter");
    };

    assert_eq!(contours.len(), 3);
    assert!(contours.iter().all(|contour| !contour.is_empty()));
}

#[test]
fn region_boolean_boundary_contours_identical_donut_are_decided() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(contours) = ring
        .boolean_boundary_contours(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self boundary contours");
    };

    assert_eq!(contours.len(), 2);
}

#[test]
fn region_boolean_boundary_contours_difference_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided difference boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(3, 1), ContourPointLocation::Outside);
}

#[test]
fn boundary_contour_nesting_alternates_material_hole_material() {
    let outer = rectangle(0, 0, 10, 10);
    let hole = rectangle(2, 2, 8, 8);
    let island = rectangle(4, 4, 6, 6);

    let Classification::Decided(result) =
        Region2::from_boundary_contours(vec![outer, hole, island], &policy()).unwrap()
    else {
        panic!("expected decided nested boundary region");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(3, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Inside);
}

#[test]
fn boundary_contour_nesting_report_audits_role_assignment() {
    let outer = rectangle(0, 0, 10, 10);
    let hole = rectangle(2, 2, 8, 8);
    let island = rectangle(4, 4, 6, 6);

    let Classification::Decided(report) =
        Region2::from_boundary_contours_report(vec![outer, hole, island], &policy()).unwrap()
    else {
        panic!("expected decided nested boundary report");
    };

    assert_eq!(report.audit.status, BoundaryContourNestingStatus::Valid);
    assert!(report.audit.is_valid());
    assert_eq!(report.audit.input_contour_count, 3);
    assert_eq!(report.audit.checked_containment_pair_count, 6);
    assert_eq!(report.audit.material_contour_count, 2);
    assert_eq!(report.audit.hole_contour_count, 1);
    assert_eq!(report.audit.contour_depths, vec![0, 1, 2]);
    assert_eq!(report.result.material_contours().len(), 2);
    assert_eq!(report.result.hole_contours().len(), 1);
    assert_region_location(&report.result, p(5, 5), RegionPointLocation::Inside);
}

#[test]
fn boundary_contour_nesting_rejects_boundary_touching_loops() {
    let outer = rectangle(0, 0, 4, 4);
    let touching = rectangle(1, 0, 3, 2);

    assert_eq!(
        Region2::from_boundary_contours(vec![outer.clone(), touching.clone()], &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        Region2::from_boundary_contours_report(vec![outer, touching], &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn region_boolean_boundary_contours_disjoint_union_keeps_two_loops() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided disjoint union boundary contours");
    };

    assert_eq!(contours.len(), 2);
    assert!(
        contours
            .iter()
            .any(|contour| contour.classify_point(&p(1, 1), &policy())
                == Classification::Decided(ContourPointLocation::Inside))
    );
    assert!(
        contours
            .iter()
            .any(|contour| contour.classify_point(&p(5, 5), &policy())
                == Classification::Decided(ContourPointLocation::Inside))
    );
}

#[test]
fn region_boolean_boundary_contours_disjoint_intersection_is_empty() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided empty intersection boundary contours");
    };

    assert!(contours.is_empty());
}

#[test]
fn region_boolean_region_point_touching_rectangles_use_regularized_identities() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch union");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 2), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 3), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided point-touch intersection");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch difference");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 3), RegionPointLocation::Outside);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch xor");
    };
    assert_eq!(xor.material_contours().len(), 2);
    assert_region_location(&xor, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&xor, p(3, 3), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contours_point_touching_rectangles_are_decided() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(union) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch union contours");
    };
    assert_eq!(union.len(), 2);

    let Classification::Decided(intersection) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided point-touch intersection contours");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch difference contours");
    };
    assert_eq!(difference.len(), 1);

    let Classification::Decided(xor) = first
        .boolean_boundary_contours(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch xor contours");
    };
    assert_eq!(xor.len(), 2);
}

#[test]
fn region_boolean_region_shared_edge_rectangles_are_regularized() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(3, -1), RegionPointLocation::Inside);
    assert_region_location(&union, p(3, 0), RegionPointLocation::Inside);
    assert_region_location(&union, p(5, 1), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided shared-edge intersection");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge difference");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, -1), RegionPointLocation::Outside);
    assert_region_location(&difference, p(3, 0), RegionPointLocation::Boundary);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge xor");
    };
    assert_eq!(xor.material_contours().len(), 1);
    assert_region_location(&xor, p(3, 0), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_full_edge_strip_xor_matches_prepared_regularization() {
    let first = region(vec![rectangle(47, -7, 60, -5)]);
    let second = region(vec![rectangle(47, -5, 60, -3)]);
    let policy = policy();
    let first_prepared = first.prepare_topology_queries(&policy);
    let second_prepared = second.prepare_topology_queries(&policy);

    for op in [BooleanOp::Union, BooleanOp::Xor] {
        let plain = first
            .boolean_region(&second, op, FillRule::NonZero, &policy)
            .unwrap();
        let prepared = first_prepared
            .boolean_region(&second_prepared, op, FillRule::NonZero, &policy)
            .unwrap();
        assert_eq!(prepared, plain);

        let Classification::Decided(result) = plain else {
            panic!("expected decided full-edge strip {op:?}");
        };
        assert_eq!(result.material_contours().len(), 1);
        assert_region_location(&result, p(50, -6), RegionPointLocation::Inside);
        assert_region_location(&result, p(50, -4), RegionPointLocation::Inside);
        assert_region_location(&result, p(50, -5), RegionPointLocation::Inside);
    }
}

#[test]
fn region_boolean_boundary_contours_shared_edge_rectangles_are_regularized() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    let Classification::Decided(union) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge union contours");
    };
    assert_eq!(union.len(), 1);

    let Classification::Decided(intersection) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided shared-edge intersection contours");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge difference contours");
    };
    assert_eq!(difference.len(), 1);

    let Classification::Decided(xor) = first
        .boolean_boundary_contours(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge xor contours");
    };
    assert_eq!(xor.len(), 1);
}

#[test]
fn region_boolean_region_boundary_overlap_with_interior_containment_identities() {
    let outer = region(vec![rectangle(0, 0, 6, 6)]);
    let inner_touching_edge = region(vec![rectangle(2, 0, 4, 2)]);

    let Classification::Decided(union) = outer
        .boolean_region(
            &inner_touching_edge,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("boundary-touching subset union should clone the container");
    };
    assert_eq!(union, outer);

    let Classification::Decided(intersection) = outer
        .boolean_region(
            &inner_touching_edge,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("boundary-touching subset intersection should clone the subset");
    };
    assert_eq!(intersection, inner_touching_edge);

    let Classification::Decided(reverse_difference) = inner_touching_edge
        .boolean_region(&outer, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("subset minus container should be empty");
    };
    assert!(reverse_difference.is_empty());

    let Classification::Decided(difference) = outer
        .boolean_region(
            &inner_touching_edge,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("container minus boundary-touching subset should rebuild a notched region");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert!(difference.hole_contours().is_empty());
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 1), RegionPointLocation::Outside);
    assert_region_location(&difference, p(3, 3), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 0), RegionPointLocation::Outside);
    assert_region_location(&difference, p(3, 2), RegionPointLocation::Boundary);
}

#[test]
fn region_boolean_boundary_contours_boundary_touching_containment_identities() {
    let outer = region(vec![rectangle(0, 0, 6, 6)]);
    let inner_touching_edge = region(vec![rectangle(2, 0, 4, 2)]);

    let Classification::Decided(union) = outer
        .boolean_boundary_contours(
            &inner_touching_edge,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("boundary-touching subset union contours should be decided");
    };
    assert_eq!(union, outer.material_contours());

    let Classification::Decided(intersection) = outer
        .boolean_boundary_contours(
            &inner_touching_edge,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("boundary-touching subset intersection contours should be decided");
    };
    assert_eq!(intersection, inner_touching_edge.material_contours());

    let Classification::Decided(reverse_difference) = inner_touching_edge
        .boolean_boundary_contours(&outer, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("subset minus container contours should be empty");
    };
    assert!(reverse_difference.is_empty());

    let Classification::Decided(difference) = outer
        .boolean_boundary_contours(
            &inner_touching_edge,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("container minus boundary-touching subset contours should be decided");
    };
    assert_eq!(difference.len(), 1);
    assert_contour_location(&difference[0], p(1, 1), ContourPointLocation::Inside);
    assert_contour_location(&difference[0], p(3, 1), ContourPointLocation::Outside);
    assert_contour_location(&difference[0], p(3, 3), ContourPointLocation::Inside);
    assert_contour_location(&difference[0], p(3, 0), ContourPointLocation::Outside);
    assert_contour_location(&difference[0], p(3, 2), ContourPointLocation::Boundary);
}

#[test]
fn region_boolean_difference_rebuilds_full_width_boundary_touching_strip() {
    let outer = region(vec![rectangle(0, 0, 6, 6)]);
    let bottom_strip = region(vec![rectangle(0, 0, 6, 2)]);

    let Classification::Decided(difference) = outer
        .boolean_region(
            &bottom_strip,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("full-width strip subtraction should produce the remaining rectangle");
    };

    assert_eq!(difference.material_contours().len(), 1);
    assert!(difference.hole_contours().is_empty());
    assert_region_location(&difference, p(3, 1), RegionPointLocation::Outside);
    assert_region_location(&difference, p(3, 3), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 2), RegionPointLocation::Boundary);
}

#[test]
fn region_boolean_difference_rebuilds_boundary_touching_donut_subset() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let cutter = donut(rectangle(2, 0, 8, 6), rectangle(4, 2, 6, 4));

    let Classification::Decided(difference) = outer
        .boolean_region(&cutter, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("boundary-touching donut subtraction should preserve the cutter hole as material");
    };

    assert_eq!(difference.material_contours().len(), 2);
    assert!(difference.hole_contours().is_empty());
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 1), RegionPointLocation::Outside);
    assert_region_location(&difference, p(5, 3), RegionPointLocation::Inside);
    assert_region_location(&difference, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&difference, p(9, 9), RegionPointLocation::Inside);
    assert_region_location(&difference, p(4, 3), RegionPointLocation::Boundary);
}

#[test]
fn region_boolean_boundary_pipeline_regularizes_shared_edges() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    let Classification::Decided(loops) =
        first.boolean_boundary_loops(&second, BooleanOp::Union, &policy()).unwrap()
    else {
        panic!("shared-edge union should be decided");
    };
    assert_eq!(loops.len(), 1);
}

#[test]
fn region_boolean_boundary_pipeline_regularizes_point_touch_branch_vertices() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(loops) =
        first.boolean_boundary_loops(&second, BooleanOp::Union, &policy()).unwrap()
    else {
        panic!("point-touch union should be decided");
    };
    assert_eq!(loops.len(), 2);
}

#[test]
fn region_boolean_prepared_boundary_reports_match_plain_on_hole_boundary_shared_strip() {
    let outer_with_hole = Region2::new(
        vec![rectangle(0, 0, 10, 10)],
        vec![rectangle(3, 3, 7, 7)],
    );
    // The strip lies in the removed hole interior and shares its entire
    // lower side with the hole boundary. According to Foster, Hormann, and
    // Popa (2019), full shared-edge boundary contacts should be regularized at
    // the boundary level before generic traversal, so this must stay decided
    // for prepared/plain and mixed-prepared call paths.
    let shared_strip = region(vec![rectangle(4, 3, 6, 5)]);
    let policy = policy();
    let prepared_outer = outer_with_hole.prepare_topology_queries(&policy);
    let prepared_strip = shared_strip.prepare_topology_queries(&policy);

    let plain_loops = match outer_with_hole
        .boolean_boundary_loops(&shared_strip, BooleanOp::Union, &policy)
        .unwrap()
    {
        Classification::Decided(loops) => loops,
        Classification::Uncertain(reason) => {
            panic!("hole-boundary shared strip union should be decided: {reason:?}")
        }
    };
    assert_eq!(
        prepared_outer
            .boolean_boundary_loops(&prepared_strip, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(plain_loops.clone())
    );
    assert_eq!(
        prepared_outer
            .boolean_boundary_loops_against_region(&shared_strip.as_view(), BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(plain_loops.clone())
    );
    assert_eq!(
        outer_with_hole
            .as_view()
            .boolean_boundary_loops_against_prepared_region(&prepared_strip, BooleanOp::Union, &policy)
            .unwrap(),
        Classification::Decided(plain_loops.clone())
    );

    let plain_contours = match outer_with_hole
        .boolean_boundary_contours(&shared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
        .unwrap()
    {
        Classification::Decided(contours) => contours,
        Classification::Uncertain(reason) => {
            panic!("hole-boundary shared strip union contours should be decided: {reason:?}")
        }
    };
    assert_eq!(
        prepared_outer
            .boolean_boundary_contours(&prepared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
            .unwrap(),
        Classification::Decided(plain_contours.clone())
    );
    for contour in plain_contours {
        assert_contour_finite(&contour);
    }

    let plain_loop_report = match outer_with_hole
        .boolean_boundary_loop_report(&shared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
        .unwrap()
    {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("hole-boundary shared strip loop report should be decided: {reason:?}")
        }
    };
    assert_eq!(&plain_loop_report.loops, &plain_loops);
    assert!(plain_loop_report.audit.is_valid());
    assert_eq!(
        prepared_outer
            .boolean_boundary_loop_report(&prepared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
            .unwrap(),
        Classification::Decided(plain_loop_report.clone())
    );

    let plain_region_report = match outer_with_hole
        .boolean_region_report(&shared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
        .unwrap()
    {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("hole-boundary shared strip region report should be decided: {reason:?}")
        }
    };
    assert!(plain_region_report.audit.is_valid());
    assert_eq!(
        prepared_outer
            .boolean_region_report(&prepared_strip, BooleanOp::Union, FillRule::NonZero, &policy)
            .unwrap(),
        Classification::Decided(plain_region_report.clone())
    );
    assert_eq!(
        prepared_outer
            .boolean_region_report_against_region(&shared_strip.as_view(), BooleanOp::Union, FillRule::NonZero, &policy)
            .unwrap(),
        Classification::Decided(plain_region_report.clone())
    );
}
