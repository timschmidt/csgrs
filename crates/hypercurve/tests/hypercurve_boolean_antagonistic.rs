//! Antagonistic boolean tests for `hypercurve`.
//!
//! These tests intentionally exercise degenerate contact cases as first-class
//! geometric facts, following C. K. Yap's exact-geometric-computation contract:
//! "exactness is a property of geometric constructions, not an implicit property
//! of scalar values" (*Towards Exact Geometric Computation*, 1997).
use hypercurve::{
    BooleanBoundaryLoopReport2, BooleanBoundaryTraversalReport2, BooleanBoundaryTraversalStatus,
    BooleanOp, BooleanRegionPipelineReport2, BooleanRegionReport2, BulgeVertex2, Classification,
    Contour2, CurvePolicy, FillRule, Region2, Real, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
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

fn region(contours: Vec<Contour2>) -> Region2 {
    Region2::from_material_contours(contours)
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

/// Validates traversal-report invariants shared by all degeneracy cases.
///
/// This mirrors the traversal contract in
/// `BooleanBoundaryTraversalReport2`, and asserts explicit blocker accounting
/// from boundary-classification to chain assembly (Yap, 1997).
fn expect_traversal_report_invariants(
    report: &BooleanBoundaryTraversalReport2,
    expected_status: BooleanBoundaryTraversalStatus,
    expected_blocker: Option<UncertaintyReason>,
) {
    assert_eq!(report.status, expected_status);
    assert_eq!(report.blocker_reason, expected_blocker);
    assert_eq!(
        report.is_ready(),
        matches!(
            expected_status,
            BooleanBoundaryTraversalStatus::Empty | BooleanBoundaryTraversalStatus::LoopsReady
        )
    );
    assert_eq!(report.loops.is_some(), report.is_ready());
    assert_eq!(
        report.classified_fragment_count,
        report.discarded_fragment_count
            + report.kept_source_direction_count
            + report.kept_reversed_count
            + report.unresolved_boundary_count,
    );
    assert_eq!(
        report.directed_fragment_count,
        report.kept_source_direction_count + report.kept_reversed_count,
    );
    if report.is_ready() {
        assert_eq!(report.open_chain_count, 0);
        assert_eq!(
            report.closed_chain_count + report.open_chain_count,
            report.assembled_chain_count
        );
    }
}

/// Asserts that a loop report is internally auditable for its API operation.
///
/// The loop-report audit is the boundary-product replay stage emphasized by
/// Greiner & Hormann's clipping traversal model (1998).
fn assert_loop_report_is_audited(report: &BooleanBoundaryLoopReport2) {
    assert_eq!(report.operation, BooleanOp::Union);
    assert!(report.audit.is_valid());
}

/// Asserts that a region report can be consumed as a certified boundary result.
///
/// This keeps Yap's "object-level certification" rule explicit in test fixtures.
fn assert_region_report_is_audited(report: &BooleanRegionReport2) {
    assert_eq!(report.operation, BooleanOp::Union);
    assert!(report.audit.is_valid());
}

/// Asserts that a pipeline report includes all required stage-audit witnesses.
///
/// The pipeline combines boundary contour, nesting, and final-region audits.
fn assert_pipeline_report_is_audited(report: &BooleanRegionPipelineReport2) {
    assert_eq!(report.operation, BooleanOp::Union);
    assert!(report.boundary_audit.is_valid());
    assert!(report.nesting_audit.is_valid());
    assert!(report.region_audit.is_valid());
}

/// Asserts traversal parity across plain, prepared, and mixed call surfaces.
///
/// Degenerate boundary semantics are expected to be stable across dispatch modes.
/// Shared-boundary, unsupported traversal, and ready statuses are validated
/// explicitly, following Foster, Hormann, and Popa (2019).
fn assert_boundary_traversal_parity(
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    policy: &CurvePolicy,
    expected_status: BooleanBoundaryTraversalStatus,
    expected_blocker: Option<UncertaintyReason>,
) {
    let plain = match first
        .boolean_boundary_traversal_report(second, op, policy)
        .unwrap()
    {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("plain traversal report should be decided: {reason:?}");
        }
    };
    expect_traversal_report_invariants(&plain, expected_status, expected_blocker);
    let first_prepared = first.prepare_topology_queries(policy);
    let second_prepared = second.prepare_topology_queries(policy);

    assert_eq!(
        first_prepared
            .boolean_boundary_traversal_report(&second_prepared, op, policy)
            .unwrap(),
        Classification::Decided(plain.clone())
    );
    assert_eq!(
        first_prepared
            .boolean_boundary_traversal_report_against_region(&second.as_view(), op, policy)
            .unwrap(),
        Classification::Decided(plain.clone())
    );
    assert_eq!(
        first
            .as_view()
            .boolean_boundary_traversal_report_against_prepared_region(
                &second_prepared,
                op,
                policy,
            )
            .unwrap(),
        Classification::Decided(plain)
    );
}

/// Asserts report parity across region, contour, loop, and pipeline report APIs.
///
/// This function verifies the full report surface is consistent for plain/prepared
/// and mixed dispatch, and that report outputs remain self-consistent across the
/// pipeline boundary.
fn assert_report_parity(
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) {
    let first_prepared = first.prepare_topology_queries(policy);
    let second_prepared = second.prepare_topology_queries(policy);

    let plain_region = match first.boolean_region(second, op, fill_rule, policy).unwrap() {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => {
            panic!("plain region boolean should be decided: {reason:?}");
        }
    };
    let plain_boundary_contour_report =
        match first.boolean_boundary_contour_report(second, op, fill_rule, policy).unwrap() {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("plain boundary-contour report should be decided: {reason:?}");
            }
        };
    let plain_boundary_loop_report =
        match first
            .boolean_boundary_loop_report(second, op, fill_rule, policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("plain boundary-loop report should be decided: {reason:?}");
            }
        };
    let plain_region_report = match first.boolean_region_report(second, op, fill_rule, policy).unwrap() {
        Classification::Decided(report) => report,
        Classification::Uncertain(reason) => {
            panic!("plain region report should be decided: {reason:?}");
        }
    };
    let plain_pipeline_report =
        match first
            .boolean_region_pipeline_report(second, op, fill_rule, policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("plain pipeline report should be decided: {reason:?}");
            }
        };

    assert_loop_report_is_audited(&plain_boundary_loop_report);
    assert_region_report_is_audited(&plain_region_report);
    assert_pipeline_report_is_audited(&plain_pipeline_report);
    assert_eq!(&plain_region_report.result, &plain_region);
    assert_eq!(
        &plain_pipeline_report.result,
        &plain_region,
        "pipeline region must match plain region boolean result"
    );
    assert_eq!(
        &plain_pipeline_report.boundary_contours,
        &plain_boundary_contour_report.contours
    );

    assert_eq!(
        first_prepared
            .boolean_boundary_contour_report(&second_prepared, op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_boundary_contour_report.clone())
    );
    assert_eq!(
        first_prepared
            .boolean_boundary_contour_report_against_region(&second.as_view(), op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_boundary_contour_report.clone())
    );
    assert_eq!(
        first
            .as_view()
            .boolean_boundary_contour_report_against_prepared_region(
                &second_prepared,
                op,
                fill_rule,
                policy,
            )
            .unwrap(),
        Classification::Decided(plain_boundary_contour_report.clone())
    );

    assert_eq!(
        first_prepared
            .boolean_boundary_loop_report(&second_prepared, op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_boundary_loop_report.clone())
    );
    assert_eq!(
        first_prepared
            .boolean_boundary_loop_report_against_region(&second.as_view(), op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_boundary_loop_report.clone())
    );
    assert_eq!(
        first
            .as_view()
            .boolean_boundary_loop_report_against_prepared_region(
                &second_prepared,
                op,
                fill_rule,
                policy,
            )
            .unwrap(),
        Classification::Decided(plain_boundary_loop_report.clone())
    );

    assert_eq!(
        first_prepared
            .boolean_region_report(&second_prepared, op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_region_report.clone())
    );
    assert_eq!(
        first_prepared
            .boolean_region_report_against_region(&second.as_view(), op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_region_report.clone())
    );
    assert_eq!(
        first
            .as_view()
            .boolean_region_report_against_prepared_region(&second_prepared, op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_region_report.clone())
    );

    assert_eq!(
        first_prepared
            .boolean_region_pipeline_report(&second_prepared, op, fill_rule, policy)
            .unwrap(),
        Classification::Decided(plain_pipeline_report.clone())
    );
    assert_eq!(
        first_prepared
            .boolean_region_pipeline_report_against_region(
                &second.as_view(),
                op,
                fill_rule,
                policy
            )
            .unwrap(),
        Classification::Decided(plain_pipeline_report.clone())
    );
    assert_eq!(
        first
            .as_view()
            .boolean_region_pipeline_report_against_prepared_region(
                &second_prepared,
                op,
                fill_rule,
                policy,
            )
            .unwrap(),
        Classification::Decided(plain_pipeline_report)
    );
}

#[test]
/// Degenerate-contact blockers are first-class API facts, not hidden errors.
///
/// The traversal surface follows Yap's exact-combinatorics doctrine
/// (Yap, *Towards Exact Geometric Computation*, 1997), while explicit boundary
/// blockers follow Foster, Hormann, and Popa's boundary-contact handling model
/// (2019).
fn antagonistic_boundary_traversal_blockers_match_prepared_and_mixed_paths() {
    let policy = policy();
    let op = BooleanOp::Union;
    let shared_edge_first = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_second = region(vec![rectangle(2, -2, 6, 0)]);
    let point_touch_first = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_second = region(vec![rectangle(2, 2, 4, 4)]);
    let overlapping_first = region(vec![rectangle(0, 0, 4, 4)]);
    let overlapping_second = region(vec![rectangle(2, -1, 6, 3)]);
    let disjoint_first = region(vec![rectangle(0, 0, 2, 2)]);
    let disjoint_second = region(vec![rectangle(6, 6, 8, 8)]);

    assert_boundary_traversal_parity(
        &shared_edge_first,
        &shared_edge_second,
        op,
        &policy,
        BooleanBoundaryTraversalStatus::UnresolvedBoundaries,
        Some(UncertaintyReason::Boundary),
    );
    assert_boundary_traversal_parity(
        &point_touch_first,
        &point_touch_second,
        op,
        &policy,
        BooleanBoundaryTraversalStatus::UnsupportedTraversal,
        Some(UncertaintyReason::Unsupported),
    );
    assert_boundary_traversal_parity(
        &overlapping_first,
        &overlapping_second,
        op,
        &policy,
        BooleanBoundaryTraversalStatus::LoopsReady,
        None,
    );
    assert_boundary_traversal_parity(
        &disjoint_first,
        &disjoint_second,
        op,
        &policy,
        BooleanBoundaryTraversalStatus::LoopsReady,
        None,
    );
}

#[test]
/// Report surfaces stay auditable and parity-complete on antagonistic inputs.
///
/// Loop and region report layers are compared across plain, prepared, and mixed
/// call paths, matching the evidence-carrying model in Yap (1997) and
/// boundary-contact regularization in Foster, Hormann, and Popa (2019).
fn antagonistic_report_surfaces_are_auditable_and_parity_complete() {
    let policy = policy();
    let fill_rule = FillRule::NonZero;
    let shared_edge_first = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_second = region(vec![rectangle(2, -2, 6, 0)]);
    let point_touch_first = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_second = region(vec![rectangle(2, 2, 4, 4)]);
    let hole_outer = Region2::new(
        vec![rectangle(0, 0, 12, 12)],
        vec![rectangle(4, 4, 8, 8)],
    );
    let hole_strip = region(vec![rectangle(6, 2, 10, 10)]);

    assert_report_parity(
        &shared_edge_first,
        &shared_edge_second,
        BooleanOp::Union,
        fill_rule,
        &policy,
    );
    assert_report_parity(
        &point_touch_first,
        &point_touch_second,
        BooleanOp::Union,
        fill_rule,
        &policy,
    );
    assert_report_parity(
        &hole_outer,
        &hole_strip,
        BooleanOp::Union,
        fill_rule,
        &policy,
    );
}
