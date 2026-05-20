use hypercurve::{
    Axis2, BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2,
    BezierBooleanArrangementReadinessReport2, BezierBooleanArrangementReadinessStatus,
    BezierBooleanAssemblyReadinessReport2, BezierBooleanAssemblyReadinessStatus,
    BezierBooleanBatchHandoffReport2, BezierBooleanBatchHandoffStatus,
    BezierBooleanConstructionReadinessReport2, BezierBooleanConstructionReadinessStatus,
    BezierBooleanCubicFragmentReport2, BezierBooleanEmissionPlanReport2,
    BezierBooleanEmissionPlanStatus, BezierBooleanFragmentConstructionStatus,
    BezierBooleanFragmentOwnershipLocation, BezierBooleanHandoffReport2,
    BezierBooleanHandoffStatus, BezierBooleanLoopAssemblyPlanReport2,
    BezierBooleanLoopAssemblyPlanStatus, BezierBooleanLoopClosureReport2,
    BezierBooleanLoopClosureStatus, BezierBooleanLoopContainmentFact2,
    BezierBooleanLoopContainmentFactReport2, BezierBooleanLoopContainmentFactStatus,
    BezierBooleanLoopGraphFactReport2, BezierBooleanLoopGraphFactStatus,
    BezierBooleanLoopGraphFacts2, BezierBooleanLoopGraphTraversalReport2,
    BezierBooleanLoopGraphTraversalStatus, BezierBooleanLoopGraphWalkReport2,
    BezierBooleanLoopGraphWalkStatus, BezierBooleanLoopNestingDepthFact2,
    BezierBooleanLoopNestingDepthFactReport2, BezierBooleanLoopNestingDepthFactStatus,
    BezierBooleanLoopNestingRoleReport2, BezierBooleanLoopNestingRoleStatus,
    BezierBooleanLoopRoleAssignmentReport2, BezierBooleanLoopRoleAssignmentStatus,
    BezierBooleanOutputLoopReport2, BezierBooleanOutputLoopRole, BezierBooleanOutputLoopStatus,
    BezierBooleanOverlapResolutionReport2, BezierBooleanOverlapResolutionStatus,
    BezierBooleanOwnershipClassificationReport2, BezierBooleanOwnershipClassificationStatus,
    BezierBooleanOwnershipFact2, BezierBooleanOwnershipFactReport2,
    BezierBooleanOwnershipFactStatus, BezierBooleanPathSchedulerReport2,
    BezierBooleanPathSchedulerStatus, BezierBooleanQuadraticFragmentReport2,
    BezierBooleanRationalQuadraticFragmentReport2, BezierBooleanRegionAssemblyReport2,
    BezierBooleanRegionAssemblyStatus, BezierBooleanResultReport2, BezierBooleanResultStatus,
    BezierBooleanSplitInsertionStatus, BezierBooleanSplitPlanAuditStatus,
    BezierBooleanSplitPlanReport2, BezierBooleanSplitPlanStatus, BezierBooleanTraversalOperand,
    BezierBooleanTraversalPreconditionReport2, BezierBooleanTraversalPreconditionStatus,
    BezierBooleanTraversalScheduleReport2, BezierBooleanTraversalScheduleStatus,
    BezierBooleanUniformOwnershipFactReport2, BezierBooleanUniformOwnershipFactStatus,
    BezierCurveRelation, BezierCuspClassification, BezierDegree, BezierEndpoint,
    BezierFitBoundKind, BezierFitErrorMetric, BezierFitReadinessStatus,
    BezierFitSourceBatchReport2, BezierFitSourcePrefixSums2, BezierFlatteningOptions,
    BezierGraphContact, BezierInflectionClassification, BezierIntersectionRegionIsolationBudget,
    BezierIntersectionRegionIsolationStopReason, BezierIntersectionRegionRefinementAction,
    BezierIntersectionRegionShape, BezierLineContactKind, BezierLineContactRelation,
    BezierLineFitRelation, BezierLineImageFitRelation, BezierLineRelation,
    BezierMonotoneGraphContactOrder, BezierMonotoneGraphOrder, BezierMonotoneSpan,
    BezierOffsetAdapterStatus, BezierOffsetCandidate2, BezierOffsetRisk,
    BezierPathRangeBatchReport2, BezierPathRangeBatchStatus, BezierPathRangeOrderReport2,
    BezierPathRangeOrderStatus, BezierPointFitRelation, BezierPointImageFitRelation,
    BezierRegionWidthStatus, BezierSimplificationBoundKind, BezierSimplificationErrorMetric,
    BooleanFragmentAction, BooleanOp, Classification, CubicBezier2, CurvePolicy, IntersectionKind,
    LineLineIntersection, LineSeg2, NumericMode, ParamRange, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, RationalQuadraticConicKind, Real, SymbolicDependencyMask,
    UncertaintyReason, ZeroStatus, bezier_intersection_region_facts,
    certify_bezier_intersection_region_isolation, isolate_bezier_intersection_regions,
    isolate_bezier_intersection_regions_until_width, refine_bezier_intersection_region,
    refine_bezier_intersection_regions, summarize_bezier_intersection_regions,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn ratio(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn half() -> Real {
    (Real::one() / Real::from(2_i8)).unwrap()
}

fn certifies_zero(value: Real) -> bool {
    matches!(
        value.refine_sign_until(-64),
        Some(hypercurve::RealSign::Zero)
    )
}

fn quadratic_through_point_at(
    parameter: Real,
    target: &Point2,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
) -> QuadraticBezier2 {
    let one_minus_t = Real::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t;
    let b1 = Real::from(2_i8) * &parameter * &one_minus_t;
    let b2 = &parameter * &parameter;
    let p0 = Point2::new(
        target.x() + &Real::from(first_offset.0),
        target.y() + &Real::from(first_offset.1),
    );
    let p1 = Point2::new(
        target.x() + &Real::from(second_offset.0),
        target.y() + &Real::from(second_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y());
    let p2_x = (numerator_x / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    let p2_y = (numerator_y / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    QuadraticBezier2::new(p0, p1, Point2::new(p2_x, p2_y))
}

fn cubic_through_point_at(
    parameter: Real,
    target: &Point2,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
    third_offset: (i32, i32),
) -> CubicBezier2 {
    let one_minus_t = Real::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t * &one_minus_t;
    let b1 = Real::from(3_i8) * &parameter * &one_minus_t * &one_minus_t;
    let b2 = Real::from(3_i8) * &parameter * &parameter * &one_minus_t;
    let b3 = &parameter * &parameter * &parameter;
    let p0 = Point2::new(
        target.x() + &Real::from(first_offset.0),
        target.y() + &Real::from(first_offset.1),
    );
    let p1 = Point2::new(
        target.x() + &Real::from(second_offset.0),
        target.y() + &Real::from(second_offset.1),
    );
    let p2 = Point2::new(
        target.x() + &Real::from(third_offset.0),
        target.y() + &Real::from(third_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x()) - &(&b2 * p2.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y()) - &(&b2 * p2.y());
    let p3_x = (numerator_x / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    let p3_y = (numerator_y / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    CubicBezier2::new(p0, p1, p2, Point2::new(p3_x, p3_y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_same_parameter_regions_include_exact(
    regions: &[hypercurve::BezierCurveIntersectionRegion],
    parameter: &Real,
) {
    assert!(
        regions.iter().any(|region| {
            region.first() == region.second()
                && region.first().start() == parameter
                && region.first().end() == parameter
        }),
        "expected same-parameter zero-width region at {parameter:?}: {regions:?}"
    );
}

fn assert_same_parameter_regions_include_bracket(
    regions: &[hypercurve::BezierCurveIntersectionRegion],
) {
    assert!(
        regions.iter().any(|region| {
            region.first() == region.second() && region.first().start() != region.first().end()
        }),
        "expected at least one nonzero same-parameter isolating bracket: {regions:?}"
    );
}

fn span(start: Real, end: Real) -> BezierMonotoneSpan {
    BezierMonotoneSpan::new(start, end)
}

#[test]
fn quadratic_bezier_facts_preserve_exact_control_polygon_shape() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let facts = curve.structural_facts();

    assert_eq!(facts.degree, BezierDegree::Quadratic);
    assert_eq!(facts.degree.control_point_count(), 3);
    assert!(facts.all_exact_rational());
    assert!(facts.has_shared_denominator_schedule());
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b10);
    assert_eq!(facts.derivative_known_zero_mask, 0b1010);
    assert!(facts.derivative_y_components_known_zero());
    assert_eq!(facts.second_difference_known_zero_mask, 0b11);
    assert_eq!(facts.curvature_known_zero_mask, 0b1);
}

#[test]
fn cubic_bezier_facts_retain_inflection_candidates_without_float_predicates() {
    let curve = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));
    let facts = curve.structural_facts();

    assert_eq!(facts.degree, BezierDegree::Cubic);
    assert_eq!(facts.degree.control_point_count(), 4);
    assert!(facts.all_exact_rational());
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b10);
    assert_ne!(
        facts.curvature_known_zero_mask, 0,
        "parallel derivative-edge pairs should be retained as exact zero witnesses"
    );
    assert_ne!(facts.curvature_known_nonzero_mask, 0);
}

#[test]
fn symbolic_bezier_control_points_are_reported_as_object_facts() {
    let curve = QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::pi(), Real::one()),
        Point2::new(Real::from(2_i8), Real::zero()),
    );
    let facts = curve.structural_facts();

    assert!(!facts.all_exact_rational());
    assert!(
        facts
            .symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
    assert_eq!(facts.endpoint_coincidence, ZeroStatus::NonZero);
}

#[test]
fn degenerate_loop_bezier_reports_coincident_endpoints_but_preserves_controls() {
    let curve = CubicBezier2::new(point(0, 0), point(5, 1), point(-3, 1), point(0, 0));
    let facts = curve.structural_facts();

    assert_eq!(curve.endpoints_coincident_status(), ZeroStatus::Zero);
    assert!(facts.endpoints_known_coincident());
    assert_eq!(facts.endpoint_delta_known_zero_mask, 0b11);
    assert_ne!(
        facts.derivative_known_nonzero_mask, 0,
        "coincident endpoints must not erase the nonzero interior control polygon"
    );
}

#[test]
fn bezier_point_at_uses_exact_de_casteljau_midpoints() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    assert_eq!(quadratic.point_at(half()), point(2, 2));

    let cubic = CubicBezier2::new(point(0, 0), point(0, 4), point(4, 4), point(4, 0));
    assert_eq!(cubic.point_at(half()), point(2, 3));
}

#[test]
fn endpoint_tangent_classification_uses_exact_derivative_vectors() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(8, 4));
    let start = quadratic.endpoint_tangent(BezierEndpoint::Start);
    let end = quadratic.endpoint_tangent(BezierEndpoint::End);

    assert_eq!(start.dx(), &r(6));
    assert_eq!(start.dy(), &r(8));
    assert_eq!(start.zero_status(), ZeroStatus::NonZero);
    assert_eq!(end.dx(), &r(10));
    assert_eq!(end.dy(), &r(0));
    assert_eq!(end.zero_status(), ZeroStatus::NonZero);

    let degenerate = CubicBezier2::new(point(0, 0), point(0, 0), point(2, 0), point(5, 0));
    let start = degenerate.endpoint_tangent(BezierEndpoint::Start);
    let end = degenerate.endpoint_tangent(BezierEndpoint::End);

    assert_eq!(start.zero_status(), ZeroStatus::Zero);
    assert_eq!(end.dx(), &r(9));
    assert_eq!(end.dy(), &r(0));
    assert_eq!(end.zero_status(), ZeroStatus::NonZero);
}

#[test]
fn parameterized_point_on_bezier_is_certified_exactly() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    assert_eq!(
        quadratic.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        quadratic.contains_point_at_parameter(&point(2, 3), half(), &policy()),
        Classification::Decided(false)
    );

    let cubic = CubicBezier2::new(point(0, 0), point(0, 4), point(4, 4), point(4, 0));
    assert_eq!(
        cubic.contains_point_at_parameter(&point(2, 3), half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        cubic.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(false)
    );
}

#[test]
fn bezier_intersection_region_facts_classify_exact_region_shapes() {
    let exact =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let bracket = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let product = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 8), ratio(3, 8)),
    );
    let invalid =
        hypercurve::BezierCurveIntersectionRegion::new(span(r(1), r(0)), span(r(0), r(1)));

    let exact_facts = bezier_intersection_region_facts(&exact);
    let bracket_facts = bezier_intersection_region_facts(&bracket);
    let product_facts = bezier_intersection_region_facts(&product);
    let invalid_facts = bezier_intersection_region_facts(&invalid);

    assert_eq!(
        exact_facts.shape,
        BezierIntersectionRegionShape::ExactPointCell
    );
    assert_eq!(
        exact_facts.first_width_status,
        BezierRegionWidthStatus::Zero
    );
    assert_eq!(
        exact_facts.second_width_status,
        BezierRegionWidthStatus::Zero
    );
    assert_eq!(
        bracket_facts.shape,
        BezierIntersectionRegionShape::SameParameterIsolatingSpan
    );
    assert_eq!(bracket_facts.same_parameter_span, Some(true));
    assert_eq!(bracket_facts.first_width, ratio(1, 4));
    assert_eq!(
        product_facts.shape,
        BezierIntersectionRegionShape::ProductCell
    );
    assert_eq!(product_facts.same_parameter_span, Some(false));
    assert_eq!(
        invalid_facts.shape,
        BezierIntersectionRegionShape::InvalidSpan
    );
    assert_eq!(
        invalid_facts.first_width_status,
        BezierRegionWidthStatus::Negative
    );
}

#[test]
fn bezier_intersection_region_summary_counts_exact_shapes() {
    let regions = [
        hypercurve::BezierCurveIntersectionRegion::new(span(r(0), r(0)), span(r(0), r(0))),
        hypercurve::BezierCurveIntersectionRegion::new(
            span(ratio(1, 4), ratio(1, 2)),
            span(ratio(1, 4), ratio(1, 2)),
        ),
        hypercurve::BezierCurveIntersectionRegion::new(
            span(ratio(1, 4), ratio(1, 2)),
            span(ratio(1, 8), ratio(3, 8)),
        ),
    ];

    let summary = summarize_bezier_intersection_regions(&regions);

    assert_eq!(summary.region_count, 3);
    assert_eq!(summary.exact_point_cells, 1);
    assert_eq!(summary.same_parameter_isolating_spans, 1);
    assert_eq!(summary.product_cells, 1);
    assert_eq!(summary.invalid_spans, 0);
    assert_eq!(summary.unknown_regions, 0);
}

#[test]
fn bezier_intersection_region_refinement_splits_exact_parameter_cells() {
    let exact =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let bracket = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let wider_first = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 8), ratio(7, 8)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let equal_product = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 8), ratio(3, 8)),
    );
    let invalid =
        hypercurve::BezierCurveIntersectionRegion::new(span(r(1), r(0)), span(r(0), r(1)));

    let exact_refinement = refine_bezier_intersection_region(&exact);
    let bracket_refinement = refine_bezier_intersection_region(&bracket);
    let wider_first_refinement = refine_bezier_intersection_region(&wider_first);
    let equal_product_refinement = refine_bezier_intersection_region(&equal_product);
    let invalid_refinement = refine_bezier_intersection_region(&invalid);

    assert_eq!(
        exact_refinement.action,
        BezierIntersectionRegionRefinementAction::RetainExactPoint
    );
    assert_eq!(exact_refinement.children, vec![exact]);
    assert_eq!(
        bracket_refinement.action,
        BezierIntersectionRegionRefinementAction::BisectBothSpans
    );
    assert_eq!(bracket_refinement.first_midpoint, Some(ratio(3, 8)));
    assert_eq!(bracket_refinement.second_midpoint, Some(ratio(3, 8)));
    assert_eq!(bracket_refinement.children.len(), 2);
    assert_eq!(
        wider_first_refinement.action,
        BezierIntersectionRegionRefinementAction::BisectFirstSpan
    );
    assert_eq!(wider_first_refinement.first_midpoint, Some(half()));
    assert_eq!(wider_first_refinement.children.len(), 2);
    assert_eq!(
        equal_product_refinement.action,
        BezierIntersectionRegionRefinementAction::BisectBothSpans
    );
    assert_eq!(equal_product_refinement.children.len(), 4);
    assert_eq!(
        invalid_refinement.action,
        BezierIntersectionRegionRefinementAction::RejectInvalidSpan
    );
    assert!(invalid_refinement.children.is_empty());
}

#[test]
fn bezier_intersection_region_batch_refinement_preserves_input_order() {
    let first =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let second = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );

    let refinements = refine_bezier_intersection_regions(&[first, second]);

    assert_eq!(refinements.len(), 2);
    assert_eq!(
        refinements[0].action,
        BezierIntersectionRegionRefinementAction::RetainExactPoint
    );
    assert_eq!(
        refinements[1].action,
        BezierIntersectionRegionRefinementAction::BisectBothSpans
    );
}

#[test]
fn bezier_intersection_region_isolation_reports_depth_limited_frontier() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let report = isolate_bezier_intersection_regions(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 8,
            max_depth: 2,
            max_terminal_regions: 8,
        },
    );

    assert_eq!(
        report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::WorklistExhausted
    );
    assert_eq!(report.steps, 7);
    assert_eq!(report.terminal_regions.len(), 4);
    assert_eq!(report.exact_point_cells, 0);
    assert_eq!(report.rejected_invalid_spans, 0);
    assert_eq!(report.deferred_unknown_regions, 0);
    assert_eq!(report.refinements.len(), 7);
    assert_eq!(report.terminal_regions[0].first().start(), &ratio(1, 4));
    assert_eq!(report.terminal_regions[3].first().end(), &ratio(1, 2));
}

#[test]
fn bezier_intersection_region_isolation_reports_step_and_terminal_budgets() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 8), ratio(3, 8)),
    );
    let step_report = isolate_bezier_intersection_regions(
        std::slice::from_ref(&region),
        BezierIntersectionRegionIsolationBudget {
            max_steps: 1,
            max_depth: 8,
            max_terminal_regions: 8,
        },
    );
    let terminal_report = isolate_bezier_intersection_regions(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 16,
            max_depth: 1,
            max_terminal_regions: 0,
        },
    );

    assert_eq!(
        step_report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::StepBudgetReached
    );
    assert_eq!(step_report.steps, 1);
    assert_eq!(
        terminal_report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::TerminalRegionBudgetReached
    );
    assert!(terminal_report.terminal_regions.is_empty());
}

#[test]
fn bezier_intersection_region_isolation_refines_until_width_target() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let report = isolate_bezier_intersection_regions_until_width(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 16,
            max_depth: 8,
            max_terminal_regions: 16,
        },
        ratio(1, 16),
    );

    assert_eq!(
        report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::TargetWidthReached
    );
    assert_eq!(report.target_max_span_width, Some(ratio(1, 16)));
    assert_eq!(report.terminal_regions.len(), 4);
    assert_eq!(report.target_satisfied_terminal_regions, 4);
    assert_eq!(report.target_unmet_terminal_regions, 0);
    assert!(report.terminal_regions.iter().all(|region| {
        let facts = bezier_intersection_region_facts(region);
        matches!(
            (&facts.first_width - &ratio(1, 16)).refine_sign_until(-64),
            Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
        ) && matches!(
            (&facts.second_width - &ratio(1, 16)).refine_sign_until(-64),
            Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
        )
    }));
}

#[test]
fn bezier_intersection_region_isolation_certificate_summarizes_terminal_frontier() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let report = isolate_bezier_intersection_regions_until_width(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 64,
            max_depth: 4,
            max_terminal_regions: 64,
        },
        ratio(1, 16),
    );

    let certificate = certify_bezier_intersection_region_isolation(&report);

    assert_eq!(
        certificate.stop_reason,
        BezierIntersectionRegionIsolationStopReason::TargetWidthReached
    );
    assert!(certificate.target_width_satisfied);
    assert_eq!(certificate.target_max_span_width, Some(ratio(1, 16)));
    assert_eq!(
        certificate.terminal_region_count,
        report.terminal_regions.len()
    );
    assert_eq!(
        certificate.terminal_summary.region_count,
        report.terminal_regions.len()
    );
    assert_eq!(
        certificate.terminal_summary.same_parameter_isolating_spans,
        report.terminal_regions.len()
    );
    assert!(certificate.all_terminal_widths_certified);
    assert!(matches!(
        (&certificate.max_first_width.unwrap() - &ratio(1, 16)).refine_sign_until(-64),
        Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
    ));
    assert!(matches!(
        (&certificate.max_second_width.unwrap() - &ratio(1, 16)).refine_sign_until(-64),
        Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
    ));
}

#[test]
fn bezier_intersection_region_isolation_certificate_marks_unmet_targets() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let report = isolate_bezier_intersection_regions_until_width(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 4,
            max_depth: 1,
            max_terminal_regions: 64,
        },
        ratio(1, 64),
    );

    let certificate = certify_bezier_intersection_region_isolation(&report);

    assert!(!certificate.target_width_satisfied);
    assert_eq!(certificate.terminal_region_count, 2);
    assert!(certificate.all_terminal_widths_certified);
    assert!(matches!(
        (&certificate.max_first_width.unwrap() - &ratio(1, 64)).refine_sign_until(-64),
        Some(hypercurve::RealSign::Positive)
    ));
    assert_eq!(certificate.source_steps, report.steps);
}

#[test]
fn bezier_intersection_region_isolation_reports_unmet_and_invalid_width_targets() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let depth_report = isolate_bezier_intersection_regions_until_width(
        std::slice::from_ref(&region),
        BezierIntersectionRegionIsolationBudget {
            max_steps: 16,
            max_depth: 1,
            max_terminal_regions: 16,
        },
        ratio(1, 16),
    );
    let invalid_report = isolate_bezier_intersection_regions_until_width(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 16,
            max_depth: 1,
            max_terminal_regions: 16,
        },
        -Real::one(),
    );

    assert_eq!(
        depth_report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::WorklistExhausted
    );
    assert_eq!(depth_report.target_satisfied_terminal_regions, 0);
    assert_eq!(depth_report.target_unmet_terminal_regions, 2);
    assert_eq!(
        invalid_report.stop_reason,
        BezierIntersectionRegionIsolationStopReason::InvalidTargetWidth
    );
    assert_eq!(invalid_report.steps, 0);
    assert!(invalid_report.terminal_regions.is_empty());
}

#[test]
fn quadratic_point_on_curve_solves_exact_parameters() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(parameters) = arch.parameters_for_point(&point(2, 2), &policy())
    else {
        panic!("integer midpoint on quadratic should have certified parameters");
    };
    assert!(
        !parameters.is_empty(),
        "quadratic point solver should retain at least one exact parameter"
    );
    for parameter in parameters {
        assert_eq!(
            arch.contains_point_at_parameter(&point(2, 2), parameter, &policy()),
            Classification::Decided(true)
        );
    }
    assert_eq!(
        arch.contains_point(&point(2, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        arch.parameters_for_point(&point(2, 3), &policy()),
        Classification::Decided(Vec::new())
    );
    assert_eq!(
        arch.contains_point(&point(2, 3), &policy()),
        Classification::Decided(false)
    );

    let constant = QuadraticBezier2::new(point(5, 5), point(5, 5), point(5, 5));
    assert_eq!(
        constant.parameters_for_point(&point(5, 5), &policy()),
        Classification::Decided(vec![Real::zero()])
    );
    assert_eq!(
        constant.contains_point(&point(6, 5), &policy()),
        Classification::Decided(false)
    );
}

#[test]
fn control_hull_box_contains_all_control_points() {
    let curve = CubicBezier2::new(point(-3, 4), point(2, -5), point(7, 8), point(1, -2));
    let Classification::Decided(bbox) = curve.control_hull_box(&policy()) else {
        panic!("integer control hull should have decided ordering");
    };

    assert_eq!(bbox.min(), &point(-3, -5));
    assert_eq!(bbox.max(), &point(7, 8));
    for control in curve.control_points() {
        assert_eq!(
            bbox.contains_point(control, &policy()),
            Classification::Decided(true)
        );
    }
}

#[test]
fn quadratic_monotone_spans_and_bounds_use_exact_derivative_roots() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));

    assert_eq!(
        curve.axis_monotone_parameters(Axis2::Y, &policy()),
        Classification::Decided(vec![half()])
    );
    let Classification::Decided(spans) = curve.monotone_spans(&policy()) else {
        panic!("integer quadratic monotone roots should be decided");
    };
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].start(), &Real::zero());
    assert_eq!(spans[0].end(), &half());
    assert_eq!(spans[1].start(), &half());
    assert_eq!(spans[1].end(), &Real::one());

    let Classification::Decided(bounds) = curve.certified_bounds(&policy()) else {
        panic!("integer quadratic bounds should be decided");
    };
    assert_eq!(bounds.min(), &point(0, 0));
    assert_eq!(bounds.max(), &point(4, 2));
}

#[test]
fn quadratic_line_relation_solves_certified_parameters() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let baseline = LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap();
    let above = LineSeg2::try_new(point(-1, 5), point(5, 5)).unwrap();

    assert_eq!(
        curve.relation_to_line(&baseline, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), Real::one()]
        })
    );
    assert_eq!(
        curve.relation_to_line(&above, &policy()),
        Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: hypercurve::LineSide::Right
        })
    );
}

#[test]
fn quadratic_cusp_classifier_handles_flat_derivative_component() {
    let cusp = QuadraticBezier2::new(point(0, 0), point(1, 0), point(0, 0));
    assert_eq!(
        cusp.cusp_classification(&policy()),
        Classification::Decided(BezierCuspClassification::Cusps {
            parameters: vec![half()]
        })
    );
    assert_eq!(
        cusp.inflection_classification(),
        BezierInflectionClassification::NotApplicable
    );
}

#[test]
fn cubic_inflection_and_coarse_curve_relation_are_certified() {
    let cubic = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));
    let Classification::Decided(BezierInflectionClassification::Inflections { parameters }) =
        cubic.inflection_classification(&policy())
    else {
        panic!("symmetric integer cubic should expose a certified inflection");
    };
    assert!(parameters.contains(&half()));

    let far = CubicBezier2::new(point(10, 10), point(11, 12), point(12, 8), point(13, 10));
    assert_eq!(
        cubic.relation_to_cubic(&far, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        cubic.relation_to_cubic(&cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
}

#[test]
fn bezier_offset_preflight_reports_exact_hazards_before_approximation() {
    let cusp = QuadraticBezier2::new(point(0, 0), point(1, 0), point(0, 0));
    let Classification::Decided(preflight) = cusp.offset_preflight(&policy()) else {
        panic!("integer quadratic cusp preflight should be decided");
    };
    assert_eq!(preflight.degree(), BezierDegree::Quadratic);
    assert_eq!(
        preflight.cusp_classification(),
        &BezierCuspClassification::Cusps {
            parameters: vec![half()]
        }
    );
    assert_eq!(
        preflight.inflection_classification(),
        &BezierInflectionClassification::NotApplicable
    );
    assert_eq!(preflight.start_tangent_status(), ZeroStatus::NonZero);
    assert_eq!(preflight.end_tangent_status(), ZeroStatus::NonZero);
    assert_eq!(preflight.endpoint_coincidence(), ZeroStatus::Zero);
    assert!(preflight.risks().contains(&BezierOffsetRisk::Cusp));
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::CoincidentEndpoints)
    );
    assert!(!preflight.is_clear());
    assert_eq!(
        preflight.construction_policy().numeric_mode,
        NumericMode::Certified
    );

    let inflected = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));
    let Classification::Decided(preflight) = inflected.offset_preflight(&policy()) else {
        panic!("integer cubic inflection preflight should be decided");
    };
    assert_eq!(preflight.degree(), BezierDegree::Cubic);
    assert!(preflight.risks().contains(&BezierOffsetRisk::Inflection));
    assert_eq!(preflight.endpoint_coincidence(), ZeroStatus::NonZero);

    let collapsed = CubicBezier2::new(point(4, 4), point(4, 4), point(4, 4), point(4, 4));
    let Classification::Decided(preflight) = collapsed.offset_preflight(&policy()) else {
        panic!("collapsed cubic preflight should be decided");
    };
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::DegeneratePoint)
    );
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::UndefinedEndpointNormal {
                endpoint: BezierEndpoint::Start
            })
    );
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::UndefinedEndpointNormal {
                endpoint: BezierEndpoint::End
            })
    );
}

#[test]
fn staged_bezier_offset_emits_exact_line_or_unresolved_preflight() {
    let line = QuadraticBezier2::new(point(0, 3), point(2, 3), point(6, 3));
    let Classification::Decided(candidate) = line
        .offset_left_staged(Real::from(2_i8), &policy())
        .unwrap()
    else {
        panic!("certified line image should produce a decided staged offset");
    };
    let BezierOffsetCandidate2::ExactLineImage { offset, preflight } = candidate else {
        panic!("certified line image should offset as an exact line primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(preflight.degree(), BezierDegree::Quadratic);
    assert_eq!(offset.line().start(), &point(0, 5));
    assert_eq!(offset.line().end(), &point(6, 5));
    assert_eq!(offset.control_point_count(), 3);
    assert_eq!(offset.fit_certificate().source_end(), 3);
    assert_eq!(offset.distance(), &Real::from(2_i8));

    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(candidate) =
        arch.offset_left_staged(Real::one(), &policy()).unwrap()
    else {
        panic!("free-form quadratic should decide as unresolved, not uncertain");
    };
    let BezierOffsetCandidate2::Unresolved {
        preflight,
        distance,
    } = &candidate
    else {
        panic!("free-form quadratic offset should remain explicitly unresolved");
    };
    assert!(preflight.is_clear());
    assert_eq!(preflight.degree(), BezierDegree::Quadratic);
    assert_eq!(distance, &Real::one());
    assert_eq!(candidate.distance(), &Real::one());
    assert_eq!(
        candidate.unresolved_preflight().unwrap().degree(),
        BezierDegree::Quadratic
    );
}

#[test]
fn staged_bezier_offset_adapter_report_classifies_exact_ready_and_blocked_cases() {
    let line = QuadraticBezier2::new(point(0, 3), point(2, 3), point(6, 3));
    let Classification::Decided(candidate) = line
        .offset_left_staged(Real::from(2_i8), &policy())
        .unwrap()
    else {
        panic!("certified line image should produce a decided staged offset");
    };
    let report = candidate.adapter_report();
    assert_eq!(
        report.status,
        BezierOffsetAdapterStatus::ExactPrimitiveLineImage
    );
    assert_eq!(report.distance_status, ZeroStatus::NonZero);
    assert!(report.has_exact_primitive);
    assert!(!report.may_attempt_certified_adapter);
    assert!(report.risks.is_empty());

    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(candidate) =
        arch.offset_left_staged(Real::one(), &policy()).unwrap()
    else {
        panic!("free-form quadratic should decide as unresolved");
    };
    let report = candidate.adapter_report();
    assert_eq!(
        report.status,
        BezierOffsetAdapterStatus::ReadyForCertifiedAdapter
    );
    assert_eq!(report.distance, Real::one());
    assert_eq!(report.distance_status, ZeroStatus::NonZero);
    assert!(!report.has_exact_primitive);
    assert!(report.may_attempt_certified_adapter);
    assert!(report.preflight.is_clear());

    let zero_report = arch
        .offset_left_staged(Real::zero(), &policy())
        .unwrap()
        .unwrap_decided_for_test()
        .adapter_report();
    assert_eq!(
        zero_report.status,
        BezierOffsetAdapterStatus::ZeroDistanceIdentity
    );
    assert_eq!(zero_report.distance_status, ZeroStatus::Zero);
    assert!(zero_report.may_attempt_certified_adapter);

    let collapsed = CubicBezier2::new(point(2, 3), point(2, 3), point(2, 3), point(2, 3));
    let report = collapsed
        .offset_left_staged(Real::from(2_i8), &policy())
        .unwrap()
        .unwrap_decided_for_test()
        .adapter_report();
    assert_eq!(
        report.status,
        BezierOffsetAdapterStatus::BlockedByPreflightRisks
    );
    assert!(report.risks.contains(&BezierOffsetRisk::DegeneratePoint));
    assert!(!report.may_attempt_certified_adapter);
}

#[test]
fn staged_bezier_right_offset_uses_negative_signed_left_distance() {
    let line = CubicBezier2::new(point(0, 3), point(2, 3), point(4, 3), point(6, 3));
    let Classification::Decided(candidate) = line
        .offset_right_staged(Real::from(2_i8), &policy())
        .unwrap()
    else {
        panic!("certified cubic line image should produce a decided right offset");
    };
    let BezierOffsetCandidate2::ExactLineImage { offset, preflight } = candidate else {
        panic!("certified cubic line image should offset as an exact line primitive");
    };
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::AllCurvatureZero)
    );
    assert_eq!(preflight.degree(), BezierDegree::Cubic);
    assert_eq!(offset.line().start(), &point(0, 1));
    assert_eq!(offset.line().end(), &point(6, 1));
    assert_eq!(offset.control_point_count(), 4);
    assert_eq!(offset.distance(), &Real::from(-2_i8));

    let arch = CubicBezier2::new(point(0, 0), point(1, 4), point(3, 4), point(4, 0));
    let Classification::Decided(candidate) = arch
        .offset_right_staged(Real::from(3_i8), &policy())
        .unwrap()
    else {
        panic!("free-form cubic should decide as unresolved, not uncertain");
    };
    let BezierOffsetCandidate2::Unresolved {
        preflight,
        distance,
    } = &candidate
    else {
        panic!("free-form cubic right offset should remain explicitly unresolved");
    };
    assert!(preflight.is_clear());
    assert_eq!(preflight.degree(), BezierDegree::Cubic);
    assert_eq!(distance, &Real::from(-3_i8));
    assert_eq!(candidate.distance(), &Real::from(-3_i8));
}

#[test]
fn staged_rational_bezier_offset_preserves_denominator_preflight() {
    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(preflight) = rational_line.offset_preflight(&policy()) else {
        panic!("same-sign rational line preflight should be decided");
    };
    assert!(preflight.is_clear());
    assert_eq!(preflight.degree(), BezierDegree::Quadratic);

    let Classification::Decided(candidate) = rational_line
        .offset_left_staged(Real::from(2_i8), &policy())
        .unwrap()
    else {
        panic!("certified rational line image should produce a decided staged offset");
    };
    let BezierOffsetCandidate2::ExactLineImage { offset, preflight } = &candidate else {
        panic!("certified rational line image should offset as an exact line primitive");
    };
    assert!(preflight.is_clear());
    assert_eq!(candidate.preflight(), preflight);
    assert_eq!(
        candidate.exact_line_image_offset().unwrap().distance(),
        &Real::from(2_i8)
    );
    assert_eq!(offset.line().start(), &point(0, 5));
    assert_eq!(offset.line().end(), &point(6, 5));
    assert_eq!(offset.control_point_count(), 3);
    assert_eq!(offset.distance(), &Real::from(2_i8));

    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(candidate) = rational_arch
        .offset_right_staged(Real::from(3_i8), &policy())
        .unwrap()
    else {
        panic!("free-form rational conic should decide as unresolved, not uncertain");
    };
    let BezierOffsetCandidate2::Unresolved {
        preflight,
        distance,
    } = &candidate
    else {
        panic!("free-form rational conic offset should remain explicitly unresolved");
    };
    assert!(preflight.is_clear());
    assert_eq!(distance, &Real::from(-3_i8));

    let mixed_weights = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(-1_i8),
    )
    .unwrap();
    let Classification::Decided(preflight) = mixed_weights.offset_preflight(&policy()) else {
        panic!("mixed-sign rational conic preflight should report a concrete risk");
    };
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::ProjectiveDenominatorBoundary)
    );
}

#[test]
fn staged_bezier_offset_keeps_degenerate_point_preflight_payload() {
    let collapsed = CubicBezier2::new(point(2, 3), point(2, 3), point(2, 3), point(2, 3));
    let Classification::Decided(candidate) = collapsed
        .offset_left_staged(Real::from(2_i8), &policy())
        .unwrap()
    else {
        panic!("collapsed cubic should decide as unresolved with preflight");
    };
    let BezierOffsetCandidate2::Unresolved {
        preflight,
        distance,
    } = &candidate
    else {
        panic!("collapsed cubic cannot produce an exact line offset");
    };
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::DegeneratePoint)
    );
    assert_eq!(distance, &Real::from(2_i8));
    assert_eq!(candidate.distance(), &Real::from(2_i8));

    let rational = RationalQuadraticBezier2::try_unit_end_weights(
        point(5, 7),
        point(5, 7),
        point(5, 7),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(candidate) = rational
        .offset_right_staged(Real::one(), &policy())
        .unwrap()
    else {
        panic!("collapsed rational conic should decide as unresolved with preflight");
    };
    let BezierOffsetCandidate2::Unresolved {
        preflight,
        distance,
    } = &candidate
    else {
        panic!("collapsed rational conic cannot produce an exact line offset");
    };
    assert!(
        preflight
            .risks()
            .contains(&BezierOffsetRisk::DegeneratePoint)
    );
    assert_eq!(distance, &Real::from(-1_i8));
}

#[test]
fn bezier_length_bounds_use_exact_chord_and_control_polygon_interval() {
    let line_quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 8));
    let line_bounds = line_quadratic.length_bounds().unwrap();
    assert_eq!(line_bounds.lower(), &Real::from(10_i8));
    assert_eq!(line_bounds.upper(), &Real::from(10_i8));
    assert_eq!(line_bounds.width(), Real::zero());
    assert!(line_bounds.is_exact());

    let loop_quadratic = QuadraticBezier2::new(point(0, 0), point(3, 4), point(0, 0));
    let loop_bounds = loop_quadratic.length_bounds().unwrap();
    assert_eq!(loop_bounds.lower(), &Real::zero());
    assert_eq!(loop_bounds.upper(), &Real::from(10_i8));
    assert!(!loop_bounds.is_exact());

    let line_cubic = CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0));
    let cubic_bounds = line_cubic.length_bounds().unwrap();
    assert_eq!(cubic_bounds.lower(), &Real::from(6_i8));
    assert_eq!(cubic_bounds.upper(), &Real::from(6_i8));
    assert!(cubic_bounds.is_exact());
}

#[test]
fn refined_bezier_length_bounds_tighten_by_exact_subdivision() {
    let curve = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let base = curve.length_bounds().unwrap();
    let refined = curve.refined_length_bounds(1).unwrap();
    let two = Real::from(2_i8);
    let sqrt_thirteen = Real::from(13_i8).sqrt().unwrap();

    assert_eq!(base.lower(), &Real::from(6_i8));
    assert_eq!(base.upper(), &Real::from(10_i8));
    assert_eq!(refined.lower(), &(&two * sqrt_thirteen));
    assert_eq!(refined.upper(), &Real::from(8_i8));
    assert_ne!(refined, base);
    assert_eq!(curve.refined_length_bounds(0).unwrap(), base);
}

#[test]
fn prefix_bezier_length_bounds_split_exactly_at_parameter() {
    let curve = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let prefix = curve
        .prefix_length_bounds(half(), &policy())
        .unwrap()
        .unwrap_decided_for_test();
    let refined_prefix = curve
        .refined_prefix_length_bounds(half(), 0, &policy())
        .unwrap()
        .unwrap_decided_for_test();
    let five_halves = (Real::from(5_i8) / Real::from(2_i8)).unwrap();
    let three_halves = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
    let sqrt_thirteen = Real::from(13_i8).sqrt().unwrap();

    assert_eq!(prefix.lower(), &sqrt_thirteen);
    assert_eq!(prefix.upper(), &(&five_halves + &three_halves));
    assert_eq!(refined_prefix, prefix);
    assert_eq!(
        curve
            .prefix_length_bounds(Real::from(2_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );
}

#[test]
fn inverse_length_parameter_region_is_certified_not_sampled() {
    let line = CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0));
    let region = line
        .inverse_length_parameter_region(Real::from(3_i8), 8, 2, &policy())
        .unwrap()
        .unwrap_decided_for_test();

    assert_eq!(region.target_length(), &Real::from(3_i8));
    assert_eq!(region.parameter_span().start(), &half());
    assert_eq!(region.parameter_span().end(), &half());
    assert_eq!(
        region.prefix_bounds_at_span_end().lower(),
        &Real::from(3_i8)
    );
    assert_eq!(
        region.prefix_bounds_at_span_end().upper(),
        &Real::from(3_i8)
    );

    let one_third = (Real::one() / Real::from(3_i8)).unwrap();
    let non_dyadic = line
        .inverse_length_parameter_region(Real::from(2_i8), 0, 0, &policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(non_dyadic.parameter_span().start(), &one_third);
    assert_eq!(non_dyadic.parameter_span().end(), &one_third);
    assert_eq!(
        non_dyadic.prefix_bounds_at_span_end().lower(),
        &Real::from(2_i8)
    );

    let quadratic_line = QuadraticBezier2::new(point(0, 0), point(3, 0), point(6, 0));
    let quadratic_non_dyadic = quadratic_line
        .inverse_length_parameter_region(Real::from(2_i8), 0, 0, &policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(quadratic_non_dyadic.parameter_span().start(), &one_third);
    assert_eq!(quadratic_non_dyadic.parameter_span().end(), &one_third);

    assert_eq!(
        line.inverse_length_parameter_region(Real::from(-1_i8), 8, 2, &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierArcLengthTarget
    );
    assert_eq!(
        line.inverse_length_parameter_region(Real::from(7_i8), 8, 2, &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierArcLengthTarget
    );
}

#[test]
fn inverse_length_does_not_linearize_nonlinear_line_images() {
    let nonlinear_line = QuadraticBezier2::new(point(0, 0), point(1, 0), point(4, 0));
    let region = nonlinear_line
        .inverse_length_parameter_region(Real::from(2_i8), 0, 0, &policy())
        .unwrap()
        .unwrap_decided_for_test();

    assert_eq!(region.parameter_span().start(), &Real::zero());
    assert_eq!(region.parameter_span().end(), &Real::one());
    assert_ne!(
        region.parameter_span().start(),
        region.parameter_span().end()
    );
}

#[test]
fn bezier_signed_area_contribution_integrates_green_boundary_exactly() {
    let line_quadratic = QuadraticBezier2::new(point(1, 1), point(3, 2), point(5, 3));
    assert_eq!(
        line_quadratic.signed_area_contribution().unwrap(),
        Real::from(-1_i8)
    );

    let arch = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    assert_eq!(arch.signed_area_contribution().unwrap(), Real::from(-8_i8));
    assert_eq!(
        arch.prefix_signed_area_contribution(half(), &policy())
            .unwrap()
            .unwrap_decided_for_test(),
        Real::from(-1_i8)
    );
    assert_eq!(
        arch.prefix_signed_area_contribution(Real::from(-1_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );

    let cubic_line = CubicBezier2::new(point(1, 1), point(2, 1), point(4, 1), point(5, 1));
    assert_eq!(
        cubic_line.signed_area_contribution().unwrap(),
        Real::from(-2_i8)
    );
}

#[test]
fn bezier_area_prefix_sums_answer_range_queries_without_reintegration() {
    let first = QuadraticBezier2::new(point(0, 0), point(3, 4), point(6, 0));
    let second = QuadraticBezier2::new(point(6, 0), point(7, 0), point(8, 0));
    let third = QuadraticBezier2::new(point(8, 0), point(8, 2), point(8, 4));
    let table = BezierAreaPrefixSums2::from_quadratics([&first, &second, &third]).unwrap();

    assert_eq!(table.segment_count(), 3);
    assert_eq!(table.prefixes()[0], Real::zero());
    assert_eq!(table.range_contribution(0..1).unwrap(), Real::from(-8_i8));
    assert_eq!(table.range_contribution(1..1).unwrap(), Real::zero());
    assert_eq!(table.range_contribution(1..3).unwrap(), Real::from(16_i8));
    assert_eq!(table.total(), &Real::from(8_i8));
    assert_eq!(
        table.range_contribution(2..4).unwrap_err(),
        hypercurve::CurveError::InvalidBezierRange
    );

    let contributions = vec![Real::from(3_i8), Real::from(-5_i8), Real::from(7_i8)];
    let direct = BezierAreaPrefixSums2::from_contributions(contributions);
    assert_eq!(direct.range_contribution(0..3).unwrap(), Real::from(5_i8));
    assert_eq!(direct.range_contribution(1..3).unwrap(), Real::from(2_i8));
}

#[test]
fn bezier_area_moments_integrate_green_boundary_identities_exactly() {
    let horizontal = CubicBezier2::new(point(1, 2), point(2, 2), point(4, 2), point(5, 2));
    let moments = horizontal.area_moments_contribution().unwrap();
    assert_eq!(moments.signed_area(), &Real::from(-4_i8));
    assert_eq!(moments.x_moment(), &Real::zero());
    assert_eq!(moments.y_moment(), &Real::from(-8_i8));

    let vertical = QuadraticBezier2::new(point(3, 1), point(3, 3), point(3, 5));
    let moments = vertical.area_moments_contribution().unwrap();
    assert_eq!(moments.signed_area(), &Real::from(6_i8));
    assert_eq!(moments.x_moment(), &Real::from(18_i8));
    assert_eq!(moments.y_moment(), &Real::zero());

    let prefix = horizontal
        .prefix_area_moments_contribution(half(), &policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(prefix.signed_area(), &Real::from(-2_i8));
    assert_eq!(prefix.x_moment(), &Real::zero());
    assert_eq!(prefix.y_moment(), &Real::from(-4_i8));
    assert_eq!(
        horizontal
            .prefix_area_moments_contribution(Real::from(2_i8), &policy())
            .unwrap_err(),
        hypercurve::CurveError::InvalidBezierParameter
    );
}

#[test]
fn bezier_fit_source_report_collects_quadratic_exact_facts() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let report = curve.fit_source_report(&policy()).unwrap();

    assert_eq!(report.degree(), BezierDegree::Quadratic);
    assert_eq!(report.structural_facts(), curve.structural_facts());
    assert!(matches!(report.control_hull(), Classification::Decided(_)));
    assert!(matches!(report.monotone_spans(), Classification::Decided(spans) if !spans.is_empty()));
    assert_eq!(
        report.cusp_classification(),
        &Classification::Decided(BezierCuspClassification::None)
    );
    assert_eq!(
        report.inflection_classification(),
        &Classification::Decided(BezierInflectionClassification::NotApplicable)
    );
    assert_eq!(report.length_bounds(), &curve.length_bounds().unwrap());
    assert_eq!(
        report.area_moments(),
        &curve.area_moments_contribution().unwrap()
    );
    assert_eq!(
        report.start_tangent(),
        &curve.endpoint_tangent(BezierEndpoint::Start)
    );
    assert_eq!(
        report.end_tangent(),
        &curve.endpoint_tangent(BezierEndpoint::End)
    );
    assert!(!report.has_exact_primitive_image());
    assert!(report.needs_higher_order_fit());
    assert_eq!(
        report.exact_line_image_fit(),
        &Classification::Decided(BezierLineImageFitRelation::NotLine)
    );
    assert_eq!(
        report.exact_point_image_fit(),
        &Classification::Decided(BezierPointImageFitRelation::NotPoint)
    );
}

#[test]
fn bezier_fit_source_report_classifies_exact_cubic_line_image() {
    let curve = CubicBezier2::new(point(0, 3), point(2, 3), point(4, 3), point(6, 3));
    let report = curve.fit_source_report(&policy()).unwrap();

    assert_eq!(report.degree(), BezierDegree::Cubic);
    assert_eq!(report.structural_facts(), curve.structural_facts());
    assert!(matches!(
        report.inflection_classification(),
        Classification::Decided(
            BezierInflectionClassification::None | BezierInflectionClassification::AllCurvatureZero
        )
    ));
    assert!(report.length_bounds().is_exact());
    assert!(report.has_exact_primitive_image());
    assert!(!report.needs_higher_order_fit());
    assert!(matches!(
        report.exact_line_image_fit(),
        Classification::Decided(BezierLineImageFitRelation::Fit(_))
    ));
    assert!(matches!(
        report.exact_point_image_fit(),
        Classification::Decided(BezierPointImageFitRelation::NotPoint)
    ));
}

#[test]
fn bezier_fit_source_batch_report_aggregates_path_range_facts() {
    let line = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let arch = QuadraticBezier2::new(point(4, 0), point(6, 4), point(8, 0));
    let line_report = line.fit_source_report(&policy()).unwrap();
    let arch_report = arch.fit_source_report(&policy()).unwrap();

    let batch = BezierFitSourceBatchReport2::from_reports([&line_report, &arch_report]);

    assert_eq!(batch.segment_count(), 2);
    assert_eq!(batch.exact_primitive_sources(), 1);
    assert_eq!(batch.higher_order_sources(), 1);
    assert_eq!(batch.uncertain_sources(), 0);
    assert!(batch.all_sources_exact_rational());
    assert!(batch.all_monotone_spans_decided());
    let expected_lower =
        line.length_bounds().unwrap().lower() + arch.length_bounds().unwrap().lower();
    let expected_area = line.area_moments_contribution().unwrap().signed_area()
        + arch.area_moments_contribution().unwrap().signed_area();
    assert!(certifies_zero(batch.total_length_lower() - &expected_lower));
    assert!(certifies_zero(batch.total_signed_area() - &expected_area));
    assert!(matches!(
        batch.total_length_width().refine_sign_until(-64),
        Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)
    ));
}

#[test]
fn bezier_fit_source_batch_constructors_match_manual_reports() {
    let curves = [
        CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0)),
        CubicBezier2::new(point(6, 0), point(7, 3), point(9, 3), point(10, 0)),
    ];
    let reports = curves
        .iter()
        .map(|curve| curve.fit_source_report(&policy()))
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    let manual = BezierFitSourceBatchReport2::from_reports(&reports);
    let constructed = BezierFitSourceBatchReport2::from_cubics(&curves, &policy()).unwrap();

    assert_eq!(constructed.segment_count(), 2);
    assert_eq!(constructed.exact_primitive_sources(), 1);
    assert_eq!(constructed.higher_order_sources(), 1);
    assert_eq!(constructed.uncertain_sources(), manual.uncertain_sources());
    assert!(certifies_zero(
        constructed.total_length_lower() - manual.total_length_lower()
    ));
    assert!(certifies_zero(
        constructed.total_signed_area() - manual.total_signed_area()
    ));
}

#[test]
fn bezier_fit_source_prefix_sums_answer_range_queries_without_rewalking_sources() {
    let sources = [
        QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0)),
        QuadraticBezier2::new(point(4, 0), point(6, 4), point(8, 0)),
        QuadraticBezier2::new(point(8, 0), point(10, 0), point(12, 0)),
    ];
    let reports = sources
        .iter()
        .map(|curve| curve.fit_source_report(&policy()))
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    let table = BezierFitSourcePrefixSums2::from_reports(&reports);
    let range = table.range_report(1..3).unwrap();
    let manual = BezierFitSourceBatchReport2::from_reports(&reports[1..3]);
    let empty = table.range_report(2..2).unwrap();

    assert_eq!(table.segment_count(), 3);
    assert_eq!(table.prefixes().len(), 4);
    assert_eq!(range.segment_count(), 2);
    assert_eq!(range.exact_primitive_sources(), 1);
    assert_eq!(range.higher_order_sources(), 1);
    assert_eq!(empty.segment_count(), 0);
    assert!(certifies_zero(
        range.total_length_lower() - manual.total_length_lower()
    ));
    assert!(certifies_zero(
        range.total_signed_area() - manual.total_signed_area()
    ));
    assert_eq!(
        table.range_report(2..4).unwrap_err(),
        hypercurve::CurveError::InvalidBezierRange
    );
}

#[test]
fn bezier_fit_source_prefix_sum_constructors_match_manual_ranges() {
    let sources = [
        CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0)),
        CubicBezier2::new(point(6, 0), point(7, 3), point(9, 3), point(10, 0)),
    ];
    let table = BezierFitSourcePrefixSums2::from_cubics(&sources, &policy()).unwrap();
    let whole = table.range_report(0..2).unwrap();
    let manual = BezierFitSourceBatchReport2::from_cubics(&sources, &policy()).unwrap();

    assert_eq!(whole.segment_count(), manual.segment_count());
    assert_eq!(
        whole.exact_primitive_sources(),
        manual.exact_primitive_sources()
    );
    assert_eq!(whole.higher_order_sources(), manual.higher_order_sources());
    assert!(certifies_zero(
        whole.total_length_lower() - manual.total_length_lower()
    ));
    assert!(certifies_zero(
        whole.total_signed_area() - manual.total_signed_area()
    ));
}

#[test]
fn bezier_area_moment_prefix_sums_answer_range_queries_without_reintegration() {
    let first = CubicBezier2::new(point(1, 2), point(2, 2), point(4, 2), point(5, 2));
    let second = CubicBezier2::new(point(5, 2), point(5, 3), point(5, 4), point(5, 6));
    let third = CubicBezier2::new(point(5, 6), point(3, 6), point(2, 6), point(1, 6));
    let table = BezierAreaMomentPrefixSums2::from_cubics([&first, &second, &third]).unwrap();

    assert_eq!(table.segment_count(), 3);
    assert_eq!(table.prefixes()[0].signed_area(), &Real::zero());
    assert_eq!(
        table.range_contribution(0..1).unwrap(),
        first.area_moments_contribution().unwrap()
    );
    let empty = table.range_contribution(1..1).unwrap();
    assert_eq!(empty.signed_area(), &Real::zero());
    assert_eq!(
        table.range_contribution(2..4).unwrap_err(),
        hypercurve::CurveError::InvalidBezierRange
    );

    let full = table.range_contribution(0..3).unwrap();
    assert_eq!(table.total(), &full);
}

#[test]
fn mixed_polynomial_bezier_curve_relation_certifies_disjoint_and_shared_endpoint() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0));
    let far_cubic = CubicBezier2::new(point(10, 10), point(11, 12), point(12, 8), point(13, 10));
    let touching_cubic = CubicBezier2::new(point(2, 0), point(3, 2), point(4, 2), point(5, 0));

    assert_eq!(
        quadratic.relation_to_cubic(&far_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        quadratic.relation_to_cubic(&touching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        touching_cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
}

#[test]
fn shared_endpoint_does_not_hide_exact_interior_quadratic_intersection() {
    let first = QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0));
    let second = QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 4));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        first.relation_to_quadratic(&second, &policy())
    else {
        panic!("shared endpoint plus midpoint crossing should retain both exact points");
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &point(1, 1)));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        second.relation_to_quadratic(&first, &policy())
    else {
        panic!("symmetric shared endpoint plus midpoint crossing should retain both exact points");
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &point(1, 1)));
}

#[test]
fn polynomial_bezier_curve_relation_certifies_endpoint_on_quadratic_interior() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let endpoint_on_arch = CubicBezier2::new(point(2, 2), point(3, 5), point(5, 5), point(6, 4));

    let relation = endpoint_on_arch.relation_to_quadratic(&arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!(
            "endpoint on quadratic interior should be certified before subdivision: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        arch.relation_to_cubic(&endpoint_on_arch, &policy()),
        Classification::Decided(BezierCurveRelation::EndpointIntersections { points })
    );
}

#[test]
fn polynomial_bezier_curve_relation_solves_same_parameter_quadratic_crossings() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let crossing = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    let relation = arch.relation_to_quadratic(&crossing, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("same-parameter quadratic crossings should be solved algebraically: {relation:?}");
    };

    assert_eq!(points.len(), 2);
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        crossing.relation_to_quadratic(&arch, &policy())
    else {
        panic!("same-parameter quadratic crossing relation should be symmetric");
    };
    assert_eq!(points.len(), 2);
}

#[test]
fn polynomial_bezier_curve_relation_solves_non_dyadic_quadratic_root_before_grid_probe() {
    let first = QuadraticBezier2::new(point(0, 0), point(3, 2), point(6, 0));
    let second = QuadraticBezier2::new(
        point(0, -1),
        Point2::new(
            Real::from(3_i8),
            (Real::from(5_i8) / Real::from(2_i8)).unwrap(),
        ),
        point(6, 2),
    );

    let root = (Real::one() / Real::from(3_i8)).unwrap();
    let relation = first.relation_to_quadratic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "same-parameter non-dyadic quadratic root should be solved algebraically: {relation:?}"
        );
    };

    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(root));
}

#[test]
fn polynomial_bezier_curve_relation_certifies_same_axis_monotone_no_hit() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let raised = QuadraticBezier2::new(point(0, 1), point(2, 5), point(4, 1));

    assert_eq!(
        arch.relation_to_quadratic(&raised, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        raised.relation_to_quadratic(&arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn polynomial_bezier_curve_relation_degree_normalizes_same_axis_no_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let elevated_raised = CubicBezier2::new(point(0, 1), point(2, 5), point(4, 5), point(6, 1));

    assert_eq!(
        quadratic.relation_to_cubic(&elevated_raised, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        elevated_raised.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn polynomial_bezier_graph_order_classifies_monotone_ranges() {
    let lower = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let upper = CubicBezier2::new(point(0, 2), point(2, 6), point(4, 6), point(6, 2));
    let crossing = CubicBezier2::new(point(0, -1), point(2, 6), point(4, 6), point(6, -1));
    let not_graph = QuadraticBezier2::new(point(0, 0), point(3, 6), point(0, 0));

    assert_eq!(
        lower.graph_order_to_cubic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );
    assert_eq!(
        upper.graph_order_to_quadratic_over_axis(&lower, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );

    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = lower.graph_order_to_cubic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("shared monotone graph crossing should retain exact or bracketed roots");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "crossing graph order should expose retained same-parameter candidates"
    );

    assert_eq!(
        lower.graph_order_to_quadratic_over_axis(&not_graph, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
}

#[test]
fn polynomial_bezier_graph_contact_order_classifies_crossings_and_tangencies() {
    let baseline = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let crossing = QuadraticBezier2::new(point(0, 1), point(2, 0), point(4, -1));
    let tangent = QuadraticBezier2::new(point(0, 1), point(2, -1), point(4, 1));
    let above = QuadraticBezier2::new(point(0, 2), point(2, 2), point(4, 2));

    assert_eq!(
        baseline.graph_contact_order_to_quadratic_over_axis(&above, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphContactOrder::FirstLess)
    );

    let Classification::Decided(BezierMonotoneGraphContactOrder::IntersectsOrTouches {
        contacts,
        spans,
    }) = baseline.graph_contact_order_to_quadratic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("represented graph crossing should expose a contact certificate");
    };
    assert!(spans.is_empty());
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter(), &half());
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);

    let Classification::Decided(BezierMonotoneGraphContactOrder::IntersectsOrTouches {
        contacts,
        spans,
    }) = baseline.graph_contact_order_to_quadratic_over_axis(&tangent, Axis2::X, &policy())
    else {
        panic!("represented graph tangent should expose a contact certificate");
    };
    assert!(spans.is_empty());
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter(), &half());
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_degree_elevated_same_image() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let elevated = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let reversed_quadratic = QuadraticBezier2::new(point(6, 0), point(3, 6), point(0, 0));
    let reversed_elevated = CubicBezier2::new(point(6, 0), point(4, 4), point(2, 4), point(0, 0));

    assert_eq!(
        quadratic.relation_to_cubic(&elevated, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        elevated.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        quadratic.relation_to_quadratic(&reversed_quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        elevated.relation_to_cubic(&reversed_elevated, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        quadratic.relation_to_cubic(&reversed_elevated, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_midpoint_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 6), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 12), point(2, 0), point(4, 0), point(6, 12));

    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("mixed-degree same-parameter midpoint hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(3, 3));

    assert_eq!(
        cubic.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points })
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_quarter_hit() {
    let middle_y = (Real::from(62_i8) / Real::from(3_i8)).unwrap();
    let quadratic = QuadraticBezier2::new(
        point(0, 0),
        Point2::new(Real::from(3_i8), middle_y),
        point(6, 0),
    );
    let cubic = CubicBezier2::new(point(0, 16), point(2, 0), point(4, 0), point(6, 64));

    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!(
            "mixed-degree graph quarter fixture should retain all same-parameter roots: {relation:?}"
        );
    };
    assert_same_parameter_regions_include_exact(&regions, &quarter);
    assert_same_parameter_regions_include_bracket(&regions);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_thirty_second_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 15), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -31));

    let thirty_second = (Real::one() / Real::from(32_i8)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "mixed-degree same-parameter thirty-second hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &quadratic.point_at(thirty_second));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_quadratic(&quadratic, &policy())
    else {
        panic!("mixed-degree same-parameter thirty-second hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_sixty_fourth_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 31), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -63));

    let sixty_fourth = (Real::one() / Real::from(64_i8)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "mixed-degree same-parameter sixty-fourth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &quadratic.point_at(sixty_fourth));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_quadratic(&quadratic, &policy())
    else {
        panic!("mixed-degree same-parameter sixty-fourth hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_one_hundred_twenty_eighth_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 63), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -127));

    let one_hundred_twenty_eighth = (Real::one() / Real::from(128_i16)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "mixed-degree same-parameter one-hundred-twenty-eighth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0].point(),
        &quadratic.point_at(one_hundred_twenty_eighth)
    );

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_quadratic(&quadratic, &policy())
    else {
        panic!("mixed-degree same-parameter one-hundred-twenty-eighth hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_two_hundred_fifty_sixth_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 127), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -255));

    let two_hundred_fifty_sixth = (Real::one() / Real::from(256_i16)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "mixed-degree same-parameter two-hundred-fifty-sixth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0].point(),
        &quadratic.point_at(two_hundred_fifty_sixth)
    );

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_quadratic(&quadratic, &policy())
    else {
        panic!("mixed-degree same-parameter two-hundred-fifty-sixth hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_mixed_degree_five_hundred_twelfth_hit() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 255), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -511));

    let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "mixed-degree same-parameter five-hundred-twelfth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &quadratic.point_at(five_hundred_twelfth));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_quadratic(&quadratic, &policy())
    else {
        panic!("mixed-degree same-parameter five-hundred-twelfth hit should be symmetric");
    };
    assert_eq!(points.len(), 1);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_quarter_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 4), point(2, 0), point(4, 0), point(6, 36));

    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!("cubic graph quarter fixture should retain all same-parameter roots: {relation:?}");
    };
    assert_same_parameter_regions_include_exact(&regions, &quarter);
    assert_same_parameter_regions_include_bracket(&regions);

    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        second.relation_to_cubic(&first, &policy())
    else {
        panic!("cubic same-parameter graph isolation should be symmetric");
    };
    assert_same_parameter_regions_include_exact(&regions, &quarter);
    assert_same_parameter_regions_include_bracket(&regions);
}

#[test]
fn cubic_dyadic_point_solver_promotes_endpoint_on_cubic() {
    let cubic = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
    let cubic_point = cubic.point_at(five_hundred_twelfth.clone());
    let probe = QuadraticBezier2::new(
        Point2::new(cubic_point.x().clone(), cubic_point.y() + Real::from(20_i8)),
        Point2::new(
            cubic_point.x() + Real::one(),
            cubic_point.y() + Real::from(10_i8),
        ),
        cubic_point.clone(),
    );

    assert_eq!(
        cubic.dyadic_parameters_for_point(&cubic_point, &policy()),
        Classification::Decided(vec![five_hundred_twelfth])
    );
    assert_eq!(
        cubic.dyadic_parameters_for_point(&point(3, 4), &policy()),
        Classification::Decided(Vec::new())
    );

    let relation = probe.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("dyadic endpoint-on-cubic hit should be certified before subdivision: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &cubic_point);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_eighth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, 329));

    let eighth = (Real::one() / Real::from(8_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!("cubic graph eighth fixture should retain all same-parameter roots: {relation:?}");
    };
    assert_same_parameter_regions_include_exact(&regions, &eighth);
    assert_same_parameter_regions_include_bracket(&regions);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_sixteenth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -495));

    let sixteenth = (Real::one() / Real::from(16_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("cubic same-parameter sixteenth hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(sixteenth));
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_thirty_second_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -17887));

    let thirty_second = (Real::one() / Real::from(32_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("cubic same-parameter thirty-second hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(thirty_second));
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_sixty_fourth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -201663));

    let sixty_fourth = (Real::one() / Real::from(64_i8)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!("cubic same-parameter sixty-fourth hit should be promoted exactly: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(sixty_fourth));
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_one_hundred_twenty_eighth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -1_853_311));

    let one_hundred_twenty_eighth = (Real::one() / Real::from(128_i16)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "cubic same-parameter one-hundred-twenty-eighth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0].point(),
        &first.point_at(one_hundred_twenty_eighth)
    );
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_two_hundred_fifty_sixth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(point(0, 1), point(2, 0), point(4, 0), point(6, -15_798_015));

    let two_hundred_fifty_sixth = (Real::one() / Real::from(256_i16)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "cubic same-parameter two-hundred-fifty-sixth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(two_hundred_fifty_sixth));
}

#[test]
fn polynomial_bezier_curve_relation_rejects_prefix_only_shared_axis_proof() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(1, 10), point(2, 0));
    let prefix_matching_cubic =
        CubicBezier2::new(point(0, 1), point(1, 11), point(2, 1), point(100, 1));

    assert_ne!(
        quadratic.relation_to_cubic(&prefix_matching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_ne!(
        quadratic.relation_to_cubic(&prefix_matching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn polynomial_bezier_graph_relation_isolates_non_dyadic_same_parameter_root() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 0), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 1), point(2, 0), point(4, -1), point(6, -1));

    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!(
            "shared monotone graph pair should isolate its non-dyadic same-parameter root: {relation:?}"
        );
    };

    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].first(), regions[0].second());
    assert_ne!(regions[0].first().start(), regions[0].first().end());

    let relation = cubic.relation_to_quadratic(&quadratic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!("shared monotone graph isolation should be symmetric: {relation:?}");
    };
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].first(), regions[0].second());
}

#[test]
fn polynomial_bezier_graph_relation_does_not_stop_at_midpoint_root() {
    let quadratic = QuadraticBezier2::new(point(0, 0), point(3, 0), point(6, 0));
    let cubic = CubicBezier2::new(point(0, 3), point(2, -2), point(4, -1), point(6, 6));

    let half = half();
    let relation = quadratic.relation_to_cubic(&cubic, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!("midpoint graph root should not hide a second non-dyadic root: {relation:?}");
    };

    assert_same_parameter_regions_include_exact(&regions, &half);
    assert_same_parameter_regions_include_bracket(&regions);
}

#[test]
fn polynomial_bezier_non_graph_cubic_candidates_promote_exact_deep_dyadic_root() {
    let first_controls = [point(0, 0), point(30, 70), point(60, -20), point(90, 30)];
    let difference = [
        ratio(-1, 1024),
        ratio(1021, 3072),
        ratio(2045, 3072),
        ratio(1023, 1024),
    ];
    let second = CubicBezier2::new(
        Point2::new(
            first_controls[0].x() - &difference[0],
            first_controls[0].y() - &difference[0],
        ),
        Point2::new(
            first_controls[1].x() - &difference[1],
            first_controls[1].y() - &difference[1],
        ),
        Point2::new(
            first_controls[2].x() - &difference[2],
            first_controls[2].y() - &difference[2],
        ),
        Point2::new(
            first_controls[3].x() - &difference[3],
            first_controls[3].y() - &difference[3],
        ),
    );
    let first = CubicBezier2::new(
        first_controls[0].clone(),
        first_controls[1].clone(),
        first_controls[2].clone(),
        first_controls[3].clone(),
    );

    let root = ratio(1, 1024);
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "non-graph cubic same-parameter candidate should promote exact deep dyadic root: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(root));
}

#[test]
fn polynomial_bezier_non_graph_cubic_candidates_retain_irreducible_bracket() {
    let first_controls = [point(0, 0), point(30, 70), point(60, -20), point(90, 30)];
    let difference = [r(-1), r(1), r(1), r(1)];
    let first = CubicBezier2::new(
        first_controls[0].clone(),
        first_controls[1].clone(),
        first_controls[2].clone(),
        first_controls[3].clone(),
    );
    let second = CubicBezier2::new(
        Point2::new(
            first_controls[0].x() - &difference[0],
            first_controls[0].y() - &difference[0],
        ),
        Point2::new(
            first_controls[1].x() - &difference[1],
            first_controls[1].y() - &difference[1],
        ),
        Point2::new(
            first_controls[2].x() - &difference[2],
            first_controls[2].y() - &difference[2],
        ),
        Point2::new(
            first_controls[3].x() - &difference[3],
            first_controls[3].y() - &difference[3],
        ),
    );

    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!(
            "non-graph cubic same-parameter candidate should retain irreducible bracket: {relation:?}"
        );
    };
    assert_same_parameter_regions_include_bracket(&regions);

    let relation = second.relation_to_cubic(&first, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
    else {
        panic!(
            "non-graph cubic same-parameter candidate isolation should be symmetric: {relation:?}"
        );
    };
    assert_same_parameter_regions_include_bracket(&regions);
}

#[test]
fn polynomial_bezier_curve_relation_certifies_cubic_five_hundred_twelfth_hit() {
    let first = CubicBezier2::new(point(0, 0), point(2, 4), point(4, 4), point(6, 0));
    let second = CubicBezier2::new(
        point(0, 1),
        point(2, 0),
        point(4, 0),
        point(6, -130_293_247),
    );

    let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
    let relation = first.relation_to_cubic(&second, &policy());
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
    else {
        panic!(
            "cubic same-parameter five-hundred-twelfth hit should be promoted exactly: {relation:?}"
        );
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &first.point_at(five_hundred_twelfth));
}

#[test]
fn polynomial_bezier_line_images_use_exact_line_intersection() {
    let horizontal = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let vertical = CubicBezier2::new(point(2, -2), point(2, -1), point(2, 1), point(2, 2));
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) =
        horizontal.relation_to_cubic(&vertical, &policy())
    else {
        panic!("certified Bezier line images should use native line intersection");
    };
    assert_eq!(
        intersection,
        LineLineIntersection::Point {
            point: point(2, 0),
            a_param: half(),
            b_param: half(),
            kind: IntersectionKind::Crossing,
        }
    );

    let overlap = QuadraticBezier2::new(point(2, 0), point(3, 0), point(4, 0));
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection {
        intersection: LineLineIntersection::Overlap { segment, .. },
    }) = horizontal.relation_to_quadratic(&overlap, &policy())
    else {
        panic!("collinear Bezier line images should report native overlap");
    };
    assert_eq!(segment.start(), &point(2, 0));
    assert_eq!(segment.end(), &point(4, 0));
}

#[test]
fn polynomial_bezier_line_image_against_curve_uses_supporting_line_roots() {
    let tangent_line = QuadraticBezier2::new(point(0, 2), point(2, 2), point(4, 2));
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        tangent_line.relation_to_quadratic(&arch, &policy())
    else {
        panic!("line-image versus quadratic should use exact supporting-line roots");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        arch.relation_to_quadratic(&tangent_line, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points })
    );

    let clipped_line = CubicBezier2::new(point(3, 2), point(3, 2), point(4, 2), point(4, 2));
    assert_eq!(
        clipped_line.relation_to_quadratic(&arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let baseline = QuadraticBezier2::new(point(-1, 0), point(1, 0), point(4, 0));
    let isolated_cubic = CubicBezier2::new(point(0, -1), point(1, 1), point(2, 1), point(3, 1));
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        baseline.relation_to_cubic(&isolated_cubic, &policy())
    else {
        panic!("line-image versus cubic isolated roots should retain curve/curve regions");
    };
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].first().start(), &Real::zero());
    assert_eq!(regions[0].first().end(), &Real::one());
    assert_ne!(regions[0].second().start(), regions[0].second().end());
}

#[test]
fn cubic_line_relation_reports_exact_and_isolated_roots() {
    let line = LineSeg2::try_new(point(-1, 0), point(4, 0)).unwrap();
    let exact_roots = CubicBezier2::new(point(0, 0), point(1, 2), point(2, -2), point(3, 0));

    assert_eq!(
        exact_roots.relation_to_line(&line, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), half(), Real::one()]
        })
    );

    let isolated_root = CubicBezier2::new(point(0, -1), point(1, 1), point(2, 1), point(3, 1));
    let Classification::Decided(BezierLineRelation::IsolatedIntersections { spans }) =
        isolated_root.relation_to_line(&line, &policy())
    else {
        panic!("non-rational cubic line root should be retained as an isolating span");
    };

    assert_eq!(spans.len(), 1);
    assert_ne!(spans[0].start(), &Real::zero());
    assert_ne!(spans[0].end(), &Real::one());
    assert_ne!(
        spans[0].start(),
        spans[0].end(),
        "non-represented cubic root should remain a nonzero bracket"
    );
}

#[test]
fn bezier_line_contact_relation_classifies_crossings_and_tangencies() {
    let baseline = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let crossing = QuadraticBezier2::new(point(0, -2), point(2, 2), point(4, 2));
    let tangent = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));

    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        crossing.relation_to_line_with_contacts(&baseline, &policy())
    else {
        panic!("quadratic crossing should expose represented line contacts");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);

    let tangent_line = LineSeg2::try_new(point(0, 2), point(4, 2)).unwrap();
    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        tangent.relation_to_line_with_contacts(&tangent_line, &policy())
    else {
        panic!("quadratic tangent should expose represented line contact");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter(), &half());
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);

    let cubic_tangent = CubicBezier2::new(point(0, 0), point(1, 2), point(3, 2), point(4, 0));
    let cubic_tangent_y = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
    let cubic_tangent_line = LineSeg2::try_new(
        Point2::new(Real::zero(), cubic_tangent_y.clone()),
        Point2::new(Real::from(4_i8), cubic_tangent_y),
    )
    .unwrap();
    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        cubic_tangent.relation_to_line_with_contacts(&cubic_tangent_line, &policy())
    else {
        panic!("cubic tangent should expose represented line contact");
    };
    assert!(contacts.iter().any(|contact| contact.parameter() == &half()
        && contact.kind() == BezierLineContactKind::Tangent));
}

#[test]
fn rational_quadratic_evaluates_homogeneous_conic_points() {
    let polynomial = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    assert_eq!(
        polynomial.point_at(half(), &policy()),
        Classification::Decided(point(2, 2))
    );

    let weighted = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let eight_thirds = (Real::from(8_i8) / Real::from(3_i8)).unwrap();
    assert_eq!(
        weighted.point_at(half(), &policy()),
        Classification::Decided(Point2::new(Real::from(2_i8), eight_thirds))
    );
}

#[test]
fn rational_quadratic_classifies_conic_weight_families_and_facts() {
    let ellipse = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        half(),
    )
    .unwrap();
    let parabola = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        Real::one(),
    )
    .unwrap();
    let hyperbola = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 1),
        point(2, 0),
        Real::from(2_i8),
    )
    .unwrap();

    assert_eq!(
        ellipse.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::EllipseLike)
    );
    assert_eq!(
        parabola.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::Parabola)
    );
    assert_eq!(
        hyperbola.conic_kind(&policy()),
        Classification::Decided(RationalQuadraticConicKind::HyperbolaLike)
    );

    let facts = ellipse.structural_facts();
    assert!(facts.all_exact_rational());
    assert_eq!(facts.weight_known_nonzero_mask, 0b111);
    assert_eq!(facts.conic_discriminant_zero_status, ZeroStatus::NonZero);
    assert_eq!(
        RationalQuadraticBezier2::try_unit_end_weights(
            point(0, 0),
            point(1, 1),
            point(2, 0),
            Real::zero()
        ),
        Err(hypercurve::CurveError::ZeroRationalBezierWeight)
    );
}

#[test]
fn rational_quadratic_monotone_spans_and_bounds_use_quotient_derivative() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();

    assert_eq!(
        curve.axis_monotone_parameters(Axis2::Y, &policy()),
        Classification::Decided(vec![half()])
    );
    let Classification::Decided(spans) = curve.monotone_spans(&policy()) else {
        panic!("equal-weight rational quadratic should expose polynomial monotone spans");
    };
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].start(), &Real::zero());
    assert_eq!(spans[0].end(), &half());
    assert_eq!(spans[1].start(), &half());
    assert_eq!(spans[1].end(), &Real::one());

    let Classification::Decided(bounds) = curve.certified_bounds(&policy()) else {
        panic!("equal-weight rational quadratic bounds should be decided");
    };
    assert_eq!(bounds.min(), &point(0, 0));
    assert_eq!(bounds.max(), &point(4, 2));
}

#[test]
fn rational_quadratic_certifies_parameterized_points_and_line_relation() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let midpoint = Point2::new(
        Real::from(2_i8),
        (Real::from(8_i8) / Real::from(3_i8)).unwrap(),
    );
    assert_eq!(
        curve.contains_point_at_parameter(&midpoint, half(), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        curve.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Decided(false)
    );

    let baseline = LineSeg2::try_new(point(-1, 0), point(5, 0)).unwrap();
    assert_eq!(
        curve.relation_to_line(&baseline, &policy()),
        Classification::Decided(BezierLineRelation::Intersects {
            parameters: vec![Real::zero(), Real::one()]
        })
    );

    let above = LineSeg2::try_new(point(-1, 5), point(5, 5)).unwrap();
    assert_eq!(
        curve.relation_to_line(&above, &policy()),
        Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: hypercurve::LineSide::Right
        })
    );
}

#[test]
fn rational_quadratic_point_on_curve_solves_exact_parameters() {
    let curve = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 3),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();

    let Classification::Decided(parameters) = curve.parameters_for_point(&point(2, 2), &policy())
    else {
        panic!("integer point on positive-weight conic should have certified parameters");
    };
    assert!(
        !parameters.is_empty(),
        "rational point solver should retain at least one exact parameter"
    );
    for parameter in parameters {
        assert_eq!(
            curve.contains_point_at_parameter(&point(2, 2), parameter, &policy()),
            Classification::Decided(true)
        );
    }
    assert_eq!(
        curve.contains_point(&point(2, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        curve.parameters_for_point(&point(2, 3), &policy()),
        Classification::Decided(Vec::new())
    );
    assert_eq!(
        curve.contains_point(&point(2, 3), &policy()),
        Classification::Decided(false)
    );

    let constant = RationalQuadraticBezier2::try_unit_end_weights(
        point(5, 5),
        point(5, 5),
        point(5, 5),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        constant.parameters_for_point(&point(5, 5), &policy()),
        Classification::Decided(vec![Real::zero()])
    );
}

#[test]
fn rational_quadratic_reports_projective_denominator_boundary() {
    let curve = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
        -Real::one(),
        Real::one(),
    )
    .unwrap();

    assert_eq!(
        curve.point_at(half(), &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
    assert_eq!(
        curve.contains_point_at_parameter(&point(2, 2), half(), &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
}

#[test]
fn rational_quadratic_curve_relation_certifies_identity_disjoint_and_shared_endpoint() {
    let first = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(1, 2),
        point(2, 0),
        half(),
    )
    .unwrap();
    let far = RationalQuadraticBezier2::try_unit_end_weights(
        point(10, 10),
        point(11, 12),
        point(12, 10),
        half(),
    )
    .unwrap();
    let touching = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, 0),
        point(3, 2),
        point(4, 0),
        half(),
    )
    .unwrap();

    assert_eq!(
        first.relation_to_rational_quadratic(&first, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        first.relation_to_rational_quadratic(&far, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        first.relation_to_rational_quadratic(&touching, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
}

#[test]
fn rational_quadratic_curve_relation_certifies_projective_weight_identity() {
    let first = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(1_i8),
        Real::from(2_i8),
        Real::from(3_i8),
    )
    .unwrap();
    let scaled = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(5_i8),
        Real::from(10_i8),
        Real::from(15_i8),
    )
    .unwrap();
    let reversed_scaled = RationalQuadraticBezier2::try_new(
        point(4, 0),
        point(2, 4),
        point(0, 0),
        Real::from(15_i8),
        Real::from(10_i8),
        Real::from(5_i8),
    )
    .unwrap();
    let non_proportional = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(1_i8),
        Real::from(2_i8),
        Real::from(4_i8),
    )
    .unwrap();

    assert_eq!(
        first.relation_to_rational_quadratic(&scaled, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        scaled.relation_to_rational_quadratic(&first, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        first.relation_to_rational_quadratic(&reversed_scaled, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_eq!(
        reversed_scaled.relation_to_rational_quadratic(&first, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
    assert_ne!(
        first.relation_to_rational_quadratic(&non_proportional, &policy()),
        Classification::Decided(BezierCurveRelation::SameCurveImage)
    );
}

#[test]
fn rational_curve_relations_promote_or_isolate_overlapping_cases() {
    let first = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        half(),
    )
    .unwrap();
    let second = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, -2),
        point(4, 2),
        half(),
    )
    .unwrap();
    let polynomial = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    match first.relation_to_rational_quadratic(&second, &policy()) {
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) => {
            assert!(!points.is_empty());
        }
        Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) => {
            assert!(!regions.is_empty());
        }
        relation => {
            panic!(
                "overlapping positive-weight conics should promote certified roots or retain subdivision regions: {relation:?}"
            );
        }
    };

    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        first.relation_to_quadratic(&polynomial, &policy())
    else {
        panic!("overlapping rational/polynomial curves should retain subdivision regions");
    };
    assert!(!regions.is_empty());
    assert_eq!(
        polynomial.relation_to_rational_quadratic(&first, &policy()),
        first.relation_to_quadratic(&polynomial, &policy())
    );
}

#[test]
fn equal_weight_rational_relations_reuse_polynomial_quadratic_dispatch() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    let rational_crossing = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, -2),
        point(4, 2),
        Real::one(),
    )
    .unwrap();
    let polynomial_crossing = QuadraticBezier2::new(point(0, 2), point(2, -2), point(4, 2));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_arch.relation_to_rational_quadratic(&rational_crossing, &policy())
    else {
        panic!("equal-weight rational quadratics should reuse polynomial intersection dispatch");
    };
    assert_eq!(points.len(), 2);

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_arch.relation_to_quadratic(&polynomial_crossing, &policy())
    else {
        panic!("equal-weight rational/polynomial relation should reuse polynomial dispatch");
    };
    assert_eq!(points.len(), 2);
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        polynomial_crossing.relation_to_rational_quadratic(&rational_arch, &policy())
    else {
        panic!("polynomial/rational equal-weight relation should be symmetric");
    };
    assert_eq!(points.len(), 2);
}

#[test]
fn matching_weight_rational_relations_promote_same_parameter_hits() {
    let first = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(3, 0),
        point(6, 0),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();
    let second = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(r(3), ratio(-1, 4)),
        Point2::new(r(6), r(-2)),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        first.relation_to_rational_quadratic(&second, &policy())
    else {
        panic!("matching non-equal rational weights should promote certified same-parameter roots");
    };
    assert_eq!(points.len(), 1);
    let expected = first.point_at(ratio(1, 3), &policy());
    assert_eq!(expected, Classification::Decided(points[0].point().clone()));
}

#[test]
fn rational_shared_endpoint_does_not_hide_exact_interior_hits() {
    let first = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(1, 2),
        point(2, 0),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();
    let target = Point2::new(Real::one(), ratio(4, 3));
    let second = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(1, 0),
        point(2, 8),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();
    let polynomial = QuadraticBezier2::new(
        point(0, 0),
        point(1, 0),
        Point2::new(Real::from(2_i8), ratio(16, 3)),
    );

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        first.relation_to_rational_quadratic(&second, &policy())
    else {
        panic!("rational shared endpoint plus interior hit should retain both exact points");
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &target));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        first.relation_to_quadratic(&polynomial, &policy())
    else {
        panic!(
            "rational/polynomial shared endpoint plus interior hit should retain both exact points"
        );
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &target));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        polynomial.relation_to_rational_quadratic(&first, &policy())
    else {
        panic!(
            "polynomial/rational shared endpoint plus interior hit should retain both exact points"
        );
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &target));
}

#[test]
fn rational_polynomial_relations_promote_same_parameter_dyadic_hits() {
    let rational = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(256, 10),
        point(512, 0),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();
    let parameter = ratio(1, 512);
    let Classification::Decided(target) = rational.point_at(parameter.clone(), &policy()) else {
        panic!("positive rational weights should evaluate at a dyadic parameter");
    };
    let polynomial = quadratic_through_point_at(parameter, &target, (5, 7), (-3, 11));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational.relation_to_quadratic(&polynomial, &policy())
    else {
        panic!("rational/polynomial relation should promote certified dyadic same-parameter hits");
    };
    assert!(points.iter().any(|point| point.point() == &target));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        polynomial.relation_to_rational_quadratic(&rational, &policy())
    else {
        panic!("polynomial/rational relation should preserve dyadic same-parameter promotion");
    };
    assert!(points.iter().any(|point| point.point() == &target));

    let cubic = cubic_through_point_at(ratio(1, 512), &target, (5, 7), (-3, 11), (13, -5));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational.relation_to_cubic(&cubic, &policy())
    else {
        panic!("rational/cubic relation should promote certified dyadic same-parameter hits");
    };
    assert!(points.iter().any(|point| point.point() == &target));

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic.relation_to_rational_quadratic(&rational, &policy())
    else {
        panic!("cubic/rational relation should preserve dyadic same-parameter promotion");
    };
    assert!(points.iter().any(|point| point.point() == &target));
}

#[test]
fn rational_relations_certify_endpoint_on_conic_interior() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 3),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let polynomial_probe = CubicBezier2::new(point(2, 2), point(3, 5), point(5, 5), point(6, 4));
    let rational_probe = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, 2),
        point(3, 5),
        point(6, 4),
        Real::from(2_i8),
    )
    .unwrap();

    let relation = polynomial_probe.relation_to_rational_quadratic(&rational_arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("polynomial endpoint on rational conic interior should be certified: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    assert_eq!(
        rational_arch.relation_to_cubic(&polynomial_probe, &policy()),
        Classification::Decided(BezierCurveRelation::EndpointIntersections { points })
    );

    let relation = rational_probe.relation_to_rational_quadratic(&rational_arch, &policy());
    let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
    else {
        panic!("rational endpoint on rational conic interior should be certified: {relation:?}");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));

    let rational_baseline = RationalQuadraticBezier2::try_unit_end_weights(
        point(-1, 0),
        point(1, 0),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let isolated_cubic = CubicBezier2::new(point(0, -1), point(1, 1), point(2, 1), point(3, 1));
    let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
        rational_baseline.relation_to_cubic(&isolated_cubic, &policy())
    else {
        panic!("rational line-image versus cubic isolated roots should retain curve/curve regions");
    };
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].first().start(), &Real::zero());
    assert_eq!(regions[0].first().end(), &Real::one());
    assert_ne!(regions[0].second().start(), regions[0].second().end());
}

#[test]
fn rational_line_image_relations_use_exact_line_intersection() {
    let horizontal = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 0),
        point(4, 0),
        half(),
    )
    .unwrap();
    let vertical = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, -2),
        point(2, 0),
        point(2, 2),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) =
        horizontal.relation_to_rational_quadratic(&vertical, &policy())
    else {
        panic!("certified rational line images should use native line intersection");
    };
    assert_eq!(
        intersection,
        LineLineIntersection::Point {
            point: point(2, 0),
            a_param: half(),
            b_param: half(),
            kind: IntersectionKind::Crossing,
        }
    );

    let polynomial_vertical =
        CubicBezier2::new(point(2, -2), point(2, -1), point(2, 1), point(2, 2));
    assert_eq!(
        horizontal.relation_to_cubic(&polynomial_vertical, &policy()),
        Classification::Decided(BezierCurveRelation::LineSegmentIntersection {
            intersection: LineLineIntersection::Point {
                point: point(2, 0),
                a_param: half(),
                b_param: half(),
                kind: IntersectionKind::Crossing,
            }
        })
    );
    assert_eq!(
        polynomial_vertical.relation_to_rational_quadratic(&horizontal, &policy()),
        horizontal.relation_to_cubic(&polynomial_vertical, &policy())
    );

    let negative_horizontal = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 0),
        point(4, 0),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let negative_vertical = RationalQuadraticBezier2::try_new(
        point(2, -2),
        point(2, 0),
        point(2, 2),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }) =
        negative_horizontal.relation_to_rational_quadratic(&negative_vertical, &policy())
    else {
        panic!("same-sign negative rational line images should use native line intersection");
    };
    assert_eq!(
        intersection,
        LineLineIntersection::Point {
            point: point(2, 0),
            a_param: half(),
            b_param: half(),
            kind: IntersectionKind::Crossing,
        }
    );
}

#[test]
fn rational_line_contact_relation_classifies_crossings_and_tangencies() {
    let baseline = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let crossing = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, -2),
        point(2, 2),
        point(4, 2),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        crossing.relation_to_line_with_contacts(&baseline, &policy())
    else {
        panic!("rational conic crossing should expose represented line contacts");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);

    let tangent = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let tangent_y = (Real::from(8_i8) / Real::from(3_i8)).unwrap();
    let tangent_line = LineSeg2::try_new(
        Point2::new(Real::zero(), tangent_y.clone()),
        Point2::new(Real::from(4_i8), tangent_y),
    )
    .unwrap();
    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        tangent.relation_to_line_with_contacts(&tangent_line, &policy())
    else {
        panic!("rational conic tangent should expose represented line contact");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter(), &half());
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);

    let negative = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
        negative.relation_to_line_with_contacts(&tangent_line, &policy())
    else {
        panic!("negative same-sign rational conic tangent should classify exactly");
    };
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);
}

#[test]
fn rational_polynomial_line_image_relations_use_supporting_line_roots() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let baseline = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        baseline.relation_to_rational_quadratic(&rational_arch, &policy())
    else {
        panic!("polynomial line image should use rational supporting-line roots");
    };
    assert_eq!(points.len(), 2);
    assert!(points.iter().any(|hit| hit.point() == &point(0, 0)));
    assert!(points.iter().any(|hit| hit.point() == &point(4, 0)));

    let clipped = QuadraticBezier2::new(point(1, 0), point(2, 0), point(3, 0));
    assert_eq!(
        clipped.relation_to_rational_quadratic(&rational_arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 2),
        point(2, 2),
        point(4, 2),
        Real::from(2_i8),
    )
    .unwrap();
    let polynomial_arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_line.relation_to_quadratic(&polynomial_arch, &policy())
    else {
        panic!("rational line image should use polynomial quadratic line roots");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));
}

#[test]
fn rational_polynomial_curve_relation_is_symmetric_for_identity_disjoint_and_endpoint() {
    let rational_polynomial = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::one(),
    )
    .unwrap();
    let quadratic = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let far_quadratic = QuadraticBezier2::new(point(50, 50), point(52, 54), point(54, 50));
    let touching_cubic = CubicBezier2::new(point(4, 0), point(5, 2), point(7, 2), point(8, 0));
    let far_cubic = CubicBezier2::new(
        point(-80, -80),
        point(-79, -77),
        point(-75, -77),
        point(-72, -80),
    );

    assert_eq!(
        rational_polynomial.relation_to_quadratic(&quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        quadratic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::SameControlPolygon)
    );
    assert_eq!(
        rational_polynomial.relation_to_quadratic(&far_quadratic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_quadratic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        rational_polynomial.relation_to_cubic(&touching_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        touching_cubic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::SharedEndpoint)
    );
    assert_eq!(
        rational_polynomial.relation_to_cubic(&far_cubic, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
    assert_eq!(
        far_cubic.relation_to_rational_quadratic(&rational_polynomial, &policy()),
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
    );
}

#[test]
fn certified_bezier_flattening_emits_only_when_flatness_is_proven() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let Classification::Decided(polyline) = curve.flatten_certified(&options, &policy()) else {
        panic!("integer quadratic should flatten under a generous certified depth");
    };

    assert_eq!(polyline.points().first(), Some(&point(0, 0)));
    assert_eq!(polyline.points().last(), Some(&point(4, 0)));
    assert_eq!(
        polyline.certificate().segment_count(),
        polyline.points().len() - 1
    );
    assert!(
        polyline.points().len() > 2,
        "curved quadratic should be split before certification"
    );

    let tiny_error = (Real::one() / Real::from(10_i8)).unwrap();
    let shallow = BezierFlatteningOptions::try_new(tiny_error, 1, &policy()).unwrap();
    assert_eq!(
        curve.flatten_certified(&shallow, &policy()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    );
    assert_eq!(
        BezierFlatteningOptions::try_new(Real::zero(), 12, &policy()),
        Err(hypercurve::CurveError::InvalidFlatteningOptions)
    );
}

#[test]
fn certified_flattening_simplifies_only_exactly_collinear_vertices() {
    let line_like = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = line_like
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let simplified = polyline
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();

    assert_eq!(simplified.points(), &[point(0, 0), point(4, 0)]);
    assert_eq!(simplified.certificate().segment_count(), 1);
    assert_eq!(
        simplified.certificate().max_error(),
        polyline.certificate().max_error()
    );
    assert_eq!(simplified.certificate().max_depth(), options.max_depth());
    let simplification = simplified
        .simplification_certificate()
        .expect("simplified polyline should carry a simplification certificate");
    assert_eq!(simplification.source_start(), 0);
    assert_eq!(simplification.source_end(), polyline.points().len());
    assert_eq!(simplification.retained_vertex_count(), 2);
    assert_eq!(
        simplification.removed_vertex_count(),
        polyline.points().len() - 2
    );
    assert_eq!(simplification.error_bound(), &Real::zero());
    assert_eq!(
        simplification.source_flattening_error(),
        polyline.certificate().max_error()
    );
    assert_eq!(
        simplification.source_flattening_max_depth(),
        options.max_depth()
    );
    assert_eq!(
        simplification.construction_policy().numeric_mode,
        NumericMode::Certified
    );
    assert_eq!(
        simplification.metric(),
        BezierSimplificationErrorMetric::ExactPolylineImageDistance
    );
    assert_eq!(
        simplification.bound_kind(),
        BezierSimplificationBoundKind::ProvenExact
    );

    let curved = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let curved_polyline = curved
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let curved_simplified = curved_polyline
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();
    assert_eq!(curved_simplified.points(), curved_polyline.points());
    assert_eq!(
        curved_simplified
            .simplification_certificate()
            .expect("unchanged simplification should still carry a certificate")
            .removed_vertex_count(),
        0
    );
}

#[test]
fn display_offset_left_offsets_certified_flattened_chords_without_topology_claims() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test()
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();
    let offset = polyline.display_offset_left(Real::one()).unwrap();

    assert_eq!(offset.segments().len(), 1);
    assert_eq!(offset.segments()[0].start(), &point(0, 1));
    assert_eq!(offset.segments()[0].end(), &point(4, 1));
    assert_eq!(
        offset.source_certificate().max_error(),
        polyline.certificate().max_error()
    );

    let right_offset = polyline.display_offset_right(Real::one()).unwrap();
    assert_eq!(right_offset.segments().len(), 1);
    assert_eq!(right_offset.segments()[0].start(), &point(0, -1));
    assert_eq!(right_offset.segments()[0].end(), &point(4, -1));
    assert_eq!(
        right_offset.source_certificate().max_error(),
        polyline.certificate().max_error()
    );
    assert_eq!(right_offset.distance(), &Real::from(-1_i8));
}

#[test]
fn checked_offset_left_promotes_certified_polyline_to_checked_curve_string() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test()
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();
    let offset = polyline
        .checked_offset_left(Real::one(), &policy())
        .unwrap()
        .unwrap_decided_for_test();

    assert_eq!(offset.curve().segments().len(), 1);
    assert_eq!(offset.curve().segments()[0].start(), &point(0, 1));
    assert_eq!(offset.curve().segments()[0].end(), &point(4, 1));
    assert_eq!(
        offset.source_certificate().max_error(),
        polyline.certificate().max_error()
    );
    assert_eq!(offset.distance(), &Real::one());

    let right_offset = polyline
        .checked_offset_right(Real::one(), &policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(right_offset.curve().segments().len(), 1);
    assert_eq!(right_offset.curve().segments()[0].start(), &point(0, -1));
    assert_eq!(right_offset.curve().segments()[0].end(), &point(4, -1));
    assert_eq!(
        right_offset.source_certificate().max_error(),
        polyline.certificate().max_error()
    );
    assert_eq!(right_offset.distance(), &Real::from(-1_i8));
}

#[test]
fn checked_offset_left_rejects_collapsed_certified_polyline() {
    let curve = CubicBezier2::new(point(2, 3), point(2, 3), point(2, 3), point(2, 3));
    let options = BezierFlatteningOptions::try_new(Real::one(), 4, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();

    assert_eq!(
        polyline.checked_offset_left(Real::one(), &policy()),
        Err(hypercurve::CurveError::ZeroLengthLine)
    );
}

#[test]
fn certified_flattened_polyline_fits_exact_line_only_for_zero_error_cases() {
    let line_like = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let curved = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();

    let line_polyline = line_like
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let Classification::Decided(BezierLineFitRelation::Fit(fit)) =
        line_polyline.fit_exact_line(&policy()).unwrap()
    else {
        panic!("exact collinear flattened Bezier should fit a line");
    };
    assert_eq!(fit.line().start(), &point(0, 0));
    assert_eq!(fit.line().end(), &point(4, 0));
    assert_eq!(
        fit.source_certificate().max_error(),
        line_polyline.certificate().max_error()
    );
    assert_eq!(fit.fit_certificate().source_start(), 0);
    assert_eq!(
        fit.fit_certificate().source_end(),
        line_polyline.points().len()
    );
    assert_eq!(fit.fit_certificate().fit_error_bound(), &Real::zero());
    assert_eq!(
        fit.fit_certificate().source_flattening_error(),
        Some(line_polyline.certificate().max_error())
    );
    assert_eq!(
        fit.fit_certificate().source_flattening_max_depth(),
        Some(options.max_depth())
    );
    assert_eq!(
        fit.fit_certificate().construction_policy().numeric_mode,
        NumericMode::Certified
    );
    assert_eq!(
        fit.fit_certificate().metric(),
        BezierFitErrorMetric::ExactEuclideanDistance
    );
    assert_eq!(
        fit.fit_certificate().bound_kind(),
        BezierFitBoundKind::ProvenExact
    );

    let curved_polyline = curved
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    assert_eq!(
        curved_polyline.fit_exact_line(&policy()).unwrap(),
        Classification::Decided(BezierLineFitRelation::NotLine)
    );
}

#[test]
fn certified_fit_readiness_reports_exact_point_line_and_higher_order_cases() {
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();

    let collapsed = CubicBezier2::new(point(2, 3), point(2, 3), point(2, 3), point(2, 3))
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let point_report = collapsed
        .fit_readiness_report(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(point_report.status(), BezierFitReadinessStatus::ExactPoint);
    assert!(point_report.has_exact_primitive_fit());
    assert!(!point_report.needs_higher_order_fit());
    assert_eq!(point_report.source_vertex_count(), collapsed.points().len());
    assert_eq!(
        point_report.source_segment_count(),
        collapsed.points().len().saturating_sub(1)
    );
    assert_eq!(
        point_report.source_certificate().max_error(),
        collapsed.certificate().max_error()
    );
    assert!(point_report.point_fit_certificate().is_some());
    assert!(point_report.line_fit_certificate().is_none());

    let line_like = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0))
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let line_report = line_like
        .fit_readiness_report(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(line_report.status(), BezierFitReadinessStatus::ExactLine);
    assert!(line_report.has_exact_primitive_fit());
    assert!(line_report.point_fit_certificate().is_none());
    assert!(line_report.line_fit_certificate().is_some());
    assert_eq!(
        line_report
            .line_fit_certificate()
            .expect("line readiness should retain a fit certificate")
            .source_end(),
        line_like.points().len()
    );

    let curved = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0))
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let curved_report = curved
        .fit_readiness_report(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    assert_eq!(
        curved_report.status(),
        BezierFitReadinessStatus::NeedsHigherOrderFit
    );
    assert!(!curved_report.has_exact_primitive_fit());
    assert!(curved_report.needs_higher_order_fit());
    assert!(curved_report.point_fit_certificate().is_none());
    assert!(curved_report.line_fit_certificate().is_none());
}

#[test]
fn certified_fit_readiness_preserves_simplification_certificate() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let simplified = polyline
        .simplify_exact_collinear(&policy())
        .unwrap_decided_for_test();

    let report = simplified
        .fit_readiness_report(&policy())
        .unwrap()
        .unwrap_decided_for_test();

    assert_eq!(report.status(), BezierFitReadinessStatus::ExactLine);
    let simplification = report
        .simplification_certificate()
        .expect("readiness should retain upstream simplification certificate");
    assert_eq!(simplification.source_end(), polyline.points().len());
    assert_eq!(
        simplification.retained_vertex_count(),
        simplified.points().len()
    );
    assert_eq!(simplification.error_bound(), &Real::zero());
}

#[test]
fn polynomial_bezier_line_image_fits_without_flattening() {
    let quadratic = QuadraticBezier2::new(point(0, 3), point(2, 3), point(6, 3));
    let fit = quadratic
        .fit_exact_line_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierLineImageFitRelation::Fit(fit) = fit else {
        panic!("collinear control polygon should fit its endpoint line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &point(0, 3));
    assert_eq!(fit.line().end(), &point(6, 3));
    assert_eq!(fit.fit_certificate().source_start(), 0);
    assert_eq!(fit.fit_certificate().source_end(), 3);
    assert_eq!(fit.fit_certificate().fit_error_bound(), &Real::zero());
    assert_eq!(fit.fit_certificate().source_flattening_error(), None);
    assert_eq!(fit.fit_certificate().source_flattening_max_depth(), None);
    assert_eq!(
        fit.fit_certificate().construction_policy().numeric_mode,
        NumericMode::Certified
    );
    let offset = fit.offset_left_exact(Real::from(2_i8)).unwrap();
    assert_eq!(offset.line().start(), &point(0, 5));
    assert_eq!(offset.line().end(), &point(6, 5));
    assert_eq!(offset.control_point_count(), 3);
    assert_eq!(offset.distance(), &Real::from(2_i8));
    assert_eq!(offset.fit_certificate(), fit.fit_certificate());
    let right_offset = fit.offset_right_exact(Real::from(2_i8)).unwrap();
    assert_eq!(right_offset.line().start(), &point(0, 1));
    assert_eq!(right_offset.line().end(), &point(6, 1));
    assert_eq!(right_offset.control_point_count(), 3);
    assert_eq!(right_offset.distance(), &Real::from(-2_i8));
    assert_eq!(right_offset.fit_certificate(), fit.fit_certificate());

    let off_line = CubicBezier2::new(point(0, 0), point(1, 2), point(3, 2), point(4, 0));
    assert_eq!(
        off_line
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test(),
        BezierLineImageFitRelation::NotLine
    );
}

#[test]
fn rational_bezier_line_image_fit_accepts_same_sign_weights() {
    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(2_i8),
    )
    .unwrap();
    let fit = rational_line
        .fit_exact_line_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierLineImageFitRelation::Fit(fit) = fit else {
        panic!("positive-weight collinear conic should fit its endpoint line image");
    };
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.line().start(), &point(0, 3));
    assert_eq!(fit.line().end(), &point(6, 3));
    let offset = fit.offset_left_exact(Real::from(2_i8)).unwrap();
    assert_eq!(offset.line().start(), &point(0, 5));
    assert_eq!(offset.fit_certificate(), fit.fit_certificate());

    let curved = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        curved
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test(),
        BezierLineImageFitRelation::NotLine
    );

    let negative_weights = RationalQuadraticBezier2::try_new(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(-1_i8),
        Real::from(-1_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let fit = negative_weights
        .fit_exact_line_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierLineImageFitRelation::Fit(fit) = fit else {
        panic!("same-sign negative collinear conic should normalize to its endpoint line image");
    };
    assert_eq!(fit.line().start(), &point(0, 3));
    assert_eq!(fit.line().end(), &point(6, 3));
    let offset = fit.offset_left_exact(Real::from(2_i8)).unwrap();
    assert_eq!(offset.fit_certificate(), fit.fit_certificate());

    let mixed_weights = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 3),
        point(2, 3),
        point(6, 3),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        mixed_weights.fit_exact_line_image(&policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn bezier_point_image_fits_certify_collapsed_control_polygons() {
    let quadratic = QuadraticBezier2::new(point(3, 5), point(3, 5), point(3, 5));
    let fit = quadratic
        .fit_exact_point_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierPointImageFitRelation::Fit(fit) = fit else {
        panic!("collapsed quadratic should fit one exact point");
    };
    assert_eq!(fit.point(), &point(3, 5));
    assert_eq!(fit.control_point_count(), 3);
    assert_eq!(fit.fit_certificate().source_end(), 3);
    assert_eq!(fit.fit_certificate().fit_error_bound(), &Real::zero());
    assert_eq!(
        fit.fit_certificate().metric(),
        BezierFitErrorMetric::ExactEuclideanDistance
    );

    let cubic = CubicBezier2::new(point(3, 5), point(3, 5), point(3, 5), point(3, 5));
    let fit = cubic
        .fit_exact_point_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierPointImageFitRelation::Fit(fit) = fit else {
        panic!("collapsed cubic should fit one exact point");
    };
    assert_eq!(fit.point(), &point(3, 5));
    assert_eq!(fit.control_point_count(), 4);
    assert_eq!(fit.fit_certificate().source_end(), 4);

    let nonpoint = QuadraticBezier2::new(point(3, 5), point(3, 5), point(4, 5));
    assert_eq!(
        nonpoint
            .fit_exact_point_image(&policy())
            .unwrap()
            .unwrap_decided_for_test(),
        BezierPointImageFitRelation::NotPoint
    );
}

#[test]
fn rational_point_image_fit_accepts_same_sign_weights() {
    let rational = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, -3),
        point(2, -3),
        point(2, -3),
        Real::from(2_i8),
    )
    .unwrap();
    let fit = rational
        .fit_exact_point_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierPointImageFitRelation::Fit(fit) = fit else {
        panic!("same-sign collapsed rational conic should fit one exact point");
    };
    assert_eq!(fit.point(), &point(2, -3));
    assert_eq!(fit.control_point_count(), 3);

    let negative_weights = RationalQuadraticBezier2::try_new(
        point(2, -3),
        point(2, -3),
        point(2, -3),
        Real::from(-1_i8),
        Real::from(-1_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let fit = negative_weights
        .fit_exact_point_image(&policy())
        .unwrap()
        .unwrap_decided_for_test();
    let BezierPointImageFitRelation::Fit(fit) = fit else {
        panic!("same-sign negative collapsed rational conic should fit one exact point");
    };
    assert_eq!(fit.point(), &point(2, -3));

    let mixed_weights = RationalQuadraticBezier2::try_unit_end_weights(
        point(2, -3),
        point(2, -3),
        point(2, -3),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        mixed_weights.fit_exact_point_image(&policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn point_image_curve_relations_use_exact_point_solvers() {
    let arch = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let point_hit = QuadraticBezier2::new(point(2, 2), point(2, 2), point(2, 2));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        point_hit.relation_to_quadratic(&arch, &policy())
    else {
        panic!("collapsed point image on quadratic should promote exact point intersection");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &point(2, 2));
    assert_eq!(
        arch.relation_to_quadratic(&point_hit, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { points })
    );

    let point_miss = QuadraticBezier2::new(point(2, 1), point(2, 1), point(2, 1));
    assert_eq!(
        point_miss.relation_to_quadratic(&arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let cubic = CubicBezier2::new(point(0, 0), point(0, 4), point(4, 4), point(4, 0));
    let cubic_midpoint = QuadraticBezier2::new(point(2, 3), point(2, 3), point(2, 3));
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        cubic_midpoint.relation_to_cubic(&cubic, &policy())
    else {
        panic!("collapsed point image at dyadic cubic point should promote certified hit");
    };
    assert_eq!(points[0].point(), &point(2, 3));
}

#[test]
fn rational_point_image_relations_use_exact_point_solvers() {
    let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let rational_midpoint = rational_arch
        .point_at(half(), &policy())
        .unwrap_decided_for_test();
    let rational_point = RationalQuadraticBezier2::try_new(
        rational_midpoint.clone(),
        rational_midpoint.clone(),
        rational_midpoint.clone(),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();

    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_point.relation_to_rational_quadratic(&rational_arch, &policy())
    else {
        panic!("collapsed same-sign rational point should promote conic point intersection");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point(), &rational_midpoint);

    let polynomial_point = QuadraticBezier2::new(
        rational_midpoint.clone(),
        rational_midpoint.clone(),
        rational_midpoint.clone(),
    );
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        rational_arch.relation_to_quadratic(&polynomial_point, &policy())
    else {
        panic!("collapsed polynomial point should promote mixed conic point intersection");
    };
    assert_eq!(points[0].point(), &rational_midpoint);

    let miss = RationalQuadraticBezier2::try_new(
        point(2, 1),
        point(2, 1),
        point(2, 1),
        Real::from(-1_i8),
        Real::from(-1_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        miss.relation_to_rational_quadratic(&rational_arch, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn matching_weight_rational_graph_relations_certify_hits_and_no_hits() {
    let first = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let crossing = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 4),
        point(2, 2),
        point(4, 4),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
        first.relation_to_rational_quadratic(&crossing, &policy())
    else {
        panic!("matching-weight rational graph crossing should promote exact same-parameter hit");
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0].point(),
        &first.point_at(half(), &policy()).unwrap_decided_for_test()
    );

    let shifted = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 1),
        point(2, 5),
        point(4, 1),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        first.relation_to_rational_quadratic(&shifted, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let negative = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let negative_shifted = RationalQuadraticBezier2::try_new(
        point(0, 1),
        point(2, 5),
        point(4, 1),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        negative.relation_to_rational_quadratic(&negative_shifted, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
}

#[test]
fn matching_weight_rational_graph_order_classifies_monotone_ranges() {
    let lower = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let upper = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 1),
        point(2, 5),
        point(4, 1),
        Real::from(2_i8),
    )
    .unwrap();

    assert_eq!(
        lower.graph_order_to_rational_quadratic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );
    assert_eq!(
        upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );

    let negative_lower = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let negative_upper = RationalQuadraticBezier2::try_new(
        point(0, 1),
        point(2, 5),
        point(4, 1),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    assert_eq!(
        negative_lower.graph_order_to_rational_quadratic_over_axis(
            &negative_upper,
            Axis2::X,
            &policy()
        ),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );

    let crossing = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 1),
        point(2, -4),
        point(4, 1),
        Real::from(2_i8),
    )
    .unwrap();
    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = lower.graph_order_to_rational_quadratic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("matching-weight rational graph crossing should retain exact or bracketed roots");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "crossing rational graph order should expose retained same-parameter candidates"
    );

    let mismatched_weight = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 1),
        point(2, 5),
        point(4, 1),
        Real::from(3_i8),
    )
    .unwrap();
    assert_eq!(
        lower.graph_order_to_rational_quadratic_over_axis(&mismatched_weight, Axis2::X, &policy(),),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
}

#[test]
fn equal_weight_rational_polynomial_graph_order_uses_polynomial_collapse() {
    let lower = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
        Real::from(2_i8),
        Real::from(2_i8),
    )
    .unwrap();
    let upper = QuadraticBezier2::new(point(0, 1), point(2, 5), point(4, 1));

    assert_eq!(
        lower.graph_order_to_quadratic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );
    assert_eq!(
        upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );

    let negative_lower = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(-2_i8),
        Real::from(-2_i8),
        Real::from(-2_i8),
    )
    .unwrap();
    assert_eq!(
        negative_lower.graph_order_to_quadratic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );

    let crossing = QuadraticBezier2::new(point(0, 1), point(2, -4), point(4, 1));
    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = lower.graph_order_to_quadratic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("equal-weight rational/polynomial graph crossing should retain roots");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "mixed graph order should expose retained same-parameter candidates"
    );

    let genuinely_rational = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(2, 4),
        point(4, 0),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        genuinely_rational.graph_order_to_quadratic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
    assert_eq!(
        upper
            .graph_order_to_rational_quadratic_over_axis(&genuinely_rational, Axis2::X, &policy(),),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
}

#[test]
fn non_equal_rational_quadratic_graph_order_certifies_strict_polynomial_gap() {
    let rational = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(ratio(1, 4), r(1)),
        Point2::new(r(1), r(1)),
        r(1),
        r(2),
        r(3),
    )
    .unwrap();
    let baseline = QuadraticBezier2::new(
        Point2::new(r(0), r(0)),
        Point2::new(half(), r(0)),
        Point2::new(r(1), r(0)),
    );

    assert_eq!(
        rational.graph_order_to_quadratic_over_axis(&baseline, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );
    assert_eq!(
        baseline.graph_order_to_rational_quadratic_over_axis(&rational, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );

    let crossing = QuadraticBezier2::new(
        Point2::new(r(0), r(2)),
        Point2::new(half(), r(0)),
        Point2::new(r(1), r(2)),
    );
    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = rational.graph_order_to_quadratic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("mixed quartic graph roots should be retained exactly or as brackets");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "non-strict mixed quartic graph order must retain root candidates"
    );
}

#[test]
fn non_equal_rational_quadratic_relation_uses_strict_graph_gap() {
    let rational = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(ratio(1, 4), r(1)),
        Point2::new(r(1), r(1)),
        r(1),
        r(2),
        r(3),
    )
    .unwrap();
    let baseline = QuadraticBezier2::new(
        Point2::new(r(0), r(0)),
        Point2::new(half(), r(0)),
        Point2::new(r(1), r(0)),
    );

    assert_eq!(
        rational.relation_to_quadratic(&baseline, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        baseline.relation_to_rational_quadratic(&rational, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let crossing = QuadraticBezier2::new(
        Point2::new(r(0), r(2)),
        Point2::new(half(), r(0)),
        Point2::new(r(1), r(2)),
    );
    assert!(matches!(
        rational.relation_to_quadratic(&crossing, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { .. })
            | Classification::Decided(BezierCurveRelation::IntersectionRegions { .. })
    ));
}

#[test]
fn non_equal_rational_cubic_graph_order_certifies_strict_polynomial_gap() {
    let rational = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(ratio(1, 4), r(1)),
        Point2::new(r(1), r(1)),
        r(1),
        r(2),
        r(3),
    )
    .unwrap();
    let baseline = CubicBezier2::new(
        Point2::new(r(0), r(0)),
        Point2::new(ratio(1, 3), r(0)),
        Point2::new(ratio(2, 3), r(0)),
        Point2::new(r(1), r(0)),
    );

    assert_eq!(
        rational.graph_order_to_cubic_over_axis(&baseline, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );
    assert_eq!(
        baseline.graph_order_to_rational_quadratic_over_axis(&rational, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );

    let negative = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(ratio(1, 4), r(1)),
        Point2::new(r(1), r(1)),
        r(-1),
        r(-2),
        r(-3),
    )
    .unwrap();
    assert_eq!(
        negative.graph_order_to_cubic_over_axis(&baseline, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );

    let crossing = CubicBezier2::new(
        Point2::new(r(0), r(2)),
        Point2::new(ratio(1, 3), r(0)),
        Point2::new(ratio(2, 3), r(0)),
        Point2::new(r(1), r(2)),
    );
    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = rational.graph_order_to_cubic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("mixed quintic graph roots should be retained exactly or as brackets");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "non-strict mixed quintic graph order must retain root candidates"
    );
}

#[test]
fn non_equal_rational_cubic_relation_uses_strict_graph_gap() {
    let rational = RationalQuadraticBezier2::try_new(
        Point2::new(r(0), r(1)),
        Point2::new(ratio(1, 4), r(1)),
        Point2::new(r(1), r(1)),
        r(1),
        r(2),
        r(3),
    )
    .unwrap();
    let baseline = CubicBezier2::new(
        Point2::new(r(0), r(0)),
        Point2::new(ratio(1, 3), r(0)),
        Point2::new(ratio(2, 3), r(0)),
        Point2::new(r(1), r(0)),
    );

    assert_eq!(
        rational.relation_to_cubic(&baseline, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );
    assert_eq!(
        baseline.relation_to_rational_quadratic(&rational, &policy()),
        Classification::Decided(BezierCurveRelation::NoIntersection)
    );

    let crossing = CubicBezier2::new(
        Point2::new(r(0), r(2)),
        Point2::new(ratio(1, 3), r(0)),
        Point2::new(ratio(2, 3), r(0)),
        Point2::new(r(1), r(2)),
    );
    assert!(matches!(
        rational.relation_to_cubic(&crossing, &policy()),
        Classification::Decided(BezierCurveRelation::IntersectionPoints { .. })
            | Classification::Decided(BezierCurveRelation::IntersectionRegions { .. })
    ));
}

#[test]
fn equal_weight_rational_cubic_graph_order_degree_normalizes_after_collapse() {
    let lower = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(3, 6),
        point(6, 0),
        Real::from(2_i8),
        Real::from(2_i8),
        Real::from(2_i8),
    )
    .unwrap();
    let upper = CubicBezier2::new(point(0, 2), point(2, 6), point(4, 6), point(6, 2));

    assert_eq!(
        lower.graph_order_to_cubic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );
    assert_eq!(
        upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
    );

    let negative_lower = RationalQuadraticBezier2::try_new(
        point(0, 0),
        point(3, 6),
        point(6, 0),
        Real::from(-2_i8),
        Real::from(-2_i8),
        Real::from(-2_i8),
    )
    .unwrap();
    assert_eq!(
        negative_lower.graph_order_to_cubic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
    );

    let crossing = CubicBezier2::new(point(0, -1), point(2, 6), point(4, 6), point(6, -1));
    let Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
        parameters,
        spans,
    }) = lower.graph_order_to_cubic_over_axis(&crossing, Axis2::X, &policy())
    else {
        panic!("equal-weight rational/cubic graph crossing should retain roots");
    };
    assert!(
        !parameters.is_empty() || !spans.is_empty(),
        "mixed rational/cubic graph order should expose retained same-parameter candidates"
    );

    let genuinely_rational = RationalQuadraticBezier2::try_unit_end_weights(
        point(0, 0),
        point(3, 6),
        point(6, 0),
        Real::from(2_i8),
    )
    .unwrap();
    assert_eq!(
        genuinely_rational.graph_order_to_cubic_over_axis(&upper, Axis2::X, &policy()),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
    assert_eq!(
        upper
            .graph_order_to_rational_quadratic_over_axis(&genuinely_rational, Axis2::X, &policy(),),
        Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone)
    );
}

#[test]
fn certified_line_fit_offsets_exactly_as_line_primitive() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 0), point(4, 0));
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let Classification::Decided(BezierLineFitRelation::Fit(fit)) =
        polyline.fit_exact_line(&policy()).unwrap()
    else {
        panic!("collinear Bezier should produce an exact line fit");
    };
    let offset = fit.offset_left_exact(Real::one()).unwrap();

    assert_eq!(offset.line().start(), &point(0, 1));
    assert_eq!(offset.line().end(), &point(4, 1));
    assert_eq!(offset.distance(), &Real::one());
    assert_eq!(
        offset.source_certificate().max_error(),
        fit.source_certificate().max_error()
    );
    assert_eq!(offset.fit_certificate(), fit.fit_certificate());

    let right_offset = fit.offset_right_exact(Real::one()).unwrap();
    assert_eq!(right_offset.line().start(), &point(0, -1));
    assert_eq!(right_offset.line().end(), &point(4, -1));
    assert_eq!(right_offset.distance(), &Real::from(-1_i8));
    assert_eq!(
        right_offset.source_certificate().max_error(),
        fit.source_certificate().max_error()
    );
    assert_eq!(right_offset.fit_certificate(), fit.fit_certificate());
}

#[test]
fn certified_flattened_polyline_fits_exact_point_with_certificate() {
    let curve = QuadraticBezier2::new(point(4, -2), point(4, -2), point(4, -2));
    let options = BezierFlatteningOptions::try_new(Real::one(), 4, &policy()).unwrap();
    let polyline = curve
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    let fit = polyline.fit_exact_point(&policy()).unwrap();
    let Classification::Decided(BezierPointFitRelation::Fit(fit)) = fit else {
        panic!("collapsed certified polyline should fit one exact point");
    };
    assert_eq!(fit.point(), &point(4, -2));
    assert_eq!(fit.source_certificate(), polyline.certificate());

    let nonpoint = QuadraticBezier2::new(point(0, 0), point(0, 0), point(2, 0))
        .flatten_certified(&options, &policy())
        .unwrap_decided_for_test();
    assert_eq!(
        nonpoint.fit_exact_point(&policy()).unwrap(),
        Classification::Decided(BezierPointFitRelation::NotPoint)
    );
}

#[test]
fn bezier_boolean_handoff_accepts_parameterized_line_image_point_events() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation = BezierCurveRelation::LineSegmentIntersection { intersection };

    let report = BezierBooleanHandoffReport2::from_relation(&relation);

    assert_eq!(report.status, BezierBooleanHandoffStatus::SplitEventsReady);
    assert!(report.can_feed_split_events());
    assert!(!report.has_blockers());
    assert_eq!(report.point_events.len(), 1);
    assert_eq!(report.point_events[0].first_param, half());
    assert_eq!(report.point_events[0].second_param, half());
    assert_eq!(report.point_events[0].point, Some(point(2, 0)));
    assert_eq!(
        report.point_events[0].kind,
        Some(IntersectionKind::Crossing)
    );
}

#[test]
fn bezier_boolean_handoff_blocks_point_witnesses_without_parameters() {
    let relation = BezierCurveRelation::IntersectionPoints {
        points: vec![hypercurve::BezierCurveIntersectionPoint::new(point(1, 1))],
    };

    let report = BezierBooleanHandoffReport2::from_relation(&relation);

    assert_eq!(
        report.status,
        BezierBooleanHandoffStatus::NeedsParameterRecovery
    );
    assert!(report.has_blockers());
    assert_eq!(report.point_witnesses_needing_parameters, 1);
    assert!(report.point_events.is_empty());
}

#[test]
fn bezier_boolean_handoff_keeps_overlap_resolution_explicit() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, 0), point(6, 0)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation = BezierCurveRelation::LineSegmentIntersection { intersection };

    let report = BezierBooleanHandoffReport2::from_relation(&relation);

    assert_eq!(
        report.status,
        BezierBooleanHandoffStatus::NeedsOverlapResolver
    );
    assert!(report.has_blockers());
    assert_eq!(report.overlap_relations_needing_resolution, 1);
    assert_eq!(report.overlap_events.len(), 1);
}

#[test]
fn bezier_boolean_overlap_resolution_extracts_sorted_split_boundaries() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(6, 0), point(2, 0)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let handoff =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection,
        });

    let report = BezierBooleanOverlapResolutionReport2::from_handoff_reports(&[handoff], &policy())
        .unwrap_decided_for_test();

    assert_eq!(report.status, BezierBooleanOverlapResolutionStatus::Ready);
    assert!(report.is_ready());
    assert_eq!(report.overlap_event_count, 1);
    assert_eq!(report.resolved_events.len(), 1);
    assert_eq!(
        report.first_curve_boundary_parameters,
        vec![half(), Real::one()]
    );
    assert_eq!(
        report.second_curve_boundary_parameters,
        vec![half(), Real::one()]
    );
    assert_eq!(
        report.resolved_events[0].second_boundary_parameters,
        vec![half(), Real::one()]
    );
    assert_eq!(report.invalid_range_count, 0);
}

#[test]
fn bezier_boolean_overlap_resolution_preserves_blockers_and_invalid_ranges() {
    let blocked = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let blocked_report =
        BezierBooleanOverlapResolutionReport2::from_handoff_reports(&[blocked], &policy())
            .unwrap_decided_for_test();
    assert_eq!(
        blocked_report.status,
        BezierBooleanOverlapResolutionStatus::Blocked
    );
    assert!(blocked_report.has_blockers());
    assert_eq!(blocked_report.blocker_count, 1);

    let invalid_event = hypercurve::BezierBooleanOverlapEvent2 {
        first_range: ParamRange::new(ratio(-1, 4), half()),
        second_range: ParamRange::new(Real::zero(), Real::one()),
    };
    let invalid =
        BezierBooleanOverlapResolutionReport2::from_overlap_events(&[invalid_event], &policy())
            .unwrap_decided_for_test();
    assert_eq!(
        invalid.status,
        BezierBooleanOverlapResolutionStatus::InvalidParameterDomain
    );
    assert!(invalid.has_blockers());
    assert_eq!(invalid.invalid_range_count, 1);
    assert!(invalid.resolved_events.is_empty());
}

#[test]
fn bezier_boolean_handoff_promotes_exact_point_regions_to_split_events() {
    let region =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let relation = BezierCurveRelation::IntersectionRegions {
        regions: vec![region],
    };

    let report = BezierBooleanHandoffReport2::from_relation(&relation);

    assert_eq!(report.status, BezierBooleanHandoffStatus::SplitEventsReady);
    assert!(report.can_feed_split_events());
    assert_eq!(report.point_events.len(), 1);
    assert_eq!(report.point_events[0].first_param, half());
    assert_eq!(report.point_events[0].second_param, half());
    assert_eq!(report.region_summary.as_ref().unwrap().exact_point_cells, 1);
}

#[test]
fn bezier_boolean_handoff_requires_isolation_for_positive_width_regions() {
    let region = hypercurve::BezierCurveIntersectionRegion::new(
        span(ratio(1, 4), ratio(1, 2)),
        span(ratio(1, 4), ratio(1, 2)),
    );
    let relation = BezierCurveRelation::IntersectionRegions {
        regions: vec![region],
    };

    let report = BezierBooleanHandoffReport2::from_relation(&relation);

    assert_eq!(
        report.status,
        BezierBooleanHandoffStatus::NeedsRegionIsolation
    );
    assert!(report.has_blockers());
    assert_eq!(
        report
            .region_summary
            .as_ref()
            .unwrap()
            .same_parameter_isolating_spans,
        1
    );
}

#[test]
fn bezier_boolean_handoff_replays_isolation_certificate_readiness() {
    let region =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let report = isolate_bezier_intersection_regions(
        &[region],
        BezierIntersectionRegionIsolationBudget {
            max_steps: 8,
            max_depth: 2,
            max_terminal_regions: 8,
        },
    );
    let certificate = certify_bezier_intersection_region_isolation(&report);

    let handoff = BezierBooleanHandoffReport2::from_isolation_certificate(&certificate);

    assert_eq!(handoff.status, BezierBooleanHandoffStatus::SplitEventsReady);
    assert!(handoff.can_feed_split_events());
    assert_eq!(
        handoff.region_summary.as_ref().unwrap().exact_point_cells,
        1
    );
    assert_eq!(handoff.isolation_certificate, Some(certificate));
}

#[test]
fn bezier_boolean_handoff_preserves_classified_uncertainty() {
    let relation = Classification::Uncertain(UncertaintyReason::Ordering);

    let report = BezierBooleanHandoffReport2::from_classified_relation(&relation);

    assert_eq!(report.status, BezierBooleanHandoffStatus::Uncertain);
    assert!(report.has_blockers());
    assert_eq!(report.uncertain_relations, 1);
    assert_eq!(report.uncertainty_reason, Some(UncertaintyReason::Ordering));
}

#[test]
fn bezier_boolean_batch_handoff_collects_split_events_and_no_events() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relations = vec![
        Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint),
        Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection }),
    ];

    let batch = BezierBooleanBatchHandoffReport2::from_classified_relations(&relations);

    assert_eq!(
        batch.status,
        BezierBooleanBatchHandoffStatus::SplitEventsReady
    );
    assert!(batch.can_feed_split_events());
    assert!(!batch.has_blockers());
    assert_eq!(batch.relation_count, 2);
    assert_eq!(batch.no_event_relation_count, 1);
    assert_eq!(batch.split_ready_relation_count, 1);
    assert_eq!(batch.point_events.len(), 1);
    assert_eq!(batch.point_events[0].first_param, half());
}

#[test]
fn bezier_boolean_batch_handoff_prioritizes_blockers() {
    let point_witness =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::IntersectionPoints {
            points: vec![hypercurve::BezierCurveIntersectionPoint::new(point(1, 1))],
        });
    let overlap = BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::SameCurveImage);
    let unresolved = BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::Unresolved);
    let uncertain = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );

    let parameter_blocked =
        BezierBooleanBatchHandoffReport2::from_handoff_reports(&[point_witness]);
    assert_eq!(
        parameter_blocked.status,
        BezierBooleanBatchHandoffStatus::NeedsParameterRecovery
    );
    assert!(parameter_blocked.has_blockers());

    let overlap_blocked = BezierBooleanBatchHandoffReport2::from_handoff_reports(&[overlap]);
    assert_eq!(
        overlap_blocked.status,
        BezierBooleanBatchHandoffStatus::NeedsOverlapResolver
    );
    assert_eq!(overlap_blocked.overlap_relations_needing_resolution, 1);

    let unresolved_blocked = BezierBooleanBatchHandoffReport2::from_handoff_reports(&[unresolved]);
    assert_eq!(
        unresolved_blocked.status,
        BezierBooleanBatchHandoffStatus::Unresolved
    );

    let uncertain_blocked = BezierBooleanBatchHandoffReport2::from_handoff_reports(&[uncertain]);
    assert_eq!(
        uncertain_blocked.status,
        BezierBooleanBatchHandoffStatus::Uncertain
    );
    assert_eq!(
        uncertain_blocked.uncertainty_reason,
        Some(UncertaintyReason::Ordering)
    );
}

#[test]
fn bezier_boolean_path_scheduler_combines_relation_and_range_split_events() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection,
        });
    let range = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
            contacts: vec![BezierGraphContact::new(
                half(),
                BezierLineContactKind::Tangent,
            )],
            spans: Vec::new(),
        },
    );

    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[range]);

    assert_eq!(
        scheduler.status,
        BezierBooleanPathSchedulerStatus::SplitEventsReady
    );
    assert!(scheduler.can_feed_split_events());
    assert!(!scheduler.has_blockers());
    assert_eq!(scheduler.relation_point_events.len(), 1);
    assert_eq!(scheduler.range_split_parameters, vec![half()]);
    assert_eq!(scheduler.represented_split_event_count, 2);
}

#[test]
fn bezier_boolean_path_scheduler_prioritizes_global_blockers() {
    let split_ready_region =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::IntersectionRegions {
            regions: vec![split_ready_region],
        });
    let overlap_range = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::Coincident,
    );

    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[overlap_range]);

    assert_eq!(
        scheduler.status,
        BezierBooleanPathSchedulerStatus::NeedsOverlapResolver
    );
    assert!(scheduler.has_blockers());
    assert!(!scheduler.can_feed_split_events());
    assert_eq!(scheduler.represented_split_event_count, 1);

    let uncertain_relation = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let ordered_range = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::FirstLess,
    );
    let uncertain_scheduler =
        BezierBooleanPathSchedulerReport2::from_reports(&[uncertain_relation], &[ordered_range]);
    assert_eq!(
        uncertain_scheduler.status,
        BezierBooleanPathSchedulerStatus::Uncertain
    );
    assert_eq!(
        uncertain_scheduler.uncertainty_reason,
        Some(UncertaintyReason::Ordering)
    );
}

#[test]
fn bezier_boolean_split_plan_extracts_ready_scheduler_parameters() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection,
        });
    let range = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
            contacts: vec![BezierGraphContact::new(
                ratio(1, 4),
                BezierLineContactKind::Crossing,
            )],
            spans: Vec::new(),
        },
    );
    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[range]);

    let plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);

    assert_eq!(plan.status, BezierBooleanSplitPlanStatus::Ready);
    assert!(plan.is_ready());
    assert!(!plan.has_blockers());
    assert_eq!(plan.first_curve_parameters, vec![half()]);
    assert_eq!(plan.second_curve_parameters, vec![half()]);
    assert_eq!(plan.shared_range_parameters, vec![ratio(1, 4)]);
    assert_eq!(plan.relation_event_count, 1);
    assert_eq!(plan.range_event_count, 1);
}

#[test]
fn bezier_boolean_split_plan_preserves_scheduler_blockers() {
    let relation = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[]);

    let plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);

    assert_eq!(plan.status, BezierBooleanSplitPlanStatus::Blocked);
    assert_eq!(
        plan.scheduler_status,
        BezierBooleanPathSchedulerStatus::Uncertain
    );
    assert!(plan.has_blockers());
    assert!(plan.first_curve_parameters.is_empty());
    assert!(plan.second_curve_parameters.is_empty());
    assert!(plan.shared_range_parameters.is_empty());
    assert_eq!(plan.uncertainty_reason, Some(UncertaintyReason::Ordering));
}

#[test]
fn bezier_boolean_split_plan_audit_certifies_ready_unit_parameters() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection,
        });
    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[]);
    let plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);

    let audit = plan.audit(&policy());

    assert_eq!(
        audit,
        Classification::Decided(hypercurve::BezierBooleanSplitPlanAuditReport2 {
            status: BezierBooleanSplitPlanAuditStatus::Valid,
            checked_parameter_count: 2,
            out_of_range_parameter_count: 0,
        })
    );
}

#[test]
fn bezier_boolean_split_plan_audit_rejects_out_of_range_ready_parameters() {
    let plan = BezierBooleanSplitPlanReport2 {
        status: BezierBooleanSplitPlanStatus::Ready,
        scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
        first_curve_parameters: vec![ratio(-1, 4)],
        second_curve_parameters: vec![half()],
        shared_range_parameters: vec![ratio(5, 4)],
        relation_event_count: 1,
        range_event_count: 1,
        uncertainty_reason: None,
    };

    let audit = plan.audit(&policy());

    assert_eq!(
        audit,
        Classification::Decided(hypercurve::BezierBooleanSplitPlanAuditReport2 {
            status: BezierBooleanSplitPlanAuditStatus::ParameterOutOfRange,
            checked_parameter_count: 3,
            out_of_range_parameter_count: 2,
        })
    );
}

#[test]
fn bezier_boolean_split_plan_audit_keeps_blocked_plans_non_insertable() {
    let relation = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[]);
    let plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);

    let audit = plan.audit(&policy());

    assert_eq!(
        audit,
        Classification::Decided(hypercurve::BezierBooleanSplitPlanAuditReport2 {
            status: BezierBooleanSplitPlanAuditStatus::Blocked,
            checked_parameter_count: 0,
            out_of_range_parameter_count: 0,
        })
    );
}

#[test]
fn bezier_boolean_split_insertion_report_separates_interior_from_endpoints() {
    let plan = BezierBooleanSplitPlanReport2 {
        status: BezierBooleanSplitPlanStatus::Ready,
        scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
        first_curve_parameters: vec![Real::zero(), ratio(1, 4)],
        second_curve_parameters: vec![half(), Real::one()],
        shared_range_parameters: vec![ratio(3, 4)],
        relation_event_count: 2,
        range_event_count: 1,
        uncertainty_reason: None,
    };

    let report = plan.insertion_report(&policy());

    let Classification::Decided(report) = report else {
        panic!("exact split insertion report should be decided");
    };
    assert_eq!(report.status, BezierBooleanSplitInsertionStatus::Ready);
    assert!(report.is_ready());
    assert_eq!(report.first_curve_interior_parameters, vec![ratio(1, 4)]);
    assert_eq!(report.second_curve_interior_parameters, vec![half()]);
    assert_eq!(report.shared_range_interior_parameters, vec![ratio(3, 4)]);
    assert_eq!(report.endpoint_parameter_count, 2);
    assert_eq!(report.interior_parameter_count, 3);
    assert_eq!(report.out_of_range_parameter_count, 0);
}

#[test]
fn bezier_boolean_split_insertion_report_preserves_non_insertion_states() {
    let endpoint_only = BezierBooleanSplitPlanReport2 {
        status: BezierBooleanSplitPlanStatus::Ready,
        scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
        first_curve_parameters: vec![Real::zero()],
        second_curve_parameters: vec![Real::one()],
        shared_range_parameters: Vec::new(),
        relation_event_count: 1,
        range_event_count: 0,
        uncertainty_reason: None,
    };
    let Classification::Decided(report) = endpoint_only.insertion_report(&policy()) else {
        panic!("endpoint-only split insertion report should be decided");
    };
    assert_eq!(
        report.status,
        BezierBooleanSplitInsertionStatus::NoInteriorSplits
    );
    assert_eq!(report.endpoint_parameter_count, 2);
    assert_eq!(report.interior_parameter_count, 0);

    let invalid = BezierBooleanSplitPlanReport2 {
        status: BezierBooleanSplitPlanStatus::Ready,
        scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
        first_curve_parameters: vec![ratio(-1, 4)],
        second_curve_parameters: Vec::new(),
        shared_range_parameters: Vec::new(),
        relation_event_count: 1,
        range_event_count: 0,
        uncertainty_reason: None,
    };
    let Classification::Decided(report) = invalid.insertion_report(&policy()) else {
        panic!("invalid split insertion report should be decided");
    };
    assert_eq!(
        report.status,
        BezierBooleanSplitInsertionStatus::InvalidParameterDomain
    );
    assert!(report.has_blockers());
    assert_eq!(report.out_of_range_parameter_count, 1);
}

#[test]
fn bezier_boolean_construction_readiness_reports_ready_interior_splits() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap();
    let intersection = first.intersect_line(&second, &policy()).unwrap();
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection,
        });

    let readiness =
        BezierBooleanConstructionReadinessReport2::from_reports(&[relation], &[], &policy());

    let Classification::Decided(readiness) = readiness else {
        panic!("ready construction report should be decided");
    };
    assert_eq!(
        readiness.status,
        BezierBooleanConstructionReadinessStatus::Ready
    );
    assert!(readiness.is_ready());
    assert!(!readiness.has_blockers());
    assert_eq!(readiness.insertion.interior_parameter_count, 2);
    assert_eq!(readiness.split_plan_audit.out_of_range_parameter_count, 0);
}

#[test]
fn bezier_boolean_construction_readiness_reports_noop_and_blocked_states() {
    let first = LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap();
    let second = LineSeg2::try_new(point(0, 0), point(0, 4)).unwrap();
    let endpoint_intersection = first.intersect_line(&second, &policy()).unwrap();
    let endpoint_relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::LineSegmentIntersection {
            intersection: endpoint_intersection,
        });
    let readiness = BezierBooleanConstructionReadinessReport2::from_reports(
        &[endpoint_relation],
        &[],
        &policy(),
    )
    .unwrap_decided_for_test();
    assert_eq!(
        readiness.status,
        BezierBooleanConstructionReadinessStatus::NoInteriorSplits
    );
    assert!(!readiness.is_ready());
    assert_eq!(readiness.insertion.endpoint_parameter_count, 2);

    let blocked_relation = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let blocked = BezierBooleanConstructionReadinessReport2::from_reports(
        &[blocked_relation],
        &[],
        &policy(),
    )
    .unwrap_decided_for_test();
    assert_eq!(
        blocked.status,
        BezierBooleanConstructionReadinessStatus::Blocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_quadratic_fragment_report_splits_ready_first_operand() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let region =
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half()));
    let relation =
        BezierBooleanHandoffReport2::from_relation(&BezierCurveRelation::IntersectionRegions {
            regions: vec![region],
        });
    let readiness =
        BezierBooleanConstructionReadinessReport2::from_reports(&[relation], &[], &policy())
            .unwrap_decided_for_test();

    let report = BezierBooleanQuadraticFragmentReport2::from_first_curve_readiness(
        &curve,
        &readiness,
        &policy(),
    )
    .unwrap_decided_for_test();

    assert_eq!(
        report.status,
        BezierBooleanFragmentConstructionStatus::Ready
    );
    assert_eq!(report.inserted_parameters, vec![half()]);
    assert_eq!(report.inserted_parameter_count, 1);
    assert_eq!(report.fragments.len(), 2);
    assert_eq!(report.fragments[0].start(), curve.start());
    assert_eq!(report.fragments[0].end(), &curve.point_at(half()));
    assert_eq!(report.fragments[1].start(), &curve.point_at(half()));
    assert_eq!(report.fragments[1].end(), curve.end());
}

#[test]
fn bezier_boolean_cubic_fragment_report_sorts_and_deduplicates_parameters() {
    let curve = CubicBezier2::new(point(0, 0), point(0, 6), point(6, 6), point(6, 0));
    let parameters = vec![
        ratio(3, 4),
        Real::zero(),
        ratio(1, 4),
        ratio(3, 4),
        Real::one(),
    ];

    let report = BezierBooleanCubicFragmentReport2::from_parameters(&curve, &parameters, &policy())
        .unwrap_decided_for_test();

    assert_eq!(
        report.status,
        BezierBooleanFragmentConstructionStatus::Ready
    );
    assert_eq!(report.source_parameter_count, 5);
    assert_eq!(report.endpoint_parameter_count, 2);
    assert_eq!(report.inserted_parameters, vec![ratio(1, 4), ratio(3, 4)]);
    assert_eq!(report.fragments.len(), 3);
    assert_eq!(report.fragments[0].start(), curve.start());
    assert_eq!(report.fragments[0].end(), &curve.point_at(ratio(1, 4)));
    assert_eq!(report.fragments[1].start(), &curve.point_at(ratio(1, 4)));
    assert_eq!(report.fragments[1].end(), &curve.point_at(ratio(3, 4)));
    assert_eq!(report.fragments[2].start(), &curve.point_at(ratio(3, 4)));
    assert_eq!(report.fragments[2].end(), curve.end());
}

#[test]
fn bezier_boolean_rational_quadratic_fragment_report_splits_homogeneously() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(point(0, 0), point(2, 4), point(4, 0), r(2))
            .unwrap();

    let report = BezierBooleanRationalQuadraticFragmentReport2::from_parameters(
        &curve,
        &[half()],
        &policy(),
    )
    .unwrap_decided_for_test();
    let midpoint = curve.point_at(half(), &policy()).unwrap_decided_for_test();

    assert_eq!(
        report.status,
        BezierBooleanFragmentConstructionStatus::Ready
    );
    assert_eq!(report.inserted_parameters, vec![half()]);
    assert_eq!(report.fragments.len(), 2);
    assert_eq!(report.fragments[0].start(), curve.start());
    assert_eq!(report.fragments[0].end(), &midpoint);
    assert_eq!(report.fragments[1].start(), &midpoint);
    assert_eq!(report.fragments[1].end(), curve.end());
    assert_eq!(
        report.fragments[0].point_at(Real::one(), &policy()),
        Classification::Decided(midpoint.clone())
    );
    assert_eq!(
        report.fragments[1].point_at(Real::zero(), &policy()),
        Classification::Decided(midpoint)
    );
}

#[test]
fn bezier_boolean_arrangement_readiness_accepts_split_fragments_and_overlaps() {
    let first = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let second = QuadraticBezier2::new(point(0, 1), point(2, -3), point(4, 1));
    let first_fragments =
        BezierBooleanQuadraticFragmentReport2::from_parameters(&first, &[half()], &policy())
            .unwrap_decided_for_test();
    let second_fragments =
        BezierBooleanQuadraticFragmentReport2::from_parameters(&second, &[half()], &policy())
            .unwrap_decided_for_test();
    let overlap = hypercurve::BezierBooleanOverlapEvent2 {
        first_range: ParamRange::new(ratio(1, 4), half()),
        second_range: ParamRange::new(half(), ratio(1, 4)),
    };
    let overlaps =
        BezierBooleanOverlapResolutionReport2::from_overlap_events(&[overlap], &policy())
            .unwrap_decided_for_test();

    let report = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
        &first_fragments,
        &second_fragments,
        &overlaps,
    );

    assert_eq!(
        report.status,
        BezierBooleanArrangementReadinessStatus::Ready
    );
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(report.first_fragment_count, 2);
    assert_eq!(report.second_fragment_count, 2);
    assert_eq!(report.resolved_overlap_count, 1);
    assert_eq!(report.overlap_boundary_parameter_count, 4);
}

#[test]
fn bezier_boolean_arrangement_readiness_preserves_noop_and_blockers() {
    let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
        .unwrap_decided_for_test();
    let noop = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        &overlaps,
    );
    assert_eq!(
        noop.status,
        BezierBooleanArrangementReadinessStatus::NoInteriorSplits
    );
    assert!(!noop.is_ready());
    assert!(!noop.has_blockers());

    let blocked_overlap = BezierBooleanOverlapResolutionReport2::from_handoff_reports(
        &[BezierBooleanHandoffReport2::from_relation(
            &BezierCurveRelation::SameCurveImage,
        )],
        &policy(),
    )
    .unwrap_decided_for_test();
    let blocked = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::Ready,
        2,
        BezierBooleanFragmentConstructionStatus::Ready,
        2,
        &blocked_overlap,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanArrangementReadinessStatus::OverlapBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_traversal_preconditions_accept_continuous_split_chains() {
    let first = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let second = QuadraticBezier2::new(point(0, 1), point(2, -3), point(4, 1));
    let first_fragments =
        BezierBooleanQuadraticFragmentReport2::from_parameters(&first, &[half()], &policy())
            .unwrap_decided_for_test();
    let second_fragments =
        BezierBooleanQuadraticFragmentReport2::from_parameters(&second, &[half()], &policy())
            .unwrap_decided_for_test();
    let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
        .unwrap_decided_for_test();
    let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
        &first_fragments,
        &second_fragments,
        &overlaps,
    );

    let report = BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
        &readiness,
        &first_fragments,
        &second_fragments,
    );

    assert_eq!(
        report.status,
        BezierBooleanTraversalPreconditionStatus::Ready
    );
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(report.first_fragment_count, 2);
    assert_eq!(report.second_fragment_count, 2);
    assert_eq!(report.first_chain_gap_count, 0);
    assert_eq!(report.second_chain_gap_count, 0);
}

#[test]
fn bezier_boolean_traversal_preconditions_preserve_noops_and_blockers() {
    let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
        .unwrap_decided_for_test();
    let noop_readiness = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        &overlaps,
    );
    let chain = vec![(point(0, 0), point(1, 0))];
    let noop = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &noop_readiness,
        &chain,
        &chain,
    );
    assert_eq!(
        noop.status,
        BezierBooleanTraversalPreconditionStatus::NoInteriorSplits
    );
    assert!(!noop.is_ready());
    assert!(!noop.has_blockers());

    let blocked_readiness = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::Blocked,
        0,
        BezierBooleanFragmentConstructionStatus::Ready,
        1,
        &overlaps,
    );
    let blocked = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &blocked_readiness,
        &[],
        &chain,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanTraversalPreconditionStatus::ReadinessBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_traversal_preconditions_reject_discontinuous_fragment_chains() {
    let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
        .unwrap_decided_for_test();
    let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::Ready,
        2,
        BezierBooleanFragmentConstructionStatus::Ready,
        1,
        &overlaps,
    );
    let broken_first = vec![(point(0, 0), point(1, 0)), (point(2, 0), point(3, 0))];
    let second = vec![(point(0, 1), point(3, 1))];

    let report = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &readiness,
        &broken_first,
        &second,
    );

    assert_eq!(
        report.status,
        BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
    );
    assert!(report.has_blockers());
    assert_eq!(report.first_chain_gap_count, 1);
}

#[test]
fn bezier_boolean_traversal_schedule_lists_ready_operand_fragments() {
    let preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            2,
            BezierBooleanFragmentConstructionStatus::Ready,
            1,
            &BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
                .unwrap_decided_for_test(),
        ),
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(2, 0))],
        &[(point(0, 1), point(2, 1))],
    );

    let report = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);

    assert_eq!(report.status, BezierBooleanTraversalScheduleStatus::Ready);
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(report.steps.len(), 3);
    assert_eq!(
        report.steps[0].operand,
        BezierBooleanTraversalOperand::First
    );
    assert_eq!(report.steps[0].fragment_index, 0);
    assert_eq!(
        report.steps[1].operand,
        BezierBooleanTraversalOperand::First
    );
    assert_eq!(report.steps[1].fragment_index, 1);
    assert_eq!(
        report.steps[2].operand,
        BezierBooleanTraversalOperand::Second
    );
    assert_eq!(report.steps[2].fragment_index, 0);
}

#[test]
fn bezier_boolean_traversal_schedule_preserves_noops_and_blockers() {
    let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
        .unwrap_decided_for_test();
    let noop_readiness = BezierBooleanArrangementReadinessReport2::from_parts(
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
        1,
        &overlaps,
    );
    let chain = vec![(point(0, 0), point(1, 0))];
    let noop_preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &noop_readiness,
        &chain,
        &chain,
    );
    let noop = BezierBooleanTraversalScheduleReport2::from_preconditions(&noop_preconditions);
    assert_eq!(
        noop.status,
        BezierBooleanTraversalScheduleStatus::NoInteriorSplits
    );
    assert!(noop.steps.is_empty());

    let blocked_preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
        &BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            2,
            BezierBooleanFragmentConstructionStatus::Ready,
            1,
            &overlaps,
        ),
        &[(point(0, 0), point(1, 0)), (point(2, 0), point(3, 0))],
        &chain,
    );
    let blocked = BezierBooleanTraversalScheduleReport2::from_preconditions(&blocked_preconditions);
    assert_eq!(
        blocked.status,
        BezierBooleanTraversalScheduleStatus::PreconditionBlocked
    );
    assert!(blocked.has_blockers());
    assert!(blocked.steps.is_empty());
}

#[test]
fn bezier_boolean_ownership_classification_applies_material_boolean_actions() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };

    let union = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Union,
        &[
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ],
    );
    assert_eq!(
        union.status,
        BezierBooleanOwnershipClassificationStatus::Ready
    );
    assert_eq!(
        union.owned_steps[0].action,
        BooleanFragmentAction::KeepSourceDirection
    );
    assert_eq!(union.owned_steps[1].action, BooleanFragmentAction::Discard);

    let difference = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Difference,
        &[
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ],
    );
    assert_eq!(
        difference.owned_steps[0].action,
        BooleanFragmentAction::KeepSourceDirection
    );
    assert_eq!(
        difference.owned_steps[1].action,
        BooleanFragmentAction::KeepReversed
    );
}

#[test]
fn bezier_boolean_ownership_classification_blocks_missing_and_boundary_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };

    let missing = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Intersection,
        &[BezierBooleanFragmentOwnershipLocation::Inside],
    );
    assert_eq!(
        missing.status,
        BezierBooleanOwnershipClassificationStatus::MissingOwnershipFacts
    );
    assert!(missing.has_blockers());
    assert_eq!(missing.missing_ownership_count, 1);

    let boundary = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Intersection,
        &[
            BezierBooleanFragmentOwnershipLocation::Inside,
            BezierBooleanFragmentOwnershipLocation::Boundary,
        ],
    );
    assert_eq!(
        boundary.status,
        BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution
    );
    assert!(boundary.has_blockers());
    assert_eq!(boundary.boundary_blocker_count, 1);
}

#[test]
fn bezier_boolean_ownership_facts_validate_keyed_schedule_order() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let facts = vec![
        BezierBooleanOwnershipFact2 {
            step: schedule.steps[0].clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
        },
        BezierBooleanOwnershipFact2 {
            step: schedule.steps[1].clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Inside,
        },
    ];

    let report = BezierBooleanOwnershipFactReport2::from_schedule_facts(&schedule, &facts);

    assert_eq!(report.status, BezierBooleanOwnershipFactStatus::Ready);
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(
        report.locations,
        vec![
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ]
    );

    let classified = report.classify(&schedule, BooleanOp::Difference);
    assert_eq!(
        classified.status,
        BezierBooleanOwnershipClassificationStatus::Ready
    );
    assert_eq!(classified.keep_source_count, 1);
    assert_eq!(classified.keep_reversed_count, 1);
}

#[test]
fn bezier_boolean_uniform_ownership_expands_operand_locations_to_keyed_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };

    let uniform = BezierBooleanUniformOwnershipFactReport2::from_schedule_locations(
        &schedule,
        BezierBooleanFragmentOwnershipLocation::Outside,
        BezierBooleanFragmentOwnershipLocation::Inside,
    );

    assert_eq!(
        uniform.status,
        BezierBooleanUniformOwnershipFactStatus::Ready
    );
    assert!(uniform.is_ready());
    assert_eq!(uniform.facts.len(), 3);
    assert_eq!(
        uniform.facts[0].opposite_location,
        BezierBooleanFragmentOwnershipLocation::Outside
    );
    assert_eq!(
        uniform.facts[1].opposite_location,
        BezierBooleanFragmentOwnershipLocation::Outside
    );
    assert_eq!(
        uniform.facts[2].opposite_location,
        BezierBooleanFragmentOwnershipLocation::Inside
    );

    let validated = uniform.validate(&schedule);
    assert_eq!(validated.status, BezierBooleanOwnershipFactStatus::Ready);

    let shortcut = BezierBooleanOwnershipFactReport2::from_uniform_operand_locations(
        &schedule,
        BezierBooleanFragmentOwnershipLocation::Outside,
        BezierBooleanFragmentOwnershipLocation::Inside,
    );
    assert_eq!(shortcut, validated);
}

#[test]
fn bezier_boolean_uniform_ownership_blocks_boundary_operand_locations() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };

    let uniform = BezierBooleanUniformOwnershipFactReport2::from_schedule_locations(
        &schedule,
        BezierBooleanFragmentOwnershipLocation::Boundary,
        BezierBooleanFragmentOwnershipLocation::Outside,
    );

    assert_eq!(
        uniform.status,
        BezierBooleanUniformOwnershipFactStatus::BoundaryNeedsResolution
    );
    assert!(uniform.has_blockers());
    assert_eq!(uniform.boundary_fact_count, 1);
    let validated = uniform.validate(&schedule);
    assert_eq!(
        validated.status,
        BezierBooleanOwnershipFactStatus::BoundaryNeedsResolution
    );
}

#[test]
fn bezier_boolean_ownership_facts_block_missing_extra_mismatch_and_boundary() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let first_fact = BezierBooleanOwnershipFact2 {
        step: schedule.steps[0].clone(),
        opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
    };
    let second_fact = BezierBooleanOwnershipFact2 {
        step: schedule.steps[1].clone(),
        opposite_location: BezierBooleanFragmentOwnershipLocation::Inside,
    };

    let missing = BezierBooleanOwnershipFactReport2::from_schedule_facts(
        &schedule,
        std::slice::from_ref(&first_fact),
    );
    assert_eq!(
        missing.status,
        BezierBooleanOwnershipFactStatus::MissingOwnershipFacts
    );
    assert_eq!(missing.missing_fact_count, 1);
    assert!(missing.has_blockers());

    let extra = BezierBooleanOwnershipFactReport2::from_schedule_facts(
        &schedule,
        &[first_fact.clone(), second_fact.clone(), first_fact.clone()],
    );
    assert_eq!(
        extra.status,
        BezierBooleanOwnershipFactStatus::ExtraOwnershipFacts
    );
    assert_eq!(extra.extra_fact_count, 1);

    let mismatch = BezierBooleanOwnershipFactReport2::from_schedule_facts(
        &schedule,
        &[second_fact.clone(), first_fact.clone()],
    );
    assert_eq!(
        mismatch.status,
        BezierBooleanOwnershipFactStatus::StepMismatch
    );
    assert_eq!(mismatch.step_mismatch_count, 2);

    let boundary = BezierBooleanOwnershipFactReport2::from_schedule_facts(
        &schedule,
        &[
            first_fact,
            BezierBooleanOwnershipFact2 {
                step: schedule.steps[1].clone(),
                opposite_location: BezierBooleanFragmentOwnershipLocation::Boundary,
            },
        ],
    );
    assert_eq!(
        boundary.status,
        BezierBooleanOwnershipFactStatus::BoundaryNeedsResolution
    );
    assert_eq!(boundary.boundary_fact_count, 1);
}

#[test]
fn bezier_boolean_emission_plan_separates_emitted_and_discarded_steps() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Union,
        &[
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ],
    );

    let plan = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);

    assert_eq!(plan.status, BezierBooleanEmissionPlanStatus::Ready);
    assert!(plan.is_ready());
    assert!(!plan.has_blockers());
    assert_eq!(plan.emitted_steps.len(), 1);
    assert_eq!(plan.discarded_steps.len(), 1);
    assert_eq!(
        plan.emitted_steps[0].action,
        BooleanFragmentAction::KeepSourceDirection
    );
    assert_eq!(plan.discard_count, 1);
}

#[test]
fn bezier_boolean_emission_plan_preserves_no_output_and_blockers() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 0,
        steps: vec![hypercurve::BezierBooleanTraversalStep2 {
            operand: BezierBooleanTraversalOperand::First,
            fragment_index: 0,
        }],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let no_output_ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Intersection,
        &[BezierBooleanFragmentOwnershipLocation::Outside],
    );
    let no_output = BezierBooleanEmissionPlanReport2::from_ownership(&no_output_ownership);
    assert_eq!(
        no_output.status,
        BezierBooleanEmissionPlanStatus::NoEmittedFragments
    );
    assert!(!no_output.is_ready());
    assert!(!no_output.has_blockers());
    assert_eq!(no_output.discarded_steps.len(), 1);

    let blocked_ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Union,
        &[BezierBooleanFragmentOwnershipLocation::Boundary],
    );
    let blocked = BezierBooleanEmissionPlanReport2::from_ownership(&blocked_ownership);
    assert_eq!(
        blocked.status,
        BezierBooleanEmissionPlanStatus::OwnershipBlocked
    );
    assert!(blocked.has_blockers());
    assert!(blocked.emitted_steps.is_empty());
}

#[test]
fn bezier_boolean_assembly_readiness_accepts_valid_emitted_references() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Xor,
        &[
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ],
    );
    let plan = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);

    let report = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&plan, 1, 1);

    assert_eq!(report.status, BezierBooleanAssemblyReadinessStatus::Ready);
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(report.emitted_step_count, 2);
    assert_eq!(report.first_emitted_count, 1);
    assert_eq!(report.second_emitted_count, 1);
    assert_eq!(report.invalid_reference_count, 0);
}

#[test]
fn bezier_boolean_assembly_readiness_rejects_invalid_references_and_preserves_blockers() {
    let stale_plan = BezierBooleanEmissionPlanReport2 {
        status: BezierBooleanEmissionPlanStatus::Ready,
        ownership_status: BezierBooleanOwnershipClassificationStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 3,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        discarded_steps: Vec::new(),
        keep_source_count: 1,
        keep_reversed_count: 0,
        discard_count: 0,
        boundary_blocker_count: 0,
        missing_ownership_count: 0,
        blocker_count: 0,
    };
    let invalid = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&stale_plan, 1, 0);
    assert_eq!(
        invalid.status,
        BezierBooleanAssemblyReadinessStatus::InvalidFragmentReference
    );
    assert!(invalid.has_blockers());
    assert_eq!(invalid.invalid_reference_count, 1);

    let blocked_plan = BezierBooleanEmissionPlanReport2 {
        status: BezierBooleanEmissionPlanStatus::OwnershipBlocked,
        ownership_status: BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution,
        operation: BooleanOp::Union,
        emitted_steps: Vec::new(),
        discarded_steps: Vec::new(),
        keep_source_count: 0,
        keep_reversed_count: 0,
        discard_count: 0,
        boundary_blocker_count: 1,
        missing_ownership_count: 0,
        blocker_count: 1,
    };
    let blocked = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&blocked_plan, 1, 1);
    assert_eq!(
        blocked.status,
        BezierBooleanAssemblyReadinessStatus::EmissionBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_loop_assembly_plan_packages_ready_emitted_references() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 1,
        second_fragment_count: 1,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::Second,
                fragment_index: 0,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
        &schedule,
        BooleanOp::Xor,
        &[
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Inside,
        ],
    );
    let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
    let readiness = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&emission, 1, 1);

    let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&readiness, &emission);

    assert_eq!(plan.status, BezierBooleanLoopAssemblyPlanStatus::Ready);
    assert!(plan.is_ready());
    assert!(!plan.has_blockers());
    assert_eq!(plan.emitted_steps.len(), 2);
    assert_eq!(plan.first_emitted_count, 1);
    assert_eq!(plan.second_emitted_count, 1);
}

#[test]
fn bezier_boolean_loop_assembly_plan_preserves_no_output_and_blockers() {
    let no_output_emission = BezierBooleanEmissionPlanReport2 {
        status: BezierBooleanEmissionPlanStatus::NoEmittedFragments,
        ownership_status: BezierBooleanOwnershipClassificationStatus::Ready,
        operation: BooleanOp::Intersection,
        emitted_steps: Vec::new(),
        discarded_steps: Vec::new(),
        keep_source_count: 0,
        keep_reversed_count: 0,
        discard_count: 1,
        boundary_blocker_count: 0,
        missing_ownership_count: 0,
        blocker_count: 0,
    };
    let no_output_ready =
        BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&no_output_emission, 1, 1);
    let no_output = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
        &no_output_ready,
        &no_output_emission,
    );
    assert_eq!(
        no_output.status,
        BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments
    );
    assert!(!no_output.is_ready());
    assert!(!no_output.has_blockers());

    let blocked_emission = BezierBooleanEmissionPlanReport2 {
        status: BezierBooleanEmissionPlanStatus::OwnershipBlocked,
        ownership_status: BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution,
        operation: BooleanOp::Union,
        emitted_steps: Vec::new(),
        discarded_steps: Vec::new(),
        keep_source_count: 0,
        keep_reversed_count: 0,
        discard_count: 0,
        boundary_blocker_count: 1,
        missing_ownership_count: 0,
        blocker_count: 1,
    };
    let blocked_ready =
        BezierBooleanAssemblyReadinessReport2::from_fragment_counts(&blocked_emission, 1, 1);
    let blocked = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
        &blocked_ready,
        &blocked_emission,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_loop_graph_walk_builds_identity_for_linear_traversal() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 1,
        second_emitted_count: 1,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);

    let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);

    assert_eq!(walk.status, BezierBooleanLoopGraphWalkStatus::Ready);
    assert!(walk.is_ready());
    assert!(!walk.has_blockers());
    assert_eq!(walk.walk_indices, vec![0, 1]);
    assert_eq!(walk.ordered_steps, plan.emitted_steps);
}

#[test]
fn bezier_boolean_loop_graph_walk_identity_preserves_traversal_blockers() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 1, 0);

    let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);

    assert_eq!(
        walk.status,
        BezierBooleanLoopGraphWalkStatus::TraversalBlocked
    );
    assert!(walk.has_blockers());
    assert!(walk.walk_indices.is_empty());
    assert!(walk.ordered_steps.is_empty());
}

#[test]
fn bezier_boolean_loop_closure_audits_exact_closed_and_open_chains() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closed = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
        &[],
    );
    assert_eq!(closed.status, BezierBooleanLoopClosureStatus::Closed);
    assert!(closed.is_closed());
    assert!(!closed.has_blockers());
    assert_eq!(closed.closed_loop_count, 1);
    assert_eq!(closed.open_chain_count, 0);
    assert_eq!(closed.adjacency_gap_count, 0);

    let open = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(1, 0)), (point(2, 0), point(0, 0))],
        &[],
    );
    assert_eq!(open.status, BezierBooleanLoopClosureStatus::OpenChains);
    assert!(!open.is_closed());
    assert!(open.has_blockers());
    assert_eq!(open.closed_loop_count, 0);
    assert_eq!(open.open_chain_count, 2);
    assert_eq!(open.adjacency_gap_count, 1);
}

#[test]
fn bezier_boolean_loop_closure_applies_reversed_emission_direction() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Difference,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Inside,
                action: BooleanFragmentAction::KeepReversed,
            },
        ],
        first_emitted_count: 1,
        second_emitted_count: 1,
        keep_source_count: 1,
        keep_reversed_count: 1,
        invalid_reference_count: 0,
        blocker_count: 0,
    };

    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(1, 0))],
        &[(point(0, 0), point(1, 0))],
    );

    assert_eq!(closure.status, BezierBooleanLoopClosureStatus::Closed);
    assert_eq!(closure.directed_fragments[1].start, point(1, 0));
    assert_eq!(closure.directed_fragments[1].end, point(0, 0));
    assert_eq!(closure.closed_loop_count, 1);
}

#[test]
fn bezier_boolean_loop_graph_traversal_blocks_branches_and_resolved_overlaps() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };

    let ready = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
    assert_eq!(ready.status, BezierBooleanLoopGraphTraversalStatus::Ready);
    assert!(ready.is_ready());
    assert!(!ready.has_blockers());
    assert_eq!(ready.emitted_step_count, 1);

    let branch = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 2, 0);
    assert_eq!(
        branch.status,
        BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
    );
    assert!(branch.has_blockers());
    assert_eq!(branch.branch_vertex_count, 2);
    assert_eq!(branch.blocker_count, 2);

    let overlap = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 1);
    assert_eq!(
        overlap.status,
        BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal
    );
    assert!(overlap.has_blockers());
    assert_eq!(overlap.resolved_overlap_count, 1);
}

#[test]
fn bezier_boolean_loop_graph_facts_validate_plan_key_and_blockers() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };

    let ready = BezierBooleanLoopGraphFactReport2::from_plan_facts(
        &plan,
        &BezierBooleanLoopGraphFacts2 {
            emitted_step_count: 1,
            branch_vertex_count: 0,
            resolved_overlap_count: 0,
        },
    );
    assert_eq!(ready.status, BezierBooleanLoopGraphFactStatus::Ready);
    assert!(ready.is_ready());
    assert!(!ready.has_blockers());

    let stale = BezierBooleanLoopGraphFactReport2::from_plan_facts(
        &plan,
        &BezierBooleanLoopGraphFacts2 {
            emitted_step_count: 2,
            branch_vertex_count: 0,
            resolved_overlap_count: 0,
        },
    );
    assert_eq!(
        stale.status,
        BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch
    );
    assert!(stale.has_blockers());

    let branch_blocked = BezierBooleanLoopGraphFactReport2::from_plan_facts(
        &plan,
        &BezierBooleanLoopGraphFacts2 {
            emitted_step_count: 1,
            branch_vertex_count: 1,
            resolved_overlap_count: 0,
        },
    );
    assert_eq!(
        branch_blocked.status,
        BezierBooleanLoopGraphFactStatus::BranchPointsNeedTraversal
    );
    assert!(branch_blocked.has_blockers());
}

#[test]
fn bezier_boolean_loop_graph_walk_validates_permutation_and_reorders_plan() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);

    let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[1, 0]);

    assert_eq!(walk.status, BezierBooleanLoopGraphWalkStatus::Ready);
    assert!(walk.is_ready());
    assert!(!walk.has_blockers());
    assert_eq!(walk.walk_indices, vec![1, 0]);
    assert_eq!(walk.ordered_steps[0].step.fragment_index, 1);
    assert_eq!(walk.ordered_steps[1].step.fragment_index, 0);

    let closure = BezierBooleanLoopClosureReport2::from_graph_walk_endpoints(
        &walk,
        &plan,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
    );
    assert_eq!(closure.status, BezierBooleanLoopClosureStatus::Closed);
    assert_eq!(closure.closed_loop_count, 2);
}

#[test]
fn bezier_boolean_loop_graph_walk_blocks_missing_extra_duplicate_and_stale_indices() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);

    let missing = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0]);
    assert_eq!(
        missing.status,
        BezierBooleanLoopGraphWalkStatus::MissingWalkSteps
    );
    assert_eq!(missing.missing_walk_step_count, 1);
    assert!(missing.has_blockers());

    let extra =
        BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 1, 0]);
    assert_eq!(
        extra.status,
        BezierBooleanLoopGraphWalkStatus::ExtraWalkSteps
    );
    assert_eq!(extra.extra_walk_step_count, 1);

    let stale = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 2]);
    assert_eq!(
        stale.status,
        BezierBooleanLoopGraphWalkStatus::OutOfRangeWalkStep
    );
    assert_eq!(stale.out_of_range_walk_step_count, 1);

    let duplicate =
        BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 0]);
    assert_eq!(
        duplicate.status,
        BezierBooleanLoopGraphWalkStatus::DuplicateWalkStep
    );
    assert_eq!(duplicate.duplicate_walk_step_count, 1);

    let blocked_closure =
        BezierBooleanLoopClosureReport2::from_graph_walk_endpoints(&duplicate, &plan, &[], &[]);
    assert_eq!(
        blocked_closure.status,
        BezierBooleanLoopClosureStatus::PlanBlocked
    );
    assert_eq!(blocked_closure.blocker_count, 1);
    assert!(blocked_closure.has_blockers());
}

#[test]
fn bezier_boolean_loop_closure_preserves_plan_blockers_and_invalid_references() {
    let blocked_plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked,
        assembly_status: BezierBooleanAssemblyReadinessStatus::InvalidFragmentReference,
        operation: BooleanOp::Union,
        emitted_steps: Vec::new(),
        first_emitted_count: 0,
        second_emitted_count: 0,
        keep_source_count: 0,
        keep_reversed_count: 0,
        invalid_reference_count: 1,
        blocker_count: 1,
    };
    let blocked = BezierBooleanLoopClosureReport2::from_fragment_endpoints(&blocked_plan, &[], &[]);
    assert_eq!(blocked.status, BezierBooleanLoopClosureStatus::PlanBlocked);
    assert!(blocked.has_blockers());
    assert_eq!(blocked.invalid_reference_count, 1);

    let stale_plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 2,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let stale = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &stale_plan,
        &[(point(0, 0), point(1, 0))],
        &[],
    );
    assert_eq!(
        stale.status,
        BezierBooleanLoopClosureStatus::InvalidFragmentReference
    );
    assert!(stale.has_blockers());
    assert_eq!(stale.invalid_reference_count, 1);
}

#[test]
fn bezier_boolean_output_loop_report_packages_closed_loop_ranges() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 1,
        keep_source_count: 3,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
        &[(point(3, 0), point(3, 0))],
    );

    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    assert_eq!(output.status, BezierBooleanOutputLoopStatus::Ready);
    assert!(output.is_ready());
    assert!(!output.has_blockers());
    assert_eq!(output.loops.len(), 2);
    assert_eq!(output.loops[0].first_directed_fragment_index, 0);
    assert_eq!(output.loops[0].directed_fragment_count, 2);
    assert_eq!(output.loops[0].anchor, point(0, 0));
    assert_eq!(output.loops[1].first_directed_fragment_index, 2);
    assert_eq!(output.loops[1].directed_fragment_count, 1);
    assert_eq!(output.loops[1].anchor, point(3, 0));
}

#[test]
fn bezier_boolean_output_loop_report_consumes_graph_walk_closure() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
    let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[1, 0]);

    let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
        &walk,
        &plan,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
    );

    assert_eq!(output.status, BezierBooleanOutputLoopStatus::Ready);
    assert!(output.is_ready());
    assert_eq!(output.loops.len(), 2);
    assert_eq!(output.loops[0].anchor, point(1, 0));
    assert_eq!(output.loops[1].anchor, point(0, 0));

    let duplicate =
        BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 0]);
    let blocked =
        BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(&duplicate, &plan, &[], &[]);
    assert_eq!(
        blocked.status,
        BezierBooleanOutputLoopStatus::ClosureBlocked
    );
    assert!(blocked.has_blockers());
    assert_eq!(blocked.blocker_count, 1);
}

#[test]
fn bezier_boolean_region_assembly_consumes_graph_walk_output_and_depth_facts() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
    let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 1]);
    let depth_facts = vec![
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 0,
            nesting_depth: 0,
        },
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 1,
            nesting_depth: 1,
        },
    ];

    let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_depth_facts(
        &walk,
        &plan,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        &depth_facts,
    );

    assert_eq!(assembly.status, BezierBooleanRegionAssemblyStatus::Ready);
    assert!(assembly.is_ready());
    assert_eq!(assembly.assigned_loop_count, 2);
    assert_eq!(assembly.material_loop_indices, vec![0]);
    assert_eq!(assembly.hole_loop_indices, vec![1]);

    let duplicate =
        BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 0]);
    let blocked = BezierBooleanRegionAssemblyReport2::from_graph_walk_depth_facts(
        &duplicate,
        &plan,
        &[],
        &[],
        &depth_facts,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_graph_walk_output_and_depth_facts() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 2,
        second_emitted_count: 0,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
    let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(&traversal, &plan, &[0, 1]);
    let depth_facts = vec![
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 0,
            nesting_depth: 0,
        },
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 1,
            nesting_depth: 1,
        },
    ];

    let result = BezierBooleanResultReport2::from_graph_walk_depth_facts(
        &walk,
        &plan,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        &depth_facts,
    );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.assigned_loop_count, 2);
    assert_eq!(result.directed_fragment_count, 2);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert_eq!(result.hole_loop_indices, vec![1]);

    let stale_depths = vec![BezierBooleanLoopNestingDepthFact2 {
        loop_index: 2,
        nesting_depth: 0,
    }];
    let blocked = BezierBooleanResultReport2::from_graph_walk_depth_facts(
        &walk,
        &plan,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        &stale_depths,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_schedule_ownership_walk_and_depth_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership_facts = schedule
        .steps
        .iter()
        .map(|step| BezierBooleanOwnershipFact2 {
            step: step.clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
        })
        .collect::<Vec<_>>();
    let depth_facts = vec![
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 0,
            nesting_depth: 0,
        },
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 1,
            nesting_depth: 1,
        },
    ];

    let result = BezierBooleanResultReport2::from_schedule_graph_walk_depth_facts(
        &schedule,
        BooleanOp::Union,
        &ownership_facts,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        0,
        0,
        &[0, 1],
        &depth_facts,
    );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.directed_fragment_count, 2);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert_eq!(result.hole_loop_indices, vec![1]);

    let boundary_facts = vec![
        ownership_facts[0].clone(),
        BezierBooleanOwnershipFact2 {
            step: schedule.steps[1].clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Boundary,
        },
    ];
    let blocked = BezierBooleanResultReport2::from_schedule_graph_walk_depth_facts(
        &schedule,
        BooleanOp::Union,
        &boundary_facts,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        0,
        0,
        &[0, 1],
        &depth_facts,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_uniform_identity_containment_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let graph_facts = BezierBooleanLoopGraphFacts2 {
        emitted_step_count: 2,
        branch_vertex_count: 0,
        resolved_overlap_count: 0,
    };

    let result =
        BezierBooleanResultReport2::from_schedule_uniform_graph_fact_identity_containment_facts(
            &schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &graph_facts,
            &[],
        );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.directed_fragment_count, 2);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert!(result.hole_loop_indices.is_empty());

    let stale_graph = BezierBooleanLoopGraphFacts2 {
        emitted_step_count: 1,
        branch_vertex_count: 0,
        resolved_overlap_count: 0,
    };
    let blocked =
        BezierBooleanResultReport2::from_schedule_uniform_graph_fact_identity_containment_facts(
            &schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &stale_graph,
            &[],
        );
    assert_eq!(
        blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_uniform_linear_identity_containment_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };

    let result =
        BezierBooleanResultReport2::from_schedule_uniform_linear_identity_containment_facts(
            &schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &[],
        );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.directed_fragment_count, 2);
    assert_eq!(result.material_loop_indices, vec![0]);

    let overlap_blocked_schedule = BezierBooleanTraversalScheduleReport2 {
        resolved_overlap_count: 1,
        overlap_boundary_parameter_count: 2,
        ..schedule
    };
    let blocked =
        BezierBooleanResultReport2::from_schedule_uniform_linear_identity_containment_facts(
            &overlap_blocked_schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &[],
        );
    assert_eq!(
        blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_uniform_linear_identity_depth_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let depth_facts = [BezierBooleanLoopNestingDepthFact2 {
        loop_index: 0,
        nesting_depth: 0,
    }];

    let result = BezierBooleanResultReport2::from_schedule_uniform_linear_identity_depth_facts(
        &schedule,
        BooleanOp::Union,
        BezierBooleanFragmentOwnershipLocation::Outside,
        BezierBooleanFragmentOwnershipLocation::Outside,
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
        &[],
        &depth_facts,
    );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.directed_fragment_count, 2);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert!(result.hole_loop_indices.is_empty());

    let missing_depth =
        BezierBooleanResultReport2::from_schedule_uniform_linear_identity_depth_facts(
            &schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &[],
        );
    assert_eq!(
        missing_depth.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(missing_depth.has_blockers());

    let overlap_blocked_schedule = BezierBooleanTraversalScheduleReport2 {
        resolved_overlap_count: 1,
        overlap_boundary_parameter_count: 2,
        ..schedule
    };
    let overlap_blocked =
        BezierBooleanResultReport2::from_schedule_uniform_linear_identity_depth_facts(
            &overlap_blocked_schedule,
            BooleanOp::Union,
            BezierBooleanFragmentOwnershipLocation::Outside,
            BezierBooleanFragmentOwnershipLocation::Outside,
            &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
            &[],
            &depth_facts,
        );
    assert_eq!(
        overlap_blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(overlap_blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_keyed_graph_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership_facts = schedule
        .steps
        .iter()
        .map(|step| BezierBooleanOwnershipFact2 {
            step: step.clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
        })
        .collect::<Vec<_>>();
    let depth_facts = vec![BezierBooleanLoopNestingDepthFact2 {
        loop_index: 0,
        nesting_depth: 0,
    }];
    let graph_facts = BezierBooleanLoopGraphFacts2 {
        emitted_step_count: 2,
        branch_vertex_count: 0,
        resolved_overlap_count: 0,
    };

    let result = BezierBooleanResultReport2::from_schedule_graph_fact_walk_depth_facts(
        &schedule,
        BooleanOp::Union,
        &ownership_facts,
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
        &[],
        &graph_facts,
        &[0, 1],
        &depth_facts,
    );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert_eq!(result.assigned_loop_count, 1);
    assert_eq!(result.directed_fragment_count, 2);

    let stale_graph_facts = BezierBooleanLoopGraphFacts2 {
        emitted_step_count: 1,
        branch_vertex_count: 0,
        resolved_overlap_count: 0,
    };
    let blocked = BezierBooleanResultReport2::from_schedule_graph_fact_walk_depth_facts(
        &schedule,
        BooleanOp::Union,
        &ownership_facts,
        &[(point(0, 0), point(1, 0)), (point(1, 0), point(0, 0))],
        &[],
        &stale_graph_facts,
        &[0, 1],
        &depth_facts,
    );
    assert_eq!(
        blocked.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_consumes_schedule_graph_walk_containment_facts() {
    let schedule = BezierBooleanTraversalScheduleReport2 {
        status: BezierBooleanTraversalScheduleStatus::Ready,
        precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
        first_fragment_count: 2,
        second_fragment_count: 0,
        steps: vec![
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 1,
            },
        ],
        resolved_overlap_count: 0,
        overlap_boundary_parameter_count: 0,
        blocker_count: 0,
    };
    let ownership_facts = schedule
        .steps
        .iter()
        .map(|step| BezierBooleanOwnershipFact2 {
            step: step.clone(),
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
        })
        .collect::<Vec<_>>();
    let containment_facts = vec![BezierBooleanLoopContainmentFact2 {
        container_loop_index: 0,
        contained_loop_index: 1,
    }];

    let result = BezierBooleanResultReport2::from_schedule_graph_walk_containment_facts(
        &schedule,
        BooleanOp::Union,
        &ownership_facts,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        0,
        0,
        &[0, 1],
        &containment_facts,
    );

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert_eq!(result.hole_loop_indices, vec![1]);

    let stale = BezierBooleanResultReport2::from_schedule_graph_fact_walk_containment_facts(
        &schedule,
        BooleanOp::Union,
        &ownership_facts,
        &[(point(0, 0), point(0, 0)), (point(1, 0), point(1, 0))],
        &[],
        &BezierBooleanLoopGraphFacts2 {
            emitted_step_count: 1,
            branch_vertex_count: 0,
            resolved_overlap_count: 0,
        },
        &[0, 1],
        &containment_facts,
    );
    assert_eq!(
        stale.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(stale.has_blockers());
}

#[test]
fn bezier_boolean_output_loop_report_preserves_closure_blockers() {
    let blocked_plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &blocked_plan,
        &[(point(0, 0), point(1, 0))],
        &[],
    );

    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    assert_eq!(output.status, BezierBooleanOutputLoopStatus::ClosureBlocked);
    assert!(!output.is_ready());
    assert!(output.has_blockers());
    assert_eq!(output.open_chain_count, 1);
    assert_eq!(output.blocker_count, 1);
}

#[test]
fn bezier_boolean_loop_nesting_roles_generate_material_hole_parity_roles() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 1,
        second_emitted_count: 1,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[(point(2, 0), point(2, 0))],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    let generated = BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(&output, &[0, 1]);

    assert_eq!(generated.status, BezierBooleanLoopNestingRoleStatus::Ready);
    assert!(generated.is_ready());
    assert!(!generated.has_blockers());
    assert_eq!(
        generated.roles,
        vec![
            BezierBooleanOutputLoopRole::Material,
            BezierBooleanOutputLoopRole::Hole
        ]
    );
    assert_eq!(generated.material_loop_count, 1);
    assert_eq!(generated.hole_loop_count, 1);

    let assigned =
        BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &generated.roles);
    assert_eq!(
        assigned.status,
        BezierBooleanLoopRoleAssignmentStatus::Ready
    );
}

#[test]
fn bezier_boolean_loop_nesting_roles_block_missing_and_extra_depths() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    let missing = BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(&output, &[]);
    assert_eq!(
        missing.status,
        BezierBooleanLoopNestingRoleStatus::MissingNestingDepthFacts
    );
    assert!(missing.has_blockers());
    assert_eq!(missing.missing_depth_count, 1);

    let extra = BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(&output, &[0, 1]);
    assert_eq!(
        extra.status,
        BezierBooleanLoopNestingRoleStatus::ExtraNestingDepthFacts
    );
    assert!(extra.has_blockers());
    assert_eq!(extra.extra_depth_count, 1);
}

fn ready_two_loop_output_report() -> BezierBooleanOutputLoopReport2 {
    BezierBooleanOutputLoopReport2 {
        status: BezierBooleanOutputLoopStatus::Ready,
        closure_status: BezierBooleanLoopClosureStatus::Closed,
        operation: BooleanOp::Union,
        directed_fragments: Vec::new(),
        loops: vec![
            hypercurve::BezierBooleanOutputLoop2 {
                first_directed_fragment_index: 0,
                directed_fragment_count: 2,
                anchor: point(0, 0),
            },
            hypercurve::BezierBooleanOutputLoop2 {
                first_directed_fragment_index: 2,
                directed_fragment_count: 2,
                anchor: point(2, 0),
            },
        ],
        closed_loop_count: 2,
        directed_fragment_count: 4,
        open_chain_count: 0,
        adjacency_gap_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    }
}

#[test]
fn bezier_boolean_loop_nesting_depth_facts_validate_keyed_loop_order() {
    let output = ready_two_loop_output_report();
    let facts = vec![
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 0,
            nesting_depth: 0,
        },
        BezierBooleanLoopNestingDepthFact2 {
            loop_index: 1,
            nesting_depth: 1,
        },
    ];

    let report = BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(&output, &facts);

    assert_eq!(
        report.status,
        BezierBooleanLoopNestingDepthFactStatus::Ready
    );
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(report.depths, vec![0, 1]);

    let roles = report.generate_roles(&output);
    assert_eq!(roles.status, BezierBooleanLoopNestingRoleStatus::Ready);
    assert_eq!(
        roles.roles,
        vec![
            BezierBooleanOutputLoopRole::Material,
            BezierBooleanOutputLoopRole::Hole,
        ]
    );
}

#[test]
fn bezier_boolean_loop_containment_facts_derive_keyed_depths() {
    let output = ready_two_loop_output_report();
    let facts = vec![BezierBooleanLoopContainmentFact2 {
        container_loop_index: 0,
        contained_loop_index: 1,
    }];

    let report = BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
        &output, &facts,
    );

    assert_eq!(report.status, BezierBooleanLoopContainmentFactStatus::Ready);
    assert!(report.is_ready());
    assert!(!report.has_blockers());
    assert_eq!(
        report.depth_facts,
        vec![
            BezierBooleanLoopNestingDepthFact2 {
                loop_index: 0,
                nesting_depth: 0,
            },
            BezierBooleanLoopNestingDepthFact2 {
                loop_index: 1,
                nesting_depth: 1,
            },
        ]
    );

    let result = BezierBooleanResultReport2::from_output_loop_containment_facts(&output, &facts);
    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert_eq!(result.material_loop_indices, vec![0]);
    assert_eq!(result.hole_loop_indices, vec![1]);
}

#[test]
fn bezier_boolean_loop_containment_facts_block_stale_self_and_duplicate_pairs() {
    let output = ready_two_loop_output_report();

    let stale = BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
        &output,
        &[BezierBooleanLoopContainmentFact2 {
            container_loop_index: 0,
            contained_loop_index: 2,
        }],
    );
    assert_eq!(
        stale.status,
        BezierBooleanLoopContainmentFactStatus::OutOfRangeLoopIndex
    );
    assert!(stale.has_blockers());

    let self_containment =
        BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
            &output,
            &[BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index: 0,
            }],
        );
    assert_eq!(
        self_containment.status,
        BezierBooleanLoopContainmentFactStatus::SelfContainment
    );

    let duplicate = BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
        &output,
        &[
            BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index: 1,
            },
            BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index: 1,
            },
        ],
    );
    assert_eq!(
        duplicate.status,
        BezierBooleanLoopContainmentFactStatus::DuplicateContainmentFact
    );
}

#[test]
fn bezier_boolean_loop_nesting_depth_facts_block_missing_extra_and_mismatch() {
    let output = ready_two_loop_output_report();
    let first = BezierBooleanLoopNestingDepthFact2 {
        loop_index: 0,
        nesting_depth: 0,
    };
    let second = BezierBooleanLoopNestingDepthFact2 {
        loop_index: 1,
        nesting_depth: 1,
    };

    let missing = BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(
        &output,
        std::slice::from_ref(&first),
    );
    assert_eq!(
        missing.status,
        BezierBooleanLoopNestingDepthFactStatus::MissingNestingDepthFacts
    );
    assert_eq!(missing.missing_fact_count, 1);
    assert!(missing.has_blockers());

    let extra = BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(
        &output,
        &[first.clone(), second.clone(), first.clone()],
    );
    assert_eq!(
        extra.status,
        BezierBooleanLoopNestingDepthFactStatus::ExtraNestingDepthFacts
    );
    assert_eq!(extra.extra_fact_count, 1);

    let mismatch =
        BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(&output, &[second, first]);
    assert_eq!(
        mismatch.status,
        BezierBooleanLoopNestingDepthFactStatus::LoopIndexMismatch
    );
    assert_eq!(mismatch.loop_index_mismatch_count, 2);
}

#[test]
fn bezier_boolean_loop_role_assignment_accepts_certified_material_and_hole_roles() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 1,
        second_emitted_count: 1,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[(point(2, 0), point(2, 0))],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    let assigned = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[
            BezierBooleanOutputLoopRole::Material,
            BezierBooleanOutputLoopRole::Hole,
        ],
    );

    assert_eq!(
        assigned.status,
        BezierBooleanLoopRoleAssignmentStatus::Ready
    );
    assert!(assigned.is_ready());
    assert!(!assigned.has_blockers());
    assert_eq!(assigned.assigned_loops.len(), 2);
    assert_eq!(assigned.material_loop_count, 1);
    assert_eq!(assigned.hole_loop_count, 1);
    assert_eq!(assigned.unknown_role_count, 0);
}

#[test]
fn bezier_boolean_loop_role_assignment_blocks_missing_extra_and_unknown_roles() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

    let missing = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &[]);
    assert_eq!(
        missing.status,
        BezierBooleanLoopRoleAssignmentStatus::MissingRoleFacts
    );
    assert!(missing.has_blockers());
    assert_eq!(missing.missing_role_count, 1);

    let extra = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[
            BezierBooleanOutputLoopRole::Material,
            BezierBooleanOutputLoopRole::Hole,
        ],
    );
    assert_eq!(
        extra.status,
        BezierBooleanLoopRoleAssignmentStatus::ExtraRoleFacts
    );
    assert!(extra.has_blockers());
    assert_eq!(extra.extra_role_count, 1);

    let unknown = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[BezierBooleanOutputLoopRole::Unknown],
    );
    assert_eq!(
        unknown.status,
        BezierBooleanLoopRoleAssignmentStatus::UnknownRole
    );
    assert!(unknown.has_blockers());
    assert_eq!(unknown.unknown_role_count, 1);
}

#[test]
fn bezier_boolean_region_assembly_packages_material_and_hole_indices() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
            hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::Second,
                    fragment_index: 0,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            },
        ],
        first_emitted_count: 1,
        second_emitted_count: 1,
        keep_source_count: 2,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[(point(2, 0), point(2, 0))],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let roles = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[
            BezierBooleanOutputLoopRole::Material,
            BezierBooleanOutputLoopRole::Hole,
        ],
    );

    let assembly = BezierBooleanRegionAssemblyReport2::from_role_assignment(&roles);

    assert_eq!(assembly.status, BezierBooleanRegionAssemblyStatus::Ready);
    assert!(assembly.is_ready());
    assert!(!assembly.has_blockers());
    assert_eq!(assembly.material_loop_indices, vec![0]);
    assert_eq!(assembly.hole_loop_indices, vec![1]);
    assert_eq!(assembly.assigned_loop_count, 2);
}

#[test]
fn bezier_boolean_region_assembly_blocks_holes_without_material_and_role_failures() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let hole_only = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[BezierBooleanOutputLoopRole::Hole],
    );

    let hole_blocked = BezierBooleanRegionAssemblyReport2::from_role_assignment(&hole_only);

    assert_eq!(
        hole_blocked.status,
        BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial
    );
    assert!(hole_blocked.has_blockers());
    assert_eq!(hole_blocked.hole_loop_indices, vec![0]);

    let missing_roles = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &[]);
    let role_blocked = BezierBooleanRegionAssemblyReport2::from_role_assignment(&missing_roles);
    assert_eq!(
        role_blocked.status,
        BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked
    );
    assert!(role_blocked.has_blockers());
}

#[test]
fn bezier_boolean_result_report_accepts_ready_higher_order_artifact() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let roles = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[BezierBooleanOutputLoopRole::Material],
    );
    let assembly = BezierBooleanRegionAssemblyReport2::from_role_assignment(&roles);

    let result = BezierBooleanResultReport2::from_region_assembly(&assembly);

    assert_eq!(result.status, BezierBooleanResultStatus::Ready);
    assert!(result.is_ready());
    assert!(!result.has_blockers());
    assert_eq!(result.assigned_loop_count, 1);
    assert_eq!(result.material_loop_count, 1);
    assert_eq!(result.hole_loop_count, 0);
    assert_eq!(result.directed_fragment_count, 1);
}

#[test]
fn bezier_boolean_result_report_preserves_region_assembly_blockers() {
    let plan = BezierBooleanLoopAssemblyPlanReport2 {
        status: BezierBooleanLoopAssemblyPlanStatus::Ready,
        assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
        operation: BooleanOp::Union,
        emitted_steps: vec![hypercurve::BezierBooleanOwnedTraversalStep2 {
            step: hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: 0,
            },
            opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            action: BooleanFragmentAction::KeepSourceDirection,
        }],
        first_emitted_count: 1,
        second_emitted_count: 0,
        keep_source_count: 1,
        keep_reversed_count: 0,
        invalid_reference_count: 0,
        blocker_count: 0,
    };
    let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
        &plan,
        &[(point(0, 0), point(0, 0))],
        &[],
    );
    let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
    let roles = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
        &output,
        &[BezierBooleanOutputLoopRole::Hole],
    );
    let assembly = BezierBooleanRegionAssemblyReport2::from_role_assignment(&roles);

    let result = BezierBooleanResultReport2::from_region_assembly(&assembly);

    assert_eq!(
        result.status,
        BezierBooleanResultStatus::RegionAssemblyBlocked
    );
    assert!(result.has_blockers());
    assert_eq!(result.blocker_count, 1);
    assert!(result.assigned_loops.is_empty());
}

#[test]
fn bezier_boolean_fragment_reports_preserve_blocked_and_invalid_states() {
    let curve = QuadraticBezier2::new(point(0, 0), point(2, 4), point(4, 0));
    let blocked_relation = BezierBooleanHandoffReport2::from_classified_relation(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let blocked_readiness = BezierBooleanConstructionReadinessReport2::from_reports(
        &[blocked_relation],
        &[],
        &policy(),
    )
    .unwrap_decided_for_test();

    let blocked = BezierBooleanQuadraticFragmentReport2::from_first_curve_readiness(
        &curve,
        &blocked_readiness,
        &policy(),
    )
    .unwrap_decided_for_test();
    assert_eq!(
        blocked.status,
        BezierBooleanFragmentConstructionStatus::Blocked
    );
    assert!(blocked.fragments.is_empty());

    let invalid =
        BezierBooleanQuadraticFragmentReport2::from_parameters(&curve, &[ratio(5, 4)], &policy())
            .unwrap_decided_for_test();
    assert_eq!(
        invalid.status,
        BezierBooleanFragmentConstructionStatus::InvalidParameterDomain
    );
    assert_eq!(invalid.out_of_range_parameter_count, 1);
    assert!(invalid.fragments.is_empty());
}

#[test]
fn bezier_path_range_order_report_classifies_strict_order_and_overlap() {
    let less = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::FirstLess,
    );
    assert_eq!(less.status, BezierPathRangeOrderStatus::FirstBeforeSecond);
    assert!(less.is_ordered());
    assert!(!less.has_blockers());

    let overlap = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::Coincident,
    );
    assert_eq!(overlap.status, BezierPathRangeOrderStatus::Overlap);
    assert!(overlap.has_blockers());
}

#[test]
fn bezier_path_range_order_report_preserves_crossing_and_tangent_contacts() {
    let crossing = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
            contacts: vec![BezierGraphContact::new(
                half(),
                BezierLineContactKind::Crossing,
            )],
            spans: Vec::new(),
        },
    );
    assert_eq!(crossing.status, BezierPathRangeOrderStatus::CrossingContact);
    assert_eq!(crossing.contacts.len(), 1);
    assert!(!crossing.has_blockers());

    let tangent = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
            contacts: vec![BezierGraphContact::new(
                half(),
                BezierLineContactKind::Tangent,
            )],
            spans: Vec::new(),
        },
    );
    assert_eq!(tangent.status, BezierPathRangeOrderStatus::TangentContact);
    assert_eq!(tangent.contacts.len(), 1);
    assert!(!tangent.has_blockers());
}

#[test]
fn bezier_path_range_order_report_keeps_unrepresented_roots_ambiguous() {
    let report = BezierPathRangeOrderReport2::from_graph_order(
        &BezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters: vec![half()],
            spans: vec![span(ratio(1, 4), ratio(1, 2))],
        },
    );

    assert_eq!(report.status, BezierPathRangeOrderStatus::Ambiguous);
    assert!(report.has_blockers());
    assert_eq!(report.unclassified_parameters, vec![half()]);
    assert_eq!(report.isolating_spans.len(), 1);
}

#[test]
fn bezier_path_range_order_report_preserves_classified_uncertainty() {
    let report = BezierPathRangeOrderReport2::from_classified_graph_contact_order(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );

    assert_eq!(report.status, BezierPathRangeOrderStatus::Uncertain);
    assert!(report.has_blockers());
    assert_eq!(report.uncertainty_reason, Some(UncertaintyReason::Ordering));
}

#[test]
fn bezier_path_range_batch_report_distinguishes_order_split_and_blockers() {
    let ordered = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::FirstLess,
    );
    let crossing = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
            contacts: vec![BezierGraphContact::new(
                half(),
                BezierLineContactKind::Crossing,
            )],
            spans: Vec::new(),
        },
    );

    let split_ready = BezierPathRangeBatchReport2::from_range_reports(&[ordered.clone(), crossing]);
    assert_eq!(
        split_ready.status,
        BezierPathRangeBatchStatus::SplitEventsReady
    );
    assert!(split_ready.can_feed_split_events());
    assert!(!split_ready.has_blockers());
    assert_eq!(split_ready.ordered_range_count, 1);
    assert_eq!(split_ready.crossing_contact_count, 1);
    assert_eq!(split_ready.split_parameters, vec![half()]);

    let overlap = BezierPathRangeOrderReport2::from_graph_contact_order(
        &BezierMonotoneGraphContactOrder::Coincident,
    );
    let blocked = BezierPathRangeBatchReport2::from_range_reports(&[ordered, overlap]);
    assert_eq!(
        blocked.status,
        BezierPathRangeBatchStatus::NeedsOverlapResolver
    );
    assert!(blocked.has_blockers());
    assert_eq!(blocked.overlap_range_count, 1);
}

#[test]
fn bezier_path_range_batch_report_prioritizes_uncertainty_and_isolation() {
    let ambiguous = BezierPathRangeOrderReport2::from_graph_order(
        &BezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters: vec![half()],
            spans: vec![span(ratio(1, 4), ratio(1, 2))],
        },
    );
    let isolated = BezierPathRangeBatchReport2::from_range_reports(&[ambiguous]);
    assert_eq!(
        isolated.status,
        BezierPathRangeBatchStatus::NeedsRegionIsolation
    );
    assert!(isolated.has_blockers());
    assert_eq!(isolated.unclassified_parameters, vec![half()]);
    assert_eq!(isolated.isolating_spans.len(), 1);

    let uncertain = BezierPathRangeOrderReport2::from_classified_graph_contact_order(
        &Classification::Uncertain(UncertaintyReason::Ordering),
    );
    let batch = BezierPathRangeBatchReport2::from_range_reports(&[uncertain]);
    assert_eq!(batch.status, BezierPathRangeBatchStatus::Uncertain);
    assert_eq!(batch.uncertain_range_count, 1);
    assert_eq!(batch.uncertainty_reason, Some(UncertaintyReason::Ordering));
}

proptest! {
    #[test]
    fn generated_bezier_boolean_handoff_exact_point_regions_are_split_ready(
        numerator in 0_i32..=128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(parameter.clone(), parameter.clone()),
            span(parameter.clone(), parameter.clone()),
        );
        let relation = BezierCurveRelation::IntersectionRegions {
            regions: vec![region],
        };

        let report = BezierBooleanHandoffReport2::from_relation(&relation);

        prop_assert_eq!(report.status, BezierBooleanHandoffStatus::SplitEventsReady);
        prop_assert!(report.can_feed_split_events());
        prop_assert_eq!(report.point_events.len(), 1);
        prop_assert_eq!(&report.point_events[0].first_param, &parameter);
        prop_assert_eq!(&report.point_events[0].second_param, &parameter);
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_handoff_positive_regions_remain_isolation_blockers(
        numerator in 0_i32..64,
        width in 1_i32..64,
    ) {
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let end = &start + &(Real::from(width) / Real::from(256_i32)).unwrap();
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end.clone()),
            span(start, end),
        );
        let relation = BezierCurveRelation::IntersectionRegions {
            regions: vec![region],
        };

        let report = BezierBooleanHandoffReport2::from_relation(&relation);

        prop_assert_eq!(report.status, BezierBooleanHandoffStatus::NeedsRegionIsolation);
        prop_assert!(report.has_blockers());
        prop_assert_eq!(
            report
                .region_summary
                .as_ref()
                .expect("region handoff should retain a summary")
                .same_parameter_isolating_spans,
            1
        );
        prop_assert!(report.point_events.is_empty());
    }

    #[test]
    fn generated_bezier_boolean_batch_handoff_exact_points_are_split_ready(
        numerator in 0_i32..=128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(parameter.clone(), parameter.clone()),
            span(parameter.clone(), parameter.clone()),
        );
        let relations = vec![
            Classification::Decided(BezierCurveRelation::NoIntersection),
            Classification::Decided(BezierCurveRelation::IntersectionRegions {
                regions: vec![region],
            }),
        ];

        let batch = BezierBooleanBatchHandoffReport2::from_classified_relations(&relations);

        prop_assert_eq!(batch.status, BezierBooleanBatchHandoffStatus::SplitEventsReady);
        prop_assert!(batch.can_feed_split_events());
        prop_assert_eq!(batch.no_event_relation_count, 1);
        prop_assert_eq!(batch.point_events.len(), 1);
        prop_assert_eq!(&batch.point_events[0].first_param, &parameter);
        prop_assert!(!batch.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_overlap_resolution_sorts_reversed_ranges(
        a in 0_i32..96,
        b in 32_i32..=128,
    ) {
        prop_assume!(a < b);
        let start = (Real::from(a) / Real::from(128_i32)).unwrap();
        let end = (Real::from(b) / Real::from(128_i32)).unwrap();
        let event = hypercurve::BezierBooleanOverlapEvent2 {
            first_range: ParamRange::new(start.clone(), end.clone()),
            second_range: ParamRange::new(end.clone(), start.clone()),
        };

        let report = BezierBooleanOverlapResolutionReport2::from_overlap_events(
            &[event],
            &policy(),
        )
        .unwrap_decided_for_test();

        prop_assert_eq!(report.status, BezierBooleanOverlapResolutionStatus::Ready);
        prop_assert_eq!(&report.first_curve_boundary_parameters, &vec![start.clone(), end.clone()]);
        prop_assert_eq!(&report.second_curve_boundary_parameters, &vec![start, end]);
        prop_assert_eq!(report.resolved_events.len(), 1);
        prop_assert_eq!(report.invalid_range_count, 0);
    }

    #[test]
    fn generated_bezier_boolean_path_scheduler_counts_represented_split_events(
        numerator in 0_i32..=128,
        tangent in any::<bool>(),
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(parameter.clone(), parameter.clone()),
            span(parameter.clone(), parameter.clone()),
        );
        let relation = BezierBooleanHandoffReport2::from_relation(
            &BezierCurveRelation::IntersectionRegions {
                regions: vec![region],
            },
        );
        let kind = if tangent {
            BezierLineContactKind::Tangent
        } else {
            BezierLineContactKind::Crossing
        };
        let range = BezierPathRangeOrderReport2::from_graph_contact_order(
            &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
                contacts: vec![BezierGraphContact::new(parameter.clone(), kind)],
                spans: Vec::new(),
            },
        );

        let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[range]);

        prop_assert_eq!(scheduler.status, BezierBooleanPathSchedulerStatus::SplitEventsReady);
        prop_assert!(scheduler.can_feed_split_events());
        prop_assert_eq!(scheduler.represented_split_event_count, 2);
        prop_assert_eq!(&scheduler.relation_point_events[0].first_param, &parameter);
        prop_assert_eq!(&scheduler.range_split_parameters, &vec![parameter]);
        prop_assert!(!scheduler.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_split_plan_preserves_ready_parameters(
        numerator in 0_i32..=128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(parameter.clone(), parameter.clone()),
            span(parameter.clone(), parameter.clone()),
        );
        let relation = BezierBooleanHandoffReport2::from_relation(
            &BezierCurveRelation::IntersectionRegions {
                regions: vec![region],
            },
        );
        let scheduler = BezierBooleanPathSchedulerReport2::from_reports(&[relation], &[]);

        let plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);

        prop_assert_eq!(plan.status, BezierBooleanSplitPlanStatus::Ready);
        prop_assert!(plan.is_ready());
        prop_assert_eq!(&plan.first_curve_parameters, &vec![parameter.clone()]);
        prop_assert_eq!(&plan.second_curve_parameters, &vec![parameter]);
        prop_assert_eq!(plan.relation_event_count, 1);
        prop_assert_eq!(plan.range_event_count, 0);
        prop_assert!(!plan.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_split_plan_audit_accepts_unit_parameters(
        numerator in 0_i32..=128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let plan = BezierBooleanSplitPlanReport2 {
            status: BezierBooleanSplitPlanStatus::Ready,
            scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
            first_curve_parameters: vec![parameter.clone()],
            second_curve_parameters: vec![parameter.clone()],
            shared_range_parameters: vec![parameter],
            relation_event_count: 1,
            range_event_count: 1,
            uncertainty_reason: None,
        };

        let audit = plan.audit(&policy());

        let Classification::Decided(audit) = audit else {
            panic!("exact dyadic unit parameter audit should be decided");
        };
        prop_assert_eq!(audit.status, BezierBooleanSplitPlanAuditStatus::Valid);
        prop_assert_eq!(audit.checked_parameter_count, 3);
        prop_assert_eq!(audit.out_of_range_parameter_count, 0);
        prop_assert!(audit.is_valid());
    }

    #[test]
    fn generated_bezier_boolean_split_insertion_report_keeps_only_interior_parameters(
        numerator in 1_i32..128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let plan = BezierBooleanSplitPlanReport2 {
            status: BezierBooleanSplitPlanStatus::Ready,
            scheduler_status: BezierBooleanPathSchedulerStatus::SplitEventsReady,
            first_curve_parameters: vec![Real::zero(), parameter.clone()],
            second_curve_parameters: vec![parameter.clone(), Real::one()],
            shared_range_parameters: vec![parameter.clone()],
            relation_event_count: 2,
            range_event_count: 1,
            uncertainty_reason: None,
        };

        let report = plan.insertion_report(&policy());

        let Classification::Decided(report) = report else {
            panic!("exact generated insertion report should be decided");
        };
        prop_assert_eq!(report.status, BezierBooleanSplitInsertionStatus::Ready);
        prop_assert_eq!(report.endpoint_parameter_count, 2);
        prop_assert_eq!(report.interior_parameter_count, 3);
        prop_assert_eq!(&report.first_curve_interior_parameters, &vec![parameter.clone()]);
        prop_assert_eq!(&report.second_curve_interior_parameters, &vec![parameter.clone()]);
        prop_assert_eq!(&report.shared_range_interior_parameters, &vec![parameter]);
        prop_assert!(report.is_ready());
    }

    #[test]
    fn generated_bezier_boolean_construction_readiness_accepts_interior_parameters(
        numerator in 1_i32..128,
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(parameter.clone(), parameter.clone()),
            span(parameter.clone(), parameter.clone()),
        );
        let relation = BezierBooleanHandoffReport2::from_relation(
            &BezierCurveRelation::IntersectionRegions {
                regions: vec![region],
            },
        );

        let readiness =
            BezierBooleanConstructionReadinessReport2::from_reports(&[relation], &[], &policy());

        let Classification::Decided(readiness) = readiness else {
            panic!("generated construction readiness should be decided");
        };
        prop_assert_eq!(readiness.status, BezierBooleanConstructionReadinessStatus::Ready);
        prop_assert!(readiness.is_ready());
        prop_assert_eq!(readiness.insertion.interior_parameter_count, 2);
        prop_assert_eq!(&readiness.insertion.first_curve_interior_parameters, &vec![parameter.clone()]);
        prop_assert_eq!(&readiness.insertion.second_curve_interior_parameters, &vec![parameter]);
    }

    #[test]
    fn generated_bezier_boolean_quadratic_fragments_cover_original_endpoints(
        first in 1_i32..96,
        second in 32_i32..128,
    ) {
        prop_assume!(first < second);
        let a = (Real::from(first) / Real::from(128_i32)).unwrap();
        let b = (Real::from(second) / Real::from(128_i32)).unwrap();
        let curve = QuadraticBezier2::new(point(-3, 2), point(5, 11), point(13, -7));

        let report = BezierBooleanQuadraticFragmentReport2::from_parameters(
            &curve,
            &[b.clone(), a.clone(), a.clone()],
            &policy(),
        )
        .unwrap_decided_for_test();

        prop_assert_eq!(report.status, BezierBooleanFragmentConstructionStatus::Ready);
        prop_assert_eq!(&report.inserted_parameters, &vec![a.clone(), b.clone()]);
        prop_assert_eq!(report.fragments.len(), 3);
        prop_assert_eq!(report.fragments.first().unwrap().start(), curve.start());
        prop_assert_eq!(report.fragments.last().unwrap().end(), curve.end());
        prop_assert_eq!(report.fragments[0].end(), &curve.point_at(a.clone()));
        prop_assert_eq!(report.fragments[1].start(), &curve.point_at(a));
        prop_assert_eq!(report.fragments[1].end(), &curve.point_at(b.clone()));
        prop_assert_eq!(report.fragments[2].start(), &curve.point_at(b));
    }

    #[test]
    fn generated_bezier_boolean_rational_quadratic_fragments_cover_original_endpoints(
        first in 1_i32..96,
        second in 32_i32..128,
        weight in 1_i32..8,
    ) {
        prop_assume!(first < second);
        let a = (Real::from(first) / Real::from(128_i32)).unwrap();
        let b = (Real::from(second) / Real::from(128_i32)).unwrap();
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(-3, 2),
            point(5, 11),
            point(13, -7),
            Real::from(weight),
        )
        .unwrap();

        let report = BezierBooleanRationalQuadraticFragmentReport2::from_parameters(
            &curve,
            &[b.clone(), a.clone(), a.clone()],
            &policy(),
        )
        .unwrap_decided_for_test();
        let point_a = curve.point_at(a.clone(), &policy()).unwrap_decided_for_test();
        let point_b = curve.point_at(b.clone(), &policy()).unwrap_decided_for_test();

        prop_assert_eq!(report.status, BezierBooleanFragmentConstructionStatus::Ready);
        prop_assert_eq!(&report.inserted_parameters, &vec![a, b]);
        prop_assert_eq!(report.fragments.len(), 3);
        prop_assert_eq!(report.fragments.first().unwrap().start(), curve.start());
        prop_assert_eq!(report.fragments.last().unwrap().end(), curve.end());
        prop_assert_eq!(report.fragments[0].end(), &point_a);
        prop_assert_eq!(report.fragments[1].start(), &point_a);
        prop_assert_eq!(report.fragments[1].end(), &point_b);
        prop_assert_eq!(report.fragments[2].start(), &point_b);
    }

    #[test]
    fn generated_bezier_boolean_arrangement_readiness_counts_fragments_and_overlap_boundaries(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        overlap_count in 0_usize..8,
    ) {
        let overlap = if overlap_count == 0 {
            BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
                .unwrap_decided_for_test()
        } else {
            let event = hypercurve::BezierBooleanOverlapEvent2 {
                first_range: ParamRange::new(ratio(1, 4), half()),
                second_range: ParamRange::new(half(), ratio(1, 4)),
            };
            BezierBooleanOverlapResolutionReport2::from_overlap_events(&[event], &policy())
                .unwrap_decided_for_test()
        };

        let report = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlap,
        );

        prop_assert_eq!(report.status, BezierBooleanArrangementReadinessStatus::Ready);
        prop_assert_eq!(report.first_fragment_count, first_count);
        prop_assert_eq!(report.second_fragment_count, second_count);
        prop_assert_eq!(report.resolved_overlap_count, usize::from(overlap_count > 0));
        prop_assert_eq!(report.overlap_boundary_parameter_count, if overlap_count > 0 { 4 } else { 0 });
        prop_assert!(report.is_ready());
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_traversal_preconditions_count_chain_gaps(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        first_gap_index in 0_usize..8,
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let mut first = Vec::new();
        let mut cursor = 0_i32;
        for index in 0..first_count {
            let start = if index > 0 && index == first_gap_index % first_count {
                cursor + 7
            } else {
                cursor
            };
            let end = start + 1;
            first.push((point(start, 0), point(end, 0)));
            cursor = end;
        }
        let mut second = Vec::new();
        for index in 0..second_count {
            second.push((point(index as i32, 1), point(index as i32 + 1, 1)));
        }

        let report = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
            &readiness,
            &first,
            &second,
        );

        let expected_gap = usize::from(first_count > 1 && first_gap_index % first_count > 0);
        prop_assert_eq!(report.first_chain_gap_count, expected_gap);
        prop_assert_eq!(report.second_chain_gap_count, 0);
        if expected_gap == 0 {
            prop_assert_eq!(report.status, BezierBooleanTraversalPreconditionStatus::Ready);
            prop_assert!(report.is_ready());
        } else {
            prop_assert_eq!(
                report.status,
                BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
            );
            prop_assert!(report.has_blockers());
        }
    }

    #[test]
    fn generated_bezier_boolean_traversal_schedule_counts_ready_worklist(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
            &readiness,
            &first,
            &second,
        );

        let report = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);

        prop_assert_eq!(report.status, BezierBooleanTraversalScheduleStatus::Ready);
        prop_assert_eq!(report.steps.len(), first_count + second_count);
        prop_assert_eq!(report.first_fragment_count, first_count);
        prop_assert_eq!(report.second_fragment_count, second_count);
        for index in 0..first_count {
            prop_assert_eq!(report.steps[index].operand, BezierBooleanTraversalOperand::First);
            prop_assert_eq!(report.steps[index].fragment_index, index);
        }
        for index in 0..second_count {
            let step = &report.steps[first_count + index];
            prop_assert_eq!(step.operand, BezierBooleanTraversalOperand::Second);
            prop_assert_eq!(step.fragment_index, index);
        }
        prop_assert!(report.is_ready());
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_ownership_classification_counts_actions(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        use_inside in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
            &readiness,
            &first,
            &second,
        );
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let location = if use_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };
        let ownerships = vec![location; first_count + second_count];

        let report = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Xor,
            &ownerships,
        );

        prop_assert_eq!(report.status, BezierBooleanOwnershipClassificationStatus::Ready);
        prop_assert_eq!(report.owned_steps.len(), first_count + second_count);
        prop_assert_eq!(report.boundary_blocker_count, 0);
        prop_assert_eq!(report.missing_ownership_count, 0);
        if use_inside {
            prop_assert_eq!(report.keep_reversed_count, first_count + second_count);
            prop_assert_eq!(report.keep_source_count, 0);
        } else {
            prop_assert_eq!(report.keep_source_count, first_count + second_count);
            prop_assert_eq!(report.keep_reversed_count, 0);
        }
        prop_assert!(report.is_ready());
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_ownership_facts_preserve_schedule_keys(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        use_boundary in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
            &readiness,
            &first,
            &second,
        );
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let facts = schedule
            .steps
            .iter()
            .enumerate()
            .map(|(index, step)| BezierBooleanOwnershipFact2 {
                step: step.clone(),
                opposite_location: if use_boundary && index == 0 {
                    BezierBooleanFragmentOwnershipLocation::Boundary
                } else {
                    BezierBooleanFragmentOwnershipLocation::Outside
                },
            })
            .collect::<Vec<_>>();

        let report = BezierBooleanOwnershipFactReport2::from_schedule_facts(&schedule, &facts);

        prop_assert_eq!(report.scheduled_step_count, first_count + second_count);
        prop_assert_eq!(report.supplied_fact_count, first_count + second_count);
        prop_assert_eq!(report.locations.len(), first_count + second_count);
        prop_assert_eq!(report.step_mismatch_count, 0);
        if use_boundary {
            prop_assert_eq!(
                report.status,
                BezierBooleanOwnershipFactStatus::BoundaryNeedsResolution
            );
            prop_assert_eq!(report.boundary_fact_count, 1);
            prop_assert!(report.has_blockers());
        } else {
            prop_assert_eq!(report.status, BezierBooleanOwnershipFactStatus::Ready);
            prop_assert!(report.is_ready());
            prop_assert!(!report.has_blockers());
        }
    }

    #[test]
    fn generated_bezier_boolean_uniform_ownership_expands_schedule_order(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        first_inside in any::<bool>(),
        second_inside in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions = BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(
            &readiness,
            &first,
            &second,
        );
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let first_location = if first_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };
        let second_location = if second_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };

        let report = BezierBooleanUniformOwnershipFactReport2::from_schedule_locations(
            &schedule,
            first_location,
            second_location,
        );

        prop_assert_eq!(report.status, BezierBooleanUniformOwnershipFactStatus::Ready);
        prop_assert_eq!(report.facts.len(), first_count + second_count);
        for index in 0..first_count {
            prop_assert_eq!(&report.facts[index].step, &schedule.steps[index]);
            prop_assert_eq!(report.facts[index].opposite_location, first_location);
        }
        for index in 0..second_count {
            let fact_index = first_count + index;
            prop_assert_eq!(
                &report.facts[fact_index].step,
                &schedule.steps[fact_index]
            );
            prop_assert_eq!(report.facts[fact_index].opposite_location, second_location);
        }
        prop_assert!(report.validate(&schedule).is_ready());
    }

    #[test]
    fn generated_bezier_boolean_emission_plan_counts_selected_steps(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        use_inside in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let location = if use_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };
        let ownerships = vec![location; first_count + second_count];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Xor,
            &ownerships,
        );

        let plan = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);

        prop_assert_eq!(plan.status, BezierBooleanEmissionPlanStatus::Ready);
        prop_assert_eq!(plan.emitted_steps.len(), first_count + second_count);
        prop_assert_eq!(plan.discarded_steps.len(), 0);
        prop_assert_eq!(plan.keep_source_count + plan.keep_reversed_count, first_count + second_count);
        prop_assert!(plan.is_ready());
        prop_assert!(!plan.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_assembly_readiness_counts_valid_references(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        use_inside in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let location = if use_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };
        let ownerships = vec![location; first_count + second_count];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Xor,
            &ownerships,
        );
        let plan = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);

        let report = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &plan,
            first_count,
            second_count,
        );

        prop_assert_eq!(report.status, BezierBooleanAssemblyReadinessStatus::Ready);
        prop_assert_eq!(report.emitted_step_count, first_count + second_count);
        prop_assert_eq!(report.first_emitted_count, first_count);
        prop_assert_eq!(report.second_emitted_count, second_count);
        prop_assert_eq!(report.invalid_reference_count, 0);
        prop_assert!(report.is_ready());
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_assembly_plan_counts_packaged_references(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        use_inside in any::<bool>(),
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let location = if use_inside {
            BezierBooleanFragmentOwnershipLocation::Inside
        } else {
            BezierBooleanFragmentOwnershipLocation::Outside
        };
        let ownerships = vec![location; first_count + second_count];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Xor,
            &ownerships,
        );
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_count,
            second_count,
        );

        let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
            &assembly,
            &emission,
        );

        prop_assert_eq!(plan.status, BezierBooleanLoopAssemblyPlanStatus::Ready);
        prop_assert_eq!(plan.emitted_steps.len(), first_count + second_count);
        prop_assert_eq!(plan.first_emitted_count, first_count);
        prop_assert_eq!(plan.second_emitted_count, second_count);
        prop_assert_eq!(plan.invalid_reference_count, 0);
        prop_assert!(plan.is_ready());
        prop_assert!(!plan.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_graph_traversal_counts_blockers(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
        branch_vertex_count in 0_usize..4,
        resolved_overlap_count in 0_usize..4,
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let ownerships = vec![
            BezierBooleanFragmentOwnershipLocation::Outside;
            first_count + second_count
        ];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Union,
            &ownerships,
        );
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_count,
            second_count,
        );
        let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
            &assembly,
            &emission,
        );

        let report = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );

        prop_assert_eq!(report.emitted_step_count, first_count + second_count);
        prop_assert_eq!(report.branch_vertex_count, branch_vertex_count);
        prop_assert_eq!(report.resolved_overlap_count, resolved_overlap_count);
        if branch_vertex_count > 0 {
            prop_assert_eq!(
                report.status,
                BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
            );
            prop_assert_eq!(report.blocker_count, branch_vertex_count);
            prop_assert!(report.has_blockers());
        } else if resolved_overlap_count > 0 {
            prop_assert_eq!(
                report.status,
                BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal
            );
            prop_assert_eq!(report.blocker_count, resolved_overlap_count);
            prop_assert!(report.has_blockers());
        } else {
            prop_assert_eq!(report.status, BezierBooleanLoopGraphTraversalStatus::Ready);
            prop_assert!(report.is_ready());
            prop_assert!(!report.has_blockers());
        }
    }

    #[test]
    fn generated_bezier_boolean_loop_graph_walk_accepts_reverse_permutations(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let ownerships = vec![
            BezierBooleanFragmentOwnershipLocation::Outside;
            first_count + second_count
        ];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Union,
            &ownerships,
        );
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_count,
            second_count,
        );
        let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
            &assembly,
            &emission,
        );
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            0,
            0,
        );
        let walk_indices = (0..(first_count + second_count)).rev().collect::<Vec<_>>();

        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            &walk_indices,
        );

        prop_assert_eq!(walk.status, BezierBooleanLoopGraphWalkStatus::Ready);
        prop_assert_eq!(walk.emitted_step_count, first_count + second_count);
        prop_assert_eq!(walk.supplied_walk_step_count, first_count + second_count);
        prop_assert_eq!(&walk.walk_indices, &walk_indices);
        prop_assert_eq!(walk.ordered_steps.len(), first_count + second_count);
        prop_assert!(walk.is_ready());
        prop_assert!(!walk.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_graph_walk_identity_covers_all_emitted_steps(
        first_count in 1_usize..8,
        second_count in 1_usize..8,
    ) {
        let overlaps = BezierBooleanOverlapResolutionReport2::from_overlap_events(&[], &policy())
            .unwrap_decided_for_test();
        let readiness = BezierBooleanArrangementReadinessReport2::from_parts(
            BezierBooleanFragmentConstructionStatus::Ready,
            first_count,
            BezierBooleanFragmentConstructionStatus::Ready,
            second_count,
            &overlaps,
        );
        let first = (0..first_count)
            .map(|index| (point(index as i32, 0), point(index as i32 + 1, 0)))
            .collect::<Vec<_>>();
        let second = (0..second_count)
            .map(|index| (point(index as i32, 1), point(index as i32 + 1, 1)))
            .collect::<Vec<_>>();
        let preconditions =
            BezierBooleanTraversalPreconditionReport2::from_endpoint_chains(&readiness, &first, &second);
        let schedule = BezierBooleanTraversalScheduleReport2::from_preconditions(&preconditions);
        let ownerships = vec![
            BezierBooleanFragmentOwnershipLocation::Outside;
            first_count + second_count
        ];
        let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
            &schedule,
            BooleanOp::Union,
            &ownerships,
        );
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_count,
            second_count,
        );
        let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
            &assembly,
            &emission,
        );
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            0,
            0,
        );

        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(
            &traversal,
            &plan,
        );

        prop_assert_eq!(walk.status, BezierBooleanLoopGraphWalkStatus::Ready);
        prop_assert_eq!(
            &walk.walk_indices,
            &(0..(first_count + second_count)).collect::<Vec<_>>()
        );
        prop_assert_eq!(&walk.ordered_steps, &plan.emitted_steps);
        prop_assert_eq!(walk.supplied_walk_step_count, first_count + second_count);
        prop_assert!(walk.is_ready());
        prop_assert!(!walk.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_closure_counts_exact_cycles(
        loop_count in 1_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };

        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );

        prop_assert_eq!(closure.status, BezierBooleanLoopClosureStatus::Closed);
        prop_assert!(closure.is_closed());
        prop_assert_eq!(closure.closed_loop_count, loop_count);
        prop_assert_eq!(closure.open_chain_count, 0);
        prop_assert_eq!(closure.adjacency_gap_count, 0);
    }

    #[test]
    fn generated_bezier_boolean_output_loop_report_preserves_loop_ranges(
        loop_count in 1_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );

        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);

        prop_assert_eq!(output.status, BezierBooleanOutputLoopStatus::Ready);
        prop_assert_eq!(output.loops.len(), loop_count);
        prop_assert_eq!(output.directed_fragment_count, loop_count * 2);
        prop_assert!(output.is_ready());
        for index in 0..loop_count {
            prop_assert_eq!(output.loops[index].first_directed_fragment_index, index * 2);
            prop_assert_eq!(output.loops[index].directed_fragment_count, 2);
            prop_assert_eq!(&output.loops[index].anchor, &point((index as i32) * 3, 0));
        }
    }

    #[test]
    fn generated_bezier_boolean_output_loop_report_consumes_identity_graph_walk(
        loop_count in 1_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            0,
            0,
        );
        let walk_indices = (0..loop_count * 2).collect::<Vec<_>>();
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            &walk_indices,
        );

        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            &walk,
            &plan,
            &endpoints,
            &[],
        );

        prop_assert_eq!(output.status, BezierBooleanOutputLoopStatus::Ready);
        prop_assert_eq!(output.loops.len(), loop_count);
        prop_assert_eq!(output.directed_fragment_count, loop_count * 2);
        prop_assert!(output.is_ready());
        prop_assert!(!output.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_region_assembly_consumes_graph_walk_depth_facts(
        loop_count in 1_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            0,
            0,
        );
        let walk_indices = (0..loop_count * 2).collect::<Vec<_>>();
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            &walk_indices,
        );
        let depth_facts = (0..loop_count)
            .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                loop_index,
                nesting_depth: loop_index * 2,
            })
            .collect::<Vec<_>>();

        let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_depth_facts(
            &walk,
            &plan,
            &endpoints,
            &[],
            &depth_facts,
        );

        prop_assert_eq!(assembly.status, BezierBooleanRegionAssemblyStatus::Ready);
        prop_assert_eq!(assembly.assigned_loop_count, loop_count);
        prop_assert_eq!(assembly.material_loop_count, loop_count);
        prop_assert_eq!(assembly.hole_loop_count, 0);
        prop_assert!(assembly.is_ready());
        prop_assert!(!assembly.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_result_consumes_graph_walk_depth_facts(loop_count in 1_usize..6) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 4;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
            &plan,
            0,
            0,
        );
        let walk_indices = (0..loop_count * 2).collect::<Vec<_>>();
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            &walk_indices,
        );
        let depth_facts = (0..loop_count)
            .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                loop_index,
                nesting_depth: loop_index * 2,
            })
            .collect::<Vec<_>>();

        let result = BezierBooleanResultReport2::from_graph_walk_depth_facts(
            &walk,
            &plan,
            &endpoints,
            &[],
            &depth_facts,
        );

        prop_assert_eq!(result.status, BezierBooleanResultStatus::Ready);
        prop_assert_eq!(result.assigned_loop_count, loop_count);
        prop_assert_eq!(result.directed_fragment_count, loop_count * 2);
        prop_assert_eq!(result.material_loop_count, loop_count);
        prop_assert_eq!(result.hole_loop_count, 0);
        prop_assert!(result.is_ready());
        prop_assert!(!result.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_result_consumes_schedule_ownership_walk_depth_facts(
        loop_count in 1_usize..6,
    ) {
        let mut steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 4;
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2 + 1,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let schedule = BezierBooleanTraversalScheduleReport2 {
            status: BezierBooleanTraversalScheduleStatus::Ready,
            precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
            first_fragment_count: loop_count * 2,
            second_fragment_count: 0,
            steps,
            resolved_overlap_count: 0,
            overlap_boundary_parameter_count: 0,
            blocker_count: 0,
        };
        let ownership_facts = schedule
            .steps
            .iter()
            .map(|step| BezierBooleanOwnershipFact2 {
                step: step.clone(),
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            })
            .collect::<Vec<_>>();
        let walk_indices = (0..loop_count * 2).collect::<Vec<_>>();
        let depth_facts = (0..loop_count)
            .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                loop_index,
                nesting_depth: loop_index * 2,
            })
            .collect::<Vec<_>>();

        let result = BezierBooleanResultReport2::from_schedule_graph_walk_depth_facts(
            &schedule,
            BooleanOp::Union,
            &ownership_facts,
            &endpoints,
            &[],
            0,
            0,
            &walk_indices,
            &depth_facts,
        );

        prop_assert_eq!(result.status, BezierBooleanResultStatus::Ready);
        prop_assert_eq!(result.assigned_loop_count, loop_count);
        prop_assert_eq!(result.directed_fragment_count, loop_count * 2);
        prop_assert_eq!(result.material_loop_count, loop_count);
        prop_assert_eq!(result.hole_loop_count, 0);
        prop_assert!(result.is_ready());
        prop_assert!(!result.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_result_consumes_schedule_ownership_walk_containment_facts(
        loop_count in 1_usize..6,
    ) {
        let mut steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 4;
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2 + 1,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let schedule = BezierBooleanTraversalScheduleReport2 {
            status: BezierBooleanTraversalScheduleStatus::Ready,
            precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
            first_fragment_count: loop_count * 2,
            second_fragment_count: 0,
            steps,
            resolved_overlap_count: 0,
            overlap_boundary_parameter_count: 0,
            blocker_count: 0,
        };
        let ownership_facts = schedule
            .steps
            .iter()
            .map(|step| BezierBooleanOwnershipFact2 {
                step: step.clone(),
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
            })
            .collect::<Vec<_>>();
        let walk_indices = (0..loop_count * 2).collect::<Vec<_>>();
        let containment_facts = (1..loop_count)
            .map(|contained_loop_index| BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index,
            })
            .collect::<Vec<_>>();

        let result = BezierBooleanResultReport2::from_schedule_graph_fact_walk_containment_facts(
            &schedule,
            BooleanOp::Union,
            &ownership_facts,
            &endpoints,
            &[],
            &BezierBooleanLoopGraphFacts2 {
                emitted_step_count: loop_count * 2,
                branch_vertex_count: 0,
                resolved_overlap_count: 0,
            },
            &walk_indices,
            &containment_facts,
        );

        prop_assert_eq!(result.status, BezierBooleanResultStatus::Ready);
        prop_assert_eq!(result.assigned_loop_count, loop_count);
        prop_assert_eq!(result.directed_fragment_count, loop_count * 2);
        prop_assert_eq!(result.material_loop_count, 1);
        prop_assert_eq!(result.hole_loop_count, loop_count.saturating_sub(1));
        prop_assert!(result.is_ready());
        prop_assert!(!result.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_result_consumes_uniform_identity_containment_facts(
        loop_count in 1_usize..6,
    ) {
        let mut steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 4;
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));
            steps.push(hypercurve::BezierBooleanTraversalStep2 {
                operand: BezierBooleanTraversalOperand::First,
                fragment_index: index * 2 + 1,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let schedule = BezierBooleanTraversalScheduleReport2 {
            status: BezierBooleanTraversalScheduleStatus::Ready,
            precondition_status: BezierBooleanTraversalPreconditionStatus::Ready,
            first_fragment_count: loop_count * 2,
            second_fragment_count: 0,
            steps,
            resolved_overlap_count: 0,
            overlap_boundary_parameter_count: 0,
            blocker_count: 0,
        };
        let containment_facts = (1..loop_count)
            .map(|contained_loop_index| BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index,
            })
            .collect::<Vec<_>>();

        let result =
            BezierBooleanResultReport2::from_schedule_uniform_graph_fact_identity_containment_facts(
                &schedule,
                BooleanOp::Union,
                BezierBooleanFragmentOwnershipLocation::Outside,
                BezierBooleanFragmentOwnershipLocation::Outside,
                &endpoints,
                &[],
                &BezierBooleanLoopGraphFacts2 {
                    emitted_step_count: loop_count * 2,
                    branch_vertex_count: 0,
                    resolved_overlap_count: 0,
                },
                &containment_facts,
            );

        prop_assert_eq!(result.status, BezierBooleanResultStatus::Ready);
        prop_assert_eq!(result.assigned_loop_count, loop_count);
        prop_assert_eq!(result.directed_fragment_count, loop_count * 2);
        prop_assert_eq!(result.material_loop_count, 1);
        prop_assert_eq!(result.hole_loop_count, loop_count.saturating_sub(1));
        prop_assert!(result.is_ready());
        prop_assert!(!result.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_graph_facts_validate_plan_keys(step_count in 1_usize..12) {
        let emitted_steps = (0..step_count)
            .map(|fragment_index| hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            })
            .collect::<Vec<_>>();
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: step_count,
            second_emitted_count: 0,
            keep_source_count: step_count,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let ready = BezierBooleanLoopGraphFactReport2::from_plan_facts(
            &plan,
            &BezierBooleanLoopGraphFacts2 {
                emitted_step_count: step_count,
                branch_vertex_count: 0,
                resolved_overlap_count: 0,
            },
        );
        let stale = BezierBooleanLoopGraphFactReport2::from_plan_facts(
            &plan,
            &BezierBooleanLoopGraphFacts2 {
                emitted_step_count: step_count + 1,
                branch_vertex_count: 0,
                resolved_overlap_count: 0,
            },
        );

        prop_assert_eq!(ready.status, BezierBooleanLoopGraphFactStatus::Ready);
        prop_assert!(ready.is_ready());
        prop_assert_eq!(
            stale.status,
            BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch
        );
        prop_assert!(stale.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_containment_facts_derive_depths(loop_count in 1_usize..8) {
        let output = BezierBooleanOutputLoopReport2 {
            status: BezierBooleanOutputLoopStatus::Ready,
            closure_status: BezierBooleanLoopClosureStatus::Closed,
            operation: BooleanOp::Union,
            directed_fragments: Vec::new(),
            loops: (0..loop_count)
                .map(|loop_index| hypercurve::BezierBooleanOutputLoop2 {
                    first_directed_fragment_index: loop_index,
                    directed_fragment_count: 1,
                    anchor: point(loop_index as i32, 0),
                })
                .collect(),
            closed_loop_count: loop_count,
            directed_fragment_count: loop_count,
            open_chain_count: 0,
            adjacency_gap_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let containment_facts = (1..loop_count)
            .map(|contained_loop_index| BezierBooleanLoopContainmentFact2 {
                container_loop_index: 0,
                contained_loop_index,
            })
            .collect::<Vec<_>>();
        let report =
            BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
                &output,
                &containment_facts,
            );

        prop_assert_eq!(
            report.status,
            BezierBooleanLoopContainmentFactStatus::Ready
        );
        prop_assert_eq!(report.depth_facts.len(), loop_count);
        prop_assert_eq!(report.depth_facts[0].nesting_depth, 0);
        for fact in report.depth_facts.iter().skip(1) {
            prop_assert_eq!(fact.nesting_depth, 1);
        }
    }

    #[test]
    fn generated_bezier_boolean_loop_nesting_roles_follow_depth_parity(
        loop_count in 1_usize..6,
        depth_offset in 0_usize..8,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );
        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
        let depths = (0..loop_count)
            .map(|index| index + depth_offset)
            .collect::<Vec<_>>();

        let generated = BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(
            &output,
            &depths,
        );

        let expected_material = depths.iter().filter(|depth| **depth % 2 == 0).count();
        prop_assert_eq!(generated.status, BezierBooleanLoopNestingRoleStatus::Ready);
        prop_assert_eq!(generated.output_loop_count, loop_count);
        prop_assert_eq!(generated.supplied_depth_count, loop_count);
        prop_assert_eq!(generated.material_loop_count, expected_material);
        prop_assert_eq!(generated.hole_loop_count, loop_count - expected_material);
        for (role, depth) in generated.roles.iter().zip(depths.iter()) {
            if depth % 2 == 0 {
                prop_assert_eq!(*role, BezierBooleanOutputLoopRole::Material);
            } else {
                prop_assert_eq!(*role, BezierBooleanOutputLoopRole::Hole);
            }
        }
        prop_assert!(generated.is_ready());
        prop_assert!(!generated.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_nesting_depth_facts_preserve_loop_keys(
        loop_count in 1_usize..6,
        depth_offset in 0_usize..8,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );
        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
        let facts = (0..loop_count)
            .map(|index| BezierBooleanLoopNestingDepthFact2 {
                loop_index: index,
                nesting_depth: index + depth_offset,
            })
            .collect::<Vec<_>>();

        let report = BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(
            &output,
            &facts,
        );

        prop_assert_eq!(report.status, BezierBooleanLoopNestingDepthFactStatus::Ready);
        prop_assert_eq!(report.output_loop_count, loop_count);
        prop_assert_eq!(report.supplied_fact_count, loop_count);
        prop_assert_eq!(report.depths.len(), loop_count);
        for (index, depth) in report.depths.iter().enumerate() {
            prop_assert_eq!(*depth, index + depth_offset);
        }
        prop_assert!(report.is_ready());
        prop_assert!(!report.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_loop_role_assignment_counts_roles(
        loop_count in 1_usize..6,
        first_hole_index in 0_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );
        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
        let hole_index = first_hole_index % loop_count;
        let roles = (0..loop_count)
            .map(|index| {
                if index >= hole_index {
                    BezierBooleanOutputLoopRole::Hole
                } else {
                    BezierBooleanOutputLoopRole::Material
                }
            })
            .collect::<Vec<_>>();

        let assigned = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
            &output,
            &roles,
        );

        prop_assert_eq!(assigned.status, BezierBooleanLoopRoleAssignmentStatus::Ready);
        prop_assert_eq!(assigned.output_loop_count, loop_count);
        prop_assert_eq!(assigned.supplied_role_count, loop_count);
        prop_assert_eq!(assigned.material_loop_count, hole_index);
        prop_assert_eq!(assigned.hole_loop_count, loop_count - hole_index);
        prop_assert_eq!(assigned.unknown_role_count, 0);
        prop_assert!(assigned.is_ready());
        prop_assert!(!assigned.has_blockers());
    }

    #[test]
    fn generated_bezier_boolean_region_assembly_preserves_role_indices(
        loop_count in 1_usize..6,
        first_hole_index in 0_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );
        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
        let hole_index = first_hole_index % loop_count;
        let roles = (0..loop_count)
            .map(|index| {
                if index >= hole_index {
                    BezierBooleanOutputLoopRole::Hole
                } else {
                    BezierBooleanOutputLoopRole::Material
                }
            })
            .collect::<Vec<_>>();
        let assigned = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
            &output,
            &roles,
        );

        let assembly = BezierBooleanRegionAssemblyReport2::from_role_assignment(&assigned);

        if hole_index == 0 {
            prop_assert_eq!(
                assembly.status,
                BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial
            );
            prop_assert!(assembly.has_blockers());
        } else {
            prop_assert_eq!(assembly.status, BezierBooleanRegionAssemblyStatus::Ready);
            prop_assert!(assembly.is_ready());
        }
        prop_assert_eq!(assembly.material_loop_indices, (0..hole_index).collect::<Vec<_>>());
        prop_assert_eq!(assembly.hole_loop_indices, (hole_index..loop_count).collect::<Vec<_>>());
        prop_assert_eq!(assembly.assigned_loop_count, loop_count);
        prop_assert_eq!(assembly.material_loop_count, hole_index);
        prop_assert_eq!(assembly.hole_loop_count, loop_count - hole_index);
    }

    #[test]
    fn generated_bezier_boolean_result_report_preserves_ready_artifact_counts(
        loop_count in 1_usize..6,
    ) {
        let mut emitted_steps = Vec::new();
        let mut endpoints = Vec::new();
        for index in 0..loop_count {
            let x = (index as i32) * 3;
            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x, 0), point(x + 1, 0)));

            emitted_steps.push(hypercurve::BezierBooleanOwnedTraversalStep2 {
                step: hypercurve::BezierBooleanTraversalStep2 {
                    operand: BezierBooleanTraversalOperand::First,
                    fragment_index: index * 2 + 1,
                },
                opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                action: BooleanFragmentAction::KeepSourceDirection,
            });
            endpoints.push((point(x + 1, 0), point(x, 0)));
        }
        let plan = BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: BezierBooleanAssemblyReadinessStatus::Ready,
            operation: BooleanOp::Union,
            emitted_steps,
            first_emitted_count: loop_count * 2,
            second_emitted_count: 0,
            keep_source_count: loop_count * 2,
            keep_reversed_count: 0,
            invalid_reference_count: 0,
            blocker_count: 0,
        };
        let closure = BezierBooleanLoopClosureReport2::from_fragment_endpoints(
            &plan,
            &endpoints,
            &[],
        );
        let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
        let roles = vec![BezierBooleanOutputLoopRole::Material; loop_count];
        let assigned = BezierBooleanLoopRoleAssignmentReport2::from_output_loops(
            &output,
            &roles,
        );
        let assembly = BezierBooleanRegionAssemblyReport2::from_role_assignment(&assigned);

        let result = BezierBooleanResultReport2::from_region_assembly(&assembly);

        prop_assert_eq!(result.status, BezierBooleanResultStatus::Ready);
        prop_assert_eq!(result.assigned_loop_count, loop_count);
        prop_assert_eq!(result.material_loop_count, loop_count);
        prop_assert_eq!(result.hole_loop_count, 0);
        prop_assert_eq!(result.directed_fragment_count, loop_count * 2);
        prop_assert_eq!(&result.material_loop_indices, &(0..loop_count).collect::<Vec<_>>());
        prop_assert!(result.is_ready());
        prop_assert!(!result.has_blockers());
    }

    #[test]
    fn generated_bezier_path_range_batch_reports_split_ready_contacts(
        numerator in 0_i32..=128,
        tangent in any::<bool>(),
    ) {
        let parameter = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let kind = if tangent {
            BezierLineContactKind::Tangent
        } else {
            BezierLineContactKind::Crossing
        };
        let contact = BezierPathRangeOrderReport2::from_graph_contact_order(
            &BezierMonotoneGraphContactOrder::IntersectsOrTouches {
                contacts: vec![BezierGraphContact::new(parameter.clone(), kind)],
                spans: Vec::new(),
            },
        );
        let ordered = BezierPathRangeOrderReport2::from_graph_contact_order(
            &BezierMonotoneGraphContactOrder::FirstGreater,
        );

        let batch = BezierPathRangeBatchReport2::from_range_reports(&[ordered, contact]);

        prop_assert_eq!(batch.status, BezierPathRangeBatchStatus::SplitEventsReady);
        prop_assert!(batch.can_feed_split_events());
        prop_assert_eq!(&batch.split_parameters, &vec![parameter]);
        prop_assert_eq!(batch.crossing_contact_count + batch.tangent_contact_count, 1);
        prop_assert!(!batch.has_blockers());
    }

    #[test]
    fn quadratic_endpoint_evaluation_is_exact(
        ax in -32_i32..32,
        ay in -32_i32..32,
        bx in -32_i32..32,
        by in -32_i32..32,
        cx in -32_i32..32,
        cy in -32_i32..32,
    ) {
        let start = point(ax, ay);
        let control = point(bx, by);
        let end = point(cx, cy);
        let curve = QuadraticBezier2::new(start.clone(), control, end.clone());

        prop_assert_eq!(curve.point_at(Real::zero()), start.clone());
        prop_assert_eq!(curve.point_at(Real::one()), end.clone());
        prop_assert_eq!(
            curve.contains_point_at_parameter(&start, Real::zero(), &policy()),
            Classification::Decided(true)
        );
        prop_assert_eq!(
            curve.contains_point_at_parameter(&end, Real::one(), &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn quadratic_point_solver_accepts_generated_arch_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let midpoint = point(mid_x, mid_y);

        prop_assert_eq!(
            arch.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        let Classification::Decided(parameters) = arch.parameters_for_point(&midpoint, &policy())
        else {
            panic!("generated quadratic midpoint should have certified parameters");
        };
        prop_assert!(!parameters.is_empty());
        for parameter in parameters {
            prop_assert_eq!(
                arch.contains_point_at_parameter(&midpoint, parameter, &policy()),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn cubic_midpoint_stays_inside_control_hull_for_integer_controls(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        dx in -8_i32..8,
        dy in -8_i32..8,
    ) {
        let curve = CubicBezier2::new(point(ax, ay), point(bx, by), point(cx, cy), point(dx, dy));
        let midpoint = curve.point_at(half());
        let bbox = match curve.control_hull_box(&policy()) {
            Classification::Decided(bbox) => bbox,
            Classification::Uncertain(reason) => panic!("integer hull ordering should be decided: {reason:?}"),
        };

        prop_assert_eq!(
            bbox.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn quadratic_certified_bounds_are_inside_control_hull_for_integer_controls(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
    ) {
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let midpoint = curve.point_at(half());
        let bounds = match curve.certified_bounds(&policy()) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => panic!("integer bounds should be decided: {reason:?}"),
        };

        prop_assert_eq!(
            bounds.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        prop_assert!(curve.monotone_spans(&policy()).map(|spans| !spans.is_empty()) == Classification::Decided(true));
    }

    #[test]
    fn rational_quadratic_positive_integer_weights_evaluate_midpoint(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();

        let Classification::Decided(_) = curve.point_at(half(), &policy()) else {
            panic!("positive integer weights should not hit the projective denominator boundary");
        };
        prop_assert_eq!(
            curve.point_at(Real::zero(), &policy()).unwrap_decided_for_test(),
            curve.start().clone()
        );
        prop_assert_eq!(
            curve.point_at(Real::one(), &policy()).unwrap_decided_for_test(),
            curve.end().clone()
        );
        let baseline = LineSeg2::try_new(point(-100, ay), point(100, ay)).unwrap();
        let relation = curve.relation_to_line(&baseline, &policy());
        prop_assert!(
            matches!(
                relation,
                Classification::Decided(BezierLineRelation::Intersects { .. })
                    | Classification::Decided(BezierLineRelation::OnSupportingLine)
                    | Classification::Decided(BezierLineRelation::Unresolved)
                    | Classification::Decided(BezierLineRelation::ControlHullDisjoint { .. })
            ),
            "positive integer rational conic line relation should decide or return an explicit unresolved relation, got {relation:?}"
        );
        prop_assert_eq!(
            curve.relation_to_rational_quadratic(&curve, &policy()),
            Classification::Decided(BezierCurveRelation::SameControlPolygon)
        );
        prop_assert!(curve.structural_facts().all_exact_rational());
    }

    #[test]
    fn rational_projective_weight_scaling_preserves_generated_conic_images(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
        scale in 2_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();
        let scaled = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0 * scale),
            Real::from(w1 * scale),
            Real::from(w2 * scale),
        )
        .unwrap();
        let reversed_scaled = RationalQuadraticBezier2::try_new(
            point(cx, cy),
            point(bx, by),
            point(ax, ay),
            Real::from(w2 * scale),
            Real::from(w1 * scale),
            Real::from(w0 * scale),
        )
        .unwrap();

        prop_assert_eq!(
            curve.relation_to_rational_quadratic(&scaled, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            scaled.relation_to_rational_quadratic(&curve, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            curve.relation_to_rational_quadratic(&reversed_scaled, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            reversed_scaled.relation_to_rational_quadratic(&curve, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
    }

    #[test]
    fn rational_point_solver_accepts_generated_positive_weight_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + 2 * height;
        let curve = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(mid_x, ay + 3 * height),
            point(end_x, ay),
            Real::from(2_i8),
        )
        .unwrap();
        let midpoint = point(mid_x, mid_y);

        prop_assert_eq!(
            curve.contains_point(&midpoint, &policy()),
            Classification::Decided(true)
        );
        let Classification::Decided(parameters) = curve.parameters_for_point(&midpoint, &policy())
        else {
            panic!("generated rational midpoint should have certified parameters");
        };
        prop_assert!(!parameters.is_empty());
        for parameter in parameters {
            prop_assert_eq!(
                curve.contains_point_at_parameter(&midpoint, parameter, &policy()),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn rational_quadratic_positive_integer_weight_bounds_contain_midpoint(
        ax in -8_i32..8,
        ay in -8_i32..8,
        bx in -8_i32..8,
        by in -8_i32..8,
        cx in -8_i32..8,
        cy in -8_i32..8,
        w0 in 1_i32..8,
        w1 in 1_i32..8,
        w2 in 1_i32..8,
    ) {
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w0),
            Real::from(w1),
            Real::from(w2),
        )
        .unwrap();
        let midpoint = curve.point_at(half(), &policy()).unwrap_decided_for_test();
        match curve.certified_bounds(&policy()) {
            Classification::Decided(bounds) => {
                prop_assert_eq!(
                    bounds.contains_point(&midpoint, &policy()),
                    Classification::Decided(true)
                );
            }
            Classification::Uncertain(reason) => {
                prop_assert!(
                    matches!(
                        reason,
                        hypercurve::UncertaintyReason::Ordering
                            | hypercurve::UncertaintyReason::RealSign
                    ),
                    "positive weights should only leave extrema ordering/sign uncertainty, got {reason:?}"
                );
            }
        }
        match curve.monotone_spans(&policy()) {
            Classification::Decided(spans) => prop_assert!(!spans.is_empty()),
            Classification::Uncertain(reason) => prop_assert!(
                matches!(
                    reason,
                    hypercurve::UncertaintyReason::Ordering
                        | hypercurve::UncertaintyReason::RealSign
                ),
                "positive weights should only leave monotone ordering/sign uncertainty, got {reason:?}"
            ),
        }
    }

    #[test]
    fn mixed_polynomial_bezier_far_boxes_are_decided(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        offset in 100_i32..200,
    ) {
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let cubic = CubicBezier2::new(
            point(ax + offset, ay + offset),
            point(bx + offset, by + offset),
            point(cx + offset, cy + offset),
            point(cx + offset + 1, cy + offset),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&cubic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
    }

    #[test]
    fn rational_polynomial_positive_weight_far_boxes_are_decided(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        w in 1_i32..8,
        offset in 100_i32..200,
    ) {
        let rational = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(bx, by),
            point(cx, cy),
            Real::from(w),
        )
        .unwrap();
        let quadratic = QuadraticBezier2::new(
            point(ax + offset, ay + offset),
            point(bx + offset, by + offset),
            point(cx + offset, cy + offset),
        );
        let cubic = CubicBezier2::new(
            point(ax - offset, ay - offset),
            point(bx - offset, by - offset),
            point(cx - offset, cy - offset),
            point(cx - offset - 1, cy - offset),
        );

        prop_assert_eq!(
            rational.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            quadratic.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            rational.relation_to_cubic(&cubic, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
        prop_assert_eq!(
            cubic.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint)
        );
    }

    #[test]
    fn large_error_flattening_keeps_generated_bezier_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
        cx in -16_i32..16,
        cy in -16_i32..16,
        dx in -16_i32..16,
        dy in -16_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, by), point(cx, cy));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, by), point(cx, cy), point(dx, dy));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let quadratic_polyline = quadratic.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let cubic_polyline = cubic.flatten_certified(&options, &policy()).unwrap_decided_for_test();

        prop_assert_eq!(quadratic_polyline.points(), &[point(ax, ay), point(cx, cy)]);
        prop_assert_eq!(quadratic_polyline.certificate().segment_count(), 1);
        prop_assert_eq!(cubic_polyline.points(), &[point(ax, ay), point(dx, dy)]);
        prop_assert_eq!(cubic_polyline.certificate().segment_count(), 1);
    }

    #[test]
    fn offset_preflight_preserves_generated_line_and_collapsed_risks(
        ax in -16_i32..16,
        ay in -16_i32..16,
        dx in 1_i32..16,
    ) {
        let line = QuadraticBezier2::new(point(ax, ay), point(ax + dx, ay), point(ax + 2 * dx, ay));
        let Classification::Decided(preflight) = line.offset_preflight(&policy()) else {
            panic!("generated nondegenerate line preflight should decide");
        };
        prop_assert!(preflight.is_clear());
        prop_assert_eq!(preflight.degree(), BezierDegree::Quadratic);
        prop_assert_eq!(preflight.construction_policy().numeric_mode, NumericMode::Certified);

        let collapsed = CubicBezier2::new(point(ax, ay), point(ax, ay), point(ax, ay), point(ax, ay));
        let Classification::Decided(preflight) = collapsed.offset_preflight(&policy()) else {
            panic!("generated collapsed cubic preflight should decide");
        };
        prop_assert!(preflight.risks().contains(&BezierOffsetRisk::DegeneratePoint));
        prop_assert!(preflight.risks().contains(&BezierOffsetRisk::CoincidentEndpoints));
        let start_normal_is_undefined = preflight.risks().contains(&BezierOffsetRisk::UndefinedEndpointNormal {
            endpoint: BezierEndpoint::Start
        });
        let end_normal_is_undefined = preflight.risks().contains(&BezierOffsetRisk::UndefinedEndpointNormal {
            endpoint: BezierEndpoint::End
        });
        prop_assert!(start_normal_is_undefined);
        prop_assert!(end_normal_is_undefined);
    }

    #[test]
    fn generated_offset_adapter_reports_separate_ready_from_exact_line_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        dx in 1_i32..16,
        lift in 1_i32..16,
    ) {
        let line = QuadraticBezier2::new(point(ax, ay), point(ax + dx, ay), point(ax + 2 * dx, ay));
        let Classification::Decided(line_candidate) = line.offset_left_staged(Real::one(), &policy()).unwrap() else {
            panic!("generated line image should decide");
        };
        let line_report = line_candidate.adapter_report();
        prop_assert_eq!(
            line_report.status,
            BezierOffsetAdapterStatus::ExactPrimitiveLineImage
        );
        prop_assert!(line_report.has_exact_primitive);

        let arch = QuadraticBezier2::new(point(ax, ay), point(ax + dx, ay + lift), point(ax + 2 * dx, ay));
        let Classification::Decided(arch_candidate) = arch.offset_left_staged(Real::one(), &policy()).unwrap() else {
            panic!("generated arch should decide as unresolved");
        };
        let arch_report = arch_candidate.adapter_report();
        prop_assert_eq!(
            arch_report.status,
            BezierOffsetAdapterStatus::ReadyForCertifiedAdapter
        );
        prop_assert!(!arch_report.has_exact_primitive);
        prop_assert!(arch_report.may_attempt_certified_adapter);

        let zero_report = arch.offset_left_staged(Real::zero(), &policy()).unwrap().unwrap_decided_for_test().adapter_report();
        prop_assert_eq!(
            zero_report.status,
            BezierOffsetAdapterStatus::ZeroDistanceIdentity
        );
        prop_assert_eq!(zero_report.distance_status, ZeroStatus::Zero);
    }

    #[test]
    fn exact_collinear_simplification_preserves_generated_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        cx in -16_i32..16,
    ) {
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let simplified = polyline.simplify_exact_collinear(&policy()).unwrap_decided_for_test();

        prop_assert_eq!(simplified.points().first(), Some(&point(ax, ay)));
        prop_assert_eq!(simplified.points().last(), Some(&point(cx, ay)));
        prop_assert!(simplified.points().len() <= polyline.points().len());
        prop_assert_eq!(simplified.certificate().max_error(), polyline.certificate().max_error());
        let certificate = simplified.simplification_certificate().unwrap();
        prop_assert_eq!(certificate.source_end(), polyline.points().len());
        prop_assert_eq!(certificate.retained_vertex_count(), simplified.points().len());
        prop_assert_eq!(
            certificate.removed_vertex_count(),
            polyline.points().len() - simplified.points().len()
        );
        prop_assert_eq!(certificate.error_bound(), &Real::zero());
        prop_assert_eq!(certificate.source_flattening_max_depth(), options.max_depth());
        prop_assert_eq!(certificate.bound_kind(), BezierSimplificationBoundKind::ProvenExact);
    }

    #[test]
    fn exact_line_fit_preserves_generated_collinear_endpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        cx in -16_i32..16,
    ) {
        prop_assume!(ax != cx);
        let curve = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let fit = polyline.fit_exact_line(&policy()).unwrap();

        let Classification::Decided(BezierLineFitRelation::Fit(fit)) = fit else {
            panic!("generated collinear quadratic should fit one exact line");
        };
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(cx, ay));
        prop_assert_eq!(fit.source_certificate().max_error(), polyline.certificate().max_error());
        prop_assert_eq!(fit.fit_certificate().source_start(), 0);
        prop_assert_eq!(fit.fit_certificate().source_end(), polyline.points().len());
        prop_assert_eq!(fit.fit_certificate().fit_error_bound(), &Real::zero());
        prop_assert_eq!(
            fit.fit_certificate().source_flattening_error(),
            Some(polyline.certificate().max_error())
        );
        prop_assert_eq!(fit.fit_certificate().source_flattening_max_depth(), Some(options.max_depth()));
    }

    #[test]
    fn exact_point_fit_preserves_generated_collapsed_flattened_polylines(
        ax in -16_i32..16,
        ay in -16_i32..16,
    ) {
        let curve = CubicBezier2::new(point(ax, ay), point(ax, ay), point(ax, ay), point(ax, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve.flatten_certified(&options, &policy()).unwrap_decided_for_test();
        let Classification::Decided(BezierPointFitRelation::Fit(fit)) =
            polyline.fit_exact_point(&policy()).unwrap()
        else {
            panic!("generated collapsed flattened cubic should fit one exact point");
        };
        prop_assert_eq!(fit.point(), &point(ax, ay));
        prop_assert_eq!(fit.source_certificate().max_error(), polyline.certificate().max_error());
        prop_assert_eq!(fit.fit_certificate().fit_error_bound(), &Real::zero());
        prop_assert_eq!(fit.fit_certificate().bound_kind(), BezierFitBoundKind::ProvenExact);
    }

    #[test]
    fn fit_readiness_reports_generated_exact_line_or_point_without_higher_order_claims(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        cx in -16_i32..16,
    ) {
        prop_assume!(ax != cx);
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();

        let line_curve = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let line_polyline = line_curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test();
        let line_report = line_polyline
            .fit_readiness_report(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        prop_assert_eq!(line_report.status(), BezierFitReadinessStatus::ExactLine);
        prop_assert!(line_report.has_exact_primitive_fit());
        prop_assert!(!line_report.needs_higher_order_fit());
        prop_assert!(line_report.line_fit_certificate().is_some());
        prop_assert!(line_report.point_fit_certificate().is_none());

        let point_curve = CubicBezier2::new(point(ax, ay), point(ax, ay), point(ax, ay), point(ax, ay));
        let point_polyline = point_curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test();
        let point_report = point_polyline
            .fit_readiness_report(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        prop_assert_eq!(point_report.status(), BezierFitReadinessStatus::ExactPoint);
        prop_assert!(point_report.has_exact_primitive_fit());
        prop_assert!(point_report.point_fit_certificate().is_some());
        prop_assert!(point_report.line_fit_certificate().is_none());
    }

    #[test]
    fn fit_readiness_routes_generated_arcs_to_higher_order_fitting(
        ax in -8_i32..8,
        ay in -8_i32..8,
        width in 2_i32..16,
        lift in 2_i32..16,
    ) {
        let curve = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + width / 2, ay + lift),
            point(ax + width, ay),
        );
        let options = BezierFlatteningOptions::try_new(Real::one(), 10, &policy()).unwrap();
        let polyline = curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test();
        let report = polyline
            .fit_readiness_report(&policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(
            report.status(),
            BezierFitReadinessStatus::NeedsHigherOrderFit
        );
        prop_assert!(!report.has_exact_primitive_fit());
        prop_assert!(report.needs_higher_order_fit());
        prop_assert!(report.point_fit_certificate().is_none());
        prop_assert!(report.line_fit_certificate().is_none());
        prop_assert_eq!(report.source_vertex_count(), polyline.points().len());
        prop_assert_eq!(report.source_certificate().max_error(), polyline.certificate().max_error());
    }

    #[test]
    fn exact_line_image_fit_preserves_generated_collinear_controls(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 1_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let curve = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let fit = curve
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        let BezierLineImageFitRelation::Fit(fit) = fit else {
            panic!("generated horizontal control polygon should be a line image");
        };

        prop_assert_eq!(fit.control_point_count(), 4);
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(dx, ay));
        prop_assert_eq!(fit.fit_certificate().source_end(), 4);
        prop_assert_eq!(fit.fit_certificate().source_flattening_error(), None);
        let offset = fit.offset_left_exact(Real::one()).unwrap();
        prop_assert_eq!(offset.line().start(), &point(ax, ay + 1));
        prop_assert_eq!(offset.fit_certificate(), fit.fit_certificate());
    }

    #[test]
    fn exact_point_image_fit_preserves_generated_collapsed_controls(
        ax in -16_i32..16,
        ay in -16_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(ax, ay), point(ax, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(ax, ay), point(ax, ay), point(ax, ay));
        let signed_weight = Real::from(weight * sign);
        let rational = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(ax, ay),
            point(ax, ay),
            Real::from(sign),
            signed_weight,
            Real::from(sign),
        ).unwrap();

        let Classification::Decided(BezierPointImageFitRelation::Fit(quadratic_fit)) =
            quadratic.fit_exact_point_image(&policy()).unwrap()
        else {
            panic!("generated collapsed quadratic should fit one exact point");
        };
        prop_assert_eq!(quadratic_fit.point(), &point(ax, ay));
        prop_assert_eq!(quadratic_fit.control_point_count(), 3);
        prop_assert_eq!(quadratic_fit.fit_certificate().source_end(), 3);
        prop_assert_eq!(quadratic_fit.fit_certificate().fit_error_bound(), &Real::zero());

        let Classification::Decided(BezierPointImageFitRelation::Fit(cubic_fit)) =
            cubic.fit_exact_point_image(&policy()).unwrap()
        else {
            panic!("generated collapsed cubic should fit one exact point");
        };
        prop_assert_eq!(cubic_fit.point(), &point(ax, ay));
        prop_assert_eq!(cubic_fit.control_point_count(), 4);
        prop_assert_eq!(cubic_fit.fit_certificate().source_end(), 4);

        let Classification::Decided(BezierPointImageFitRelation::Fit(rational_fit)) =
            rational.fit_exact_point_image(&policy()).unwrap()
        else {
            panic!("generated same-sign collapsed rational should fit one exact point");
        };
        prop_assert_eq!(rational_fit.point(), &point(ax, ay));
        prop_assert_eq!(rational_fit.control_point_count(), 3);
        prop_assert_eq!(rational_fit.fit_certificate().source_end(), 3);
    }

    #[test]
    fn point_image_relations_preserve_generated_quadratic_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let midpoint = arch.point_at(half());
        let collapsed = QuadraticBezier2::new(midpoint.clone(), midpoint.clone(), midpoint.clone());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            collapsed.relation_to_quadratic(&arch, &policy())
        else {
            panic!("generated collapsed point image at quadratic midpoint should promote a hit");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &midpoint);
    }

    #[test]
    fn rational_line_image_fit_preserves_generated_same_sign_weight_lines(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 1_i32..16,
        bc in 0_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let signed_weight = Real::from(weight * sign);
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(bx, ay),
            point(cx, ay),
            Real::from(sign),
            signed_weight,
            Real::from(sign),
        )
        .unwrap();
        let fit = curve
            .fit_exact_line_image(&policy())
            .unwrap()
            .unwrap_decided_for_test();
        let BezierLineImageFitRelation::Fit(fit) = fit else {
            panic!("generated same-sign horizontal rational should be a line image");
        };

        prop_assert_eq!(fit.control_point_count(), 3);
        prop_assert_eq!(fit.line().start(), &point(ax, ay));
        prop_assert_eq!(fit.line().end(), &point(cx, ay));
    }

    #[test]
    fn rational_line_contact_relation_classifies_generated_tangencies(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let tangent_y = Real::from(ay)
            + ((Real::from(2_i32 * weight * height) / Real::from(1_i32 + weight)).unwrap());
        let signed_endpoint_weight = Real::from(sign);
        let signed_control_weight = Real::from(weight * sign);
        let curve = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
            signed_endpoint_weight.clone(),
            signed_control_weight,
            signed_endpoint_weight,
        )
        .unwrap();
        let tangent = LineSeg2::try_new(
            Point2::new(Real::from(ax), tangent_y.clone()),
            Point2::new(Real::from(end_x), tangent_y),
        )
        .unwrap();

        let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
            curve.relation_to_line_with_contacts(&tangent, &policy())
        else {
            panic!("generated same-sign rational tangent should expose represented line contact");
        };
        prop_assert!(contacts.iter().any(|contact| contact.parameter() == &half()
            && contact.kind() == BezierLineContactKind::Tangent));
    }

    #[test]
    fn matching_weight_rational_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
        gap in 1_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let signed_endpoint_weight = Real::from(sign);
        let signed_control_weight = Real::from(weight * sign);
        let lower = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(ax + width, ay + 2 * height),
            point(ax + 2 * width, ay),
            signed_endpoint_weight.clone(),
            signed_control_weight.clone(),
            signed_endpoint_weight.clone(),
        )
        .unwrap();
        let upper = RationalQuadraticBezier2::try_new(
            point(ax, ay + gap),
            point(ax + width, ay + 2 * height + gap),
            point(ax + 2 * width, ay + gap),
            signed_endpoint_weight,
            signed_control_weight,
            Real::from(sign),
        )
        .unwrap();

        prop_assert_eq!(
            lower.graph_order_to_rational_quadratic_over_axis(&upper, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
    }

    #[test]
    fn equal_weight_rational_polynomial_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
        gap in 1_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let signed_weight = Real::from(weight * sign);
        let lower = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(ax + width, ay + 2 * height),
            point(ax + 2 * width, ay),
            signed_weight.clone(),
            signed_weight.clone(),
            signed_weight,
        )
        .unwrap();
        let upper = QuadraticBezier2::new(
            point(ax, ay + gap),
            point(ax + width, ay + 2 * height + gap),
            point(ax + 2 * width, ay + gap),
        );

        prop_assert_eq!(
            lower.graph_order_to_quadratic_over_axis(&upper, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
    }

    #[test]
    fn non_equal_rational_polynomial_strict_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        gap in 1_i32..16,
        base_weight in 1_i32..8,
        extra_weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let w0_unsigned = base_weight;
        let w1_unsigned = base_weight + extra_weight;
        let w2_unsigned = base_weight + 2 * extra_weight;
        let signed_w0 = Real::from(sign * w0_unsigned);
        let signed_w1 = Real::from(sign * w1_unsigned);
        let signed_w2 = Real::from(sign * w2_unsigned);
        let start_x = Real::from(ax);
        let end_x = Real::from(ax + width);
        let control_x_offset =
            (Real::from(width * w0_unsigned) / Real::from(2 * w1_unsigned)).unwrap();
        let rational = RationalQuadraticBezier2::try_new(
            Point2::new(start_x.clone(), Real::from(ay + gap)),
            Point2::new(&start_x + &control_x_offset, Real::from(ay + gap)),
            Point2::new(end_x.clone(), Real::from(ay + gap)),
            signed_w0,
            signed_w1,
            signed_w2,
        )
        .unwrap();
        let polynomial = QuadraticBezier2::new(
            Point2::new(start_x, Real::from(ay)),
            Point2::new((Real::from(2 * ax + width) / Real::from(2_i8)).unwrap(), Real::from(ay)),
            Point2::new(end_x, Real::from(ay)),
        );

        prop_assert_eq!(
            rational.graph_order_to_quadratic_over_axis(&polynomial, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
        prop_assert_eq!(
            polynomial.graph_order_to_rational_quadratic_over_axis(&rational, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            rational.relation_to_quadratic(&polynomial, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            polynomial.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn non_equal_rational_cubic_strict_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        gap in 1_i32..16,
        base_weight in 1_i32..8,
        extra_weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let w0_unsigned = base_weight;
        let w1_unsigned = base_weight + extra_weight;
        let w2_unsigned = base_weight + 2 * extra_weight;
        let signed_w0 = Real::from(sign * w0_unsigned);
        let signed_w1 = Real::from(sign * w1_unsigned);
        let signed_w2 = Real::from(sign * w2_unsigned);
        let start_x = Real::from(ax);
        let end_x = Real::from(ax + width);
        let control_x_offset =
            (Real::from(width * w0_unsigned) / Real::from(2 * w1_unsigned)).unwrap();
        let rational = RationalQuadraticBezier2::try_new(
            Point2::new(start_x.clone(), Real::from(ay + gap)),
            Point2::new(&start_x + &control_x_offset, Real::from(ay + gap)),
            Point2::new(end_x.clone(), Real::from(ay + gap)),
            signed_w0,
            signed_w1,
            signed_w2,
        )
        .unwrap();
        let one_third = (Real::from(width) / Real::from(3_i8)).unwrap();
        let two_thirds = (Real::from(2 * width) / Real::from(3_i8)).unwrap();
        let polynomial = CubicBezier2::new(
            Point2::new(start_x.clone(), Real::from(ay)),
            Point2::new(&start_x + &one_third, Real::from(ay)),
            Point2::new(&start_x + &two_thirds, Real::from(ay)),
            Point2::new(end_x, Real::from(ay)),
        );

        prop_assert_eq!(
            rational.graph_order_to_cubic_over_axis(&polynomial, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
        prop_assert_eq!(
            polynomial.graph_order_to_rational_quadratic_over_axis(&rational, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            rational.relation_to_cubic(&polynomial, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            polynomial.relation_to_rational_quadratic(&rational, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn equal_weight_rational_cubic_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
        gap in 1_i32..16,
        weight in 1_i32..8,
        sign in prop_oneof![Just(1_i32), Just(-1_i32)],
    ) {
        let signed_weight = Real::from(weight * sign);
        let lower = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
            signed_weight.clone(),
            signed_weight.clone(),
            signed_weight,
        )
        .unwrap();
        let upper = CubicBezier2::new(
            point(ax, ay + gap),
            point(ax + 2 * width, ay + 2 * height + gap),
            point(ax + 4 * width, ay + 2 * height + gap),
            point(ax + 6 * width, ay + gap),
        );

        prop_assert_eq!(
            lower.graph_order_to_cubic_over_axis(&upper, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            upper.graph_order_to_rational_quadratic_over_axis(&lower, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
    }

    #[test]
    fn line_image_curve_relation_finds_generated_quadratic_tangencies(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let tangent_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let tangent = QuadraticBezier2::new(
            point(ax, tangent_y),
            point(mid_x, tangent_y),
            point(end_x, tangent_y),
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            tangent.relation_to_quadratic(&arch, &policy())
        else {
            panic!("generated tangent line image should return an exact point");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &point(mid_x, tangent_y));
    }

    #[test]
    fn line_contact_relation_classifies_generated_quadratic_tangencies(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let tangent_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let tangent = LineSeg2::try_new(point(ax, tangent_y), point(end_x, tangent_y)).unwrap();

        let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) =
            arch.relation_to_line_with_contacts(&tangent, &policy())
        else {
            panic!("generated quadratic tangent should expose represented line contact");
        };
        prop_assert_eq!(contacts.len(), 1);
        prop_assert_eq!(contacts[0].parameter(), &half());
        prop_assert_eq!(contacts[0].kind(), BezierLineContactKind::Tangent);
    }

    #[test]
    fn endpoint_on_quadratic_relation_handles_generated_midpoints(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let mid_y = ay + height;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let probe = CubicBezier2::new(
            point(mid_x, mid_y),
            point(mid_x + 1, mid_y + 3),
            point(end_x + 1, mid_y + 3),
            point(end_x + 2, mid_y + 2),
        );

        let relation = probe.relation_to_quadratic(&arch, &policy());
        let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
        else {
            panic!("generated endpoint on quadratic midpoint should be certified: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &point(mid_x, mid_y));
    }

    #[test]
    fn same_parameter_quadratic_crossings_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let crossing = QuadraticBezier2::new(
            point(ax, ay + height),
            point(mid_x, ay - height),
            point(end_x, ay + height),
        );

        let relation = arch.relation_to_quadratic(&crossing, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated same-parameter quadratic crossings should be exact points: {relation:?}");
        };
        prop_assert_eq!(points.len(), 2);
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            crossing.relation_to_quadratic(&arch, &policy())
        else {
            panic!("generated same-parameter quadratic crossing relation should be symmetric");
        };
        prop_assert_eq!(points.len(), 2);
    }

    #[test]
    fn same_axis_monotone_quadratics_with_positive_gap_are_generated_no_hits(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
        gap in 1_i32..16,
    ) {
        prop_assume!(gap <= height);
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let arch = QuadraticBezier2::new(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
        );
        let raised = QuadraticBezier2::new(
            point(ax, ay + gap),
            point(mid_x, ay + 2 * height + gap),
            point(end_x, ay + gap),
        );

        prop_assert_eq!(
            arch.relation_to_quadratic(&raised, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            raised.relation_to_quadratic(&arch, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn degree_normalized_same_axis_quadratic_cubic_gaps_are_generated_no_hits(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
        gap in 1_i32..16,
    ) {
        prop_assume!(gap <= height);
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let elevated_raised = CubicBezier2::new(
            point(ax, ay + gap),
            point(ax + 2 * width, ay + 2 * height + gap),
            point(ax + 4 * width, ay + 2 * height + gap),
            point(ax + 6 * width, ay + gap),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&elevated_raised, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
        prop_assert_eq!(
            elevated_raised.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::NoIntersection)
        );
    }

    #[test]
    fn degree_normalized_same_axis_graph_orders_are_generated(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
        gap in 1_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let raised = CubicBezier2::new(
            point(ax, ay + gap),
            point(ax + 2 * width, ay + 2 * height + gap),
            point(ax + 4 * width, ay + 2 * height + gap),
            point(ax + 6 * width, ay + gap),
        );
        let lowered = CubicBezier2::new(
            point(ax, ay - gap),
            point(ax + 2 * width, ay + 2 * height - gap),
            point(ax + 4 * width, ay + 2 * height - gap),
            point(ax + 6 * width, ay - gap),
        );

        prop_assert_eq!(
            quadratic.graph_order_to_cubic_over_axis(&raised, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstLess)
        );
        prop_assert_eq!(
            quadratic.graph_order_to_cubic_over_axis(&lowered, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
        prop_assert_eq!(
            raised.graph_order_to_quadratic_over_axis(&quadratic, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphOrder::FirstGreater)
        );
        prop_assert_eq!(
            quadratic.graph_contact_order_to_cubic_over_axis(&raised, Axis2::X, &policy()),
            Classification::Decided(BezierMonotoneGraphContactOrder::FirstLess)
        );
    }

    #[test]
    fn degree_elevated_quadratic_cubic_pairs_are_generated_same_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in -16_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let elevated = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 2 * height),
            point(ax + 4 * width, ay + 2 * height),
            point(ax + 6 * width, ay),
        );
        let reversed_elevated = CubicBezier2::new(
            point(ax + 6 * width, ay),
            point(ax + 4 * width, ay + 2 * height),
            point(ax + 2 * width, ay + 2 * height),
            point(ax, ay),
        );

        prop_assert_eq!(
            quadratic.relation_to_cubic(&elevated, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            elevated.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            quadratic.relation_to_cubic(&reversed_elevated, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
        prop_assert_eq!(
            reversed_elevated.relation_to_quadratic(&quadratic, &policy()),
            Classification::Decided(BezierCurveRelation::SameCurveImage)
        );
    }

    #[test]
    fn mixed_degree_midpoint_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 3 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + 6 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 6 * height),
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            quadratic.relation_to_cubic(&cubic, &policy())
        else {
            panic!("generated mixed-degree midpoint hit should be promoted exactly");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &quadratic.point_at(half()));
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree midpoint hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_quarter_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 62 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + 48 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 192 * height),
        );

        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated mixed-degree graph quarter roots should be retained completely: {relation:?}");
        };
        assert_same_parameter_regions_include_exact(&regions, &quarter);
        assert_same_parameter_regions_include_bracket(&regions);
    }

    #[test]
    fn mixed_degree_thirty_second_hits_are_generated_exact_points(
        ax in 0_i32..2,
        ay in 0_i32..2,
        width in 1_i32..4,
        height in 1_i32..3,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 15 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 31 * height),
        );

        let thirty_second = (Real::one() / Real::from(32_i8)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree thirty-second hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &quadratic.point_at(thirty_second));

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree thirty-second hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_sixty_fourth_hits_are_generated_exact_points(
        ax in 0_i32..2,
        ay in 0_i32..2,
        width in 1_i32..4,
        height in 1_i32..3,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 31 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 63 * height),
        );

        let sixty_fourth = (Real::one() / Real::from(64_i8)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree sixty-fourth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &quadratic.point_at(sixty_fourth));

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree sixty-fourth hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_one_hundred_twenty_eighth_hits_are_generated_exact_points(
        ax in 0_i32..2,
        ay in 0_i32..2,
        width in 1_i32..4,
        height in 1_i32..3,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 63 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 127 * height),
        );

        let one_hundred_twenty_eighth = (Real::one() / Real::from(128_i16)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree one-hundred-twenty-eighth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &quadratic.point_at(one_hundred_twenty_eighth)
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree one-hundred-twenty-eighth hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_two_hundred_fifty_sixth_hits_are_generated_exact_points(
        ax in 0_i32..2,
        ay in 0_i32..2,
        width in 1_i32..4,
        height in 1_i32..3,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 127 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 255 * height),
        );

        let two_hundred_fifty_sixth = (Real::one() / Real::from(256_i16)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree two-hundred-fifty-sixth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &quadratic.point_at(two_hundred_fifty_sixth)
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree two-hundred-fifty-sixth hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_five_hundred_twelfth_hits_are_generated_exact_points(
        ax in 0_i32..2,
        ay in 0_i32..2,
        width in 1_i32..4,
        height in 1_i32..3,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay + 255 * height),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 511 * height),
        );

        let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated mixed-degree five-hundred-twelfth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &quadratic.point_at(five_hundred_twelfth)
        );

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            cubic.relation_to_quadratic(&quadratic, &policy())
        else {
            panic!("generated mixed-degree five-hundred-twelfth hit should be symmetric");
        };
        prop_assert_eq!(points.len(), 1);
    }

    #[test]
    fn mixed_degree_non_dyadic_graph_roots_are_generated_isolated_regions(
        ax in -2_i32..2,
        ay in -2_i32..2,
        width in 1_i32..4,
        height in 1_i32..4,
    ) {
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            point(ax + 3 * width, ay),
            point(ax + 6 * width, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay - height),
            point(ax + 6 * width, ay - height),
        );

        let relation = quadratic.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated non-dyadic graph root should be isolated, not dropped: {relation:?}");
        };
        prop_assert_eq!(regions.len(), 1);
        prop_assert_eq!(regions[0].first(), regions[0].second());

        let relation = cubic.relation_to_quadratic(&quadratic, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated non-dyadic graph root should be symmetric: {relation:?}");
        };
        prop_assert_eq!(regions.len(), 1);
        prop_assert_eq!(regions[0].first(), regions[0].second());
    }

    #[test]
    fn cubic_non_graph_deep_dyadic_same_parameter_roots_are_generated_exact_points(
        ax in -4_i32..4,
        ay in -4_i32..4,
        sx in 1_i32..4,
        sy in 1_i32..4,
    ) {
        let first_controls = [
            point(ax, ay),
            point(ax + 30 * sx, ay + 70 * sy),
            point(ax + 60 * sx, ay - 20 * sy),
            point(ax + 90 * sx, ay + 30 * sy),
        ];
        let difference = [
            ratio(-1, 1024),
            ratio(1021, 3072),
            ratio(2045, 3072),
            ratio(1023, 1024),
        ];
        let first = CubicBezier2::new(
            first_controls[0].clone(),
            first_controls[1].clone(),
            first_controls[2].clone(),
            first_controls[3].clone(),
        );
        let second = CubicBezier2::new(
            Point2::new(
                first_controls[0].x() - &difference[0],
                first_controls[0].y() - &difference[0],
            ),
            Point2::new(
                first_controls[1].x() - &difference[1],
                first_controls[1].y() - &difference[1],
            ),
            Point2::new(
                first_controls[2].x() - &difference[2],
                first_controls[2].y() - &difference[2],
            ),
            Point2::new(
                first_controls[3].x() - &difference[3],
                first_controls[3].y() - &difference[3],
            ),
        );

        let root = ratio(1, 1024);
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated non-graph cubic deep dyadic root should be exact: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(root));
    }

    #[test]
    fn cubic_non_graph_irreducible_same_parameter_roots_are_generated_regions(
        ax in -4_i32..4,
        ay in -4_i32..4,
        sx in 1_i32..4,
        sy in 1_i32..4,
    ) {
        let first_controls = [
            point(ax, ay),
            point(ax + 30 * sx, ay + 70 * sy),
            point(ax + 60 * sx, ay - 20 * sy),
            point(ax + 90 * sx, ay + 30 * sy),
        ];
        let difference = [r(-1), r(1), r(1), r(1)];
        let first = CubicBezier2::new(
            first_controls[0].clone(),
            first_controls[1].clone(),
            first_controls[2].clone(),
            first_controls[3].clone(),
        );
        let second = CubicBezier2::new(
            Point2::new(
                first_controls[0].x() - &difference[0],
                first_controls[0].y() - &difference[0],
            ),
            Point2::new(
                first_controls[1].x() - &difference[1],
                first_controls[1].y() - &difference[1],
            ),
            Point2::new(
                first_controls[2].x() - &difference[2],
                first_controls[2].y() - &difference[2],
            ),
            Point2::new(
                first_controls[3].x() - &difference[3],
                first_controls[3].y() - &difference[3],
            ),
        );

        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated non-graph cubic irreducible root should be retained: {relation:?}");
        };
        prop_assert!(!regions.is_empty());
        let has_nonzero_same_parameter_region = regions.iter().any(|region| {
            region.first() == region.second() && region.first().start() != region.first().end()
        });
        prop_assert!(has_nonzero_same_parameter_region);
    }

    #[test]
    fn cubic_quarter_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + 4 * height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 36 * height),
        );

        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated cubic graph quarter roots should be retained completely: {relation:?}");
        };
        assert_same_parameter_regions_include_exact(&regions, &quarter);
        assert_same_parameter_regions_include_bracket(&regions);

        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) =
            second.relation_to_cubic(&first, &policy())
        else {
            panic!("generated cubic graph quarter isolation should be symmetric");
        };
        assert_same_parameter_regions_include_exact(&regions, &quarter);
        assert_same_parameter_regions_include_bracket(&regions);
    }

    #[test]
    fn endpoint_on_cubic_relation_handles_generated_five_hundred_twelfth_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let cubic = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
        let cubic_point = cubic.point_at(five_hundred_twelfth.clone());
        let probe = QuadraticBezier2::new(
            Point2::new(
                cubic_point.x().clone(),
                cubic_point.y() + Real::from(20 * height),
            ),
            Point2::new(
                cubic_point.x() + Real::from(width),
                cubic_point.y() + Real::from(10 * height),
            ),
            cubic_point.clone(),
        );

        prop_assert_eq!(
            cubic.dyadic_parameters_for_point(&cubic_point, &policy()),
            Classification::Decided(vec![five_hundred_twelfth])
        );
        let relation = probe.relation_to_cubic(&cubic, &policy());
        let Classification::Decided(BezierCurveRelation::EndpointIntersections { points }) = relation
        else {
            panic!("generated endpoint on cubic five-hundred-twelfth point should be certified: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &cubic_point);
    }

    #[test]
    fn cubic_eighth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay + 329 * height),
        );

        let eighth = (Real::one() / Real::from(8_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionRegions { regions }) = relation
        else {
            panic!("generated cubic graph eighth roots should be retained completely: {relation:?}");
        };
        assert_same_parameter_regions_include_exact(&regions, &eighth);
        assert_same_parameter_regions_include_bracket(&regions);
    }

    #[test]
    fn cubic_sixteenth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 495 * height),
        );

        let sixteenth = (Real::one() / Real::from(16_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic sixteenth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(sixteenth));
    }

    #[test]
    fn cubic_thirty_second_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 17887 * height),
        );

        let thirty_second = (Real::one() / Real::from(32_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic thirty-second hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(thirty_second));
    }

    #[test]
    fn cubic_sixty_fourth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 201663 * height),
        );

        let sixty_fourth = (Real::one() / Real::from(64_i8)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic sixty-fourth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(sixty_fourth));
    }

    #[test]
    fn cubic_one_hundred_twenty_eighth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 1_853_311 * height),
        );

        let one_hundred_twenty_eighth = (Real::one() / Real::from(128_i16)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic one-hundred-twenty-eighth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &first.point_at(one_hundred_twenty_eighth)
        );
    }

    #[test]
    fn cubic_two_hundred_fifty_sixth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 15_798_015 * height),
        );

        let two_hundred_fifty_sixth = (Real::one() / Real::from(256_i16)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic two-hundred-fifty-sixth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(
            points[0].point(),
            &first.point_at(two_hundred_fifty_sixth)
        );
    }

    #[test]
    fn cubic_five_hundred_twelfth_hits_are_generated_exact_points(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let first = CubicBezier2::new(
            point(ax, ay),
            point(ax + 2 * width, ay + 4 * height),
            point(ax + 4 * width, ay + 4 * height),
            point(ax + 6 * width, ay),
        );
        let second = CubicBezier2::new(
            point(ax, ay + height),
            point(ax + 2 * width, ay),
            point(ax + 4 * width, ay),
            point(ax + 6 * width, ay - 130_293_247 * height),
        );

        let five_hundred_twelfth = (Real::one() / Real::from(512_i16)).unwrap();
        let relation = first.relation_to_cubic(&second, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated cubic five-hundred-twelfth hit should be promoted exactly: {relation:?}");
        };
        prop_assert_eq!(points.len(), 1);
        prop_assert_eq!(points[0].point(), &first.point_at(five_hundred_twelfth));
    }

    #[test]
    fn equal_weight_rational_crossings_follow_generated_polynomial_dispatch(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        height in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let rational_arch = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay),
            point(mid_x, ay + 2 * height),
            point(end_x, ay),
            Real::one(),
        )
        .unwrap();
        let rational_crossing = RationalQuadraticBezier2::try_unit_end_weights(
            point(ax, ay + height),
            point(mid_x, ay - height),
            point(end_x, ay + height),
            Real::one(),
        )
        .unwrap();

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            rational_arch.relation_to_rational_quadratic(&rational_crossing, &policy())
        else {
            panic!("generated equal-weight rational crossings should reuse polynomial dispatch");
        };
        prop_assert_eq!(points.len(), 2);
    }

    #[test]
    fn matching_weight_rational_crossings_promote_generated_third_parameter_hits(
        ax in -16_i32..16,
        ay in -16_i32..16,
        width in 1_i32..16,
        lift in 1_i32..16,
    ) {
        let mid_x = ax + width;
        let end_x = ax + 2 * width;
        let first = RationalQuadraticBezier2::try_new(
            point(ax, ay),
            point(mid_x, ay),
            point(end_x, ay),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        )
        .unwrap();
        let second = RationalQuadraticBezier2::try_new(
            Point2::new(r(ax), r(ay + lift)),
            Point2::new(r(mid_x), Real::from(ay) - ratio(lift, 4)),
            Point2::new(r(end_x), r(ay - 2 * lift)),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        )
        .unwrap();

        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) =
            first.relation_to_rational_quadratic(&second, &policy())
        else {
            panic!("generated matching-weight rational crossings should promote exact t=1/3 hits");
        };
        prop_assert_eq!(points.len(), 1);
        let Classification::Decided(expected) = first.point_at(ratio(1, 3), &policy()) else {
            panic!("positive matching weights should evaluate at t=1/3");
        };
        prop_assert_eq!(points[0].point(), &expected);
    }

    #[test]
    fn rational_polynomial_dyadic_hits_are_generated_exact_points(
        lift in 1_i32..16,
        numerator in 1_i32..512,
    ) {
        let rational = RationalQuadraticBezier2::try_new(
            point(0, 0),
            point(256, lift),
            point(512, 0),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        )
        .unwrap();
        let parameter = ratio(numerator, 512);
        let Classification::Decided(target) = rational.point_at(parameter.clone(), &policy()) else {
            panic!("positive rational weights should evaluate at generated dyadic parameters");
        };
        let polynomial = quadratic_through_point_at(parameter, &target, (5, 7), (-3, 11));

        let relation = rational.relation_to_quadratic(&polynomial, &policy());
        let Classification::Decided(BezierCurveRelation::IntersectionPoints { points }) = relation
        else {
            panic!("generated rational/polynomial dyadic hit should be promoted exactly: {relation:?}");
        };
        prop_assert!(points.iter().any(|point| point.point() == &target));
    }

    #[test]
    fn exact_length_bounds_collapse_for_generated_axis_collinear_beziers(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(dx, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let expected = Real::from(dx - ax);

        let quadratic_bounds = quadratic.length_bounds().unwrap();
        let cubic_bounds = cubic.length_bounds().unwrap();
        let refined_quadratic = quadratic.refined_length_bounds(3).unwrap();
        let refined_cubic = cubic.refined_length_bounds(3).unwrap();
        let quadratic_prefix = quadratic
            .refined_prefix_length_bounds(half(), 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();
        let cubic_prefix = cubic
            .refined_prefix_length_bounds(half(), 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();
        let inverse_zero = cubic
            .inverse_length_parameter_region(Real::zero(), 4, 2, &policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(quadratic_bounds.lower(), &expected);
        prop_assert_eq!(quadratic_bounds.upper(), &expected);
        prop_assert!(quadratic_bounds.is_exact());
        prop_assert_eq!(cubic_bounds.lower(), &expected);
        prop_assert_eq!(cubic_bounds.upper(), &expected);
        prop_assert!(cubic_bounds.is_exact());
        prop_assert_eq!(refined_quadratic, quadratic_bounds);
        prop_assert_eq!(refined_cubic, cubic_bounds);
        prop_assert!(quadratic_prefix.is_exact());
        prop_assert!(cubic_prefix.is_exact());
        prop_assert_eq!(inverse_zero.parameter_span().start(), &Real::zero());
        prop_assert_eq!(inverse_zero.parameter_span().end(), &Real::zero());
    }

    #[test]
    fn fit_source_reports_generated_line_images_as_primitive_sources(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 1_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(dx, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));

        let quadratic_report = quadratic.fit_source_report(&policy()).unwrap();
        let cubic_report = cubic.fit_source_report(&policy()).unwrap();

        prop_assert_eq!(quadratic_report.degree(), BezierDegree::Quadratic);
        prop_assert_eq!(cubic_report.degree(), BezierDegree::Cubic);
        prop_assert!(quadratic_report.length_bounds().is_exact());
        prop_assert!(cubic_report.length_bounds().is_exact());
        prop_assert!(quadratic_report.has_exact_primitive_image());
        prop_assert!(cubic_report.has_exact_primitive_image());
        prop_assert!(!quadratic_report.needs_higher_order_fit());
        prop_assert!(!cubic_report.needs_higher_order_fit());
        prop_assert!(matches!(
            quadratic_report.exact_line_image_fit(),
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        ));
        prop_assert!(matches!(
            cubic_report.exact_line_image_fit(),
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        ));
        prop_assert!(matches!(quadratic_report.monotone_spans(), Classification::Decided(spans) if !spans.is_empty()));
        prop_assert!(matches!(cubic_report.monotone_spans(), Classification::Decided(spans) if !spans.is_empty()));
    }

    #[test]
    fn fit_source_batch_reports_generated_mixed_primitive_and_higher_order_sources(
        ax in -16_i32..16,
        ay in -16_i32..16,
        dx in 2_i32..16,
        lift in 1_i32..16,
    ) {
        let line = QuadraticBezier2::new(point(ax, ay), point(ax + dx, ay), point(ax + 2 * dx, ay));
        let arch = QuadraticBezier2::new(
            point(ax + 2 * dx, ay),
            point(ax + 3 * dx, ay + lift),
            point(ax + 4 * dx, ay),
        );

        let batch = BezierFitSourceBatchReport2::from_quadratics([&line, &arch], &policy()).unwrap();

        prop_assert_eq!(batch.segment_count(), 2);
        prop_assert_eq!(batch.exact_primitive_sources(), 1);
        prop_assert_eq!(batch.higher_order_sources(), 1);
        prop_assert_eq!(batch.uncertain_sources(), 0);
        prop_assert!(batch.all_sources_exact_rational());
        prop_assert!(batch.all_monotone_spans_decided());
        prop_assert!(matches!(
            batch.total_length_width().refine_sign_until(-64),
            Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)
        ));
    }

    #[test]
    fn fit_source_prefix_sums_match_generated_tail_batches(
        ax in -16_i32..16,
        ay in -16_i32..16,
        dx in 2_i32..16,
        lift in 1_i32..16,
    ) {
        let first = QuadraticBezier2::new(point(ax, ay), point(ax + dx, ay), point(ax + 2 * dx, ay));
        let second = QuadraticBezier2::new(
            point(ax + 2 * dx, ay),
            point(ax + 3 * dx, ay + lift),
            point(ax + 4 * dx, ay),
        );
        let third = QuadraticBezier2::new(
            point(ax + 4 * dx, ay),
            point(ax + 5 * dx, ay),
            point(ax + 6 * dx, ay),
        );
        let sources = [&first, &second, &third];
        let table = BezierFitSourcePrefixSums2::from_quadratics(sources, &policy()).unwrap();
        let tail = table.range_report(1..3).unwrap();

        prop_assert_eq!(tail.segment_count(), 2);
        prop_assert_eq!(tail.exact_primitive_sources(), 1);
        prop_assert_eq!(tail.higher_order_sources(), 1);
        prop_assert_eq!(tail.uncertain_sources(), 0);
        prop_assert!(tail.all_sources_exact_rational());
        prop_assert!(tail.all_monotone_spans_decided());
        prop_assert!(matches!(
            tail.total_length_width().refine_sign_until(-64),
            Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)
        ));
    }

    #[test]
    fn inverse_length_exact_for_generated_linear_parameterizations(
        ax in -16_i32..16,
        ay in -16_i32..16,
        dx in 3_i32..32,
        thirds in 1_i32..3,
    ) {
        let end_x = ax + dx;
        let target = (Real::from(dx * thirds) / Real::from(3_i8)).unwrap();
        let expected_parameter = (Real::from(thirds) / Real::from(3_i8)).unwrap();
        let quadratic = QuadraticBezier2::new(
            point(ax, ay),
            Point2::new(
                (Real::from((2 * ax) + dx) / Real::from(2_i8)).unwrap(),
                Real::from(ay),
            ),
            point(end_x, ay),
        );
        let cubic = CubicBezier2::new(
            point(ax, ay),
            Point2::new(
                (Real::from((3 * ax) + dx) / Real::from(3_i8)).unwrap(),
                Real::from(ay),
            ),
            Point2::new(
                (Real::from((3 * ax) + (2 * dx)) / Real::from(3_i8)).unwrap(),
                Real::from(ay),
            ),
            point(end_x, ay),
        );

        let quadratic_region = quadratic
            .inverse_length_parameter_region(target.clone(), 0, 0, &policy())
            .unwrap()
            .unwrap_decided_for_test();
        let cubic_region = cubic
            .inverse_length_parameter_region(target.clone(), 0, 0, &policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(quadratic_region.parameter_span().start(), &expected_parameter);
        prop_assert_eq!(quadratic_region.parameter_span().end(), &expected_parameter);
        prop_assert_eq!(quadratic_region.prefix_bounds_at_span_end().lower(), &target);
        prop_assert_eq!(cubic_region.parameter_span().start(), &expected_parameter);
        prop_assert_eq!(cubic_region.parameter_span().end(), &expected_parameter);
        prop_assert_eq!(cubic_region.prefix_bounds_at_span_end().upper(), &target);
    }

    #[test]
    fn signed_area_contribution_matches_generated_horizontal_line_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let quadratic = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(dx, ay));
        let cubic = CubicBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay), point(dx, ay));
        let expected = ((Real::from(-ay) * Real::from(dx - ax)) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(quadratic.signed_area_contribution().unwrap(), expected.clone());
        prop_assert_eq!(cubic.signed_area_contribution().unwrap(), expected.clone());
        prop_assert_eq!(
            quadratic
                .prefix_signed_area_contribution(Real::zero(), &policy())
                .unwrap()
                .unwrap_decided_for_test(),
            Real::zero()
        );
        prop_assert_eq!(
            cubic
                .prefix_signed_area_contribution(Real::one(), &policy())
                .unwrap()
                .unwrap_decided_for_test(),
            expected
        );
    }

    #[test]
    fn area_prefix_sums_match_generated_horizontal_line_image_totals(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let first = QuadraticBezier2::new(point(ax, ay), point(bx, ay), point(cx, ay));
        let second = QuadraticBezier2::new(point(cx, ay), point(cx + bc, ay), point(dx, ay));
        let table = BezierAreaPrefixSums2::from_quadratics([&first, &second]).unwrap();
        let expected = ((Real::from(-ay) * Real::from(dx - ax)) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(table.segment_count(), 2);
        prop_assert_eq!(table.total(), &expected);
        prop_assert_eq!(table.range_contribution(0..2).unwrap(), expected);
        prop_assert_eq!(table.range_contribution(1..1).unwrap(), Real::zero());
    }

    #[test]
    fn area_moments_match_generated_axis_aligned_line_images(
        ax in -16_i32..16,
        ay in -16_i32..16,
        bx in -16_i32..16,
        by in -16_i32..16,
    ) {
        let horizontal = CubicBezier2::new(point(ax, ay), point(ax, ay), point(bx, ay), point(bx, ay));
        let horizontal_moments = horizontal.area_moments_contribution().unwrap();
        let dx = Real::from(bx - ax);
        let horizontal_area = ((Real::from(-ay) * dx.clone()) / Real::from(2_i8)).unwrap();
        let horizontal_y_moment =
            ((Real::from(-ay * ay) * dx.clone()) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(horizontal_moments.signed_area(), &horizontal_area);
        prop_assert_eq!(horizontal_moments.x_moment(), &Real::zero());
        prop_assert_eq!(horizontal_moments.y_moment(), &horizontal_y_moment);

        let vertical = QuadraticBezier2::new(point(ax, ay), point(ax, by), point(ax, by));
        let vertical_moments = vertical.area_moments_contribution().unwrap();
        let dy = Real::from(by - ay);
        let vertical_area = ((Real::from(ax) * dy.clone()) / Real::from(2_i8)).unwrap();
        let vertical_x_moment =
            ((Real::from(ax * ax) * dy.clone()) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(vertical_moments.signed_area(), &vertical_area);
        prop_assert_eq!(vertical_moments.x_moment(), &vertical_x_moment);
        prop_assert_eq!(vertical_moments.y_moment(), &Real::zero());
    }

    #[test]
    fn area_moment_prefix_sums_match_generated_horizontal_line_image_totals(
        ax in -16_i32..16,
        ay in -16_i32..16,
        ab in 0_i32..16,
        bc in 0_i32..16,
        cd in 0_i32..16,
    ) {
        let bx = ax + ab;
        let cx = bx + bc;
        let dx = cx + cd;
        let first = CubicBezier2::new(point(ax, ay), point(ax, ay), point(bx, ay), point(cx, ay));
        let second = CubicBezier2::new(point(cx, ay), point(cx, ay), point(cx, ay), point(dx, ay));
        let table = BezierAreaMomentPrefixSums2::from_cubics([&first, &second]).unwrap();
        let total_dx = Real::from(dx - ax);
        let expected_area = ((Real::from(-ay) * total_dx.clone()) / Real::from(2_i8)).unwrap();
        let expected_y_moment =
            ((Real::from(-ay * ay) * total_dx) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(table.segment_count(), 2);
        prop_assert_eq!(table.total().signed_area(), &expected_area);
        prop_assert_eq!(table.total().x_moment(), &Real::zero());
        prop_assert_eq!(table.total().y_moment(), &expected_y_moment);
        let empty = table.range_contribution(1..1).unwrap();
        prop_assert_eq!(empty.signed_area(), &Real::zero());
    }

    #[test]
    fn display_offset_left_preserves_generated_chord_count_for_nonzero_lines(
        ax in -16_i32..16,
        ay in -16_i32..16,
        cx in -16_i32..16,
    ) {
        prop_assume!(ax != cx);
        let curve = QuadraticBezier2::new(point(ax, ay), point((ax + cx) / 2, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test()
            .simplify_exact_collinear(&policy())
            .unwrap_decided_for_test();
        let offset = polyline.display_offset_left(Real::one()).unwrap();
        let right_offset = polyline.display_offset_right(Real::one()).unwrap();

        prop_assert_eq!(offset.segments().len(), polyline.points().len() - 1);
        prop_assert_eq!(right_offset.segments().len(), polyline.points().len() - 1);
        prop_assert_eq!(offset.source_certificate().segment_count(), polyline.certificate().segment_count());
        prop_assert_eq!(right_offset.source_certificate().segment_count(), polyline.certificate().segment_count());
        prop_assert_eq!(right_offset.distance(), &Real::from(-1_i8));
    }

    #[test]
    fn checked_offset_left_preserves_generated_line_certificate(
        ax in -16_i32..16,
        ay in -16_i32..16,
        cx in -16_i32..16,
        distance in 1_i32..8,
    ) {
        prop_assume!(ax != cx);
        let curve = QuadraticBezier2::new(point(ax, ay), point((ax + cx) / 2, ay), point(cx, ay));
        let options = BezierFlatteningOptions::try_new(Real::from(1000_i32), 4, &policy()).unwrap();
        let polyline = curve
            .flatten_certified(&options, &policy())
            .unwrap_decided_for_test()
            .simplify_exact_collinear(&policy())
            .unwrap_decided_for_test();
        let offset = polyline
            .checked_offset_left(Real::from(distance), &policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(offset.curve().segments().len(), 1);
        prop_assert_eq!(offset.source_certificate().segment_count(), polyline.certificate().segment_count());
        prop_assert_eq!(offset.distance(), &Real::from(distance));
        let right_offset = polyline
            .checked_offset_right(Real::from(distance), &policy())
            .unwrap()
            .unwrap_decided_for_test();

        prop_assert_eq!(right_offset.curve().segments().len(), 1);
        prop_assert_eq!(right_offset.source_certificate().segment_count(), polyline.certificate().segment_count());
        prop_assert_eq!(right_offset.distance(), &Real::from(-distance));
    }

    #[test]
    fn generated_bezier_region_facts_classify_same_parameter_spans(
        numerator in 0_i32..64,
        width in 1_i32..64,
    ) {
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let end = &start + &(Real::from(width) / Real::from(256_i32)).unwrap();
        prop_assume!(matches!(end.refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end.clone()),
            span(start, end),
        );

        let facts = bezier_intersection_region_facts(&region);

        prop_assert_eq!(
            facts.shape,
            BezierIntersectionRegionShape::SameParameterIsolatingSpan
        );
        prop_assert_eq!(facts.first_width_status, BezierRegionWidthStatus::Positive);
        prop_assert_eq!(facts.second_width_status, BezierRegionWidthStatus::Positive);
        prop_assert_eq!(facts.same_parameter_span, Some(true));
    }

    #[test]
    fn generated_bezier_region_refinement_splits_same_parameter_span_at_midpoint(
        numerator in 0_i32..64,
        width in 1_i32..64,
    ) {
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let step = (Real::from(width) / Real::from(256_i32)).unwrap();
        let end = &start + &step;
        prop_assume!(matches!(end.refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end.clone()),
            span(start.clone(), end.clone()),
        );

        let refinement = refine_bezier_intersection_region(&region);
        let midpoint = ((&start + &end) / Real::from(2_i8)).unwrap();

        prop_assert_eq!(
            refinement.action,
            BezierIntersectionRegionRefinementAction::BisectBothSpans
        );
        prop_assert_eq!(refinement.first_midpoint, Some(midpoint.clone()));
        prop_assert_eq!(refinement.second_midpoint, Some(midpoint));
        prop_assert_eq!(refinement.children.len(), 2);
        prop_assert_eq!(refinement.children[0].first().start(), &start);
        prop_assert_eq!(refinement.children[1].first().end(), &end);
    }

    #[test]
    fn generated_bezier_region_isolation_depth_budget_retains_power_of_two_frontier(
        numerator in 0_i32..32,
        width in 1_i32..32,
        depth in 0_usize..4,
    ) {
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let end = &start + &(Real::from(width) / Real::from(128_i32)).unwrap();
        prop_assume!(matches!(end.refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end.clone()),
            span(start, end),
        );

        let report = isolate_bezier_intersection_regions(
            &[region],
            BezierIntersectionRegionIsolationBudget {
                max_steps: 128,
                max_depth: depth,
                max_terminal_regions: 128,
            },
        );

        prop_assert_eq!(
            report.stop_reason,
            BezierIntersectionRegionIsolationStopReason::WorklistExhausted
        );
        prop_assert_eq!(report.terminal_regions.len(), 1_usize << depth);
        prop_assert_eq!(report.rejected_invalid_spans, 0);
        prop_assert_eq!(report.deferred_unknown_regions, 0);
    }

    #[test]
    fn generated_bezier_region_isolation_width_target_retains_certified_frontier(
        numerator in 0_i32..16,
        width_power in 3_u32..6,
    ) {
        let width_denominator = 1_i32 << width_power;
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let end = &start + &(Real::one() / Real::from(width_denominator)).unwrap();
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let second_end = end.clone();
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end),
            span(start, second_end),
        );
        let target = (Real::one() / Real::from(width_denominator * 4)).unwrap();

        let report = isolate_bezier_intersection_regions_until_width(
            &[region],
            BezierIntersectionRegionIsolationBudget {
                max_steps: 64,
                max_depth: 4,
                max_terminal_regions: 64,
            },
            target.clone(),
        );

        prop_assert_eq!(
            report.stop_reason,
            BezierIntersectionRegionIsolationStopReason::TargetWidthReached
        );
        prop_assert_eq!(report.target_satisfied_terminal_regions, report.terminal_regions.len());
        prop_assert_eq!(report.target_unmet_terminal_regions, 0);
        for terminal in &report.terminal_regions {
            let facts = bezier_intersection_region_facts(terminal);
            prop_assert!(matches!(
                (&facts.first_width - &target).refine_sign_until(-64),
                Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
            ));
            prop_assert!(matches!(
                (&facts.second_width - &target).refine_sign_until(-64),
                Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
            ));
        }
    }

    #[test]
    fn generated_bezier_region_isolation_certificates_bound_terminal_widths(
        numerator in 0_i32..16,
        width_power in 3_u32..6,
    ) {
        let width_denominator = 1_i32 << width_power;
        let start = (Real::from(numerator) / Real::from(128_i32)).unwrap();
        let end = &start + &(Real::one() / Real::from(width_denominator)).unwrap();
        prop_assume!(matches!((&Real::one() - &end).refine_sign_until(-64), Some(hypercurve::RealSign::Positive | hypercurve::RealSign::Zero)));
        let region = hypercurve::BezierCurveIntersectionRegion::new(
            span(start.clone(), end.clone()),
            span(start, end),
        );
        let target = (Real::one() / Real::from(width_denominator * 4)).unwrap();
        let report = isolate_bezier_intersection_regions_until_width(
            &[region],
            BezierIntersectionRegionIsolationBudget {
                max_steps: 64,
                max_depth: 4,
                max_terminal_regions: 64,
            },
            target.clone(),
        );

        let certificate = certify_bezier_intersection_region_isolation(&report);

        prop_assert_eq!(certificate.terminal_region_count, report.terminal_regions.len());
        prop_assert_eq!(certificate.terminal_summary.region_count, report.terminal_regions.len());
        prop_assert!(certificate.target_width_satisfied);
        prop_assert!(certificate.all_terminal_widths_certified);
        let max_first = certificate
            .max_first_width
            .expect("certified frontier should expose a first-span maximum");
        let max_second = certificate
            .max_second_width
            .expect("certified frontier should expose a second-span maximum");
        prop_assert!(matches!(
            (&max_first - &target).refine_sign_until(-64),
            Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
        ));
        prop_assert!(matches!(
            (&max_second - &target).refine_sign_until(-64),
            Some(hypercurve::RealSign::Negative | hypercurve::RealSign::Zero)
        ));
    }
}

trait ClassificationTestExt<T> {
    fn unwrap_decided_for_test(self) -> T;
}

impl<T> ClassificationTestExt<T> for Classification<T> {
    fn unwrap_decided_for_test(self) -> T {
        match self {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("expected decided classification: {reason:?}")
            }
        }
    }
}
