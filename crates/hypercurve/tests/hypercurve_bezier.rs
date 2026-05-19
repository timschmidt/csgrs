use hypercurve::{
    Axis2, BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierBooleanHandoffReport2,
    BezierBooleanHandoffStatus, BezierCurveRelation, BezierCuspClassification, BezierDegree,
    BezierEndpoint, BezierFitBoundKind, BezierFitErrorMetric, BezierFitReadinessStatus,
    BezierFitSourceBatchReport2, BezierFitSourcePrefixSums2, BezierFlatteningOptions,
    BezierInflectionClassification, BezierIntersectionRegionIsolationBudget,
    BezierIntersectionRegionIsolationStopReason, BezierIntersectionRegionRefinementAction,
    BezierIntersectionRegionShape, BezierLineContactKind, BezierLineContactRelation,
    BezierLineFitRelation, BezierLineImageFitRelation, BezierLineRelation,
    BezierMonotoneGraphContactOrder, BezierMonotoneGraphOrder, BezierMonotoneSpan,
    BezierOffsetAdapterStatus, BezierOffsetCandidate2, BezierOffsetRisk, BezierPointFitRelation,
    BezierPointImageFitRelation, BezierRegionWidthStatus, BezierSimplificationBoundKind,
    BezierSimplificationErrorMetric, Classification, CubicBezier2, CurvePolicy, IntersectionKind,
    LineLineIntersection, LineSeg2, NumericMode, Point2, QuadraticBezier2,
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
