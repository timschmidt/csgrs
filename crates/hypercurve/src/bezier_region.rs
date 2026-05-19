//! Exact facts for retained Bezier curve/curve intersection regions.
//!
//! Bezier intersection predicates sometimes return conservative parameter
//! regions instead of represented algebraic roots. This module records what is
//! already certified about those regions before a later root-isolation pass
//! refines them. That follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): retain exact object structure and
//! report explicit uncertainty rather than choosing topology from samples. The
//! product-cell view is the standard subdivision/clipping shape used by
//! Sederberg and Nishita, "Curve intersection using Bezier clipping" (1990).

use hyperreal::{Real, RealSign};
use std::collections::VecDeque;

use crate::{BezierCurveIntersectionRegion, BezierMonotoneSpan};

/// Certified sign/width status for one retained parameter span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierRegionWidthStatus {
    /// The span endpoints are certified equal.
    Zero,
    /// The span has certified positive width.
    Positive,
    /// The span endpoints are certified reversed.
    Negative,
    /// The width sign was not certified by the current exact predicate budget.
    Unknown,
}

/// Coarse exact shape of one retained Bezier intersection region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierIntersectionRegionShape {
    /// Both parameter spans are certified zero-width.
    ExactPointCell,
    /// The same positive-width span is retained on both curves.
    SameParameterIsolatingSpan,
    /// The region is a non-degenerate product cell in the two-curve parameter plane.
    ProductCell,
    /// At least one span is certified reversed.
    InvalidSpan,
    /// The current exact predicate budget could not classify the region shape.
    Unknown,
}

/// Exact diagnostic facts for one retained Bezier intersection region.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierIntersectionRegionFacts {
    /// Width of the first curve parameter span, `first.end - first.start`.
    pub first_width: Real,
    /// Width of the second curve parameter span, `second.end - second.start`.
    pub second_width: Real,
    /// Certified sign status for `first_width`.
    pub first_width_status: BezierRegionWidthStatus,
    /// Certified sign status for `second_width`.
    pub second_width_status: BezierRegionWidthStatus,
    /// Whether the first and second parameter spans are certified identical.
    pub same_parameter_span: Option<bool>,
    /// Coarse certified region shape.
    pub shape: BezierIntersectionRegionShape,
}

/// Summary facts for a list of retained Bezier intersection regions.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierIntersectionRegionSummary {
    /// Number of input regions inspected.
    pub region_count: usize,
    /// Per-region exact facts.
    pub regions: Vec<BezierIntersectionRegionFacts>,
    /// Number of exact point cells.
    pub exact_point_cells: usize,
    /// Number of same-parameter positive-width isolating spans.
    pub same_parameter_isolating_spans: usize,
    /// Number of non-degenerate product cells.
    pub product_cells: usize,
    /// Number of certified invalid reversed spans.
    pub invalid_spans: usize,
    /// Number of regions whose shape was not certified.
    pub unknown_regions: usize,
}

/// Exact next action for a retained Bezier intersection region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierIntersectionRegionRefinementAction {
    /// The region is already represented by a zero-width parameter cell.
    RetainExactPoint,
    /// Bisect the first curve parameter span.
    BisectFirstSpan,
    /// Bisect the second curve parameter span.
    BisectSecondSpan,
    /// Bisect both parameter spans.
    BisectBothSpans,
    /// Drop the region because at least one span is certified reversed.
    RejectInvalidSpan,
    /// Leave the region untouched until a stronger predicate budget is available.
    DeferUnknown,
}

/// Exact bisection proposal for one retained Bezier intersection region.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierIntersectionRegionRefinement {
    /// Certified facts used to choose the action.
    pub facts: BezierIntersectionRegionFacts,
    /// The exact next refinement action.
    pub action: BezierIntersectionRegionRefinementAction,
    /// Midpoint of the first span when that span is bisected.
    pub first_midpoint: Option<Real>,
    /// Midpoint of the second span when that span is bisected.
    pub second_midpoint: Option<Real>,
    /// Child regions produced by exact bisection.
    pub children: Vec<BezierCurveIntersectionRegion>,
}

/// Budget for bounded retained-region isolation refinement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierIntersectionRegionIsolationBudget {
    /// Maximum number of refinement rounds popped from the worklist.
    pub max_steps: usize,
    /// Maximum depth assigned to any child region.
    pub max_depth: usize,
    /// Maximum number of live terminal regions to retain in the report.
    pub max_terminal_regions: usize,
}

/// Reason a bounded retained-region isolation pass stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierIntersectionRegionIsolationStopReason {
    /// The worklist was exhausted inside the supplied budget.
    WorklistExhausted,
    /// Every retained terminal region satisfied the requested width target.
    TargetWidthReached,
    /// The pass reached [`BezierIntersectionRegionIsolationBudget::max_steps`].
    StepBudgetReached,
    /// Adding more retained terminal regions would exceed
    /// [`BezierIntersectionRegionIsolationBudget::max_terminal_regions`].
    TerminalRegionBudgetReached,
    /// The requested width target was certified negative or not certified.
    InvalidTargetWidth,
}

/// Report from bounded retained-region isolation refinement.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierIntersectionRegionIsolationReport {
    /// Caller-supplied refinement budget.
    pub budget: BezierIntersectionRegionIsolationBudget,
    /// Optional exact maximum terminal parameter-span width requested by the caller.
    pub target_max_span_width: Option<Real>,
    /// Reason the pass stopped.
    pub stop_reason: BezierIntersectionRegionIsolationStopReason,
    /// Number of regions popped from the worklist and inspected.
    pub steps: usize,
    /// Number of exact point cells retained.
    pub exact_point_cells: usize,
    /// Number of invalid regions rejected.
    pub rejected_invalid_spans: usize,
    /// Number of regions deferred because the current exact predicate budget
    /// could not certify their shape or width order.
    pub deferred_unknown_regions: usize,
    /// Number of terminal regions that satisfied `target_max_span_width`.
    pub target_satisfied_terminal_regions: usize,
    /// Number of terminal regions retained before satisfying `target_max_span_width`.
    pub target_unmet_terminal_regions: usize,
    /// Terminal regions kept for the next algebraic or subdivision pass.
    pub terminal_regions: Vec<BezierCurveIntersectionRegion>,
    /// Refinement actions in deterministic worklist order.
    pub refinements: Vec<BezierIntersectionRegionRefinement>,
}

/// Compact certificate for a retained-region isolation frontier.
///
/// This is the audit object downstream Bezier/conic solvers can carry instead
/// of re-walking every terminal parameter cell. It records the certified shape
/// summary, certified maximum first/second span widths when available, and
/// whether the caller's target-width contract was actually met. The certificate
/// deliberately does not promote a small cell to an intersection point; it
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), by keeping approximation/refinement facts separate
/// from combinatorial topology. The frontier shape remains the
/// subdivision/clipping product-cell view of Sederberg and Nishita, "Curve
/// intersection using Bezier clipping" (1990).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierIntersectionRegionIsolationCertificate {
    /// Stop reason from the source isolation report.
    pub stop_reason: BezierIntersectionRegionIsolationStopReason,
    /// Optional exact target maximum span width requested by the caller.
    pub target_max_span_width: Option<Real>,
    /// Whether every terminal region certified the requested target width.
    pub target_width_satisfied: bool,
    /// Number of terminal regions summarized by this certificate.
    pub terminal_region_count: usize,
    /// Exact summary of the retained terminal regions.
    pub terminal_summary: BezierIntersectionRegionSummary,
    /// Maximum certified first-parameter width across retained terminal cells.
    pub max_first_width: Option<Real>,
    /// Maximum certified second-parameter width across retained terminal cells.
    pub max_second_width: Option<Real>,
    /// Whether every retained terminal width had a certified nonnegative sign.
    pub all_terminal_widths_certified: bool,
    /// Number of source worklist regions inspected by the isolation pass.
    pub source_steps: usize,
    /// Number of invalid spans rejected before the frontier.
    pub rejected_invalid_spans: usize,
    /// Number of unknown regions retained or deferred before a stronger predicate budget.
    pub deferred_unknown_regions: usize,
}

/// Computes exact diagnostic facts for one retained Bezier intersection region.
///
/// The result is not a topology decision. It classifies only the retained
/// parameter-cell shape: represented point, same-parameter bracket, product
/// cell, invalid reversed span, or unknown. Later Bezier/conic solvers can use
/// these facts to schedule algebraic refinement without re-probing scalar
/// endpoints.
pub fn bezier_intersection_region_facts(
    region: &BezierCurveIntersectionRegion,
) -> BezierIntersectionRegionFacts {
    let first_width = region.first().end() - region.first().start();
    let second_width = region.second().end() - region.second().start();
    let first_width_status = width_status(&first_width);
    let second_width_status = width_status(&second_width);
    let same_parameter_span = same_span(region);
    let shape = match (first_width_status, second_width_status, same_parameter_span) {
        (BezierRegionWidthStatus::Negative, _, _) | (_, BezierRegionWidthStatus::Negative, _) => {
            BezierIntersectionRegionShape::InvalidSpan
        }
        (BezierRegionWidthStatus::Unknown, _, _)
        | (_, BezierRegionWidthStatus::Unknown, _)
        | (_, _, None) => BezierIntersectionRegionShape::Unknown,
        (BezierRegionWidthStatus::Zero, BezierRegionWidthStatus::Zero, _) => {
            BezierIntersectionRegionShape::ExactPointCell
        }
        (BezierRegionWidthStatus::Positive, BezierRegionWidthStatus::Positive, Some(true)) => {
            BezierIntersectionRegionShape::SameParameterIsolatingSpan
        }
        _ => BezierIntersectionRegionShape::ProductCell,
    };

    BezierIntersectionRegionFacts {
        first_width,
        second_width,
        first_width_status,
        second_width_status,
        same_parameter_span,
        shape,
    }
}

/// Summarizes exact diagnostic facts for retained Bezier intersection regions.
pub fn summarize_bezier_intersection_regions(
    regions: &[BezierCurveIntersectionRegion],
) -> BezierIntersectionRegionSummary {
    let facts = regions
        .iter()
        .map(bezier_intersection_region_facts)
        .collect::<Vec<_>>();
    let mut summary = BezierIntersectionRegionSummary {
        region_count: regions.len(),
        regions: facts,
        exact_point_cells: 0,
        same_parameter_isolating_spans: 0,
        product_cells: 0,
        invalid_spans: 0,
        unknown_regions: 0,
    };
    for facts in &summary.regions {
        match facts.shape {
            BezierIntersectionRegionShape::ExactPointCell => summary.exact_point_cells += 1,
            BezierIntersectionRegionShape::SameParameterIsolatingSpan => {
                summary.same_parameter_isolating_spans += 1;
            }
            BezierIntersectionRegionShape::ProductCell => summary.product_cells += 1,
            BezierIntersectionRegionShape::InvalidSpan => summary.invalid_spans += 1,
            BezierIntersectionRegionShape::Unknown => summary.unknown_regions += 1,
        }
    }
    summary
}

/// Builds an exact bisection proposal for a retained Bezier intersection region.
///
/// This is a scheduler, not a curve/curve solver. Exact point cells are kept,
/// invalid reversed spans are rejected, unknown regions are deferred, and
/// positive-width regions are bisected using exact midpoint arithmetic. The
/// subdivision policy follows the product-cell refinement used by Sederberg and
/// Nishita, "Curve intersection using Bezier clipping" (1990), while the
/// explicit reject/defer/report boundary follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
pub fn refine_bezier_intersection_region(
    region: &BezierCurveIntersectionRegion,
) -> BezierIntersectionRegionRefinement {
    let facts = bezier_intersection_region_facts(region);
    let mut refinement = BezierIntersectionRegionRefinement {
        facts,
        action: BezierIntersectionRegionRefinementAction::DeferUnknown,
        first_midpoint: None,
        second_midpoint: None,
        children: Vec::new(),
    };

    refinement.action = match refinement.facts.shape {
        BezierIntersectionRegionShape::ExactPointCell => {
            refinement.children.push(region.clone());
            BezierIntersectionRegionRefinementAction::RetainExactPoint
        }
        BezierIntersectionRegionShape::InvalidSpan => {
            BezierIntersectionRegionRefinementAction::RejectInvalidSpan
        }
        BezierIntersectionRegionShape::Unknown => {
            BezierIntersectionRegionRefinementAction::DeferUnknown
        }
        BezierIntersectionRegionShape::SameParameterIsolatingSpan => {
            bisect_same_parameter_region(region, &mut refinement);
            BezierIntersectionRegionRefinementAction::BisectBothSpans
        }
        BezierIntersectionRegionShape::ProductCell => {
            bisect_product_region(region, &mut refinement)
        }
    };
    refinement
}

/// Builds exact bisection proposals for a retained-region batch.
pub fn refine_bezier_intersection_regions(
    regions: &[BezierCurveIntersectionRegion],
) -> Vec<BezierIntersectionRegionRefinement> {
    regions
        .iter()
        .map(refine_bezier_intersection_region)
        .collect()
}

/// Runs bounded exact bisection over retained Bezier intersection regions.
///
/// This pass is intentionally conservative: it refines only the parameter
/// boxes already retained by earlier predicates and never evaluates geometry
/// from samples. Exact point cells and depth-limited cells are retained,
/// invalid cells are rejected, and unknown cells are deferred with an explicit
/// report. That is the exact-computation contract advocated by Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
/// worklist bisection is the report-bearing analogue of the subdivision cells
/// used by Sederberg and Nishita, "Curve intersection using Bezier clipping"
/// (1990).
pub fn isolate_bezier_intersection_regions(
    regions: &[BezierCurveIntersectionRegion],
    budget: BezierIntersectionRegionIsolationBudget,
) -> BezierIntersectionRegionIsolationReport {
    isolate_bezier_intersection_regions_with_target(regions, budget, None)
}

/// Runs bounded retained-region isolation until exact parameter widths satisfy a target.
///
/// A terminal region satisfies `max_span_width` only when both parameter-span
/// widths are certified less than or equal to that exact value. This exposes a
/// caller-controlled algebraic-refinement frontier without turning a small box
/// into a point. That separation is the key API boundary in Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// numerical narrowing is reported as certified information, not silently
/// promoted to a topological decision.
pub fn isolate_bezier_intersection_regions_until_width(
    regions: &[BezierCurveIntersectionRegion],
    budget: BezierIntersectionRegionIsolationBudget,
    max_span_width: Real,
) -> BezierIntersectionRegionIsolationReport {
    match max_span_width.refine_sign_until(-64) {
        Some(RealSign::Positive) | Some(RealSign::Zero) => {
            isolate_bezier_intersection_regions_with_target(regions, budget, Some(max_span_width))
        }
        Some(RealSign::Negative) | None => BezierIntersectionRegionIsolationReport {
            budget,
            target_max_span_width: Some(max_span_width),
            stop_reason: BezierIntersectionRegionIsolationStopReason::InvalidTargetWidth,
            steps: 0,
            exact_point_cells: 0,
            rejected_invalid_spans: 0,
            deferred_unknown_regions: 0,
            target_satisfied_terminal_regions: 0,
            target_unmet_terminal_regions: 0,
            terminal_regions: Vec::new(),
            refinements: Vec::new(),
        },
    }
}

/// Builds a compact certificate over a retained-region isolation report.
///
/// The certificate is a replayable summary of exact facts already present in
/// the terminal frontier: certified shape counts, maximum certified parameter
/// widths, rejected invalid spans, and target-width satisfaction. It is useful
/// as a handoff to a later algebraic root isolator because it preserves the
/// exact refinement frontier without treating terminal boxes as solved roots.
pub fn certify_bezier_intersection_region_isolation(
    report: &BezierIntersectionRegionIsolationReport,
) -> BezierIntersectionRegionIsolationCertificate {
    let terminal_summary = summarize_bezier_intersection_regions(&report.terminal_regions);
    let mut max_first_width = None;
    let mut max_second_width = None;
    let mut all_terminal_widths_certified = true;

    for facts in &terminal_summary.regions {
        match facts.first_width_status {
            BezierRegionWidthStatus::Zero | BezierRegionWidthStatus::Positive => {
                update_max_width(&mut max_first_width, &facts.first_width);
            }
            BezierRegionWidthStatus::Negative | BezierRegionWidthStatus::Unknown => {
                all_terminal_widths_certified = false;
            }
        }
        match facts.second_width_status {
            BezierRegionWidthStatus::Zero | BezierRegionWidthStatus::Positive => {
                update_max_width(&mut max_second_width, &facts.second_width);
            }
            BezierRegionWidthStatus::Negative | BezierRegionWidthStatus::Unknown => {
                all_terminal_widths_certified = false;
            }
        }
    }

    let target_width_satisfied = match report.target_max_span_width.as_ref() {
        Some(_) => {
            report.target_unmet_terminal_regions == 0
                && report.deferred_unknown_regions == 0
                && report.stop_reason
                    == BezierIntersectionRegionIsolationStopReason::TargetWidthReached
        }
        None => false,
    };

    BezierIntersectionRegionIsolationCertificate {
        stop_reason: report.stop_reason,
        target_max_span_width: report.target_max_span_width.clone(),
        target_width_satisfied,
        terminal_region_count: report.terminal_regions.len(),
        terminal_summary,
        max_first_width,
        max_second_width,
        all_terminal_widths_certified,
        source_steps: report.steps,
        rejected_invalid_spans: report.rejected_invalid_spans,
        deferred_unknown_regions: report.deferred_unknown_regions,
    }
}

fn isolate_bezier_intersection_regions_with_target(
    regions: &[BezierCurveIntersectionRegion],
    budget: BezierIntersectionRegionIsolationBudget,
    target_max_span_width: Option<Real>,
) -> BezierIntersectionRegionIsolationReport {
    let mut worklist = VecDeque::new();
    for region in regions {
        worklist.push_back((region.clone(), 0_usize));
    }

    let mut report = BezierIntersectionRegionIsolationReport {
        budget,
        target_max_span_width,
        stop_reason: BezierIntersectionRegionIsolationStopReason::WorklistExhausted,
        steps: 0,
        exact_point_cells: 0,
        rejected_invalid_spans: 0,
        deferred_unknown_regions: 0,
        target_satisfied_terminal_regions: 0,
        target_unmet_terminal_regions: 0,
        terminal_regions: Vec::new(),
        refinements: Vec::new(),
    };

    while let Some((region, depth)) = worklist.pop_front() {
        if report.steps >= budget.max_steps {
            report.stop_reason = BezierIntersectionRegionIsolationStopReason::StepBudgetReached;
            break;
        }
        report.steps += 1;

        if let Some(target) = report.target_max_span_width.clone() {
            let facts = bezier_intersection_region_facts(&region);
            match facts.shape {
                BezierIntersectionRegionShape::ExactPointCell => {
                    report.exact_point_cells += 1;
                    if !push_terminal_with_target_status(&mut report, region, Some(true)) {
                        break;
                    }
                    continue;
                }
                BezierIntersectionRegionShape::InvalidSpan => {
                    report.rejected_invalid_spans += 1;
                    continue;
                }
                BezierIntersectionRegionShape::Unknown => {
                    report.deferred_unknown_regions += 1;
                    if !push_terminal_with_target_status(&mut report, region, None) {
                        break;
                    }
                    continue;
                }
                _ if region_within_target_width(&facts, &target) => {
                    if !push_terminal_with_target_status(&mut report, region, Some(true)) {
                        break;
                    }
                    continue;
                }
                _ => {}
            }
        }

        let refinement = refine_bezier_intersection_region(&region);
        match refinement.action {
            BezierIntersectionRegionRefinementAction::RetainExactPoint => {
                report.exact_point_cells += 1;
                if !push_terminal_with_target_status(&mut report, region, Some(true)) {
                    break;
                }
            }
            BezierIntersectionRegionRefinementAction::RejectInvalidSpan => {
                report.rejected_invalid_spans += 1;
            }
            BezierIntersectionRegionRefinementAction::DeferUnknown => {
                report.deferred_unknown_regions += 1;
                if !push_terminal_with_target_status(&mut report, region, None) {
                    break;
                }
            }
            BezierIntersectionRegionRefinementAction::BisectFirstSpan
            | BezierIntersectionRegionRefinementAction::BisectSecondSpan
            | BezierIntersectionRegionRefinementAction::BisectBothSpans => {
                if depth >= budget.max_depth {
                    let target_status = report
                        .target_max_span_width
                        .as_ref()
                        .map(|target| region_within_target_width(&refinement.facts, target));
                    if !push_terminal_with_target_status(&mut report, region, target_status) {
                        break;
                    }
                } else {
                    for child in &refinement.children {
                        worklist.push_back((child.clone(), depth + 1));
                    }
                }
            }
        }
        report.refinements.push(refinement);
    }

    if report.target_max_span_width.is_some()
        && report.stop_reason == BezierIntersectionRegionIsolationStopReason::WorklistExhausted
        && report.target_unmet_terminal_regions == 0
        && report.deferred_unknown_regions == 0
    {
        report.stop_reason = BezierIntersectionRegionIsolationStopReason::TargetWidthReached;
    }

    report
}

fn width_status(width: &Real) -> BezierRegionWidthStatus {
    match width.refine_sign_until(-64) {
        Some(RealSign::Zero) => BezierRegionWidthStatus::Zero,
        Some(RealSign::Positive) => BezierRegionWidthStatus::Positive,
        Some(RealSign::Negative) => BezierRegionWidthStatus::Negative,
        None => BezierRegionWidthStatus::Unknown,
    }
}

fn same_span(region: &BezierCurveIntersectionRegion) -> Option<bool> {
    let starts = region.first().start() - region.second().start();
    let ends = region.first().end() - region.second().end();
    match (starts.refine_sign_until(-64), ends.refine_sign_until(-64)) {
        (Some(RealSign::Zero), Some(RealSign::Zero)) => Some(true),
        (Some(_), Some(_)) => Some(false),
        _ => None,
    }
}

fn push_terminal_with_target_status(
    report: &mut BezierIntersectionRegionIsolationReport,
    region: BezierCurveIntersectionRegion,
    target_satisfied: Option<bool>,
) -> bool {
    if report.terminal_regions.len() >= report.budget.max_terminal_regions {
        report.stop_reason =
            BezierIntersectionRegionIsolationStopReason::TerminalRegionBudgetReached;
        return false;
    }
    match target_satisfied {
        Some(true) => report.target_satisfied_terminal_regions += 1,
        Some(false) => report.target_unmet_terminal_regions += 1,
        None => {}
    }
    report.terminal_regions.push(region);
    true
}

fn region_within_target_width(facts: &BezierIntersectionRegionFacts, target: &Real) -> bool {
    width_within_target(&facts.first_width, target)
        && width_within_target(&facts.second_width, target)
}

fn width_within_target(width: &Real, target: &Real) -> bool {
    matches!(
        (width - target).refine_sign_until(-64),
        Some(RealSign::Negative | RealSign::Zero)
    )
}

fn update_max_width(current: &mut Option<Real>, candidate: &Real) {
    let replace = match current.as_ref() {
        Some(current) => matches!(
            (candidate - current).refine_sign_until(-64),
            Some(RealSign::Positive)
        ),
        None => true,
    };
    if replace {
        *current = Some(candidate.clone());
    }
}

fn midpoint(span: &BezierMonotoneSpan) -> Real {
    ((span.start() + span.end()) / Real::from(2_i8)).unwrap()
}

fn split_span(span: &BezierMonotoneSpan, mid: &Real) -> (BezierMonotoneSpan, BezierMonotoneSpan) {
    (
        BezierMonotoneSpan::new(span.start().clone(), mid.clone()),
        BezierMonotoneSpan::new(mid.clone(), span.end().clone()),
    )
}

fn bisect_same_parameter_region(
    region: &BezierCurveIntersectionRegion,
    refinement: &mut BezierIntersectionRegionRefinement,
) {
    let first_mid = midpoint(region.first());
    let second_mid = midpoint(region.second());
    let (first_left, first_right) = split_span(region.first(), &first_mid);
    let (second_left, second_right) = split_span(region.second(), &second_mid);
    refinement.first_midpoint = Some(first_mid);
    refinement.second_midpoint = Some(second_mid);
    refinement
        .children
        .push(BezierCurveIntersectionRegion::new(first_left, second_left));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_right,
        second_right,
    ));
}

fn bisect_product_region(
    region: &BezierCurveIntersectionRegion,
    refinement: &mut BezierIntersectionRegionRefinement,
) -> BezierIntersectionRegionRefinementAction {
    match (
        refinement.facts.first_width_status,
        refinement.facts.second_width_status,
    ) {
        (BezierRegionWidthStatus::Positive, BezierRegionWidthStatus::Zero) => {
            bisect_first(region, refinement);
            BezierIntersectionRegionRefinementAction::BisectFirstSpan
        }
        (BezierRegionWidthStatus::Zero, BezierRegionWidthStatus::Positive) => {
            bisect_second(region, refinement);
            BezierIntersectionRegionRefinementAction::BisectSecondSpan
        }
        (BezierRegionWidthStatus::Positive, BezierRegionWidthStatus::Positive) => {
            match width_order(
                &refinement.facts.first_width,
                &refinement.facts.second_width,
            ) {
                Some(RealSign::Positive) => {
                    bisect_first(region, refinement);
                    BezierIntersectionRegionRefinementAction::BisectFirstSpan
                }
                Some(RealSign::Negative) => {
                    bisect_second(region, refinement);
                    BezierIntersectionRegionRefinementAction::BisectSecondSpan
                }
                Some(RealSign::Zero) => {
                    bisect_both_product(region, refinement);
                    BezierIntersectionRegionRefinementAction::BisectBothSpans
                }
                None => BezierIntersectionRegionRefinementAction::DeferUnknown,
            }
        }
        _ => BezierIntersectionRegionRefinementAction::DeferUnknown,
    }
}

fn width_order(first_width: &Real, second_width: &Real) -> Option<RealSign> {
    (first_width - second_width).refine_sign_until(-64)
}

fn bisect_first(
    region: &BezierCurveIntersectionRegion,
    refinement: &mut BezierIntersectionRegionRefinement,
) {
    let first_mid = midpoint(region.first());
    let (first_left, first_right) = split_span(region.first(), &first_mid);
    refinement.first_midpoint = Some(first_mid);
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_left,
        region.second().clone(),
    ));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_right,
        region.second().clone(),
    ));
}

fn bisect_second(
    region: &BezierCurveIntersectionRegion,
    refinement: &mut BezierIntersectionRegionRefinement,
) {
    let second_mid = midpoint(region.second());
    let (second_left, second_right) = split_span(region.second(), &second_mid);
    refinement.second_midpoint = Some(second_mid);
    refinement.children.push(BezierCurveIntersectionRegion::new(
        region.first().clone(),
        second_left,
    ));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        region.first().clone(),
        second_right,
    ));
}

fn bisect_both_product(
    region: &BezierCurveIntersectionRegion,
    refinement: &mut BezierIntersectionRegionRefinement,
) {
    let first_mid = midpoint(region.first());
    let second_mid = midpoint(region.second());
    let (first_left, first_right) = split_span(region.first(), &first_mid);
    let (second_left, second_right) = split_span(region.second(), &second_mid);
    refinement.first_midpoint = Some(first_mid);
    refinement.second_midpoint = Some(second_mid);
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_left.clone(),
        second_left.clone(),
    ));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_left,
        second_right.clone(),
    ));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_right.clone(),
        second_left,
    ));
    refinement.children.push(BezierCurveIntersectionRegion::new(
        first_right,
        second_right,
    ));
}
