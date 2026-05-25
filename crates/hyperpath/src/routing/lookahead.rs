//! Exact lookahead feed-schedule replay for retained path chains.
//!
//! This module certifies a controller-style lookahead proposal without turning
//! it into sampled motion. Retained path elements supply exact span lengths,
//! retained tangent spans supply exact join classes, and `hypersolve` replays
//! algebraic process constraints against the proposed speeds. That is the
//! object/predicate boundary advocated by Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): a numeric proposal is
//! kept as a candidate until exact predicates certify the relevant decisions.
//!
//! The span transition rows use the standard path-parameterization identity
//! `v_1^2 - v_0^2 = 2*a*L`, applied as two inequalities to bound both
//! acceleration and deceleration over a retained path length. This is the
//! same squared-speed formulation underlying Bobrow, Dubowsky, and Gibson,
//! "Time-Optimal Control of Robotic Manipulators Along Specified Paths"
//! (1985), specialized here to scalar feed along an already chosen path.

use hyperlimit::PredicatePolicy;
use hyperreal::{Real, RealSign};
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, PreparedProblem, Problem,
    certify_candidate, context_from_problem,
};

use crate::tangent::{TangentJoinClass, TangentSpan, classify_tangent_join};

use super::RouteCertificationError;
use super::feed::{
    CornerLookaheadJoinClass, CornerLookaheadJoinReport, CornerLookaheadLimitReport,
    FeedPathElement,
};

/// Exact local lookahead speed proposal for a retained route.
///
/// `corner_feeds` and `corner_radii` are indexed by adjacent-span join, so
/// both vectors must have `route.len() - 1` entries. Entry and exit feed are
/// attached to the path endpoints. This keeps controller lookahead state as
/// explicit retained data rather than hiding it in sampled machine positions.
#[derive(Clone, Debug, PartialEq)]
pub struct LookaheadFeedSchedule {
    /// Exact candidate feed at the route entry.
    pub entry_feed: Real,
    /// Exact candidate feed at each adjacent-span join.
    pub corner_feeds: Vec<Real>,
    /// Exact retained blend radius at each adjacent-span join.
    pub corner_radii: Vec<Real>,
    /// Exact candidate feed at the route exit.
    pub exit_feed: Real,
}

/// Exact acceleration-feasibility replay for one retained path element.
///
/// The replay checks feed caps at both endpoints and the symmetric
/// squared-speed travel bound `|v_1^2 - v_0^2| <= 2*a_max*L`. No time step,
/// interpolation, or floating velocity sample is introduced.
#[derive(Clone, Debug)]
pub struct LookaheadSpanTransitionReport {
    /// Zero-based path-element index.
    pub index: u64,
    /// Exact retained path-element length.
    pub path_length: Real,
    /// Exact candidate feed at the element start.
    pub start_feed: Real,
    /// Exact candidate feed at the element end.
    pub end_feed: Real,
    /// Exact maximum feed-rate cap.
    pub max_feed_rate: Real,
    /// Exact maximum acceleration/deceleration magnitude.
    pub max_acceleration: Real,
    /// Exact replay report for cap and squared-speed travel rows.
    pub certification: CandidateCertificationReport,
}

/// Exact lookahead replay report for local corner and span constraints.
#[derive(Clone, Debug)]
pub struct LookaheadFeedScheduleReport {
    /// Per-join corner speed/radius certifications.
    pub corners: CornerLookaheadLimitReport,
    /// Per-span endpoint-speed transition certifications.
    pub spans: Vec<LookaheadSpanTransitionReport>,
}

impl LookaheadFeedScheduleReport {
    /// Return whether every corner and span transition is certified.
    pub fn all_satisfied(&self) -> bool {
        self.corners.all_satisfied()
            && self
                .spans
                .iter()
                .all(|span| span.certification.all_satisfied())
    }

    /// Return the first span with a certified violation or undecided row.
    pub fn first_unsatisfied_span(&self) -> Option<usize> {
        self.spans
            .iter()
            .position(|span| !span.certification.all_satisfied())
    }
}

/// Certify a local lookahead feed schedule for a retained mixed path.
///
/// The route and tangent spans must describe the same path elements. For each
/// internal join, exact tangent predicates select whether the proposed corner
/// feed is straight-through, radius-limited, or a required stop. For each path
/// element, exact length plus endpoint speeds replay the squared-speed
/// acceleration bound. Erkorkmaz and Altintas, "High speed CNC system design.
/// Part I" (2001), discuss jerk-limited controller trajectories; this function
/// deliberately stays one layer lower by certifying the lookahead speed nodes
/// that such a trajectory generator would later consume.
pub fn certify_lookahead_feed_schedule(
    route: &[FeedPathElement],
    spans: &[TangentSpan],
    schedule: &LookaheadFeedSchedule,
    max_feed_rate: Real,
    max_acceleration: Real,
    policy: PredicatePolicy,
) -> Result<LookaheadFeedScheduleReport, RouteCertificationError> {
    if route.is_empty() || spans.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    if route.len() != spans.len()
        || schedule.corner_feeds.len() != route.len().saturating_sub(1)
        || schedule.corner_radii.len() != route.len().saturating_sub(1)
    {
        return Err(RouteCertificationError::ScheduleShapeMismatch);
    }

    require_nonnegative_feed(&schedule.entry_feed)?;
    require_nonnegative_feed(&schedule.exit_feed)?;
    for feed in &schedule.corner_feeds {
        require_nonnegative_feed(feed)?;
    }
    for radius in &schedule.corner_radii {
        require_positive_corner_radius(radius)?;
    }
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&max_acceleration)?;

    let corners = certify_local_corner_limits(
        spans,
        &schedule.corner_feeds,
        &schedule.corner_radii,
        max_feed_rate.clone(),
        max_acceleration.clone(),
        policy,
    )?;

    let mut span_reports = Vec::with_capacity(route.len());
    for (index, element) in route.iter().enumerate() {
        let path_length = element_length(element, policy)?;
        let start_feed = if index == 0 {
            schedule.entry_feed.clone()
        } else {
            schedule.corner_feeds[index - 1].clone()
        };
        let end_feed = schedule
            .corner_feeds
            .get(index)
            .cloned()
            .unwrap_or_else(|| schedule.exit_feed.clone());
        let certification = certify_span_transition_candidate(
            path_length.clone(),
            start_feed.clone(),
            end_feed.clone(),
            max_feed_rate.clone(),
            max_acceleration.clone(),
        );
        span_reports.push(LookaheadSpanTransitionReport {
            index: index as u64,
            path_length,
            start_feed,
            end_feed,
            max_feed_rate: max_feed_rate.clone(),
            max_acceleration: max_acceleration.clone(),
            certification,
        });
    }

    Ok(LookaheadFeedScheduleReport {
        corners,
        spans: span_reports,
    })
}

fn certify_local_corner_limits(
    spans: &[TangentSpan],
    corner_feeds: &[Real],
    corner_radii: &[Real],
    max_feed_rate: Real,
    max_acceleration: Real,
    policy: PredicatePolicy,
) -> Result<CornerLookaheadLimitReport, RouteCertificationError> {
    let mut joins = Vec::with_capacity(spans.len().saturating_sub(1));
    for (index, pair) in spans.windows(2).enumerate() {
        let tangent_join = classify_tangent_join(
            &pair[0].end,
            &pair[0].end_tangent,
            &pair[1].start,
            &pair[1].start_tangent,
            policy,
        );
        let class = match tangent_join.class {
            TangentJoinClass::G1Continuous => CornerLookaheadJoinClass::StraightThrough,
            TangentJoinClass::Corner => CornerLookaheadJoinClass::RadiusLimitedCorner,
            TangentJoinClass::ReversedTangent => CornerLookaheadJoinClass::ReversalStop,
            TangentJoinClass::DegenerateTangent
            | TangentJoinClass::EndpointMismatch
            | TangentJoinClass::Unknown => {
                return Err(RouteCertificationError::UnsupportedRouteGeometry);
            }
        };
        let certification = certify_corner_candidate(
            class,
            corner_feeds[index].clone(),
            max_feed_rate.clone(),
            max_acceleration.clone(),
            corner_radii[index].clone(),
        );
        joins.push(CornerLookaheadJoinReport {
            index: index as u64,
            tangent_join,
            class,
            candidate_corner_feed: corner_feeds[index].clone(),
            max_feed_rate: max_feed_rate.clone(),
            max_acceleration: max_acceleration.clone(),
            corner_radius: corner_radii[index].clone(),
            certification,
        });
    }
    Ok(CornerLookaheadLimitReport { joins })
}

fn certify_corner_candidate(
    class: CornerLookaheadJoinClass,
    candidate_corner_feed: Real,
    max_feed_rate: Real,
    max_acceleration: Real,
    corner_radius: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let feed = problem.add_variable("corner_feed", candidate_corner_feed);
    problem.add_constraint(feed_cap_constraint(
        "lookahead local corner feed cap",
        max_feed_rate,
        feed,
        "corner_feed",
    ));
    match class {
        CornerLookaheadJoinClass::StraightThrough => {}
        CornerLookaheadJoinClass::RadiusLimitedCorner => {
            let feed_expr = Expr::symbol(feed.into(), "corner_feed");
            problem.add_constraint(Constraint {
                name: "lookahead local corner centripetal limit".to_string(),
                kind: ConstraintKind::GreaterOrEqual,
                residual: Expr::real(max_acceleration * corner_radius)
                    - feed_expr.clone() * feed_expr,
                weight: Real::one(),
                active: true,
            });
        }
        CornerLookaheadJoinClass::ReversalStop => {
            problem.add_constraint(Constraint::equality(
                "lookahead local reversal stop",
                Expr::symbol(feed.into(), "corner_feed"),
            ));
        }
    }
    certify_problem(problem)
}

fn certify_span_transition_candidate(
    path_length: Real,
    start_feed: Real,
    end_feed: Real,
    max_feed_rate: Real,
    max_acceleration: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let start = problem.add_variable("start_feed", start_feed);
    let end = problem.add_variable("end_feed", end_feed);
    problem.add_constraint(feed_cap_constraint(
        "lookahead span start feed cap",
        max_feed_rate.clone(),
        start,
        "start_feed",
    ));
    problem.add_constraint(feed_cap_constraint(
        "lookahead span end feed cap",
        max_feed_rate,
        end,
        "end_feed",
    ));

    let start_expr = Expr::symbol(start.into(), "start_feed");
    let end_expr = Expr::symbol(end.into(), "end_feed");
    let travel_budget = Expr::real(Real::from(2) * max_acceleration * path_length);
    let start_sq = start_expr.clone() * start_expr;
    let end_sq = end_expr.clone() * end_expr;
    problem.add_constraint(Constraint {
        name: "lookahead span acceleration distance".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: travel_budget.clone() - (end_sq.clone() - start_sq.clone()),
        weight: Real::one(),
        active: true,
    });
    problem.add_constraint(Constraint {
        name: "lookahead span deceleration distance".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: travel_budget - (start_sq - end_sq),
        weight: Real::one(),
        active: true,
    });
    certify_problem(problem)
}

fn element_length(
    element: &FeedPathElement,
    policy: PredicatePolicy,
) -> Result<Real, RouteCertificationError> {
    match element {
        FeedPathElement::Line(segment) => segment
            .axis_length(policy)
            .ok_or(RouteCertificationError::UnsupportedRouteGeometry),
        FeedPathElement::ExplicitArc(arc) => arc
            .certified_sweep_length()
            .ok_or(RouteCertificationError::UnsupportedRouteGeometry),
    }
}

fn feed_cap_constraint(
    name: impl Into<String>,
    max_feed_rate: Real,
    feed: hypersolve::VariableId,
    symbol_name: &'static str,
) -> Constraint {
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(max_feed_rate) - Expr::symbol(feed.into(), symbol_name),
        weight: Real::one(),
        active: true,
    }
}

fn certify_problem(problem: Problem) -> CandidateCertificationReport {
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    certify_candidate(&prepared, &context)
}

fn require_nonnegative_feed(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeFeedRate),
        _ => Ok(()),
    }
}

fn require_positive_feed(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeFeedRate),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroFeedRate),
        _ => Ok(()),
    }
}

fn require_positive_acceleration(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeAcceleration),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroAcceleration),
        _ => Ok(()),
    }
}

fn require_positive_corner_radius(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeCornerRadius),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroCornerRadius),
        _ => Ok(()),
    }
}
