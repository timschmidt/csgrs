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

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, Sign, classify_real_sign, compare_reals};
use hyperreal::Real;
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, Problem, certify_candidate,
    context_from_problem,
};

use crate::tangent::{TangentJoinClass, TangentSpan, classify_tangent_join};

use super::RouteCertificationError;
use super::feed::{
    CornerLookaheadJoinClass, CornerLookaheadJoinReport, CornerLookaheadLimitReport,
    FeedPathElement,
};
use super::jerk_schedule::{PlannedMonotonicJerkTransition, plan_monotonic_jerk_transition};

/// Exact local lookahead speed proposal for a retained route.
///
/// `corner_feeds` and `corner_radii` are indexed by adjacent-span join, so
/// both vectors must have `route.len() - 1` entries. At a true corner, zero
/// radius explicitly denotes an unblended source corner and therefore
/// certifies only with zero corner feed; a positive radius denotes a retained
/// geometric blend. Exact G1 joins do not need a corner radius. Entry and exit
/// feed are attached to the path endpoints. This keeps controller lookahead
/// state as explicit retained data rather than hiding it in sampled machine
/// positions.
#[derive(Clone, Debug, PartialEq)]
pub struct LookaheadFeedSchedule {
    /// Exact candidate feed at the route entry.
    pub entry_feed: Real,
    /// Exact candidate feed at each adjacent-span join.
    pub corner_feeds: Vec<Real>,
    /// Exact retained blend radius, or zero for an unblended exact stop.
    pub corner_radii: Vec<Real>,
    /// Exact candidate feed at the route exit.
    pub exit_feed: Real,
}

/// Caller-owned feed ceilings and retained blend radii for exact lookahead
/// planning.
///
/// These are limits, not requested speed nodes. The planner may lower any node
/// during its forward and reverse reachability passes. A caller that requires
/// a stop sets that node's limit to zero. A positive radius permits a true
/// corner to receive a positive geometric speed ceiling; it does not by itself
/// override a zero caller limit.
#[derive(Clone, Debug, PartialEq)]
pub struct LookaheadFeedPlanningLimits {
    /// Maximum permitted feed at the route entry.
    pub maximum_entry_feed: Real,
    /// Per-join caller-owned feed ceilings.
    pub maximum_corner_feeds: Vec<Real>,
    /// Per-join retained geometric blend radii.
    pub corner_radii: Vec<Real>,
    /// Maximum permitted feed at the route exit.
    pub maximum_exit_feed: Real,
}

/// Exact two-pass lookahead proposal and its independent replay.
///
/// `effective_node_feed_limits` contains the caller, global, and geometric
/// ceilings after exact tangent classification. `forward_node_feeds` records
/// the acceleration-reachable forward pass. `schedule` contains the final
/// reverse-pass result, `caller_limit_certifications` replays every caller
/// ceiling, and `certification` independently replays every global, corner,
/// reversal, and span constraint through Hypersolve.
#[derive(Clone, Debug)]
pub struct PlannedLookaheadFeedSchedule {
    /// Effective entry, join, and exit limits in node order.
    pub effective_node_feed_limits: Vec<Real>,
    /// Node feeds after the exact forward acceleration pass.
    pub forward_node_feeds: Vec<Real>,
    /// Final exact feed schedule after reverse deceleration propagation.
    pub schedule: LookaheadFeedSchedule,
    /// Independent Hypersolve replay of every caller-owned node ceiling.
    pub caller_limit_certifications: Vec<CandidateCertificationReport>,
    /// Independent exact corner and transition replay.
    pub certification: LookaheadFeedScheduleReport,
}

impl PlannedLookaheadFeedSchedule {
    /// Return whether caller, corner, global, and reachability rows all replayed.
    pub fn all_satisfied(&self) -> bool {
        self.caller_limit_certifications
            .iter()
            .all(CandidateCertificationReport::all_satisfied)
            && self.certification.all_satisfied()
    }
}

/// One exact positive-node component refined as a unit for jerk feasibility.
///
/// Components are maximal contiguous runs of structurally positive speed
/// nodes in the acceleration-only schedule. Exact zero nodes separate them,
/// so scaling one component cannot create motion through a retained stop.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JerkFeasibleNodeComponent {
    /// First positive node index, inclusive.
    pub first_node_index: u64,
    /// Last positive node index, inclusive.
    pub last_node_index: u64,
    /// Number of exact divisions by two applied uniformly to this component.
    pub uniform_halvings: u32,
}

/// Exact lookahead schedule after bounded component-local jerk refinement.
///
/// `acceleration_plan` retains the original forward/reverse proposal. The
/// final `schedule` can only lower its positive nodes by exact powers of two;
/// exact zero nodes never move. Caller and lookahead reports replay the final
/// nodes independently, while every span with at least one positive boundary
/// retains a fully certified two-phase monotonic transition.
#[derive(Clone, Debug)]
pub struct PlannedJerkFeasibleLookaheadSchedule {
    /// Original exact acceleration-only forward/reverse proposal.
    pub acceleration_plan: PlannedLookaheadFeedSchedule,
    /// Maximal positive components and their exact refinement counts.
    pub positive_node_components: Vec<JerkFeasibleNodeComponent>,
    /// Final exact feed schedule after component-local refinement.
    pub schedule: LookaheadFeedSchedule,
    /// Independent Hypersolve replay of every final caller-owned node ceiling.
    pub caller_limit_certifications: Vec<CandidateCertificationReport>,
    /// Independent exact corner and acceleration-transition replay of final nodes.
    pub lookahead_certification: LookaheadFeedScheduleReport,
    /// Certified monotonic transition for each nonzero span; zero/zero spans are `None`.
    pub span_transitions: Vec<Option<PlannedMonotonicJerkTransition>>,
}

impl PlannedJerkFeasibleLookaheadSchedule {
    /// Return whether both planning layers and every retained replay succeeded.
    pub fn all_satisfied(&self) -> bool {
        self.acceleration_plan.all_satisfied()
            && self
                .caller_limit_certifications
                .iter()
                .all(CandidateCertificationReport::all_satisfied)
            && self.lookahead_certification.all_satisfied()
            && self
                .span_transitions
                .iter()
                .flatten()
                .all(PlannedMonotonicJerkTransition::all_satisfied)
    }
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

/// Plan and certify an exact forward/reverse lookahead schedule.
///
/// The proposer uses the standard squared-speed reachability relation
/// `v_next^2 <= v_current^2 + 2*a_max*length`. The forward pass propagates
/// acceleration reachability and the reverse pass propagates deceleration
/// reachability. At each true corner, the effective node limit also includes
/// `v^2 <= a_max*radius`; exact G1 joins need no geometric corner reduction,
/// while a reversal is fixed at zero. Caller-owned node limits can impose
/// additional stops or process caps.
///
/// This function constructs a candidate and then invokes
/// [`certify_lookahead_feed_schedule`] as an independent replay. No sampled
/// time step, floating proposal, or firmware behavior participates.
pub fn plan_lookahead_feed_schedule(
    route: &[FeedPathElement],
    spans: &[TangentSpan],
    limits: &LookaheadFeedPlanningLimits,
    max_feed_rate: Real,
    max_acceleration: Real,
    policy: PredicatePolicy,
) -> Result<PlannedLookaheadFeedSchedule, RouteCertificationError> {
    if route.is_empty() || spans.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    if route.len() != spans.len()
        || limits.maximum_corner_feeds.len() != route.len().saturating_sub(1)
        || limits.corner_radii.len() != route.len().saturating_sub(1)
    {
        return Err(RouteCertificationError::ScheduleShapeMismatch);
    }

    require_nonnegative_feed(&limits.maximum_entry_feed, policy)?;
    require_nonnegative_feed(&limits.maximum_exit_feed, policy)?;
    for feed in &limits.maximum_corner_feeds {
        require_nonnegative_feed(feed, policy)?;
    }
    for radius in &limits.corner_radii {
        require_nonnegative_corner_radius(radius, policy)?;
    }
    require_positive_feed(&max_feed_rate, policy)?;
    require_positive_acceleration(&max_acceleration, policy)?;

    let path_lengths = route
        .iter()
        .map(|element| element_length(element, policy))
        .collect::<Result<Vec<_>, _>>()?;
    let mut effective_node_feed_limits = Vec::with_capacity(route.len().saturating_add(1));
    effective_node_feed_limits.push(minimum_real(
        limits.maximum_entry_feed.clone(),
        max_feed_rate.clone(),
        policy,
    )?);
    for (index, pair) in spans.windows(2).enumerate() {
        let class = lookahead_join_class(&pair[0], &pair[1], policy)?;
        let geometric_limit = match class {
            CornerLookaheadJoinClass::StraightThrough => max_feed_rate.clone(),
            CornerLookaheadJoinClass::RadiusLimitedCorner => (max_acceleration.clone()
                * &limits.corner_radii[index])
                .sqrt()
                .map_err(|_| RouteCertificationError::UnsupportedRadical)?,
            CornerLookaheadJoinClass::ReversalStop => Real::zero(),
        };
        let caller_and_global = minimum_real(
            limits.maximum_corner_feeds[index].clone(),
            max_feed_rate.clone(),
            policy,
        )?;
        effective_node_feed_limits.push(minimum_real(caller_and_global, geometric_limit, policy)?);
    }
    effective_node_feed_limits.push(minimum_real(
        limits.maximum_exit_feed.clone(),
        max_feed_rate.clone(),
        policy,
    )?);

    let doubled_acceleration = Real::from(2) * &max_acceleration;
    let mut forward_node_feeds = effective_node_feed_limits.clone();
    for edge in 0..route.len() {
        let reachable = (&forward_node_feeds[edge] * &forward_node_feeds[edge]
            + &doubled_acceleration * &path_lengths[edge])
            .sqrt()
            .map_err(|_| RouteCertificationError::UnsupportedRadical)?;
        forward_node_feeds[edge + 1] =
            minimum_real(forward_node_feeds[edge + 1].clone(), reachable, policy)?;
    }

    let mut selected_node_feeds = forward_node_feeds.clone();
    for edge in (0..route.len()).rev() {
        let reachable = (&selected_node_feeds[edge + 1] * &selected_node_feeds[edge + 1]
            + &doubled_acceleration * &path_lengths[edge])
            .sqrt()
            .map_err(|_| RouteCertificationError::UnsupportedRadical)?;
        selected_node_feeds[edge] =
            minimum_real(selected_node_feeds[edge].clone(), reachable, policy)?;
    }

    let schedule = LookaheadFeedSchedule {
        entry_feed: selected_node_feeds[0].clone(),
        corner_feeds: selected_node_feeds[1..route.len()].to_vec(),
        corner_radii: limits.corner_radii.clone(),
        exit_feed: selected_node_feeds[route.len()].clone(),
    };
    let caller_limit_certifications = certify_caller_node_limits(&selected_node_feeds, limits);
    let certification = certify_lookahead_feed_schedule(
        route,
        spans,
        &schedule,
        max_feed_rate,
        max_acceleration,
        policy,
    )?;
    if !caller_limit_certifications
        .iter()
        .all(CandidateCertificationReport::all_satisfied)
        || !certification.all_satisfied()
    {
        return Err(RouteCertificationError::LookaheadProposalUncertified);
    }

    Ok(PlannedLookaheadFeedSchedule {
        effective_node_feed_limits,
        forward_node_feeds,
        schedule,
        caller_limit_certifications,
        certification,
    })
}

/// Plan exact lookahead nodes and conservatively refine them until every
/// nonzero-boundary monotonic jerk transition certifies.
///
/// The acceleration-only forward/reverse result is partitioned into maximal
/// contiguous positive-node components separated by exact zero stops. For one
/// component at a time, the planner tries the exact two-phase monotonic
/// transition on every adjacent retained span. If any replay fails only because
/// the proposal exceeds a dynamic limit, every node in that component is
/// divided by two exactly and the complete component is replayed again.
/// Relative node feeds inside the component are therefore retained and no zero
/// node can become positive. The caller bounds the refinement count; exhausting
/// it fails instead of returning an uncertified schedule.
///
/// This is deliberately conservative rather than time-optimal. It couples the
/// current monotonic transition primitive to lookahead without solving a
/// floating cubic, sampling a controller trajectory, or treating a sharp
/// corner as retained blend geometry.
#[allow(clippy::too_many_arguments)]
pub fn plan_jerk_feasible_lookahead_schedule(
    route: &[FeedPathElement],
    spans: &[TangentSpan],
    limits: &LookaheadFeedPlanningLimits,
    max_feed_rate: Real,
    max_acceleration: Real,
    max_jerk: Real,
    maximum_component_halvings: u32,
    policy: PredicatePolicy,
) -> Result<PlannedJerkFeasibleLookaheadSchedule, RouteCertificationError> {
    require_positive_jerk(&max_jerk, policy)?;
    let acceleration_plan = plan_lookahead_feed_schedule(
        route,
        spans,
        limits,
        max_feed_rate.clone(),
        max_acceleration.clone(),
        policy,
    )?;
    let mut selected_node_feeds = schedule_node_feeds(&acceleration_plan.schedule);
    let component_ranges = positive_node_components(&selected_node_feeds, policy)?;
    let mut positive_node_components = Vec::with_capacity(component_ranges.len());

    for (first_node, last_node) in component_ranges {
        let first_span = first_node.saturating_sub(1);
        let last_span = last_node.min(route.len().saturating_sub(1));
        let mut uniform_halvings = 0_u32;
        loop {
            let mut component_certifies = true;
            for span_index in first_span..=last_span {
                match plan_span_monotonic_transition(
                    &route[span_index],
                    &selected_node_feeds[span_index],
                    &selected_node_feeds[span_index + 1],
                    max_feed_rate.clone(),
                    max_acceleration.clone(),
                    max_jerk.clone(),
                    policy,
                ) {
                    Ok(Some(_)) => {}
                    Ok(None) => {
                        return Err(RouteCertificationError::JerkProposalUncertified);
                    }
                    Err(RouteCertificationError::JerkProposalUncertified) => {
                        component_certifies = false;
                        break;
                    }
                    Err(error) => return Err(error),
                }
            }
            if component_certifies {
                break;
            }
            if uniform_halvings == maximum_component_halvings {
                return Err(RouteCertificationError::JerkRefinementBudgetExceeded);
            }
            for feed in &mut selected_node_feeds[first_node..=last_node] {
                *feed = (&*feed / Real::from(2))
                    .map_err(|_| RouteCertificationError::UnsupportedDivision)?;
            }
            uniform_halvings += 1;
        }
        positive_node_components.push(JerkFeasibleNodeComponent {
            first_node_index: first_node as u64,
            last_node_index: last_node as u64,
            uniform_halvings,
        });
    }

    let schedule = schedule_from_node_feeds(
        &selected_node_feeds,
        limits.corner_radii.clone(),
        route.len(),
    );
    let caller_limit_certifications = certify_caller_node_limits(&selected_node_feeds, limits);
    let lookahead_certification = certify_lookahead_feed_schedule(
        route,
        spans,
        &schedule,
        max_feed_rate.clone(),
        max_acceleration.clone(),
        policy,
    )?;
    let span_transitions = route
        .iter()
        .enumerate()
        .map(|(index, element)| {
            plan_span_monotonic_transition(
                element,
                &selected_node_feeds[index],
                &selected_node_feeds[index + 1],
                max_feed_rate.clone(),
                max_acceleration.clone(),
                max_jerk.clone(),
                policy,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    if !caller_limit_certifications
        .iter()
        .all(CandidateCertificationReport::all_satisfied)
        || !lookahead_certification.all_satisfied()
        || !span_transitions
            .iter()
            .flatten()
            .all(PlannedMonotonicJerkTransition::all_satisfied)
    {
        return Err(RouteCertificationError::JerkProposalUncertified);
    }

    Ok(PlannedJerkFeasibleLookaheadSchedule {
        acceleration_plan,
        positive_node_components,
        schedule,
        caller_limit_certifications,
        lookahead_certification,
        span_transitions,
    })
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

    require_nonnegative_feed(&schedule.entry_feed, policy)?;
    require_nonnegative_feed(&schedule.exit_feed, policy)?;
    for feed in &schedule.corner_feeds {
        require_nonnegative_feed(feed, policy)?;
    }
    for radius in &schedule.corner_radii {
        require_nonnegative_corner_radius(radius, policy)?;
    }
    require_positive_feed(&max_feed_rate, policy)?;
    require_positive_acceleration(&max_acceleration, policy)?;

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
        let class = lookahead_join_class_from_tangent(tangent_join.class)?;
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

fn schedule_node_feeds(schedule: &LookaheadFeedSchedule) -> Vec<Real> {
    std::iter::once(schedule.entry_feed.clone())
        .chain(schedule.corner_feeds.iter().cloned())
        .chain(std::iter::once(schedule.exit_feed.clone()))
        .collect()
}

fn schedule_from_node_feeds(
    node_feeds: &[Real],
    corner_radii: Vec<Real>,
    route_len: usize,
) -> LookaheadFeedSchedule {
    LookaheadFeedSchedule {
        entry_feed: node_feeds[0].clone(),
        corner_feeds: node_feeds[1..route_len].to_vec(),
        corner_radii,
        exit_feed: node_feeds[route_len].clone(),
    }
}

fn positive_node_components(
    node_feeds: &[Real],
    policy: PredicatePolicy,
) -> Result<Vec<(usize, usize)>, RouteCertificationError> {
    let mut components = Vec::new();
    let mut node_index = 0;
    while node_index < node_feeds.len() {
        if !feed_is_positive(&node_feeds[node_index], policy)? {
            node_index += 1;
            continue;
        }
        let first_node = node_index;
        while node_index + 1 < node_feeds.len()
            && feed_is_positive(&node_feeds[node_index + 1], policy)?
        {
            node_index += 1;
        }
        components.push((first_node, node_index));
        node_index += 1;
    }
    Ok(components)
}

#[allow(clippy::too_many_arguments)]
fn plan_span_monotonic_transition(
    element: &FeedPathElement,
    start_feed: &Real,
    end_feed: &Real,
    max_feed_rate: Real,
    max_acceleration: Real,
    max_jerk: Real,
    policy: PredicatePolicy,
) -> Result<Option<PlannedMonotonicJerkTransition>, RouteCertificationError> {
    if !feed_is_positive(start_feed, policy)? && !feed_is_positive(end_feed, policy)? {
        return Ok(None);
    }
    plan_monotonic_jerk_transition(
        element,
        start_feed.clone(),
        end_feed.clone(),
        max_feed_rate,
        max_acceleration,
        max_jerk,
        policy,
    )
    .map(Some)
}

fn feed_is_positive(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<bool, RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeFeedRate),
        Some(Sign::Zero) => Ok(false),
        Some(Sign::Positive) => Ok(true),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn certify_caller_node_limits(
    selected_node_feeds: &[Real],
    limits: &LookaheadFeedPlanningLimits,
) -> Vec<CandidateCertificationReport> {
    let caller_node_limits = std::iter::once(&limits.maximum_entry_feed)
        .chain(limits.maximum_corner_feeds.iter())
        .chain(std::iter::once(&limits.maximum_exit_feed));
    selected_node_feeds
        .iter()
        .cloned()
        .zip(caller_node_limits.cloned())
        .map(|(candidate, maximum)| certify_node_limit_candidate(candidate, maximum))
        .collect()
}

fn certify_node_limit_candidate(
    candidate_feed: Real,
    maximum_feed: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let feed = problem.add_variable("planned_feed", candidate_feed);
    problem.add_constraint(feed_cap_constraint(
        "lookahead caller node feed cap",
        maximum_feed,
        feed,
        "planned_feed",
    ));
    certify_problem(problem)
}

fn element_length(
    element: &FeedPathElement,
    _policy: PredicatePolicy,
) -> Result<Real, RouteCertificationError> {
    match element {
        FeedPathElement::Line(segment) => segment
            .euclidean_length()
            .map_err(|_| RouteCertificationError::UnsupportedRouteGeometry),
        FeedPathElement::ExplicitArc(arc) => arc
            .certified_sweep_length()
            .ok_or(RouteCertificationError::UnsupportedRouteGeometry),
        FeedPathElement::CubicPh(curve) => Ok(curve.exact_length()),
        FeedPathElement::QuinticPh(curve) => Ok(curve.exact_length()),
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
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    certify_candidate(&analysis, &context)
}

fn require_nonnegative_feed(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeFeedRate),
        Some(Sign::Zero | Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn require_positive_feed(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeFeedRate),
        Some(Sign::Zero) => Err(RouteCertificationError::ZeroFeedRate),
        Some(Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn require_positive_acceleration(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeAcceleration),
        Some(Sign::Zero) => Err(RouteCertificationError::ZeroAcceleration),
        Some(Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn require_positive_jerk(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeJerk),
        Some(Sign::Zero) => Err(RouteCertificationError::ZeroJerk),
        Some(Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn require_nonnegative_corner_radius(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeCornerRadius),
        Some(Sign::Zero | Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn lookahead_join_class(
    incoming: &TangentSpan,
    outgoing: &TangentSpan,
    policy: PredicatePolicy,
) -> Result<CornerLookaheadJoinClass, RouteCertificationError> {
    let tangent_join = classify_tangent_join(
        &incoming.end,
        &incoming.end_tangent,
        &outgoing.start,
        &outgoing.start_tangent,
        policy,
    );
    lookahead_join_class_from_tangent(tangent_join.class)
}

fn lookahead_join_class_from_tangent(
    class: TangentJoinClass,
) -> Result<CornerLookaheadJoinClass, RouteCertificationError> {
    match class {
        TangentJoinClass::G1Continuous => Ok(CornerLookaheadJoinClass::StraightThrough),
        TangentJoinClass::Corner => Ok(CornerLookaheadJoinClass::RadiusLimitedCorner),
        TangentJoinClass::ReversedTangent => Ok(CornerLookaheadJoinClass::ReversalStop),
        TangentJoinClass::DegenerateTangent
        | TangentJoinClass::EndpointMismatch
        | TangentJoinClass::Unknown => Err(RouteCertificationError::UnsupportedRouteGeometry),
    }
}

fn minimum_real(
    left: Real,
    right: Real,
    policy: PredicatePolicy,
) -> Result<Real, RouteCertificationError> {
    match compare_reals(&left, &right, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => Ok(left),
        Some(Ordering::Greater) => Ok(right),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}
