//! Exact jerk-ramp feed-schedule replay for retained route spans.
//!
//! This module certifies per-span motion proposals whose acceleration varies
//! linearly from `a0` to `a1` over an exact time `T`. The retained route
//! supplies exact path length, while `hypersolve` replays the kinematic
//! equalities and process inequalities without sampled controller traces. This
//! follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): constructed numeric schedules are candidates until
//! exact algebraic predicates certify them.
//!
//! The equations are the constant-jerk span identities used in jerk-limited
//! CNC trajectory generation, e.g. Erkorkmaz and Altintas, "High speed CNC
//! system design. Part I: jerk limited trajectory generation and quintic
//! spline interpolation" (2001). They are also a local path-parameterization
//! replay in the sense of Bobrow, Dubowsky, and Gibson, "Time-Optimal Control
//! of Robotic Manipulators Along Specified Paths" (1985): path geometry is
//! fixed first, then exact scalar motion constraints are certified over it.

use hyperlimit::PredicatePolicy;
use hyperreal::{Real, RealSign};
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, PreparedProblem, Problem,
    certify_candidate, context_from_problem,
};

use super::RouteCertificationError;
use super::feed::FeedPathElement;

/// Exact constant-jerk proposal for one retained path span.
#[derive(Clone, Debug, PartialEq)]
pub struct JerkRampSpanProposal {
    /// Exact feed at the span start.
    pub start_feed: Real,
    /// Exact feed at the span end.
    pub end_feed: Real,
    /// Exact signed acceleration at the span start.
    pub start_acceleration: Real,
    /// Exact signed acceleration at the span end.
    pub end_acceleration: Real,
    /// Exact traversal time for the span.
    pub traversal_time: Real,
}

/// Exact replay report for one jerk-ramp span.
#[derive(Clone, Debug)]
pub struct JerkRampSpanReport {
    /// Zero-based retained route element index.
    pub index: u64,
    /// Exact retained path length for this span.
    pub path_length: Real,
    /// Exact span proposal replayed by `hypersolve`.
    pub proposal: JerkRampSpanProposal,
    /// Exact maximum feed-rate cap.
    pub max_feed_rate: Real,
    /// Exact maximum absolute acceleration cap.
    pub max_acceleration: Real,
    /// Exact maximum absolute jerk cap.
    pub max_jerk: Real,
    /// Exact solver report for feed, acceleration, jerk, velocity, and length rows.
    pub certification: CandidateCertificationReport,
}

/// Exact report for a retained multi-span jerk-ramp schedule.
#[derive(Clone, Debug)]
pub struct JerkRampFeedScheduleReport {
    /// Per-route-element jerk-ramp certifications.
    pub spans: Vec<JerkRampSpanReport>,
}

/// Exact constant-jerk phase proposal inside one retained route element.
///
/// A multi-phase controller schedule splits one path element into exact scalar
/// phase lengths. Each phase then reuses [`JerkRampSpanProposal`] for the
/// constant-jerk kinematics, and the element-level report certifies that
/// adjacent phase endpoint states match and that phase lengths sum to the
/// retained route length.
#[derive(Clone, Debug, PartialEq)]
pub struct JerkRampPhaseProposal {
    /// Exact path length assigned to this phase.
    pub path_length: Real,
    /// Exact constant-jerk kinematic proposal for this phase.
    pub ramp: JerkRampSpanProposal,
}

/// Exact replay report for one constant-jerk phase.
#[derive(Clone, Debug)]
pub struct JerkRampPhaseReport {
    /// Zero-based phase index within the retained route element.
    pub index: u64,
    /// Exact phase proposal replayed by `hypersolve`.
    pub proposal: JerkRampPhaseProposal,
    /// Exact solver report for this phase's local kinematic rows.
    pub certification: CandidateCertificationReport,
}

/// Exact multi-phase replay report for one retained route element.
#[derive(Clone, Debug)]
pub struct JerkRampElementPhaseReport {
    /// Zero-based retained route element index.
    pub index: u64,
    /// Exact retained route element length.
    pub route_length: Real,
    /// Per-phase constant-jerk replay reports.
    pub phases: Vec<JerkRampPhaseReport>,
    /// Exact replay that phase lengths sum to [`Self::route_length`].
    pub length_certification: CandidateCertificationReport,
    /// Exact replay that adjacent phase feed and acceleration states match.
    pub continuity: Vec<CandidateCertificationReport>,
}

/// Exact retained multi-phase jerk schedule report.
#[derive(Clone, Debug)]
pub struct MultiPhaseJerkRampFeedScheduleReport {
    /// Per-route-element multi-phase certifications.
    pub elements: Vec<JerkRampElementPhaseReport>,
}

impl JerkRampFeedScheduleReport {
    /// Return whether every span proposal satisfies the exact replay rows.
    pub fn all_satisfied(&self) -> bool {
        self.spans
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

impl JerkRampElementPhaseReport {
    /// Return whether this retained element's phases, length, and continuity certify.
    pub fn all_satisfied(&self) -> bool {
        self.phases
            .iter()
            .all(|phase| phase.certification.all_satisfied())
            && self.length_certification.all_satisfied()
            && self
                .continuity
                .iter()
                .all(CandidateCertificationReport::all_satisfied)
    }

    /// Return the first phase with a certified violation or undecided row.
    pub fn first_unsatisfied_phase(&self) -> Option<usize> {
        self.phases
            .iter()
            .position(|phase| !phase.certification.all_satisfied())
    }
}

impl MultiPhaseJerkRampFeedScheduleReport {
    /// Return whether every retained element's multi-phase schedule certifies.
    pub fn all_satisfied(&self) -> bool {
        self.elements
            .iter()
            .all(JerkRampElementPhaseReport::all_satisfied)
    }

    /// Return the first retained element with a certified violation or undecided row.
    pub fn first_unsatisfied_element(&self) -> Option<usize> {
        self.elements
            .iter()
            .position(|element| !element.all_satisfied())
    }
}

/// Certify a retained per-span constant-jerk feed schedule.
///
/// Each route element is measured exactly, then paired with one
/// [`JerkRampSpanProposal`]. The replay checks endpoint feed caps, signed
/// acceleration caps, jerk cap `(j*T)^2 - (a1-a0)^2 >= 0`, the velocity
/// identity `2*(v1-v0) - (a0+a1)*T = 0`, and the denominator-free length
/// identity `6*L - 6*v0*T - (2*a0+a1)*T^2 = 0`. A midpoint feed cap is also
/// replayed from `v(T/2) = v0 + T*(3*a0+a1)/8`, which guards a common
/// antagonistic case where endpoint speeds are valid but the affine
/// acceleration proposal drives the interior feed outside the process window.
pub fn certify_jerk_ramp_feed_schedule(
    route: &[FeedPathElement],
    proposals: &[JerkRampSpanProposal],
    max_feed_rate: Real,
    max_acceleration: Real,
    max_jerk: Real,
    policy: PredicatePolicy,
) -> Result<JerkRampFeedScheduleReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    if route.len() != proposals.len() {
        return Err(RouteCertificationError::ScheduleShapeMismatch);
    }
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&max_acceleration)?;
    require_positive_jerk(&max_jerk)?;

    let mut spans = Vec::with_capacity(route.len());
    for (index, (element, proposal)) in route.iter().zip(proposals).enumerate() {
        validate_proposal(proposal)?;
        let path_length = element_length(element, policy)?;
        let certification = certify_jerk_ramp_span_candidate(
            path_length.clone(),
            proposal,
            max_feed_rate.clone(),
            max_acceleration.clone(),
            max_jerk.clone(),
        );
        spans.push(JerkRampSpanReport {
            index: index as u64,
            path_length,
            proposal: proposal.clone(),
            max_feed_rate: max_feed_rate.clone(),
            max_acceleration: max_acceleration.clone(),
            max_jerk: max_jerk.clone(),
            certification,
        });
    }

    Ok(JerkRampFeedScheduleReport { spans })
}

/// Certify a retained multi-phase constant-jerk feed schedule.
///
/// This is the multi-phase counterpart of [`certify_jerk_ramp_feed_schedule`].
/// The caller supplies an exact phase chain for each retained route element.
/// Every phase is certified by the constant-jerk rows from Erkorkmaz and
/// Altintas (2001), then the element report replays two additional exact
/// facts: phase lengths sum to the retained path length, and adjacent phases
/// have identical feed and acceleration states. Following Yap's exact
/// computation paradigm, a controller's phase decomposition remains a
/// candidate until all local kinematic, continuity, and path-length predicates
/// are certified exactly.
pub fn certify_multi_phase_jerk_ramp_feed_schedule(
    route: &[FeedPathElement],
    phases: &[Vec<JerkRampPhaseProposal>],
    max_feed_rate: Real,
    max_acceleration: Real,
    max_jerk: Real,
    policy: PredicatePolicy,
) -> Result<MultiPhaseJerkRampFeedScheduleReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    if route.len() != phases.len() || phases.iter().any(Vec::is_empty) {
        return Err(RouteCertificationError::ScheduleShapeMismatch);
    }
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&max_acceleration)?;
    require_positive_jerk(&max_jerk)?;

    let mut elements = Vec::with_capacity(route.len());
    for (index, (element, element_phases)) in route.iter().zip(phases).enumerate() {
        let route_length = element_length(element, policy)?;
        let mut phase_reports = Vec::with_capacity(element_phases.len());
        for (phase_index, phase) in element_phases.iter().enumerate() {
            validate_phase(phase)?;
            let certification = certify_jerk_ramp_span_candidate(
                phase.path_length.clone(),
                &phase.ramp,
                max_feed_rate.clone(),
                max_acceleration.clone(),
                max_jerk.clone(),
            );
            phase_reports.push(JerkRampPhaseReport {
                index: phase_index as u64,
                proposal: phase.clone(),
                certification,
            });
        }
        let length_certification =
            certify_phase_length_sum(route_length.clone(), element_phases.iter());
        let continuity = element_phases
            .windows(2)
            .map(certify_phase_continuity)
            .collect();
        elements.push(JerkRampElementPhaseReport {
            index: index as u64,
            route_length,
            phases: phase_reports,
            length_certification,
            continuity,
        });
    }

    Ok(MultiPhaseJerkRampFeedScheduleReport { elements })
}

fn certify_jerk_ramp_span_candidate(
    path_length: Real,
    proposal: &JerkRampSpanProposal,
    max_feed_rate: Real,
    max_acceleration: Real,
    max_jerk: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let start_feed = problem.add_variable("start_feed", proposal.start_feed.clone());
    let end_feed = problem.add_variable("end_feed", proposal.end_feed.clone());
    let start_acceleration =
        problem.add_variable("start_acceleration", proposal.start_acceleration.clone());
    let end_acceleration =
        problem.add_variable("end_acceleration", proposal.end_acceleration.clone());
    let time = problem.add_variable("time", proposal.traversal_time.clone());

    problem.add_constraint(feed_cap_constraint(
        "jerk ramp start feed cap",
        max_feed_rate.clone(),
        start_feed,
        "start_feed",
    ));
    problem.add_constraint(feed_cap_constraint(
        "jerk ramp end feed cap",
        max_feed_rate.clone(),
        end_feed,
        "end_feed",
    ));
    problem.add_constraint(acceleration_cap_constraint(
        "jerk ramp start acceleration cap",
        max_acceleration.clone(),
        start_acceleration,
        "start_acceleration",
    ));
    problem.add_constraint(acceleration_cap_constraint(
        "jerk ramp end acceleration cap",
        max_acceleration.clone(),
        end_acceleration,
        "end_acceleration",
    ));
    problem.add_constraint(jerk_cap_constraint(
        "jerk ramp signed jerk cap",
        max_jerk,
        start_acceleration,
        end_acceleration,
        time,
    ));
    problem.add_constraint(velocity_identity_constraint(
        start_feed,
        end_feed,
        start_acceleration,
        end_acceleration,
        time,
    ));
    problem.add_constraint(length_identity_constraint(
        path_length,
        start_feed,
        start_acceleration,
        end_acceleration,
        time,
    ));
    problem.add_constraint(midpoint_feed_nonnegative_constraint(
        start_feed,
        start_acceleration,
        end_acceleration,
        time,
    ));
    problem.add_constraint(midpoint_feed_cap_constraint(
        max_feed_rate,
        start_feed,
        start_acceleration,
        end_acceleration,
        time,
    ));

    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    certify_candidate(&prepared, &context)
}

fn certify_phase_length_sum<'a>(
    route_length: Real,
    phases: impl Iterator<Item = &'a JerkRampPhaseProposal>,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let mut sum = Expr::real(Real::zero());
    for (index, phase) in phases.enumerate() {
        let name = format!("phase_{index}_length");
        let variable = problem.add_variable(name.clone(), phase.path_length.clone());
        sum = sum + Expr::symbol(variable.into(), name);
    }
    problem.add_constraint(Constraint::equality(
        "multi phase jerk length sum",
        sum - Expr::real(route_length),
    ));
    certify_problem(problem)
}

fn certify_phase_continuity(pair: &[JerkRampPhaseProposal]) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let first_end_feed = problem.add_variable("first_end_feed", pair[0].ramp.end_feed.clone());
    let second_start_feed =
        problem.add_variable("second_start_feed", pair[1].ramp.start_feed.clone());
    let first_end_acceleration = problem.add_variable(
        "first_end_acceleration",
        pair[0].ramp.end_acceleration.clone(),
    );
    let second_start_acceleration = problem.add_variable(
        "second_start_acceleration",
        pair[1].ramp.start_acceleration.clone(),
    );
    problem.add_constraint(Constraint::equality(
        "multi phase jerk feed continuity",
        Expr::symbol(first_end_feed.into(), "first_end_feed")
            - Expr::symbol(second_start_feed.into(), "second_start_feed"),
    ));
    problem.add_constraint(Constraint::equality(
        "multi phase jerk acceleration continuity",
        Expr::symbol(first_end_acceleration.into(), "first_end_acceleration")
            - Expr::symbol(
                second_start_acceleration.into(),
                "second_start_acceleration",
            ),
    ));
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

fn acceleration_cap_constraint(
    name: impl Into<String>,
    max_acceleration: Real,
    acceleration: hypersolve::VariableId,
    symbol_name: &'static str,
) -> Constraint {
    let acceleration_expr = Expr::symbol(acceleration.into(), symbol_name);
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(max_acceleration.clone() * max_acceleration)
            - acceleration_expr.clone() * acceleration_expr,
        weight: Real::one(),
        active: true,
    }
}

fn jerk_cap_constraint(
    name: impl Into<String>,
    max_jerk: Real,
    start_acceleration: hypersolve::VariableId,
    end_acceleration: hypersolve::VariableId,
    time: hypersolve::VariableId,
) -> Constraint {
    let time_expr = Expr::symbol(time.into(), "time");
    let jerk_budget = Expr::real(max_jerk) * time_expr.clone();
    let delta_acceleration = Expr::symbol(end_acceleration.into(), "end_acceleration")
        - Expr::symbol(start_acceleration.into(), "start_acceleration");
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: jerk_budget.clone() * jerk_budget
            - delta_acceleration.clone() * delta_acceleration,
        weight: Real::one(),
        active: true,
    }
}

fn velocity_identity_constraint(
    start_feed: hypersolve::VariableId,
    end_feed: hypersolve::VariableId,
    start_acceleration: hypersolve::VariableId,
    end_acceleration: hypersolve::VariableId,
    time: hypersolve::VariableId,
) -> Constraint {
    Constraint::equality(
        "jerk ramp velocity identity",
        Expr::real(Real::from(2))
            * (Expr::symbol(end_feed.into(), "end_feed")
                - Expr::symbol(start_feed.into(), "start_feed"))
            - (Expr::symbol(start_acceleration.into(), "start_acceleration")
                + Expr::symbol(end_acceleration.into(), "end_acceleration"))
                * Expr::symbol(time.into(), "time"),
    )
}

fn length_identity_constraint(
    path_length: Real,
    start_feed: hypersolve::VariableId,
    start_acceleration: hypersolve::VariableId,
    end_acceleration: hypersolve::VariableId,
    time: hypersolve::VariableId,
) -> Constraint {
    let time_expr = Expr::symbol(time.into(), "time");
    Constraint::equality(
        "jerk ramp length identity",
        Expr::real(Real::from(6) * path_length)
            - Expr::real(Real::from(6))
                * Expr::symbol(start_feed.into(), "start_feed")
                * time_expr.clone()
            - (Expr::real(Real::from(2))
                * Expr::symbol(start_acceleration.into(), "start_acceleration")
                + Expr::symbol(end_acceleration.into(), "end_acceleration"))
                * time_expr.clone()
                * time_expr,
    )
}

fn midpoint_feed_nonnegative_constraint(
    start_feed: hypersolve::VariableId,
    start_acceleration: hypersolve::VariableId,
    end_acceleration: hypersolve::VariableId,
    time: hypersolve::VariableId,
) -> Constraint {
    Constraint {
        name: "jerk ramp midpoint feed nonnegative".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(Real::from(8)) * Expr::symbol(start_feed.into(), "start_feed")
            + Expr::symbol(time.into(), "time")
                * (Expr::real(Real::from(3))
                    * Expr::symbol(start_acceleration.into(), "start_acceleration")
                    + Expr::symbol(end_acceleration.into(), "end_acceleration")),
        weight: Real::one(),
        active: true,
    }
}

fn midpoint_feed_cap_constraint(
    max_feed_rate: Real,
    start_feed: hypersolve::VariableId,
    start_acceleration: hypersolve::VariableId,
    end_acceleration: hypersolve::VariableId,
    time: hypersolve::VariableId,
) -> Constraint {
    Constraint {
        name: "jerk ramp midpoint feed cap".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(Real::from(8) * max_feed_rate)
            - (Expr::real(Real::from(8)) * Expr::symbol(start_feed.into(), "start_feed")
                + Expr::symbol(time.into(), "time")
                    * (Expr::real(Real::from(3))
                        * Expr::symbol(start_acceleration.into(), "start_acceleration")
                        + Expr::symbol(end_acceleration.into(), "end_acceleration"))),
        weight: Real::one(),
        active: true,
    }
}

fn validate_proposal(proposal: &JerkRampSpanProposal) -> Result<(), RouteCertificationError> {
    require_nonnegative_feed(&proposal.start_feed)?;
    require_nonnegative_feed(&proposal.end_feed)?;
    match proposal.traversal_time.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeTime),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroTime),
        _ => Ok(()),
    }
}

fn validate_phase(phase: &JerkRampPhaseProposal) -> Result<(), RouteCertificationError> {
    match phase.path_length.structural_facts().sign {
        Some(RealSign::Negative) | Some(RealSign::Zero) => {
            return Err(RouteCertificationError::UnsupportedRouteGeometry);
        }
        _ => {}
    }
    validate_proposal(&phase.ramp)
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

fn require_positive_jerk(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeJerk),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroJerk),
        _ => Ok(()),
    }
}
