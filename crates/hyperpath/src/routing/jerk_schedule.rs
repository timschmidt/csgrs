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
