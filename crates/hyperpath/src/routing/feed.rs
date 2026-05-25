//! Exact feed-rate replay reports beyond constant and acceleration-only moves.
//!
//! This submodule keeps controller-style feed proposals in the path domain:
//! retained route geometry supplies exact length, while `hypersolve` replays
//! algebraic residuals and inequalities. That is the boundary advocated by
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): constructed candidates are not accepted until exact predicates
//! certify them. The specific four-phase S-curve profile is the standard
//! jerk-limited rest-to-rest `+j, -j, -j, +j` motion law used in CNC/robotics
//! time-parameterization literature, e.g. Erkorkmaz and Altintas, "High speed
//! CNC system design. Part I: jerk limited trajectory generation and quintic
//! spline interpolation" (2001), and the path-parameterization framing of
//! Bobrow, Dubowsky, and Gibson, "Time-Optimal Control of Robotic Manipulators
//! Along Specified Paths" (1985). Farouki's PH-curve work motivates carrying
//! feed laws as algebraic objects compatible with later exact curve-length
//! packages instead of as sampled controller traces.

use hyperlimit::PredicatePolicy;
use hyperreal::{Real, RealSign};
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, PreparedProblem, Problem,
    certify_candidate, context_from_problem,
};

use crate::segment::LinePathSegment;
use crate::solve::symmetric_jerk_limited_feed_time_equation;

use super::{RouteCertificationError, route_axis_length};

/// Exact symmetric jerk-limited traversal time certification report.
///
/// The retained profile has four equal-duration jerk phases with signs
/// `+j, -j, -j, +j`, starting and ending at rest. Exact integration gives:
///
/// - path length: `L = j*T^3/32`
/// - peak feed: `v_peak = j*T^2/16`
/// - peak acceleration: `a_peak = j*T/4`
///
/// The report replays the length equality plus denominator-free peak-limit
/// inequalities through `hypersolve`. It does not certify lookahead blending,
/// segment-corner dynamics, cutter engagement, machine envelopes, or material
/// process constraints; those remain separate exact reports before CAM output.
#[derive(Clone, Debug)]
pub struct JerkLimitedFeedTimeReport {
    /// Exact retained route length.
    pub path_length: Real,
    /// Exact maximum allowed feed rate.
    pub max_feed_rate: Real,
    /// Exact maximum allowed acceleration.
    pub max_acceleration: Real,
    /// Exact jerk magnitude used by the replay profile.
    pub jerk: Real,
    /// Exact candidate traversal time.
    pub target_time: Real,
    /// Exact peak feed implied by `jerk` and `target_time`.
    pub peak_feed_rate: Real,
    /// Exact peak acceleration implied by `jerk` and `target_time`.
    pub peak_acceleration: Real,
    /// Exact solver replay report for length and peak-limit constraints.
    pub certification: CandidateCertificationReport,
}

/// Certify a symmetric four-phase jerk-limited feed traversal time.
///
/// The retained route is measured as exact axis-aligned length. The candidate
/// time is then certified against the jerk-limited S-curve residual
/// `j*T^3 - 32*L = 0`; peak feed and peak acceleration are checked as exact
/// inequality rows `16*v_max - j*T^2 >= 0` and `4*a_max - j*T >= 0`.
///
/// A report may therefore be returned with certified violations when the
/// supplied time or limits are inconsistent. Structural input failures, such as
/// negative limits or non-axis-aligned retained geometry, are returned as
/// [`RouteCertificationError`] values before building the replay problem.
pub fn certify_symmetric_jerk_limited_feed_time(
    route: &[LinePathSegment],
    max_feed_rate: Real,
    max_acceleration: Real,
    jerk: Real,
    target_time: Real,
    policy: PredicatePolicy,
) -> Result<JerkLimitedFeedTimeReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&max_acceleration)?;
    require_positive_jerk(&jerk)?;
    if target_time.structural_facts().sign == Some(RealSign::Negative) {
        return Err(RouteCertificationError::NegativeTime);
    }

    let path_length = route_axis_length(route, policy)
        .ok_or(RouteCertificationError::UnsupportedRouteGeometry)?;
    let peak_feed_rate = (jerk.clone() * target_time.clone() * target_time.clone()
        / Real::from(16))
    .map_err(|_| RouteCertificationError::UnsupportedDivision)?;
    let peak_acceleration = (jerk.clone() * target_time.clone() / Real::from(4))
        .map_err(|_| RouteCertificationError::UnsupportedDivision)?;

    let mut problem = Problem::default();
    let time = problem.add_variable("time", target_time.clone());
    problem.add_constraint(symmetric_jerk_limited_feed_time_equation(
        "symmetric jerk limited feed time",
        path_length.clone(),
        jerk.clone(),
        time,
    ));
    problem.add_constraint(peak_feed_limit_constraint(
        "symmetric jerk peak feed limit",
        max_feed_rate.clone(),
        jerk.clone(),
        time,
    ));
    problem.add_constraint(peak_acceleration_limit_constraint(
        "symmetric jerk peak acceleration limit",
        max_acceleration.clone(),
        jerk.clone(),
        time,
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    Ok(JerkLimitedFeedTimeReport {
        path_length,
        max_feed_rate,
        max_acceleration,
        jerk,
        target_time,
        peak_feed_rate,
        peak_acceleration,
        certification: certify_candidate(&prepared, &context),
    })
}

fn peak_feed_limit_constraint(
    name: impl Into<String>,
    max_feed_rate: Real,
    jerk: Real,
    time: hypersolve::VariableId,
) -> Constraint {
    let time_expr = Expr::symbol(time.into(), "time");
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(Real::from(16) * max_feed_rate)
            - Expr::real(jerk) * time_expr.clone() * time_expr,
        weight: Real::one(),
        active: true,
    }
}

fn peak_acceleration_limit_constraint(
    name: impl Into<String>,
    max_acceleration: Real,
    jerk: Real,
    time: hypersolve::VariableId,
) -> Constraint {
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(Real::from(4) * max_acceleration)
            - Expr::real(jerk) * Expr::symbol(time.into(), "time"),
        weight: Real::one(),
        active: true,
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
