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

use crate::arc::ExplicitCircularArc;
use crate::segment::LinePathSegment;
use crate::solve::symmetric_jerk_limited_feed_time_equation;
use crate::tangent::{TangentJoinClass, TangentJoinReport, TangentSpan, classify_tangent_join};

use super::{
    AccelerationLimitedFeedTimeReport, ConstantFeedTimeReport, RouteCertificationError,
    acceleration_limited_feed_time_equation, classify_acceleration_limited_profile,
    route_axis_length,
};

/// Retained feed-replay path element for exact mixed line/arc routes.
///
/// This is a metric carrier, not a topology materializer. Lines contribute
/// their certified axis-aligned length, and explicit circular arcs contribute
/// [`ExplicitCircularArc::certified_sweep_length`]. That keeps Farouki's
/// curve-length and path-parameterization view compatible with Yap's exact
/// object/predicate boundary: unsupported line directions or undecidable arc
/// sweep lengths reject before any feed residual is replayed.
#[derive(Clone, Debug, PartialEq)]
pub enum FeedPathElement {
    /// Exact line path segment, accepted only when axis-aligned.
    Line(LinePathSegment),
    /// Exact explicit circular arc with a certified symbolic sweep length.
    ExplicitArc(ExplicitCircularArc),
}

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

/// Exact lookahead classification for one retained path join.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CornerLookaheadJoinClass {
    /// The join is G1-continuous, so this report only certifies the feed cap.
    StraightThrough,
    /// The join is a true corner and must respect the retained corner radius.
    RadiusLimitedCorner,
    /// The join reverses direction and requires a zero corner speed.
    ReversalStop,
}

/// Exact corner-speed replay report for one adjacent tangent-span join.
///
/// The report carries the tangent predicate result and a solver replay of the
/// process limit applied to the candidate junction feed. For corners, the
/// replay row is the squared-speed form of the centripetal bound `v^2 <= a*r`
/// for a caller-retained blend radius. This is the path-parameterization view
/// used by Bobrow, Dubowsky, and Gibson (1985): the geometric path is retained,
/// while dynamic limits are algebraic constraints on motion along that path.
/// Yap's exact-computation discipline is preserved by accepting no numeric
/// controller proposal until those inequalities are certified exactly.
#[derive(Clone, Debug)]
pub struct CornerLookaheadJoinReport {
    /// Zero-based adjacent-span join index.
    pub index: u64,
    /// Exact endpoint/tangent predicate report used to choose the limit class.
    pub tangent_join: TangentJoinReport,
    /// Lookahead class selected from the tangent predicate report.
    pub class: CornerLookaheadJoinClass,
    /// Exact candidate feed rate at the join.
    pub candidate_corner_feed: Real,
    /// Exact maximum feed-rate cap.
    pub max_feed_rate: Real,
    /// Exact maximum centripetal acceleration cap.
    pub max_acceleration: Real,
    /// Exact retained corner/blend radius used for true corners.
    pub corner_radius: Real,
    /// Exact replay report for the feed cap, corner bound, and reversal stop.
    pub certification: CandidateCertificationReport,
}

/// Exact lookahead replay report for a retained tangent-span chain.
#[derive(Clone, Debug)]
pub struct CornerLookaheadLimitReport {
    /// Per-adjacent-join reports in traversal order.
    pub joins: Vec<CornerLookaheadJoinReport>,
}

impl CornerLookaheadLimitReport {
    /// Return whether every join's candidate corner feed was certified.
    pub fn all_satisfied(&self) -> bool {
        self.joins
            .iter()
            .all(|join| join.certification.all_satisfied())
    }

    /// Return the first join with a certified violation or undecided row.
    pub fn first_unsatisfied_join(&self) -> Option<usize> {
        self.joins
            .iter()
            .position(|join| !join.certification.all_satisfied())
    }
}

/// Certify constant-feed traversal time for a retained mixed line/arc route.
///
/// This is the curved-segment counterpart of
/// [`super::certify_constant_feed_time`]. The route length is first replayed
/// from retained path objects: axis-aligned lines use exact coordinate
/// differences, while explicit circular arcs use symbolic `pi`/`acos` sweep
/// lengths when the arc sweep class is certified. The feed residual remains
/// `length - feed * time = 0`, so invalid process parameters and unsupported
/// geometry are reported before solver replay.
pub fn certify_constant_feed_time_for_path(
    route: &[FeedPathElement],
    feed_rate: Real,
    target_time: Real,
    policy: PredicatePolicy,
) -> Result<ConstantFeedTimeReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    require_positive_feed(&feed_rate)?;
    if target_time.structural_facts().sign == Some(RealSign::Negative) {
        return Err(RouteCertificationError::NegativeTime);
    }
    let path_length = route_path_length(route, policy)?;
    let mut problem = Problem::default();
    let time = problem.add_variable("time", target_time.clone());
    problem.add_constraint(crate::solve::constant_feed_time_equation(
        "constant feed time",
        path_length.clone(),
        feed_rate.clone(),
        time,
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(ConstantFeedTimeReport {
        path_length,
        feed_rate,
        target_time,
        certification: certify_candidate(&prepared, &context),
    })
}

/// Certify symmetric acceleration-limited traversal for a mixed line/arc route.
///
/// The profile classification is still the exact comparison
/// `acceleration * length` versus `max_feed^2`; the only change from the
/// line-only API is that `length` may include certified circular-arc sweep
/// terms. No sampling, chord approximation, or controller lookahead is
/// introduced.
pub fn certify_acceleration_limited_feed_time_for_path(
    route: &[FeedPathElement],
    max_feed_rate: Real,
    acceleration: Real,
    target_time: Real,
    policy: PredicatePolicy,
) -> Result<AccelerationLimitedFeedTimeReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&acceleration)?;
    if target_time.structural_facts().sign == Some(RealSign::Negative) {
        return Err(RouteCertificationError::NegativeTime);
    }
    let path_length = route_path_length(route, policy)?;
    let profile =
        classify_acceleration_limited_profile(&path_length, &max_feed_rate, &acceleration, policy)
            .ok_or(RouteCertificationError::UnknownFeedProfile)?;
    let mut problem = Problem::default();
    let time = problem.add_variable("time", target_time.clone());
    problem.add_constraint(acceleration_limited_feed_time_equation(
        "acceleration limited feed time",
        path_length.clone(),
        max_feed_rate.clone(),
        acceleration.clone(),
        time,
        profile,
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(AccelerationLimitedFeedTimeReport {
        path_length,
        max_feed_rate,
        acceleration,
        target_time,
        profile,
        certification: certify_candidate(&prepared, &context),
    })
}

/// Certify a symmetric four-phase jerk-limited feed profile for a mixed route.
///
/// The S-curve equations are identical to
/// [`certify_symmetric_jerk_limited_feed_time`], but retained explicit arcs may
/// contribute exact symbolic sweep length. This directly addresses curved
/// route feed replay without converting arcs to polylines.
pub fn certify_symmetric_jerk_limited_feed_time_for_path(
    route: &[FeedPathElement],
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

    let path_length = route_path_length(route, policy)?;
    build_jerk_limited_report(
        path_length,
        max_feed_rate,
        max_acceleration,
        jerk,
        target_time,
    )
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
    build_jerk_limited_report(
        path_length,
        max_feed_rate,
        max_acceleration,
        jerk,
        target_time,
    )
}

/// Certify exact lookahead corner-speed limits for retained tangent spans.
///
/// This is a process-constraint report for a path chain, not a mesh or stock
/// boolean. Adjacent [`TangentSpan`] values are classified by exact endpoint
/// and tangent predicates. G1 joins replay only the global feed cap; true
/// corners additionally replay `a_max * r_corner - v_corner^2 >= 0`; reversed
/// tangents replay an exact stop row `v_corner = 0`. The constraints are kept
/// denominator-free and algebraic so `hypersolve` can audit the caller's
/// candidate speed under Yap's "construct then certify" model.
///
/// The caller supplies one retained corner radius for this chain. A future
/// richer scheduler can call this function per corner with locally selected
/// radii, but the certification model remains the same: geometry decides the
/// join class, and exact replay decides whether the proposed process speed is
/// admissible.
pub fn certify_corner_lookahead_limits(
    spans: &[TangentSpan],
    candidate_corner_feed: Real,
    max_feed_rate: Real,
    max_acceleration: Real,
    corner_radius: Real,
    policy: PredicatePolicy,
) -> Result<CornerLookaheadLimitReport, RouteCertificationError> {
    if spans.len() < 2 {
        return Err(RouteCertificationError::EmptyRoute);
    }
    require_nonnegative_feed(&candidate_corner_feed)?;
    require_positive_feed(&max_feed_rate)?;
    require_positive_acceleration(&max_acceleration)?;
    require_positive_corner_radius(&corner_radius)?;

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
        let certification = certify_corner_lookahead_join_candidate(
            class,
            candidate_corner_feed.clone(),
            max_feed_rate.clone(),
            max_acceleration.clone(),
            corner_radius.clone(),
        );
        joins.push(CornerLookaheadJoinReport {
            index: index as u64,
            tangent_join,
            class,
            candidate_corner_feed: candidate_corner_feed.clone(),
            max_feed_rate: max_feed_rate.clone(),
            max_acceleration: max_acceleration.clone(),
            corner_radius: corner_radius.clone(),
            certification,
        });
    }

    Ok(CornerLookaheadLimitReport { joins })
}

fn build_jerk_limited_report(
    path_length: Real,
    max_feed_rate: Real,
    max_acceleration: Real,
    jerk: Real,
    target_time: Real,
) -> Result<JerkLimitedFeedTimeReport, RouteCertificationError> {
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

fn certify_corner_lookahead_join_candidate(
    class: CornerLookaheadJoinClass,
    candidate_corner_feed: Real,
    max_feed_rate: Real,
    max_acceleration: Real,
    corner_radius: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let feed = problem.add_variable("corner_feed", candidate_corner_feed);
    problem.add_constraint(corner_feed_cap_constraint(
        "lookahead corner feed cap",
        max_feed_rate,
        feed,
    ));
    match class {
        CornerLookaheadJoinClass::StraightThrough => {}
        CornerLookaheadJoinClass::RadiusLimitedCorner => {
            problem.add_constraint(corner_centripetal_limit_constraint(
                "lookahead corner centripetal limit",
                max_acceleration,
                corner_radius,
                feed,
            ));
        }
        CornerLookaheadJoinClass::ReversalStop => {
            problem.add_constraint(Constraint::equality(
                "lookahead reversal stop",
                Expr::symbol(feed.into(), "corner_feed"),
            ));
        }
    }
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    certify_candidate(&prepared, &context)
}

fn route_path_length(
    route: &[FeedPathElement],
    policy: PredicatePolicy,
) -> Result<Real, RouteCertificationError> {
    route.iter().try_fold(Real::zero(), |sum, element| {
        element_length(element, policy).map(|length| sum + length)
    })
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

fn corner_feed_cap_constraint(
    name: impl Into<String>,
    max_feed_rate: Real,
    feed: hypersolve::VariableId,
) -> Constraint {
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(max_feed_rate) - Expr::symbol(feed.into(), "corner_feed"),
        weight: Real::one(),
        active: true,
    }
}

fn corner_centripetal_limit_constraint(
    name: impl Into<String>,
    max_acceleration: Real,
    corner_radius: Real,
    feed: hypersolve::VariableId,
) -> Constraint {
    let feed_expr = Expr::symbol(feed.into(), "corner_feed");
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(max_acceleration * corner_radius) - feed_expr.clone() * feed_expr,
        weight: Real::one(),
        active: true,
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

fn require_positive_corner_radius(value: &Real) -> Result<(), RouteCertificationError> {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Err(RouteCertificationError::NegativeCornerRadius),
        Some(RealSign::Zero) => Err(RouteCertificationError::ZeroCornerRadius),
        _ => Ok(()),
    }
}
