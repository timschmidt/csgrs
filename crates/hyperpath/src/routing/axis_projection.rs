//! Exact dense-axis projection for affine path spans.
//!
//! For an affine span parameterized by scalar path distance `s`, each machine
//! coordinate has a constant derivative `dq_i/ds`. The scalar path limits are
//! therefore bounded by
//!
//! `|dq_i/ds| * v <= V_i`,
//! `|dq_i/ds| * a <= A_i`, and
//! `|dq_i/ds| * j <= J_i`.
//!
//! This module selects the exact route-wide minimum across any number of dense
//! axes and retained spans, then independently replays every inequality through
//! `hypersolve`. The affine qualification is intentional: curved geometry or a
//! nonlinear kinematic map also has higher-derivative terms and must not reuse
//! this projection without separately certifying them.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, Sign, classify_real_sign, compare_reals};
use hyperreal::Real;
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, Problem, certify_candidate,
    context_from_problem,
};

use super::RouteCertificationError;

/// Exact positive dynamic limits for one dense machine axis.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisMotionLimits {
    /// Maximum absolute axis velocity.
    pub maximum_velocity: Real,
    /// Maximum absolute axis acceleration.
    pub maximum_acceleration: Real,
    /// Maximum absolute axis jerk.
    pub maximum_jerk: Real,
}

/// Constant absolute machine-coordinate derivatives for one affine path span.
///
/// Entry `i` is the exact nonnegative value `|dq_i/ds|` for dense axis `i`.
/// The vector length must equal the axis-limit count. At least one derivative
/// must be structurally positive so the span represents machine motion.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineSpanAxisProjection {
    /// Exact absolute `dq_i/ds` values in dense-axis order.
    pub absolute_axis_derivatives: Vec<Real>,
}

/// Dense span/axis location which exactly attains one selected scalar limit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AxisProjectionBottleneck {
    /// Zero-based affine-span index.
    pub span_index: u64,
    /// Zero-based dense-axis index.
    pub axis_index: u64,
}

/// Independent projection replay for one affine span and one dense axis.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisProjectionRowReport {
    /// Zero-based affine-span index.
    pub span_index: u64,
    /// Zero-based dense-axis index.
    pub axis_index: u64,
    /// Exact absolute `dq_i/ds` used by all three inequalities.
    pub absolute_axis_derivative: Real,
    /// Exact axis limits against which the scalar candidate was replayed.
    pub axis_limits: AxisMotionLimits,
    /// Hypersolve replay of velocity, acceleration, and jerk projection rows.
    pub certification: CandidateCertificationReport,
}

/// Independent replay of one proposed route-wide affine projection.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisProjectedMotionLimitsReport {
    /// Proposed maximum scalar path feed.
    pub maximum_path_feed: Real,
    /// Proposed maximum scalar path acceleration.
    pub maximum_path_acceleration: Real,
    /// Proposed maximum scalar path jerk.
    pub maximum_path_jerk: Real,
    /// One three-row replay for every affine span/axis pair, including zeros.
    pub rows: Vec<AxisProjectionRowReport>,
}

impl AxisProjectedMotionLimitsReport {
    /// Return whether every axis projection inequality replayed exactly.
    pub fn all_satisfied(&self) -> bool {
        !self.rows.is_empty()
            && self
                .rows
                .iter()
                .all(|row| row.certification.all_satisfied())
    }
}

/// Exact route-wide affine projection and its independent certification.
#[derive(Clone, Debug, PartialEq)]
pub struct PlannedAxisProjectedMotionLimits {
    /// Exact maximum scalar path feed.
    pub maximum_path_feed: Real,
    /// Exact maximum scalar path acceleration.
    pub maximum_path_acceleration: Real,
    /// Exact maximum scalar path jerk.
    pub maximum_path_jerk: Real,
    /// First exact row attaining the feed limit.
    pub feed_bottleneck: AxisProjectionBottleneck,
    /// First exact row attaining the acceleration limit.
    pub acceleration_bottleneck: AxisProjectionBottleneck,
    /// First exact row attaining the jerk limit.
    pub jerk_bottleneck: AxisProjectionBottleneck,
    /// Independent replay of every span/axis inequality.
    pub certification: AxisProjectedMotionLimitsReport,
    /// Exact equality replay at the three retained bottleneck rows.
    pub bottleneck_certification: CandidateCertificationReport,
}

impl PlannedAxisProjectedMotionLimits {
    /// Return whether every inequality and each selected limiting equality replayed.
    pub fn all_satisfied(&self) -> bool {
        self.certification.all_satisfied() && self.bottleneck_certification.all_satisfied()
    }
}

/// Select and certify exact route-wide limits for affine dense-axis spans.
///
/// Each positive derivative contributes `axis_limit / |dq_i/ds|` as a scalar
/// candidate. Structurally zero derivatives contribute no limit. The exact
/// minimum is selected independently for velocity, acceleration, and jerk;
/// ties retain the first span/axis row for deterministic evidence.
pub fn plan_axis_projected_motion_limits(
    projections: &[AffineSpanAxisProjection],
    axis_limits: &[AxisMotionLimits],
    policy: PredicatePolicy,
) -> Result<PlannedAxisProjectedMotionLimits, RouteCertificationError> {
    validate_projection_inputs(projections, axis_limits, policy)?;

    let mut feed: Option<(Real, AxisProjectionBottleneck)> = None;
    let mut acceleration: Option<(Real, AxisProjectionBottleneck)> = None;
    let mut jerk: Option<(Real, AxisProjectionBottleneck)> = None;
    for (span_index, projection) in projections.iter().enumerate() {
        for (axis_index, derivative) in projection.absolute_axis_derivatives.iter().enumerate() {
            if !is_positive_projection(derivative, policy)? {
                continue;
            }
            let location = AxisProjectionBottleneck {
                span_index: span_index as u64,
                axis_index: axis_index as u64,
            };
            feed = select_minimum(
                feed,
                (&axis_limits[axis_index].maximum_velocity / derivative)
                    .map_err(|_| RouteCertificationError::UnsupportedDivision)?,
                location,
                policy,
            )?;
            acceleration = select_minimum(
                acceleration,
                (&axis_limits[axis_index].maximum_acceleration / derivative)
                    .map_err(|_| RouteCertificationError::UnsupportedDivision)?,
                location,
                policy,
            )?;
            jerk = select_minimum(
                jerk,
                (&axis_limits[axis_index].maximum_jerk / derivative)
                    .map_err(|_| RouteCertificationError::UnsupportedDivision)?,
                location,
                policy,
            )?;
        }
    }

    let (maximum_path_feed, feed_bottleneck) =
        feed.ok_or(RouteCertificationError::DegenerateAxisProjection)?;
    let (maximum_path_acceleration, acceleration_bottleneck) =
        acceleration.ok_or(RouteCertificationError::DegenerateAxisProjection)?;
    let (maximum_path_jerk, jerk_bottleneck) =
        jerk.ok_or(RouteCertificationError::DegenerateAxisProjection)?;
    let certification = certify_axis_projected_motion_limits(
        projections,
        axis_limits,
        maximum_path_feed.clone(),
        maximum_path_acceleration.clone(),
        maximum_path_jerk.clone(),
        policy,
    )?;
    let bottleneck_certification = certify_bottlenecks(
        projections,
        axis_limits,
        &maximum_path_feed,
        &maximum_path_acceleration,
        &maximum_path_jerk,
        feed_bottleneck,
        acceleration_bottleneck,
        jerk_bottleneck,
    );
    if !certification.all_satisfied() || !bottleneck_certification.all_satisfied() {
        return Err(RouteCertificationError::AxisProjectionProposalUncertified);
    }

    Ok(PlannedAxisProjectedMotionLimits {
        maximum_path_feed,
        maximum_path_acceleration,
        maximum_path_jerk,
        feed_bottleneck,
        acceleration_bottleneck,
        jerk_bottleneck,
        certification,
        bottleneck_certification,
    })
}

/// Replay proposed scalar limits against every affine dense-axis inequality.
///
/// This does not require a proposal to be maximal; it certifies safety. The
/// planner separately retains exact bottleneck equalities to establish that its
/// selected values are the route-wide minima.
pub fn certify_axis_projected_motion_limits(
    projections: &[AffineSpanAxisProjection],
    axis_limits: &[AxisMotionLimits],
    maximum_path_feed: Real,
    maximum_path_acceleration: Real,
    maximum_path_jerk: Real,
    policy: PredicatePolicy,
) -> Result<AxisProjectedMotionLimitsReport, RouteCertificationError> {
    validate_projection_inputs(projections, axis_limits, policy)?;
    require_positive(
        &maximum_path_feed,
        RouteCertificationError::NegativeFeedRate,
        RouteCertificationError::ZeroFeedRate,
        policy,
    )?;
    require_positive(
        &maximum_path_acceleration,
        RouteCertificationError::NegativeAcceleration,
        RouteCertificationError::ZeroAcceleration,
        policy,
    )?;
    require_positive(
        &maximum_path_jerk,
        RouteCertificationError::NegativeJerk,
        RouteCertificationError::ZeroJerk,
        policy,
    )?;

    let mut rows = Vec::with_capacity(projections.len().saturating_mul(axis_limits.len()));
    for (span_index, projection) in projections.iter().enumerate() {
        for (axis_index, derivative) in projection.absolute_axis_derivatives.iter().enumerate() {
            rows.push(AxisProjectionRowReport {
                span_index: span_index as u64,
                axis_index: axis_index as u64,
                absolute_axis_derivative: derivative.clone(),
                axis_limits: axis_limits[axis_index].clone(),
                certification: certify_projection_row(
                    derivative.clone(),
                    &axis_limits[axis_index],
                    maximum_path_feed.clone(),
                    maximum_path_acceleration.clone(),
                    maximum_path_jerk.clone(),
                ),
            });
        }
    }
    Ok(AxisProjectedMotionLimitsReport {
        maximum_path_feed,
        maximum_path_acceleration,
        maximum_path_jerk,
        rows,
    })
}

fn validate_projection_inputs(
    projections: &[AffineSpanAxisProjection],
    axis_limits: &[AxisMotionLimits],
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    if projections.is_empty() || axis_limits.is_empty() {
        return Err(RouteCertificationError::EmptyAxisProjection);
    }
    for limits in axis_limits {
        require_positive(
            &limits.maximum_velocity,
            RouteCertificationError::NegativeFeedRate,
            RouteCertificationError::ZeroFeedRate,
            policy,
        )?;
        require_positive(
            &limits.maximum_acceleration,
            RouteCertificationError::NegativeAcceleration,
            RouteCertificationError::ZeroAcceleration,
            policy,
        )?;
        require_positive(
            &limits.maximum_jerk,
            RouteCertificationError::NegativeJerk,
            RouteCertificationError::ZeroJerk,
            policy,
        )?;
    }
    for projection in projections {
        if projection.absolute_axis_derivatives.len() != axis_limits.len() {
            return Err(RouteCertificationError::AxisProjectionShapeMismatch);
        }
        let mut has_positive = false;
        for derivative in &projection.absolute_axis_derivatives {
            match classify_real_sign(derivative, policy).value() {
                Some(Sign::Negative) => {
                    return Err(RouteCertificationError::NegativeAxisProjection);
                }
                Some(Sign::Zero) => {}
                Some(Sign::Positive) => has_positive = true,
                None => return Err(RouteCertificationError::PredicateUnresolved),
            }
        }
        if !has_positive {
            return Err(RouteCertificationError::DegenerateAxisProjection);
        }
    }
    Ok(())
}

fn require_positive(
    value: &Real,
    negative: RouteCertificationError,
    zero: RouteCertificationError,
    policy: PredicatePolicy,
) -> Result<(), RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(negative),
        Some(Sign::Zero) => Err(zero),
        Some(Sign::Positive) => Ok(()),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn is_positive_projection(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<bool, RouteCertificationError> {
    match classify_real_sign(value, policy).value() {
        Some(Sign::Negative) => Err(RouteCertificationError::NegativeAxisProjection),
        Some(Sign::Zero) => Ok(false),
        Some(Sign::Positive) => Ok(true),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn select_minimum(
    current: Option<(Real, AxisProjectionBottleneck)>,
    candidate: Real,
    location: AxisProjectionBottleneck,
    policy: PredicatePolicy,
) -> Result<Option<(Real, AxisProjectionBottleneck)>, RouteCertificationError> {
    let Some((selected, selected_location)) = current else {
        return Ok(Some((candidate, location)));
    };
    match compare_reals(&candidate, &selected, policy).value() {
        Some(Ordering::Less) => Ok(Some((candidate, location))),
        Some(Ordering::Equal | Ordering::Greater) => Ok(Some((selected, selected_location))),
        None => Err(RouteCertificationError::PredicateUnresolved),
    }
}

fn certify_projection_row(
    derivative: Real,
    axis_limits: &AxisMotionLimits,
    maximum_path_feed: Real,
    maximum_path_acceleration: Real,
    maximum_path_jerk: Real,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let feed = problem.add_variable("path_feed", maximum_path_feed);
    let acceleration = problem.add_variable("path_acceleration", maximum_path_acceleration);
    let jerk = problem.add_variable("path_jerk", maximum_path_jerk);
    problem.add_constraint(projection_constraint(
        "affine axis velocity projection",
        axis_limits.maximum_velocity.clone(),
        derivative.clone(),
        feed,
        "path_feed",
    ));
    problem.add_constraint(projection_constraint(
        "affine axis acceleration projection",
        axis_limits.maximum_acceleration.clone(),
        derivative.clone(),
        acceleration,
        "path_acceleration",
    ));
    problem.add_constraint(projection_constraint(
        "affine axis jerk projection",
        axis_limits.maximum_jerk.clone(),
        derivative,
        jerk,
        "path_jerk",
    ));
    certify_problem(problem)
}

#[allow(clippy::too_many_arguments)]
fn certify_bottlenecks(
    projections: &[AffineSpanAxisProjection],
    axis_limits: &[AxisMotionLimits],
    maximum_path_feed: &Real,
    maximum_path_acceleration: &Real,
    maximum_path_jerk: &Real,
    feed_bottleneck: AxisProjectionBottleneck,
    acceleration_bottleneck: AxisProjectionBottleneck,
    jerk_bottleneck: AxisProjectionBottleneck,
) -> CandidateCertificationReport {
    let mut problem = Problem::default();
    let feed = problem.add_variable("path_feed", maximum_path_feed.clone());
    let acceleration = problem.add_variable("path_acceleration", maximum_path_acceleration.clone());
    let jerk = problem.add_variable("path_jerk", maximum_path_jerk.clone());
    add_bottleneck_equality(
        &mut problem,
        "affine feed bottleneck equality",
        projections,
        axis_limits,
        feed_bottleneck,
        feed,
        "path_feed",
        |limits| limits.maximum_velocity.clone(),
    );
    add_bottleneck_equality(
        &mut problem,
        "affine acceleration bottleneck equality",
        projections,
        axis_limits,
        acceleration_bottleneck,
        acceleration,
        "path_acceleration",
        |limits| limits.maximum_acceleration.clone(),
    );
    add_bottleneck_equality(
        &mut problem,
        "affine jerk bottleneck equality",
        projections,
        axis_limits,
        jerk_bottleneck,
        jerk,
        "path_jerk",
        |limits| limits.maximum_jerk.clone(),
    );
    certify_problem(problem)
}

#[allow(clippy::too_many_arguments)]
fn add_bottleneck_equality(
    problem: &mut Problem,
    name: &'static str,
    projections: &[AffineSpanAxisProjection],
    axis_limits: &[AxisMotionLimits],
    location: AxisProjectionBottleneck,
    candidate: hypersolve::VariableId,
    symbol_name: &'static str,
    limit: impl FnOnce(&AxisMotionLimits) -> Real,
) {
    let span_index = location.span_index as usize;
    let axis_index = location.axis_index as usize;
    let derivative = projections[span_index].absolute_axis_derivatives[axis_index].clone();
    problem.add_constraint(Constraint::equality(
        name,
        Expr::real(limit(&axis_limits[axis_index]))
            - Expr::real(derivative) * Expr::symbol(candidate.into(), symbol_name),
    ));
}

fn projection_constraint(
    name: &'static str,
    axis_limit: Real,
    derivative: Real,
    candidate: hypersolve::VariableId,
    symbol_name: &'static str,
) -> Constraint {
    Constraint {
        name: name.to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: Expr::real(axis_limit)
            - Expr::real(derivative) * Expr::symbol(candidate.into(), symbol_name),
        weight: Real::one(),
        active: true,
    }
}

fn certify_problem(problem: Problem) -> CandidateCertificationReport {
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    certify_candidate(&analysis, &context)
}
