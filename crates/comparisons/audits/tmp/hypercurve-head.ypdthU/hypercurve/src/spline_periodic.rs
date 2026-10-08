//! Exact periodic spline construction and parameter normalization.

use crate::{
    CurveContext, CurveError, CurveFamily2, CurveOperation2, CurveParameterSide2, ExactCurveError,
    ExactCurveResult, Point2, Real, UncertaintyReason,
};

/// Exact periodicity evidence retained by a spline carrier.
#[derive(Clone, Debug, PartialEq)]
pub enum SplinePeriodicity2 {
    /// The spline is evaluated only over its finite active knot domain.
    NonPeriodic,
    /// The spline repeats after one exact positive parameter period.
    Periodic {
        /// Exact positive period.
        period: Real,
    },
}

pub(crate) struct PeriodicSplineExpansion2 {
    pub(crate) control_points: Vec<Point2>,
    pub(crate) knots: Vec<Real>,
    pub(crate) period: Real,
}

impl SplinePeriodicity2 {
    /// Returns whether this spline carries certified periodic semantics.
    pub const fn is_periodic(&self) -> bool {
        matches!(self, Self::Periodic { .. })
    }

    /// Returns the exact period when this spline is periodic.
    pub const fn period(&self) -> Option<&Real> {
        match self {
            Self::NonPeriodic => None,
            Self::Periodic { period } => Some(period),
        }
    }
}

pub(crate) fn expand_periodic_spline(
    degree: usize,
    mut control_points: Vec<Point2>,
    period_knots: Vec<Real>,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<PeriodicSplineExpansion2> {
    let unique_control_count = control_points.len();
    let valid_layout = degree >= 1
        && unique_control_count > degree
        && period_knots.len() == unique_control_count + 1;
    if !valid_layout {
        return Err(periodic_error(family, CurveError::InvalidPeriodicSpline));
    }

    for pair in period_knots.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Some(std::cmp::Ordering::Greater) => {
                return Err(periodic_error(family, CurveError::InvalidPeriodicSpline));
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Construction,
                    family,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }

    let period = period_knots
        .last()
        .expect("validated periodic knot sequence is nonempty")
        - &period_knots[0];
    match crate::classify::compare_reals(&Real::zero(), &period, policy) {
        Some(std::cmp::Ordering::Less) => {}
        Some(_) => {
            return Err(periodic_error(family, CurveError::InvalidPeriodicSpline));
        }
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Construction,
                family,
                UncertaintyReason::Ordering,
            ));
        }
    }

    control_points.extend_from_within(..degree);
    let mut knots = Vec::with_capacity(period_knots.len() + 2 * degree);
    knots.extend(
        period_knots[unique_control_count - degree..unique_control_count]
            .iter()
            .map(|knot| knot - &period),
    );
    knots.extend(period_knots.iter().cloned());
    knots.extend(
        period_knots
            .iter()
            .skip(1)
            .take(degree)
            .map(|knot| knot + &period),
    );

    Ok(PeriodicSplineExpansion2 {
        control_points,
        knots,
        period,
    })
}

pub(crate) fn wrap_periodic_parameter(
    parameter: &Real,
    domain_start: &Real,
    domain_end: &Real,
    periodicity: &SplinePeriodicity2,
    side: CurveParameterSide2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    let Some(period) = periodicity.period() else {
        return Err(ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            family,
            CurveError::CurveIsNotPeriodic,
        ));
    };
    let displacement = parameter - domain_start;
    let remainder = match displacement.rem_euclid_certified(period) {
        Ok(remainder) => remainder,
        Err(_) if policy.permits_approximate_512() => {
            terminal_periodic_remainder(&displacement, period, family, policy)?
        }
        Err(_) => return Err(periodic_evaluation_blocker(family)),
    };
    match crate::classify::compare_reals(&remainder, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) if side == CurveParameterSide2::Left => {
            Ok(domain_end.clone())
        }
        Some(std::cmp::Ordering::Equal) => Ok(domain_start.clone()),
        Some(_) => Ok(domain_start + remainder),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            UncertaintyReason::Ordering,
        )),
    }
}

#[cold]
fn terminal_periodic_remainder(
    displacement: &Real,
    period: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    let quotient = (displacement / period).map_err(|_| periodic_evaluation_blocker(family))?;
    let half =
        (Real::one() / Real::from(2_i8)).expect("the exact positive integer two has a reciprocal");
    let nearest = (&quotient + half)
        .floor_certified()
        .map_err(|_| periodic_evaluation_blocker(family))?;
    let nearest = Real::integer(nearest);
    // `nearest` differs from floor(quotient) by at most one.  The policy is
    // therefore consumed only to choose between those two exact integers; the
    // returned remainder retains the authored symbolic displacement and period.
    let floor = match crate::classify::compare_reals(&quotient, &nearest, policy) {
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => nearest,
        Some(std::cmp::Ordering::Less) => nearest - Real::one(),
        None => return Err(periodic_evaluation_blocker(family)),
    };
    Ok(displacement - &(floor * period))
}

fn periodic_evaluation_blocker(family: CurveFamily2) -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Evaluation,
        family,
        UncertaintyReason::Ordering,
    )
}

fn periodic_error(family: CurveFamily2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Construction, family, cause)
}
