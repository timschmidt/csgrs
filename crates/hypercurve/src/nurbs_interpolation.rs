//! Exact global interpolation for planar NURBS curves.

use hypersolve::{
    BareissError, PredicateCertainty, PredicatePolicy, solve_dense_linear_system_bareiss_multi_rhs,
};
use std::cmp::Ordering;
use std::sync::{Arc, Mutex, OnceLock};

use crate::policy::resolve_certified_operation;
use crate::{
    CurveContext, CurveError, CurveFamily2, CurveOperation2, CurveOutcome, ExactCurveError,
    ExactCurveResult, NurbsCurve2, Point2, Real, UncertaintyReason,
};

const INTERPOLATION_SOLVE_PRECISION: i32 = -128;
const MAX_RETAINED_UNIFORM_INTERPOLATION_SYSTEMS: usize = 8;

static UNIFORM_INTERPOLATION_SYSTEMS: OnceLock<Mutex<Vec<Arc<InterpolationSystem>>>> =
    OnceLock::new();

#[derive(Clone, Copy)]
enum DistanceParameterization {
    ChordLength,
    Centripetal,
}

struct InterpolationSystem {
    degree: usize,
    point_count: usize,
    control_weights: Arc<[Real]>,
    knots: Arc<[Real]>,
    coefficient_matrix: Arc<[Vec<Real>]>,
    denominators: Arc<[Real]>,
}

impl NurbsCurve2 {
    /// Globally interpolates exact points at exact, strictly increasing parameters.
    ///
    /// A clamped knot vector is derived by the standard averaging construction.
    /// Unit control weights produce a polynomial B-spline represented by the
    /// top-level NURBS carrier. The outcome covers every parameter, solve, and
    /// replay decision under the selected policy.
    pub fn interpolate_global(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsCurve2>> {
        resolve_certified_operation(policy, |attempt| {
            let knots = averaged_interpolation_knots(degree, &data_points, &parameters, attempt)?;
            interpolate_with_inputs(
                degree,
                data_points,
                parameters,
                vec![Real::one(); knots.len() - degree - 1],
                knots,
                attempt,
            )
        })
    }

    /// Globally interpolates exact points at uniformly spaced exact parameters.
    ///
    /// The outcome records any terminal decision consumed by solving or
    /// replaying the complete exact interpolation.
    pub fn interpolate_uniform(
        degree: usize,
        data_points: Vec<Point2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsCurve2>> {
        resolve_certified_operation(policy, |attempt| {
            let system = uniform_interpolation_system(degree, &data_points, attempt)?;
            interpolate_with_precomputed_system(
                degree,
                data_points,
                system.control_weights.iter().cloned().collect(),
                system.knots.iter().cloned().collect(),
                &system.coefficient_matrix,
                &system.denominators,
                attempt,
            )
        })
    }

    /// Globally interpolates using exact Euclidean chord-length parameters.
    pub fn interpolate_chord_length(
        degree: usize,
        data_points: Vec<Point2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsCurve2>> {
        resolve_certified_operation(policy, |attempt| {
            interpolate_distance_parameterized(
                degree,
                data_points,
                DistanceParameterization::ChordLength,
                attempt,
            )
        })
    }

    /// Globally interpolates using exact centripetal parameters.
    pub fn interpolate_centripetal(
        degree: usize,
        data_points: Vec<Point2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsCurve2>> {
        resolve_certified_operation(policy, |attempt| {
            interpolate_distance_parameterized(
                degree,
                data_points,
                DistanceParameterization::Centripetal,
                attempt,
            )
        })
    }

    /// Interpolates with explicit exact parameters, control weights, and knots.
    ///
    /// The fixed control weights make this a linear homogeneous interpolation
    /// problem. Every solved coordinate is replayed against the coefficient
    /// matrix by `hypersolve`. Those rows are the exact homogenized authored
    /// point constraints; together with certified nonzero row denominators,
    /// their residual reports are the authoritative interpolation proof. The
    /// outcome records any selected terminal consumed along that complete path.
    pub fn interpolate_with_parameters_and_knots(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
        control_weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsCurve2>> {
        resolve_certified_operation(policy, |attempt| {
            interpolate_with_inputs(
                degree,
                data_points,
                parameters,
                control_weights,
                knots,
                attempt,
            )
        })
    }
}

fn interpolate_with_inputs(
    degree: usize,
    data_points: Vec<Point2>,
    parameters: Vec<Real>,
    control_weights: Vec<Real>,
    knots: Vec<Real>,
    policy: &CurveContext,
) -> ExactCurveResult<NurbsCurve2> {
    validate_interpolation_inputs(
        degree,
        &data_points,
        &parameters,
        &control_weights,
        &knots,
        policy,
    )?;
    let coefficient_matrix = build_interpolation_coefficient_matrix(
        degree,
        data_points.len(),
        &parameters,
        &control_weights,
        &knots,
        policy,
    )?;
    let denominators = interpolation_denominators(&coefficient_matrix, policy)?;
    interpolate_with_precomputed_system(
        degree,
        data_points,
        control_weights,
        knots,
        &coefficient_matrix,
        &denominators,
        policy,
    )
}

fn build_interpolation_coefficient_matrix(
    degree: usize,
    point_count: usize,
    parameters: &[Real],
    control_weights: &[Real],
    knots: &[Real],
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Vec<Real>>> {
    parameters
        .iter()
        .map(|parameter| {
            weighted_basis_row(
                degree,
                point_count,
                knots,
                control_weights,
                parameter,
                policy,
            )
        })
        .collect()
}

fn interpolation_denominators(
    coefficient_matrix: &[Vec<Real>],
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Real>> {
    coefficient_matrix
        .iter()
        .map(|row| {
            let denominator = row.iter().fold(Real::zero(), |sum, value| sum + value);
            certify_interpolation_denominator(&denominator, policy)?;
            Ok(denominator)
        })
        .collect()
}

fn interpolate_with_precomputed_system(
    degree: usize,
    data_points: Vec<Point2>,
    control_weights: Vec<Real>,
    knots: Vec<Real>,
    coefficient_matrix: &[Vec<Real>],
    denominators: &[Real],
    policy: &CurveContext,
) -> ExactCurveResult<NurbsCurve2> {
    debug_assert_eq!(data_points.len(), coefficient_matrix.len());
    debug_assert_eq!(data_points.len(), denominators.len());
    let mut rhs_x = Vec::with_capacity(data_points.len());
    let mut rhs_y = Vec::with_capacity(data_points.len());
    for (point, denominator) in data_points.iter().zip(denominators) {
        rhs_x.push(point.x() * denominator);
        rhs_y.push(point.y() * denominator);
    }
    let [x_solution, y_solution] =
        solve_interpolation_coordinates_bareiss(coefficient_matrix, &[rhs_x, rhs_y], policy)?;
    let control_points = x_solution
        .iter()
        .cloned()
        .zip(y_solution.iter().cloned())
        .map(|(x, y)| Point2::new(x, y))
        .collect::<Vec<_>>();
    let curve = NurbsCurve2::try_new_raw(degree, control_points, control_weights, knots, policy)
        .map_err(remap_interpolation_error)?;
    Ok(curve)
}

fn uniform_interpolation_system(
    degree: usize,
    data_points: &[Point2],
    policy: &CurveContext,
) -> ExactCurveResult<Arc<InterpolationSystem>> {
    let point_count = data_points.len();
    let systems = UNIFORM_INTERPOLATION_SYSTEMS.get_or_init(|| Mutex::new(Vec::new()));
    if let Some(system) = systems
        .lock()
        .expect("uniform interpolation system cache mutex poisoned")
        .iter()
        .find(|system| system.degree == degree && system.point_count == point_count)
    {
        return Ok(Arc::clone(system));
    }

    let parameters = uniform_interpolation_parameters(point_count)?;
    let knots = averaged_interpolation_knots(degree, data_points, &parameters, policy)?;
    let control_weights = vec![Real::one(); knots.len() - degree - 1];
    validate_interpolation_inputs(
        degree,
        data_points,
        &parameters,
        &control_weights,
        &knots,
        policy,
    )?;
    let coefficient_matrix = build_interpolation_coefficient_matrix(
        degree,
        point_count,
        &parameters,
        &control_weights,
        &knots,
        policy,
    )?;
    let denominators = interpolation_denominators(&coefficient_matrix, policy)?;
    let system = Arc::new(InterpolationSystem {
        degree,
        point_count,
        control_weights: control_weights.into(),
        knots: knots.into(),
        coefficient_matrix: coefficient_matrix.into(),
        denominators: denominators.into(),
    });

    let mut systems = systems
        .lock()
        .expect("uniform interpolation system cache mutex poisoned");
    if let Some(retained) = systems
        .iter()
        .find(|retained| retained.degree == degree && retained.point_count == point_count)
    {
        return Ok(Arc::clone(retained));
    }
    if systems.len() == MAX_RETAINED_UNIFORM_INTERPOLATION_SYSTEMS {
        let _ = systems.remove(0);
    }
    systems.push(Arc::clone(&system));
    Ok(system)
}

fn solve_interpolation_coordinates_bareiss(
    coefficient_matrix: &[Vec<Real>],
    right_hand_sides: &[Vec<Real>; 2],
    policy: &CurveContext,
) -> ExactCurveResult<[Vec<Real>; 2]> {
    let evidence = solve_dense_linear_system_bareiss_multi_rhs(
        coefficient_matrix,
        right_hand_sides,
        INTERPOLATION_SOLVE_PRECISION,
        interpolation_predicate_policy(policy),
    )
    .map_err(interpolation_solve_error)?;
    if evidence.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    for replay in &evidence.residual_replays {
        if !replay.accepted {
            let row = replay
                .rows
                .iter()
                .find(|row| !matches!(row.sign, hyperreal::RealSign::Zero))
                .map_or(0, |row| row.row_index);
            return Err(ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                CurveError::InconsistentNurbsInterpolationSolution { row },
            ));
        }
    }
    Ok(evidence
        .solutions
        .try_into()
        .expect("two right-hand sides were supplied"))
}

fn interpolation_predicate_policy(policy: &CurveContext) -> PredicatePolicy {
    #[cfg(feature = "predicates")]
    {
        policy.predicate_policy()
    }
    #[cfg(not(feature = "predicates"))]
    {
        let _ = policy;
        PredicatePolicy::STRICT
    }
}

fn interpolate_distance_parameterized(
    degree: usize,
    data_points: Vec<Point2>,
    parameterization: DistanceParameterization,
    policy: &CurveContext,
) -> ExactCurveResult<NurbsCurve2> {
    let parameters = distance_interpolation_parameters(&data_points, parameterization, policy)?;
    let knots = averaged_interpolation_knots(degree, &data_points, &parameters, policy)?;
    interpolate_with_inputs(
        degree,
        data_points,
        parameters,
        vec![Real::one(); knots.len() - degree - 1],
        knots,
        policy,
    )
}

fn distance_interpolation_parameters(
    data_points: &[Point2],
    parameterization: DistanceParameterization,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Real>> {
    if data_points.len() < 2 {
        return Err(invalid_interpolation());
    }
    let mut increments = Vec::with_capacity(data_points.len() - 1);
    for pair in data_points.windows(2) {
        let chord = pair[0].distance_squared(&pair[1]).sqrt().map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                cause.into(),
            )
        })?;
        let increment = match parameterization {
            DistanceParameterization::ChordLength => chord,
            DistanceParameterization::Centripetal => chord.sqrt().map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    cause.into(),
                )
            })?,
        };
        match crate::classify::compare_reals(&Real::zero(), &increment, policy) {
            Some(Ordering::Less) => increments.push(increment),
            Some(_) => return Err(invalid_interpolation()),
            None => return Err(blocked_interpolation(UncertaintyReason::RealSign)),
        }
    }
    let total = increments
        .iter()
        .fold(Real::zero(), |sum, increment| sum + increment);
    let mut parameters = Vec::with_capacity(data_points.len());
    parameters.push(Real::zero());
    let mut cumulative = Real::zero();
    for increment in increments.iter().take(increments.len() - 1) {
        cumulative += increment;
        parameters.push((cumulative.clone() / total.clone()).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                cause.into(),
            )
        })?);
    }
    parameters.push(Real::one());
    Ok(parameters)
}

fn certify_interpolation_denominator(
    denominator: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match crate::classify::compare_reals(denominator, &Real::zero(), policy) {
        Some(Ordering::Less | Ordering::Greater) => Ok(()),
        Some(Ordering::Equal) => Err(ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            CurveError::ZeroNurbsDenominator,
        )),
        None => Err(blocked_interpolation(UncertaintyReason::RealSign)),
    }
}

fn averaged_interpolation_knots(
    degree: usize,
    data_points: &[Point2],
    parameters: &[Real],
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Real>> {
    if degree < 1 || data_points.len() != parameters.len() || data_points.len() <= degree {
        return Err(invalid_interpolation());
    }
    validate_parameters(parameters, policy)?;
    let mut knots = Vec::with_capacity(data_points.len() + degree + 1);
    knots.extend(std::iter::repeat_n(parameters[0].clone(), degree + 1));
    let divisor = interpolation_usize_real(degree)?;
    for first in 1..data_points.len() - degree {
        let sum = parameters[first..first + degree]
            .iter()
            .fold(Real::zero(), |sum, parameter| sum + parameter);
        knots.push((sum / divisor.clone()).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                CurveError::UnsupportedNurbsInterpolationDivision { index: first },
            )
        })?);
    }
    knots.extend(std::iter::repeat_n(
        parameters[parameters.len() - 1].clone(),
        degree + 1,
    ));
    Ok(knots)
}

fn uniform_interpolation_parameters(point_count: usize) -> ExactCurveResult<Vec<Real>> {
    if point_count < 2 {
        return Err(invalid_interpolation());
    }
    let denominator = interpolation_usize_real(point_count - 1)?;
    (0..point_count)
        .map(|index| {
            (interpolation_usize_real(index)? / denominator.clone()).map_err(|_| {
                ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    CurveError::UnsupportedNurbsInterpolationDivision { index },
                )
            })
        })
        .collect()
}

fn interpolation_usize_real(value: usize) -> ExactCurveResult<Real> {
    u64::try_from(value)
        .map(Real::from)
        .map_err(|_| invalid_interpolation())
}

fn validate_interpolation_inputs(
    degree: usize,
    data_points: &[Point2],
    parameters: &[Real],
    control_weights: &[Real],
    knots: &[Real],
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let point_count = data_points.len();
    if degree < 1
        || point_count <= degree
        || parameters.len() != point_count
        || control_weights.len() != point_count
        || knots.len() != point_count + degree + 1
    {
        return Err(invalid_interpolation());
    }
    validate_parameters(parameters, policy)?;
    for pair in knots.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return Err(invalid_interpolation()),
            None => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
        }
    }
    match (
        crate::classify::compare_reals(&parameters[0], &knots[degree], policy),
        crate::classify::compare_reals(&parameters[point_count - 1], &knots[point_count], policy),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(()),
        (Some(_), Some(_)) => Err(invalid_interpolation()),
        _ => Err(blocked_interpolation(UncertaintyReason::Ordering)),
    }
}

fn validate_parameters(parameters: &[Real], policy: &CurveContext) -> ExactCurveResult<()> {
    if parameters.len() < 2 {
        return Err(invalid_interpolation());
    }
    for pair in parameters.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], policy) {
            Some(Ordering::Less) => {}
            Some(_) => return Err(invalid_interpolation()),
            None => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
        }
    }
    Ok(())
}

fn weighted_basis_row(
    degree: usize,
    control_count: usize,
    knots: &[Real],
    control_weights: &[Real],
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Real>> {
    let span = interpolation_span(degree, control_count, knots, parameter, policy)?;
    let mut basis = vec![Real::one()];
    let mut left = vec![Real::zero(); degree + 1];
    let mut right = vec![Real::zero(); degree + 1];
    for order in 1..=degree {
        left[order] = parameter - &knots[span + 1 - order];
        right[order] = &knots[span + order] - parameter;
        basis.push(Real::zero());
        let mut saved = Real::zero();
        for index in 0..order {
            let denominator = &right[index + 1] + &left[order - index];
            let term = (basis[index].clone() / denominator).map_err(|_| {
                ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    CurveError::UnsupportedNurbsInterpolationDivision { index: order },
                )
            })?;
            basis[index] = &saved + &right[index + 1] * &term;
            saved = &left[order - index] * term;
        }
        basis[order] = saved;
    }
    let mut row = vec![Real::zero(); control_count];
    let first_control = span - degree;
    for (local_index, value) in basis.into_iter().enumerate() {
        let control_index = first_control + local_index;
        row[control_index] = value * &control_weights[control_index];
    }
    Ok(row)
}

fn interpolation_span(
    degree: usize,
    control_count: usize,
    knots: &[Real],
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<usize> {
    let last_control = control_count - 1;
    match crate::classify::compare_reals(parameter, &knots[control_count], policy) {
        Some(Ordering::Equal) => return Ok(last_control),
        Some(_) => {}
        None => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
    }
    for span in degree..=last_control {
        match (
            crate::classify::compare_reals(&knots[span], parameter, policy),
            crate::classify::compare_reals(parameter, &knots[span + 1], policy),
        ) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less)) => return Ok(span),
            (Some(_), Some(_)) => {}
            _ => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
        }
    }
    Err(invalid_interpolation())
}

fn interpolation_solve_error(error: BareissError) -> ExactCurveError {
    match error {
        BareissError::DimensionMismatch => invalid_interpolation(),
        BareissError::UndecidedPivot { .. } => blocked_interpolation(UncertaintyReason::RealSign),
        BareissError::Singular { pivot } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            CurveError::SingularNurbsInterpolation { pivot },
        ),
        BareissError::UnsupportedDivision { pivot } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            CurveError::UnsupportedNurbsInterpolationDivision { index: pivot },
        ),
        BareissError::UnsupportedSolutionDivision { column } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            CurveError::UnsupportedNurbsInterpolationDivision { index: column },
        ),
        BareissError::UnknownResidual => blocked_interpolation(UncertaintyReason::Predicate),
    }
}

fn remap_interpolation_error(error: ExactCurveError) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { cause, .. } => {
            ExactCurveError::invalid(CurveOperation2::Interpolation, CurveFamily2::Nurbs, cause)
        }
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            blocker.reason(),
        ),
    }
}

fn invalid_interpolation() -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Interpolation,
        CurveFamily2::Nurbs,
        CurveError::InvalidNurbsInterpolation,
    )
}

fn blocked_interpolation(reason: UncertaintyReason) -> ExactCurveError {
    ExactCurveError::blocked(CurveOperation2::Interpolation, CurveFamily2::Nurbs, reason)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(x.into(), y.into())
    }

    #[test]
    fn uniform_system_cache_is_exact_and_independent_of_data_coordinates() {
        let first_points = vec![point(0, 0), point(1, 4), point(4, 3), point(6, 0)];
        let second_points = vec![point(-2, 7), point(0, 1), point(5, -3), point(9, 2)];
        let first_system =
            uniform_interpolation_system(3, &first_points, &CurveContext::STRICT).unwrap();
        let second_system =
            uniform_interpolation_system(3, &second_points, &CurveContext::STRICT).unwrap();
        assert!(Arc::ptr_eq(&first_system, &second_system));
        assert!(
            first_system
                .coefficient_matrix
                .iter()
                .flatten()
                .all(|value| value.exact_rational_ref().is_some())
        );

        let curve =
            NurbsCurve2::interpolate_uniform(3, second_points.clone(), &CurveContext::STRICT)
                .unwrap()
                .into_value();
        let parameters = uniform_interpolation_parameters(second_points.len()).unwrap();
        for (parameter, expected) in parameters.iter().zip(second_points) {
            let actual = curve
                .point_at(parameter, &CurveContext::STRICT)
                .unwrap()
                .into_value();
            assert_eq!(actual, expected);
        }
    }
}
