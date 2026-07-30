//! Exact global interpolation for planar NURBS curves.

use hypersolve::{BareissError, determinant_bareiss, solve_dense_linear_system_bareiss_multi_rhs};
use std::cmp::Ordering;
use std::sync::{Arc, Mutex, OnceLock};

use crate::{
    CurveError, CurveFamily2, CurveOperation2, CurvePolicy, ExactCurveError, ExactCurveResult,
    NurbsCurve2, Point2, Real, UncertaintyReason,
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

struct InterpolationCoordinateSolve {
    solution: Vec<Real>,
    residual_replayed: bool,
}

struct InterpolationSystem {
    degree: usize,
    point_count: usize,
    parameters: Arc<[Real]>,
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
    /// top-level NURBS carrier.
    pub fn interpolate_global(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
    ) -> ExactCurveResult<NurbsCurve2> {
        let knots = averaged_interpolation_knots(degree, &data_points, &parameters)?;
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            vec![Real::one(); knots.len() - degree - 1],
            knots,
        )
    }

    /// Globally interpolates exact points at uniformly spaced exact parameters.
    pub fn interpolate_uniform(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsCurve2> {
        let system = uniform_interpolation_system(degree, &data_points)?;
        interpolate_with_precomputed_system(
            degree,
            data_points,
            &system.parameters,
            system.control_weights.iter().cloned().collect(),
            system.knots.iter().cloned().collect(),
            &system.coefficient_matrix,
            &system.denominators,
        )
    }

    /// Globally interpolates using exact Euclidean chord-length parameters.
    pub fn interpolate_chord_length(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsCurve2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            DistanceParameterization::ChordLength,
        )
    }

    /// Globally interpolates using exact centripetal parameters.
    pub fn interpolate_centripetal(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsCurve2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            DistanceParameterization::Centripetal,
        )
    }

    /// Interpolates with explicit exact parameters, control weights, and knots.
    ///
    /// The fixed control weights make this a linear homogeneous interpolation
    /// problem. Every solved coordinate is replayed against the coefficient
    /// matrix by `hypersolve`, then every constructed curve point is replayed
    /// against its authored interpolation constraint.
    pub fn interpolate_with_parameters_and_knots(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
        control_weights: Vec<Real>,
        knots: Vec<Real>,
    ) -> ExactCurveResult<NurbsCurve2> {
        interpolate_with_inputs(degree, data_points, parameters, control_weights, knots)
    }
}

fn interpolate_with_inputs(
    degree: usize,
    data_points: Vec<Point2>,
    parameters: Vec<Real>,
    control_weights: Vec<Real>,
    knots: Vec<Real>,
) -> ExactCurveResult<NurbsCurve2> {
    validate_interpolation_inputs(degree, &data_points, &parameters, &control_weights, &knots)?;
    let coefficient_matrix = build_interpolation_coefficient_matrix(
        degree,
        data_points.len(),
        &parameters,
        &control_weights,
        &knots,
    )?;
    let denominators = interpolation_denominators(&coefficient_matrix)?;
    interpolate_with_precomputed_system(
        degree,
        data_points,
        &parameters,
        control_weights,
        knots,
        &coefficient_matrix,
        &denominators,
    )
}

fn build_interpolation_coefficient_matrix(
    degree: usize,
    point_count: usize,
    parameters: &[Real],
    control_weights: &[Real],
    knots: &[Real],
) -> ExactCurveResult<Vec<Vec<Real>>> {
    parameters
        .iter()
        .map(|parameter| weighted_basis_row(degree, point_count, knots, control_weights, parameter))
        .collect()
}

fn interpolation_denominators(coefficient_matrix: &[Vec<Real>]) -> ExactCurveResult<Vec<Real>> {
    coefficient_matrix
        .iter()
        .map(|row| {
            let denominator = row.iter().fold(Real::zero(), |sum, value| sum + value);
            certify_interpolation_denominator(&denominator)?;
            Ok(denominator)
        })
        .collect()
}

fn interpolate_with_precomputed_system(
    degree: usize,
    data_points: Vec<Point2>,
    parameters: &[Real],
    control_weights: Vec<Real>,
    knots: Vec<Real>,
    coefficient_matrix: &[Vec<Real>],
    denominators: &[Real],
) -> ExactCurveResult<NurbsCurve2> {
    debug_assert_eq!(data_points.len(), coefficient_matrix.len());
    debug_assert_eq!(data_points.len(), denominators.len());
    let mut rhs_x = Vec::with_capacity(data_points.len());
    let mut rhs_y = Vec::with_capacity(data_points.len());
    for (point, denominator) in data_points.iter().zip(denominators) {
        rhs_x.push(point.x() * denominator);
        rhs_y.push(point.y() * denominator);
    }
    let replay_residuals = coefficient_matrix
        .iter()
        .flatten()
        .chain(&rhs_x)
        .chain(&rhs_y)
        .all(|value| value.exact_rational_ref().is_some());
    let (x_solve, y_solve) = if replay_residuals {
        solve_interpolation_coordinates_bareiss(coefficient_matrix, &[rhs_x, rhs_y])?
    } else {
        let determinant = interpolation_determinant(coefficient_matrix)?;
        let x_solve = solve_interpolation_coordinate_cramer_identity(
            coefficient_matrix,
            &rhs_x,
            &determinant,
        )?;
        let y_solve = solve_interpolation_coordinate_cramer_identity(
            coefficient_matrix,
            &rhs_y,
            &determinant,
        )?;
        (x_solve, y_solve)
    };
    let control_points = x_solve
        .solution
        .iter()
        .cloned()
        .zip(y_solve.solution.iter().cloned())
        .map(|(x, y)| Point2::new(x, y))
        .collect::<Vec<_>>();
    let curve = NurbsCurve2::try_new(degree, control_points, control_weights, knots)
        .map_err(remap_interpolation_error)?;
    if x_solve.residual_replayed && y_solve.residual_replayed {
        for (parameter, expected) in parameters.iter().zip(&data_points) {
            let actual = curve
                .point_at(parameter)
                .map_err(remap_interpolation_error)?;
            match exact_point_equal(&actual, expected) {
                Ok(()) => {}
                Err(ExactCurveError::Blocked(_)) => {
                    break;
                }
                Err(error) => return Err(error),
            }
        }
    }
    Ok(curve)
}

fn uniform_interpolation_system(
    degree: usize,
    data_points: &[Point2],
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
    let knots = averaged_interpolation_knots(degree, data_points, &parameters)?;
    let control_weights = vec![Real::one(); knots.len() - degree - 1];
    validate_interpolation_inputs(degree, data_points, &parameters, &control_weights, &knots)?;
    let coefficient_matrix = build_interpolation_coefficient_matrix(
        degree,
        point_count,
        &parameters,
        &control_weights,
        &knots,
    )?;
    let denominators = interpolation_denominators(&coefficient_matrix)?;
    let system = Arc::new(InterpolationSystem {
        degree,
        point_count,
        parameters: parameters.into(),
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

fn interpolation_determinant(coefficient_matrix: &[Vec<Real>]) -> ExactCurveResult<Real> {
    let evidence = determinant_bareiss(coefficient_matrix, INTERPOLATION_SOLVE_PRECISION)
        .map_err(interpolation_solve_error)?;
    match crate::classify::compare_reals(
        &evidence.determinant,
        &Real::zero(),
        &CurvePolicy::certified(),
    ) {
        Some(Ordering::Less | Ordering::Greater) => Ok(evidence.determinant),
        Some(Ordering::Equal) => Err(ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            CurveError::SingularNurbsInterpolation {
                pivot: coefficient_matrix.len().saturating_sub(1),
            },
        )),
        None => Err(blocked_interpolation(UncertaintyReason::RealSign)),
    }
}

fn solve_interpolation_coordinates_bareiss(
    coefficient_matrix: &[Vec<Real>],
    right_hand_sides: &[Vec<Real>; 2],
) -> ExactCurveResult<(InterpolationCoordinateSolve, InterpolationCoordinateSolve)> {
    let evidence = solve_dense_linear_system_bareiss_multi_rhs(
        coefficient_matrix,
        right_hand_sides,
        INTERPOLATION_SOLVE_PRECISION,
    )
    .map_err(interpolation_solve_error)?;
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
    let mut solutions = evidence.solutions.into_iter();
    let x_solve = InterpolationCoordinateSolve {
        solution: solutions
            .next()
            .expect("two right-hand sides were supplied"),
        residual_replayed: true,
    };
    let y_solve = InterpolationCoordinateSolve {
        solution: solutions
            .next()
            .expect("two right-hand sides were supplied"),
        residual_replayed: true,
    };
    Ok((x_solve, y_solve))
}

fn solve_interpolation_coordinate_cramer_identity(
    coefficient_matrix: &[Vec<Real>],
    rhs: &[Real],
    determinant: &Real,
) -> ExactCurveResult<InterpolationCoordinateSolve> {
    let mut replaced = coefficient_matrix.to_vec();
    let mut solution = Vec::with_capacity(coefficient_matrix.len());
    for column in 0..coefficient_matrix.len() {
        for (row, value) in rhs.iter().enumerate() {
            replaced[row][column] = value.clone();
        }
        let numerator = determinant_bareiss(&replaced, INTERPOLATION_SOLVE_PRECISION)
            .map_err(interpolation_solve_error)?
            .determinant;
        let value = (numerator.clone() / determinant.clone()).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                CurveError::UnsupportedNurbsInterpolationDivision { index: column },
            )
        })?;
        solution.push(value);
        for (row, coefficients) in coefficient_matrix.iter().enumerate() {
            replaced[row][column] = coefficients[column].clone();
        }
    }

    Ok(InterpolationCoordinateSolve {
        solution,
        residual_replayed: false,
    })
}

fn interpolate_distance_parameterized(
    degree: usize,
    data_points: Vec<Point2>,
    parameterization: DistanceParameterization,
) -> ExactCurveResult<NurbsCurve2> {
    let parameters = distance_interpolation_parameters(&data_points, parameterization)?;
    let knots = averaged_interpolation_knots(degree, &data_points, &parameters)?;
    interpolate_with_inputs(
        degree,
        data_points,
        parameters,
        vec![Real::one(); knots.len() - degree - 1],
        knots,
    )
}

fn distance_interpolation_parameters(
    data_points: &[Point2],
    parameterization: DistanceParameterization,
) -> ExactCurveResult<Vec<Real>> {
    if data_points.len() < 2 {
        return Err(invalid_interpolation());
    }
    let policy = CurvePolicy::certified();
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
        match crate::classify::compare_reals(&Real::zero(), &increment, &policy) {
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

fn certify_interpolation_denominator(denominator: &Real) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match crate::classify::compare_reals(denominator, &Real::zero(), &policy) {
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
) -> ExactCurveResult<Vec<Real>> {
    if degree < 1 || data_points.len() != parameters.len() || data_points.len() <= degree {
        return Err(invalid_interpolation());
    }
    validate_strict_parameters(parameters)?;
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
    validate_strict_parameters(parameters)?;
    let policy = CurvePolicy::certified();
    for pair in knots.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], &policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return Err(invalid_interpolation()),
            None => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
        }
    }
    match (
        crate::classify::compare_reals(&parameters[0], &knots[degree], &policy),
        crate::classify::compare_reals(&parameters[point_count - 1], &knots[point_count], &policy),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(()),
        (Some(_), Some(_)) => Err(invalid_interpolation()),
        _ => Err(blocked_interpolation(UncertaintyReason::Ordering)),
    }
}

fn validate_strict_parameters(parameters: &[Real]) -> ExactCurveResult<()> {
    if parameters.len() < 2 {
        return Err(invalid_interpolation());
    }
    let policy = CurvePolicy::certified();
    for pair in parameters.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], &policy) {
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
) -> ExactCurveResult<Vec<Real>> {
    let span = interpolation_span(degree, control_count, knots, parameter)?;
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
) -> ExactCurveResult<usize> {
    let policy = CurvePolicy::certified();
    let last_control = control_count - 1;
    match crate::classify::compare_reals(parameter, &knots[control_count], &policy) {
        Some(Ordering::Equal) => return Ok(last_control),
        Some(_) => {}
        None => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
    }
    for span in degree..=last_control {
        match (
            crate::classify::compare_reals(&knots[span], parameter, &policy),
            crate::classify::compare_reals(parameter, &knots[span + 1], &policy),
        ) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less)) => return Ok(span),
            (Some(_), Some(_)) => {}
            _ => return Err(blocked_interpolation(UncertaintyReason::Ordering)),
        }
    }
    Err(invalid_interpolation())
}

fn exact_scalar_equal(first: &Real, second: &Real) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match crate::classify::compare_reals(first, second, &policy) {
        Some(Ordering::Equal) => Ok(()),
        Some(_) => Err(invalid_interpolation()),
        None => Err(blocked_interpolation(UncertaintyReason::RealSign)),
    }
}

fn exact_point_equal(first: &Point2, second: &Point2) -> ExactCurveResult<()> {
    exact_scalar_equal(first.x(), second.x())?;
    exact_scalar_equal(first.y(), second.y())
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
        let first_system = uniform_interpolation_system(3, &first_points).unwrap();
        let second_system = uniform_interpolation_system(3, &second_points).unwrap();
        assert!(Arc::ptr_eq(&first_system, &second_system));
        assert!(
            first_system
                .coefficient_matrix
                .iter()
                .flatten()
                .all(|value| value.exact_rational_ref().is_some())
        );

        let curve = NurbsCurve2::interpolate_uniform(3, second_points.clone()).unwrap();
        for (parameter, expected) in first_system.parameters.iter().zip(second_points) {
            let actual = curve.point_at(parameter).unwrap();
            exact_point_equal(&actual, &expected).unwrap();
        }
    }
}
