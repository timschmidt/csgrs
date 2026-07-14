//! Exact global interpolation for planar NURBS curves.

use std::cmp::Ordering;
use std::rc::Rc;

use hyperreal::RealSign;
use hypersolve::{
    BareissError, DenseResidualReplayError, determinant_bareiss, replay_dense_linear_residuals,
};

use crate::{
    CurveError, CurveFamily2, CurveOperation2, CurvePolicy, CurveSource2, ExactCurveError,
    ExactCurveResult, NurbsCurve2, Point2, Real, UncertaintyReason,
};

const INTERPOLATION_SOLVE_PRECISION: i32 = -128;

/// Exact linear solve used for global NURBS interpolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NurbsInterpolationSolvePath2 {
    /// Fraction-free Bareiss/Cramer construction with every matrix and curve constraint replayed.
    DenseBareissCramerResidualReplay,
    /// Fraction-free Bareiss/Cramer construction certified by the nonzero determinant identity.
    ///
    /// This path is retained when the exact scalar package cannot normalize a
    /// mathematically zero symbolic residual, such as nested-radical
    /// centripetal parameter expressions.
    DenseBareissCramerIdentity,
}

/// Exact parameter construction used for global interpolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NurbsInterpolationParameterization2 {
    /// Parameters were supplied exactly by the caller.
    AuthoredExact,
    /// Parameters are `i / n` over the closed unit interval.
    Uniform,
    /// Parameter increments are exact Euclidean chord lengths.
    ChordLength,
    /// Parameter increments are square roots of exact Euclidean chord lengths.
    Centripetal,
}

/// Retained exact evidence for one global NURBS interpolation.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsInterpolationReport2 {
    degree: usize,
    source: Option<CurveSource2>,
    data_points: Rc<[Point2]>,
    parameters: Rc<[Real]>,
    control_weights: Rc<[Real]>,
    knots: Rc<[Real]>,
    coefficient_matrix: Rc<[Vec<Real>]>,
    determinant: Real,
    x_numerators: Rc<[Real]>,
    y_numerators: Rc<[Real]>,
    parameterization: NurbsInterpolationParameterization2,
    solve_path: NurbsInterpolationSolvePath2,
}

/// Exact interpolated NURBS and its retained construction proof.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsInterpolation2 {
    curve: NurbsCurve2,
    report: NurbsInterpolationReport2,
}

struct InterpolationCoordinateSolve {
    solution: Vec<Real>,
    numerators: Vec<Real>,
    residual_replayed: bool,
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
    ) -> ExactCurveResult<NurbsInterpolation2> {
        let knots = averaged_interpolation_knots(degree, &data_points, &parameters, None)?;
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            vec![Real::one(); knots.len() - degree - 1],
            knots,
            None,
            NurbsInterpolationParameterization2::AuthoredExact,
        )
    }

    /// Globally interpolates exact points with stable source provenance.
    pub fn interpolate_global_with_source(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        let knots = averaged_interpolation_knots(degree, &data_points, &parameters, Some(source))?;
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            vec![Real::one(); knots.len() - degree - 1],
            knots,
            Some(source),
            NurbsInterpolationParameterization2::AuthoredExact,
        )
    }

    /// Globally interpolates exact points at uniformly spaced exact parameters.
    pub fn interpolate_uniform(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        let parameters = uniform_interpolation_parameters(data_points.len(), None)?;
        let knots = averaged_interpolation_knots(degree, &data_points, &parameters, None)?;
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            vec![Real::one(); knots.len() - degree - 1],
            knots,
            None,
            NurbsInterpolationParameterization2::Uniform,
        )
    }

    /// Uniformly interpolates exact points with stable source provenance.
    pub fn interpolate_uniform_with_source(
        degree: usize,
        data_points: Vec<Point2>,
        source: CurveSource2,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        let parameters = uniform_interpolation_parameters(data_points.len(), Some(source))?;
        let knots = averaged_interpolation_knots(degree, &data_points, &parameters, Some(source))?;
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            vec![Real::one(); knots.len() - degree - 1],
            knots,
            Some(source),
            NurbsInterpolationParameterization2::Uniform,
        )
    }

    /// Globally interpolates using exact Euclidean chord-length parameters.
    pub fn interpolate_chord_length(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            None,
            NurbsInterpolationParameterization2::ChordLength,
        )
    }

    /// Chord-length interpolation with stable source provenance.
    pub fn interpolate_chord_length_with_source(
        degree: usize,
        data_points: Vec<Point2>,
        source: CurveSource2,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            Some(source),
            NurbsInterpolationParameterization2::ChordLength,
        )
    }

    /// Globally interpolates using exact centripetal parameters.
    pub fn interpolate_centripetal(
        degree: usize,
        data_points: Vec<Point2>,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            None,
            NurbsInterpolationParameterization2::Centripetal,
        )
    }

    /// Centripetal interpolation with stable source provenance.
    pub fn interpolate_centripetal_with_source(
        degree: usize,
        data_points: Vec<Point2>,
        source: CurveSource2,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_distance_parameterized(
            degree,
            data_points,
            Some(source),
            NurbsInterpolationParameterization2::Centripetal,
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
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            control_weights,
            knots,
            None,
            NurbsInterpolationParameterization2::AuthoredExact,
        )
    }

    /// Explicit exact interpolation with stable source provenance.
    pub fn interpolate_with_parameters_and_knots_with_source(
        degree: usize,
        data_points: Vec<Point2>,
        parameters: Vec<Real>,
        control_weights: Vec<Real>,
        knots: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<NurbsInterpolation2> {
        interpolate_with_inputs(
            degree,
            data_points,
            parameters,
            control_weights,
            knots,
            Some(source),
            NurbsInterpolationParameterization2::AuthoredExact,
        )
    }
}

impl NurbsInterpolation2 {
    /// Returns the exact interpolated NURBS.
    pub const fn curve(&self) -> &NurbsCurve2 {
        &self.curve
    }

    /// Returns retained interpolation construction evidence.
    pub const fn report(&self) -> &NurbsInterpolationReport2 {
        &self.report
    }

    /// Consumes this result and returns the exact interpolated NURBS.
    pub fn into_curve(self) -> NurbsCurve2 {
        self.curve
    }

    /// Consumes this result and returns the curve and retained report.
    pub fn into_parts(self) -> (NurbsCurve2, NurbsInterpolationReport2) {
        (self.curve, self.report)
    }
}

impl NurbsInterpolationReport2 {
    /// Returns the interpolated NURBS degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns stable source identity when supplied.
    pub const fn source(&self) -> Option<CurveSource2> {
        self.source
    }

    /// Returns the exact interpolation point constraints.
    pub fn data_points(&self) -> &[Point2] {
        &self.data_points
    }

    /// Returns exact interpolation parameters in authored order.
    pub fn parameters(&self) -> &[Real] {
        &self.parameters
    }

    /// Returns fixed exact control weights used by the linear solve.
    pub fn control_weights(&self) -> &[Real] {
        &self.control_weights
    }

    /// Returns the exact clamped or explicitly authored knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.knots
    }

    /// Returns the exact weighted B-spline coefficient matrix.
    pub fn coefficient_matrix(&self) -> &[Vec<Real>] {
        &self.coefficient_matrix
    }

    /// Returns the certified nonzero matrix determinant.
    pub const fn determinant(&self) -> &Real {
        &self.determinant
    }

    /// Returns exact Cramer numerators for the affine x controls.
    pub fn x_numerators(&self) -> &[Real] {
        &self.x_numerators
    }

    /// Returns exact Cramer numerators for the affine y controls.
    pub fn y_numerators(&self) -> &[Real] {
        &self.y_numerators
    }

    /// Returns how exact interpolation parameters were constructed.
    pub const fn parameterization(&self) -> NurbsInterpolationParameterization2 {
        self.parameterization
    }

    /// Returns the exact solver path used for construction and replay.
    pub const fn solve_path(&self) -> NurbsInterpolationSolvePath2 {
        self.solve_path
    }
}

fn interpolate_with_inputs(
    degree: usize,
    data_points: Vec<Point2>,
    parameters: Vec<Real>,
    control_weights: Vec<Real>,
    knots: Vec<Real>,
    source: Option<CurveSource2>,
    parameterization: NurbsInterpolationParameterization2,
) -> ExactCurveResult<NurbsInterpolation2> {
    validate_interpolation_inputs(
        degree,
        &data_points,
        &parameters,
        &control_weights,
        &knots,
        source,
    )?;
    let coefficient_matrix = parameters
        .iter()
        .map(|parameter| {
            weighted_basis_row(
                degree,
                data_points.len(),
                &knots,
                &control_weights,
                parameter,
                source,
            )
        })
        .collect::<ExactCurveResult<Vec<_>>>()?;
    let mut rhs_x = Vec::with_capacity(data_points.len());
    let mut rhs_y = Vec::with_capacity(data_points.len());
    for (point, row) in data_points.iter().zip(&coefficient_matrix) {
        let denominator = row.iter().fold(Real::zero(), |sum, value| sum + value);
        certify_interpolation_denominator(&denominator, source)?;
        rhs_x.push(point.x() * &denominator);
        rhs_y.push(point.y() * denominator);
    }
    let replay_residuals = coefficient_matrix
        .iter()
        .flatten()
        .chain(&rhs_x)
        .chain(&rhs_y)
        .all(|value| value.exact_rational_ref().is_some());
    let determinant = interpolation_determinant(&coefficient_matrix, source)?;
    let x_solve = solve_interpolation_coordinate(
        &coefficient_matrix,
        &rhs_x,
        &determinant,
        replay_residuals,
        source,
    )?;
    let y_solve = solve_interpolation_coordinate(
        &coefficient_matrix,
        &rhs_y,
        &determinant,
        replay_residuals,
        source,
    )?;
    let control_points = x_solve
        .solution
        .iter()
        .cloned()
        .zip(y_solve.solution.iter().cloned())
        .map(|(x, y)| Point2::new(x, y))
        .collect::<Vec<_>>();
    let curve = match source {
        Some(source) => NurbsCurve2::try_new_with_source(
            degree,
            control_points,
            control_weights.clone(),
            knots.clone(),
            source,
        ),
        None => NurbsCurve2::try_new(
            degree,
            control_points,
            control_weights.clone(),
            knots.clone(),
        ),
    }
    .map_err(|error| remap_interpolation_error(error, source))?;
    let mut solve_path = if x_solve.residual_replayed && y_solve.residual_replayed {
        NurbsInterpolationSolvePath2::DenseBareissCramerResidualReplay
    } else {
        NurbsInterpolationSolvePath2::DenseBareissCramerIdentity
    };
    if solve_path == NurbsInterpolationSolvePath2::DenseBareissCramerResidualReplay {
        for (parameter, expected) in parameters.iter().zip(&data_points) {
            let actual = curve
                .point_at(parameter)
                .map_err(|error| remap_interpolation_error(error, source))?;
            match exact_point_equal(&actual, expected, source) {
                Ok(()) => {}
                Err(ExactCurveError::Blocked(_)) => {
                    solve_path = NurbsInterpolationSolvePath2::DenseBareissCramerIdentity;
                    break;
                }
                Err(error) => return Err(error),
            }
        }
    }
    Ok(NurbsInterpolation2 {
        curve,
        report: NurbsInterpolationReport2 {
            degree,
            source,
            data_points: data_points.into(),
            parameters: parameters.into(),
            control_weights: control_weights.into(),
            knots: knots.into(),
            coefficient_matrix: coefficient_matrix.into(),
            determinant,
            x_numerators: x_solve.numerators.into(),
            y_numerators: y_solve.numerators.into(),
            parameterization,
            solve_path,
        },
    })
}

fn interpolation_determinant(
    coefficient_matrix: &[Vec<Real>],
    source: Option<CurveSource2>,
) -> ExactCurveResult<Real> {
    let report = determinant_bareiss(coefficient_matrix, INTERPOLATION_SOLVE_PRECISION)
        .map_err(|error| interpolation_solve_error(error, source))?;
    match crate::classify::compare_reals(
        &report.determinant,
        &Real::zero(),
        &CurvePolicy::certified(),
    ) {
        Some(Ordering::Less | Ordering::Greater) => Ok(report.determinant),
        Some(Ordering::Equal) => Err(ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            CurveError::SingularNurbsInterpolation {
                pivot: coefficient_matrix.len().saturating_sub(1),
            },
        )),
        None => Err(blocked_interpolation(source, UncertaintyReason::RealSign)),
    }
}

fn solve_interpolation_coordinate(
    coefficient_matrix: &[Vec<Real>],
    rhs: &[Real],
    determinant: &Real,
    replay_residuals: bool,
    source: Option<CurveSource2>,
) -> ExactCurveResult<InterpolationCoordinateSolve> {
    let mut replaced = coefficient_matrix.to_vec();
    let mut numerators = Vec::with_capacity(coefficient_matrix.len());
    let mut solution = Vec::with_capacity(coefficient_matrix.len());
    for column in 0..coefficient_matrix.len() {
        for (row, value) in rhs.iter().enumerate() {
            replaced[row][column] = value.clone();
        }
        let numerator = determinant_bareiss(&replaced, INTERPOLATION_SOLVE_PRECISION)
            .map_err(|error| interpolation_solve_error(error, source))?
            .determinant;
        let value = (numerator.clone() / determinant.clone()).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                source,
                CurveError::UnsupportedNurbsInterpolationDivision { index: column },
            )
        })?;
        numerators.push(numerator);
        solution.push(value);
        for (row, coefficients) in coefficient_matrix.iter().enumerate() {
            replaced[row][column] = coefficients[column].clone();
        }
    }

    let residual_replayed = if replay_residuals {
        match replay_dense_linear_residuals(
            coefficient_matrix,
            rhs,
            &solution,
            INTERPOLATION_SOLVE_PRECISION,
        ) {
            Ok(report) if report.accepted => true,
            Ok(report) => {
                let row = report
                    .rows
                    .iter()
                    .find(|row| row.sign != RealSign::Zero)
                    .map_or(0, |row| row.row_index);
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    source,
                    CurveError::InconsistentNurbsInterpolationSolution { row },
                ));
            }
            Err(DenseResidualReplayError::UnknownResidual) => false,
            Err(DenseResidualReplayError::DimensionMismatch) => {
                return Err(invalid_interpolation(source));
            }
        }
    } else {
        false
    };

    Ok(InterpolationCoordinateSolve {
        solution,
        numerators,
        residual_replayed,
    })
}

fn interpolate_distance_parameterized(
    degree: usize,
    data_points: Vec<Point2>,
    source: Option<CurveSource2>,
    parameterization: NurbsInterpolationParameterization2,
) -> ExactCurveResult<NurbsInterpolation2> {
    let parameters = distance_interpolation_parameters(&data_points, source, parameterization)?;
    let knots = averaged_interpolation_knots(degree, &data_points, &parameters, source)?;
    interpolate_with_inputs(
        degree,
        data_points,
        parameters,
        vec![Real::one(); knots.len() - degree - 1],
        knots,
        source,
        parameterization,
    )
}

fn distance_interpolation_parameters(
    data_points: &[Point2],
    source: Option<CurveSource2>,
    parameterization: NurbsInterpolationParameterization2,
) -> ExactCurveResult<Vec<Real>> {
    if data_points.len() < 2 {
        return Err(invalid_interpolation(source));
    }
    let policy = CurvePolicy::certified();
    let mut increments = Vec::with_capacity(data_points.len() - 1);
    for pair in data_points.windows(2) {
        let chord = pair[0].distance_squared(&pair[1]).sqrt().map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                source,
                cause.into(),
            )
        })?;
        let increment = match parameterization {
            NurbsInterpolationParameterization2::ChordLength => chord,
            NurbsInterpolationParameterization2::Centripetal => chord.sqrt().map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    source,
                    cause.into(),
                )
            })?,
            NurbsInterpolationParameterization2::AuthoredExact
            | NurbsInterpolationParameterization2::Uniform => {
                return Err(invalid_interpolation(source));
            }
        };
        match crate::classify::compare_reals(&Real::zero(), &increment, &policy) {
            Some(Ordering::Less) => increments.push(increment),
            Some(_) => return Err(invalid_interpolation(source)),
            None => return Err(blocked_interpolation(source, UncertaintyReason::RealSign)),
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
                source,
                cause.into(),
            )
        })?);
    }
    parameters.push(Real::one());
    Ok(parameters)
}

fn certify_interpolation_denominator(
    denominator: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match crate::classify::compare_reals(denominator, &Real::zero(), &policy) {
        Some(Ordering::Less | Ordering::Greater) => Ok(()),
        Some(Ordering::Equal) => Err(ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            CurveError::ZeroNurbsDenominator,
        )),
        None => Err(blocked_interpolation(source, UncertaintyReason::RealSign)),
    }
}

fn averaged_interpolation_knots(
    degree: usize,
    data_points: &[Point2],
    parameters: &[Real],
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<Real>> {
    if degree < 1 || data_points.len() != parameters.len() || data_points.len() <= degree {
        return Err(invalid_interpolation(source));
    }
    validate_strict_parameters(parameters, source)?;
    let mut knots = Vec::with_capacity(data_points.len() + degree + 1);
    knots.extend(std::iter::repeat_n(parameters[0].clone(), degree + 1));
    let divisor = interpolation_usize_real(degree, source)?;
    for first in 1..data_points.len() - degree {
        let sum = parameters[first..first + degree]
            .iter()
            .fold(Real::zero(), |sum, parameter| sum + parameter);
        knots.push((sum / divisor.clone()).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::Interpolation,
                CurveFamily2::Nurbs,
                source,
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

fn uniform_interpolation_parameters(
    point_count: usize,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<Real>> {
    if point_count < 2 {
        return Err(invalid_interpolation(source));
    }
    let denominator = interpolation_usize_real(point_count - 1, source)?;
    (0..point_count)
        .map(|index| {
            (interpolation_usize_real(index, source)? / denominator.clone()).map_err(|_| {
                ExactCurveError::invalid(
                    CurveOperation2::Interpolation,
                    CurveFamily2::Nurbs,
                    source,
                    CurveError::UnsupportedNurbsInterpolationDivision { index },
                )
            })
        })
        .collect()
}

fn interpolation_usize_real(value: usize, source: Option<CurveSource2>) -> ExactCurveResult<Real> {
    u64::try_from(value)
        .map(Real::from)
        .map_err(|_| invalid_interpolation(source))
}

fn validate_interpolation_inputs(
    degree: usize,
    data_points: &[Point2],
    parameters: &[Real],
    control_weights: &[Real],
    knots: &[Real],
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    let point_count = data_points.len();
    if degree < 1
        || point_count <= degree
        || parameters.len() != point_count
        || control_weights.len() != point_count
        || knots.len() != point_count + degree + 1
    {
        return Err(invalid_interpolation(source));
    }
    validate_strict_parameters(parameters, source)?;
    let policy = CurvePolicy::certified();
    for pair in knots.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], &policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return Err(invalid_interpolation(source)),
            None => return Err(blocked_interpolation(source, UncertaintyReason::Ordering)),
        }
    }
    match (
        crate::classify::compare_reals(&parameters[0], &knots[degree], &policy),
        crate::classify::compare_reals(&parameters[point_count - 1], &knots[point_count], &policy),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(()),
        (Some(_), Some(_)) => Err(invalid_interpolation(source)),
        _ => Err(blocked_interpolation(source, UncertaintyReason::Ordering)),
    }
}

fn validate_strict_parameters(
    parameters: &[Real],
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    if parameters.len() < 2 {
        return Err(invalid_interpolation(source));
    }
    let policy = CurvePolicy::certified();
    for pair in parameters.windows(2) {
        match crate::classify::compare_reals(&pair[0], &pair[1], &policy) {
            Some(Ordering::Less) => {}
            Some(_) => return Err(invalid_interpolation(source)),
            None => return Err(blocked_interpolation(source, UncertaintyReason::Ordering)),
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<Real>> {
    let span = interpolation_span(degree, control_count, knots, parameter, source)?;
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
                    source,
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<usize> {
    let policy = CurvePolicy::certified();
    let last_control = control_count - 1;
    match crate::classify::compare_reals(parameter, &knots[control_count], &policy) {
        Some(Ordering::Equal) => return Ok(last_control),
        Some(_) => {}
        None => return Err(blocked_interpolation(source, UncertaintyReason::Ordering)),
    }
    for span in degree..=last_control {
        match (
            crate::classify::compare_reals(&knots[span], parameter, &policy),
            crate::classify::compare_reals(parameter, &knots[span + 1], &policy),
        ) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less)) => return Ok(span),
            (Some(_), Some(_)) => {}
            _ => return Err(blocked_interpolation(source, UncertaintyReason::Ordering)),
        }
    }
    Err(invalid_interpolation(source))
}

fn exact_scalar_equal(
    first: &Real,
    second: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match crate::classify::compare_reals(first, second, &policy) {
        Some(Ordering::Equal) => Ok(()),
        Some(_) => Err(invalid_interpolation(source)),
        None => Err(blocked_interpolation(source, UncertaintyReason::RealSign)),
    }
}

fn exact_point_equal(
    first: &Point2,
    second: &Point2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    exact_scalar_equal(first.x(), second.x(), source)?;
    exact_scalar_equal(first.y(), second.y(), source)
}

fn interpolation_solve_error(error: BareissError, source: Option<CurveSource2>) -> ExactCurveError {
    match error {
        BareissError::DimensionMismatch => invalid_interpolation(source),
        BareissError::UndecidedPivot { .. } => {
            blocked_interpolation(source, UncertaintyReason::RealSign)
        }
        BareissError::Singular { pivot } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            CurveError::SingularNurbsInterpolation { pivot },
        ),
        BareissError::UnsupportedDivision { pivot } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            CurveError::UnsupportedNurbsInterpolationDivision { index: pivot },
        ),
        BareissError::UnsupportedSolutionDivision { column } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            CurveError::UnsupportedNurbsInterpolationDivision { index: column },
        ),
        BareissError::UnknownResidual => {
            blocked_interpolation(source, UncertaintyReason::Predicate)
        }
    }
}

fn remap_interpolation_error(
    error: ExactCurveError,
    source: Option<CurveSource2>,
) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { cause, .. } => ExactCurveError::invalid(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            cause,
        ),
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            CurveOperation2::Interpolation,
            CurveFamily2::Nurbs,
            source,
            blocker.reason(),
        ),
    }
}

fn invalid_interpolation(source: Option<CurveSource2>) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Interpolation,
        CurveFamily2::Nurbs,
        source,
        CurveError::InvalidNurbsInterpolation,
    )
}

fn blocked_interpolation(
    source: Option<CurveSource2>,
    reason: UncertaintyReason,
) -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Interpolation,
        CurveFamily2::Nurbs,
        source,
        reason,
    )
}
