//! Exact resultant helpers for low-degree parametric curve intersections.
//!
//! Curve crates own topology, span subdivision, and curve-specific validity
//! rules. This module supplies the bounded exact algebraic handoff they need:
//! two polynomial parametric curves in power basis are substituted into
//! `x1(t) - x2(u)` and `y1(t) - y2(u)`, the second parameter is eliminated by
//! Sylvester resultants, and the retained-parameter polynomial is reconstructed
//! by exact interpolation. This follows the elimination view of Bezier clipping, while keeping
//! the exact-computation boundary explicit: the resultant is candidate
//! algebraic evidence, not an accepted topology event, until a downstream
//! curve package replays it against retained geometry.

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::bareiss::{BareissError, determinant_bareiss};
use crate::curve_substitution::RationalParametricCurve2;
use crate::resultant::{
    UnivariateResultantError, resultant_univariate_polynomials, sylvester_matrix,
};

/// Selects which curve parameter remains after exact elimination.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveResultantParameter {
    /// Retain the first curve's parameter.
    First,
    /// Retain the second curve's parameter.
    Second,
}

/// Power-basis representation of a 2D polynomial parametric curve.
///
/// Coefficients are stored in ascending power order. For example,
/// `x_coefficients = [1, 2, 3]` means `x(t) = 1 + 2*t + 3*t^2`.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialParametricCurve2 {
    /// Power-basis coefficients for the x-coordinate.
    pub x_coefficients: Vec<Real>,
    /// Power-basis coefficients for the y-coordinate.
    pub y_coefficients: Vec<Real>,
}

impl PolynomialParametricCurve2 {
    /// Construct a 2D polynomial parametric curve in power basis.
    pub const fn new(x_coefficients: Vec<Real>, y_coefficients: Vec<Real>) -> Self {
        Self {
            x_coefficients,
            y_coefficients,
        }
    }
}

/// Exact polynomial in two parameters.
///
/// `coefficients[first_power][second_power]` multiplies
/// `first_parameter^first_power * second_parameter^second_power`. Rows may be
/// ragged; omitted coefficients are zero.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariatePolynomial {
    /// Coefficient grid in ascending powers of both parameters.
    pub coefficients: Vec<Vec<Real>>,
}

impl BivariatePolynomial {
    /// Constructs a bivariate polynomial from its ascending-power grid.
    pub const fn new(coefficients: Vec<Vec<Real>>) -> Self {
        Self { coefficients }
    }
}

/// Configuration for bounded exact curve resultant construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurveIntersectionResultantConfig {
    /// Precision bound used when coefficient signs are certified.
    pub min_precision: i32,
    /// Maximum retained-parameter degree this bounded helper may interpolate.
    pub max_resultant_degree: usize,
}

impl Default for CurveIntersectionResultantConfig {
    fn default() -> Self {
        Self {
            min_precision: hyperlimit::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            max_resultant_degree: 32,
        }
    }
}

/// Final status for a curve intersection resultant report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveIntersectionResultantStatus {
    /// The retained-parameter resultant polynomial was constructed exactly.
    Constructed,
    /// At least one coordinate polynomial was empty.
    EmptyCoordinatePolynomial,
    /// A coefficient needed for degree trimming could not be certified.
    UndecidedCoefficient,
    /// The conservative resultant degree bound exceeded the configured budget.
    DegreeBoundExceeded,
    /// A sampled Sylvester resultant failed.
    ResultantError,
    /// Exact interpolation required a division that failed.
    InterpolationDivisionFailed,
    /// A homogeneous rational curve had certified zero weight everywhere.
    InvalidHomogeneousWeight,
}

/// One exact sample used to reconstruct the retained-parameter resultant.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionResultantSample {
    /// Retained parameter value used for this sample.
    pub parameter_value: Real,
    /// Exact sampled resultant after eliminating the other parameter.
    pub resultant: Real,
}

/// Exact report for eliminating one parameter from two parametric curves.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionResultantReport {
    /// Final construction status.
    pub status: CurveIntersectionResultantStatus,
    /// Parameter that remains in `resultant_coefficients`.
    pub retained_parameter: CurveResultantParameter,
    /// Parameter eliminated by the Sylvester resultant.
    pub eliminated_parameter: CurveResultantParameter,
    /// Conservative degree bound used to choose interpolation samples.
    pub degree_bound: usize,
    /// Sampled exact resultants in retained-parameter order.
    pub samples: Vec<CurveIntersectionResultantSample>,
    /// Resultant polynomial coefficients in ascending retained-parameter power
    /// order. Empty unless `status == Constructed`.
    pub resultant_coefficients: Vec<Real>,
    /// Sampled resultant error, if construction failed at that boundary.
    pub resultant_error: Option<UnivariateResultantError>,
}

/// Final status for a rational lift of the eliminated system parameter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveIntersectionParameterLiftStatus {
    /// Every potentially useful adjugate ratio was constructed exactly.
    Constructed,
    /// At least one bivariate equation was empty.
    EmptyEquation,
    /// A coefficient needed for degree certification remained undecided.
    UndecidedCoefficient,
    /// One equation is constant in the eliminated parameter, so a linear
    /// common-root lift cannot certify the pairing.
    UnsupportedEliminatedDegree,
    /// The conservative cofactor degree exceeded the configured budget.
    DegreeBoundExceeded,
    /// An exact cofactor determinant could not be constructed.
    DeterminantError,
    /// Exact cofactor interpolation or trimming did not complete.
    InterpolationFailed,
}

/// One exact adjugate ratio for lifting the eliminated parameter.
///
/// At a retained root where the specialized Sylvester matrix has nullity one,
/// at least one returned denominator is nonzero and
/// `eliminated = numerator(retained) / denominator(retained)`.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameterLiftMap {
    /// Sylvester row whose two signed cofactors define this ratio.
    pub cofactor_row: usize,
    /// Numerator coefficients in ascending retained-parameter power order.
    pub numerator_coefficients: Vec<Real>,
    /// Denominator coefficients in ascending retained-parameter power order.
    pub denominator_coefficients: Vec<Real>,
}

/// Exact report for linear common-root lifts of one bivariate system parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameterLiftReport {
    /// Final construction status.
    pub status: CurveIntersectionParameterLiftStatus,
    /// Parameter supplied to every returned rational map.
    pub retained_parameter: CurveResultantParameter,
    /// Parameter produced by every returned rational map.
    pub lifted_parameter: CurveResultantParameter,
    /// Conservative interpolation degree used for every signed cofactor.
    pub degree_bound: usize,
    /// Candidate adjugate ratios. Identically zero denominators are omitted.
    pub maps: Vec<CurveIntersectionParameterLiftMap>,
    /// Exact determinant failure, when construction stopped at that boundary.
    pub determinant_error: Option<BareissError>,
}

/// Eliminates one parameter from two exact bivariate polynomial equations.
///
/// Both equations use [`BivariatePolynomial`]'s `(first, second)` coefficient
/// order. The implementation certifies their bidegrees, evaluates exact
/// univariate specializations, skips the finitely many specializations whose
/// eliminated-variable degree drops, and reconstructs the resultant by exact
/// interpolation. The returned polynomial is candidate evidence only; callers
/// must replay isolated roots against their original equations and geometric
/// branch conditions.
pub fn resultant_bivariate_polynomial_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    let eliminated_parameter = match retained_parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    };
    if bivariate_polynomial_is_empty(first_equation)
        || bivariate_polynomial_is_empty(second_equation)
    {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            eliminated_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    let Ok(first_degree) = certified_bivariate_degree(first_equation, config.min_precision) else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(second_degree) = certified_bivariate_degree(second_equation, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    if first_degree.is_none() || second_degree.is_none() {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::Constructed,
            retained_parameter,
            eliminated_parameter,
            0,
            vec![CurveIntersectionResultantSample {
                parameter_value: Real::zero(),
                resultant: Real::zero(),
            }],
            vec![Real::zero()],
            None,
        );
    }
    let first_degree = first_degree.expect("nonzero bivariate equation has a bidegree");
    let second_degree = second_degree.expect("nonzero bivariate equation has a bidegree");
    let (first_retained_degree, first_eliminated_degree) =
        oriented_bidegree(first_degree, retained_parameter);
    let (second_retained_degree, second_eliminated_degree) =
        oriented_bidegree(second_degree, retained_parameter);
    let degree_bound = second_eliminated_degree * first_retained_degree
        + first_eliminated_degree * second_retained_degree;
    if degree_bound > config.max_resultant_degree {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::DegreeBoundExceeded,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    let mut samples = Vec::with_capacity(degree_bound + 1);
    let mut index = 0_usize;
    while samples.len() <= degree_bound {
        let parameter_value = Real::from(index as u64);
        index += 1;
        let mut first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter_value,
            retained_parameter,
        );
        let mut second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter_value,
            retained_parameter,
        );
        // Existing curve-resultant callers define both coordinate differences
        // as retained minus eliminated. Preserve that orientation when the
        // second authored parameter is retained.
        if retained_parameter == CurveResultantParameter::Second {
            first
                .iter_mut()
                .for_each(|coefficient| *coefficient = -coefficient.clone());
            second
                .iter_mut()
                .for_each(|coefficient| *coefficient = -coefficient.clone());
        }
        let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            return undecided_report(retained_parameter, eliminated_parameter);
        };
        // Specializing the retained parameter may cancel an eliminated
        // leading coefficient. A lower-degree Sylvester determinant is not the
        // specialization of the generic resultant, so replace that sample.
        if first_sample_degree != Some(first_eliminated_degree)
            || second_sample_degree != Some(second_eliminated_degree)
        {
            continue;
        }
        let resultant =
            match resultant_univariate_polynomials(&first, &second, config.min_precision) {
                Ok(report) => report.resultant,
                Err(error) => {
                    return curve_resultant_report(
                        CurveIntersectionResultantStatus::ResultantError,
                        retained_parameter,
                        eliminated_parameter,
                        degree_bound,
                        samples,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
        samples.push(CurveIntersectionResultantSample {
            parameter_value,
            resultant,
        });
    }

    let Some(resultant_coefficients) = interpolate_samples(&samples, config.min_precision) else {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::InterpolationDivisionFailed,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            samples,
            Vec::new(),
            None,
        );
    };
    let Ok(resultant_coefficients) =
        trim_trailing_zeroes(resultant_coefficients, config.min_precision)
    else {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::UndecidedCoefficient,
            retained_parameter,
            eliminated_parameter,
            degree_bound,
            samples,
            Vec::new(),
            None,
        );
    };
    curve_resultant_report(
        CurveIntersectionResultantStatus::Constructed,
        retained_parameter,
        eliminated_parameter,
        degree_bound,
        samples,
        resultant_coefficients,
        None,
    )
}

/// Constructs rational lifts for the eliminated parameter of a bivariate system.
///
/// After specializing the retained parameter, the Sylvester matrix annihilates
/// `[u^(n-1), ..., u, 1]` at every common root `u`. When that matrix has
/// nullity one, any nonzero adjugate column is a scalar multiple of this power
/// vector, so the ratio of its last two entries is exactly `u`. This function
/// interpolates those signed cofactors as exact polynomials of the retained
/// parameter. Callers must still prove that a denominator is nonzero at their
/// retained root and replay the resulting rational image against the intended
/// lifted-root interval.
pub fn linear_parameter_lifts_bivariate_polynomial_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionParameterLiftReport {
    let lifted_parameter = opposite_parameter(retained_parameter);
    if bivariate_polynomial_is_empty(first_equation)
        || bivariate_polynomial_is_empty(second_equation)
    {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::EmptyEquation,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    }
    let (Ok(first_degree), Ok(second_degree)) = (
        certified_bivariate_degree(first_equation, config.min_precision),
        certified_bivariate_degree(second_equation, config.min_precision),
    ) else {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    };
    let (Some(first_degree), Some(second_degree)) = (first_degree, second_degree) else {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    };
    let (first_retained_degree, first_lifted_degree) =
        oriented_bidegree(first_degree, retained_parameter);
    let (second_retained_degree, second_lifted_degree) =
        oriented_bidegree(second_degree, retained_parameter);
    if first_lifted_degree == 0 || second_lifted_degree == 0 {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree,
            retained_parameter,
            lifted_parameter,
            0,
            Vec::new(),
            None,
        );
    }
    let sylvester_dimension = first_lifted_degree + second_lifted_degree;
    let retained_entry_degree = first_retained_degree.max(second_retained_degree);
    let degree_bound = (sylvester_dimension - 1) * retained_entry_degree;
    if degree_bound > config.max_resultant_degree {
        return parameter_lift_report(
            CurveIntersectionParameterLiftStatus::DegreeBoundExceeded,
            retained_parameter,
            lifted_parameter,
            degree_bound,
            Vec::new(),
            None,
        );
    }

    let mut parameters = Vec::with_capacity(degree_bound + 1);
    let mut numerators = vec![Vec::with_capacity(degree_bound + 1); sylvester_dimension];
    let mut denominators = vec![Vec::with_capacity(degree_bound + 1); sylvester_dimension];
    let mut sample_index = 0_usize;
    while parameters.len() <= degree_bound {
        let parameter = Real::from(sample_index as u64);
        sample_index += 1;
        let first = evaluate_bivariate_at_retained_parameter(
            first_equation,
            &parameter,
            retained_parameter,
        );
        let second = evaluate_bivariate_at_retained_parameter(
            second_equation,
            &parameter,
            retained_parameter,
        );
        let (Ok(first_sample_degree), Ok(second_sample_degree)) = (
            certified_nonzero_degree(&first, config.min_precision),
            certified_nonzero_degree(&second, config.min_precision),
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        if first_sample_degree != Some(first_lifted_degree)
            || second_sample_degree != Some(second_lifted_degree)
        {
            continue;
        }
        let sylvester = sylvester_matrix(&first, &second);
        for cofactor_row in 0..sylvester_dimension {
            let numerator = match signed_cofactor(
                &sylvester,
                cofactor_row,
                sylvester_dimension - 2,
                config.min_precision,
            ) {
                Ok(value) => value,
                Err(error) => {
                    return parameter_lift_report(
                        CurveIntersectionParameterLiftStatus::DeterminantError,
                        retained_parameter,
                        lifted_parameter,
                        degree_bound,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
            let denominator = match signed_cofactor(
                &sylvester,
                cofactor_row,
                sylvester_dimension - 1,
                config.min_precision,
            ) {
                Ok(value) => value,
                Err(error) => {
                    return parameter_lift_report(
                        CurveIntersectionParameterLiftStatus::DeterminantError,
                        retained_parameter,
                        lifted_parameter,
                        degree_bound,
                        Vec::new(),
                        Some(error),
                    );
                }
            };
            numerators[cofactor_row].push(numerator);
            denominators[cofactor_row].push(denominator);
        }
        parameters.push(parameter);
    }

    let mut maps = Vec::with_capacity(sylvester_dimension);
    for cofactor_row in 0..sylvester_dimension {
        let Some(numerator) = interpolate_parameter_values(
            &parameters,
            &numerators[cofactor_row],
            config.min_precision,
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        let Some(denominator) = interpolate_parameter_values(
            &parameters,
            &denominators[cofactor_row],
            config.min_precision,
        ) else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::InterpolationFailed,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        let Ok(denominator_is_zero) =
            is_certified_zero_polynomial(&denominator, config.min_precision)
        else {
            return parameter_lift_report(
                CurveIntersectionParameterLiftStatus::UndecidedCoefficient,
                retained_parameter,
                lifted_parameter,
                degree_bound,
                Vec::new(),
                None,
            );
        };
        if !denominator_is_zero {
            maps.push(CurveIntersectionParameterLiftMap {
                cofactor_row,
                numerator_coefficients: numerator,
                denominator_coefficients: denominator,
            });
        }
    }
    parameter_lift_report(
        CurveIntersectionParameterLiftStatus::Constructed,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        maps,
        None,
    )
}

/// Eliminate one parameter from a pair of 2D polynomial parametric curves.
///
/// For `retained_parameter == First`, this constructs
/// `Res_u(x1(t) - x2(u), y1(t) - y2(u))`. For `Second`, the roles are swapped
/// and the returned polynomial is in the second curve's parameter. The degree
/// bound is the classical bidegree resultant bound
/// `deg_u(g)*deg_t(f) + deg_u(f)*deg_t(g)`, evaluated after certified trimming.
/// The implementation samples that many plus one exact integer parameter
/// values and interpolates with exact Lagrange basis polynomials. Sylvester's
/// determinant resultant supplies elimination evidence; the exact-decision discipline requires
/// downstream exact replay before topology accepts a candidate root.
pub fn resultant_parametric_curve_intersection(
    first: &PolynomialParametricCurve2,
    second: &PolynomialParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    if first.x_coefficients.is_empty()
        || first.y_coefficients.is_empty()
        || second.x_coefficients.is_empty()
        || second.y_coefficients.is_empty()
    {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }
    resultant_bivariate_polynomial_system(
        &parametric_coordinate_difference(&first.x_coefficients, &second.x_coefficients),
        &parametric_coordinate_difference(&first.y_coefficients, &second.y_coefficients),
        retained_parameter,
        config,
    )
}

/// Eliminate one parameter from a pair of homogeneous rational parametric curves.
///
/// This is the rational/conic counterpart to
/// [`resultant_parametric_curve_intersection`]. It constructs homogeneous
/// coordinate equations
/// `X1(t) * W2(u) - X2(u) * W1(t)` and
/// `Y1(t) * W2(u) - Y2(u) * W1(t)`, then eliminates the other parameter by
/// exact Sylvester resultants. The cross-multiplied form follows the standard
/// rational Bezier model in the Bernstein/de Casteljau construction, and
/// keeps Bezier clipping inside the exact EGC
/// rule: denominator validity and topology are not inferred from this helper;
/// downstream curve code must replay candidates against retained geometry.
pub fn resultant_rational_parametric_curve_intersection(
    first: &RationalParametricCurve2,
    second: &RationalParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    if rational_curve_has_empty_polynomial(first) || rational_curve_has_empty_polynomial(second) {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }
    let Ok(first_weight_zero) = is_certified_zero_polynomial(&first.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, opposite_parameter(retained_parameter));
    };
    let Ok(second_weight_zero) = is_certified_zero_polynomial(&second.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, opposite_parameter(retained_parameter));
    };
    if first_weight_zero || second_weight_zero {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::InvalidHomogeneousWeight,
            retained_parameter,
            opposite_parameter(retained_parameter),
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    resultant_bivariate_polynomial_system(
        &rational_cross_equation(
            &first.x_numerator,
            &first.weight,
            &second.x_numerator,
            &second.weight,
        ),
        &rational_cross_equation(
            &first.y_numerator,
            &first.weight,
            &second.y_numerator,
            &second.weight,
        ),
        retained_parameter,
        config,
    )
}

fn curve_resultant_report(
    status: CurveIntersectionResultantStatus,
    retained_parameter: CurveResultantParameter,
    eliminated_parameter: CurveResultantParameter,
    degree_bound: usize,
    samples: Vec<CurveIntersectionResultantSample>,
    resultant_coefficients: Vec<Real>,
    resultant_error: Option<UnivariateResultantError>,
) -> CurveIntersectionResultantReport {
    CurveIntersectionResultantReport {
        status,
        retained_parameter,
        eliminated_parameter,
        degree_bound,
        samples,
        resultant_coefficients,
        resultant_error,
    }
}

fn parameter_lift_report(
    status: CurveIntersectionParameterLiftStatus,
    retained_parameter: CurveResultantParameter,
    lifted_parameter: CurveResultantParameter,
    degree_bound: usize,
    maps: Vec<CurveIntersectionParameterLiftMap>,
    determinant_error: Option<BareissError>,
) -> CurveIntersectionParameterLiftReport {
    CurveIntersectionParameterLiftReport {
        status,
        retained_parameter,
        lifted_parameter,
        degree_bound,
        maps,
        determinant_error,
    }
}

fn signed_cofactor(
    matrix: &[Vec<Real>],
    removed_row: usize,
    removed_column: usize,
    min_precision: i32,
) -> Result<Real, BareissError> {
    let minor = matrix
        .iter()
        .enumerate()
        .filter(|(row, _)| *row != removed_row)
        .map(|(_, row)| {
            row.iter()
                .enumerate()
                .filter(|(column, _)| *column != removed_column)
                .map(|(_, coefficient)| coefficient.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let determinant = determinant_bareiss(&minor, min_precision)?.determinant;
    Ok(if (removed_row + removed_column).is_multiple_of(2) {
        determinant
    } else {
        -determinant
    })
}

fn interpolate_parameter_values(
    parameters: &[Real],
    values: &[Real],
    min_precision: i32,
) -> Option<Vec<Real>> {
    if parameters.len() != values.len() {
        return None;
    }
    let samples = parameters
        .iter()
        .zip(values)
        .map(
            |(parameter_value, resultant)| CurveIntersectionResultantSample {
                parameter_value: parameter_value.clone(),
                resultant: resultant.clone(),
            },
        )
        .collect::<Vec<_>>();
    let coefficients = interpolate_samples(&samples, min_precision)?;
    trim_trailing_zeroes(coefficients, min_precision).ok()
}

fn undecided_report(
    retained_parameter: CurveResultantParameter,
    eliminated_parameter: CurveResultantParameter,
) -> CurveIntersectionResultantReport {
    curve_resultant_report(
        CurveIntersectionResultantStatus::UndecidedCoefficient,
        retained_parameter,
        eliminated_parameter,
        0,
        Vec::new(),
        Vec::new(),
        None,
    )
}

#[derive(Clone, Copy)]
struct BivariateDegree {
    first: usize,
    second: usize,
}

const fn opposite_parameter(parameter: CurveResultantParameter) -> CurveResultantParameter {
    match parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    }
}

const fn oriented_bidegree(
    degree: BivariateDegree,
    retained_parameter: CurveResultantParameter,
) -> (usize, usize) {
    match retained_parameter {
        CurveResultantParameter::First => (degree.first, degree.second),
        CurveResultantParameter::Second => (degree.second, degree.first),
    }
}

fn bivariate_polynomial_is_empty(polynomial: &BivariatePolynomial) -> bool {
    polynomial.coefficients.is_empty() || polynomial.coefficients.iter().all(Vec::is_empty)
}

fn certified_bivariate_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<BivariateDegree>, ()> {
    let first = certified_first_parameter_degree(polynomial, min_precision)?;
    let second = certified_second_parameter_degree(polynomial, min_precision)?;
    match (first, second) {
        (Some(first), Some(second)) => Ok(Some(BivariateDegree { first, second })),
        (None, None) => Ok(None),
        _ => unreachable!("a nonzero bivariate polynomial has both axis degrees"),
    }
}

fn certified_first_parameter_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    for (index, row) in polynomial.coefficients.iter().enumerate().rev() {
        let mut unknown = false;
        for coefficient in row {
            match coefficient.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => {}
                CertifiedRealSign::Known { .. } => return Ok(Some(index)),
                CertifiedRealSign::Unknown { .. } => unknown = true,
            }
        }
        if unknown {
            return Err(());
        }
    }
    Ok(None)
}

fn certified_second_parameter_degree(
    polynomial: &BivariatePolynomial,
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    let coefficient_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    for index in (0..coefficient_count).rev() {
        let mut unknown = false;
        for row in &polynomial.coefficients {
            let Some(coefficient) = row.get(index) else {
                continue;
            };
            match coefficient.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => {}
                CertifiedRealSign::Known { .. } => return Ok(Some(index)),
                CertifiedRealSign::Unknown { .. } => unknown = true,
            }
        }
        if unknown {
            return Err(());
        }
    }
    Ok(None)
}

fn certified_nonzero_degree(
    coefficients: &[Real],
    min_precision: i32,
) -> Result<Option<usize>, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(Some(index)),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(None)
}

fn trim_trailing_zeroes(coefficients: Vec<Real>, min_precision: i32) -> Result<Vec<Real>, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(coefficients[..=index].to_vec()),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(vec![Real::zero()])
}

fn rational_curve_has_empty_polynomial(curve: &RationalParametricCurve2) -> bool {
    curve.x_numerator.is_empty() || curve.y_numerator.is_empty() || curve.weight.is_empty()
}

fn is_certified_zero_polynomial(coefficients: &[Real], min_precision: i32) -> Result<bool, ()> {
    for coefficient in coefficients {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(false),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(true)
}

fn parametric_coordinate_difference(first: &[Real], second: &[Real]) -> BivariatePolynomial {
    let mut coefficients = vec![vec![Real::zero(); second.len()]; first.len()];
    for (index, coefficient) in first.iter().enumerate() {
        coefficients[index][0] += coefficient;
    }
    for (index, coefficient) in second.iter().enumerate() {
        coefficients[0][index] -= coefficient;
    }
    BivariatePolynomial::new(coefficients)
}

fn rational_cross_equation(
    first_coordinate: &[Real],
    first_weight: &[Real],
    second_coordinate: &[Real],
    second_weight: &[Real],
) -> BivariatePolynomial {
    let first_count = first_coordinate.len().max(first_weight.len());
    let second_count = second_coordinate.len().max(second_weight.len());
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first_index, first_coordinate) in first_coordinate.iter().enumerate() {
        for (second_index, second_weight) in second_weight.iter().enumerate() {
            coefficients[first_index][second_index] += first_coordinate * second_weight;
        }
    }
    for (first_index, first_weight) in first_weight.iter().enumerate() {
        for (second_index, second_coordinate) in second_coordinate.iter().enumerate() {
            coefficients[first_index][second_index] -= first_weight * second_coordinate;
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn evaluate_bivariate_at_retained_parameter(
    polynomial: &BivariatePolynomial,
    parameter: &Real,
    retained_parameter: CurveResultantParameter,
) -> Vec<Real> {
    match retained_parameter {
        CurveResultantParameter::First => {
            let coefficient_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..coefficient_count)
                .map(|second_power| {
                    polynomial
                        .coefficients
                        .iter()
                        .rev()
                        .fold(Real::zero(), |value, row| {
                            value * parameter
                                + row.get(second_power).cloned().unwrap_or_else(Real::zero)
                        })
                })
                .collect()
        }
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| eval_univariate(row, parameter))
            .collect(),
    }
}

fn eval_univariate(coefficients: &[Real], value: &Real) -> Real {
    let mut result = Real::zero();
    for coefficient in coefficients.iter().rev() {
        result = result * value.clone() + coefficient.clone();
    }
    result
}

fn interpolate_samples(
    samples: &[CurveIntersectionResultantSample],
    min_precision: i32,
) -> Option<Vec<Real>> {
    let mut result = vec![Real::zero(); samples.len()];
    for (sample_index, sample) in samples.iter().enumerate() {
        let mut basis = vec![Real::one()];
        let mut denominator = Real::one();
        for (other_index, other) in samples.iter().enumerate() {
            if sample_index == other_index {
                continue;
            }
            basis = multiply_by_linear_factor(basis, -other.parameter_value.clone());
            denominator *= sample.parameter_value.clone() - other.parameter_value.clone();
        }
        let scale = (sample.resultant.clone() / denominator).ok()?;
        for (index, coefficient) in basis.into_iter().enumerate() {
            result[index] += coefficient * scale.clone();
        }
    }
    trim_trailing_zeroes(result, min_precision).ok()
}

fn multiply_by_linear_factor(polynomial: Vec<Real>, constant: Real) -> Vec<Real> {
    let mut result = vec![Real::zero(); polynomial.len() + 1];
    for (index, coefficient) in polynomial.into_iter().enumerate() {
        result[index] += coefficient.clone() * constant.clone();
        result[index + 1] += coefficient;
    }
    result
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn line_x_axis() -> PolynomialParametricCurve2 {
        PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(0)])
    }

    #[test]
    fn curve_resultant_finds_parabola_horizontal_intersections() {
        let parabola = PolynomialParametricCurve2::new(
            vec![real(0), real(1)],
            vec![real(0), real(0), real(1)],
        );
        let horizontal = PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]);

        let report = resultant_parametric_curve_intersection(
            &parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.degree_bound, 2);
        assert_eq!(
            report.resultant_coefficients,
            vec![real(-1), real(0), real(1)]
        );
        assert_eq!(report.samples.len(), 3);
    }

    #[test]
    fn curve_resultant_reports_parallel_disjoint_line_constant() {
        let first = line_x_axis();
        let second = PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]);

        let report = resultant_parametric_curve_intersection(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.resultant_coefficients, vec![real(-1)]);
    }

    #[test]
    fn curve_resultant_retaining_second_swaps_parameters() {
        let vertical = PolynomialParametricCurve2::new(vec![real(2)], vec![real(0), real(1)]);
        let horizontal = line_x_axis();

        let report = resultant_parametric_curve_intersection(
            &vertical,
            &horizontal,
            CurveResultantParameter::Second,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.retained_parameter, CurveResultantParameter::Second);
        assert_eq!(report.eliminated_parameter, CurveResultantParameter::First);
        assert_eq!(report.resultant_coefficients, vec![real(-2), real(1)]);
    }

    #[test]
    fn bivariate_system_eliminates_either_parameter_with_the_same_root() {
        // `u-t=0` and `u+t-1=0` meet only at `t=u=1/2`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);

        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = resultant_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            assert_eq!(report.degree_bound, 2);
            assert_eq!(report.resultant_coefficients.len(), 2);
            assert_eq!(
                &report.resultant_coefficients[0] * real(2),
                -report.resultant_coefficients[1].clone()
            );
        }
    }

    #[test]
    fn bivariate_linear_lifts_recover_either_parameter_from_adjugate_ratios() {
        // `u-t=0` and `u+t-1=0` meet only at `t=u=1/2`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(1)], vec![real(1)]]);
        let half = (real(1) / real(2)).unwrap();

        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = linear_parameter_lifts_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(
                report.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert_eq!(report.retained_parameter, retained_parameter);
            assert_eq!(
                report.lifted_parameter,
                opposite_parameter(retained_parameter)
            );
            assert_eq!(report.degree_bound, 1);
            assert!(!report.maps.is_empty());
            assert!(report.maps.iter().any(|map| {
                let denominator = eval_univariate(&map.denominator_coefficients, &half);
                if denominator == real(0) {
                    return false;
                }
                let numerator = eval_univariate(&map.numerator_coefficients, &half);
                numerator == &half * denominator
            }));
        }
    }

    #[test]
    fn bivariate_linear_lifts_recover_a_nonlinear_common_root() {
        // `u^2-t=0` and `4u+4t-3=0` share `(t,u)=(1/4,1/2)`.
        let first = BivariatePolynomial::new(vec![vec![real(0), real(0), real(1)], vec![real(-1)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-3), real(4)], vec![real(4)]]);
        let quarter = (real(1) / real(4)).unwrap();
        let half = (real(1) / real(2)).unwrap();

        for (retained_parameter, retained, lifted) in [
            (CurveResultantParameter::First, &quarter, &half),
            (CurveResultantParameter::Second, &half, &quarter),
        ] {
            let report = linear_parameter_lifts_bivariate_polynomial_system(
                &first,
                &second,
                retained_parameter,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(
                report.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert!(report.maps.iter().any(|map| {
                let denominator = eval_univariate(&map.denominator_coefficients, retained);
                if denominator == real(0) {
                    return false;
                }
                let numerator = eval_univariate(&map.numerator_coefficients, retained);
                numerator == lifted * denominator
            }));
        }
    }

    #[test]
    fn bivariate_linear_lifts_report_nonlinear_fibers_explicitly() {
        // The first equation vanishes as a polynomial in `u` at `t=1/2`, so
        // the specialized common-root fiber is not certified by a nullity-one
        // Sylvester lift.
        let first = BivariatePolynomial::new(vec![vec![real(-1)], vec![real(2)]]);
        let second = BivariatePolynomial::new(vec![vec![real(-1), real(0), real(2)]]);
        let report = linear_parameter_lifts_bivariate_polynomial_system(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(
            report.status,
            CurveIntersectionParameterLiftStatus::UnsupportedEliminatedDegree
        );
        assert!(report.maps.is_empty());
    }

    #[test]
    fn bivariate_system_reports_an_identically_degenerate_equation() {
        let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
        let nonzero = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
        let report = resultant_bivariate_polynomial_system(
            &zero,
            &nonzero,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(report.resultant_coefficients, vec![real(0)]);
        assert_eq!(report.samples.len(), 1);
    }

    #[test]
    fn curve_resultant_rejects_empty_inputs_and_degree_over_budget() {
        let invalid = PolynomialParametricCurve2::new(Vec::new(), vec![real(0)]);
        let line = line_x_axis();
        let invalid_report = resultant_parametric_curve_intersection(
            &invalid,
            &line,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );
        assert_eq!(
            invalid_report.status,
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial
        );

        let quadratic = PolynomialParametricCurve2::new(
            vec![real(0), real(0), real(1)],
            vec![real(0), real(1)],
        );
        let limited = resultant_parametric_curve_intersection(
            &quadratic,
            &quadratic,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                max_resultant_degree: 0,
                ..CurveIntersectionResultantConfig::default()
            },
        );
        assert_eq!(
            limited.status,
            CurveIntersectionResultantStatus::DegreeBoundExceeded
        );
    }

    #[test]
    fn rational_curve_resultant_cross_multiplies_weighted_parabola() {
        let weighted_parabola = RationalParametricCurve2::new(
            vec![real(0), real(1)],
            vec![real(0), real(0), real(1)],
            vec![real(1), real(1)],
        );
        let horizontal = RationalParametricCurve2::from_polynomial(
            &PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]),
        );

        let report = resultant_rational_parametric_curve_intersection(
            &weighted_parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(
            report.resultant_coefficients,
            vec![real(-1), real(-1), real(1)]
        );
    }

    #[test]
    fn rational_curve_resultant_rejects_certified_zero_weight() {
        let invalid =
            RationalParametricCurve2::new(vec![real(0), real(1)], vec![real(1)], vec![real(0)]);
        let line = RationalParametricCurve2::from_polynomial(&line_x_axis());

        let report = resultant_rational_parametric_curve_intersection(
            &invalid,
            &line,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(
            report.status,
            CurveIntersectionResultantStatus::InvalidHomogeneousWeight
        );
    }

    #[test]
    fn rational_resultant_skips_specialized_degree_drop_samples() {
        let first = RationalParametricCurve2::new(
            vec![real(5), real(51), real(-15), real(-18)],
            vec![real(6), real(12), real(-12)],
            vec![real(1), real(3), real(-3)],
        );
        let second = RationalParametricCurve2::new(
            vec![real(28), real(160), real(-252), real(69)],
            vec![real(4), real(84), real(-84)],
            vec![real(1), real(9), real(-9)],
        );
        let report = resultant_rational_parametric_curve_intersection(
            &first,
            &second,
            CurveResultantParameter::Second,
            CurveIntersectionResultantConfig::default(),
        );

        assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
        let expected = [
            -1, -491, -9495, 66678, 1737203, 1859523, -33269637, 76011300, -71374833, 24098715,
            5240457, -5130702, 771282,
        ]
        .map(real);
        assert_eq!(report.resultant_coefficients.len(), expected.len());
        for (coefficient, expected_coefficient) in
            report.resultant_coefficients.iter().zip(&expected)
        {
            assert_eq!(
                coefficient * &expected[0],
                expected_coefficient * &report.resultant_coefficients[0]
            );
        }
        assert_eq!(report.samples.len(), report.degree_bound + 1);
        assert!(
            report
                .samples
                .last()
                .is_some_and(|sample| sample.parameter_value > real(report.degree_bound as i64)),
            "a degree-drop sample must be replaced, not interpolated at lower Sylvester degree"
        );
    }

    proptest! {
        #[test]
        fn generated_horizontal_line_intersects_parabola_at_exact_height(
            height in 1_i16..=32,
        ) {
            let height = i64::from(height);
            let parabola = PolynomialParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(0), real(0), real(1)],
            );
            let horizontal = PolynomialParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(height)],
            );

            let report = resultant_parametric_curve_intersection(
                &parabola,
                &horizontal,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            );

            prop_assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            prop_assert_eq!(
                report.resultant_coefficients,
                vec![real(-height), real(0), real(1)]
            );
        }

        #[test]
        fn generated_rational_weighted_parabola_resultant_cross_multiplies_height(
            height in 1_i16..=32,
        ) {
            let height = i64::from(height);
            let weighted_parabola = RationalParametricCurve2::new(
                vec![real(0), real(1)],
                vec![real(0), real(0), real(1)],
                vec![real(1), real(1)],
            );
            let horizontal = RationalParametricCurve2::from_polynomial(
                &PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(height)]),
            );

            let report = resultant_rational_parametric_curve_intersection(
                &weighted_parabola,
                &horizontal,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            );

            prop_assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
            prop_assert_eq!(
                report.resultant_coefficients,
                vec![real(-height), real(-height), real(1)]
            );
        }
    }
}
