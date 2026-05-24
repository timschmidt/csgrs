//! Exact resultant helpers for low-degree parametric curve intersections.
//!
//! Curve crates own topology, span subdivision, and curve-specific validity
//! rules. This module supplies the bounded exact algebraic handoff they need:
//! two polynomial parametric curves in power basis are substituted into
//! `x1(t) - x2(u)` and `y1(t) - y2(u)`, the second parameter is eliminated by
//! Sylvester resultants, and the retained-parameter polynomial is reconstructed
//! by exact interpolation. This follows the elimination view of Sederberg and
//! Nishita, "Curve intersection using Bezier clipping" (1990), while keeping
//! Yap's exact-computation boundary explicit: the resultant is candidate
//! algebraic evidence, not an accepted topology event, until a downstream
//! curve package replays it against retained geometry.

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::curve_substitution::RationalParametricCurve2;
use crate::resultant::{UnivariateResultantError, resultant_univariate_polynomials};

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
            min_precision: -64,
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

/// Eliminate one parameter from a pair of 2D polynomial parametric curves.
///
/// For `retained_parameter == First`, this constructs
/// `Res_u(x1(t) - x2(u), y1(t) - y2(u))`. For `Second`, the roles are swapped
/// and the returned polynomial is in the second curve's parameter. The degree
/// bound is the classical bidegree resultant bound
/// `deg_u(g)*deg_t(f) + deg_u(f)*deg_t(g)`, evaluated after certified trimming.
/// The implementation samples that many plus one exact integer parameter
/// values and interpolates with exact Lagrange basis polynomials. Sylvester's
/// determinant resultant supplies elimination evidence; Yap (1997) requires
/// downstream exact replay before topology accepts a candidate root.
pub fn resultant_parametric_curve_intersection(
    first: &PolynomialParametricCurve2,
    second: &PolynomialParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    let (retained, eliminated, eliminated_parameter) = match retained_parameter {
        CurveResultantParameter::First => (first, second, CurveResultantParameter::Second),
        CurveResultantParameter::Second => (second, first, CurveResultantParameter::First),
    };

    if retained.x_coefficients.is_empty()
        || retained.y_coefficients.is_empty()
        || eliminated.x_coefficients.is_empty()
        || eliminated.y_coefficients.is_empty()
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

    let Ok(retained_x_degree) = certified_degree(&retained.x_coefficients, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(retained_y_degree) = certified_degree(&retained.y_coefficients, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_x_degree) =
        certified_degree(&eliminated.x_coefficients, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_y_degree) =
        certified_degree(&eliminated.y_coefficients, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };

    let degree_bound =
        eliminated_y_degree * retained_x_degree + eliminated_x_degree * retained_y_degree;
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
    for index in 0..=degree_bound {
        let parameter_value = Real::from(index as i64);
        let x_value = eval_univariate(&retained.x_coefficients, &parameter_value);
        let y_value = eval_univariate(&retained.y_coefficients, &parameter_value);
        let x_difference = shifted_negative_polynomial(&eliminated.x_coefficients, x_value);
        let y_difference = shifted_negative_polynomial(&eliminated.y_coefficients, y_value);
        let resultant = match resultant_univariate_polynomials(
            &x_difference,
            &y_difference,
            config.min_precision,
        ) {
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

/// Eliminate one parameter from a pair of homogeneous rational parametric curves.
///
/// This is the rational/conic counterpart to
/// [`resultant_parametric_curve_intersection`]. It constructs homogeneous
/// coordinate equations
/// `X1(t) * W2(u) - X2(u) * W1(t)` and
/// `Y1(t) * W2(u) - Y2(u) * W1(t)`, then eliminates the other parameter by
/// exact Sylvester resultants. The cross-multiplied form follows the standard
/// rational Bezier model in Farin, *Curves and Surfaces for CAGD* (2002), and
/// keeps Sederberg-Nishita style curve elimination (1990) inside Yap's EGC
/// rule: denominator validity and topology are not inferred from this helper;
/// downstream curve code must replay candidates against retained geometry.
pub fn resultant_rational_parametric_curve_intersection(
    first: &RationalParametricCurve2,
    second: &RationalParametricCurve2,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> CurveIntersectionResultantReport {
    let (retained, eliminated, eliminated_parameter) = match retained_parameter {
        CurveResultantParameter::First => (first, second, CurveResultantParameter::Second),
        CurveResultantParameter::Second => (second, first, CurveResultantParameter::First),
    };

    if rational_curve_has_empty_polynomial(retained)
        || rational_curve_has_empty_polynomial(eliminated)
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
    let Ok(retained_weight_zero) =
        is_certified_zero_polynomial(&retained.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_weight_zero) =
        is_certified_zero_polynomial(&eliminated.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    if retained_weight_zero || eliminated_weight_zero {
        return curve_resultant_report(
            CurveIntersectionResultantStatus::InvalidHomogeneousWeight,
            retained_parameter,
            eliminated_parameter,
            0,
            Vec::new(),
            Vec::new(),
            None,
        );
    }

    let Ok(retained_x_degree) = certified_degree(&retained.x_numerator, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(retained_y_degree) = certified_degree(&retained.y_numerator, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(retained_weight_degree) = certified_degree(&retained.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_x_degree) = certified_degree(&eliminated.x_numerator, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_y_degree) = certified_degree(&eliminated.y_numerator, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };
    let Ok(eliminated_weight_degree) = certified_degree(&eliminated.weight, config.min_precision)
    else {
        return undecided_report(retained_parameter, eliminated_parameter);
    };

    let x_retained_degree = retained_x_degree.max(retained_weight_degree);
    let y_retained_degree = retained_y_degree.max(retained_weight_degree);
    let x_eliminated_degree = eliminated_x_degree.max(eliminated_weight_degree);
    let y_eliminated_degree = eliminated_y_degree.max(eliminated_weight_degree);
    let degree_bound =
        y_eliminated_degree * x_retained_degree + x_eliminated_degree * y_retained_degree;
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
    for index in 0..=degree_bound {
        let parameter_value = Real::from(index as i64);
        let x_numerator = eval_univariate(&retained.x_numerator, &parameter_value);
        let y_numerator = eval_univariate(&retained.y_numerator, &parameter_value);
        let weight = eval_univariate(&retained.weight, &parameter_value);
        let x_difference = polynomial_difference(
            scale_polynomial(&eliminated.weight, x_numerator),
            scale_polynomial(&eliminated.x_numerator, weight.clone()),
        );
        let y_difference = polynomial_difference(
            scale_polynomial(&eliminated.weight, y_numerator),
            scale_polynomial(&eliminated.y_numerator, weight),
        );
        let resultant = match resultant_univariate_polynomials(
            &x_difference,
            &y_difference,
            config.min_precision,
        ) {
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

fn certified_degree(coefficients: &[Real], min_precision: i32) -> Result<usize, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(index),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(0)
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

fn shifted_negative_polynomial(polynomial: &[Real], shift: Real) -> Vec<Real> {
    let mut coefficients = polynomial
        .iter()
        .map(|coefficient| -coefficient.clone())
        .collect::<Vec<_>>();
    coefficients[0] += shift;
    coefficients
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

fn scale_polynomial(polynomial: &[Real], scale: Real) -> Vec<Real> {
    polynomial
        .iter()
        .map(|coefficient| coefficient.clone() * scale.clone())
        .collect()
}

fn polynomial_difference(left: Vec<Real>, right: Vec<Real>) -> Vec<Real> {
    let len = left.len().max(right.len());
    let mut result = vec![Real::zero(); len];
    for (index, coefficient) in left.into_iter().enumerate() {
        result[index] += coefficient;
    }
    for (index, coefficient) in right.into_iter().enumerate() {
        result[index] -= coefficient;
    }
    result
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
