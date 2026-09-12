//! Binary algebraic-number construction for represented real roots.
//!
//! This module constructs the first bounded independent-root arithmetic
//! package for `hypersolve`: `gamma = alpha + beta`, `alpha - beta`,
//! `alpha * beta`, and guarded `alpha / beta` for two represented real
//! algebraic roots.  It uses exact
//! resultants to eliminate one source variable, then asks the existing Sturm
//! refinement package to certify that the exact image interval contains one
//! distinct real root of the resultant. That last step is the important
//! boundary: the resultant defines candidate algebraic values, while the
//! isolating interval is accepted only after exact replay, not after primitive
//! sampling.
//!
//! The construction combines a Sylvester resultant, Sturm root counting, the
//! standard real-root isolation model, and exact replay.

use std::borrow::Cow;
use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootArithmeticOp, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, validate_algebraic_root_representation,
};
use crate::integer_interpolation::{
    interpolate_integer_samples_up_to_scale, primitive_integer_polynomial,
};
use crate::resultant::resultant_exact_rational_polynomials_value;
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval, square_free_part,
};

const MAX_BINARY_RESULTANT_DEGREE: usize = 9;

/// Status for constructing a binary arithmetic image of two algebraic roots.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootBinaryTransformStatus {
    /// The arithmetic image has exact polynomial and isolating-interval
    /// evidence.
    Transformed,
    /// One or both source roots failed structural validation.
    InvalidEvidence,
    /// This bounded package does not support the requested arithmetic
    /// operation.
    UnsupportedOperation,
    /// Division was requested, but the divisor interval was not certified
    /// away from zero.
    DenominatorMayContainZero,
    /// The source coefficients are not exact rationals.
    UnsupportedCoefficient,
    /// The resultant degree exceeds the configured bounded exact package.
    UnsupportedDegree,
    /// The exact image interval did not certify exactly one resultant root.
    NonIsolatingImageInterval,
    /// The resultant polynomial or certified interval failed representation
    /// validation.
    InvalidTransformedEvidence,
    /// Exact comparisons, resultant construction, or interpolation did not
    /// decide.
    Undecided,
}

/// Report for independent-root arithmetic construction.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootBinaryTransformReport {
    /// Requested arithmetic operation.
    pub operation: AlgebraicRootArithmeticOp,
    /// Final construction status.
    pub status: AlgebraicRootBinaryTransformStatus,
    /// Resulting represented root when construction succeeds or failed
    /// validation has useful evidence to inspect.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Construct `alpha (+|-|*|/) beta` as a represented algebraic root.
///
/// The supported operations are addition, subtraction, multiplication, and
/// denominator-certified division.  The defining polynomial is obtained by
/// exact univariate resultants:
///
/// - `alpha + beta`: `Res_x(P(x), Q(y - x))`
/// - `alpha - beta`: `Res_x(P(x), Q(x - y))`
/// - `alpha * beta`: `Res_x(P(x), x^m Q(y / x))`, where `m = deg(Q)`
/// - `alpha / beta`: `Res_x(P(x), y^m Q(x / y))`, after proving `beta != 0`
///
/// The resultant is sampled at exact integer `y` values and interpolated
/// exactly.  This keeps the multivariate polynomial machinery bounded while
/// preserving a replayable elimination certificate.  Quotient construction is
/// still not a total field API: it refuses divisors whose isolating interval
/// may contain zero.
pub fn transform_algebraic_roots_binary(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> AlgebraicRootBinaryTransformReport {
    if !left.is_valid()
        || !right.is_valid()
        || validate_algebraic_root_representation(left, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
        || validate_algebraic_root_representation(right, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::InvalidEvidence,
            None,
            Some("binary algebraic construction requires valid represented roots".to_owned()),
        );
    }
    if !matches!(
        operation,
        AlgebraicRootArithmeticOp::Add
            | AlgebraicRootArithmeticOp::Subtract
            | AlgebraicRootArithmeticOp::Multiply
            | AlgebraicRootArithmeticOp::Divide
    ) {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::UnsupportedOperation,
            None,
            Some(
                "bounded independent-root construction supports add/subtract/multiply/divide"
                    .to_owned(),
            ),
        );
    }
    if operation == AlgebraicRootArithmeticOp::Divide
        && interval_contains_zero(&right.interval, policy).unwrap_or(true)
    {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero,
            None,
            Some(
                "independent division requires divisor interval evidence away from zero".to_owned(),
            ),
        );
    }
    if !has_exact_coefficients(&left.polynomial_coefficients)
        || !has_exact_coefficients(&right.polynomial_coefficients)
    {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::UnsupportedCoefficient,
            None,
            Some("binary algebraic construction requires exact-rational coefficients".to_owned()),
        );
    }

    let mut left_polynomial: Cow<'_, [Real]> = Cow::Borrowed(&left.polynomial_coefficients);
    let mut right_polynomial: Cow<'_, [Real]> = Cow::Borrowed(&right.polynomial_coefficients);
    let left_degree = left_polynomial.len() - 1;
    let right_degree = right_polynomial.len() - 1;
    if left_degree
        .checked_mul(right_degree)
        .is_none_or(|degree| degree > MAX_BINARY_RESULTANT_DEGREE)
    {
        let square_free_polynomials =
            if left.polynomial_coefficients == right.polynomial_coefficients {
                square_free_part(left_polynomial.into_owned(), PredicatePolicy::STRICT)
                    .map(|polynomial| (polynomial.clone(), polynomial))
            } else {
                square_free_part(left_polynomial.into_owned(), PredicatePolicy::STRICT).zip(
                    square_free_part(right_polynomial.into_owned(), PredicatePolicy::STRICT),
                )
            };
        let Some((square_free_left, square_free_right)) = square_free_polynomials else {
            return binary_report(
                operation,
                AlgebraicRootBinaryTransformStatus::Undecided,
                None,
                Some("could not square-free oversized binary source carriers".to_owned()),
            );
        };
        left_polynomial = Cow::Owned(square_free_left);
        right_polynomial = Cow::Owned(square_free_right);
    }
    let Some(resultant_degree) =
        (left_polynomial.len() - 1).checked_mul(right_polynomial.len() - 1)
    else {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::UnsupportedDegree,
            None,
            Some("binary algebraic resultant degree overflowed the bounded package".to_owned()),
        );
    };
    if resultant_degree == 0 || resultant_degree > MAX_BINARY_RESULTANT_DEGREE {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::UnsupportedDegree,
            None,
            Some("binary algebraic resultant degree is outside the bounded package".to_owned()),
        );
    }

    let Some(polynomial_coefficients) = resultant_polynomial_for_binary_image(
        &left_polynomial,
        &right_polynomial,
        operation,
        resultant_degree,
    ) else {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::Undecided,
            None,
            Some("could not construct binary resultant polynomial exactly".to_owned()),
        );
    };
    let Some(image_interval) =
        binary_image_interval(&left.interval, &right.interval, operation, policy)
    else {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::Undecided,
            None,
            Some("could not construct exact binary image interval".to_owned()),
        );
    };
    let refinement = refine_isolated_univariate_polynomial_interval(
        &polynomial_coefficients,
        &image_interval,
        RootIsolationConfig {
            policy,
            max_interval_width: None,
            max_refinement_steps: 2,
        },
    );
    let Some(interval) = refinement.refined_interval else {
        return binary_report(
            operation,
            match refinement.status {
                IsolatedRootRefinementStatus::NonUnitIsolation => {
                    AlgebraicRootBinaryTransformStatus::NonIsolatingImageInterval
                }
                IsolatedRootRefinementStatus::InvalidPolynomial
                | IsolatedRootRefinementStatus::InvalidInterval => {
                    AlgebraicRootBinaryTransformStatus::InvalidTransformedEvidence
                }
                IsolatedRootRefinementStatus::Undecided
                | IsolatedRootRefinementStatus::Refined
                | IsolatedRootRefinementStatus::ExactRoot => {
                    AlgebraicRootBinaryTransformStatus::Undecided
                }
            },
            None,
            refinement.message,
        );
    };

    let mut representation = AlgebraicRootRepresentation {
        constraint_index: left.constraint_index,
        symbol: left.symbol,
        interval_index: left.interval_index,
        polynomial_coefficients,
        interval,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    if !representation.is_valid() {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::InvalidTransformedEvidence,
            Some(representation),
            Some("binary algebraic result evidence did not validate".to_owned()),
        );
    }

    binary_report(
        operation,
        AlgebraicRootBinaryTransformStatus::Transformed,
        Some(representation),
        None,
    )
}

fn resultant_polynomial_for_binary_image(
    left_polynomial: &[Real],
    right_polynomial: &[Real],
    operation: AlgebraicRootArithmeticOp,
    resultant_degree: usize,
) -> Option<Vec<Real>> {
    let shared_polynomial = left_polynomial == right_polynomial;
    let left_polynomial = primitive_integer_polynomial(left_polynomial)?;
    let right_polynomial: Cow<'_, [Real]> = if shared_polynomial {
        Cow::Borrowed(&left_polynomial)
    } else {
        Cow::Owned(primitive_integer_polynomial(right_polynomial)?)
    };
    let mut samples = Vec::with_capacity(resultant_degree + 1);
    for sample in 0..=resultant_degree {
        let y = Real::from(sample as i64);
        let right_in_x = match operation {
            AlgebraicRootArithmeticOp::Add => {
                compose_with_signed_unit_linear(&right_polynomial, y, true)
            }
            AlgebraicRootArithmeticOp::Subtract => {
                compose_with_signed_unit_linear(&right_polynomial, -y, false)
            }
            AlgebraicRootArithmeticOp::Multiply => {
                reciprocal_product_polynomial(&right_polynomial, &y)
            }
            AlgebraicRootArithmeticOp::Divide => quotient_product_polynomial(&right_polynomial, &y),
            AlgebraicRootArithmeticOp::Negate => return None,
        };
        let resultant =
            resultant_exact_rational_polynomials_value(&left_polynomial, &right_in_x, -64).ok()?;
        samples.push(resultant);
    }
    let polynomial = interpolate_integer_samples_up_to_scale(&samples)?;
    trim_exact_rational_polynomial(polynomial)
}

fn binary_image_interval(
    left: &IsolatedRootInterval,
    right: &IsolatedRootInterval,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let value = match operation {
        AlgebraicRootArithmeticOp::Add => ValueInterval {
            lower: left.lower.clone() + right.lower.clone(),
            upper: left.upper.clone() + right.upper.clone(),
        },
        AlgebraicRootArithmeticOp::Subtract => ValueInterval {
            lower: left.lower.clone() - right.upper.clone(),
            upper: left.upper.clone() - right.lower.clone(),
        },
        AlgebraicRootArithmeticOp::Multiply => interval_mul(
            &ValueInterval {
                lower: left.lower.clone(),
                upper: left.upper.clone(),
            },
            &ValueInterval {
                lower: right.lower.clone(),
                upper: right.upper.clone(),
            },
            policy,
        )?,
        AlgebraicRootArithmeticOp::Divide => interval_div(
            &ValueInterval {
                lower: left.lower.clone(),
                upper: left.upper.clone(),
            },
            &ValueInterval {
                lower: right.lower.clone(),
                upper: right.upper.clone(),
            },
            policy,
        )?,
        AlgebraicRootArithmeticOp::Negate => return None,
    };
    Some(IsolatedRootInterval {
        lower: value.lower,
        upper: value.upper,
        exact_root: None,
        distinct_root_count: 1,
    })
}

fn reciprocal_product_polynomial(right_polynomial: &[Real], y: &Real) -> Vec<Real> {
    reversed_ascending_power_products(right_polynomial.iter(), y)
}

fn quotient_product_polynomial(right_polynomial: &[Real], y: &Real) -> Vec<Real> {
    reversed_ascending_power_products(right_polynomial.iter().rev(), y)
}

fn reversed_ascending_power_products<'a>(
    coefficients: impl ExactSizeIterator<Item = &'a Real>,
    y: &Real,
) -> Vec<Real> {
    let mut result = Vec::with_capacity(coefficients.len());
    let mut y_power = Real::one();
    for (power, coefficient) in coefficients.enumerate() {
        if power == 0 {
            result.push(coefficient.clone());
        } else {
            y_power *= y.clone();
            result.push(coefficient.clone() * y_power.clone());
        }
    }
    result.reverse();
    result
}

fn compose_with_signed_unit_linear(
    polynomial: &[Real],
    constant: Real,
    negative_linear: bool,
) -> Vec<Real> {
    let Some((leading, lower_coefficients)) = polynomial.split_last() else {
        return Vec::new();
    };
    let mut result = vec![leading.clone()];
    for coefficient in lower_coefficients.iter().rev() {
        result = multiply_by_signed_unit_linear_factor(&result, constant.clone(), negative_linear);
        result[0] = result[0].clone() + coefficient.clone();
    }
    result
}

fn multiply_by_signed_unit_linear_factor(
    polynomial: &[Real],
    constant: Real,
    negative_linear: bool,
) -> Vec<Real> {
    let Some((first, coefficients)) = polynomial.split_first() else {
        return Vec::new();
    };
    let mut result = Vec::with_capacity(polynomial.len() + 1);
    result.push(first.clone() * constant.clone());
    let mut previous = first;
    for coefficient in coefficients {
        let signed_previous = if negative_linear {
            -previous.clone()
        } else {
            previous.clone()
        };
        result.push(signed_previous + coefficient.clone() * constant.clone());
        previous = coefficient;
    }
    result.push(if negative_linear {
        -previous.clone()
    } else {
        previous.clone()
    });
    result
}

fn interval_mul(
    left: &ValueInterval,
    right: &ValueInterval,
    policy: PredicatePolicy,
) -> Option<ValueInterval> {
    let mut products = [
        left.lower.clone() * right.lower.clone(),
        left.lower.clone() * right.upper.clone(),
        left.upper.clone() * right.lower.clone(),
        left.upper.clone() * right.upper.clone(),
    ];
    sort_reals_exact(&mut products, policy)?;
    Some(ValueInterval {
        lower: products[0].clone(),
        upper: products[3].clone(),
    })
}

fn interval_div(
    left: &ValueInterval,
    right: &ValueInterval,
    policy: PredicatePolicy,
) -> Option<ValueInterval> {
    let lower_order = compare_reals(&right.lower, &Real::zero(), policy).value()?;
    let upper_order = compare_reals(&right.upper, &Real::zero(), policy).value()?;
    if lower_order != Ordering::Greater && upper_order != Ordering::Less {
        return None;
    }
    let lower_reciprocal =
        crate::policy_division::reciprocal_after_certified_nonzero(&right.lower).ok()?;
    let upper_reciprocal =
        crate::policy_division::reciprocal_after_certified_nonzero(&right.upper).ok()?;
    let mut reciprocal = [lower_reciprocal, upper_reciprocal];
    sort_reals_exact(&mut reciprocal, policy)?;
    interval_mul(
        left,
        &ValueInterval {
            lower: reciprocal[0].clone(),
            upper: reciprocal[1].clone(),
        },
        policy,
    )
}

fn interval_contains_zero(
    interval: &IsolatedRootInterval,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = compare_reals(&interval.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals(&interval.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn trim_exact_rational_polynomial(mut polynomial: Vec<Real>) -> Option<Vec<Real>> {
    while polynomial.len() > 1 {
        if polynomial.last()?.exact_rational_ref()?.is_zero() {
            polynomial.pop();
        } else {
            break;
        }
    }
    (polynomial.len() > 1).then_some(polynomial)
}

fn sort_reals_exact(values: &mut [Real], policy: PredicatePolicy) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering = compare_reals(&values[cursor], &values[cursor - 1], policy).value()?;
            if ordering != Ordering::Less {
                break;
            }
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

fn has_exact_coefficients(polynomial: &[Real]) -> bool {
    polynomial.len() > 1
        && polynomial
            .iter()
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
}

#[derive(Clone, Debug)]
struct ValueInterval {
    lower: Real,
    upper: Real,
}

fn binary_report(
    operation: AlgebraicRootArithmeticOp,
    status: AlgebraicRootBinaryTransformStatus,
    representation: Option<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootBinaryTransformReport {
    AlgebraicRootBinaryTransformReport {
        operation,
        status,
        representation,
        message,
    }
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;
    use crate::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn fraction(numerator: i64, denominator: u64) -> Real {
        Real::from(hyperreal::Rational::fraction(numerator, denominator).unwrap())
    }

    fn sqrt_root(square: i64, lower: i64, upper: i64) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: square as usize,
            symbol: SymbolId(square as u32),
            interval_index: 0,
            polynomial_coefficients: vec![-real(square), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(lower),
                upper: real(upper),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    fn positive_cubic_root(
        constant_numerator: i64,
        constant_denominator: u64,
        lower_numerator: i64,
        lower_denominator: u64,
        upper_numerator: i64,
        upper_denominator: u64,
    ) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: constant_denominator as usize,
            symbol: SymbolId(constant_denominator as u32),
            interval_index: 0,
            polynomial_coefficients: vec![
                -fraction(constant_numerator, constant_denominator),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ],
            interval: IsolatedRootInterval {
                lower: fraction(lower_numerator, lower_denominator),
                upper: fraction(upper_numerator, upper_denominator),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    #[test]
    fn binary_constructs_sum_of_independent_square_roots() {
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![
                Real::one(),
                Real::zero(),
                real(-10),
                Real::zero(),
                Real::one()
            ]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn binary_constructs_difference_of_independent_square_roots() {
        let left = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: fraction(7, 5),
                upper: fraction(3, 2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_root(2, 1, 2)
        };
        let right = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: fraction(5, 3),
                upper: fraction(7, 4),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_root(3, 1, 2)
        };
        let report = transform_algebraic_roots_binary(
            &left,
            &right,
            AlgebraicRootArithmeticOp::Subtract,
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![
                Real::one(),
                Real::zero(),
                real(-10),
                Real::zero(),
                Real::one()
            ]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn binary_reuses_square_free_sources_beyond_the_stored_degree_cap() {
        let left = sqrt_root(2, 1, 2);
        let right = sqrt_root(3, 1, 2);
        let expected = transform_algebraic_roots_binary(
            &left,
            &right,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            expected.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );

        let mut repeated_left = left;
        repeated_left.polynomial_coefficients = vec![
            real(-8),
            Real::zero(),
            real(12),
            Real::zero(),
            real(-6),
            Real::zero(),
            Real::one(),
        ];
        let mut repeated_right = right;
        repeated_right.polynomial_coefficients = vec![
            real(-27),
            Real::zero(),
            real(27),
            Real::zero(),
            real(-9),
            Real::zero(),
            Real::one(),
        ];
        assert_eq!(
            transform_algebraic_roots_binary(
                &repeated_left,
                &repeated_right,
                AlgebraicRootArithmeticOp::Add,
                PredicatePolicy::STRICT,
            ),
            expected
        );
    }

    #[test]
    fn binary_reuses_a_shared_square_free_carrier_for_distinct_conjugates() {
        let mut positive = sqrt_root(2, 1, 2);
        positive.polynomial_coefficients = vec![
            real(-8),
            Real::zero(),
            real(12),
            Real::zero(),
            real(-6),
            Real::zero(),
            Real::one(),
        ];
        let mut negative = positive.clone();
        negative.interval_index = 1;
        negative.interval = IsolatedRootInterval {
            lower: real(-2),
            upper: real(-1),
            exact_root: None,
            distinct_root_count: 1,
        };

        let report = transform_algebraic_roots_binary(
            &negative,
            &positive,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        assert_eq!(
            report.representation.unwrap().interval.exact_root,
            Some(Real::zero())
        );
    }

    #[test]
    fn binary_keeps_the_resultant_cap_after_square_free_reduction() {
        let quartic = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![
                real(-2),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ],
            ..sqrt_root(2, 1, 2)
        };
        let cubic = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-3), Real::zero(), Real::zero(), Real::one()],
            ..sqrt_root(3, 1, 2)
        };
        let report = transform_algebraic_roots_binary(
            &quartic,
            &cubic,
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::UnsupportedDegree
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn binary_resultant_clears_source_denominators_before_elimination() {
        let left = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![fraction(-2, 3), Real::zero(), fraction(1, 3)],
            ..sqrt_root(2, 1, 2)
        };
        let right = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![fraction(-3, 5), Real::zero(), fraction(1, 5)],
            ..sqrt_root(3, 1, 2)
        };
        let report = transform_algebraic_roots_binary(
            &left,
            &right,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        assert_eq!(
            report.representation.unwrap().polynomial_coefficients,
            vec![
                Real::one(),
                Real::zero(),
                real(-10),
                Real::zero(),
                Real::one()
            ]
        );
    }

    #[test]
    fn exact_rational_resultant_trimming_fails_closed() {
        assert_eq!(
            trim_exact_rational_polynomial(vec![real(1), Real::zero(), Real::one()]),
            Some(vec![real(1), Real::zero(), Real::one()])
        );
        assert_eq!(
            trim_exact_rational_polynomial(vec![real(1), Real::one(), Real::zero()]),
            Some(vec![real(1), Real::one()])
        );
        assert!(trim_exact_rational_polynomial(vec![real(1), Real::pi()]).is_none());
    }

    #[test]
    fn binary_constructs_product_of_independent_square_roots() {
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(36), Real::zero(), real(-12), Real::zero(), Real::one()]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn binary_constructs_product_of_independent_cubic_roots() {
        let report = transform_algebraic_roots_binary(
            &positive_cubic_root(1, 2, 3, 4, 4, 5),
            &positive_cubic_root(1, 3, 2, 3, 3, 4),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert!(root.is_valid());
        let expected = positive_cubic_root(1, 6, 11, 20, 14, 25);
        let comparison = crate::algebraic::compare_algebraic_root_representations_by_difference(
            root,
            &expected,
            crate::algebraic::AlgebraicRootRefinementComparisonConfig {
                policy: PredicatePolicy::STRICT,
                ..crate::algebraic::AlgebraicRootRefinementComparisonConfig::default()
            },
        );
        assert_eq!(
            comparison.comparison.status,
            crate::algebraic::AlgebraicRootComparisonStatus::Compared
        );
        assert_eq!(comparison.comparison.ordering, Some(Ordering::Equal));
    }

    #[test]
    fn binary_rejects_nonisolating_image_interval() {
        let wide = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_root(2, 1, 2)
        };
        let report = transform_algebraic_roots_binary(
            &wide,
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::NonIsolatingImageInterval
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn binary_replays_source_validation_instead_of_trusting_stale_status() {
        let mut invalid = sqrt_root(2, 1, 2);
        invalid.interval.lower = real(2);
        invalid.interval.upper = real(1);
        let report = transform_algebraic_roots_binary(
            &invalid,
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::InvalidEvidence
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn binary_constructs_quotient_of_independent_square_roots() {
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(4), Real::zero(), real(-12), Real::zero(), real(9)]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn binary_interval_division_reuses_policy_nonzero_endpoints() {
        let denominator = crate::test_support::exact_normal_positive();
        let half = fraction(1, 2);
        let numerator = denominator.clone() * &half;
        let quotient = interval_div(
            &ValueInterval {
                lower: numerator.clone(),
                upper: numerator,
            },
            &ValueInterval {
                lower: denominator.clone(),
                upper: denominator * Real::from(2),
            },
            PredicatePolicy::STRICT,
        )
        .expect("policy-certified endpoints should construct a quotient interval");
        assert_eq!(
            quotient.lower.exact_rational_normal_form(),
            fraction(1, 4).exact_rational()
        );
        assert_eq!(
            quotient.upper.exact_rational_normal_form(),
            half.exact_rational()
        );

        assert!(
            interval_div(
                &ValueInterval {
                    lower: Real::one(),
                    upper: Real::one(),
                },
                &ValueInterval {
                    lower: crate::test_support::terminal_zero(),
                    upper: Real::one(),
                },
                PredicatePolicy::STRICT,
            )
            .is_none()
        );
    }

    #[test]
    fn binary_refuses_division_when_denominator_interval_may_contain_zero() {
        let denominator = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_root(3, 1, 2)
        };
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &denominator,
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
        );
        assert!(report.representation.is_none());
    }

    proptest! {
        #[test]
        fn generated_signed_unit_linear_compositions_evaluate_exactly(
            coefficients in prop::collection::vec(-5_i16..=5, 1..=6),
            constant in -4_i16..=4,
            argument in -4_i16..=4,
            negative_linear in any::<bool>(),
        ) {
            let polynomial = coefficients
                .into_iter()
                .map(|coefficient| real(i64::from(coefficient)))
                .collect::<Vec<_>>();
            let constant = real(i64::from(constant));
            let argument = real(i64::from(argument));
            let composed = compose_with_signed_unit_linear(
                &polynomial,
                constant.clone(),
                negative_linear,
            );
            let evaluate = |polynomial: &[Real], argument: &Real| {
                polynomial.iter().rev().fold(Real::zero(), |value, coefficient| {
                    value * argument.clone() + coefficient.clone()
                })
            };
            let actual = evaluate(&composed, &argument);
            let signed_argument = if negative_linear {
                -argument
            } else {
                argument
            };
            let expected = evaluate(&polynomial, &(constant + signed_argument));
            prop_assert_eq!(
                actual.exact_rational_normal_form(),
                expected.exact_rational_normal_form()
            );
        }

        #[test]
        fn generated_square_root_products_match_expected_resultant_shape(
            left in 2_i16..=9,
            right in 10_i16..=18,
        ) {
            let left = i64::from(left);
            let right = i64::from(right);
            let report = transform_algebraic_roots_binary(
                &sqrt_root(left, 1, 5),
                &sqrt_root(right, 1, 5),
                AlgebraicRootArithmeticOp::Multiply,
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootBinaryTransformStatus::Transformed);
            let transformed = report.representation.as_ref().unwrap();
            let product = left * right;
            prop_assert_eq!(
                transformed.polynomial_coefficients.as_slice(),
                vec![real(product * product), Real::zero(), real(-2 * product), Real::zero(), Real::one()]
            );
            prop_assert!(transformed.is_valid());
        }
    }
}
