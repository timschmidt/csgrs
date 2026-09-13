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
/// After the existing oversized-carrier square-free reduction, division may
/// remove an exact factor `x^k` from `Q` when the divisor interval excludes zero
/// under STRICT. This preserves the selected divisor and avoids an identically
/// zero resultant caused by an unused zero root in both carriers. For an
/// already nonzero resultant its signed primitive orientation is preserved.
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
    let removed_zero_factors = if operation == AlgebraicRootArithmeticOp::Divide {
        remove_certified_divisor_zero_factors(&mut right_polynomial, &right.interval)
    } else {
        0
    };
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

    let Some(mut polynomial_coefficients) = resultant_polynomial_for_binary_image(
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
    // Res(P, x^k S) = ((-1)^deg(P) P(0))^k Res(P, S). Positive content
    // normalization removes the magnitude, not this sign. If P(0) = 0 the
    // old resultant vanished identically and had no orientation to preserve.
    if removed_zero_factors % 2 == 1
        && let Some(constant) = left_polynomial[0].exact_rational_ref()
        && !constant.is_zero()
        && (constant.is_negative() != ((left_polynomial.len() - 1) % 2 == 1))
    {
        for coefficient in &mut polynomial_coefficients {
            *coefficient = -coefficient.clone();
        }
    }
    let Some(mut image_interval) =
        binary_image_interval(&left.interval, &right.interval, operation, policy)
    else {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::Undecided,
            None,
            Some("could not construct exact binary image interval".to_owned()),
        );
    };
    let refine = |interval: &IsolatedRootInterval| {
        refine_isolated_univariate_polynomial_interval(
            &polynomial_coefficients,
            interval,
            RootIsolationConfig {
                policy,
                max_interval_width: None,
                max_refinement_steps: 2,
            },
        )
    };
    let refinement = refine(&image_interval);
    // Ordinary/nonpoint refinement needs no extra strict endpoint probe. A
    // collapsed image is rejected as InvalidInterval when its witness is
    // missing; recover only after that typed gate, without matching a message.
    let refinement = if refinement.status == IsolatedRootRefinementStatus::InvalidInterval
        && let Some(witness) = certified_binary_point_witness(
            &left.interval,
            &right.interval,
            &image_interval,
            operation,
            policy,
        ) {
        image_interval.exact_root = Some(witness);
        // Do not retain the rejected report while replaying all the original
        // containment, polynomial-vanishing and uniqueness obligations.
        drop(refinement);
        refine(&image_interval)
    } else {
        refinement
    };
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

fn certified_binary_point_witness(
    left: &IsolatedRootInterval,
    right: &IsolatedRootInterval,
    image: &IsolatedRootInterval,
    operation: AlgebraicRootArithmeticOp,
    policy: PredicatePolicy,
) -> Option<Real> {
    // An approximate endpoint equality is never sufficient for a witness.
    if compare_reals(&image.lower, &image.upper, PredicatePolicy::STRICT).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    // Approximate multiplication/division may have selected extrema without
    // certifying the entire image. Replay their construction under STRICT.
    if policy != PredicatePolicy::STRICT
        && matches!(
            operation,
            AlgebraicRootArithmeticOp::Multiply | AlgebraicRootArithmeticOp::Divide
        )
    {
        let strict = binary_image_interval(left, right, operation, PredicatePolicy::STRICT)?;
        (compare_reals(&strict.lower, &strict.upper, PredicatePolicy::STRICT).value()
            == Some(Ordering::Equal))
        .then_some(strict.lower)
    } else {
        Some(image.lower.clone())
    }
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

fn remove_certified_divisor_zero_factors(
    polynomial: &mut Cow<'_, [Real]>,
    interval: &IsolatedRootInterval,
) -> usize {
    let count = polynomial
        .iter()
        .take_while(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_some_and(|q| q.is_zero())
        })
        .count();
    if count == 0
        || count + 1 >= polynomial.len()
        || interval_contains_zero(interval, PredicatePolicy::STRICT) != Some(false)
    {
        return 0;
    }
    // Borrow a suffix for the ordinary path; reuse square-free storage on the
    // oversized path. Original source evidence and interval stay untouched.
    match polynomial {
        Cow::Borrowed(coefficients) => *coefficients = &coefficients[count..],
        Cow::Owned(coefficients) => {
            coefficients.drain(..count);
        }
    }
    count
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
#[path = "algebraic_binary/zero_factor_tests.rs"]
mod zero_factor_tests;

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

    fn point_root(value: Real, polynomial: Vec<Real>) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: 7,
            symbol: SymbolId(11),
            interval_index: 3,
            polynomial_coefficients: polynomial,
            interval: IsolatedRootInterval {
                lower: value.clone(),
                upper: value.clone(),
                exact_root: Some(value),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    #[test]
    fn binary_point_images_preserve_rational_witnesses() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for a in -4..=4 {
                for b in -4..=4 {
                    let left = point_root(fraction(a, 3), vec![real(-a), real(3)]);
                    let right = point_root(fraction(b, 5), vec![real(-b), real(5)]);
                    for operation in [
                        AlgebraicRootArithmeticOp::Add,
                        AlgebraicRootArithmeticOp::Subtract,
                        AlgebraicRootArithmeticOp::Multiply,
                        AlgebraicRootArithmeticOp::Divide,
                    ] {
                        let report =
                            transform_algebraic_roots_binary(&left, &right, operation, policy);
                        if operation == AlgebraicRootArithmeticOp::Divide && b == 0 {
                            assert_eq!(
                                report.status,
                                AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
                            );
                            assert!(report.representation.is_none());
                            continue;
                        }
                        let expected = match operation {
                            AlgebraicRootArithmeticOp::Add => fraction(5 * a + 3 * b, 15),
                            AlgebraicRootArithmeticOp::Subtract => fraction(5 * a - 3 * b, 15),
                            AlgebraicRootArithmeticOp::Multiply => fraction(a * b, 15),
                            AlgebraicRootArithmeticOp::Divide => {
                                fraction(5 * a * b.signum(), 3 * b.unsigned_abs())
                            }
                            AlgebraicRootArithmeticOp::Negate => unreachable!(),
                        };
                        assert_eq!(
                            report.status,
                            AlgebraicRootBinaryTransformStatus::Transformed,
                            "{a}, {b}, {operation:?}"
                        );
                        let root = report.representation.unwrap();
                        assert_eq!(root.interval.lower, expected);
                        assert_eq!(root.interval.upper, expected);
                        assert_eq!(root.interval.exact_root, Some(expected));
                        assert_eq!(
                            (root.constraint_index, root.symbol, root.interval_index),
                            (7, SymbolId(11), 3)
                        );
                        assert_eq!(
                            validate_algebraic_root_representation(&root, PredicatePolicy::STRICT)
                                .status,
                            AlgebraicRootValidationStatus::Valid
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn binary_point_images_preserve_nonrational_witnesses() {
        let value = real(2).sqrt().unwrap();
        let left = point_root(value.clone(), vec![real(-2), Real::zero(), Real::one()]);
        let cases = [
            (AlgebraicRootArithmeticOp::Add, &value * real(2)),
            (AlgebraicRootArithmeticOp::Subtract, Real::zero()),
            (AlgebraicRootArithmeticOp::Multiply, real(2)),
            (AlgebraicRootArithmeticOp::Divide, Real::one()),
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (operation, expected) in &cases {
                let report = transform_algebraic_roots_binary(&left, &left, *operation, policy);
                assert_eq!(
                    report.status,
                    AlgebraicRootBinaryTransformStatus::Transformed,
                    "{operation:?}"
                );
                let root = report.representation.unwrap();
                assert_eq!(
                    compare_reals(
                        root.interval.exact_root.as_ref().unwrap(),
                        expected,
                        PredicatePolicy::STRICT
                    )
                    .value(),
                    Some(Ordering::Equal)
                );
                assert_eq!(
                    validate_algebraic_root_representation(&root, PredicatePolicy::STRICT).status,
                    AlgebraicRootValidationStatus::Valid
                );
            }
        }
    }

    #[test]
    fn binary_point_images_preserve_zero_products_with_wide_inputs() {
        let zero = point_root(Real::zero(), vec![Real::zero(), Real::one()]);
        let wide = sqrt_root(2, 1, 2);
        for (left, right) in [(&zero, &wide), (&wide, &zero)] {
            let report = transform_algebraic_roots_binary(
                left,
                right,
                AlgebraicRootArithmeticOp::Multiply,
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
    }

    #[test]
    fn binary_point_images_require_strict_equality_even_under_approximate_policy() {
        let zero = point_root(Real::zero(), vec![Real::zero(), Real::one()]);
        let upper = crate::test_support::terminal_zero() + real(2).powi_i64(-3000).unwrap();
        assert_eq!(
            compare_reals(&Real::zero(), &upper, PredicatePolicy::STRICT).value(),
            None
        );
        assert_eq!(
            compare_reals(&Real::zero(), &upper, PredicatePolicy::APPROXIMATE_512).value(),
            Some(Ordering::Equal)
        );
        let unresolved = IsolatedRootInterval {
            lower: Real::zero(),
            upper,
            exact_root: None,
            distinct_root_count: 1,
        };
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let image = binary_image_interval(
                &unresolved,
                &zero.interval,
                AlgebraicRootArithmeticOp::Add,
                policy,
            )
            .unwrap();
            assert!(image.exact_root.is_none());
            assert!(
                certified_binary_point_witness(
                    &unresolved,
                    &zero.interval,
                    &image,
                    AlgebraicRootArithmeticOp::Add,
                    policy,
                )
                .is_none()
            );
        }
    }

    #[test]
    fn binary_point_images_do_not_change_half_open_ownership() {
        let zero = point_root(Real::zero(), vec![Real::zero(), Real::one()]);
        let interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::one(),
            exact_root: None,
            distinct_root_count: 1,
        };
        let image = binary_image_interval(
            &interval,
            &zero.interval,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        )
        .unwrap();
        assert!(image.exact_root.is_none());
        assert!(
            certified_binary_point_witness(
                &interval,
                &zero.interval,
                &image,
                AlgebraicRootArithmeticOp::Add,
                PredicatePolicy::STRICT,
            )
            .is_none()
        );
        // (0, 1] owns only the root 1, even though the closed interval has two roots.
        let report = refine_isolated_univariate_polynomial_interval(
            &[Real::zero(), real(-1), Real::one()],
            &image,
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 2,
            },
        );
        assert_eq!(report.status, IsolatedRootRefinementStatus::ExactRoot);
        assert_eq!(
            report.refined_interval.unwrap().exact_root,
            Some(Real::one())
        );
    }

    #[test]
    fn binary_point_images_do_not_promote_approximately_chosen_extrema() {
        let offset = crate::test_support::terminal_zero();
        let width = real(2).powi_i64(-3000).unwrap();
        let left = IsolatedRootInterval {
            lower: offset.clone(),
            upper: &offset + &width,
            exact_root: None,
            distinct_root_count: 1,
        };
        let right = IsolatedRootInterval {
            lower: -left.upper.clone(),
            upper: -offset,
            exact_root: None,
            distinct_root_count: 1,
        };
        for interval in [&left, &right] {
            assert_eq!(
                compare_reals(&interval.lower, &interval.upper, PredicatePolicy::STRICT).value(),
                Some(Ordering::Less)
            );
        }
        assert!(
            binary_image_interval(
                &left,
                &right,
                AlgebraicRootArithmeticOp::Multiply,
                PredicatePolicy::STRICT
            )
            .is_none()
        );
        let image = binary_image_interval(
            &left,
            &right,
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        )
        .unwrap();
        // Positive-width inputs do not supply a singleton image. This example
        // leaves even the selected endpoint equality uncertified under STRICT.
        assert!(image.exact_root.is_none());
        assert!(
            certified_binary_point_witness(
                &left,
                &right,
                &image,
                AlgebraicRootArithmeticOp::Multiply,
                PredicatePolicy::APPROXIMATE_512,
            )
            .is_none()
        );
    }

    #[test]
    fn binary_point_images_require_certified_extrema_for_a_symmetric_product() {
        let radius = crate::test_support::exact_normal_positive();
        let interval = IsolatedRootInterval {
            lower: -radius.clone(),
            upper: radius,
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            compare_reals(&interval.lower, &interval.upper, PredicatePolicy::STRICT).value(),
            Some(Ordering::Less)
        );
        let strict = binary_image_interval(
            &interval,
            &interval,
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::STRICT,
        );
        let approximate = binary_image_interval(
            &interval,
            &interval,
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        )
        .unwrap();
        let strict_witness = strict.as_ref().and_then(|image| {
            certified_binary_point_witness(
                &interval,
                &interval,
                image,
                AlgebraicRootArithmeticOp::Multiply,
                PredicatePolicy::STRICT,
            )
        });
        let approximate_witness = certified_binary_point_witness(
            &interval,
            &interval,
            &approximate,
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy::APPROXIMATE_512,
        );
        println!(
            "symmetric product: strict_interval={}, strict_witness={}, approximate_witness={}",
            strict.is_some(),
            strict_witness.is_some(),
            approximate_witness.is_some()
        );
        assert!(approximate.exact_root.is_none());
        assert!(approximate_witness.is_none());
    }

    #[test]
    fn binary_point_images_still_replay_the_polynomial() {
        let one = point_root(Real::one(), vec![real(-1), Real::one()]);
        let zero = point_root(Real::zero(), vec![Real::zero(), Real::one()]);
        let mut image = binary_image_interval(
            &one.interval,
            &zero.interval,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        )
        .unwrap();
        assert!(image.exact_root.is_none());
        let rejected = refine_isolated_univariate_polynomial_interval(
            &[real(-1), Real::one()],
            &image,
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 2,
            },
        );
        // The public refiner's contract is unchanged: callers still need an
        // exact witness even when a point is a root of this polynomial.
        assert_eq!(
            rejected.status,
            IsolatedRootRefinementStatus::InvalidInterval
        );
        assert!(rejected.refined_interval.is_none());
        image.exact_root = certified_binary_point_witness(
            &one.interval,
            &zero.interval,
            &image,
            AlgebraicRootArithmeticOp::Add,
            PredicatePolicy::STRICT,
        );
        assert_eq!(image.exact_root, Some(Real::one()));
        let report = refine_isolated_univariate_polynomial_interval(
            &[real(-2), Real::one()],
            &image,
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 2,
            },
        );
        assert_eq!(
            report.status,
            IsolatedRootRefinementStatus::NonUnitIsolation
        );
        assert!(report.refined_interval.is_none());
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
