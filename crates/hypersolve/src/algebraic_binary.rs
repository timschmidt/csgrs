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

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootArithmeticOp, AlgebraicRootKind, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    validate_algebraic_root_representation,
};
use crate::resultant::resultant_univariate_polynomials;
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval,
};

const MAX_BINARY_RESULTANT_DEGREE: usize = 8;

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
    if !left.is_valid() || !right.is_valid() {
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

    let resultant_degree =
        (left.polynomial_coefficients.len() - 1) * (right.polynomial_coefficients.len() - 1);
    if resultant_degree == 0 || resultant_degree > MAX_BINARY_RESULTANT_DEGREE {
        return binary_report(
            operation,
            AlgebraicRootBinaryTransformStatus::UnsupportedDegree,
            None,
            Some("binary algebraic resultant degree is outside the bounded package".to_owned()),
        );
    }

    let Some(polynomial_coefficients) = resultant_polynomial_for_binary_image(
        &left.polynomial_coefficients,
        &right.polynomial_coefficients,
        operation,
        resultant_degree,
        policy,
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
        kind: if interval.exact_root.is_some() {
            AlgebraicRootKind::ExactRationalWitness
        } else {
            AlgebraicRootKind::IsolatingInterval
        },
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
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let mut samples = Vec::with_capacity(resultant_degree + 1);
    for sample in 0..=resultant_degree {
        let y = Real::from(sample as i64);
        let right_in_x = match operation {
            AlgebraicRootArithmeticOp::Add => {
                compose_with_linear(right_polynomial, y, -Real::one())
            }
            AlgebraicRootArithmeticOp::Subtract => {
                compose_with_linear(right_polynomial, -y, Real::one())
            }
            AlgebraicRootArithmeticOp::Multiply => {
                reciprocal_product_polynomial(right_polynomial, &y)
            }
            AlgebraicRootArithmeticOp::Divide => quotient_product_polynomial(right_polynomial, &y),
            AlgebraicRootArithmeticOp::Negate => return None,
        };
        let resultant = resultant_univariate_polynomials(left_polynomial, &right_in_x, -64)
            .ok()?
            .resultant;
        samples.push(resultant);
    }
    trim_real_polynomial(interpolate_integer_samples(&samples)?, policy)
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
    let degree = right_polynomial.len() - 1;
    let mut result = vec![Real::zero(); degree + 1];
    for (power, coefficient) in right_polynomial.iter().enumerate() {
        result[degree - power] = coefficient.clone() * real_pow(y, power);
    }
    result
}

fn quotient_product_polynomial(right_polynomial: &[Real], y: &Real) -> Vec<Real> {
    let degree = right_polynomial.len() - 1;
    right_polynomial
        .iter()
        .enumerate()
        .map(|(power, coefficient)| coefficient.clone() * real_pow(y, degree - power))
        .collect()
}

fn compose_with_linear(polynomial: &[Real], constant: Real, linear: Real) -> Vec<Real> {
    let mut result = vec![Real::zero()];
    let mut power = vec![Real::one()];
    for coefficient in polynomial {
        add_scaled_polynomial(&mut result, &power, coefficient);
        power = multiply_by_linear_factor(&power, constant.clone(), linear.clone());
    }
    result
}

fn add_scaled_polynomial(target: &mut Vec<Real>, polynomial: &[Real], scale: &Real) {
    if target.len() < polynomial.len() {
        target.resize(polynomial.len(), Real::zero());
    }
    for (index, coefficient) in polynomial.iter().enumerate() {
        target[index] = target[index].clone() + coefficient.clone() * scale.clone();
    }
}

fn multiply_by_linear_factor(polynomial: &[Real], constant: Real, linear: Real) -> Vec<Real> {
    let mut result = vec![Real::zero(); polynomial.len() + 1];
    for (degree, coefficient) in polynomial.iter().enumerate() {
        result[degree] = result[degree].clone() + coefficient.clone() * constant.clone();
        result[degree + 1] = result[degree + 1].clone() + coefficient.clone() * linear.clone();
    }
    result
}

fn interpolate_integer_samples(samples: &[Real]) -> Option<Vec<Real>> {
    let mut result = vec![Real::zero(); samples.len()];
    for (index, value) in samples.iter().enumerate() {
        let mut basis = vec![Real::one()];
        let mut denominator = Real::one();
        for other in 0..samples.len() {
            if other == index {
                continue;
            }
            basis = multiply_by_linear_factor(&basis, -Real::from(other as i64), Real::one());
            denominator *= Real::from(index as i64 - other as i64);
        }
        let scale = (value.clone() / denominator).ok()?;
        for (degree, coefficient) in basis.into_iter().enumerate() {
            result[degree] = result[degree].clone() + coefficient * scale.clone();
        }
    }
    Some(result)
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
    if interval_value_contains_zero(right, policy)? {
        return None;
    }
    let lower_reciprocal = (Real::one() / right.lower.clone()).ok()?;
    let upper_reciprocal = (Real::one() / right.upper.clone()).ok()?;
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
    interval_value_contains_zero(
        &ValueInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        },
        policy,
    )
}

fn interval_value_contains_zero(interval: &ValueInterval, policy: PredicatePolicy) -> Option<bool> {
    let lower = compare_reals_with_policy(&interval.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&interval.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn real_pow(base: &Real, exponent: usize) -> Real {
    let mut value = Real::one();
    for _ in 0..exponent {
        value *= base.clone();
    }
    value
}

fn trim_real_polynomial(mut polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    while polynomial.len() > 1 {
        let trailing = polynomial.last()?;
        match compare_reals_with_policy(trailing, &Real::zero(), policy).value()? {
            Ordering::Equal => {
                polynomial.pop();
            }
            Ordering::Less | Ordering::Greater => break,
        }
    }
    (polynomial.len() > 1).then_some(polynomial)
}

fn sort_reals_exact(values: &mut [Real], policy: PredicatePolicy) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering =
                compare_reals_with_policy(&values[cursor], &values[cursor - 1], policy).value()?;
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
            kind: AlgebraicRootKind::IsolatingInterval,
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
            PredicatePolicy,
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
    fn binary_constructs_product_of_independent_square_roots() {
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Multiply,
            PredicatePolicy,
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
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::NonIsolatingImageInterval
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn binary_constructs_quotient_of_independent_square_roots() {
        let report = transform_algebraic_roots_binary(
            &sqrt_root(2, 1, 2),
            &sqrt_root(3, 1, 2),
            AlgebraicRootArithmeticOp::Divide,
            PredicatePolicy,
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
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootBinaryTransformStatus::DenominatorMayContainZero
        );
        assert!(report.representation.is_none());
    }

    proptest! {
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
                PredicatePolicy,
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
