//! Polynomial-image construction for represented algebraic roots.
//!
//! This module constructs `beta = q(alpha)` for an exact represented root
//! `alpha` and an exact-rational polynomial `q`. Unlike affine or Mobius
//! images, this needs an elimination step: if `P(alpha)=0`, then `beta` is a
//! root of `Res_x(P(x), q(x)-y)`. The implementation keeps the elimination
//! polynomial and the mapped isolating interval as replayable evidence.
//!
//! This is intentionally bounded. It accepts only image polynomials whose
//! derivative has a certified nonzero sign over the source isolating interval,
//! so the image interval contains one image of the one represented source
//! root. The univariate resultant is sampled at exact integer image values and
//! then interpolated exactly, avoiding primitive approximations while reusing
//! the crate's certified resultant machinery.
//! See Sylvester, "On a Theory of the Syzygetic Relations..." (1853), Collins
//! and Loos, "Real Zeros of Polynomials" (1982), and Chee K. Yap, "Towards
//! Exact Geometric Computation" (1997).

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, validate_algebraic_root_representation,
};
use crate::resultant::resultant_univariate_polynomials;
use crate::root_isolation::IsolatedRootInterval;

const MAX_SYLVESTER_DIMENSION: usize = 8;

/// Status for constructing a polynomial image of a represented algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootPolynomialImageStatus {
    /// The image was represented by exact resultant and interval evidence.
    Transformed,
    /// The input represented root failed structural validation.
    InvalidEvidence,
    /// The image polynomial is empty, constant-only where unsupported, or has
    /// non-exact coefficients.
    InvalidImagePolynomial,
    /// The image derivative could not be separated from zero on the source
    /// interval, so the image may not preserve one-root isolation.
    NonMonotoneImage,
    /// The bounded exact resultant package refused the Sylvester dimension.
    UnsupportedDegree,
    /// The resultant polynomial or mapped interval failed validation.
    InvalidTransformedEvidence,
    /// Exact comparisons or scalar operations did not decide.
    Undecided,
}

/// Report for `beta = q(alpha)`.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootPolynomialImageReport {
    /// Final image-construction status.
    pub status: AlgebraicRootPolynomialImageStatus,
    /// Exact image polynomial `q`, in ascending powers of `alpha`.
    pub image_coefficients: Vec<Real>,
    /// Resulting represented root when construction succeeds or when failed
    /// validation has useful evidence to inspect.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Construct an exact represented value for `q(alpha)`.
///
/// Coefficients are in ascending power order. The source polynomial `P` and
/// the image polynomial `q` must have exact-rational coefficients. The function
/// first certifies that `q'` has one nonzero sign on the source interval; then
/// it builds `Res_x(P(x), q(x)-y)` as the defining polynomial for the image.
/// This follows Yap's EGC separation: constructed algebraic values carry
/// exact replay evidence, and unsupported topology remains reportable.
pub fn transform_algebraic_root_polynomial_image(
    root: &AlgebraicRootRepresentation,
    image_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootPolynomialImageReport {
    if !root.is_valid() {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidEvidence,
            image_coefficients.to_vec(),
            None,
            Some("algebraic root representation must be valid before transformation".to_owned()),
        );
    }
    if root.polynomial_coefficients.len() <= 1
        || root
            .polynomial_coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidEvidence,
            image_coefficients.to_vec(),
            None,
            Some(
                "polynomial image construction requires exact-rational source evidence".to_owned(),
            ),
        );
    }
    let Some(image) = trim_real_polynomial(image_coefficients.to_vec(), policy) else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::Undecided,
            image_coefficients.to_vec(),
            None,
            Some("could not trim image polynomial exactly".to_owned()),
        );
    };
    if image.is_empty()
        || image
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial,
            image,
            None,
            Some(
                "polynomial image construction requires exact-rational image coefficients"
                    .to_owned(),
            ),
        );
    }
    if image.len() == 1 {
        return exact_constant_image(root, image, policy);
    }
    if let Some(witness) = root.exact_rational_witness() {
        return exact_constant_image(
            root,
            vec![evaluate_real_polynomial(&image, witness)],
            policy,
        );
    }
    let Some(derivative_sign) = certify_derivative_interval_sign(&image, &root.interval, policy)
    else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::NonMonotoneImage,
            image,
            None,
            Some("image derivative is not certified nonzero over the source interval".to_owned()),
        );
    };
    if derivative_sign == Ordering::Equal {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::NonMonotoneImage,
            image,
            None,
            Some("image derivative is certified zero over the source interval".to_owned()),
        );
    }

    let source_degree = root.polynomial_coefficients.len() - 1;
    let image_degree = image.len() - 1;
    let sylvester_dimension = source_degree + image_degree.max(1);
    if sylvester_dimension > MAX_SYLVESTER_DIMENSION {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::UnsupportedDegree,
            image,
            None,
            Some(
                "polynomial image resultant exceeds the bounded exact resultant package".to_owned(),
            ),
        );
    }

    let Some(polynomial_coefficients) =
        resultant_polynomial_for_image(&root.polynomial_coefficients, &image, policy)
    else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::Undecided,
            image,
            None,
            Some("could not construct resultant image polynomial exactly".to_owned()),
        );
    };
    let Some(interval) = polynomial_image_interval(&root.interval, &image, policy) else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::Undecided,
            image,
            None,
            Some("could not construct polynomial image interval exactly".to_owned()),
        );
    };
    let kind = if interval.exact_root.is_some() {
        AlgebraicRootKind::ExactRationalWitness
    } else {
        AlgebraicRootKind::IsolatingInterval
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        kind,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    if !representation.is_valid() {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidTransformedEvidence,
            image,
            Some(representation),
            Some("polynomial image evidence did not validate".to_owned()),
        );
    }

    polynomial_image_report(
        AlgebraicRootPolynomialImageStatus::Transformed,
        image,
        Some(representation),
        None,
    )
}

fn exact_constant_image(
    root: &AlgebraicRootRepresentation,
    image: Vec<Real>,
    policy: PredicatePolicy,
) -> AlgebraicRootPolynomialImageReport {
    let value = image[0].clone();
    let interval = IsolatedRootInterval {
        lower: value.clone(),
        upper: value.clone(),
        exact_root: Some(value.clone()),
        distinct_root_count: 1,
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients: vec![-value, Real::one()],
        interval,
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    polynomial_image_report(
        AlgebraicRootPolynomialImageStatus::Transformed,
        image,
        Some(representation),
        None,
    )
}

fn resultant_polynomial_for_image(
    source_polynomial: &[Real],
    image_polynomial: &[Real],
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let source_degree = source_polynomial.len() - 1;
    let mut samples = Vec::with_capacity(source_degree + 1);
    for sample in 0..=source_degree {
        let y = Real::from(sample as i64);
        let mut shifted_image = image_polynomial.to_vec();
        shifted_image[0] = shifted_image[0].clone() - y;
        let resultant = resultant_univariate_polynomials(source_polynomial, &shifted_image, -64)
            .ok()?
            .resultant;
        samples.push(resultant);
    }
    let mut polynomial = interpolate_integer_samples(&samples)?;
    if source_degree % 2 == 1 {
        for coefficient in &mut polynomial {
            *coefficient = -coefficient.clone();
        }
    }
    trim_real_polynomial(polynomial, policy)
}

fn polynomial_image_interval(
    interval: &IsolatedRootInterval,
    image_polynomial: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let first = evaluate_real_polynomial(image_polynomial, &interval.lower);
    let second = evaluate_real_polynomial(image_polynomial, &interval.upper);
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    let exact_root = interval
        .exact_root
        .as_ref()
        .map(|root| evaluate_real_polynomial(image_polynomial, root));
    Some(IsolatedRootInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
        exact_root,
        distinct_root_count: interval.distinct_root_count,
    })
}

fn certify_derivative_interval_sign(
    image_polynomial: &[Real],
    interval: &IsolatedRootInterval,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    let derivative = derivative_coefficients(image_polynomial);
    if derivative.is_empty() {
        return Some(Ordering::Equal);
    }
    let derivative_interval = evaluate_interval_polynomial(
        &derivative,
        &ValueInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        },
        policy,
    )?;
    let lower =
        compare_reals_with_policy(&derivative_interval.lower, &Real::zero(), policy).value()?;
    let upper =
        compare_reals_with_policy(&derivative_interval.upper, &Real::zero(), policy).value()?;
    if lower == Ordering::Greater {
        Some(Ordering::Greater)
    } else if upper == Ordering::Less {
        Some(Ordering::Less)
    } else if lower == Ordering::Equal && upper == Ordering::Equal {
        Some(Ordering::Equal)
    } else {
        None
    }
}

fn derivative_coefficients(polynomial: &[Real]) -> Vec<Real> {
    polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient.clone() * Real::from(degree as i64))
        .collect()
}

fn evaluate_real_polynomial(polynomial: &[Real], point: &Real) -> Real {
    polynomial
        .iter()
        .rev()
        .fold(Real::zero(), |value, coefficient| {
            value * point.clone() + coefficient.clone()
        })
}

#[derive(Clone, Debug)]
struct ValueInterval {
    lower: Real,
    upper: Real,
}

fn evaluate_interval_polynomial(
    polynomial: &[Real],
    point: &ValueInterval,
    policy: PredicatePolicy,
) -> Option<ValueInterval> {
    let mut value = ValueInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for coefficient in polynomial.iter().rev() {
        value = interval_add(
            interval_mul(&value, point, policy)?,
            &ValueInterval {
                lower: coefficient.clone(),
                upper: coefficient.clone(),
            },
        );
    }
    Some(value)
}

fn interval_add(left: ValueInterval, right: &ValueInterval) -> ValueInterval {
    ValueInterval {
        lower: left.lower + right.lower.clone(),
        upper: left.upper + right.upper.clone(),
    }
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

fn multiply_by_linear_factor(polynomial: &[Real], constant: Real, linear: Real) -> Vec<Real> {
    let mut result = vec![Real::zero(); polynomial.len() + 1];
    for (degree, coefficient) in polynomial.iter().enumerate() {
        result[degree] = result[degree].clone() + coefficient.clone() * constant.clone();
        result[degree + 1] = result[degree + 1].clone() + coefficient.clone() * linear.clone();
    }
    result
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
    if polynomial.is_empty() {
        polynomial.push(Real::zero());
    }
    Some(polynomial)
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

fn polynomial_image_report(
    status: AlgebraicRootPolynomialImageStatus,
    image_coefficients: Vec<Real>,
    representation: Option<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootPolynomialImageReport {
    AlgebraicRootPolynomialImageReport {
        status,
        image_coefficients,
        representation,
        message,
    }
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn sqrt_two_positive() -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: crate::SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
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
    fn polynomial_image_constructs_square_of_sqrt_two() {
        let report = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[Real::zero(), Real::zero(), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(4), real(-4), Real::one()]
        );
        assert_eq!(root.interval.lower, Real::one());
        assert_eq!(root.interval.upper, real(4));
        assert!(root.is_valid());
    }

    #[test]
    fn polynomial_image_constructs_quadratic_expression_of_sqrt_two() {
        let report = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[Real::zero(), Real::one(), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(2), real(-4), Real::one()]
        );
        assert_eq!(root.interval.lower, real(2));
        assert_eq!(root.interval.upper, real(6));
        assert!(root.is_valid());
    }

    #[test]
    fn polynomial_image_rejects_nonmonotone_source_interval() {
        let report = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[Real::zero(), real(-3), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::NonMonotoneImage
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn polynomial_image_preserves_rational_witnesses() {
        let rational = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            ..sqrt_two_positive()
        };
        let report = transform_algebraic_root_polynomial_image(
            &rational,
            &[real(1), real(2), real(3)],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(root.exact_rational_witness(), Some(&real(34)));
        assert!(root.is_valid());
    }

    proptest! {
        #[test]
        fn generated_rational_witness_polynomial_image_matches_integer_evaluation(
            root in -12_i16..=12,
            constant in -12_i16..=12,
            linear in -12_i16..=12,
            quadratic in 1_i16..=12,
        ) {
            let root = i64::from(root);
            let constant = i64::from(constant);
            let linear = i64::from(linear);
            let quadratic = i64::from(quadratic);
            let represented = AlgebraicRootRepresentation {
                polynomial_coefficients: vec![real(-root), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(root),
                    upper: real(root),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                kind: AlgebraicRootKind::ExactRationalWitness,
                ..sqrt_two_positive()
            };

            let report = transform_algebraic_root_polynomial_image(
                &represented,
                &[real(constant), real(linear), real(quadratic)],
                PredicatePolicy::default(),
            );

            prop_assert_eq!(report.status, AlgebraicRootPolynomialImageStatus::Transformed);
            let expected = constant + linear * root + quadratic * root * root;
            let transformed = report.representation.as_ref().unwrap();
            prop_assert_eq!(transformed.exact_rational_witness(), Some(&real(expected)));
            prop_assert!(transformed.is_valid());
        }
    }
}
