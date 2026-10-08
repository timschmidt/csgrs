//! Polynomial-image construction for represented algebraic roots.
//!
//! This module constructs `beta = q(alpha)` for an exact represented root
//! `alpha` and an exact-rational polynomial `q`. Unlike affine or Mobius
//! images, this needs an elimination step: if `P(alpha)=0`, then `beta` is a
//! root of `Res_x(P(x), q(x)-y)`. The implementation keeps the elimination
//! polynomial and the mapped isolating interval as replayable evidence.
//!
//! This is intentionally bounded. A nonzero derivative uses the endpoint-image
//! fast path. Stationary or initially nonmonotone maps instead use exact
//! interval evaluation and source-root refinement until Bernstein variation
//! certifies one distinct resultant root. The univariate resultant is sampled
//! at exact integer image values and then interpolated exactly, avoiding
//! primitive approximations while reusing the crate's certified machinery.
//! It combines a Sylvester resultant with the standard real-root isolation
//! model and exact replay.

use std::borrow::Cow;
use std::cmp::Ordering;

use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, validate_algebraic_root_representation,
};
use crate::algebraic_mobius::{
    AlgebraicRootMobiusTransformStatus, transform_algebraic_root_mobius,
};
use crate::integer_interpolation::{
    interpolate_integer_samples_up_to_scale, primitive_integer_polynomial,
};
use crate::resultant::{
    quotient_ring_resultant_polynomial, resultant_exact_rational_polynomials_value,
};
use crate::root_isolation::{
    IsolatedRootInterval, certify_algebraic_image_interval, square_free_part,
};

const MAX_SYLVESTER_DIMENSION: usize = 8;

/// Status for constructing a polynomial image of a represented algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootPolynomialImageStatus {
    /// The image was represented by exact resultant and interval evidence.
    Transformed,
    /// The input represented root failed structural validation.
    InvalidEvidence,
    /// The image polynomial is empty or has a non-exact-rational coefficient.
    InvalidImagePolynomial,
    /// Bounded refinement could not certify one distinct image root.
    ImageIsolationFailed,
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
/// Coefficients are in ascending power order. The source polynomial `P`, its
/// stored isolator, and the image polynomial `q` must be exact-rational. The function
/// builds `Res_x(P(x), q(x)-y)` as the defining polynomial for the image,
/// then certifies a one-root image interval directly or by exact refinement.
/// This follows the exact EGC separation: constructed algebraic values carry
/// exact replay evidence, and unsupported topology remains reportable.
pub fn transform_algebraic_root_polynomial_image(
    root: &AlgebraicRootRepresentation,
    image_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootPolynomialImageReport {
    if !root.is_valid()
        || validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidEvidence,
            image_coefficients.to_vec(),
            None,
            Some("algebraic root representation must be valid before transformation".to_owned()),
        );
    }
    if root
        .polynomial_coefficients
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
        || root.interval.lower.exact_rational_ref().is_none()
        || root.interval.upper.exact_rational_ref().is_none()
        || root
            .interval
            .exact_root
            .as_ref()
            .is_some_and(|witness| witness.exact_rational_ref().is_none())
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
    if image_coefficients.is_empty()
        || image_coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial,
            image_coefficients.to_vec(),
            None,
            Some(
                "polynomial image construction requires a nonempty exact-rational polynomial"
                    .to_owned(),
            ),
        );
    }
    let image = trim_exact_rational_polynomial(image_coefficients);
    if image.len() == 1 {
        let value = image[0].clone();
        return exact_constant_image(root, image, value);
    }
    if let Some(witness) = root.exact_point_witness() {
        let Some(value) = evaluate_rational_polynomial(&image, witness) else {
            return polynomial_image_report(
                AlgebraicRootPolynomialImageStatus::Undecided,
                image,
                None,
                Some("could not evaluate an exact rational source witness".to_owned()),
            );
        };
        return exact_constant_image(root, image, value);
    }
    if image.len() > 2
        && let Some(value) = exact_constant_source_relation(&root.polynomial_coefficients, &image)
    {
        return exact_constant_image(root, image, value);
    }
    if image.len() == 2 {
        let affine = transform_algebraic_root_mobius(
            root,
            image[1].clone(),
            image[0].clone(),
            Real::zero(),
            Real::one(),
            policy,
        );
        if affine.status == AlgebraicRootMobiusTransformStatus::Transformed
            && let Some(mut representation) = affine.representation
            && let Some(polynomial) =
                primitive_integer_polynomial(&representation.polynomial_coefficients)
        {
            representation.polynomial_coefficients = polynomial;
            representation.validation =
                validate_algebraic_root_representation(&representation, policy);
            if representation.is_valid() {
                return polynomial_image_report(
                    AlgebraicRootPolynomialImageStatus::Transformed,
                    image,
                    Some(representation),
                    None,
                );
            }
        }
    }

    let image_degree = image.len() - 1;
    let mut source_polynomial: Cow<'_, [Real]> = Cow::Borrowed(&root.polynomial_coefficients);
    if (source_polynomial.len() - 1)
        .checked_add(image_degree)
        .is_none_or(|dimension| dimension > MAX_SYLVESTER_DIMENSION)
    {
        let Some(square_free_source) =
            square_free_part(source_polynomial.into_owned(), PredicatePolicy::STRICT)
        else {
            return polynomial_image_report(
                AlgebraicRootPolynomialImageStatus::Undecided,
                image,
                None,
                Some("could not square-free an oversized polynomial-image carrier".to_owned()),
            );
        };
        source_polynomial = Cow::Owned(square_free_source);
        if image.len() > 2
            && let Some(value) = exact_constant_source_relation(&source_polynomial, &image)
        {
            return exact_constant_image(root, image, value);
        }
    }
    let source_degree = source_polynomial.len() - 1;
    let Some(sylvester_dimension) = source_degree.checked_add(image_degree) else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::UnsupportedDegree,
            image,
            None,
            Some("polynomial image resultant dimension overflowed".to_owned()),
        );
    };
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

    let Some(polynomial_coefficients) = resultant_polynomial_for_image(&source_polynomial, &image)
    else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::Undecided,
            image,
            None,
            Some("could not construct resultant image polynomial exactly".to_owned()),
        );
    };
    let Some(interval) = certified_polynomial_image_interval(
        &source_polynomial,
        &root.interval,
        &image,
        &polynomial_coefficients,
        policy,
    ) else {
        return polynomial_image_report(
            AlgebraicRootPolynomialImageStatus::ImageIsolationFailed,
            image,
            None,
            Some("could not certify a one-root polynomial image interval".to_owned()),
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
    value: Real,
) -> AlgebraicRootPolynomialImageReport {
    let interval = IsolatedRootInterval {
        lower: value.clone(),
        upper: value.clone(),
        exact_root: Some(value.clone()),
        distinct_root_count: 1,
    };
    let representation = AlgebraicRootRepresentation {
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
    debug_assert_eq!(
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid,
        "canonical exact polynomial-image output must validate",
    );
    polynomial_image_report(
        AlgebraicRootPolynomialImageStatus::Transformed,
        image,
        Some(representation),
        None,
    )
}

fn exact_constant_source_relation(source: &[Real], image: &[Real]) -> Option<Real> {
    if source.len() <= 1 || source.len() != image.len() {
        return None;
    }
    for (source_coefficient, image_coefficient) in source[1..source.len() - 1]
        .iter()
        .zip(&image[1..image.len() - 1])
    {
        let source_is_zero = source_coefficient.exact_rational_ref()?.is_zero();
        let image_is_zero = image_coefficient.exact_rational_ref()?.is_zero();
        if source_is_zero != image_is_zero {
            return None;
        }
    }
    let source_leading = source.last()?.exact_rational_ref()?;
    if source_leading.is_zero() {
        return None;
    }
    let image_leading = image.last()?.exact_rational_ref()?;
    let scale = image_leading / source_leading;
    for (source_coefficient, image_coefficient) in source[1..source.len() - 1]
        .iter()
        .zip(&image[1..image.len() - 1])
    {
        let source_coefficient = source_coefficient.exact_rational_ref()?;
        if source_coefficient.is_zero() {
            continue;
        }
        if image_coefficient.exact_rational_ref()? != &(source_coefficient * &scale) {
            return None;
        }
    }
    let constant = image[0].exact_rational_ref()? - &(source[0].exact_rational_ref()? * scale);
    Some(Real::from(constant))
}

fn resultant_polynomial_for_image(
    source_polynomial: &[Real],
    image_polynomial: &[Real],
) -> Option<Vec<Real>> {
    let source_degree = source_polynomial.len() - 1;
    let source_polynomial = primitive_integer_polynomial(source_polynomial)?;
    let (image_polynomial, image_scale) = primitive_integer_image_relation(image_polynomial)?;
    let mut polynomial = quotient_ring_resultant_polynomial(
        &source_polynomial,
        &image_polynomial,
        std::slice::from_ref(&image_scale),
    )
    .or_else(|| {
        let mut samples = Vec::with_capacity(source_degree + 1);
        for sample in 0..=source_degree {
            let y = Real::from(sample as i64) * image_scale.clone();
            let mut shifted_image = image_polynomial.to_vec();
            shifted_image[0] = shifted_image[0].clone() - y;
            let resultant =
                resultant_exact_rational_polynomials_value(&source_polynomial, &shifted_image, -64)
                    .ok()?;
            samples.push(resultant);
        }
        interpolate_integer_samples_up_to_scale(&samples)
    })?;
    if source_degree % 2 == 1 {
        for coefficient in &mut polynomial {
            *coefficient = -coefficient.clone();
        }
    }
    trim_owned_exact_rational_polynomial(polynomial)
}

fn primitive_integer_image_relation(image_polynomial: &[Real]) -> Option<(Vec<Real>, Real)> {
    let y_coefficient = Rational::one();
    let mut rationals = image_polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    rationals.push(&y_coefficient);
    let mut coefficients = Rational::primitive_bigint_ratio(&rationals)
        .into_iter()
        .map(Rational::from_bigint)
        .map(Real::from)
        .collect::<Vec<_>>();
    let image_scale = coefficients.pop()?;
    Some((coefficients, image_scale))
}

fn polynomial_image_interval(
    interval: &IsolatedRootInterval,
    image_polynomial: &[Real],
) -> Option<IsolatedRootInterval> {
    let first = evaluate_rational_polynomial(image_polynomial, &interval.lower)?;
    let second = evaluate_rational_polynomial(image_polynomial, &interval.upper)?;
    let mut endpoints = [first, second];
    sort_exact_rational_reals(&mut endpoints)?;
    let [lower, upper] = endpoints;
    let exact_root = match &interval.exact_root {
        Some(root) => Some(evaluate_rational_polynomial(image_polynomial, root)?),
        None => None,
    };
    Some(IsolatedRootInterval {
        lower,
        upper,
        exact_root,
        distinct_root_count: interval.distinct_root_count,
    })
}

fn certified_polynomial_image_interval(
    source_polynomial: &[Real],
    source_interval: &IsolatedRootInterval,
    image_polynomial: &[Real],
    resultant_polynomial: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let derivative = derivative_coefficients(image_polynomial)?;
    certify_algebraic_image_interval(
        source_polynomial,
        source_interval,
        resultant_polynomial,
        policy,
        |source_interval| {
            polynomial_image_enclosure(source_interval, image_polynomial, &derivative)
        },
    )
}

fn polynomial_image_enclosure(
    interval: &IsolatedRootInterval,
    image_polynomial: &[Real],
    derivative: &[Real],
) -> Option<IsolatedRootInterval> {
    if interval.exact_root.is_some()
        || certify_polynomial_interval_sign(derivative, interval)
            .is_some_and(|sign| sign != Ordering::Equal)
    {
        return polynomial_image_interval(interval, image_polynomial);
    }
    let image = evaluate_interval_polynomial(
        image_polynomial,
        &ValueInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        },
    )?;
    Some(IsolatedRootInterval {
        lower: image.lower,
        upper: image.upper,
        exact_root: None,
        distinct_root_count: 1,
    })
}

fn certify_polynomial_interval_sign(
    polynomial: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    if polynomial.is_empty() {
        return Some(Ordering::Equal);
    }
    let value_interval = evaluate_rational_interval_polynomial(
        polynomial,
        &ValueInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        },
    )?;
    let lower = rational_sign(&value_interval.lower);
    let upper = rational_sign(&value_interval.upper);
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

fn derivative_coefficients(polynomial: &[Real]) -> Option<Vec<Real>> {
    polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| {
            Some(Real::from(
                coefficient.exact_rational_ref()? * Rational::new(degree as i64),
            ))
        })
        .collect()
}

fn evaluate_rational_polynomial(polynomial: &[Real], point: &Real) -> Option<Real> {
    let point = point.exact_rational_ref()?;
    let mut coefficients = polynomial.iter().rev();
    let mut value = coefficients.next()?.exact_rational_ref()?.clone();
    for coefficient in coefficients {
        value = &value * point + coefficient.exact_rational_ref()?;
    }
    Some(Real::from(value))
}

#[derive(Clone, Debug)]
struct ValueInterval {
    lower: Real,
    upper: Real,
}

fn evaluate_interval_polynomial(
    polynomial: &[Real],
    point: &ValueInterval,
) -> Option<ValueInterval> {
    let value = evaluate_rational_interval_polynomial(polynomial, point)?;
    Some(ValueInterval {
        lower: Real::from(value.lower),
        upper: Real::from(value.upper),
    })
}

fn evaluate_rational_interval_polynomial(
    polynomial: &[Real],
    point: &ValueInterval,
) -> Option<RationalValueInterval> {
    let point = RationalValueInterval {
        lower: point.lower.exact_rational_ref()?.clone(),
        upper: point.upper.exact_rational_ref()?.clone(),
    };
    let mut coefficients = polynomial.iter().rev();
    let Some(leading) = coefficients.next() else {
        return Some(RationalValueInterval {
            lower: Rational::zero(),
            upper: Rational::zero(),
        });
    };
    let leading = leading.exact_rational_ref()?;
    let mut value = RationalValueInterval {
        lower: leading.clone(),
        upper: leading.clone(),
    };
    if let Some(coefficient) = coefficients.next() {
        let (lower, upper) = if leading.is_negative() {
            (leading * &point.upper, leading * &point.lower)
        } else {
            (leading * &point.lower, leading * &point.upper)
        };
        let coefficient = coefficient.exact_rational_ref()?;
        value = RationalValueInterval {
            lower: lower + coefficient,
            upper: upper + coefficient,
        };
    }
    for coefficient in coefficients {
        let product = rational_interval_mul(&value, &point)?;
        let coefficient = coefficient.exact_rational_ref()?;
        value = RationalValueInterval {
            lower: product.lower + coefficient,
            upper: product.upper + coefficient,
        };
    }
    Some(value)
}

struct RationalValueInterval {
    lower: Rational,
    upper: Rational,
}

fn rational_interval_mul(
    left: &RationalValueInterval,
    right: &RationalValueInterval,
) -> Option<RationalValueInterval> {
    let mut products = [
        &left.lower * &right.lower,
        &left.lower * &right.upper,
        &left.upper * &right.lower,
        &left.upper * &right.upper,
    ];
    sort_rationals(&mut products)?;
    let [lower, _, _, upper] = products;
    Some(RationalValueInterval { lower, upper })
}

fn trim_exact_rational_polynomial(polynomial: &[Real]) -> Vec<Real> {
    let mut len = polynomial.len();
    while len > 1
        && polynomial[len - 1]
            .exact_rational_ref()
            .is_some_and(Rational::is_zero)
    {
        len -= 1;
    }
    polynomial[..len].to_vec()
}

fn trim_owned_exact_rational_polynomial(mut polynomial: Vec<Real>) -> Option<Vec<Real>> {
    if polynomial.is_empty() {
        return None;
    }
    while polynomial.len() > 1
        && polynomial
            .last()?
            .exact_rational_ref()
            .is_some_and(Rational::is_zero)
    {
        polynomial.pop();
    }
    polynomial
        .iter()
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
        .then_some(polynomial)
}

fn rational_sign(value: &Rational) -> Ordering {
    if value.is_negative() {
        Ordering::Less
    } else if value.is_zero() {
        Ordering::Equal
    } else {
        Ordering::Greater
    }
}

fn sort_exact_rational_reals(values: &mut [Real]) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering = values[cursor]
                .exact_rational_ref()?
                .partial_cmp(values[cursor - 1].exact_rational_ref()?)?;
            if ordering != Ordering::Less {
                break;
            }
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

fn sort_rationals(values: &mut [Rational]) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            if values[cursor].partial_cmp(&values[cursor - 1])? != Ordering::Less {
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
    use hyperlimit::compare_reals;
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn fraction(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
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
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(root.polynomial_coefficients, vec![real(-2), Real::one()]);
        assert_eq!(root.exact_point_witness(), Some(&real(2)));
        assert_eq!(
            report.image_coefficients,
            vec![Real::zero(), Real::zero(), Real::one()]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn polynomial_image_detects_a_constant_source_relation() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_polynomial_image(
                &sqrt_two_positive(),
                &[real(-1), Real::zero(), real(3)],
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            assert_eq!(
                report.representation.unwrap().exact_point_witness(),
                Some(&real(5))
            );
            assert_eq!(
                report.image_coefficients,
                vec![real(-1), Real::zero(), real(3)]
            );
        }
    }

    #[test]
    fn polynomial_image_square_frees_an_oversized_repeated_carrier() {
        let repeated = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![
                real(16),
                Real::zero(),
                real(-32),
                Real::zero(),
                real(24),
                Real::zero(),
                real(-8),
                Real::zero(),
                Real::one(),
            ],
            ..sqrt_two_positive()
        };
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let constant = transform_algebraic_root_polynomial_image(
                &repeated,
                &[Real::zero(), Real::zero(), Real::one()],
                policy,
            );
            assert_eq!(
                constant.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            assert_eq!(
                constant.representation.unwrap().exact_point_witness(),
                Some(&real(2))
            );

            let report = transform_algebraic_root_polynomial_image(
                &repeated,
                &[Real::zero(), Real::one(), Real::one()],
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            let image = report.representation.unwrap();
            assert_eq!(
                image.polynomial_coefficients,
                vec![real(2), real(-4), Real::one()]
            );
            assert_eq!(
                validate_algebraic_root_representation(&image, PredicatePolicy::STRICT).status,
                AlgebraicRootValidationStatus::Valid
            );
        }
    }

    #[test]
    fn polynomial_image_keeps_the_degree_cap_after_square_free_reduction() {
        let square_free = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![
                real(-20),
                Real::zero(),
                real(10),
                Real::zero(),
                Real::zero(),
                real(-2),
                Real::zero(),
                Real::one(),
            ],
            ..sqrt_two_positive()
        };
        let report = transform_algebraic_root_polynomial_image(
            &square_free,
            &[Real::zero(), Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::UnsupportedDegree
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn polynomial_image_replays_cached_source_validation_strictly() {
        let mut stale = sqrt_two_positive();
        stale.interval.lower = real(2);
        stale.interval.upper = real(1);
        assert!(stale.is_valid());

        let report = transform_algebraic_root_polynomial_image(
            &stale,
            &[Real::zero(), Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::InvalidEvidence
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn polynomial_image_rejects_empty_and_nonrational_authored_storage() {
        let empty = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            empty.status,
            AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial
        );

        let exact_normal_zero =
            real(2).powi_i64(-3000).unwrap() - crate::test_support::exact_normal_positive();
        assert!(exact_normal_zero.exact_rational_ref().is_none());
        let unsupported = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[Real::zero(), Real::one(), exact_normal_zero],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            unsupported.status,
            AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial
        );
        assert!(unsupported.representation.is_none());

        let nonrational_endpoint = real(2).sqrt().unwrap();
        assert!(nonrational_endpoint.exact_rational_ref().is_none());
        let mut nonrational_interval = sqrt_two_positive();
        nonrational_interval.interval = IsolatedRootInterval {
            lower: nonrational_endpoint.clone(),
            upper: nonrational_endpoint,
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            validate_algebraic_root_representation(&nonrational_interval, PredicatePolicy::STRICT)
                .status,
            AlgebraicRootValidationStatus::Valid
        );
        let unsupported = transform_algebraic_root_polynomial_image(
            &nonrational_interval,
            &[Real::zero(), Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            unsupported.status,
            AlgebraicRootPolynomialImageStatus::InvalidEvidence
        );
        assert!(unsupported.representation.is_none());
    }

    #[test]
    fn polynomial_image_constructs_quadratic_expression_of_sqrt_two() {
        let report = transform_algebraic_root_polynomial_image(
            &sqrt_two_positive(),
            &[Real::zero(), Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
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
    fn polynomial_image_clears_disparate_denominators_before_elimination() {
        let represented = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![fraction(-2, 3), Real::zero(), fraction(1, 3)],
            ..sqrt_two_positive()
        };
        let image_polynomial = [fraction(1, 5), fraction(1, 7), fraction(1, 11)];
        let report = transform_algebraic_root_polynomial_image(
            &represented,
            &image_polynomial,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let image = report.representation.as_ref().unwrap();
        assert!(image.is_valid());
        assert_eq!(
            image.interval.lower,
            evaluate_rational_polynomial(&image_polynomial, &real(1)).unwrap()
        );
        assert_eq!(
            image.interval.upper,
            evaluate_rational_polynomial(&image_polynomial, &real(2)).unwrap()
        );
        assert!(image.polynomial_coefficients.iter().all(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_some_and(Rational::is_integer)
        }));
    }

    #[test]
    fn polynomial_image_refines_a_nonmonotone_source_interval() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_polynomial_image(
                &sqrt_two_positive(),
                &[Real::zero(), real(-3), Real::one()],
                policy,
            );

            assert_eq!(
                report.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            assert!(report.representation.as_ref().unwrap().is_valid());
        }
    }

    #[test]
    fn polynomial_image_represents_a_stationary_algebraic_map() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_polynomial_image(
                &sqrt_two_positive(),
                &[Real::zero(), real(-6), Real::zero(), Real::one()],
                policy,
            );

            assert_eq!(
                report.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            let image = report.representation.as_ref().unwrap();
            assert_eq!(
                image.polynomial_coefficients,
                vec![real(-32), Real::zero(), Real::one()]
            );
            assert!(image.is_valid());
        }
    }

    #[test]
    fn polynomial_image_refines_away_a_foreign_resultant_root() {
        let mut selected = sqrt_two_positive();
        selected.polynomial_coefficients = vec![real(6), real(-2), real(-3), Real::one()];
        selected.validation =
            validate_algebraic_root_representation(&selected, PredicatePolicy::STRICT);
        assert!(selected.is_valid());

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_polynomial_image(
                &selected,
                &[Real::zero(), real(-4), Real::one()],
                policy,
            );

            assert_eq!(
                report.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            let image = report.representation.as_ref().unwrap();
            assert!(image.is_valid());
            assert_eq!(
                compare_reals(&image.interval.upper, &real(-3), policy).value(),
                Some(Ordering::Less)
            );
        }
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
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(root.exact_point_witness(), Some(&real(34)));
        assert_eq!(report.image_coefficients, vec![real(1), real(2), real(3)]);
        assert!(root.is_valid());
    }

    proptest! {
        #[test]
        fn generated_interval_horner_contains_endpoint_and_midpoint_values(
            coefficients in prop::collection::vec(-8_i64..=8, 1..8),
            lower in -4_i64..=4,
            width in 0_i64..=4,
        ) {
            let upper = lower + width;
            let polynomial = coefficients.into_iter().map(real).collect::<Vec<_>>();
            let enclosure = evaluate_interval_polynomial(
                &polynomial,
                &ValueInterval {
                    lower: real(lower),
                    upper: real(upper),
                },
            )
            .unwrap();
            let midpoint = (real(lower) + real(upper)) / real(2);
            for point in [real(lower), midpoint.unwrap(), real(upper)] {
                let value = evaluate_rational_polynomial(&polynomial, &point).unwrap();
                prop_assert_ne!(
                    compare_reals(&enclosure.lower, &value, PredicatePolicy::STRICT).value(),
                    Some(Ordering::Greater)
                );
                prop_assert_ne!(
                    compare_reals(&enclosure.upper, &value, PredicatePolicy::STRICT).value(),
                    Some(Ordering::Less)
                );
            }
        }

        #[test]
        fn generated_constant_source_relations_are_detected_exactly(
            source_constant in -32_i64..=32,
            source_linear in -32_i64..=32,
            source_leading in (-16_i64..=16).prop_filter("nonzero leading coefficient", |value| *value != 0),
            scale in (-16_i64..=16).prop_filter("nonzero relation scale", |value| *value != 0),
            constant in -32_i64..=32,
        ) {
            let source = vec![
                real(source_constant),
                real(source_linear),
                real(source_leading),
            ];
            let image = vec![
                real(scale * source_constant + constant),
                real(scale * source_linear),
                real(scale * source_leading),
            ];
            prop_assert_eq!(
                exact_constant_source_relation(&source, &image),
                Some(real(constant))
            );

            let mut perturbed = image;
            perturbed[1] += Real::one();
            prop_assert_eq!(exact_constant_source_relation(&source, &perturbed), None);
        }

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
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootPolynomialImageStatus::Transformed);
            prop_assert_eq!(
                report.image_coefficients,
                vec![real(constant), real(linear), real(quadratic)]
            );
            let expected = constant + linear * root + quadratic * root * root;
            let transformed = report.representation.as_ref().unwrap();
            prop_assert_eq!(transformed.exact_point_witness(), Some(&real(expected)));
            prop_assert!(transformed.is_valid());
        }
    }
}
