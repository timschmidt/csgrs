//! Linear-fractional construction for represented algebraic roots.
//!
//! This module is a narrow algebraic-construction backend for
//! [`crate::AlgebraicRootRepresentation`]. It implements
//! `beta = (a * alpha + b) / (c * alpha + d)` by exact polynomial substitution
//! and exact interval image construction. It is not a generic algebraic-number
//! field: unsupported denominator and invertibility cases remain reportable
//! blockers.
//!
//! The construction follows the exact-object boundary advocated by the exactness boundary:
//! coordinates are represented by replayable algebraic evidence, and later
//! predicates decide topology from certified reports rather than primitive
//! approximations. Isolating intervals follow the standard real-root model.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, validate_algebraic_root_representation,
};
use crate::root_isolation::IsolatedRootInterval;

/// Status for constructing a linear-fractional image of an algebraic root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootMobiusTransformStatus {
    /// The image was represented by exact transformed polynomial and interval
    /// evidence.
    Transformed,
    /// The input represented root failed structural validation.
    InvalidEvidence,
    /// `a*d - b*c` is zero, so the transform is constant rather than
    /// invertible on algebraic evidence.
    NonInvertible,
    /// `c*alpha + d` may be zero on the source isolating interval.
    DenominatorMayVanish,
    /// The transformed polynomial or interval did not validate.
    InvalidTransformedEvidence,
    /// Exact comparisons or scalar divisions did not decide.
    Undecided,
}

/// Report for `beta = (a * alpha + b) / (c * alpha + d)`.
///
/// Coefficients are exact [`Real`] values and are named by their algebraic
/// role. For a source root of `P(x)` with degree `n`, the resulting polynomial
/// is
///
/// `Q(y) = (a - c*y)^n * P((d*y - b) / (a - c*y))`.
///
/// This is the standard exact inverse-substitution construction for a Mobius
/// transform. The source interval is mapped by exact endpoint evaluation after
/// proving the denominator does not cross zero. No midpoint sampling or
/// primitive tolerance participates in the proof.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootMobiusTransformReport {
    /// Final transform status.
    pub status: AlgebraicRootMobiusTransformStatus,
    /// Numerator scale `a`.
    pub numerator_scale: Real,
    /// Numerator offset `b`.
    pub numerator_offset: Real,
    /// Denominator scale `c`.
    pub denominator_scale: Real,
    /// Denominator offset `d`.
    pub denominator_offset: Real,
    /// Resulting represented root when construction succeeds or when failed
    /// validation has useful evidence to inspect.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Construct the exact linear-fractional image of a represented root.
///
/// The transform is accepted only when the source evidence is valid,
/// `a*d - b*c != 0`, and `c*x + d` is certified nonzero over the full source
/// isolating interval. These checks keep the function within the exact EGC
/// paradigm: algebraic construction is exact and report-bearing, while
/// non-invertible or domain-uncertain cases are explicit blockers.
pub fn transform_algebraic_root_mobius(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootMobiusTransformReport {
    if !root.is_valid() {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::InvalidEvidence,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
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
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::InvalidEvidence,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some(
                "linear-fractional construction requires exact-rational polynomial evidence"
                    .to_owned(),
            ),
        );
    }

    let determinant = numerator_scale.clone() * denominator_offset.clone()
        - numerator_offset.clone() * denominator_scale.clone();
    let Some(determinant_order) =
        compare_reals_with_policy(&determinant, &Real::zero(), policy).value()
    else {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("could not certify linear-fractional determinant".to_owned()),
        );
    };
    if determinant_order == Ordering::Equal {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::NonInvertible,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("linear-fractional construction requires nonzero a*d - b*c".to_owned()),
        );
    }

    let Some(denominator_interval) = linear_interval_image(
        &root.interval,
        &denominator_scale,
        &denominator_offset,
        policy,
    ) else {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("could not evaluate denominator interval exactly".to_owned()),
        );
    };
    if interval_contains_zero(&denominator_interval, policy).unwrap_or(true) {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("denominator may vanish on the source isolating interval".to_owned()),
        );
    }

    let Some(polynomial_coefficients) = mobius_transformed_polynomial(
        &root.polynomial_coefficients,
        &numerator_scale,
        &numerator_offset,
        &denominator_scale,
        &denominator_offset,
        policy,
    ) else {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("could not construct transformed polynomial exactly".to_owned()),
        );
    };
    let Some(interval) = mobius_transformed_interval(
        &root.interval,
        &numerator_scale,
        &numerator_offset,
        &denominator_scale,
        &denominator_offset,
        policy,
    ) else {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("could not construct transformed interval exactly".to_owned()),
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
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::InvalidTransformedEvidence,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            Some(representation),
            Some("transformed algebraic-root evidence did not validate".to_owned()),
        );
    }

    mobius_report(
        AlgebraicRootMobiusTransformStatus::Transformed,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        Some(representation),
        None,
    )
}

fn mobius_transformed_polynomial(
    polynomial: &[Real],
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let degree = polynomial.len().checked_sub(1)?;
    let inverse_numerator = vec![-numerator_offset.clone(), denominator_offset.clone()];
    let inverse_denominator = vec![numerator_scale.clone(), -denominator_scale.clone()];
    let mut transformed = vec![Real::zero(); degree + 1];
    for (power, coefficient) in polynomial.iter().enumerate() {
        let left = polynomial_pow(&inverse_numerator, power);
        let right = polynomial_pow(&inverse_denominator, degree - power);
        let mut term = polynomial_mul(&left, &right);
        for value in &mut term {
            *value = value.clone() * coefficient.clone();
        }
        polynomial_accumulate(&mut transformed, &term);
    }
    trim_polynomial(transformed, policy)
}

fn mobius_transformed_interval(
    interval: &IsolatedRootInterval,
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let first = eval_mobius(
        &interval.lower,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
    )?;
    let second = eval_mobius(
        &interval.upper,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
    )?;
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    let exact_root = match &interval.exact_root {
        Some(root) => Some(eval_mobius(
            root,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
        )?),
        None => None,
    };
    Some(IsolatedRootInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
        exact_root,
        distinct_root_count: interval.distinct_root_count,
    })
}

fn linear_interval_image(
    interval: &IsolatedRootInterval,
    scale: &Real,
    offset: &Real,
    policy: PredicatePolicy,
) -> Option<AlgebraicValueInterval> {
    let first = scale.clone() * interval.lower.clone() + offset.clone();
    let second = scale.clone() * interval.upper.clone() + offset.clone();
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    Some(AlgebraicValueInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
    })
}

fn eval_mobius(
    value: &Real,
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
) -> Option<Real> {
    let numerator = numerator_scale.clone() * value.clone() + numerator_offset.clone();
    let denominator = denominator_scale.clone() * value.clone() + denominator_offset.clone();
    (numerator / denominator).ok()
}

#[derive(Clone, Debug)]
struct AlgebraicValueInterval {
    lower: Real,
    upper: Real,
}

fn interval_contains_zero(value: &AlgebraicValueInterval, policy: PredicatePolicy) -> Option<bool> {
    let lower = compare_reals_with_policy(&value.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&value.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn polynomial_pow(base: &[Real], exponent: usize) -> Vec<Real> {
    let mut result = vec![Real::one()];
    for _ in 0..exponent {
        result = polynomial_mul(&result, base);
    }
    result
}

fn polynomial_mul(left: &[Real], right: &[Real]) -> Vec<Real> {
    let mut product = vec![Real::zero(); left.len() + right.len() - 1];
    for (left_index, left_coefficient) in left.iter().enumerate() {
        for (right_index, right_coefficient) in right.iter().enumerate() {
            let index = left_index + right_index;
            product[index] =
                product[index].clone() + left_coefficient.clone() * right_coefficient.clone();
        }
    }
    product
}

fn polynomial_accumulate(target: &mut Vec<Real>, term: &[Real]) {
    if target.len() < term.len() {
        target.resize(term.len(), Real::zero());
    }
    for (index, coefficient) in term.iter().enumerate() {
        target[index] = target[index].clone() + coefficient.clone();
    }
}

fn trim_polynomial(mut polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
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

fn mobius_report(
    status: AlgebraicRootMobiusTransformStatus,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    representation: Option<AlgebraicRootRepresentation>,
    message: Option<String>,
) -> AlgebraicRootMobiusTransformReport {
    AlgebraicRootMobiusTransformReport {
        status,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
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

    fn sqrt_two() -> AlgebraicRootRepresentation {
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
    fn mobius_transform_constructs_inverse_of_sqrt_two() {
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            real(2),
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(-2), Real::zero(), real(4)]
        );
        assert_eq!(root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(root.interval.upper, Real::one());
        assert!(root.is_valid());
    }

    #[test]
    fn mobius_transform_constructs_shifted_reciprocal() {
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::one(),
            real(1),
            Real::one(),
            real(3),
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.polynomial_coefficients,
            vec![real(-1), real(-2), real(7)]
        );
        assert_eq!(root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(root.interval.upper, (real(3) / real(5)).unwrap());
        assert!(root.is_valid());
    }

    #[test]
    fn mobius_transform_rejects_poles_and_constant_maps() {
        let pole = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::one(),
            Real::zero(),
            Real::one(),
            real(-1),
            PredicatePolicy,
        );
        assert_eq!(
            pole.status,
            AlgebraicRootMobiusTransformStatus::DenominatorMayVanish
        );

        let constant = transform_algebraic_root_mobius(
            &sqrt_two(),
            real(2),
            real(4),
            Real::one(),
            real(2),
            PredicatePolicy,
        );
        assert_eq!(
            constant.status,
            AlgebraicRootMobiusTransformStatus::NonInvertible
        );
    }

    #[test]
    fn mobius_transform_preserves_exact_rational_witnesses() {
        let rational = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-3), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(3),
                upper: real(3),
                exact_root: Some(real(3)),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            ..sqrt_two()
        };

        let report = transform_algebraic_root_mobius(
            &rational,
            real(2),
            real(1),
            Real::one(),
            real(1),
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.exact_rational_witness(),
            Some(&(real(7) / real(4)).unwrap())
        );
        assert_eq!(root.interval.lower, (real(7) / real(4)).unwrap());
        assert_eq!(root.interval.upper, (real(7) / real(4)).unwrap());
        assert!(root.is_valid());
    }

    proptest! {
        #[test]
        fn generated_rational_witness_mobius_matches_exact_fraction(
            root in -24_i16..=24,
            numerator_scale in -8_i16..=8,
            numerator_offset in -8_i16..=8,
            denominator_scale in -8_i16..=8,
            denominator_offset in -8_i16..=8,
        ) {
            let root = i64::from(root);
            let a = i64::from(numerator_scale);
            let b = i64::from(numerator_offset);
            let c = i64::from(denominator_scale);
            let d = i64::from(denominator_offset);
            prop_assume!(a * d - b * c != 0);
            prop_assume!(c * root + d != 0);
            let represented = AlgebraicRootRepresentation {
                polynomial_coefficients: vec![real(-root), Real::one()],
                interval: IsolatedRootInterval {
                    lower: real(root),
                    upper: real(root),
                    exact_root: Some(real(root)),
                    distinct_root_count: 1,
                },
                kind: AlgebraicRootKind::ExactRationalWitness,
                ..sqrt_two()
            };

            let report = transform_algebraic_root_mobius(
                &represented,
                real(a),
                real(b),
                real(c),
                real(d),
                PredicatePolicy,
            );

            prop_assert_eq!(report.status, AlgebraicRootMobiusTransformStatus::Transformed);
            let expected = (real(a * root + b) / real(c * root + d)).unwrap();
            let transformed = report.representation.as_ref().unwrap();
            prop_assert_eq!(transformed.exact_rational_witness(), Some(&expected));
            prop_assert!(transformed.is_valid());
        }

        #[test]
        fn generated_interval_mobius_keeps_endpoint_order_for_positive_denominator(
            lower in 1_i16..=12,
            width in 1_i16..=8,
            numerator_scale in -4_i16..=4,
            numerator_offset in -4_i16..=4,
            denominator_scale in 0_i16..=4,
            denominator_offset in 1_i16..=8,
        ) {
            let lower = i64::from(lower);
            let upper = lower + i64::from(width);
            let a = i64::from(numerator_scale);
            let b = i64::from(numerator_offset);
            let c = i64::from(denominator_scale);
            let d = i64::from(denominator_offset);
            prop_assume!(a * d - b * c != 0);
            let root = AlgebraicRootRepresentation {
                interval: IsolatedRootInterval {
                    lower: real(lower),
                    upper: real(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                ..sqrt_two()
            };

            let report = transform_algebraic_root_mobius(
                &root,
                real(a),
                real(b),
                real(c),
                real(d),
                PredicatePolicy,
            );

            prop_assert_eq!(report.status, AlgebraicRootMobiusTransformStatus::Transformed);
            let transformed = report.representation.as_ref().unwrap();
            let left = (real(a * lower + b) / real(c * lower + d)).unwrap();
            let right = (real(a * upper + b) / real(c * upper + d)).unwrap();
            let expected_lower = if left < right { left.clone() } else { right.clone() };
            let expected_upper = if left < right { right } else { left };
            prop_assert_eq!(&transformed.interval.lower, &expected_lower);
            prop_assert_eq!(&transformed.interval.upper, &expected_upper);
            prop_assert!(transformed.is_valid());
        }
    }
}
