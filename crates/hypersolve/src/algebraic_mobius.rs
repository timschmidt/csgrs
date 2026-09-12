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
//! approximations. Positive-width isolating intervals own `(lower, upper]`;
//! decreasing images refine source endpoint roots before reversing bounds.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::{Rational, Real};

use crate::algebraic::{
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    algebraic_root_interval_endpoints_are_roots, canonical_linear_value_representation,
    refine_reversed_algebraic_root_ownership, validate_algebraic_root_representation,
};
use crate::root_isolation::{
    ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS, ALGEBRAIC_IMAGE_REFINEMENT_STEPS, IsolatedRootInterval,
    IsolatedRootRefinementStatus, RootIsolationConfig, polynomial_vanishes_at_owned_root,
    refine_isolated_univariate_polynomial_interval,
};

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
    /// `c*alpha + d` vanishes at the selected root, or a conservative source
    /// interval could not be separated from a possible pole within the bound.
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
/// transform. A possibly refined source interval is mapped by exact endpoint
/// evaluation after proving the denominator does not cross zero on that branch.
/// No midpoint sampling or primitive tolerance participates in the proof.
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
/// `a*d - b*c != 0`, and `c*x + d` is certified nonzero at the selected root.
/// A conservative interval that contains a foreign pole is boundedly refined
/// until one pole-free branch remains. These checks keep the function within
/// the exact EGC paradigm: algebraic construction is exact and report-bearing,
/// while non-invertible or domain-uncertain cases are explicit blockers.
pub fn transform_algebraic_root_mobius(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootMobiusTransformReport {
    if !root.is_valid()
        || validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
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
    let Some(determinant_order) = mobius_determinant_order(
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

    if root.exact_point_witness().is_some() {
        return transform_exact_point_mobius(
            root,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            policy,
        );
    }

    let denominator_status = denominator_interval_status(
        &root.interval,
        &denominator_scale,
        &denominator_offset,
        policy,
    );
    if denominator_status == DenominatorIntervalStatus::Undecided {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("could not evaluate denominator interval exactly".to_owned()),
        );
    }
    if denominator_status == DenominatorIntervalStatus::MayContainZero {
        return transform_mobius_after_denominator_refinement(
            root,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            policy,
        );
    }

    if determinant_order == Ordering::Less {
        match algebraic_root_interval_endpoints_are_roots(root, policy) {
            Some(true) => {
                return transform_mobius_after_ownership_refinement(
                    root,
                    numerator_scale,
                    numerator_offset,
                    denominator_scale,
                    denominator_offset,
                    policy,
                );
            }
            Some(false) => {}
            None => {
                return mobius_report(
                    AlgebraicRootMobiusTransformStatus::Undecided,
                    numerator_scale,
                    numerator_offset,
                    denominator_scale,
                    denominator_offset,
                    None,
                    Some(
                        "could not decide source endpoint ownership for decreasing Mobius transform"
                            .to_owned(),
                    ),
                );
            }
        }
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
        determinant_order,
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
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
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

#[allow(clippy::too_many_arguments)]
#[cold]
fn transform_mobius_after_denominator_refinement(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootMobiusTransformReport {
    let denominator_polynomial = [denominator_offset.clone(), denominator_scale.clone()];
    match polynomial_vanishes_at_owned_root(
        &root.polynomial_coefficients,
        &denominator_polynomial,
        &root.interval,
        policy,
    ) {
        Some(true) => mobius_report(
            AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some("denominator vanishes at the selected algebraic root".to_owned()),
        ),
        Some(false) => {
            let Some(refined) = refine_root_away_from_mobius_denominator(
                root,
                &denominator_scale,
                &denominator_offset,
                policy,
            ) else {
                return mobius_report(
                    AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
                    numerator_scale,
                    numerator_offset,
                    denominator_scale,
                    denominator_offset,
                    None,
                    Some(
                        "could not produce a pole-free source interval within the refinement bound"
                            .to_owned(),
                    ),
                );
            };
            transform_algebraic_root_mobius(
                &refined,
                numerator_scale,
                numerator_offset,
                denominator_scale,
                denominator_offset,
                policy,
            )
        }
        None => mobius_report(
            AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some(
                "denominator may vanish at the selected root or on its source interval".to_owned(),
            ),
        ),
    }
}

#[allow(clippy::too_many_arguments)]
#[cold]
fn transform_mobius_after_ownership_refinement(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootMobiusTransformReport {
    let Some(refined) = refine_reversed_algebraic_root_ownership(root, policy) else {
        return mobius_report(
            AlgebraicRootMobiusTransformStatus::Undecided,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            None,
            Some(
                "could not refine source endpoint ownership for decreasing Mobius transform"
                    .to_owned(),
            ),
        );
    };
    transform_algebraic_root_mobius(
        &refined,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn transform_exact_point_mobius(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootMobiusTransformReport {
    let witness = root
        .exact_point_witness()
        .expect("exact-point Mobius path requires a point witness");
    if let (Some(witness), Some(a), Some(b), Some(c), Some(d)) = (
        witness.exact_rational_ref(),
        numerator_scale.exact_rational_ref(),
        numerator_offset.exact_rational_ref(),
        denominator_scale.exact_rational_ref(),
        denominator_offset.exact_rational_ref(),
    ) {
        let Some(value) = eval_rational_mobius_values(witness, a, b, c, d) else {
            return mobius_report(
                AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
                numerator_scale,
                numerator_offset,
                denominator_scale,
                denominator_offset,
                None,
                Some("denominator vanishes at the exact source point".to_owned()),
            );
        };
        return exact_mobius_image(
            root,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            value,
        );
    }
    let denominator = denominator_scale.clone() * witness.clone() + denominator_offset.clone();
    match compare_reals(&denominator, &Real::zero(), policy).value() {
        Some(Ordering::Equal) => {
            return mobius_report(
                AlgebraicRootMobiusTransformStatus::DenominatorMayVanish,
                numerator_scale,
                numerator_offset,
                denominator_scale,
                denominator_offset,
                None,
                Some("denominator vanishes at the exact source point".to_owned()),
            );
        }
        Some(Ordering::Less | Ordering::Greater) => {}
        None => {
            return mobius_report(
                AlgebraicRootMobiusTransformStatus::Undecided,
                numerator_scale,
                numerator_offset,
                denominator_scale,
                denominator_offset,
                None,
                Some("could not certify the denominator at the exact source point".to_owned()),
            );
        }
    }
    let Some(value) = eval_mobius(
        witness,
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
            Some("could not evaluate the Mobius transform at the exact source point".to_owned()),
        );
    };
    exact_mobius_image(
        root,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        value,
    )
}

fn refine_root_away_from_mobius_denominator(
    root: &AlgebraicRootRepresentation,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Box<AlgebraicRootRepresentation>> {
    let mut refined_root = Box::new(root.clone());
    for _ in 0..ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS {
        let refinement = refine_isolated_univariate_polynomial_interval(
            &refined_root.polynomial_coefficients,
            &refined_root.interval,
            RootIsolationConfig {
                policy,
                max_interval_width: None,
                max_refinement_steps: ALGEBRAIC_IMAGE_REFINEMENT_STEPS,
            },
        );
        if !matches!(
            refinement.status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ) {
            return None;
        }
        let interval = refinement.refined_interval?;
        if interval == refined_root.interval {
            return None;
        }
        refined_root.interval = interval;
        refined_root.validation = validate_algebraic_root_representation(&refined_root, policy);
        if !refined_root.is_valid() {
            return None;
        }
        match denominator_interval_status(
            &refined_root.interval,
            denominator_scale,
            denominator_offset,
            policy,
        ) {
            DenominatorIntervalStatus::ExcludesZero => return Some(refined_root),
            DenominatorIntervalStatus::MayContainZero => {}
            DenominatorIntervalStatus::Undecided => return None,
        }
    }
    None
}

fn mobius_determinant_order(
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if let (Some(a), Some(b), Some(c), Some(d)) = (
        numerator_scale.exact_rational_ref(),
        numerator_offset.exact_rational_ref(),
        denominator_scale.exact_rational_ref(),
        denominator_offset.exact_rational_ref(),
    ) {
        if b.is_zero() || c.is_zero() {
            return Some(rational_product_order(a, d));
        }
        if a.is_zero() || d.is_zero() {
            return Some(rational_product_order(b, c).reverse());
        }
        return Some(rational_order(&(a * d - b * c)));
    }
    let determinant = numerator_scale.clone() * denominator_offset.clone()
        - numerator_offset.clone() * denominator_scale.clone();
    compare_reals(&determinant, &Real::zero(), policy).value()
}

#[inline(always)]
fn exact_mobius_image(
    root: &AlgebraicRootRepresentation,
    numerator_scale: Real,
    numerator_offset: Real,
    denominator_scale: Real,
    denominator_offset: Real,
    value: Real,
) -> AlgebraicRootMobiusTransformReport {
    let representation = canonical_linear_value_representation(root, value);
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

/// Composes a univariate polynomial with a linear-fractional parameter map.
///
/// For a degree-`n` power-basis polynomial `P`, this returns the normalized
/// power-basis coefficients of
///
/// `(c*x + d)^n * P((a*x + b) / (c*x + d))`.
///
/// The homogeneous denominator clearing introduces no roots on an interval
/// where `c*x + d` is certified nonzero. `None` reports an empty input or a
/// transformed polynomial whose nonzero degree could not be certified under
/// `policy`; no approximate coefficient is constructed.
pub fn compose_univariate_polynomial_linear_fractional(
    polynomial: &[Real],
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    // `mobius_transformed_polynomial` accepts the forward root map
    // `y = (a*x+b)/(c*x+d)` and performs its inverse substitution. Supplying
    // the inverse map here makes that same authority perform the requested
    // direct composition without duplicating homogeneous Horner machinery.
    let inverse_numerator_scale = denominator_offset.clone();
    let inverse_numerator_offset = -numerator_offset.clone();
    let inverse_denominator_scale = -denominator_scale.clone();
    let inverse_denominator_offset = numerator_scale.clone();
    mobius_transformed_polynomial(
        polynomial,
        &inverse_numerator_scale,
        &inverse_numerator_offset,
        &inverse_denominator_scale,
        &inverse_denominator_offset,
        policy,
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
    if polynomial
        .iter()
        .chain([
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
        ])
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
    {
        return mobius_transformed_polynomial_horner(
            polynomial,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            policy,
        );
    }

    if let Some(transformed) = mobius_transformed_polynomial_real_horner(
        polynomial,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    ) {
        return Some(transformed);
    }

    mobius_transformed_polynomial_power_sum(
        polynomial,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    )
}

fn mobius_transformed_polynomial_real_horner(
    polynomial: &[Real],
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let inverse_numerator = (-numerator_offset.clone(), denominator_offset);
    let inverse_denominator = (numerator_scale, -denominator_scale.clone());
    let mut transformed = Vec::with_capacity(polynomial.len());
    transformed.push(polynomial.last()?.clone());
    let mut denominator_power = Vec::with_capacity(polynomial.len());
    denominator_power.push(Real::one());
    for coefficient in polynomial[..polynomial.len() - 1].iter().rev() {
        real_polynomial_mul_linear_in_place(
            &mut transformed,
            &inverse_numerator.0,
            inverse_numerator.1,
        );
        real_polynomial_mul_linear_in_place(
            &mut denominator_power,
            inverse_denominator.0,
            &inverse_denominator.1,
        );
        for (target, basis) in transformed.iter_mut().zip(&denominator_power) {
            *target = target.clone() + coefficient.clone() * basis.clone();
        }
    }
    trim_polynomial(transformed, policy)
}

fn real_polynomial_mul_linear_in_place(polynomial: &mut Vec<Real>, constant: &Real, linear: &Real) {
    let old_len = polynomial.len();
    debug_assert!(old_len < polynomial.capacity());
    polynomial.push(polynomial[old_len - 1].clone() * linear.clone());
    for index in (1..old_len).rev() {
        polynomial[index] = polynomial[index].clone() * constant.clone()
            + polynomial[index - 1].clone() * linear.clone();
    }
    polynomial[0] = polynomial[0].clone() * constant.clone();
}

fn mobius_transformed_polynomial_power_sum(
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

fn mobius_transformed_polynomial_horner(
    polynomial: &[Real],
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    _policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let inverse_numerator = (
        -numerator_offset.exact_rational_ref()?,
        denominator_offset.exact_rational_ref()?,
    );
    let inverse_denominator = (
        numerator_scale.exact_rational_ref()?,
        -denominator_scale.exact_rational_ref()?,
    );
    let mut transformed = Vec::with_capacity(polynomial.len());
    transformed.push(polynomial.last()?.exact_rational_ref()?.clone());
    if inverse_denominator.1.is_zero() {
        let mut denominator_power = Rational::one();
        for coefficient in polynomial[..polynomial.len() - 1].iter().rev() {
            rational_polynomial_mul_linear_in_place(
                &mut transformed,
                &inverse_numerator.0,
                inverse_numerator.1,
            );
            denominator_power = &denominator_power * inverse_denominator.0;
            transformed[0] =
                &transformed[0] + coefficient.exact_rational_ref()? * &denominator_power;
        }
        return Some(trim_rational_polynomial_into_reals(transformed));
    }
    let mut denominator_power = Vec::with_capacity(polynomial.len());
    denominator_power.push(Rational::one());
    for coefficient in polynomial[..polynomial.len() - 1].iter().rev() {
        rational_polynomial_mul_linear_in_place(
            &mut transformed,
            &inverse_numerator.0,
            inverse_numerator.1,
        );
        rational_polynomial_mul_linear_in_place(
            &mut denominator_power,
            inverse_denominator.0,
            &inverse_denominator.1,
        );
        let coefficient = coefficient.exact_rational_ref()?;
        for (target, basis) in transformed.iter_mut().zip(&denominator_power) {
            *target = &*target + coefficient * basis;
        }
    }
    Some(trim_rational_polynomial_into_reals(transformed))
}

fn trim_rational_polynomial_into_reals(mut polynomial: Vec<Rational>) -> Vec<Real> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(Rational::is_zero) {
        polynomial.pop();
    }
    polynomial.into_iter().map(Real::from).collect()
}

fn rational_polynomial_mul_linear_in_place(
    polynomial: &mut Vec<Rational>,
    constant: &Rational,
    linear: &Rational,
) {
    let old_len = polynomial.len();
    debug_assert!(old_len < polynomial.capacity());
    polynomial.push(&polynomial[old_len - 1] * linear);
    for index in (1..old_len).rev() {
        polynomial[index] = &polynomial[index] * constant + &polynomial[index - 1] * linear;
    }
    polynomial[0] = &polynomial[0] * constant;
}

fn mobius_transformed_interval(
    interval: &IsolatedRootInterval,
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    determinant_order: Ordering,
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let first = eval_mobius(
        &interval.lower,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    )?;
    let second = eval_mobius(
        &interval.upper,
        numerator_scale,
        numerator_offset,
        denominator_scale,
        denominator_offset,
        policy,
    )?;
    let (lower, upper) = match determinant_order {
        Ordering::Less => (second, first),
        Ordering::Greater => (first, second),
        Ordering::Equal => return None,
    };
    let exact_root = match &interval.exact_root {
        Some(root) => {
            let value = eval_mobius(
                root,
                numerator_scale,
                numerator_offset,
                denominator_scale,
                denominator_offset,
                policy,
            )?;
            value.exact_rational_ref().is_some().then_some(value)
        }
        None => None,
    };
    Some(IsolatedRootInterval {
        lower,
        upper,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DenominatorIntervalStatus {
    ExcludesZero,
    MayContainZero,
    Undecided,
}

fn denominator_interval_status(
    interval: &IsolatedRootInterval,
    scale: &Real,
    offset: &Real,
    policy: PredicatePolicy,
) -> DenominatorIntervalStatus {
    if let (Some(lower), Some(upper), Some(scale), Some(offset)) = (
        interval.lower.exact_rational_ref(),
        interval.upper.exact_rational_ref(),
        scale.exact_rational_ref(),
        offset.exact_rational_ref(),
    ) {
        if scale.is_zero() {
            return if matches!(rational_order(offset), Ordering::Less | Ordering::Greater) {
                DenominatorIntervalStatus::ExcludesZero
            } else {
                DenominatorIntervalStatus::MayContainZero
            };
        }
        let is_point = lower == upper;
        let lower = scale * lower + offset;
        let lower = rational_order(&lower);
        if is_point {
            return if matches!(lower, Ordering::Less | Ordering::Greater) {
                DenominatorIntervalStatus::ExcludesZero
            } else {
                DenominatorIntervalStatus::MayContainZero
            };
        }
        let upper = scale * upper + offset;
        let upper = rational_order(&upper);
        return if matches!(
            (lower, upper),
            (Ordering::Less, Ordering::Less) | (Ordering::Greater, Ordering::Greater)
        ) {
            DenominatorIntervalStatus::ExcludesZero
        } else {
            DenominatorIntervalStatus::MayContainZero
        };
    }
    let Some(interval) = linear_interval_image(interval, scale, offset, policy) else {
        return DenominatorIntervalStatus::Undecided;
    };
    if interval_contains_zero(&interval, policy).unwrap_or(true) {
        DenominatorIntervalStatus::MayContainZero
    } else {
        DenominatorIntervalStatus::ExcludesZero
    }
}

fn eval_mobius(
    value: &Real,
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    if let (Some(value), Some(a), Some(b), Some(c), Some(d)) = (
        value.exact_rational_ref(),
        numerator_scale.exact_rational_ref(),
        numerator_offset.exact_rational_ref(),
        denominator_scale.exact_rational_ref(),
        denominator_offset.exact_rational_ref(),
    ) {
        return eval_rational_mobius_values(value, a, b, c, d);
    }
    let numerator = numerator_scale.clone() * value.clone() + numerator_offset.clone();
    let denominator = denominator_scale.clone() * value.clone() + denominator_offset.clone();
    match numerator / denominator {
        Ok(quotient) => Some(quotient),
        Err(_) => eval_mobius_policy_fallback(
            value,
            numerator_scale,
            numerator_offset,
            denominator_scale,
            denominator_offset,
            policy,
        ),
    }
}

fn eval_rational_mobius_values(
    value: &Rational,
    numerator_scale: &Rational,
    numerator_offset: &Rational,
    denominator_scale: &Rational,
    denominator_offset: &Rational,
) -> Option<Real> {
    let numerator = numerator_scale * value + numerator_offset;
    let denominator = denominator_scale * value + denominator_offset;
    (!denominator.is_zero()).then(|| Real::from(numerator / denominator))
}

#[allow(clippy::too_many_arguments)]
#[cold]
fn eval_mobius_policy_fallback(
    value: &Real,
    numerator_scale: &Real,
    numerator_offset: &Real,
    denominator_scale: &Real,
    denominator_offset: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let denominator = denominator_scale.clone() * value.clone() + denominator_offset.clone();
    if !matches!(
        compare_reals(&denominator, &Real::zero(), policy).value(),
        Some(Ordering::Less | Ordering::Greater)
    ) {
        return None;
    }
    let numerator = numerator_scale.clone() * value.clone() + numerator_offset.clone();
    Some(numerator * denominator.inverse_ref_assuming_nonzero().ok()?)
}

#[derive(Clone, Debug)]
struct AlgebraicValueInterval {
    lower: Real,
    upper: Real,
}

fn interval_contains_zero(value: &AlgebraicValueInterval, policy: PredicatePolicy) -> Option<bool> {
    let lower = compare_reals(&value.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals(&value.upper, &Real::zero(), policy).value()?;
    Some(lower != Ordering::Greater && upper != Ordering::Less)
}

fn rational_order(value: &Rational) -> Ordering {
    if value.is_negative() {
        Ordering::Less
    } else if value.is_zero() {
        Ordering::Equal
    } else {
        Ordering::Greater
    }
}

fn rational_product_order(left: &Rational, right: &Rational) -> Ordering {
    match (rational_order(left), rational_order(right)) {
        (Ordering::Equal, _) | (_, Ordering::Equal) => Ordering::Equal,
        (Ordering::Less, Ordering::Greater) | (Ordering::Greater, Ordering::Less) => Ordering::Less,
        (Ordering::Less, Ordering::Less) | (Ordering::Greater, Ordering::Greater) => {
            Ordering::Greater
        }
    }
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
        match compare_reals(trailing, &Real::zero(), policy).value()? {
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
    use crate::root_isolation::{
        IsolatedRootRefinementStatus, RootIsolationConfig,
        refine_isolated_univariate_polynomial_interval,
    };

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
            PredicatePolicy::APPROXIMATE_512,
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
    fn mobius_transform_retains_an_arbitrary_exact_point_witness() {
        let source = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: crate::SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(2),
                upper: real(2),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let scale = real(2).sqrt().unwrap();
        let offset = Real::pi();
        let expected = real(2) * scale.clone() + offset.clone();
        let report = transform_algebraic_root_mobius(
            &source,
            scale,
            offset,
            Real::zero(),
            Real::one(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("exact-Real point image");
        assert_eq!(transformed.exact_point_witness(), Some(&expected));
        assert_eq!(transformed.interval.lower, expected);
        assert_eq!(
            validate_algebraic_root_representation(&transformed, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid
        );
    }

    #[test]
    fn mobius_interval_reuses_policy_nonzero_denominator() {
        let scale = crate::test_support::exact_normal_positive();
        assert_eq!(scale.inverse_ref(), Err(hyperreal::Problem::UnknownZero));
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            scale.clone(),
            Real::zero(),
            Real::zero(),
            scale,
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("identity Mobius image");
        assert_eq!(
            compare_reals(
                &transformed.interval.lower,
                &Real::one(),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &transformed.interval.upper,
                &real(2),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert!(transformed.is_valid());

        assert!(
            eval_mobius(
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &crate::test_support::terminal_zero(),
                PredicatePolicy::STRICT,
            )
            .is_none()
        );
    }

    #[test]
    fn mobius_transform_constructs_shifted_reciprocal() {
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::one(),
            real(1),
            Real::one(),
            real(3),
            PredicatePolicy::APPROXIMATE_512,
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
    fn decreasing_mobius_transform_excludes_a_foreign_source_endpoint_root() {
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(2), real(-2), real(-1), Real::one()],
            ..sqrt_two()
        };

        let report = transform_algebraic_root_mobius(
            &source,
            real(-1),
            Real::zero(),
            Real::zero(),
            Real::one(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("refined reflected root");
        assert!(transformed.interval.upper < real(-1));
        assert!(matches!(
            refine_isolated_univariate_polynomial_interval(
                &transformed.polynomial_coefficients,
                &transformed.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));

        let selected_upper_endpoint = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-2), Real::one()],
            ..sqrt_two()
        };
        let endpoint_report = transform_algebraic_root_mobius(
            &selected_upper_endpoint,
            real(-1),
            Real::zero(),
            Real::zero(),
            Real::one(),
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            endpoint_report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let endpoint_image = endpoint_report
            .representation
            .expect("owned endpoint must survive reversal");
        assert_eq!(endpoint_image.interval.lower, real(-2));
        assert_eq!(endpoint_image.interval.upper, real(-2));
        assert_eq!(endpoint_image.interval.exact_root, Some(real(-2)));
    }

    #[test]
    fn mobius_transform_refines_away_from_an_excluded_endpoint_pole() {
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::zero(),
            Real::one(),
            Real::one(),
            real(-1),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("pole-free reciprocal image");
        assert!(matches!(
            refine_isolated_univariate_polynomial_interval(
                &transformed.polynomial_coefficients,
                &transformed.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));
    }

    #[test]
    fn mobius_transform_refines_away_from_an_interior_foreign_pole() {
        let report = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::zero(),
            Real::one(),
            Real::one(),
            (real(-3) / real(2)).unwrap(),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("separated reciprocal image");
        assert!(matches!(
            refine_isolated_univariate_polynomial_interval(
                &transformed.polynomial_coefficients,
                &transformed.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));
    }

    #[test]
    fn mobius_transform_rejects_a_pole_at_the_owned_root() {
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two()
        };
        let report = transform_algebraic_root_mobius(
            &source,
            Real::zero(),
            Real::one(),
            Real::one(),
            real(-2),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::DenominatorMayVanish
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn mobius_exact_point_ignores_foreign_poles_in_its_outer_interval() {
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(3),
                exact_root: Some(real(2)),
                distinct_root_count: 1,
            },
            ..sqrt_two()
        };
        let report = transform_algebraic_root_mobius(
            &source,
            Real::zero(),
            Real::one(),
            Real::one(),
            real(-1),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("exact point image");
        assert_eq!(transformed.interval.exact_root, Some(Real::one()));
        assert_eq!(transformed.interval.lower, Real::one());
        assert_eq!(transformed.interval.upper, Real::one());
    }

    #[test]
    fn mobius_transform_refines_foreign_poles_and_rejects_constant_maps() {
        let foreign_pole = transform_algebraic_root_mobius(
            &sqrt_two(),
            Real::one(),
            Real::zero(),
            Real::one(),
            real(-1),
            PredicatePolicy::APPROXIMATE_512,
        );
        assert_eq!(
            foreign_pole.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        assert!(foreign_pole.representation.is_some());

        let constant = transform_algebraic_root_mobius(
            &sqrt_two(),
            real(2),
            real(4),
            Real::one(),
            real(2),
            PredicatePolicy::APPROXIMATE_512,
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
            ..sqrt_two()
        };

        let report = transform_algebraic_root_mobius(
            &rational,
            real(2),
            real(1),
            Real::one(),
            real(1),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.exact_point_witness(),
            Some(&(real(7) / real(4)).unwrap())
        );
        assert_eq!(root.interval.lower, (real(7) / real(4)).unwrap());
        assert_eq!(root.interval.upper, (real(7) / real(4)).unwrap());
        assert_eq!(
            root.polynomial_coefficients,
            vec![-(real(7) / real(4)).unwrap(), Real::one()]
        );
        assert!(root.is_valid());
    }

    #[test]
    fn mobius_transform_replays_cached_source_validation_strictly() {
        let mut stale = sqrt_two();
        stale.interval.lower = real(3);
        stale.interval.upper = real(2);
        assert!(stale.is_valid());

        let report = transform_algebraic_root_mobius(
            &stale,
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::InvalidEvidence
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn mobius_transform_preserves_exact_real_source_coefficients() {
        let sqrt_two_value = real(2).sqrt().expect("positive exact square root");
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![-sqrt_two_value.clone(), Real::one()],
            ..sqrt_two()
        };
        assert_eq!(
            validate_algebraic_root_representation(&source, PredicatePolicy::STRICT).status,
            AlgebraicRootValidationStatus::Valid
        );

        let report = transform_algebraic_root_mobius(
            &source,
            Real::one(),
            Real::one(),
            Real::one(),
            real(3),
            PredicatePolicy::STRICT,
        );

        assert_eq!(
            report.status,
            AlgebraicRootMobiusTransformStatus::Transformed
        );
        let transformed = report.representation.expect("exact-Real Mobius image");
        assert_eq!(transformed.polynomial_coefficients.len(), 2);
        assert_eq!(
            compare_reals(
                &transformed.polynomial_coefficients[0],
                &(-Real::one() - sqrt_two_value.clone()),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &transformed.polynomial_coefficients[1],
                &(real(3) + sqrt_two_value),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(transformed.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(transformed.interval.upper, (real(3) / real(5)).unwrap());
        assert!(transformed.is_valid());
    }

    #[test]
    fn rational_homogeneous_horner_matches_retained_power_sum() {
        let polynomial = [real(-7), real(3), Real::zero(), real(-2), real(5), real(1)];
        let numerator_scale = real(2);
        let numerator_offset = real(-1);
        let denominator_scale = real(1);
        let denominator_offset = real(3);

        let horner = mobius_transformed_polynomial_horner(
            &polynomial,
            &numerator_scale,
            &numerator_offset,
            &denominator_scale,
            &denominator_offset,
            PredicatePolicy::APPROXIMATE_512,
        )
        .unwrap();
        let power_sum = mobius_transformed_polynomial_power_sum(
            &polynomial,
            &numerator_scale,
            &numerator_offset,
            &denominator_scale,
            &denominator_offset,
            PredicatePolicy::APPROXIMATE_512,
        )
        .unwrap();

        assert_eq!(horner, power_sum);
    }

    #[test]
    fn exact_real_homogeneous_horner_matches_retained_power_sum() {
        let sqrt_two = real(2).sqrt().expect("positive exact square root");
        let polynomial = [
            -sqrt_two.clone(),
            Real::pi(),
            Real::zero(),
            sqrt_two,
            Real::one(),
        ];
        let numerator_scale = real(2);
        let numerator_offset = real(-1);
        let denominator_scale = real(1);
        let denominator_offset = real(3);

        let horner = mobius_transformed_polynomial_real_horner(
            &polynomial,
            &numerator_scale,
            &numerator_offset,
            &denominator_scale,
            &denominator_offset,
            PredicatePolicy::STRICT,
        )
        .unwrap();
        let power_sum = mobius_transformed_polynomial_power_sum(
            &polynomial,
            &numerator_scale,
            &numerator_offset,
            &denominator_scale,
            &denominator_offset,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(horner.len(), power_sum.len());
        for (horner, power_sum) in horner.iter().zip(&power_sum) {
            assert_eq!(
                compare_reals(horner, power_sum, PredicatePolicy::STRICT).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn linear_fractional_composition_clears_the_authored_denominator() {
        // P(t) = t^2 - 2 under t = 1 / (1-u) becomes
        // (1-u)^2 P(1/(1-u)) = -1 + 4u - 2u^2.
        let transformed = compose_univariate_polynomial_linear_fractional(
            &[real(-2), Real::zero(), Real::one()],
            &Real::zero(),
            &Real::one(),
            &real(-1),
            &Real::one(),
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(transformed, vec![real(-1), real(4), real(-2)]);
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn generated_exact_real_homogeneous_horner_matches_retained_power_sum(
            coefficient_pairs in prop::collection::vec((-3_i8..=3, -3_i8..=3), 1..6),
        ) {
            let sqrt_two = real(2).sqrt().expect("positive exact square root");
            let polynomial = coefficient_pairs
                .into_iter()
                .map(|(rational, radical)| {
                    real(i64::from(rational)) + real(i64::from(radical)) * sqrt_two.clone()
                })
                .collect::<Vec<_>>();
            let numerator_scale = real(2);
            let numerator_offset = real(-1);
            let denominator_scale = real(1);
            let denominator_offset = real(3);

            let horner = mobius_transformed_polynomial_real_horner(
                &polynomial,
                &numerator_scale,
                &numerator_offset,
                &denominator_scale,
                &denominator_offset,
                PredicatePolicy::STRICT,
            );
            let power_sum = mobius_transformed_polynomial_power_sum(
                &polynomial,
                &numerator_scale,
                &numerator_offset,
                &denominator_scale,
                &denominator_offset,
                PredicatePolicy::STRICT,
            );

            prop_assert!(horner.is_some());
            prop_assert!(power_sum.is_some());
            let horner = horner.unwrap();
            let power_sum = power_sum.unwrap();
            prop_assert_eq!(horner.len(), power_sum.len());
            for (horner, power_sum) in horner.iter().zip(&power_sum) {
                prop_assert_eq!(
                    compare_reals(horner, power_sum, PredicatePolicy::STRICT).value(),
                    Some(Ordering::Equal)
                );
            }
        }
    }

    proptest! {
        #[test]
        fn generated_rational_homogeneous_horner_matches_retained_power_sum(
            coefficients in prop::collection::vec(-8_i16..=8, 1..7),
            numerator_scale in -5_i16..=5,
            numerator_offset in -5_i16..=5,
            denominator_scale in -5_i16..=5,
            denominator_offset in -5_i16..=5,
        ) {
            let polynomial: Vec<Real> =
                coefficients.into_iter().map(|coefficient| real(i64::from(coefficient))).collect();
            let numerator_scale = real(i64::from(numerator_scale));
            let numerator_offset = real(i64::from(numerator_offset));
            let denominator_scale = real(i64::from(denominator_scale));
            let denominator_offset = real(i64::from(denominator_offset));

            let horner = mobius_transformed_polynomial_horner(
                &polynomial,
                &numerator_scale,
                &numerator_offset,
                &denominator_scale,
                &denominator_offset,
                PredicatePolicy::APPROXIMATE_512,
            );
            let power_sum = mobius_transformed_polynomial_power_sum(
                &polynomial,
                &numerator_scale,
                &numerator_offset,
                &denominator_scale,
                &denominator_offset,
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(horner, power_sum);
        }

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
                ..sqrt_two()
            };

            let report = transform_algebraic_root_mobius(
                &represented,
                real(a),
                real(b),
                real(c),
                real(d),
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootMobiusTransformStatus::Transformed);
            let expected = (real(a * root + b) / real(c * root + d)).unwrap();
            let transformed = report.representation.as_ref().unwrap();
            prop_assert_eq!(transformed.exact_point_witness(), Some(&expected));
            prop_assert_eq!(
                &transformed.polynomial_coefficients,
                &vec![-expected.clone(), Real::one()]
            );
            prop_assert_eq!(&transformed.interval.lower, &expected);
            prop_assert_eq!(&transformed.interval.upper, &expected);
            prop_assert!(transformed.is_valid());
        }

        #[test]
        fn generated_interval_mobius_keeps_endpoint_order_away_from_a_pole(
            lower in 1_i16..=12,
            width in 1_i16..=8,
            numerator_scale in -4_i16..=4,
            numerator_offset in -4_i16..=4,
            denominator_scale in -4_i16..=4,
            denominator_offset in -8_i16..=8,
        ) {
            let lower = i64::from(lower);
            let upper = lower + i64::from(width);
            let a = i64::from(numerator_scale);
            let b = i64::from(numerator_offset);
            let c = i64::from(denominator_scale);
            let d = i64::from(denominator_offset);
            prop_assume!(a * d - b * c != 0);
            let denominator_at_lower = c * lower + d;
            let denominator_at_upper = c * upper + d;
            prop_assume!(
                (denominator_at_lower < 0 && denominator_at_upper < 0)
                    || (denominator_at_lower > 0 && denominator_at_upper > 0)
            );
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
                PredicatePolicy::APPROXIMATE_512,
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
