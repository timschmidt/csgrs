//! Exact signed square-root images of represented real algebraic roots.
//!
//! If `P(alpha)=0` and `beta^2=alpha`, then `P(beta^2)=0`. The selected
//! source isolator maps monotonically onto one nonnegative square-root
//! interval; reflecting that interval selects the negative sheet. This keeps
//! the defining polynomial and branch interval exact without sampling a
//! primitive approximation.

use std::cmp::Ordering;

use hyperlimit::PredicatePolicy;
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    algebraic_root_interval_endpoints_are_roots, canonical_linear_value_representation,
    refine_reversed_algebraic_root_ownership, represented_root_sign_admitted,
    validate_algebraic_root_representation,
};
use crate::root_isolation::IsolatedRootInterval;

const SQRT_WITNESS_INTERVAL_PRECISION: i32 = -128;

/// Status for constructing one signed square-root algebraic image.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootSquareRootStatus {
    /// The signed square root has exact polynomial and isolating evidence.
    Transformed,
    /// The source representation was invalid.
    InvalidEvidence,
    /// The requested branch was outside `-1..=1`.
    InvalidBranch,
    /// The represented radicand was certified negative.
    NegativeRadicand,
    /// Branch zero was requested for a certified nonzero radicand.
    NonzeroZeroBranch,
    /// Exact sign classification did not decide.
    UndecidedSign,
    /// The constructed representation failed exact validation.
    InvalidTransformedEvidence,
}

/// Exact construction report for `branch*sqrt(alpha)`.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootSquareRootReport {
    /// Requested signed radical branch.
    pub branch: i8,
    /// Final construction status.
    pub status: AlgebraicRootSquareRootStatus,
    /// Resulting represented root when construction succeeded or retained
    /// failed evidence is useful to inspect.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Constructs the exact represented value `branch*sqrt(alpha)`.
///
/// `branch` is `-1`, `0`, or `1`. A zero radicand canonicalizes every branch
/// to the exact zero root. Nonzero branch zero is rejected rather than used as
/// approximate construction evidence. Every construction predicate is
/// evaluated under `STRICT`; approximate equality is a terminal decision
/// policy and never selects a square-root image.
pub fn square_root_algebraic_root_representation(
    root: &AlgebraicRootRepresentation,
    branch: i8,
) -> AlgebraicRootSquareRootReport {
    if !root.is_valid()
        || validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
        return report(
            branch,
            AlgebraicRootSquareRootStatus::InvalidEvidence,
            None,
            "square-root construction requires valid represented evidence",
        );
    }
    if !(-1..=1).contains(&branch) {
        return report(
            branch,
            AlgebraicRootSquareRootStatus::InvalidBranch,
            None,
            "a signed algebraic square-root branch must be -1, 0, or 1",
        );
    }
    let Some(sign) = represented_root_sign_admitted(root, PredicatePolicy::STRICT) else {
        return report(
            branch,
            AlgebraicRootSquareRootStatus::UndecidedSign,
            None,
            "the represented radicand sign was not certified",
        );
    };
    match sign {
        Ordering::Less => {
            return report(
                branch,
                AlgebraicRootSquareRootStatus::NegativeRadicand,
                None,
                "a real algebraic square root requires a nonnegative radicand",
            );
        }
        Ordering::Equal => {
            return transformed_zero(root, branch);
        }
        Ordering::Greater if branch == 0 => {
            return report(
                branch,
                AlgebraicRootSquareRootStatus::NonzeroZeroBranch,
                None,
                "branch zero is valid only for an exactly zero radicand",
            );
        }
        Ordering::Greater => {}
    }

    // Isolators own (lower, upper]. A decreasing map may exchange a selected
    // upper endpoint root with an excluded lower endpoint root. Resolve that
    // ownership in the source before transporting its singleton certificate.
    if branch < 0 && root.exact_point_witness().is_none() {
        match algebraic_root_interval_endpoints_are_roots(root, PredicatePolicy::STRICT) {
            Some(false) => {}
            Some(true) => {
                if let Some(refined) =
                    refine_reversed_algebraic_root_ownership(root, PredicatePolicy::STRICT)
                {
                    return square_root_algebraic_root_representation(&refined, branch);
                }
                return report(
                    branch,
                    AlgebraicRootSquareRootStatus::UndecidedSign,
                    None,
                    "could not refine source endpoint ownership for the negative square root",
                );
            }
            None => {
                return report(
                    branch,
                    AlgebraicRootSquareRootStatus::UndecidedSign,
                    None,
                    "could not decide source endpoint ownership for the negative square root",
                );
            }
        }
    }
    let Some(interval) = signed_sqrt_interval_for_positive_source(&root.interval, branch) else {
        return report(
            branch,
            AlgebraicRootSquareRootStatus::UndecidedSign,
            None,
            "could not construct exact bounds for the square-root image",
        );
    };
    if let Some(image_root) = &interval.exact_root {
        return transformed_exact_root(root, branch, image_root.clone());
    }
    let polynomial_coefficients = if let Some(source_root) = root.exact_point_witness() {
        vec![-source_root.clone(), Real::zero(), Real::one()]
    } else {
        let Some(polynomial) = compose_polynomial_at_square(&root.polynomial_coefficients) else {
            return report(
                branch,
                AlgebraicRootSquareRootStatus::InvalidTransformedEvidence,
                None,
                "square-root image polynomial degree exceeds addressable storage",
            );
        };
        polynomial
    };
    // Square root is injective on the positive half-axis. Exact mapped bounds
    // preserve the source singleton, so P(y²) needs no new Sturm chain. An
    // exact source witness instead gives y²-r, which has one root per signed
    // half-axis for every positive exact Real r.
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
    representation.validation =
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT);
    if !representation.is_valid() {
        return AlgebraicRootSquareRootReport {
            branch,
            status: AlgebraicRootSquareRootStatus::InvalidTransformedEvidence,
            representation: Some(representation),
            message: Some("square-root image evidence did not validate".to_owned()),
        };
    }
    AlgebraicRootSquareRootReport {
        branch,
        status: AlgebraicRootSquareRootStatus::Transformed,
        representation: Some(representation),
        message: None,
    }
}

fn compose_polynomial_at_square(polynomial: &[Real]) -> Option<Vec<Real>> {
    let output_len = polynomial.len().checked_mul(2)?.checked_sub(1)?;
    let mut composed = Vec::with_capacity(output_len);
    for (power, coefficient) in polynomial.iter().enumerate() {
        if power != 0 {
            composed.push(Real::zero());
        }
        composed.push(coefficient.clone());
    }
    Some(composed)
}

/// Maps an interval whose selected root was already proved positive.
fn signed_sqrt_interval_for_positive_source(
    source: &IsolatedRootInterval,
    branch: i8,
) -> Option<IsolatedRootInterval> {
    if let Some(source_root) = &source.exact_root {
        let positive = source_root.clone().sqrt().ok()?;
        if let Some(rational) = positive.exact_rational_normal_form() {
            let positive = Real::new(rational);
            let root = if branch < 0 { -positive } else { positive };
            return Some(IsolatedRootInterval {
                lower: root.clone(),
                upper: root.clone(),
                exact_root: Some(root),
                distinct_root_count: 1,
            });
        }
        let [lower, upper] = positive.certified_dyadic_interval(SQRT_WITNESS_INTERVAL_PRECISION)?;
        // Keep the root strictly inside the dyadic bracket even when a Real
        // witness has a rational value that its normal form did not expose.
        let padding = hyperreal::Rational::from(2_i8)
            .powi(SQRT_WITNESS_INTERVAL_PRECISION.into())
            .ok()?;
        return Some(signed_sqrt_interval_from_positive_bounds(
            nonnegative_dyadic_bound(lower - padding.clone()),
            Real::new(upper + padding),
            branch,
        ));
    }
    // Without an exact witness, `represented_root_sign == Greater` proves
    // this lower endpoint is nonnegative, including the zero-touching case.
    Some(signed_sqrt_interval_from_positive_bounds(
        source.lower.clone().sqrt().ok()?,
        source.upper.clone().sqrt().ok()?,
        branch,
    ))
}

fn nonnegative_dyadic_bound(bound: hyperreal::Rational) -> Real {
    if bound.is_negative() {
        Real::zero()
    } else {
        Real::new(bound)
    }
}

fn signed_sqrt_interval_from_positive_bounds(
    positive_lower: Real,
    positive_upper: Real,
    branch: i8,
) -> IsolatedRootInterval {
    let (lower, upper) = if branch < 0 {
        (-positive_upper, -positive_lower)
    } else {
        (positive_lower, positive_upper)
    };
    IsolatedRootInterval {
        lower,
        upper,
        exact_root: None,
        distinct_root_count: 1,
    }
}

fn transformed_zero(
    source: &AlgebraicRootRepresentation,
    branch: i8,
) -> AlgebraicRootSquareRootReport {
    transformed_exact_root(source, branch, Real::zero())
}

fn transformed_exact_root(
    source: &AlgebraicRootRepresentation,
    branch: i8,
    root: Real,
) -> AlgebraicRootSquareRootReport {
    let representation = canonical_linear_value_representation(source, root);
    debug_assert_eq!(
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid,
        "canonical exact square-root output must validate",
    );
    AlgebraicRootSquareRootReport {
        branch,
        status: AlgebraicRootSquareRootStatus::Transformed,
        representation: Some(representation),
        message: None,
    }
}

fn report(
    branch: i8,
    status: AlgebraicRootSquareRootStatus,
    representation: Option<AlgebraicRootRepresentation>,
    message: impl Into<String>,
) -> AlgebraicRootSquareRootReport {
    AlgebraicRootSquareRootReport {
        branch,
        status,
        representation,
        message: Some(message.into()),
    }
}

#[cfg(test)]
mod tests {
    use hyperlimit::compare_reals;
    use hyperreal::Rational;
    use num::bigint::{BigInt, BigUint};
    use proptest::prelude::*;

    use super::*;
    use crate::algebraic::represented_root_sign;
    use crate::root_isolation::{
        IsolatedRootRefinementStatus, RootIsolationConfig,
        refine_isolated_univariate_polynomial_interval,
    };
    use crate::symbolic::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn ratio_big(numerator: BigUint, denominator: &BigUint) -> Real {
        Real::new(
            Rational::from_bigint_fraction(BigInt::from(numerator), denominator.clone()).unwrap(),
        )
    }

    fn dyadic(exponent: usize) -> Real {
        ratio_big(BigUint::from(1_u8), &(BigUint::from(1_u8) << exponent))
    }

    fn positive_sqrt_two() -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
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

    fn exact_witness(value: i64) -> AlgebraicRootRepresentation {
        let value = real(value);
        AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-value.clone(), Real::one()],
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

    fn source_above_a_close_neighbor(separation_exponent: usize) -> AlgebraicRootRepresentation {
        let unit = dyadic(separation_exponent);
        let first = real(2) - unit.clone();
        let selected = real(2) + unit;
        let mut source = positive_sqrt_two();
        source.polynomial_coefficients = vec![
            first.clone() * selected.clone(),
            -(first + selected),
            Real::one(),
        ];
        source.interval = IsolatedRootInterval {
            lower: real(2),
            upper: (real(5) / real(2)).unwrap(),
            exact_root: None,
            distinct_root_count: 1,
        };
        source
    }

    #[test]
    fn signed_square_root_images_keep_exact_branch_isolators() {
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&positive_sqrt_two(), branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            let image = report.representation.unwrap();
            assert_eq!(
                image.polynomial_coefficients,
                vec![
                    real(-2),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::one()
                ],
            );
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                assert_eq!(
                    represented_root_sign(&image, policy),
                    Some(if branch < 0 {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    }),
                );
            }
        }
    }

    #[test]
    fn square_root_rejects_negative_and_nonzero_zero_branches() {
        let mut negative = positive_sqrt_two();
        negative.interval = IsolatedRootInterval {
            lower: real(-2),
            upper: real(-1),
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            square_root_algebraic_root_representation(&negative, 1).status,
            AlgebraicRootSquareRootStatus::NegativeRadicand,
        );
        assert_eq!(
            square_root_algebraic_root_representation(&positive_sqrt_two(), 0).status,
            AlgebraicRootSquareRootStatus::NonzeroZeroBranch,
        );
        assert_eq!(
            square_root_algebraic_root_representation(&positive_sqrt_two(), 2).status,
            AlgebraicRootSquareRootStatus::InvalidBranch,
        );
    }

    #[test]
    fn square_root_replays_cached_source_validation_strictly() {
        let mut stale = positive_sqrt_two();
        stale.interval.lower = real(2);
        stale.interval.upper = real(1);
        assert!(stale.is_valid());

        let report = square_root_algebraic_root_representation(&stale, 1);
        assert_eq!(
            report.status,
            AlgebraicRootSquareRootStatus::InvalidEvidence
        );
        assert!(report.representation.is_none());
    }

    #[test]
    fn square_root_transports_the_selected_source_isolator() {
        let source = source_above_a_close_neighbor(200);
        assert!(
            refine_isolated_univariate_polynomial_interval(
                &source.polynomial_coefficients,
                &source.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .refined_interval
            .is_some()
        );
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            assert!(report.representation.is_some());
        }
    }

    #[test]
    fn square_root_exact_bounds_do_not_absorb_arbitrarily_close_foreign_roots() {
        let source = source_above_a_close_neighbor(4200);
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            let image = report.representation.unwrap();
            // Squaring the mapped bounds recovers the original separating
            // boundary exactly; outward rounding would absorb the root below 2.
            let (lower, upper) = if branch < 0 {
                (&image.interval.upper, &image.interval.lower)
            } else {
                (&image.interval.lower, &image.interval.upper)
            };
            assert_eq!(
                compare_reals(
                    &(lower * lower),
                    &source.interval.lower,
                    PredicatePolicy::STRICT
                )
                .value(),
                Some(Ordering::Equal),
            );
            assert_eq!(
                compare_reals(
                    &(upper * upper),
                    &source.interval.upper,
                    PredicatePolicy::STRICT
                )
                .value(),
                Some(Ordering::Equal),
            );
        }
    }

    #[test]
    fn negative_square_root_preserves_excluded_and_selected_endpoint_ownership() {
        // (x-1)(x-4), selecting the owned upper endpoint 4 and excluding 1.
        let mut source = positive_sqrt_two();
        source.polynomial_coefficients = vec![real(4), real(-5), Real::one()];
        source.interval.upper = real(4);
        let image = square_root_algebraic_root_representation(&source, -1);
        assert_eq!(image.status, AlgebraicRootSquareRootStatus::Transformed);
        assert_eq!(
            image.representation.unwrap().exact_point_witness(),
            Some(&real(-2))
        );

        // (x-1)(x²-2), with an excluded lower root and a nonendpoint root.
        source.polynomial_coefficients = vec![real(2), real(-2), real(-1), Real::one()];
        source.interval.upper = real(2);
        let image = square_root_algebraic_root_representation(&source, -1);
        assert_eq!(image.status, AlgebraicRootSquareRootStatus::Transformed);
        let image = image.representation.unwrap();
        assert_eq!(
            compare_reals(&image.interval.upper, &real(-1), PredicatePolicy::STRICT).value(),
            Some(Ordering::Less),
            "the excluded source root must not become an owned image endpoint",
        );
        assert!(
            refine_isolated_univariate_polynomial_interval(
                &image.polynomial_coefficients,
                &image.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .refined_interval
            .is_some()
        );
    }

    #[test]
    fn square_root_of_an_arbitrary_exact_witness_keeps_its_quadratic_relation() {
        let value = real(2).sqrt().unwrap();
        let source = canonical_linear_value_representation(&positive_sqrt_two(), value.clone());
        for branch in [-1, 1] {
            let image = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(image.status, AlgebraicRootSquareRootStatus::Transformed);
            let image = image.representation.unwrap();
            assert_eq!(
                image.polynomial_coefficients,
                vec![-value.clone(), Real::zero(), Real::one()]
            );
            assert!(
                refine_isolated_univariate_polynomial_interval(
                    &image.polynomial_coefficients,
                    &image.interval,
                    RootIsolationConfig {
                        policy: PredicatePolicy::STRICT,
                        max_interval_width: None,
                        max_refinement_steps: 0,
                    },
                )
                .refined_interval
                .is_some()
            );
        }
    }

    #[test]
    fn nonzero_exact_witness_keeps_an_exact_square_root_witness() {
        let source = exact_witness(4);
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            assert_eq!(
                report
                    .representation
                    .as_ref()
                    .unwrap()
                    .exact_point_witness(),
                Some(&real(2 * i64::from(branch))),
            );
            assert_eq!(
                report.representation.unwrap().polynomial_coefficients,
                vec![real(-2 * i64::from(branch)), Real::one()],
            );
        }
    }

    #[test]
    fn irrational_square_root_of_an_exact_witness_uses_an_isolator() {
        let mut source = exact_witness(2);
        source.polynomial_coefficients = vec![
            real(64),
            real(-192),
            real(240),
            real(-160),
            real(60),
            real(-12),
            Real::one(),
        ];
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            let image = report.representation.unwrap();
            assert!(image.exact_point_witness().is_none());
            assert_eq!(
                image.polynomial_coefficients,
                vec![real(-2), Real::zero(), Real::one()],
            );
            assert_eq!(
                represented_root_sign(&image, PredicatePolicy::STRICT),
                Some(if branch < 0 {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }),
            );
        }
    }

    #[test]
    fn every_zero_square_root_branch_canonicalizes_exactly() {
        let mut zero = positive_sqrt_two();
        zero.polynomial_coefficients = vec![Real::zero(), Real::one()];
        zero.interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::zero(),
            exact_root: Some(Real::zero()),
            distinct_root_count: 1,
        };
        for branch in -1..=1 {
            let report = square_root_algebraic_root_representation(&zero, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            let representation = report.representation.unwrap();
            assert_eq!(representation.exact_point_witness(), Some(&Real::zero()),);
            assert_eq!(
                validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT,)
                    .status,
                AlgebraicRootValidationStatus::Valid,
            );
        }
    }

    proptest! {
        #[test]
        fn square_composition_places_every_source_coefficient_exactly(
            coefficients in prop::collection::vec(-32_i64..33, 2..20),
        ) {
            let polynomial = coefficients.iter().copied().map(real).collect::<Vec<_>>();
            let composed = compose_polynomial_at_square(&polynomial).unwrap();
            prop_assert_eq!(composed.len(), polynomial.len() * 2 - 1);
            for (power, coefficient) in composed.iter().enumerate() {
                if power % 2 == 0 {
                    prop_assert_eq!(coefficient, &polynomial[power / 2]);
                } else {
                    prop_assert_eq!(coefficient, &Real::zero());
                }
            }
        }

        #[test]
        fn exact_rational_source_shortcut_retains_an_independent_unit_isolator(
            numerator in 1_i64..64,
            denominator in 1_i64..32,
        ) {
            let source_root = (real(numerator) / real(denominator)).unwrap();
            let source = AlgebraicRootRepresentation {
                constraint_index: 7,
                symbol: SymbolId(3),
                interval_index: 2,
                polynomial_coefficients: vec![-source_root.clone(), Real::one()],
                interval: IsolatedRootInterval {
                    lower: source_root.clone(),
                    upper: source_root.clone(),
                    exact_root: Some(source_root.clone()),
                    distinct_root_count: 1,
                },
                validation: AlgebraicRootValidationReport {
                    status: AlgebraicRootValidationStatus::Valid,
                    message: None,
                },
            };
            let positive = source_root.sqrt().unwrap();

            for branch in [-1, 1] {
                let report = square_root_algebraic_root_representation(&source, branch);
                prop_assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
                let image = report.representation.unwrap();
                prop_assert_eq!(image.constraint_index, source.constraint_index);
                prop_assert_eq!(image.symbol, source.symbol);
                prop_assert_eq!(image.interval_index, source.interval_index);
                let expected = if branch < 0 {
                    -positive.clone()
                } else {
                    positive.clone()
                };
                let value = image
                    .polynomial_coefficients
                    .iter()
                    .rev()
                    .fold(Real::zero(), |value, coefficient| {
                        value * expected.clone() + coefficient.clone()
                    });
                prop_assert_eq!(
                    compare_reals(&value, &Real::zero(), PredicatePolicy::STRICT).value(),
                    Some(Ordering::Equal),
                );
                prop_assert_ne!(
                    compare_reals(&image.interval.lower, &expected, PredicatePolicy::STRICT)
                        .value(),
                    Some(Ordering::Greater),
                );
                prop_assert_ne!(
                    compare_reals(&image.interval.upper, &expected, PredicatePolicy::STRICT)
                        .value(),
                    Some(Ordering::Less),
                );
                prop_assert_eq!(
                    validate_algebraic_root_representation(&image, PredicatePolicy::STRICT).status,
                    AlgebraicRootValidationStatus::Valid,
                );
                let replay = refine_isolated_univariate_polynomial_interval(
                    &image.polynomial_coefficients,
                    &image.interval,
                    RootIsolationConfig {
                        policy: PredicatePolicy::STRICT,
                        max_interval_width: None,
                        max_refinement_steps: 0,
                    },
                );
                prop_assert!(replay.refined_interval.is_some());
                prop_assert!(matches!(
                    replay.status,
                    IsolatedRootRefinementStatus::Refined
                        | IsolatedRootRefinementStatus::ExactRoot
                ));
            }
        }
    }
}
