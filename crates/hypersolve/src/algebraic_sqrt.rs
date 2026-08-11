//! Exact signed square-root images of represented real algebraic roots.
//!
//! If `P(alpha)=0` and `beta^2=alpha`, then `P(beta^2)=0`. The selected
//! source isolator maps monotonically onto one nonnegative square-root
//! interval; reflecting that interval selects the negative sheet. This keeps
//! the defining polynomial and branch interval exact without sampling a
//! primitive approximation.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, represented_root_sign, validate_algebraic_root_representation,
};
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval,
};

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
    /// The mapped interval did not certify one distinct image root.
    NonIsolatingImageInterval,
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
    if !root.is_valid() {
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
    let Some(sign) = represented_root_sign(root, PredicatePolicy::STRICT) else {
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

    let mut polynomial_coefficients =
        vec![Real::zero(); root.polynomial_coefficients.len() * 2 - 1];
    for (power, coefficient) in root.polynomial_coefficients.iter().enumerate() {
        polynomial_coefficients[power * 2] = coefficient.clone();
    }
    let Some(interval) = signed_sqrt_interval(&root.interval, branch) else {
        return report(
            branch,
            AlgebraicRootSquareRootStatus::UndecidedSign,
            None,
            "could not construct exact dyadic bounds for the square-root image",
        );
    };
    let refinement = refine_isolated_univariate_polynomial_interval(
        &polynomial_coefficients,
        &interval,
        RootIsolationConfig {
            policy: PredicatePolicy::STRICT,
            max_interval_width: None,
            max_refinement_steps: 2,
        },
    );
    let Some(interval) = refinement.refined_interval else {
        let status = match refinement.status {
            IsolatedRootRefinementStatus::InvalidPolynomial
            | IsolatedRootRefinementStatus::InvalidInterval => {
                AlgebraicRootSquareRootStatus::InvalidTransformedEvidence
            }
            IsolatedRootRefinementStatus::NonUnitIsolation => {
                AlgebraicRootSquareRootStatus::NonIsolatingImageInterval
            }
            IsolatedRootRefinementStatus::Undecided
            | IsolatedRootRefinementStatus::Refined
            | IsolatedRootRefinementStatus::ExactRoot => {
                AlgebraicRootSquareRootStatus::UndecidedSign
            }
        };
        return AlgebraicRootSquareRootReport {
            branch,
            status,
            representation: None,
            message: refinement.message,
        };
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

fn signed_sqrt_interval(source: &IsolatedRootInterval, branch: i8) -> Option<IsolatedRootInterval> {
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
    }
    let zero = Real::zero();
    let lower = match compare_reals(&source.lower, &zero, PredicatePolicy::STRICT).value()? {
        Ordering::Less => zero,
        Ordering::Equal | Ordering::Greater => source.lower.clone(),
    };
    let lower = lower.sqrt().ok()?.certified_dyadic_interval(-128)?;
    let upper = source
        .upper
        .clone()
        .sqrt()
        .ok()?
        .certified_dyadic_interval(-128)?;
    let positive_lower = Real::new(lower[0].clone());
    let positive_upper = Real::new(upper[1].clone());
    let (lower, upper) = if branch < 0 {
        (-positive_upper, -positive_lower)
    } else {
        (positive_lower, positive_upper)
    };
    Some(IsolatedRootInterval {
        lower,
        upper,
        exact_root: None,
        distinct_root_count: 1,
    })
}

fn transformed_zero(
    source: &AlgebraicRootRepresentation,
    branch: i8,
) -> AlgebraicRootSquareRootReport {
    let zero = Real::zero();
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: source.constraint_index,
        symbol: source.symbol,
        interval_index: source.interval_index,
        polynomial_coefficients: vec![zero.clone(), Real::one()],
        interval: IsolatedRootInterval {
            lower: zero.clone(),
            upper: zero.clone(),
            exact_root: Some(zero),
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation =
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT);
    AlgebraicRootSquareRootReport {
        branch,
        status: if representation.is_valid() {
            AlgebraicRootSquareRootStatus::Transformed
        } else {
            AlgebraicRootSquareRootStatus::InvalidTransformedEvidence
        },
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
    use super::*;
    use crate::symbolic::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
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
            kind: AlgebraicRootKind::IsolatingInterval,
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
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
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
    }

    #[test]
    fn nonzero_exact_witness_keeps_an_exact_square_root_witness() {
        let source = exact_witness(4);
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            assert_eq!(
                report.representation.unwrap().exact_rational_witness(),
                Some(&real(2 * i64::from(branch))),
            );
        }
    }

    #[test]
    fn irrational_square_root_of_an_exact_witness_uses_an_isolator() {
        let source = exact_witness(2);
        for branch in [-1, 1] {
            let report = square_root_algebraic_root_representation(&source, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            assert!(
                report
                    .representation
                    .unwrap()
                    .exact_rational_witness()
                    .is_none()
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
        zero.kind = AlgebraicRootKind::ExactRationalWitness;
        for branch in -1..=1 {
            let report = square_root_algebraic_root_representation(&zero, branch);
            assert_eq!(report.status, AlgebraicRootSquareRootStatus::Transformed);
            assert_eq!(
                report.representation.unwrap().exact_rational_witness(),
                Some(&Real::zero()),
            );
        }
    }
}
