//! Rational-function images of represented algebraic roots.
//!
//! This module constructs exact evidence for `beta = p(alpha) / q(alpha)`,
//! where `alpha` is a represented real algebraic root and `p`, `q` are
//! exact-rational polynomials.  The result is another
//! [`crate::AlgebraicRootRepresentation`], not a primitive approximation.
//! That distinction is the EGC boundary from Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): construction retains
//! exact replayable objects, and later predicates decide signs/topology from
//! certificates.
//!
//! The implementation intentionally composes the already-certified pieces:
//! denominator domain checking uses interval polynomial evaluation, numerator
//! and denominator values are represented by resultant-backed polynomial
//! images (Sylvester, "On a Theory of the Syzygetic Relations..." (1853)),
//! and the quotient is represented by the bounded binary resultant package
//! using Sturm/Collins-Loos isolation evidence.

use hyperlimit::PredicatePolicy;
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport, AlgebraicRootArithmeticStatus,
    AlgebraicRootKind, AlgebraicRootRationalEvaluationReport,
    AlgebraicRootRationalEvaluationStatus, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    arithmetic_algebraic_root_representations, evaluate_rational_expression_at_algebraic_root,
    validate_algebraic_root_representation,
};
use crate::algebraic_polynomial_image::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    transform_algebraic_root_polynomial_image,
};
use crate::root_isolation::IsolatedRootInterval;

/// Status for constructing a rational-function image of a represented root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootRationalImageStatus {
    /// The rational image was represented exactly.
    Transformed,
    /// The input represented root failed structural validation.
    InvalidEvidence,
    /// The numerator polynomial is empty or unsupported.
    InvalidNumeratorPolynomial,
    /// The denominator polynomial is empty or unsupported.
    InvalidDenominatorPolynomial,
    /// The denominator is exactly zero at a rational witness.
    CertifiedZeroDenominator,
    /// The denominator interval may contain zero, so the rational map is not
    /// certified on the source isolating interval.
    DenominatorMayContainZero,
    /// The numerator value could not be represented by the bounded polynomial
    /// image package.
    NumeratorImageFailed,
    /// The denominator value could not be represented by the bounded
    /// polynomial image package.
    DenominatorImageFailed,
    /// The quotient image could not be represented by the bounded arithmetic
    /// package.
    QuotientConstructionFailed,
    /// The transformed evidence failed structural validation.
    InvalidTransformedEvidence,
    /// Exact comparison or scalar arithmetic did not decide.
    Undecided,
}

/// Report for `beta = p(alpha) / q(alpha)`.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicRootRationalImageReport {
    /// Final rational-image construction status.
    pub status: AlgebraicRootRationalImageStatus,
    /// Exact numerator coefficients in ascending powers of `alpha`.
    pub numerator_coefficients: Vec<Real>,
    /// Exact denominator coefficients in ascending powers of `alpha`.
    pub denominator_coefficients: Vec<Real>,
    /// Domain-checking evidence for the rational expression.
    pub evaluation: AlgebraicRootRationalEvaluationReport,
    /// Exact representation of `p(alpha)` when construction reached it.
    pub numerator_image: Option<AlgebraicRootPolynomialImageReport>,
    /// Exact representation of `q(alpha)` when construction reached it.
    pub denominator_image: Option<AlgebraicRootPolynomialImageReport>,
    /// Exact quotient construction report when construction reached it.
    pub quotient: Option<AlgebraicRootArithmeticReport>,
    /// Resulting represented root when construction succeeds or failed
    /// validation has useful evidence to inspect.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Construct exact algebraic evidence for `p(alpha) / q(alpha)`.
///
/// Coefficients are supplied in ascending power order.  The denominator is
/// certified away from zero before quotient construction, including the exact
/// rational-witness case.  Nonlinear images are deliberately bounded by the
/// existing polynomial-image and binary-resultant packages; unsupported degree
/// or non-monotone image intervals remain explicit report statuses instead of
/// falling back to sampled arithmetic.
pub fn transform_algebraic_root_rational_image(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootRationalImageReport {
    let evaluation = evaluate_rational_expression_at_algebraic_root(
        root,
        numerator_coefficients,
        denominator_coefficients,
        policy,
    );
    match evaluation.status {
        AlgebraicRootRationalEvaluationStatus::InvalidEvidence => {
            return rational_image_report(
                AlgebraicRootRationalImageStatus::InvalidEvidence,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some("algebraic root representation must be valid before transformation".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::InvalidPolynomial => {
            let status = if matches!(
                evaluation.numerator.status,
                crate::AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial
            ) {
                AlgebraicRootRationalImageStatus::InvalidNumeratorPolynomial
            } else {
                AlgebraicRootRationalImageStatus::InvalidDenominatorPolynomial
            };
            return rational_image_report(
                status,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some(
                    "rational image requires supported numerator and denominator polynomials"
                        .into(),
                ),
            );
        }
        AlgebraicRootRationalEvaluationStatus::CertifiedZeroDenominator => {
            return rational_image_report(
                AlgebraicRootRationalImageStatus::CertifiedZeroDenominator,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some("denominator evaluates exactly to zero".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::DenominatorMayContainZero => {
            return rational_image_report(
                AlgebraicRootRationalImageStatus::DenominatorMayContainZero,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some("denominator is not certified away from zero".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::Undecided => {
            return rational_image_report(
                AlgebraicRootRationalImageStatus::Undecided,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some("rational expression domain check did not decide".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness => {
            if let Some(value) = evaluation.exact_value.clone() {
                let representation = exact_constant_representation(root, value, policy);
                let status = if representation.is_valid() {
                    AlgebraicRootRationalImageStatus::Transformed
                } else {
                    AlgebraicRootRationalImageStatus::InvalidTransformedEvidence
                };
                return rational_image_report(
                    status,
                    numerator_coefficients,
                    denominator_coefficients,
                    evaluation,
                    RationalImageArtifacts {
                        representation: Some(representation),
                        ..RationalImageArtifacts::default()
                    },
                    None,
                );
            }
            return rational_image_report(
                AlgebraicRootRationalImageStatus::Undecided,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts::default(),
                Some("exact rational evaluation did not carry an exact value".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::IntervalEvaluated => {}
    }

    let numerator_image =
        transform_algebraic_root_polynomial_image(root, numerator_coefficients, policy);
    if numerator_image.status != AlgebraicRootPolynomialImageStatus::Transformed {
        let message = numerator_image.message.clone();
        return rational_image_report(
            polynomial_image_failure_status(
                numerator_image.status.clone(),
                AlgebraicRootRationalImageStatus::NumeratorImageFailed,
            ),
            numerator_coefficients,
            denominator_coefficients,
            evaluation,
            RationalImageArtifacts {
                numerator_image: Some(numerator_image),
                ..RationalImageArtifacts::default()
            },
            message,
        );
    }

    let denominator_image =
        transform_algebraic_root_polynomial_image(root, denominator_coefficients, policy);
    if denominator_image.status != AlgebraicRootPolynomialImageStatus::Transformed {
        let message = denominator_image.message.clone();
        return rational_image_report(
            polynomial_image_failure_status(
                denominator_image.status.clone(),
                AlgebraicRootRationalImageStatus::DenominatorImageFailed,
            ),
            numerator_coefficients,
            denominator_coefficients,
            evaluation,
            RationalImageArtifacts {
                numerator_image: Some(numerator_image),
                denominator_image: Some(denominator_image),
                ..RationalImageArtifacts::default()
            },
            message,
        );
    }

    let numerator = numerator_image
        .representation
        .as_ref()
        .expect("transformed polynomial image carries representation");
    let denominator = denominator_image
        .representation
        .as_ref()
        .expect("transformed polynomial image carries representation");
    let quotient = arithmetic_algebraic_root_representations(
        numerator,
        Some(denominator),
        AlgebraicRootArithmeticOp::Divide,
    );
    match quotient.status {
        AlgebraicRootArithmeticStatus::ComputedRepresentation => {
            let representation = quotient.result_representation.clone();
            let status = if representation.as_ref().is_some_and(|root| root.is_valid()) {
                AlgebraicRootRationalImageStatus::Transformed
            } else {
                AlgebraicRootRationalImageStatus::InvalidTransformedEvidence
            };
            rational_image_report(
                status,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts {
                    numerator_image: Some(numerator_image),
                    denominator_image: Some(denominator_image),
                    quotient: Some(quotient),
                    representation,
                },
                None,
            )
        }
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness => {
            if let Some(value) = quotient.exact_result.clone() {
                let representation = exact_constant_representation(root, value, policy);
                let status = if representation.is_valid() {
                    AlgebraicRootRationalImageStatus::Transformed
                } else {
                    AlgebraicRootRationalImageStatus::InvalidTransformedEvidence
                };
                rational_image_report(
                    status,
                    numerator_coefficients,
                    denominator_coefficients,
                    evaluation,
                    RationalImageArtifacts {
                        numerator_image: Some(numerator_image),
                        denominator_image: Some(denominator_image),
                        quotient: Some(quotient),
                        representation: Some(representation),
                    },
                    None,
                )
            } else {
                rational_image_report(
                    AlgebraicRootRationalImageStatus::Undecided,
                    numerator_coefficients,
                    denominator_coefficients,
                    evaluation,
                    RationalImageArtifacts {
                        numerator_image: Some(numerator_image),
                        denominator_image: Some(denominator_image),
                        quotient: Some(quotient),
                        ..RationalImageArtifacts::default()
                    },
                    Some("exact quotient did not carry a rational witness".into()),
                )
            }
        }
        AlgebraicRootArithmeticStatus::InvalidEvidence => rational_image_report(
            AlgebraicRootRationalImageStatus::InvalidTransformedEvidence,
            numerator_coefficients,
            denominator_coefficients,
            evaluation,
            RationalImageArtifacts {
                numerator_image: Some(numerator_image),
                denominator_image: Some(denominator_image),
                quotient: Some(quotient),
                ..RationalImageArtifacts::default()
            },
            Some("quotient construction produced invalid evidence".into()),
        ),
        AlgebraicRootArithmeticStatus::NonRationalInput
        | AlgebraicRootArithmeticStatus::Undecided => {
            let message = quotient.message.clone();
            rational_image_report(
                AlgebraicRootRationalImageStatus::QuotientConstructionFailed,
                numerator_coefficients,
                denominator_coefficients,
                evaluation,
                RationalImageArtifacts {
                    numerator_image: Some(numerator_image),
                    denominator_image: Some(denominator_image),
                    quotient: Some(quotient),
                    ..RationalImageArtifacts::default()
                },
                message,
            )
        }
    }
}

fn polynomial_image_failure_status(
    status: AlgebraicRootPolynomialImageStatus,
    fallback: AlgebraicRootRationalImageStatus,
) -> AlgebraicRootRationalImageStatus {
    match status {
        AlgebraicRootPolynomialImageStatus::InvalidEvidence
        | AlgebraicRootPolynomialImageStatus::InvalidTransformedEvidence => {
            AlgebraicRootRationalImageStatus::InvalidTransformedEvidence
        }
        AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial => fallback,
        AlgebraicRootPolynomialImageStatus::NonMonotoneImage
        | AlgebraicRootPolynomialImageStatus::UnsupportedDegree
        | AlgebraicRootPolynomialImageStatus::Undecided => fallback,
        AlgebraicRootPolynomialImageStatus::Transformed => unreachable!("handled by caller"),
    }
}

fn exact_constant_representation(
    source: &AlgebraicRootRepresentation,
    value: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootRepresentation {
    let interval = IsolatedRootInterval {
        lower: value.clone(),
        upper: value.clone(),
        exact_root: Some(value.clone()),
        distinct_root_count: 1,
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: source.constraint_index,
        symbol: source.symbol,
        interval_index: source.interval_index,
        polynomial_coefficients: vec![-value, Real::one()],
        interval,
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    representation
}

#[derive(Default)]
struct RationalImageArtifacts {
    numerator_image: Option<AlgebraicRootPolynomialImageReport>,
    denominator_image: Option<AlgebraicRootPolynomialImageReport>,
    quotient: Option<AlgebraicRootArithmeticReport>,
    representation: Option<AlgebraicRootRepresentation>,
}

fn rational_image_report(
    status: AlgebraicRootRationalImageStatus,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    evaluation: AlgebraicRootRationalEvaluationReport,
    artifacts: RationalImageArtifacts,
    message: Option<String>,
) -> AlgebraicRootRationalImageReport {
    AlgebraicRootRationalImageReport {
        status,
        numerator_coefficients: numerator_coefficients.to_vec(),
        denominator_coefficients: denominator_coefficients.to_vec(),
        evaluation,
        numerator_image: artifacts.numerator_image,
        denominator_image: artifacts.denominator_image,
        quotient: artifacts.quotient,
        representation: artifacts.representation,
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

    fn sqrt_two_positive() -> AlgebraicRootRepresentation {
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

    #[test]
    fn rational_image_constructs_linear_fractional_value() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::zero(), Real::one()],
            &[Real::one(), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let root = report.representation.as_ref().unwrap();
        assert!(root.is_valid());
        assert_eq!(root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(root.interval.upper, (real(2) / real(3)).unwrap());
    }

    #[test]
    fn rational_image_preserves_rational_witnesses() {
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

        let report = transform_algebraic_root_rational_image(
            &rational,
            &[real(1), real(2), real(3)],
            &[real(1), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.exact_rational_witness(),
            Some(&(real(34) / real(4)).unwrap())
        );
        assert!(root.is_valid());
    }

    #[test]
    fn rational_image_refuses_uncertified_denominator_domain() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::DenominatorMayContainZero
        );
        assert!(report.representation.is_none());
        assert!(report.numerator_image.is_none());
    }

    #[test]
    fn rational_image_reports_nonlinear_denominator_image_failure() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one(), Real::one()],
            &[real(10), real(-3), Real::one()],
            PredicatePolicy::default(),
        );

        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::DenominatorImageFailed
        );
        assert!(report.denominator_image.is_some());
        assert!(report.representation.is_none());
    }

    proptest! {
        #[test]
        fn generated_rational_witness_rational_image_matches_exact_fraction(
            root in -10_i16..=10,
            numerator_constant in -8_i16..=8,
            numerator_linear in -8_i16..=8,
            denominator_constant in -8_i16..=8,
            denominator_linear in -8_i16..=8,
        ) {
            let root = i64::from(root);
            let nc = i64::from(numerator_constant);
            let nl = i64::from(numerator_linear);
            let dc = i64::from(denominator_constant);
            let dl = i64::from(denominator_linear);
            prop_assume!(dc + dl * root != 0);
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

            let report = transform_algebraic_root_rational_image(
                &represented,
                &[real(nc), real(nl)],
                &[real(dc), real(dl)],
                PredicatePolicy::default(),
            );

            prop_assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let expected = (real(nc + nl * root) / real(dc + dl * root)).unwrap();
            prop_assert_eq!(
                report.representation.as_ref().unwrap().exact_rational_witness(),
                Some(&expected)
            );
        }
    }
}
