//! Rational-function images of represented algebraic roots.
//!
//! This module constructs exact evidence for `beta = p(alpha) / q(alpha)`,
//! where `alpha` is a represented real algebraic root and `p`, `q` are exact
//! polynomials. General exact [`Real`] coefficients are supported for constant
//! and linear-fractional maps; nonlinear elimination remains exact-rational
//! and bounded. The result is another
//! [`crate::AlgebraicRootRepresentation`], not a primitive approximation.
//! That distinction is the EGC boundary from the exact-geometric-computation model: construction retains
//! exact replayable objects, and later predicates decide signs/topology from
//! certificates.
//!
//! After proving that the authored denominator does not vanish at the selected
//! root, the preferred path reduces both map polynomials modulo the defining
//! source relation and directly eliminates the source coordinate with
//! `Res_x(P(x), p(x) - y*q(x))`.  This is the rational analogue of the
//! polynomial-image construction and avoids requiring `p(alpha)` and
//! `q(alpha)` to be individually monotone.  If that direct path cannot certify
//! a one-root image interval, the implementation falls back to composing the
//! already-certified pieces: numerator and denominator polynomial images plus
//! the bounded binary quotient package.  Both paths keep domain uncertainty
//! report-bearing instead of replacing it with primitive sampling.

use std::cmp::Ordering;
use std::sync::OnceLock;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::{Rational, Real};

use crate::algebraic::{
    AlgebraicPolynomialValueInterval, AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport,
    AlgebraicRootArithmeticStatus, AlgebraicRootPolynomialEvaluationReport,
    AlgebraicRootRationalEvaluationReport, AlgebraicRootRationalEvaluationStatus,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    arithmetic_algebraic_root_representations, canonical_linear_value_representation,
    evaluate_rational_expression_at_algebraic_root,
    evaluate_rational_expression_with_denominator_evaluation,
    validate_algebraic_root_representation,
};
use crate::algebraic_mobius::{
    AlgebraicRootMobiusTransformStatus, transform_algebraic_root_mobius,
};
use crate::algebraic_polynomial_image::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    transform_algebraic_root_polynomial_image,
};
use crate::integer_interpolation::{
    interpolate_integer_samples_up_to_scale, primitive_integer_polynomial,
    primitive_integer_polynomial_gcd,
};
use crate::interval::rational_interval_product;
use crate::resultant::{quotient_ring_resultant_polynomial, resultant_univariate_polynomials};
use crate::root_isolation::{
    ALGEBRAIC_IMAGE_REFINEMENT_ROUNDS, ALGEBRAIC_IMAGE_REFINEMENT_STEPS, IsolatedRootInterval,
    IsolatedRootRefinementStatus, RootIsolationConfig, certify_algebraic_image_interval,
    polynomial_div_rem, polynomial_vanishes_at_owned_root,
    refine_isolated_univariate_polynomial_interval, square_free_part,
};

const MAX_RATIONAL_IMAGE_SYLVESTER_DIMENSION: usize = 16;

/// Status for constructing a rational-function image of a represented root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicRootRationalImageStatus {
    /// The rational image was represented exactly.
    Transformed,
    /// Evaluation certified that the image is disjoint from a requested
    /// closed target interval, so no representation was constructed.
    ImageIntervalDisjoint,
    /// The input represented root failed structural validation.
    InvalidEvidence,
    /// The numerator polynomial is empty or unsupported.
    InvalidNumeratorPolynomial,
    /// The denominator polynomial is empty or unsupported.
    InvalidDenominatorPolynomial,
    /// The authored denominator is exactly zero at the selected root.
    CertifiedZeroDenominator,
    /// Bounded exact refinement could not separate the denominator from zero
    /// on a source isolating interval.
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
    /// Domain-checking evidence for the rational expression, possibly over an
    /// exactly refined source interval.
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

struct RationalImageBatchContext<'a> {
    root: &'a AlgebraicRootRepresentation,
    denominator_coefficients: &'a [Real],
    denominator_evaluation: AlgebraicRootPolynomialEvaluationReport,
    strengthened_domain: OnceLock<RationalImageDomainEvidence>,
    source_polynomial: OnceLock<Option<Vec<Real>>>,
    policy: PredicatePolicy,
}

#[derive(Clone, Copy, Default)]
struct RationalImageRetention<'a> {
    direct_map: Option<&'a OnceLock<Option<DirectRationalMap>>>,
    source_polynomial: Option<&'a OnceLock<Option<Vec<Real>>>>,
    resultant_polynomial: Option<&'a OnceLock<Option<Vec<Real>>>>,
}

enum RationalImageDomainEvidence {
    Unchanged,
    CertifiedPole,
    Refined(Box<RefinedRationalImageDomain>),
}

struct RefinedRationalImageDomain {
    root: AlgebraicRootRepresentation,
    denominator_evaluation: AlgebraicRootPolynomialEvaluationReport,
}

struct DirectRationalMap {
    numerator: Vec<Real>,
    denominator: Vec<Real>,
    constant_value: Option<Real>,
    source_reduced_map: OnceLock<Option<Box<DirectRationalMap>>>,
    derivative_numerator: OnceLock<Option<Vec<Real>>>,
    cleared_coefficients: OnceLock<Option<(Vec<Real>, Vec<Real>)>>,
}

impl<'a> RationalImageBatchContext<'a> {
    fn new(
        root: &'a AlgebraicRootRepresentation,
        denominator_coefficients: &'a [Real],
        policy: PredicatePolicy,
    ) -> Self {
        Self {
            root,
            denominator_coefficients,
            denominator_evaluation: crate::algebraic::evaluate_polynomial_at_algebraic_root(
                root,
                denominator_coefficients,
                policy,
            ),
            strengthened_domain: OnceLock::new(),
            source_polynomial: OnceLock::new(),
            policy,
        }
    }

    fn transform(&self, numerator_coefficients: &[Real]) -> AlgebraicRootRationalImageReport {
        let mut root = self.root;
        let mut evaluation = evaluate_rational_expression_with_denominator_evaluation(
            self.root,
            numerator_coefficients,
            self.denominator_evaluation.clone(),
            self.policy,
        );
        if evaluation.status == AlgebraicRootRationalEvaluationStatus::DenominatorMayContainZero {
            match self.strengthened_domain.get_or_init(|| {
                strengthen_rational_image_denominator(
                    self.root,
                    self.denominator_coefficients,
                    self.policy,
                    &self.denominator_evaluation,
                )
            }) {
                RationalImageDomainEvidence::Unchanged => {}
                RationalImageDomainEvidence::CertifiedPole => {
                    mark_rational_image_evaluation_as_certified_pole(&mut evaluation);
                }
                RationalImageDomainEvidence::Refined(refined) => {
                    root = &refined.root;
                    evaluation = evaluate_rational_expression_with_denominator_evaluation(
                        &refined.root,
                        numerator_coefficients,
                        refined.denominator_evaluation.clone(),
                        self.policy,
                    );
                }
            }
        }
        transform_algebraic_root_rational_image_with_domain_evaluation(
            root,
            numerator_coefficients,
            self.denominator_coefficients,
            None,
            self.policy,
            evaluation,
            RationalImageRetention {
                source_polynomial: Some(&self.source_polynomial),
                ..RationalImageRetention::default()
            },
        )
    }
}

/// Transform several rational expressions with one shared denominator.
///
/// The denominator certificate, any selected-root pole proof and source
/// refinement, and the exact source polynomial are resolved once inside this
/// completed operation. Every returned element is the same exact report
/// produced by [`transform_algebraic_root_rational_image`] for the
/// corresponding numerator. A zero-length batch performs no evaluation.
pub fn transform_algebraic_root_rational_images<const N: usize>(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: [&[Real]; N],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
) -> [AlgebraicRootRationalImageReport; N] {
    if N == 0 {
        return numerator_coefficients.map(|_| unreachable!("an empty batch has no numerator"));
    }
    let context = RationalImageBatchContext::new(root, denominator_coefficients, policy);
    numerator_coefficients.map(|numerator| context.transform(numerator))
}

/// Reusable rational map for several roots of one source polynomial.
///
/// The exact resultant polynomial of a rational map depends on the source
/// polynomial and map coefficients, but not on which real root interval is
/// selected. This carrier retains source-modulo map reduction and any remaining
/// elimination lazily while every
/// [`Self::transform`] call still evaluates the denominator, constructs a
/// one-root image interval (refining stationary maps when necessary), and
/// validates the represented root independently.
pub struct AlgebraicRootRationalMap {
    source_polynomial_coefficients: Vec<Real>,
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    direct_map: OnceLock<Option<DirectRationalMap>>,
    source_polynomial: OnceLock<Option<Vec<Real>>>,
    resultant_polynomial: OnceLock<Option<Vec<Real>>>,
    policy: PredicatePolicy,
}

impl AlgebraicRootRationalMap {
    /// Constructs one rational map over a defining source polynomial.
    pub fn new(
        source_polynomial_coefficients: &[Real],
        numerator_coefficients: &[Real],
        denominator_coefficients: &[Real],
        policy: PredicatePolicy,
    ) -> Self {
        Self {
            source_polynomial_coefficients: source_polynomial_coefficients.to_vec(),
            numerator_coefficients: numerator_coefficients.to_vec(),
            denominator_coefficients: denominator_coefficients.to_vec(),
            direct_map: OnceLock::new(),
            source_polynomial: OnceLock::new(),
            resultant_polynomial: OnceLock::new(),
            policy,
        }
    }

    /// Constructs exact image evidence for one represented source root.
    pub fn transform(
        &self,
        root: &AlgebraicRootRepresentation,
    ) -> AlgebraicRootRationalImageReport {
        self.transform_with_target(root, None)
    }

    /// Constructs an image only when it may meet a closed target interval.
    pub fn transform_in_interval(
        &self,
        root: &AlgebraicRootRepresentation,
        target: &AlgebraicPolynomialValueInterval,
    ) -> AlgebraicRootRationalImageReport {
        self.transform_with_target(root, Some(target))
    }

    fn transform_with_target(
        &self,
        root: &AlgebraicRootRepresentation,
        target: Option<&AlgebraicPolynomialValueInterval>,
    ) -> AlgebraicRootRationalImageReport {
        if root.polynomial_coefficients != self.source_polynomial_coefficients {
            return transform_algebraic_root_rational_image_with_target(
                root,
                &self.numerator_coefficients,
                &self.denominator_coefficients,
                target,
                self.policy,
            );
        }
        let evaluation = evaluate_rational_expression_at_algebraic_root(
            root,
            &self.numerator_coefficients,
            &self.denominator_coefficients,
            self.policy,
        );
        transform_algebraic_root_rational_image_with_evaluation(
            root,
            &self.numerator_coefficients,
            &self.denominator_coefficients,
            target,
            self.policy,
            evaluation,
            RationalImageRetention {
                direct_map: Some(&self.direct_map),
                source_polynomial: Some(&self.source_polynomial),
                resultant_polynomial: Some(&self.resultant_polynomial),
            },
        )
    }
}

/// Construct exact algebraic evidence for `p(alpha) / q(alpha)`.
///
/// Coefficients are supplied in ascending power order. The authored denominator
/// is certified nonzero at the selected root before cancellation or
/// source-modulo reduction. A coarse interval containing zero triggers exact
/// GCD selection and bounded source refinement; a true algebraic pole is
/// reported distinctly. Constant and linear-fractional maps may use general
/// exact [`Real`] coefficients. Nonlinear images require exact-rational
/// coefficients and remain bounded by the existing resultant packages.
/// Stationary maps use exact interval evaluation and Sturm-certified source
/// refinement; unsupported degree or exhausted refinement remains explicit
/// instead of falling back to sampled arithmetic. Decreasing maps refine away
/// endpoint roots when needed to preserve `(lower, upper]` ownership.
pub fn transform_algebraic_root_rational_image(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
) -> AlgebraicRootRationalImageReport {
    transform_algebraic_root_rational_image_with_target(
        root,
        numerator_coefficients,
        denominator_coefficients,
        None,
        policy,
    )
}

/// Construct exact algebraic evidence for `p(alpha) / q(alpha)` only when its
/// evaluation may intersect a closed target interval.
///
/// A conservative rational evaluation enclosure is formed before elimination.
/// When that enclosure is provably disjoint from `target`, the report returns
/// [`AlgebraicRootRationalImageStatus::ImageIntervalDisjoint`] without building
/// a resultant. Inconclusive or boundary-touching enclosures continue through
/// the ordinary exact construction.
pub fn transform_algebraic_root_rational_image_in_interval(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    target: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> AlgebraicRootRationalImageReport {
    transform_algebraic_root_rational_image_with_target(
        root,
        numerator_coefficients,
        denominator_coefficients,
        Some(target),
        policy,
    )
}

fn transform_algebraic_root_rational_image_with_target(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    target: Option<&AlgebraicPolynomialValueInterval>,
    policy: PredicatePolicy,
) -> AlgebraicRootRationalImageReport {
    let evaluation = evaluate_rational_expression_at_algebraic_root(
        root,
        numerator_coefficients,
        denominator_coefficients,
        policy,
    );
    transform_algebraic_root_rational_image_with_evaluation(
        root,
        numerator_coefficients,
        denominator_coefficients,
        target,
        policy,
        evaluation,
        RationalImageRetention::default(),
    )
}

fn transform_algebraic_root_rational_image_with_evaluation(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    target: Option<&AlgebraicPolynomialValueInterval>,
    policy: PredicatePolicy,
    evaluation: AlgebraicRootRationalEvaluationReport,
    retention: RationalImageRetention<'_>,
) -> AlgebraicRootRationalImageReport {
    let (refined_root, evaluation) = strengthen_rational_image_domain_evidence(
        root,
        numerator_coefficients,
        denominator_coefficients,
        policy,
        evaluation,
    );
    let root = refined_root.as_ref().unwrap_or(root);
    transform_algebraic_root_rational_image_with_domain_evaluation(
        root,
        numerator_coefficients,
        denominator_coefficients,
        target,
        policy,
        evaluation,
        retention,
    )
}

fn transform_algebraic_root_rational_image_with_domain_evaluation(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    target: Option<&AlgebraicPolynomialValueInterval>,
    policy: PredicatePolicy,
    evaluation: AlgebraicRootRationalEvaluationReport,
    retention: RationalImageRetention<'_>,
) -> AlgebraicRootRationalImageReport {
    if target.is_some_and(|target| rational_evaluation_is_disjoint(&evaluation, target, policy)) {
        return rational_image_report(
            AlgebraicRootRationalImageStatus::ImageIntervalDisjoint,
            numerator_coefficients,
            denominator_coefficients,
            evaluation,
            RationalImageArtifacts::default(),
            Some("rational image evaluation is disjoint from the target interval".into()),
        );
    }
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
        AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness => {
            if let Some(value) = evaluation.exact_value.clone() {
                let representation = canonical_linear_value_representation(root, value);
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
                Some("exact Real evaluation did not carry an exact value".into()),
            );
        }
        AlgebraicRootRationalEvaluationStatus::IntervalEvaluated => {}
    }

    if let Some(representation) = direct_rational_image_representation(
        root,
        numerator_coefficients,
        denominator_coefficients,
        policy,
        retention,
    ) {
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

    let numerator_image =
        transform_algebraic_root_polynomial_image(root, numerator_coefficients, policy);
    if numerator_image.status != AlgebraicRootPolynomialImageStatus::Transformed {
        let message = numerator_image.message.clone();
        return rational_image_report(
            polynomial_image_failure_status(
                numerator_image.status.clone(),
                AlgebraicRootRationalImageStatus::InvalidNumeratorPolynomial,
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
                AlgebraicRootRationalImageStatus::InvalidDenominatorPolynomial,
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
        policy,
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
        AlgebraicRootArithmeticStatus::ComputedExactRealWitness => {
            if let Some(value) = quotient.exact_result.clone() {
                let representation = canonical_linear_value_representation(root, value);
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
                    Some("exact quotient did not carry an exact value witness".into()),
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

fn strengthen_rational_image_domain_evidence(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
    mut evaluation: AlgebraicRootRationalEvaluationReport,
) -> (
    Option<AlgebraicRootRepresentation>,
    AlgebraicRootRationalEvaluationReport,
) {
    if evaluation.status != AlgebraicRootRationalEvaluationStatus::DenominatorMayContainZero {
        return (None, evaluation);
    }
    match strengthen_rational_image_denominator(
        root,
        denominator_coefficients,
        policy,
        &evaluation.denominator,
    ) {
        RationalImageDomainEvidence::Unchanged => (None, evaluation),
        RationalImageDomainEvidence::CertifiedPole => {
            mark_rational_image_evaluation_as_certified_pole(&mut evaluation);
            (None, evaluation)
        }
        RationalImageDomainEvidence::Refined(refined) => {
            let RefinedRationalImageDomain {
                root,
                denominator_evaluation,
            } = *refined;
            let evaluation = evaluate_rational_expression_with_denominator_evaluation(
                &root,
                numerator_coefficients,
                denominator_evaluation,
                policy,
            );
            (Some(root), evaluation)
        }
    }
}

fn strengthen_rational_image_denominator(
    root: &AlgebraicRootRepresentation,
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
    denominator_evaluation: &AlgebraicRootPolynomialEvaluationReport,
) -> RationalImageDomainEvidence {
    if !matches!(
        denominator_evaluation.status,
        crate::AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero
    ) || !has_exact_rational_coefficients(&root.polynomial_coefficients)
        || !has_exact_rational_coefficients(denominator_coefficients)
    {
        return RationalImageDomainEvidence::Unchanged;
    }
    match polynomial_vanishes_at_owned_root(
        &root.polynomial_coefficients,
        denominator_coefficients,
        &root.interval,
        policy,
    ) {
        Some(true) => return RationalImageDomainEvidence::CertifiedPole,
        Some(false) => {}
        None => return RationalImageDomainEvidence::Unchanged,
    }
    let mut refined_root = root.clone();
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
            break;
        }
        let Some(interval) = refinement.refined_interval else {
            break;
        };
        if interval == refined_root.interval {
            break;
        }
        refined_root.interval = interval;
        refined_root.validation = validate_algebraic_root_representation(&refined_root, policy);
        if !refined_root.is_valid() {
            break;
        }
        let refined_denominator = crate::algebraic::evaluate_polynomial_at_algebraic_root(
            &refined_root,
            denominator_coefficients,
            policy,
        );
        if matches!(
            refined_denominator.status,
            crate::AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
                | crate::AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedNegative
        ) || matches!(
            refined_denominator.status,
            crate::AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
                | crate::AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        ) && refined_denominator.sign != Some(Ordering::Equal)
        {
            return RationalImageDomainEvidence::Refined(Box::new(RefinedRationalImageDomain {
                root: refined_root,
                denominator_evaluation: refined_denominator,
            }));
        }
        if matches!(
            refined_denominator.status,
            crate::AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
                | crate::AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
        ) && refined_denominator.sign == Some(Ordering::Equal)
        {
            return RationalImageDomainEvidence::CertifiedPole;
        }
        if refined_denominator.status
            != crate::AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero
        {
            break;
        }
    }
    RationalImageDomainEvidence::Unchanged
}

fn mark_rational_image_evaluation_as_certified_pole(
    evaluation: &mut AlgebraicRootRationalEvaluationReport,
) {
    evaluation.status = AlgebraicRootRationalEvaluationStatus::CertifiedZeroDenominator;
    evaluation.exact_value = None;
    evaluation.interval_value = None;
    evaluation.sign = None;
    evaluation.message =
        Some("the authored denominator vanishes at the selected algebraic root".to_owned());
}

fn direct_rational_image_representation(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
    retention: RationalImageRetention<'_>,
) -> Option<AlgebraicRootRepresentation> {
    let owned_direct_map;
    let direct_map = if let Some(retained) = retention.direct_map {
        retained
            .get_or_init(|| {
                direct_rational_map(numerator_coefficients, denominator_coefficients, policy)
            })
            .as_ref()?
    } else {
        owned_direct_map =
            direct_rational_map(numerator_coefficients, denominator_coefficients, policy)?;
        &owned_direct_map
    };
    if let Some(value) = &direct_map.constant_value {
        return Some(canonical_linear_value_representation(root, value.clone()));
    }
    if let Some(representation) = direct_mobius_image_representation(root, direct_map, policy) {
        return Some(representation);
    }

    if !has_exact_rational_coefficients(&root.polynomial_coefficients)
        || !has_exact_rational_coefficients(&direct_map.numerator)
        || !has_exact_rational_coefficients(&direct_map.denominator)
    {
        return None;
    }
    let source_degree = root.polynomial_coefficients.len().checked_sub(1)?;
    let original_rational_degree = direct_map
        .numerator
        .len()
        .max(direct_map.denominator.len())
        .checked_sub(1)?;
    let reduce_modulo_source = direct_map.numerator.len() >= root.polynomial_coefficients.len()
        || direct_map.denominator.len() >= root.polynomial_coefficients.len();
    if !reduce_modulo_source
        && source_degree
            .checked_add(original_rational_degree.max(1))
            .is_none_or(|dimension| dimension > MAX_RATIONAL_IMAGE_SYLVESTER_DIMENSION)
    {
        return None;
    }
    let owned_source_polynomial;
    let source_polynomial = if let Some(retained) = retention.source_polynomial {
        retained
            .get_or_init(|| primitive_integer_polynomial(&root.polynomial_coefficients))
            .as_deref()?
    } else {
        owned_source_polynomial = primitive_integer_polynomial(&root.polynomial_coefficients)?;
        &owned_source_polynomial
    };
    let owned_reduced_map;
    let direct_map = if reduce_modulo_source && retention.direct_map.is_some() {
        direct_map
            .source_reduced_map
            .get_or_init(|| {
                reduce_rational_map_modulo_source(direct_map, source_polynomial, policy)
                    .map(Box::new)
            })
            .as_deref()?
    } else if reduce_modulo_source {
        owned_reduced_map =
            reduce_rational_map_modulo_source(direct_map, source_polynomial, policy)?;
        &owned_reduced_map
    } else {
        direct_map
    };
    if let Some(value) = &direct_map.constant_value {
        return Some(canonical_linear_value_representation(root, value.clone()));
    }
    if let Some(representation) = direct_mobius_image_representation(root, direct_map, policy) {
        return Some(representation);
    }
    let rational_degree = direct_map
        .numerator
        .len()
        .max(direct_map.denominator.len())
        .checked_sub(1)?;
    if source_degree
        .checked_add(rational_degree.max(1))
        .is_none_or(|dimension| dimension > MAX_RATIONAL_IMAGE_SYLVESTER_DIMENSION)
    {
        return None;
    }
    let derivative_numerator = direct_map
        .derivative_numerator
        .get_or_init(|| {
            rational_derivative_numerator(&direct_map.numerator, &direct_map.denominator, policy)
        })
        .as_ref()?;
    let (resultant_numerator, resultant_denominator) = direct_map
        .cleared_coefficients
        .get_or_init(|| {
            clear_rational_map_denominators(&direct_map.numerator, &direct_map.denominator)
        })
        .as_ref()?;
    let polynomial_coefficients = if let Some(retained) = retention.resultant_polynomial {
        retained
            .get_or_init(|| {
                resultant_polynomial_for_rational_image(
                    source_polynomial,
                    resultant_numerator,
                    resultant_denominator,
                    source_degree,
                    policy,
                )
            })
            .clone()?
    } else {
        resultant_polynomial_for_rational_image(
            source_polynomial,
            resultant_numerator,
            resultant_denominator,
            source_degree,
            policy,
        )?
    };
    let interval = certified_rational_image_interval(
        root,
        &direct_map.numerator,
        &direct_map.denominator,
        derivative_numerator,
        &polynomial_coefficients,
        policy,
    )?;
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
    representation = promote_linear_square_free_image_root(representation, policy);
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    Some(representation)
}

fn direct_mobius_image_representation(
    root: &AlgebraicRootRepresentation,
    direct_map: &DirectRationalMap,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootRepresentation> {
    if direct_map.numerator.len() > 2 || direct_map.denominator.len() > 2 {
        return None;
    }
    let mobius = transform_algebraic_root_mobius(
        root,
        direct_map
            .numerator
            .get(1)
            .cloned()
            .unwrap_or_else(Real::zero),
        direct_map.numerator[0].clone(),
        direct_map
            .denominator
            .get(1)
            .cloned()
            .unwrap_or_else(Real::zero),
        direct_map.denominator[0].clone(),
        policy,
    );
    if mobius.status != AlgebraicRootMobiusTransformStatus::Transformed {
        return None;
    }
    let mut representation = mobius.representation?;
    if let Some(polynomial) = primitive_integer_polynomial(&representation.polynomial_coefficients)
    {
        representation.polynomial_coefficients = polynomial;
    }
    representation = promote_linear_square_free_image_root(representation, policy);
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    representation.is_valid().then_some(representation)
}

fn reduce_rational_map_modulo_source(
    direct_map: &DirectRationalMap,
    source_polynomial: &[Real],
    policy: PredicatePolicy,
) -> Option<DirectRationalMap> {
    let reduce = |polynomial: &[Real]| {
        if polynomial.len() < source_polynomial.len() {
            Some(polynomial.to_vec())
        } else {
            polynomial_div_rem(polynomial.to_vec(), source_polynomial, policy)
                .map(|(_, remainder)| remainder)
        }
    };
    let numerator = reduce(&direct_map.numerator)?;
    let denominator = reduce(&direct_map.denominator)?;
    direct_rational_map(&numerator, &denominator, policy)
}

fn promote_linear_square_free_image_root(
    representation: AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> AlgebraicRootRepresentation {
    if representation.exact_point_witness().is_some() {
        return representation;
    }
    if let [constant, linear] = representation.polynomial_coefficients.as_slice() {
        let Some(linear_reciprocal) =
            crate::policy_division::reciprocal_with_policy(linear, policy)
        else {
            return representation;
        };
        let root = -constant.clone() * linear_reciprocal;
        return promote_image_root_candidate(representation, root, policy);
    }
    if let [constant, linear, quadratic] = representation.polynomial_coefficients.as_slice() {
        let discriminant = linear * linear - Real::from(4_i8) * quadratic * constant;
        match compare_reals(&discriminant, &Real::zero(), policy).value() {
            Some(Ordering::Less | Ordering::Greater) => return representation,
            Some(Ordering::Equal) => {
                let denominator = Real::from(2_i8) * quadratic;
                let Some(reciprocal) =
                    crate::policy_division::reciprocal_with_policy(&denominator, policy)
                else {
                    return representation;
                };
                let root = -linear.clone() * reciprocal;
                return promote_image_root_candidate(representation, root, policy);
            }
            None => {}
        }
    }
    let Some(square_free) =
        square_free_part(representation.polynomial_coefficients.clone(), policy)
    else {
        return representation;
    };
    let [constant, linear] = square_free.as_slice() else {
        return representation;
    };
    let Some(linear_reciprocal) = crate::policy_division::reciprocal_with_policy(linear, policy)
    else {
        return representation;
    };
    let root = -constant.clone() * linear_reciprocal;
    promote_image_root_candidate(representation, root, policy)
}

fn promote_image_root_candidate(
    representation: AlgebraicRootRepresentation,
    root: Real,
    policy: PredicatePolicy,
) -> AlgebraicRootRepresentation {
    let inside = matches!(
        compare_reals(&representation.interval.lower, &root, policy).value(),
        Some(Ordering::Less | Ordering::Equal)
    ) && matches!(
        compare_reals(&root, &representation.interval.upper, policy).value(),
        Some(Ordering::Less | Ordering::Equal)
    );
    if !inside
        || compare_reals(
            &Real::eval_poly(&representation.polynomial_coefficients, &root),
            &Real::zero(),
            policy,
        )
        .value()
            != Some(Ordering::Equal)
    {
        return representation;
    }
    canonical_linear_value_representation(&representation, root)
}

fn direct_rational_map(
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
) -> Option<DirectRationalMap> {
    let mut numerator = trim_real_polynomial(numerator_coefficients.to_vec(), policy)?;
    let mut denominator = trim_real_polynomial(denominator_coefficients.to_vec(), policy)?;
    let constant_value = constant_rational_map_value(&numerator, &denominator, policy);
    if constant_value.is_some() || (numerator.len() <= 2 && denominator.len() <= 2) {
        return Some(DirectRationalMap {
            numerator,
            denominator,
            constant_value,
            source_reduced_map: OnceLock::new(),
            derivative_numerator: OnceLock::new(),
            cleared_coefficients: OnceLock::new(),
        });
    }
    if !has_exact_rational_coefficients(&numerator)
        || !has_exact_rational_coefficients(&denominator)
    {
        return None;
    }
    // Two nonconstant linears either are coprime or describe a constant map;
    // `constant_rational_map_value` handles the latter without a polynomial GCD.
    if constant_value.is_none()
        && numerator.len() > 1
        && denominator.len() > 1
        && (numerator.len() != 2 || denominator.len() != 2)
    {
        let gcd = primitive_integer_polynomial_gcd(&numerator, &denominator)?;
        if gcd.len() > 1 {
            let (reduced_numerator, numerator_remainder) =
                polynomial_div_rem(numerator, &gcd, policy)?;
            let (reduced_denominator, denominator_remainder) =
                polynomial_div_rem(denominator, &gcd, policy)?;
            if !is_exact_zero_polynomial(&numerator_remainder, policy)
                || !is_exact_zero_polynomial(&denominator_remainder, policy)
            {
                return None;
            }
            numerator = reduced_numerator;
            denominator = reduced_denominator;
        }
    }
    Some(DirectRationalMap {
        numerator,
        denominator,
        constant_value,
        source_reduced_map: OnceLock::new(),
        derivative_numerator: OnceLock::new(),
        cleared_coefficients: OnceLock::new(),
    })
}

fn is_exact_zero_polynomial(polynomial: &[Real], policy: PredicatePolicy) -> bool {
    if polynomial
        .iter()
        .all(|coefficient| coefficient.exact_rational_ref().is_some())
    {
        return polynomial.iter().all(|coefficient| {
            coefficient
                .exact_rational_ref()
                .expect("exact rational coefficients were checked")
                .is_zero()
        });
    }
    polynomial.iter().all(|coefficient| {
        compare_reals(coefficient, &Real::zero(), policy).value() == Some(Ordering::Equal)
    })
}

fn clear_rational_map_denominators(
    numerator: &[Real],
    denominator: &[Real],
) -> Option<(Vec<Real>, Vec<Real>)> {
    let rationals = numerator
        .iter()
        .chain(denominator)
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let mut integers = Rational::primitive_bigint_ratio(&rationals)
        .into_iter()
        .map(Rational::from_bigint)
        .map(Real::from);
    let numerator = integers.by_ref().take(numerator.len()).collect();
    let denominator = integers.collect();
    Some((numerator, denominator))
}

fn resultant_polynomial_for_rational_image(
    source_polynomial: &[Real],
    numerator: &[Real],
    denominator: &[Real],
    source_degree: usize,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let polynomial = quotient_ring_resultant_polynomial(source_polynomial, numerator, denominator)
        .or_else(|| {
            let mut samples = Vec::with_capacity(source_degree + 1);
            for sample in 0..=source_degree {
                let y = Real::from(sample as i64);
                let relation = polynomial_sub(numerator, &polynomial_scale(denominator, &y));
                let resultant = resultant_univariate_polynomials(source_polynomial, &relation, -64)
                    .ok()?
                    .resultant;
                samples.push(resultant);
            }
            interpolate_integer_samples_up_to_scale(&samples)
        })?;
    trim_real_polynomial(primitive_integer_polynomial(&polynomial)?, policy)
}

fn rational_image_interval(
    interval: &IsolatedRootInterval,
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    if let Some(root) = interval.exact_root.as_ref() {
        let value = evaluate_rational_polynomial(numerator, denominator, root, policy)?;
        return Some(IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value),
            distinct_root_count: 1,
        });
    }
    let first = evaluate_rational_polynomial(numerator, denominator, &interval.lower, policy)?;
    let second = evaluate_rational_polynomial(numerator, denominator, &interval.upper, policy)?;
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    let [lower, upper] = endpoints;
    Some(IsolatedRootInterval {
        lower,
        upper,
        exact_root: None,
        distinct_root_count: interval.distinct_root_count,
    })
}

fn certified_rational_image_interval(
    root: &AlgebraicRootRepresentation,
    numerator: &[Real],
    denominator: &[Real],
    derivative_numerator: &[Real],
    image_polynomial: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    certify_algebraic_image_interval(
        &root.polynomial_coefficients,
        &root.interval,
        image_polynomial,
        policy,
        |source_interval| {
            rational_image_enclosure(
                source_interval,
                numerator,
                denominator,
                derivative_numerator,
                policy,
            )
        },
    )
}

fn rational_image_enclosure(
    interval: &IsolatedRootInterval,
    numerator: &[Real],
    denominator: &[Real],
    derivative_numerator: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    if interval.exact_root.is_some() {
        return rational_image_interval(interval, numerator, denominator, policy);
    }
    let source = ValueInterval {
        lower: interval.lower.clone(),
        upper: interval.upper.clone(),
    };
    let derivative_interval = evaluate_interval_polynomial(derivative_numerator, &source, policy)?;
    let derivative_sign = interval_strict_sign(&derivative_interval, policy)?;
    if derivative_sign != Ordering::Equal {
        return rational_image_interval(interval, numerator, denominator, policy);
    }
    let numerator_interval = evaluate_interval_polynomial(numerator, &source, policy)?;
    let denominator_interval = evaluate_interval_polynomial(denominator, &source, policy)?;
    if interval_strict_sign(&denominator_interval, policy)? == Ordering::Equal {
        return None;
    }
    let lower_reciprocal =
        crate::policy_division::reciprocal_with_policy(&denominator_interval.lower, policy)?;
    let upper_reciprocal =
        crate::policy_division::reciprocal_with_policy(&denominator_interval.upper, policy)?;
    let quotient = interval_mul(
        &numerator_interval,
        &ValueInterval {
            lower: upper_reciprocal,
            upper: lower_reciprocal,
        },
        policy,
    )?;
    Some(IsolatedRootInterval {
        lower: quotient.lower,
        upper: quotient.upper,
        exact_root: None,
        distinct_root_count: 1,
    })
}

fn evaluate_rational_polynomial(
    numerator: &[Real],
    denominator: &[Real],
    point: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let numerator = Real::eval_poly(numerator, point);
    let denominator = Real::eval_poly(denominator, point);
    Some(numerator * crate::policy_division::reciprocal_with_policy(&denominator, policy)?)
}

fn constant_rational_map_value(
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<Real> {
    if has_exact_rational_coefficients(numerator) && has_exact_rational_coefficients(denominator) {
        let zero = Rational::zero();
        let max_len = numerator.len().max(denominator.len());
        let denominator_pivot_index = (0..max_len).find(|index| {
            !denominator
                .get(*index)
                .map_or(&zero, |coefficient| {
                    coefficient
                        .exact_rational_ref()
                        .expect("exact coefficients were checked")
                })
                .is_zero()
        })?;
        let denominator_pivot = denominator[denominator_pivot_index]
            .exact_rational_ref()
            .expect("exact coefficients were checked");
        let numerator_pivot = numerator
            .get(denominator_pivot_index)
            .map_or(&zero, |coefficient| {
                coefficient
                    .exact_rational_ref()
                    .expect("exact coefficients were checked")
            });
        for index in 0..max_len {
            let numerator_coefficient = numerator.get(index).map_or(&zero, |coefficient| {
                coefficient
                    .exact_rational_ref()
                    .expect("exact coefficients were checked")
            });
            let denominator_coefficient = denominator.get(index).map_or(&zero, |coefficient| {
                coefficient
                    .exact_rational_ref()
                    .expect("exact coefficients were checked")
            });
            if numerator_coefficient * denominator_pivot
                != denominator_coefficient * numerator_pivot
            {
                return None;
            }
        }
        return Some(Real::from(numerator_pivot / denominator_pivot));
    }

    let max_len = numerator.len().max(denominator.len());
    let mut scale = None;
    for index in 0..max_len {
        let denominator_coefficient = denominator.get(index).cloned().unwrap_or_else(Real::zero);
        if compare_reals(&denominator_coefficient, &Real::zero(), policy).value()?
            == Ordering::Equal
        {
            continue;
        }
        let numerator_coefficient = numerator.get(index).cloned().unwrap_or_else(Real::zero);
        scale = Some(
            numerator_coefficient
                * crate::policy_division::reciprocal_with_policy(&denominator_coefficient, policy)?,
        );
        break;
    }
    let scale = scale?;
    let scaled_denominator = polynomial_scale(denominator, &scale);
    let difference = trim_real_polynomial(polynomial_sub(numerator, &scaled_denominator), policy)?;
    (difference.len() == 1
        && compare_reals(&difference[0], &Real::zero(), policy).value()? == Ordering::Equal)
        .then_some(scale)
}

fn rational_derivative_numerator(
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    if let [denominator] = denominator {
        let derivative = derivative_coefficients(numerator);
        return trim_real_polynomial(polynomial_scale(&derivative, denominator), policy);
    }
    if let [numerator] = numerator {
        let scale = -numerator.clone();
        let derivative = derivative_coefficients(denominator);
        return trim_real_polynomial(polynomial_scale(&derivative, &scale), policy);
    }
    trim_real_polynomial(
        polynomial_sub(
            &polynomial_mul(&derivative_coefficients(numerator), denominator),
            &polynomial_mul(numerator, &derivative_coefficients(denominator)),
        ),
        policy,
    )
}

fn derivative_coefficients(polynomial: &[Real]) -> Vec<Real> {
    polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient.clone() * Real::from(degree as i64))
        .collect()
}

fn evaluate_interval_polynomial(
    polynomial: &[Real],
    point: &ValueInterval,
    policy: PredicatePolicy,
) -> Option<ValueInterval> {
    let mut coefficients = polynomial.iter().rev();
    let Some(leading) = coefficients.next() else {
        return Some(ValueInterval {
            lower: Real::zero(),
            upper: Real::zero(),
        });
    };
    let mut value = ValueInterval {
        lower: leading.clone(),
        upper: leading.clone(),
    };
    for coefficient in coefficients {
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
    if let (Some(left_lower), Some(left_upper), Some(right_lower), Some(right_upper)) = (
        left.lower.exact_rational_ref(),
        left.upper.exact_rational_ref(),
        right.lower.exact_rational_ref(),
        right.upper.exact_rational_ref(),
    ) {
        let (lower, upper) =
            rational_interval_product(left_lower, left_upper, right_lower, right_upper);
        return Some(ValueInterval {
            lower: Real::from(lower),
            upper: Real::from(upper),
        });
    }
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

fn interval_strict_sign(interval: &ValueInterval, policy: PredicatePolicy) -> Option<Ordering> {
    let lower = compare_reals(&interval.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals(&interval.upper, &Real::zero(), policy).value()?;
    if lower == Ordering::Greater {
        Some(Ordering::Greater)
    } else if upper == Ordering::Less {
        Some(Ordering::Less)
    } else {
        Some(Ordering::Equal)
    }
}

#[derive(Clone, Debug)]
struct ValueInterval {
    lower: Real,
    upper: Real,
}

fn polynomial_scale(polynomial: &[Real], scale: &Real) -> Vec<Real> {
    polynomial
        .iter()
        .map(|coefficient| coefficient.clone() * scale.clone())
        .collect()
}

fn polynomial_sub(left: &[Real], right: &[Real]) -> Vec<Real> {
    let len = left.len().max(right.len());
    (0..len)
        .map(|index| {
            left.get(index).cloned().unwrap_or_else(Real::zero)
                - right.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

fn polynomial_mul(left: &[Real], right: &[Real]) -> Vec<Real> {
    if left.is_empty() || right.is_empty() {
        return vec![Real::zero()];
    }
    let mut product = vec![Real::zero(); left.len() + right.len() - 1];
    for (left_index, left_coefficient) in left.iter().enumerate() {
        for (right_index, right_coefficient) in right.iter().enumerate() {
            product[left_index + right_index] = product[left_index + right_index].clone()
                + left_coefficient.clone() * right_coefficient.clone();
        }
    }
    product
}

fn trim_real_polynomial(mut polynomial: Vec<Real>, policy: PredicatePolicy) -> Option<Vec<Real>> {
    while polynomial.len() > 1 {
        let trailing = polynomial.last()?;
        if let Some(trailing) = trailing.exact_rational_ref() {
            if trailing.is_zero() {
                polynomial.pop();
                continue;
            }
            break;
        }
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

fn has_exact_rational_coefficients(polynomial: &[Real]) -> bool {
    !polynomial.is_empty()
        && polynomial
            .iter()
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
}

fn rational_evaluation_is_disjoint(
    evaluation: &AlgebraicRootRationalEvaluationReport,
    target: &AlgebraicPolynomialValueInterval,
    policy: PredicatePolicy,
) -> bool {
    let value_interval = match evaluation.status {
        AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness
        | AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness => {
            evaluation.exact_value.as_ref().map(|value| (value, value))
        }
        AlgebraicRootRationalEvaluationStatus::IntervalEvaluated => evaluation
            .interval_value
            .as_ref()
            .map(|interval| (&interval.lower, &interval.upper)),
        _ => None,
    };
    value_interval.is_some_and(|(lower, upper)| {
        compare_reals(upper, &target.lower, policy).value() == Some(Ordering::Less)
            || compare_reals(lower, &target.upper, policy).value() == Some(Ordering::Greater)
    })
}

fn polynomial_image_failure_status(
    status: AlgebraicRootPolynomialImageStatus,
    invalid_polynomial: AlgebraicRootRationalImageStatus,
    fallback: AlgebraicRootRationalImageStatus,
) -> AlgebraicRootRationalImageStatus {
    match status {
        AlgebraicRootPolynomialImageStatus::InvalidEvidence
        | AlgebraicRootPolynomialImageStatus::InvalidTransformedEvidence => {
            AlgebraicRootRationalImageStatus::InvalidTransformedEvidence
        }
        AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial => invalid_polynomial,
        AlgebraicRootPolynomialImageStatus::ImageIsolationFailed
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

    fn fraction(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
    }

    #[test]
    fn rational_image_divisions_reuse_policy_nonzero_evidence() {
        let denominator = crate::test_support::exact_normal_positive();
        let half = fraction(1, 2);
        let numerator = denominator.clone() * &half;
        assert_eq!(
            &numerator / &denominator,
            Err(hyperreal::Problem::UnknownZero)
        );

        let value = evaluate_rational_polynomial(
            core::slice::from_ref(&numerator),
            core::slice::from_ref(&denominator),
            &real(7),
            PredicatePolicy::STRICT,
        )
        .expect("the policy-certified constant denominator should divide");
        assert_eq!(value.exact_rational_normal_form(), half.exact_rational());
        assert_eq!(
            constant_rational_map_value(
                core::slice::from_ref(&numerator),
                core::slice::from_ref(&denominator),
                PredicatePolicy::STRICT,
            )
            .and_then(|value| value.exact_rational_normal_form()),
            half.exact_rational()
        );

        let interval = rational_image_interval(
            &IsolatedRootInterval {
                lower: real(0),
                upper: real(1),
                exact_root: Some(fraction(1, 3)),
                distinct_root_count: 1,
            },
            core::slice::from_ref(&numerator),
            core::slice::from_ref(&denominator),
            PredicatePolicy::STRICT,
        )
        .expect("policy-certified endpoint denominators should construct an image");
        assert_eq!(
            interval.lower.exact_rational_normal_form(),
            half.exact_rational()
        );
        assert_eq!(
            interval.upper.exact_rational_normal_form(),
            half.exact_rational()
        );

        assert!(
            crate::policy_division::reciprocal_with_policy(
                &crate::test_support::terminal_zero(),
                PredicatePolicy::STRICT,
            )
            .is_none()
        );
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
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let root = report.representation.as_ref().unwrap();
        assert!(root.is_valid());
        assert_eq!(root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(root.interval.upper, (real(2) / real(3)).unwrap());
    }

    #[test]
    fn rational_image_preserves_partition_ownership_for_reversed_maps() {
        // (x - 1)(x^2 - 2) has an excluded root at the lower endpoint and
        // the selected sqrt(2) root in (1, 2]. Negation would move the
        // excluded -1 image to the included upper endpoint unless the source
        // interval is first refined.
        let mut source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(2), real(-2), real(-1), Real::one()],
            ..sqrt_two_positive()
        };
        source.validation =
            validate_algebraic_root_representation(&source, PredicatePolicy::STRICT);
        assert!(source.is_valid());

        let report = transform_algebraic_root_rational_image(
            &source,
            &[Real::zero(), real(-1)],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert!(image.is_valid());
        assert_eq!(
            compare_reals(&image.interval.upper, &real(-1), PredicatePolicy::STRICT).value(),
            Some(Ordering::Less)
        );
        assert!(matches!(
            refine_isolated_univariate_polynomial_interval(
                &image.polynomial_coefficients,
                &image.interval,
                RootIsolationConfig {
                    policy: PredicatePolicy::STRICT,
                    max_interval_width: None,
                    max_refinement_steps: 0,
                },
            )
            .status,
            IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
        ));

        // A selected root authored at the included upper endpoint becomes an
        // exact point before a decreasing map can place it on an excluded
        // image endpoint.
        let mut upper_endpoint = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-2), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two_positive()
        };
        upper_endpoint.validation =
            validate_algebraic_root_representation(&upper_endpoint, PredicatePolicy::STRICT);
        let report = transform_algebraic_root_rational_image(
            &upper_endpoint,
            &[Real::zero(), real(-1)],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert_eq!(
            report
                .representation
                .as_ref()
                .unwrap()
                .exact_point_witness(),
            Some(&real(-2))
        );
    }

    #[test]
    fn rational_image_cancels_a_common_factor_outside_the_isolated_root() {
        // The represented root is sqrt(2), while the defining polynomial also
        // carries the root -3. The rational map is x after cancelling x + 3;
        // retaining that harmless common factor makes the direct resultant
        // vanish identically even though the denominator is positive on [1, 2].
        let mut root = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-6), real(-2), real(3), real(1)],
            ..sqrt_two_positive()
        };
        root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
        assert!(root.is_valid());

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &root,
                &[Real::zero(), real(3), Real::one()],
                &[real(3), Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let image = report.representation.as_ref().unwrap();
            assert!(image.is_valid());
            assert_eq!(image.interval.lower, real(1));
            assert_eq!(image.interval.upper, real(2));
        }
    }

    #[test]
    fn rational_image_batch_matches_independent_shared_denominator_transforms() {
        let root = sqrt_two_positive();
        let denominator = [real(3), Real::one()];
        let numerators = [
            [real(1), real(2), Real::one()],
            [real(-2), Real::one(), Real::one()],
        ];
        let reports = transform_algebraic_root_rational_images(
            &root,
            [&numerators[0], &numerators[1]],
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );

        for (report, numerator) in reports.into_iter().zip(numerators) {
            assert_eq!(
                report,
                transform_algebraic_root_rational_image(
                    &root,
                    &numerator,
                    &denominator,
                    PredicatePolicy::APPROXIMATE_512,
                )
            );
        }
    }

    #[test]
    fn rational_image_batch_preserves_invalid_source_reports_and_empty_noop() {
        let mut stale = sqrt_two_positive();
        stale.interval.upper = Real::zero();
        let denominator = [real(3), Real::one()];
        let numerators = [[real(1), real(2)], [real(-2), Real::one()]];
        let reports = transform_algebraic_root_rational_images(
            &stale,
            [&numerators[0], &numerators[1]],
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );

        for (report, numerator) in reports.into_iter().zip(numerators) {
            assert_eq!(
                report,
                transform_algebraic_root_rational_image(
                    &stale,
                    &numerator,
                    &denominator,
                    PredicatePolicy::APPROXIMATE_512,
                )
            );
        }
        let empty: [AlgebraicRootRationalImageReport; 0] = transform_algebraic_root_rational_images(
            &stale,
            [],
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert!(empty.is_empty());
    }

    #[test]
    fn rational_image_batch_shares_strengthened_domain_evidence_exactly() {
        let numerators = [vec![Real::one()], vec![Real::zero(), Real::one()]];
        for denominator in [
            vec![real(2), real(-2), Real::one()],
            sqrt_two_positive().polynomial_coefficients,
        ] {
            let batch = transform_algebraic_root_rational_images(
                &sqrt_two_positive(),
                [&numerators[0], &numerators[1]],
                &denominator,
                PredicatePolicy::STRICT,
            );

            for (report, numerator) in batch.into_iter().zip(&numerators) {
                assert_eq!(
                    report,
                    transform_algebraic_root_rational_image(
                        &sqrt_two_positive(),
                        numerator,
                        &denominator,
                        PredicatePolicy::STRICT,
                    )
                );
            }
        }
    }

    #[test]
    fn rational_map_reuses_elimination_across_source_roots() {
        let mut positive = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![Real::zero(), real(-2), Real::zero(), Real::one()],
            ..sqrt_two_positive()
        };
        positive.validation =
            validate_algebraic_root_representation(&positive, PredicatePolicy::STRICT);
        let negative = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: real(-2),
                upper: real(-1),
                exact_root: None,
                distinct_root_count: 1,
            },
            interval_index: 1,
            ..positive.clone()
        };
        let numerator = [Real::zero(), Real::one(), Real::one()];
        let denominator = [Real::one()];
        let map = AlgebraicRootRationalMap::new(
            &positive.polynomial_coefficients,
            &numerator,
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );
        assert!(map.resultant_polynomial.get().is_none());
        assert!(map.direct_map.get().is_none());

        for root in [&positive, &negative] {
            assert_eq!(
                map.transform(root),
                transform_algebraic_root_rational_image(
                    root,
                    &numerator,
                    &denominator,
                    PredicatePolicy::APPROXIMATE_512,
                )
            );
            assert!(map.resultant_polynomial.get().is_some());
            let direct_map = map
                .direct_map
                .get()
                .and_then(Option::as_ref)
                .expect("direct map is retained after a transformed root");
            assert!(direct_map.derivative_numerator.get().is_some());
            assert!(direct_map.cleared_coefficients.get().is_some());
        }
    }

    #[test]
    fn rational_map_falls_back_for_a_distinct_source_polynomial() {
        let mut scaled = sqrt_two_positive();
        scaled.polynomial_coefficients = scaled
            .polynomial_coefficients
            .iter()
            .cloned()
            .map(|value| value * real(2))
            .collect();
        scaled.validation =
            validate_algebraic_root_representation(&scaled, PredicatePolicy::APPROXIMATE_512);
        assert!(scaled.is_valid());
        let numerator = [Real::zero(), Real::one(), Real::one()];
        let denominator = [Real::one()];
        let map = AlgebraicRootRationalMap::new(
            &sqrt_two_positive().polynomial_coefficients,
            &numerator,
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            map.transform(&scaled),
            transform_algebraic_root_rational_image(
                &scaled,
                &numerator,
                &denominator,
                PredicatePolicy::APPROXIMATE_512,
            )
        );
        assert!(map.direct_map.get().is_none());
        assert!(map.resultant_polynomial.get().is_none());
    }

    #[test]
    fn bounded_rational_image_skips_disjoint_elimination() {
        let report = transform_algebraic_root_rational_image_in_interval(
            &sqrt_two_positive(),
            &[real(2), Real::one()],
            &[Real::one()],
            &AlgebraicPolynomialValueInterval {
                lower: Real::zero(),
                upper: Real::one(),
            },
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::ImageIntervalDisjoint
        );
        assert!(report.representation.is_none());
        assert!(report.numerator_image.is_none());
        assert!(report.denominator_image.is_none());
        assert!(report.quotient.is_none());
    }

    #[test]
    fn bounded_rational_image_retains_boundary_touching_enclosures() {
        let report = transform_algebraic_root_rational_image_in_interval(
            &sqrt_two_positive(),
            &[real(-1), Real::one()],
            &[Real::one()],
            &AlgebraicPolynomialValueInterval {
                lower: Real::zero(),
                upper: Real::one(),
            },
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.representation.as_ref().unwrap().is_valid());
    }

    #[test]
    fn rational_map_target_rejection_keeps_construction_caches_cold() {
        let root = sqrt_two_positive();
        let map = AlgebraicRootRationalMap::new(
            &root.polynomial_coefficients,
            &[real(2), Real::one()],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        let report = map.transform_in_interval(
            &root,
            &AlgebraicPolynomialValueInterval {
                lower: Real::zero(),
                upper: Real::one(),
            },
        );

        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::ImageIntervalDisjoint
        );
        assert!(map.direct_map.get().is_none());
        assert!(map.source_polynomial.get().is_none());
        assert!(map.resultant_polynomial.get().is_none());

        let pole = AlgebraicRootRationalMap::new(
            &root.polynomial_coefficients,
            &[Real::one()],
            &root.polynomial_coefficients,
            PredicatePolicy::STRICT,
        )
        .transform_in_interval(
            &root,
            &AlgebraicPolynomialValueInterval {
                lower: real(100),
                upper: real(101),
            },
        );
        assert_eq!(
            pole.status,
            AlgebraicRootRationalImageStatus::CertifiedZeroDenominator
        );
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
            ..sqrt_two_positive()
        };

        let report = transform_algebraic_root_rational_image(
            &rational,
            &[real(1), real(2), real(3)],
            &[real(1), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let root = report.representation.as_ref().unwrap();
        assert_eq!(
            root.exact_point_witness(),
            Some(&(real(34) / real(4)).unwrap())
        );
        assert!(root.is_valid());
    }

    #[test]
    fn rational_image_does_not_mislabel_an_exact_real_point_value() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let source = AlgebraicRootRepresentation {
            interval: IsolatedRootInterval {
                lower: sqrt_two_value.clone(),
                upper: sqrt_two_value.clone(),
                exact_root: Some(sqrt_two_value.clone()),
                distinct_root_count: 1,
            },
            ..sqrt_two_positive()
        };

        let report = transform_algebraic_root_rational_image(
            &source,
            &[Real::zero(), Real::one()],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert_eq!(
            report.evaluation.status,
            AlgebraicRootRationalEvaluationStatus::EvaluatedExactRealWitness
        );
        let root = report.representation.as_ref().unwrap();
        assert!(root.interval.exact_root.is_none());
        assert_eq!(root.interval.lower, sqrt_two_value);
        assert_eq!(root.interval.lower, root.interval.upper);
        assert!(root.is_valid());
    }

    #[test]
    fn rational_image_reuses_mobius_support_for_an_exact_real_source() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![-sqrt_two_value, Real::one()],
            interval: IsolatedRootInterval {
                lower: real(1),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two_positive()
        };
        let source = AlgebraicRootRepresentation {
            validation: validate_algebraic_root_representation(&source, PredicatePolicy::STRICT),
            ..source
        };
        assert!(source.is_valid());

        let report = transform_algebraic_root_rational_image(
            &source,
            &[Real::zero(), Real::one()],
            &[Real::one(), Real::one()],
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert!(image.is_valid());
        assert!(image.interval.exact_root.is_none());
        assert!(report.numerator_image.is_none());
        assert!(report.denominator_image.is_none());
    }

    #[test]
    fn rational_image_reuses_mobius_support_for_exact_real_map_coefficients() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let constant = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            core::slice::from_ref(&sqrt_two_value),
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            constant.status,
            AlgebraicRootRationalImageStatus::Transformed
        );
        let constant = constant.representation.as_ref().unwrap();
        assert!(constant.interval.exact_root.is_none());
        assert_eq!(constant.interval.lower, sqrt_two_value);
        assert_eq!(constant.interval.lower, constant.interval.upper);

        let affine = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[sqrt_two_value, Real::one()],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(affine.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(affine.representation.as_ref().unwrap().is_valid());
        assert!(affine.numerator_image.is_none());
        assert!(affine.denominator_image.is_none());
    }

    #[test]
    fn rational_image_distinguishes_unsupported_nonlinear_map_coefficients() {
        let sqrt_two_value = real(2).sqrt().unwrap();
        let invalid_numerator = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one(), Real::zero(), sqrt_two_value.clone()],
            &[Real::one()],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            invalid_numerator.status,
            AlgebraicRootRationalImageStatus::InvalidNumeratorPolynomial
        );
        assert_eq!(
            invalid_numerator
                .numerator_image
                .as_ref()
                .map(|report| &report.status),
            Some(&AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial)
        );

        let invalid_denominator = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one()],
            &[real(10), Real::zero(), sqrt_two_value],
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            invalid_denominator.status,
            AlgebraicRootRationalImageStatus::InvalidDenominatorPolynomial
        );
        assert_eq!(
            invalid_denominator
                .denominator_image
                .as_ref()
                .map(|report| &report.status),
            Some(&AlgebraicRootPolynomialImageStatus::InvalidImagePolynomial)
        );
    }

    #[test]
    fn rational_image_promotes_a_repeated_rational_image_root() {
        let source = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-1), Real::one(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::zero(),
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            ..sqrt_two_positive()
        };

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &source,
                &[Real::zero(), real(2), real(2)],
                &[Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let root = report.representation.as_ref().unwrap();
            assert_eq!(root.exact_point_witness(), Some(&real(2)));
            assert_eq!(root.polynomial_coefficients, vec![real(-2), Real::one()]);
            assert!(root.is_valid());
        }
    }

    #[test]
    fn rational_image_reduces_both_map_sides_modulo_the_source_relation() {
        let numerator = [Real::zero(), Real::one()];
        let denominator = [real(-1), Real::zero(), Real::one()];
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &numerator,
            &denominator,
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert_eq!(report.numerator_coefficients, numerator);
        assert_eq!(report.denominator_coefficients, denominator);
        let image = report.representation.as_ref().unwrap();
        assert_eq!(
            image.polynomial_coefficients,
            sqrt_two_positive().polynomial_coefficients
        );
        assert!(image.is_valid());
    }

    #[test]
    fn rational_image_reduces_an_oversized_map_before_the_degree_budget() {
        let mut source_multiple = vec![Real::zero(); 19];
        source_multiple[18] = Real::one();
        let oversized = polynomial_sub(
            &polynomial_mul(
                &sqrt_two_positive().polynomial_coefficients,
                &source_multiple,
            ),
            &[Real::zero(), real(-1)],
        );
        assert_eq!(oversized.len(), 21);

        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &oversized,
            &[Real::one()],
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert_eq!(
            image.polynomial_coefficients,
            sqrt_two_positive().polynomial_coefficients
        );
        assert!(image.is_valid());
    }

    #[test]
    fn rational_map_caches_a_source_reduced_map_without_forcing_a_resultant() {
        let root = sqrt_two_positive();
        let numerator = [Real::zero(), Real::zero(), Real::one()];
        let map = AlgebraicRootRationalMap::new(
            &root.polynomial_coefficients,
            &numerator,
            &[Real::one()],
            PredicatePolicy::STRICT,
        );

        let report = map.transform(&root);

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert_eq!(
            report
                .representation
                .as_ref()
                .and_then(AlgebraicRootRepresentation::exact_point_witness),
            Some(&real(2))
        );
        let direct_map = map
            .direct_map
            .get()
            .and_then(Option::as_ref)
            .expect("the retained direct map was initialized");
        assert!(direct_map.source_reduced_map.get().is_some());
        assert!(map.resultant_polynomial.get().is_none());
    }

    #[test]
    fn rational_image_refines_away_an_excluded_endpoint_pole() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one()],
            &[real(-1), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.representation.as_ref().unwrap().is_valid());
        assert!(report.numerator_image.is_none());
    }

    #[test]
    fn rational_image_refines_a_dependency_broadened_denominator_interval() {
        // x^2 - 2x + 2 is strictly positive, but ordinary interval Horner on
        // [1, 2] produces [0, 2]. Exact coprimality proves sqrt(2) is not a
        // pole, after which bounded source refinement separates the range.
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one()],
            &[real(2), real(-2), Real::one()],
            PredicatePolicy::STRICT,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert_eq!(
            report.evaluation.status,
            AlgebraicRootRationalEvaluationStatus::IntervalEvaluated
        );
        assert!(report.representation.as_ref().unwrap().is_valid());
    }

    #[test]
    fn rational_image_certifies_an_algebraic_pole_before_cancellation() {
        for numerator in [vec![Real::one()], vec![real(-2), Real::zero(), Real::one()]] {
            let report = transform_algebraic_root_rational_image(
                &sqrt_two_positive(),
                &numerator,
                &[real(-2), Real::zero(), Real::one()],
                PredicatePolicy::STRICT,
            );

            assert_eq!(
                report.status,
                AlgebraicRootRationalImageStatus::CertifiedZeroDenominator
            );
            assert_eq!(
                report.evaluation.status,
                AlgebraicRootRationalEvaluationStatus::CertifiedZeroDenominator
            );
            assert!(report.representation.is_none());
            assert!(report.numerator_image.is_none());
        }
    }

    #[test]
    fn rational_image_direct_resultant_accepts_nonmonotone_denominator_image() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one(), Real::one()],
            &[real(10), real(-3), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.numerator_image.is_none());
        assert!(report.denominator_image.is_none());
        assert!(report.representation.as_ref().unwrap().is_valid());
    }

    #[test]
    fn rational_image_supports_degree_twelve_source_with_cubic_map() {
        let mut polynomial_coefficients = vec![Real::zero(); 13];
        polynomial_coefficients[0] = real(-64);
        polynomial_coefficients[12] = Real::one();
        let represented = AlgebraicRootRepresentation {
            polynomial_coefficients,
            ..sqrt_two_positive()
        };
        let report = transform_algebraic_root_rational_image(
            &represented,
            &[Real::zero(), Real::one(), Real::zero(), Real::one()],
            &[real(2), Real::one(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.representation.as_ref().unwrap().is_valid());
    }

    #[test]
    fn rational_image_clears_disparate_denominators_before_elimination() {
        let represented = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![fraction(-2, 3), Real::zero(), fraction(1, 3)],
            ..sqrt_two_positive()
        };
        let numerator = [fraction(1, 5), fraction(1, 7)];
        let denominator = [fraction(2, 11), fraction(1, 13)];
        let report = transform_algebraic_root_rational_image(
            &represented,
            &numerator,
            &denominator,
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert!(image.is_valid());
        assert_eq!(
            image.interval.lower,
            evaluate_rational_polynomial(
                &numerator,
                &denominator,
                &real(1),
                PredicatePolicy::STRICT,
            )
            .unwrap()
        );
        assert_eq!(
            image.interval.upper,
            evaluate_rational_polynomial(
                &numerator,
                &denominator,
                &real(2),
                PredicatePolicy::STRICT,
            )
            .unwrap()
        );
        assert!(image.polynomial_coefficients.iter().all(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_some_and(Rational::is_integer)
        }));
    }

    #[test]
    fn rational_image_norm_keeps_a_shared_scale_when_sample_degree_drops() {
        let represented = AlgebraicRootRepresentation {
            polynomial_coefficients: vec![real(-4), Real::zero(), real(2)],
            ..sqrt_two_positive()
        };
        let report = transform_algebraic_root_rational_image(
            &represented,
            &[Real::zero(), Real::one(), Real::one()],
            &[Real::one(), Real::zero(), Real::one()],
            PredicatePolicy::APPROXIMATE_512,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert_eq!(
            image.polynomial_coefficients,
            vec![real(2), real(-12), real(9)]
        );
        assert!(image.is_valid());
    }

    #[test]
    fn rational_image_refines_a_nonmonotone_source_interval() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &sqrt_two_positive(),
                &[Real::one()],
                &[real(10), real(-3), Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            assert!(report.numerator_image.is_none());
            assert!(report.denominator_image.is_none());
            assert!(report.representation.as_ref().unwrap().is_valid());
        }
    }

    #[test]
    fn rational_image_represents_a_stationary_algebraic_map() {
        // At alpha = sqrt(2), q(alpha) = alpha^3 - 6 alpha = -4 alpha and
        // q'(alpha) = 0. Endpoint mapping is therefore insufficient even
        // though the exact image is an ordinary isolated algebraic number.
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &sqrt_two_positive(),
                &[Real::zero(), real(-6), Real::zero(), Real::one()],
                &[Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let image = report.representation.as_ref().unwrap();
            assert!(image.is_valid());
            assert_eq!(
                image.polynomial_coefficients,
                vec![real(-32), Real::zero(), Real::one()]
            );
            assert_eq!(
                compare_reals(&image.interval.upper, &Real::zero(), policy).value(),
                Some(Ordering::Less)
            );
            assert_eq!(
                compare_reals(&image.interval.lower, &real(-5), policy).value(),
                Some(Ordering::Less)
            );
        }
    }

    #[test]
    fn rational_image_refines_away_a_foreign_resultant_root() {
        // P also has the root 3. For q(x) = x^2 - 4x, q(3) = -3 lies in the
        // initial endpoint image [-4, -3] of the selected sqrt(2) interval.
        // The selected image may claim unit isolation only after exact source
        // refinement excludes that distinct foreign resultant root.
        let mut selected = sqrt_two_positive();
        selected.polynomial_coefficients = vec![real(6), real(-2), real(-3), Real::one()];
        selected.validation =
            validate_algebraic_root_representation(&selected, PredicatePolicy::STRICT);
        assert!(selected.is_valid());

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &selected,
                &[Real::zero(), real(-4), Real::one()],
                &[Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let image = report.representation.as_ref().unwrap();
            assert!(image.is_valid());
            assert_eq!(
                compare_reals(&image.interval.upper, &real(-3), policy).value(),
                Some(Ordering::Less)
            );
        }
    }

    #[test]
    fn rational_image_preserves_a_refinement_discovered_rational_root() {
        let mut selected = sqrt_two_positive();
        selected.polynomial_coefficients = vec![real(6), real(-2), real(-3), Real::one()];
        selected.interval = IsolatedRootInterval {
            lower: real(2),
            upper: real(4),
            exact_root: None,
            distinct_root_count: 1,
        };
        selected.validation =
            validate_algebraic_root_representation(&selected, PredicatePolicy::STRICT);
        assert!(selected.is_valid());

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let report = transform_algebraic_root_rational_image(
                &selected,
                &[Real::zero(), real(-4), Real::one()],
                &[Real::one()],
                policy,
            );

            assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            assert_eq!(
                report
                    .representation
                    .as_ref()
                    .unwrap()
                    .exact_point_witness(),
                Some(&real(-3))
            );
        }
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
                ..sqrt_two_positive()
            };

            let report = transform_algebraic_root_rational_image(
                &represented,
                &[real(nc), real(nl)],
                &[real(dc), real(dl)],
                PredicatePolicy::APPROXIMATE_512,
            );

            prop_assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
            let expected = (real(nc + nl * root) / real(dc + dl * root)).unwrap();
            prop_assert_eq!(
                report.representation.as_ref().unwrap().exact_point_witness(),
                Some(&expected)
            );
        }

        #[test]
        fn generated_shared_factors_preserve_the_selected_rational_image(
            factor_root in -3_i8..=1,
            numerator_constant in -5_i8..=5,
            numerator_linear in prop::sample::select(vec![-4_i8, -3, -2, -1, 1, 2, 3, 4]),
            denominator_constant in 1_i8..=5,
        ) {
            let factor = [real(-i64::from(factor_root)), Real::one()];
            let reduced_numerator = [
                real(i64::from(numerator_constant)),
                real(i64::from(numerator_linear)),
            ];
            let reduced_denominator = [real(i64::from(denominator_constant))];
            let authored_numerator = polynomial_mul(&factor, &reduced_numerator);
            let authored_denominator = polynomial_mul(&factor, &reduced_denominator);

            let authored = transform_algebraic_root_rational_image(
                &sqrt_two_positive(),
                &authored_numerator,
                &authored_denominator,
                PredicatePolicy::STRICT,
            );
            let reduced = transform_algebraic_root_rational_image(
                &sqrt_two_positive(),
                &reduced_numerator,
                &reduced_denominator,
                PredicatePolicy::STRICT,
            );
            prop_assert_eq!(
                &authored.status,
                &AlgebraicRootRationalImageStatus::Transformed
            );
            prop_assert_eq!(
                &reduced.status,
                &AlgebraicRootRationalImageStatus::Transformed
            );
            let authored = authored.representation.as_ref().unwrap();
            let reduced = reduced.representation.as_ref().unwrap();
            prop_assert_eq!(
                &authored.polynomial_coefficients,
                &reduced.polynomial_coefficients
            );
            prop_assert!(authored.is_valid());
            prop_assert!(reduced.is_valid());
            prop_assert_ne!(
                compare_reals(
                    &authored.interval.lower,
                    &reduced.interval.upper,
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(Ordering::Greater)
            );
            prop_assert_ne!(
                compare_reals(
                    &reduced.interval.lower,
                    &authored.interval.upper,
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(Ordering::Greater)
            );
        }

        #[test]
        fn generated_rational_image_batches_match_singleton_order(
            denominator_constant in 1_i8..=5,
            first in (-4_i8..=4, -4_i8..=4),
            second in (-4_i8..=4, -4_i8..=4),
            third in (-4_i8..=4, -4_i8..=4),
        ) {
            let denominator = [real(i64::from(denominator_constant)), Real::one()];
            let numerators = [
                [real(i64::from(first.0)), real(i64::from(first.1))],
                [real(i64::from(second.0)), real(i64::from(second.1))],
                [real(i64::from(third.0)), real(i64::from(third.1))],
            ];
            let batch = transform_algebraic_root_rational_images(
                &sqrt_two_positive(),
                [&numerators[0], &numerators[1], &numerators[2]],
                &denominator,
                PredicatePolicy::STRICT,
            );

            for (report, numerator) in batch.into_iter().zip(numerators) {
                prop_assert_eq!(
                    report,
                    transform_algebraic_root_rational_image(
                        &sqrt_two_positive(),
                        &numerator,
                        &denominator,
                        PredicatePolicy::STRICT,
                    )
                );
            }
        }
    }
}
