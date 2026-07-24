//! Rational-function images of represented algebraic roots.
//!
//! This module constructs exact evidence for `beta = p(alpha) / q(alpha)`,
//! where `alpha` is a represented real algebraic root and `p`, `q` are
//! exact-rational polynomials.  The result is another
//! [`crate::AlgebraicRootRepresentation`], not a primitive approximation.
//! That distinction is the EGC boundary from the exact-geometric-computation model: construction retains
//! exact replayable objects, and later predicates decide signs/topology from
//! certificates.
//!
//! The preferred path directly eliminates the source coordinate with
//! `Res_x(P(x), p(x) - y*q(x))`.  This is the rational analogue of the
//! polynomial-image construction and avoids requiring `p(alpha)` and
//! `q(alpha)` to be individually monotone.  If that direct path cannot certify
//! a one-root image interval, the implementation falls back to composing the
//! already-certified pieces: numerator and denominator polynomial images plus
//! the bounded binary quotient package.  Both paths keep domain uncertainty
//! report-bearing instead of replacing it with primitive sampling.

use std::cmp::Ordering;
use std::sync::OnceLock;

use hyperlimit::PredicatePolicy;
use hyperlimit::compare_reals_with_policy;
use hyperreal::{Rational, Real};

use crate::algebraic::{
    AlgebraicPolynomialValueInterval, AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport,
    AlgebraicRootArithmeticStatus, AlgebraicRootKind, AlgebraicRootPolynomialEvaluationReport,
    AlgebraicRootRationalEvaluationReport, AlgebraicRootRationalEvaluationStatus,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    arithmetic_algebraic_root_representations, evaluate_rational_expression_at_algebraic_root,
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
};
use crate::resultant::{quotient_ring_resultant_polynomial, resultant_univariate_polynomials};
use crate::root_isolation::IsolatedRootInterval;

const MAX_RATIONAL_IMAGE_SYLVESTER_DIMENSION: usize = 8;

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

/// Reusable rational-image transform for expressions with one denominator.
///
/// Construction evaluates and retains the denominator at the represented
/// source root. Each call to [`Self::transform`] evaluates only its numerator,
/// then follows the same resultant and fallback paths as
/// [`transform_algebraic_root_rational_image`]. This is useful for projective
/// coordinate vectors and derivative vectors whose components share a
/// denominator.
pub struct PreparedAlgebraicRootRationalImage<'a> {
    root: &'a AlgebraicRootRepresentation,
    denominator_coefficients: &'a [Real],
    denominator_evaluation: AlgebraicRootPolynomialEvaluationReport,
    source_polynomial: OnceLock<Option<Vec<Real>>>,
    policy: PredicatePolicy,
}

#[derive(Clone, Copy, Default)]
struct RationalImageRetention<'a> {
    source_polynomial: Option<&'a OnceLock<Option<Vec<Real>>>>,
    resultant_polynomial: Option<&'a OnceLock<Option<Vec<Real>>>>,
}

impl<'a> PreparedAlgebraicRootRationalImage<'a> {
    /// Prepares a shared-denominator transform at `root`.
    pub fn new(
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
            source_polynomial: OnceLock::new(),
            policy,
        }
    }

    /// Constructs exact evidence for one numerator over the retained denominator.
    pub fn transform(&self, numerator_coefficients: &[Real]) -> AlgebraicRootRationalImageReport {
        let evaluation = evaluate_rational_expression_with_denominator_evaluation(
            self.root,
            numerator_coefficients,
            self.denominator_evaluation.clone(),
            self.policy,
        );
        transform_algebraic_root_rational_image_with_evaluation(
            self.root,
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

/// Reusable rational map for several roots of one source polynomial.
///
/// The exact resultant polynomial of a rational map depends on the source
/// polynomial and map coefficients, but not on which real root interval is
/// selected. This carrier retains that elimination lazily while every
/// [`Self::transform`] call still evaluates the denominator, certifies local
/// monotonicity, constructs the image interval, and validates the represented
/// root independently.
pub struct PreparedAlgebraicRootRationalMap {
    source_polynomial_coefficients: Vec<Real>,
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    source_polynomial: OnceLock<Option<Vec<Real>>>,
    resultant_polynomial: OnceLock<Option<Vec<Real>>>,
    policy: PredicatePolicy,
}

impl PreparedAlgebraicRootRationalMap {
    /// Prepares one rational map over a defining source polynomial.
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

    /// Returns whether direct elimination has been attempted and retained.
    pub fn is_resultant_cached(&self) -> bool {
        self.resultant_polynomial.get().is_some()
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
                source_polynomial: Some(&self.source_polynomial),
                resultant_polynomial: Some(&self.resultant_polynomial),
            },
        )
    }
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

fn direct_rational_image_representation(
    root: &AlgebraicRootRepresentation,
    numerator_coefficients: &[Real],
    denominator_coefficients: &[Real],
    policy: PredicatePolicy,
    retention: RationalImageRetention<'_>,
) -> Option<AlgebraicRootRepresentation> {
    if !has_exact_coefficients(&root.polynomial_coefficients) {
        return None;
    }
    let numerator = trim_real_polynomial(numerator_coefficients.to_vec(), policy)?;
    let denominator = trim_real_polynomial(denominator_coefficients.to_vec(), policy)?;
    if !has_exact_coefficients(&numerator) || !has_exact_coefficients(&denominator) {
        return None;
    }
    if let Some(value) = constant_rational_map_value(&numerator, &denominator, policy) {
        return Some(exact_constant_representation(root, value, policy));
    }
    let derivative_numerator = rational_derivative_numerator(&numerator, &denominator, policy)?;
    let derivative_interval = evaluate_interval_polynomial(
        &derivative_numerator,
        &ValueInterval {
            lower: root.interval.lower.clone(),
            upper: root.interval.upper.clone(),
        },
        policy,
    )?;
    if !interval_is_strictly_away_from_zero(&derivative_interval, policy)? {
        return None;
    }

    if numerator.len() <= 2 && denominator.len() <= 2 {
        let mobius = transform_algebraic_root_mobius(
            root,
            numerator.get(1).cloned().unwrap_or_else(Real::zero),
            numerator[0].clone(),
            denominator.get(1).cloned().unwrap_or_else(Real::zero),
            denominator[0].clone(),
            policy,
        );
        if mobius.status == AlgebraicRootMobiusTransformStatus::Transformed
            && let Some(mut representation) = mobius.representation
            && let Some(polynomial) =
                primitive_integer_polynomial(&representation.polynomial_coefficients)
        {
            representation.polynomial_coefficients = polynomial;
            representation.validation =
                validate_algebraic_root_representation(&representation, policy);
            return representation.is_valid().then_some(representation);
        }
    }

    let source_degree = root.polynomial_coefficients.len().checked_sub(1)?;
    let rational_degree = numerator.len().max(denominator.len()).checked_sub(1)?;
    if source_degree + rational_degree.max(1) > MAX_RATIONAL_IMAGE_SYLVESTER_DIMENSION {
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
    let (resultant_numerator, resultant_denominator) =
        clear_rational_map_denominators(&numerator, &denominator)?;
    let polynomial_coefficients = if let Some(retained) = retention.resultant_polynomial {
        retained
            .get_or_init(|| {
                resultant_polynomial_for_rational_image(
                    source_polynomial,
                    &resultant_numerator,
                    &resultant_denominator,
                    source_degree,
                    policy,
                )
            })
            .clone()?
    } else {
        resultant_polynomial_for_rational_image(
            source_polynomial,
            &resultant_numerator,
            &resultant_denominator,
            source_degree,
            policy,
        )?
    };
    let interval = rational_image_interval(&root.interval, &numerator, &denominator, policy)?;
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: root.constraint_index,
        symbol: root.symbol,
        interval_index: root.interval_index,
        polynomial_coefficients,
        interval,
        kind: AlgebraicRootKind::IsolatingInterval,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    if representation.interval.exact_root.is_some() {
        representation.kind = AlgebraicRootKind::ExactRationalWitness;
    }
    representation.validation = validate_algebraic_root_representation(&representation, policy);
    Some(representation)
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
    let mut integers = Rational::primitive_integer_ratio(&rationals)
        .into_iter()
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
    trim_real_polynomial(polynomial, policy)
}

fn rational_image_interval(
    interval: &IsolatedRootInterval,
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<IsolatedRootInterval> {
    let first = evaluate_rational_polynomial(numerator, denominator, &interval.lower)?;
    let second = evaluate_rational_polynomial(numerator, denominator, &interval.upper)?;
    let mut endpoints = [first, second];
    sort_reals_exact(&mut endpoints, policy)?;
    let exact_root = match interval.exact_root.as_ref() {
        Some(root) => Some(evaluate_rational_polynomial(numerator, denominator, root)?),
        None => None,
    };
    Some(IsolatedRootInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
        exact_root,
        distinct_root_count: interval.distinct_root_count,
    })
}

fn evaluate_rational_polynomial(
    numerator: &[Real],
    denominator: &[Real],
    point: &Real,
) -> Option<Real> {
    (evaluate_real_polynomial(numerator, point) / evaluate_real_polynomial(denominator, point)).ok()
}

fn constant_rational_map_value(
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<Real> {
    let max_len = numerator.len().max(denominator.len());
    let mut scale = None;
    for index in 0..max_len {
        let denominator_coefficient = denominator.get(index).cloned().unwrap_or_else(Real::zero);
        if compare_reals_with_policy(&denominator_coefficient, &Real::zero(), policy).value()?
            == Ordering::Equal
        {
            continue;
        }
        let numerator_coefficient = numerator.get(index).cloned().unwrap_or_else(Real::zero);
        scale = Some((numerator_coefficient / denominator_coefficient).ok()?);
        break;
    }
    let scale = scale?;
    let scaled_denominator = polynomial_scale(denominator, &scale);
    let difference = trim_real_polynomial(polynomial_sub(numerator, &scaled_denominator), policy)?;
    (difference.len() == 1
        && compare_reals_with_policy(&difference[0], &Real::zero(), policy).value()?
            == Ordering::Equal)
        .then_some(scale)
}

fn rational_derivative_numerator(
    numerator: &[Real],
    denominator: &[Real],
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
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

fn evaluate_real_polynomial(polynomial: &[Real], point: &Real) -> Real {
    polynomial
        .iter()
        .rev()
        .fold(Real::zero(), |value, coefficient| {
            value * point.clone() + coefficient.clone()
        })
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

fn interval_is_strictly_away_from_zero(
    interval: &ValueInterval,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = compare_reals_with_policy(&interval.lower, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(&interval.upper, &Real::zero(), policy).value()?;
    Some(lower == Ordering::Greater || upper == Ordering::Less)
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

fn has_exact_coefficients(polynomial: &[Real]) -> bool {
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
        AlgebraicRootRationalEvaluationStatus::EvaluatedExactRationalWitness => {
            evaluation.exact_value.as_ref().map(|value| (value, value))
        }
        AlgebraicRootRationalEvaluationStatus::IntervalEvaluated => evaluation
            .interval_value
            .as_ref()
            .map(|interval| (&interval.lower, &interval.upper)),
        _ => None,
    };
    value_interval.is_some_and(|(lower, upper)| {
        compare_reals_with_policy(upper, &target.lower, policy).value() == Some(Ordering::Less)
            || compare_reals_with_policy(lower, &target.upper, policy).value()
                == Some(Ordering::Greater)
    })
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

    fn fraction(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
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
            PredicatePolicy,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let root = report.representation.as_ref().unwrap();
        assert!(root.is_valid());
        assert_eq!(root.interval.lower, (real(1) / real(2)).unwrap());
        assert_eq!(root.interval.upper, (real(2) / real(3)).unwrap());
    }

    #[test]
    fn prepared_rational_images_match_independent_shared_denominator_transforms() {
        let root = sqrt_two_positive();
        let denominator = [real(3), Real::one()];
        let numerators = [
            [real(1), real(2), Real::one()],
            [real(-2), Real::one(), Real::one()],
        ];
        let prepared =
            PreparedAlgebraicRootRationalImage::new(&root, &denominator, PredicatePolicy);

        for numerator in numerators {
            assert_eq!(
                prepared.transform(&numerator),
                transform_algebraic_root_rational_image(
                    &root,
                    &numerator,
                    &denominator,
                    PredicatePolicy,
                )
            );
        }
    }

    #[test]
    fn prepared_rational_map_reuses_elimination_across_source_roots() {
        let positive = sqrt_two_positive();
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
        let map = PreparedAlgebraicRootRationalMap::new(
            &positive.polynomial_coefficients,
            &numerator,
            &denominator,
            PredicatePolicy,
        );
        assert!(!map.is_resultant_cached());

        for root in [&positive, &negative] {
            assert_eq!(
                map.transform(root),
                transform_algebraic_root_rational_image(
                    root,
                    &numerator,
                    &denominator,
                    PredicatePolicy,
                )
            );
            assert!(map.is_resultant_cached());
        }
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
            PredicatePolicy,
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
            PredicatePolicy,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.representation.as_ref().unwrap().is_valid());
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
            PredicatePolicy,
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
            PredicatePolicy,
        );

        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::DenominatorMayContainZero
        );
        assert!(report.representation.is_none());
        assert!(report.numerator_image.is_none());
    }

    #[test]
    fn rational_image_direct_resultant_accepts_nonmonotone_denominator_image() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one(), Real::one()],
            &[real(10), real(-3), Real::one()],
            PredicatePolicy,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        assert!(report.numerator_image.is_none());
        assert!(report.denominator_image.is_none());
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
            PredicatePolicy,
        );

        assert_eq!(report.status, AlgebraicRootRationalImageStatus::Transformed);
        let image = report.representation.as_ref().unwrap();
        assert!(image.is_valid());
        assert_eq!(
            image.interval.lower,
            evaluate_rational_polynomial(&numerator, &denominator, &real(1)).unwrap()
        );
        assert_eq!(
            image.interval.upper,
            evaluate_rational_polynomial(&numerator, &denominator, &real(2)).unwrap()
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
            PredicatePolicy,
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
    fn rational_image_falls_back_to_denominator_image_failure_when_map_is_not_monotone() {
        let report = transform_algebraic_root_rational_image(
            &sqrt_two_positive(),
            &[Real::one()],
            &[real(10), real(-3), Real::one()],
            PredicatePolicy,
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
                PredicatePolicy,
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
