//! Exact represented images of correlated selected-root tensors.
//!
//! Binary algebraic arithmetic is useful for genuinely independent values,
//! but repeatedly applying it to coordinates that share selected source roots
//! loses correlation and multiplies eliminant degrees.  This module keeps the
//! complete relation as one dense tensor, eliminates each already selected
//! source axis exactly, square-frees only the final univariate eliminant, and
//! selects the authored image with an exact isolating interval.
//!
//! Construction is always `STRICT`.  `APPROXIMATE_512` is a terminal equality
//! policy and is deliberately absent from this API.

use hyperlimit::PredicatePolicy;
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, validate_algebraic_root_representation,
};
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval, square_free_part,
};
use crate::tensor_resultant::{
    DenseTensorPolynomial, TensorConstraintResultantReport, TensorConstraintResultantStatus,
    resultant_tensor_polynomial_univariate_constraint,
};

/// Status for constructing one correlated tensor image.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AlgebraicTensorImageStatus {
    /// The image has an exact final polynomial and singleton isolator.
    Transformed,
    /// One selected source root failed STRICT structural validation.
    InvalidSourceEvidence,
    /// Tensor rank did not equal `source_roots.len() + 1`.
    InvalidRelationShape,
    /// A selected source constraint could not be square-freed exactly.
    SourceSquareFreeFailed,
    /// One exact constrained-axis resultant failed.
    EliminationFailed,
    /// The final image eliminant was constant or could not be square-freed.
    ImageSquareFreeFailed,
    /// The supplied exact image interval did not isolate one distinct root.
    NonIsolatingImageInterval,
    /// Root-count or coefficient predicates remained undecided under STRICT.
    Undecided,
    /// The final represented-root payload failed STRICT validation.
    InvalidTransformedEvidence,
}

/// Exact construction report for a correlated selected-root tensor image.
#[derive(Clone, Debug, PartialEq)]
pub struct AlgebraicTensorImageReport {
    /// Final construction status.
    pub status: AlgebraicTensorImageStatus,
    /// Number of source axes eliminated successfully.
    pub elimination_count: usize,
    /// Failed elimination evidence, retained only on that failure path.
    pub failed_elimination: Option<TensorConstraintResultantReport>,
    /// Resulting represented image, or failed final validation evidence.
    pub representation: Option<AlgebraicRootRepresentation>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Constructs one exact represented value from a correlated tensor relation.
///
/// The final tensor axis is the image variable. Every preceding axis matches
/// the source root at the same index. `image_interval` must enclose the
/// authored value; it is accepted only after STRICT Sturm replay proves that
/// the square-free final eliminant has exactly one distinct root there.
pub fn represent_algebraic_tensor_image(
    relation: &DenseTensorPolynomial,
    source_roots: &[AlgebraicRootRepresentation],
    image_interval: &IsolatedRootInterval,
) -> AlgebraicTensorImageReport {
    if relation.dimensions().len() != source_roots.len() + 1 || source_roots.is_empty() {
        return report(
            AlgebraicTensorImageStatus::InvalidRelationShape,
            0,
            None,
            None,
            "a correlated tensor image requires one source root per non-output axis",
        );
    }
    for source in source_roots {
        if !source.is_valid()
            || validate_algebraic_root_representation(source, PredicatePolicy::STRICT).status
                != AlgebraicRootValidationStatus::Valid
        {
            return report(
                AlgebraicTensorImageStatus::InvalidSourceEvidence,
                0,
                None,
                None,
                "tensor-image source evidence did not validate under STRICT",
            );
        }
    }

    let original_source_count = source_roots.len();
    let mut source_roots = source_roots.to_vec();
    let mut relation = relation.clone();
    let mut source_index = 0;
    while source_index < source_roots.len() {
        let duplicate = source_roots[..source_index]
            .iter()
            .position(|source| source == &source_roots[source_index]);
        if let Some(retained_index) = duplicate {
            let Some(diagonal) = relation.substitute_equal_axes(retained_index, source_index)
            else {
                return report(
                    AlgebraicTensorImageStatus::InvalidRelationShape,
                    0,
                    None,
                    None,
                    "duplicate tensor-image source axes could not be identified exactly",
                );
            };
            relation = diagonal;
            source_roots.remove(source_index);
        } else {
            source_index += 1;
        }
    }

    let mut constraints = Vec::with_capacity(source_roots.len());
    for (source_index, source) in source_roots.iter().enumerate() {
        let Some(constraint) = square_free_part(
            source.polynomial_coefficients.clone(),
            PredicatePolicy::STRICT,
        ) else {
            return report(
                AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                source_index,
                None,
                None,
                "a tensor-image source constraint could not be square-freed exactly",
            );
        };
        constraints.push(canonicalize_proven_rational_coefficients(constraint));
    }

    // Keep every selected source axis canonical from the outset. Repeating
    // this reduction for every still-live source after each resultant avoids
    // degree growth in powers already implied by those source constraints.
    for (axis, constraint) in constraints.iter().enumerate() {
        let Some(reduced) = relation.reduce_axis_modulo(axis, constraint, PredicatePolicy::STRICT)
        else {
            return report(
                AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                axis,
                None,
                None,
                "a tensor-image source axis could not be reduced in its exact quotient ring",
            );
        };
        relation = reduced;
    }
    relation = canonicalize_proven_rational_tensor(relation);

    for (source_index, constraint) in constraints.iter().enumerate() {
        let elimination = resultant_tensor_polynomial_univariate_constraint(
            &relation,
            constraint,
            0,
            PredicatePolicy::MAX_REFINEMENT_PRECISION,
        );
        if elimination.status != TensorConstraintResultantStatus::Constructed {
            let message = elimination
                .message
                .clone()
                .unwrap_or_else(|| "one tensor-image constrained-axis resultant failed".to_owned());
            return AlgebraicTensorImageReport {
                status: AlgebraicTensorImageStatus::EliminationFailed,
                elimination_count: source_index,
                failed_elimination: Some(elimination),
                representation: None,
                message: Some(message),
            };
        }
        relation = elimination
            .resultant
            .expect("a constructed tensor resultant retains its polynomial");
        relation = canonicalize_proven_rational_tensor(relation);
        for (axis, remaining_constraint) in constraints.iter().skip(source_index + 1).enumerate() {
            let Some(reduced) =
                relation.reduce_axis_modulo(axis, remaining_constraint, PredicatePolicy::STRICT)
            else {
                return report(
                    AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                    source_index + axis + 1,
                    None,
                    None,
                    "a remaining tensor-image source axis could not be reduced in its exact quotient ring",
                );
            };
            relation = reduced;
        }
        relation = canonicalize_proven_rational_tensor(relation);
    }
    if relation.dimensions().len() != 1 {
        return report(
            AlgebraicTensorImageStatus::InvalidRelationShape,
            original_source_count,
            None,
            None,
            "tensor-image elimination did not leave one output axis",
        );
    }
    let Some(polynomial_coefficients) =
        square_free_part(relation.coefficients().to_vec(), PredicatePolicy::STRICT)
    else {
        return report(
            AlgebraicTensorImageStatus::ImageSquareFreeFailed,
            original_source_count,
            None,
            None,
            "the final tensor-image eliminant could not be square-freed exactly",
        );
    };
    // Resultant interpolation can retain a large arithmetic DAG even when a
    // coefficient is exactly rational. Collapse only values whose bounded
    // symbolic normal form proves that fact before Sturm replay; non-rational
    // canonical `Real` coefficient fields remain untouched.
    let polynomial_coefficients =
        canonicalize_proven_rational_coefficients(polynomial_coefficients);
    if polynomial_coefficients.len() <= 1 {
        return report(
            AlgebraicTensorImageStatus::ImageSquareFreeFailed,
            original_source_count,
            None,
            None,
            "the final tensor-image eliminant was constant",
        );
    }

    let refinement = refine_isolated_univariate_polynomial_interval(
        &polynomial_coefficients,
        image_interval,
        RootIsolationConfig {
            policy: PredicatePolicy::STRICT,
            max_interval_width: None,
            max_refinement_steps: 4,
        },
    );
    let Some(interval) = refinement.refined_interval else {
        let status = match refinement.status {
            IsolatedRootRefinementStatus::NonUnitIsolation => {
                AlgebraicTensorImageStatus::NonIsolatingImageInterval
            }
            IsolatedRootRefinementStatus::InvalidPolynomial
            | IsolatedRootRefinementStatus::InvalidInterval => {
                AlgebraicTensorImageStatus::InvalidTransformedEvidence
            }
            IsolatedRootRefinementStatus::Undecided
            | IsolatedRootRefinementStatus::Refined
            | IsolatedRootRefinementStatus::ExactRoot => AlgebraicTensorImageStatus::Undecided,
        };
        return AlgebraicTensorImageReport {
            status,
            elimination_count: original_source_count,
            failed_elimination: None,
            representation: None,
            message: refinement.message,
        };
    };
    let first_source = &source_roots[0];
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: first_source.constraint_index,
        symbol: first_source.symbol,
        interval_index: first_source.interval_index,
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
    representation.validation =
        validate_algebraic_root_representation(&representation, PredicatePolicy::STRICT);
    if !representation.is_valid() {
        return AlgebraicTensorImageReport {
            status: AlgebraicTensorImageStatus::InvalidTransformedEvidence,
            elimination_count: original_source_count,
            failed_elimination: None,
            representation: Some(representation),
            message: Some("tensor-image representation did not validate under STRICT".to_owned()),
        };
    }
    AlgebraicTensorImageReport {
        status: AlgebraicTensorImageStatus::Transformed,
        elimination_count: original_source_count,
        failed_elimination: None,
        representation: Some(representation),
        message: None,
    }
}

fn canonicalize_proven_rational_coefficients(coefficients: Vec<Real>) -> Vec<Real> {
    coefficients
        .into_iter()
        .map(|coefficient| {
            coefficient
                .exact_rational_normal_form()
                .map(Real::new)
                .unwrap_or(coefficient)
        })
        .collect()
}

fn canonicalize_proven_rational_tensor(polynomial: DenseTensorPolynomial) -> DenseTensorPolynomial {
    let dimensions = polynomial.dimensions().to_vec();
    let coefficients =
        canonicalize_proven_rational_coefficients(polynomial.coefficients().to_vec());
    DenseTensorPolynomial::try_new(dimensions, coefficients)
        .expect("canonicalizing tensor coefficients preserves its validated shape")
}

fn report(
    status: AlgebraicTensorImageStatus,
    elimination_count: usize,
    failed_elimination: Option<TensorConstraintResultantReport>,
    representation: Option<AlgebraicRootRepresentation>,
    message: impl Into<String>,
) -> AlgebraicTensorImageReport {
    AlgebraicTensorImageReport {
        status,
        elimination_count,
        failed_elimination,
        representation,
        message: Some(message.into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperreal::Real;

    use crate::algebraic::{
        AlgebraicRootKind, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
        represented_root_sign,
    };
    use crate::symbolic::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn square_root(square: i64) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: square as usize,
            symbol: SymbolId(square as u32),
            interval_index: 0,
            polynomial_coefficients: vec![-real(square), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: real(square),
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

    fn flat_index(dimensions: &[usize], exponents: &[usize]) -> usize {
        dimensions
            .iter()
            .zip(exponents)
            .fold(0, |index, (dimension, exponent)| {
                index * dimension + exponent
            })
    }

    fn sum_relation(count: usize, constant: Real) -> DenseTensorPolynomial {
        let dimensions = vec![2; count + 1];
        let mut coefficients = vec![Real::zero(); dimensions.iter().product()];
        coefficients[0] = -constant;
        for axis in 0..count {
            let mut exponent = vec![0; count + 1];
            exponent[axis] = 1;
            coefficients[flat_index(&dimensions, &exponent)] = -Real::one();
        }
        let mut exponent = vec![0; count + 1];
        exponent[count] = 1;
        coefficients[flat_index(&dimensions, &exponent)] = Real::one();
        DenseTensorPolynomial::try_new(dimensions, coefficients).unwrap()
    }

    #[test]
    fn tensor_image_represents_four_correlated_selected_roots() {
        let sources = [2_i64, 3, 5, 7].map(square_root);
        let report = represent_algebraic_tensor_image(
            &sum_relation(4, Real::zero()),
            &sources,
            &IsolatedRootInterval {
                lower: real(8),
                upper: real(9),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(report.elimination_count, 4);
        let representation = report.representation.unwrap();
        assert_eq!(representation.polynomial_coefficients.len(), 17);
        assert!(
            representation
                .polynomial_coefficients
                .iter()
                .all(|coefficient| coefficient.exact_rational().is_some())
        );
        assert_eq!(
            represented_root_sign(&representation, PredicatePolicy::STRICT),
            Some(std::cmp::Ordering::Greater)
        );
    }

    #[test]
    fn tensor_image_retains_exact_real_coefficient_fields() {
        let sqrt_eleven = real(11).sqrt().expect("positive exact square root");
        let report = represent_algebraic_tensor_image(
            &sum_relation(1, sqrt_eleven.clone()),
            &[square_root(2)],
            &IsolatedRootInterval {
                lower: real(4),
                upper: real(5),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        let actual = report.representation.unwrap().polynomial_coefficients;
        let expected = vec![real(9), real(-2) * sqrt_eleven, Real::one()];
        assert_eq!(actual.len(), expected.len());
        assert!(actual.iter().zip(expected).all(|(actual, expected)| {
            hyperlimit::compare_reals(actual, &expected, PredicatePolicy::STRICT).value()
                == Some(std::cmp::Ordering::Equal)
        }));
    }

    #[test]
    fn tensor_image_identifies_duplicate_selected_source_axes_exactly() {
        let source = square_root(2);
        let report = represent_algebraic_tensor_image(
            &sum_relation(2, Real::zero()),
            &[source.clone(), source],
            &IsolatedRootInterval {
                lower: real(2),
                upper: real(3),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(report.elimination_count, 2);
        assert_eq!(
            report.representation.unwrap().polynomial_coefficients,
            vec![real(-8), Real::zero(), Real::one()]
        );
    }

    #[test]
    fn tensor_image_rejects_nonisolating_and_misshaped_evidence() {
        let source = square_root(2);
        let relation = sum_relation(1, Real::zero());
        assert_eq!(
            represent_algebraic_tensor_image(
                &relation,
                std::slice::from_ref(&source),
                &IsolatedRootInterval {
                    lower: real(-2),
                    upper: real(2),
                    exact_root: None,
                    distinct_root_count: 1,
                },
            )
            .status,
            AlgebraicTensorImageStatus::NonIsolatingImageInterval
        );
        assert_eq!(
            represent_algebraic_tensor_image(&relation, &[], &source.interval).status,
            AlgebraicTensorImageStatus::InvalidRelationShape
        );
    }
}
