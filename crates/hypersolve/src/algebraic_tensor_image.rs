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

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::algebraic::{
    AlgebraicRootAffineRelation, AlgebraicRootKind, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, algebraic_root_affine_relation,
    validate_algebraic_root_representation,
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

/// Reduces a selected root to a rational or quadratic `x^2-q` factor when a
/// bounded rational proposal is proved by exact polynomial divisibility.
///
/// This is an explicit recursive-frame optimization, not a topology
/// predicate. Approximation only proposes `q`; exact division proves the
/// source eliminant contains the factor, and STRICT validation proves the
/// existing selected interval isolates one root of that factor.
pub fn compact_algebraic_root_low_degree_witness(
    root: &AlgebraicRootRepresentation,
) -> Option<AlgebraicRootRepresentation> {
    if !root.is_valid()
        || validate_algebraic_root_representation(root, PredicatePolicy::STRICT).status
            != AlgebraicRootValidationStatus::Valid
    {
        return None;
    }
    if let Some(witness) = exact_bounded_denominator_root_in_interval(
        &root.polynomial_coefficients,
        &root.interval,
        64,
    ) {
        let mut compact = root.clone();
        compact.polynomial_coefficients = vec![-witness.clone(), Real::one()];
        compact.kind = AlgebraicRootKind::ExactRationalWitness;
        compact.interval = IsolatedRootInterval {
            lower: witness.clone(),
            upper: witness.clone(),
            exact_root: Some(witness),
            distinct_root_count: 1,
        };
        compact.validation =
            validate_algebraic_root_representation(&compact, PredicatePolicy::STRICT);
        if compact.is_valid() {
            return Some(compact);
        }
    }
    let lower = root.interval.lower.to_f64_lossy()?;
    let upper = root.interval.upper.to_f64_lossy()?;
    if !lower.is_finite() || !upper.is_finite() {
        return None;
    }
    let midpoint = lower / 2.0 + upper / 2.0;
    let square = midpoint * midpoint;
    for denominator in 1_i64..=64 {
        let numerator = (square * denominator as f64).round();
        if !numerator.is_finite() || numerator < 0.0 || numerator > i64::MAX as f64 {
            continue;
        }
        let rational = hyperreal::Rational::fraction(numerator as i64, denominator as u64).ok()?;
        let factor = vec![-Real::new(rational), Real::zero(), Real::one()];
        if !exact_polynomial_divides(&root.polynomial_coefficients, &factor) {
            continue;
        }
        let mut compact = root.clone();
        compact.polynomial_coefficients = factor;
        compact.validation =
            validate_algebraic_root_representation(&compact, PredicatePolicy::STRICT);
        if compact.is_valid() {
            return Some(compact);
        }
    }
    None
}

fn exact_polynomial_divides(polynomial: &[Real], factor: &[Real]) -> bool {
    if polynomial.len() < factor.len() || factor.last() != Some(&Real::one()) {
        return false;
    }
    let factor_degree = factor.len() - 1;
    let mut remainder = polynomial.to_vec();
    for degree in (factor_degree..remainder.len()).rev() {
        let coefficient = remainder[degree].clone();
        for (factor_degree_index, factor_coefficient) in factor.iter().enumerate() {
            let index = degree - factor_degree + factor_degree_index;
            remainder[index] = remainder[index].clone() - &coefficient * factor_coefficient;
        }
    }
    remainder[..factor_degree].iter().all(|coefficient| {
        coefficient
            .exact_rational_ref()
            .is_some_and(|coefficient| coefficient.is_zero())
    })
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
        let affine =
            source_roots[..source_index]
                .iter()
                .enumerate()
                .find_map(|(retained_index, source)| {
                    let affine = if source == &source_roots[source_index] {
                        Some(AlgebraicRootAffineRelation {
                            scale: Real::one(),
                            offset: Real::zero(),
                        })
                    } else {
                        algebraic_root_affine_relation(
                            source,
                            &source_roots[source_index],
                            PredicatePolicy::STRICT,
                        )
                    }?;
                    Some((retained_index, affine))
                });
        if let Some((retained_index, affine)) = affine {
            let Some(diagonal) = relation.substitute_affine_axis(
                retained_index,
                source_index,
                &affine.scale,
                &affine.offset,
            ) else {
                return report(
                    AlgebraicTensorImageStatus::InvalidRelationShape,
                    0,
                    None,
                    None,
                    "affine-related tensor-image source axes could not be collapsed exactly",
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
        relation = if let Some(independent) =
            relation.remove_certified_independent_axis(0, PredicatePolicy::MAX_REFINEMENT_PRECISION)
        {
            // A validated selected source exists, but this relation no longer
            // depends on it. Removing the axis preserves the represented zero
            // set and avoids raising the image polynomial to the source
            // constraint degree merely to square-free that multiplicity later.
            independent
        } else {
            let elimination = resultant_tensor_polynomial_univariate_constraint(
                &relation,
                constraint,
                0,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            );
            if elimination.status != TensorConstraintResultantStatus::Constructed {
                let message = elimination.message.clone().unwrap_or_else(|| {
                    "one tensor-image constrained-axis resultant failed".to_owned()
                });
                return AlgebraicTensorImageReport {
                    status: AlgebraicTensorImageStatus::EliminationFailed,
                    elimination_count: source_index,
                    failed_elimination: Some(elimination),
                    representation: None,
                    message: Some(message),
                };
            }
            elimination
                .resultant
                .expect("a constructed tensor resultant retains its polynomial")
        };
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

    let exact_linear_root = if polynomial_coefficients.len() == 2 {
        (-polynomial_coefficients[0].clone() / polynomial_coefficients[1].clone())
            .ok()
            .and_then(|root| root.exact_rational_normal_form().map(Real::new))
            .filter(|root| {
                let exact_witness_matches =
                    image_interval.exact_root.as_ref().is_none_or(|exact_root| {
                        compare_reals(exact_root, root, PredicatePolicy::STRICT).value()
                            == Some(std::cmp::Ordering::Equal)
                    });
                exact_witness_matches
                    && matches!(
                        compare_reals(&image_interval.lower, root, PredicatePolicy::STRICT).value(),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                    )
                    && matches!(
                        compare_reals(root, &image_interval.upper, PredicatePolicy::STRICT).value(),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                    )
            })
    } else {
        None
    };
    // A nonlinear eliminant can retain exact cardinal roots from conjugate
    // source tuples that are not the authored tuple. Interval containment and
    // exact polynomial incidence alone therefore do not identify the image;
    // the general isolation path below must first prove that the supplied
    // image enclosure contains exactly one distinct eliminant root. A linear
    // eliminant is the sole safe shortcut because it has one global root.
    let exact_rational_root = exact_linear_root;
    let interval = if let Some(root) = &exact_rational_root {
        IsolatedRootInterval {
            lower: root.clone(),
            upper: root.clone(),
            exact_root: Some(root.clone()),
            distinct_root_count: 1,
        }
    } else {
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
        interval
    };
    let first_source = &source_roots[0];
    let polynomial_coefficients =
        exact_rational_root.map_or(polynomial_coefficients, |root| vec![-root, Real::one()]);
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

/// Recovers a selected cardinal image without using approximation as proof,
/// leaving every other algebraic image on the unchanged exact isolator path.
#[cfg(test)]
fn exact_cardinal_root_in_interval(
    polynomial_coefficients: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Real> {
    // These three coordinates dominate canonical CAD frames and need no
    // approximate proposal or rational-root factorization. Other rational
    // images retain their complete algebraic isolator.
    for candidate in [Real::zero(), Real::from(-1_i8), Real::one()] {
        let value = polynomial_coefficients
            .iter()
            .rev()
            .fold(Real::zero(), |value, coefficient| {
                value * &candidate + coefficient
            });
        if !value
            .exact_rational_ref()
            .is_some_and(|value| value.is_zero())
        {
            continue;
        }
        if matches!(
            compare_reals(&interval.lower, &candidate, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) && matches!(
            compare_reals(&candidate, &interval.upper, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            return Some(candidate);
        }
    }
    None
}

fn exact_bounded_denominator_root_in_interval(
    polynomial_coefficients: &[Real],
    interval: &IsolatedRootInterval,
    max_denominator: i64,
) -> Option<Real> {
    let lower = interval.lower.to_f64_lossy()?;
    let upper = interval.upper.to_f64_lossy()?;
    if !lower.is_finite() || !upper.is_finite() {
        return None;
    }
    let midpoint = lower / 2.0 + upper / 2.0;
    for denominator in 1..=max_denominator {
        let numerator = (midpoint * denominator as f64).round();
        if !numerator.is_finite() || numerator < i64::MIN as f64 || numerator > i64::MAX as f64 {
            continue;
        }
        let rational = hyperreal::Rational::fraction(numerator as i64, denominator as u64).ok()?;
        let candidate = Real::new(rational);
        if candidate
            .to_f64_lossy()
            .is_none_or(|candidate| candidate < lower || candidate > upper)
        {
            continue;
        }
        if !matches!(
            compare_reals(&interval.lower, &candidate, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) || !matches!(
            compare_reals(&candidate, &interval.upper, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            continue;
        }
        let value = polynomial_coefficients
            .iter()
            .rev()
            .fold(Real::zero(), |value, coefficient| {
                value * &candidate + coefficient
            });
        if value
            .exact_rational_ref()
            .is_some_and(|value| value.is_zero())
        {
            return Some(candidate);
        }
    }
    None
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
        AlgebraicRootAffineTransformStatus, AlgebraicRootKind, AlgebraicRootValidationReport,
        AlgebraicRootValidationStatus, represented_root_sign, transform_algebraic_root_affine,
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

    #[test]
    fn exact_small_rational_image_candidates_require_exact_replay() {
        let tenth = (Real::one() / real(10)).unwrap();
        let interval = IsolatedRootInterval {
            lower: -tenth.clone(),
            upper: tenth,
            exact_root: None,
            distinct_root_count: 1,
        };
        // z(z^2-2) has the selected rational root zero inside the isolator.
        assert_eq!(
            exact_cardinal_root_in_interval(
                &[Real::zero(), real(-2), Real::zero(), Real::one()],
                &interval,
            ),
            Some(Real::zero()),
        );
        // The same candidate hint must not be accepted without exact
        // polynomial incidence.
        assert_eq!(
            exact_cardinal_root_in_interval(
                &[Real::one(), real(-2), Real::zero(), Real::one()],
                &interval,
            ),
            None,
        );
    }

    #[test]
    fn selected_root_rational_compaction_requires_exact_replay() {
        let half = (Real::one() / real(2)).unwrap();
        let eighth = (Real::one() / real(8)).unwrap();
        let root = AlgebraicRootRepresentation {
            constraint_index: 7,
            symbol: SymbolId(11),
            interval_index: 0,
            polynomial_coefficients: vec![real(2), real(-4), real(-1), real(2)],
            interval: IsolatedRootInterval {
                lower: &half - &eighth,
                upper: &half + eighth,
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let compact = compact_algebraic_root_low_degree_witness(&root)
            .expect("the selected half root must compact exactly");
        assert_eq!(compact.interval.exact_root, Some(half.clone()));
        assert_eq!(compact.polynomial_coefficients, vec![-half, Real::one()]);

        let mut nonroot = root;
        nonroot.polynomial_coefficients[0] = real(1);
        assert!(compact_algebraic_root_low_degree_witness(&nonroot).is_none());
    }

    #[test]
    fn selected_root_quadratic_compaction_requires_exact_factor() {
        let three_quarters = (real(3) / real(4)).unwrap();
        let root = AlgebraicRootRepresentation {
            constraint_index: 7,
            symbol: SymbolId(12),
            interval_index: 0,
            polynomial_coefficients: vec![real(3), real(-6), real(-4), real(8)],
            interval: IsolatedRootInterval {
                lower: real(-1),
                upper: -three_quarters.clone(),
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let compact = compact_algebraic_root_low_degree_witness(&root)
            .expect("the selected negative square root must compact exactly");
        assert_eq!(
            compact.polynomial_coefficients,
            vec![-three_quarters, Real::zero(), Real::one()]
        );

        let mut nonfactor = root;
        nonfactor.polynomial_coefficients[0] = real(4);
        assert!(compact_algebraic_root_low_degree_witness(&nonfactor).is_none());
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
    fn tensor_image_collapses_certified_affine_selected_source_axes() {
        let source = square_root(2);
        let shifted =
            transform_algebraic_root_affine(&source, Real::one(), real(3), PredicatePolicy::STRICT);
        assert_eq!(
            shifted.status,
            AlgebraicRootAffineTransformStatus::Transformed
        );
        let report = represent_algebraic_tensor_image(
            &sum_relation(2, Real::zero()),
            &[source, shifted.representation.unwrap()],
            &IsolatedRootInterval {
                lower: real(5),
                upper: real(6),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(report.elimination_count, 2);
        assert_eq!(
            report.representation.unwrap().polynomial_coefficients,
            vec![Real::one(), real(-6), Real::one()]
        );
    }

    #[test]
    fn tensor_image_materializes_an_enclosed_exact_linear_eliminant() {
        let source = square_root(2);
        let shifted =
            transform_algebraic_root_affine(&source, Real::one(), real(3), PredicatePolicy::STRICT)
                .representation
                .unwrap();
        let first = DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
            .unwrap();
        let second =
            DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
                .unwrap();
        let output =
            DenseTensorPolynomial::from_axis_polynomial(3, 2, &[Real::zero(), Real::one()])
                .unwrap();
        let difference = second.subtract(&first).unwrap();
        let relation = output
            .subtract(&difference.multiply(&difference).unwrap())
            .unwrap();
        let report = represent_algebraic_tensor_image(
            &relation,
            &[source, shifted],
            &IsolatedRootInterval {
                lower: Real::zero(),
                upper: real(10),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        let representation = report.representation.unwrap();
        assert_eq!(
            representation.polynomial_coefficients,
            vec![real(-9), Real::one()]
        );
        assert_eq!(representation.interval.exact_root, Some(real(9)));
    }

    #[test]
    fn tensor_image_does_not_select_an_extraneous_cardinal_conjugate_image() {
        let coefficients = [23775_i64, 141480, -110296, -47200, -10000]
            .map(real)
            .to_vec();
        let source = |interval_index, lower: Real, upper: Real| AlgebraicRootRepresentation {
            constraint_index: 17,
            symbol: SymbolId(23),
            interval_index,
            polynomial_coefficients: coefficients.clone(),
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let first = source(
            0,
            (real(-5) / real(16)).unwrap(),
            (real(25) / real(32)).unwrap(),
        );
        let second = source(
            1,
            (real(25) / real(32)).unwrap(),
            (real(15) / real(8)).unwrap(),
        );
        let first_axis =
            DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
                .unwrap();
        let second_axis =
            DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
                .unwrap();
        let output =
            DenseTensorPolynomial::from_axis_polynomial(3, 2, &[Real::zero(), Real::one()])
                .unwrap();
        let difference = first_axis.subtract(&second_axis).unwrap();
        let relation = output
            .subtract(&difference.multiply(&difference).unwrap())
            .unwrap();

        let report = represent_algebraic_tensor_image(
            &relation,
            &[first, second],
            &IsolatedRootInterval {
                lower: Real::zero(),
                upper: real(10),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(
            represented_root_sign(
                &report
                    .representation
                    .expect("the authored nonzero image must be represented"),
                PredicatePolicy::STRICT,
            ),
            Some(std::cmp::Ordering::Greater)
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
