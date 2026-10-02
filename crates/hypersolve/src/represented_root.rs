//! Exact values, intervals, ratios and tensor images built from isolated
//! algebraic roots.
//!
//! Every decision uses certified STRICT predicates. An outcome is either a
//! decided exact value, an unsupported construction (the representation or
//! elimination cannot be built), or an undecided predicate.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign, ZeroKnowledge};

use crate::bivariate_arithmetic::polynomial_derivative;
use crate::radical_expression::TwoSquareRootExpression;
use crate::real_interval::{RealInterval, strict_compare_reals, strict_real_sign};
use crate::tensor_support::*;
use crate::*;

/// Outcome of an exact construction over represented algebraic roots.
#[derive(Clone, Debug, PartialEq)]
pub enum RepresentedOutcome<T> {
    /// The exact value was constructed and certified.
    Decided(T),
    /// The representation or elimination needed for the value is unavailable.
    Unsupported,
    /// A required STRICT predicate was not decided.
    Undecided,
    /// A value required to be nonzero, such as a denominator, is certified zero.
    Vanishes,
}

impl<T> RepresentedOutcome<T> {
    /// Applies `map` to a decided value.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> RepresentedOutcome<U> {
        match self {
            Self::Decided(value) => RepresentedOutcome::Decided(map(value)),
            Self::Unsupported => RepresentedOutcome::Unsupported,
            Self::Undecided => RepresentedOutcome::Undecided,
            Self::Vanishes => RepresentedOutcome::Vanishes,
        }
    }

    /// Returns the same non-decided outcome for another value type, or `None`
    /// when the value is decided.
    pub fn uncertain<U>(self) -> Option<RepresentedOutcome<U>> {
        match self {
            Self::Decided(_) => None,
            Self::Unsupported => Some(RepresentedOutcome::Unsupported),
            Self::Undecided => Some(RepresentedOutcome::Undecided),
            Self::Vanishes => Some(RepresentedOutcome::Vanishes),
        }
    }
}

/// Exact represented coordinate interval over represented algebraic roots.
fn represented_coordinate_interval(lower: &Real, upper: &Real) -> Option<IsolatedRootInterval> {
    match strict_compare_reals(lower, upper)? {
        Ordering::Greater => None,
        Ordering::Equal => Some(IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: Some(lower.clone()),
            distinct_root_count: 1,
        }),
        Ordering::Less => Some(IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: None,
            distinct_root_count: 1,
        }),
    }
}

/// Exact represented univariate coordinate over represented algebraic roots.
pub fn represented_univariate_coordinate(
    coefficients: &[Real],
    lower: &Real,
    upper: &Real,
    provenance: &AlgebraicRootRepresentation,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    if coefficients.len() <= 1 {
        return RepresentedOutcome::Unsupported;
    }
    let Some(interval) = represented_coordinate_interval(lower, upper) else {
        return RepresentedOutcome::Undecided;
    };
    let coefficients = if let Some(root) = interval.exact_root.as_ref() {
        if strict_real_sign(&Real::eval_poly(coefficients, root)) != Some(RealSign::Zero) {
            return RepresentedOutcome::Undecided;
        }
        vec![-root.clone(), Real::one()]
    } else {
        // The exact retained-fiber construction and conservative point box
        // prove that at least one authored coordinate root lies here. A
        // strictly signed derivative enclosure proves that the global image
        // eliminant has at most one root here, completing the singleton proof
        // without a degree-sized Sturm chain. Multiple-root images decline
        // this path and retain the complete global construction fallback.
        let derivative = polynomial_derivative(coefficients);
        let parameter_interval = RealInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        };
        let Some(derivative_bounds) =
            RealInterval::evaluate_power_basis(&derivative, &parameter_interval)
        else {
            return RepresentedOutcome::Undecided;
        };
        let derivative_nonzero = strict_compare_reals(&derivative_bounds.lower, &Real::zero())
            == Some(Ordering::Greater)
            || strict_compare_reals(&derivative_bounds.upper, &Real::zero())
                == Some(Ordering::Less);
        if !derivative_nonzero {
            return RepresentedOutcome::Undecided;
        }
        coefficients.to_vec()
    };
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: provenance.constraint_index,
        symbol: provenance.symbol,
        interval_index: provenance.interval_index,
        polynomial_coefficients: coefficients,
        interval,
        validation: provenance.validation.clone(),
    };
    representation.validation =
        validate_algebraic_root_representation(&representation, crate::PredicatePolicy::STRICT);
    if representation.is_valid() {
        RepresentedOutcome::Decided(representation)
    } else {
        RepresentedOutcome::Unsupported
    }
}

/// Exact represented tensor coordinate over represented algebraic roots.
fn represented_tensor_coordinate(
    relation: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    lower: &Real,
    upper: &Real,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let Some(interval) = represented_coordinate_interval(lower, upper) else {
        return RepresentedOutcome::Undecided;
    };
    if sources.is_empty() {
        if relation.dimensions().len() != 1 {
            return RepresentedOutcome::Unsupported;
        }
        let Some(root) = interval.exact_root.as_ref() else {
            return RepresentedOutcome::Undecided;
        };
        return match strict_real_sign(&Real::eval_poly(relation.coefficients(), root)) {
            Some(RealSign::Zero) => {
                RepresentedOutcome::Decided(AlgebraicRootRepresentation::from_exact_value(root))
            }
            Some(RealSign::Negative | RealSign::Positive) => RepresentedOutcome::Unsupported,
            None => RepresentedOutcome::Undecided,
        };
    }
    #[cfg(test)]
    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
        eprintln!(
            "tensor image begin dimensions={:?} sources={}",
            relation.dimensions(),
            sources.len()
        );
    }
    let report = represent_algebraic_tensor_image(relation, sources, &interval);
    #[cfg(test)]
    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
        eprintln!("tensor image end status={:?}", report.status);
    }
    match report.status {
        AlgebraicTensorImageStatus::Transformed => RepresentedOutcome::Decided(
            report
                .representation
                .expect("a transformed tensor image retains its representation"),
        ),
        AlgebraicTensorImageStatus::NonIsolatingImageInterval
        | AlgebraicTensorImageStatus::Undecided => RepresentedOutcome::Undecided,
        AlgebraicTensorImageStatus::InvalidSourceEvidence
        | AlgebraicTensorImageStatus::InvalidRelationShape
        | AlgebraicTensorImageStatus::SourceSquareFreeFailed
        | AlgebraicTensorImageStatus::EliminationFailed
        | AlgebraicTensorImageStatus::ImageSquareFreeFailed
        | AlgebraicTensorImageStatus::InvalidTransformedEvidence => RepresentedOutcome::Unsupported,
    }
}

/// Refines selected source isolators until one exact tensor-image root is
/// separated. A repeated source/image state proves that further subdivision
/// cannot add evidence and remains an explicit predicate blocker; otherwise
/// no resource-shaped refinement ceiling changes the mathematical result.
pub fn represented_tensor_coordinate_refined(
    relation: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    initial_refinement_steps: usize,
    _hot_refinement_limit: usize,
    _trace_operation: &'static str,
    mut image_interval: impl FnMut(&[AlgebraicRootRepresentation], usize) -> Option<RealInterval>,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let mut refinement_steps = initial_refinement_steps;
    let mut previous = None;
    #[cfg(feature = "dispatch-trace")]
    if initial_refinement_steps > _hot_refinement_limit {
        hyperreal::dispatch_trace::record(
            "hypersolve",
            _trace_operation,
            "unbounded-cold-continuation",
        );
    }
    loop {
        let refined_sources = sources
            .iter()
            .map(|source| refined_represented_root(source, refinement_steps))
            .collect::<Vec<_>>();
        if let Some(mut interval) = image_interval(&refined_sources, refinement_steps) {
            // A retained exact `Real` expression can collapse interval
            // arithmetic to one non-rational endpoint before its canonical
            // univariate polynomial has been replayed. Replaying that endpoint
            // against the eliminant asks Hyperreal to rediscover a deep
            // eliminant cancellation and can reject otherwise valid evidence.
            // Replace only this degenerate non-rational enclosure with certified
            // dyadic bounds. The tensor-image authority still proves singleton
            // isolation under STRICT; no approximation selects the root.
            if strict_compare_reals(&interval.lower, &interval.upper) == Some(Ordering::Equal)
                && interval.lower.exact_rational_normal_form().is_none()
            {
                let precision = refinement_steps.max(64).min(i32::MAX as usize) as i32;
                if let Some([lower, upper]) = interval.lower.certified_rational_interval(-precision)
                {
                    interval = RealInterval {
                        lower: Real::new(lower),
                        upper: Real::new(upper),
                    };
                }
            }
            let unchanged = previous
                .as_ref()
                .is_some_and(|(old_sources, old_interval)| {
                    old_sources == &refined_sources && old_interval == &interval
                });
            #[cfg(test)]
            if relation.dimensions().len() == 4
                && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            {
                eprintln!(
                    "tensor coordinate refinement operation={_trace_operation} steps={refinement_steps} unchanged={unchanged}"
                );
            }
            match represented_tensor_coordinate(
                relation,
                &refined_sources,
                &interval.lower,
                &interval.upper,
            ) {
                decided @ RepresentedOutcome::Decided(_) => return decided,
                RepresentedOutcome::Unsupported => {
                    return RepresentedOutcome::Unsupported;
                }
                RepresentedOutcome::Undecided | RepresentedOutcome::Vanishes if unchanged => {
                    return RepresentedOutcome::Undecided;
                }
                RepresentedOutcome::Undecided | RepresentedOutcome::Vanishes => {}
            }
            previous = Some((refined_sources, interval));
        }
        let Some(next_steps) = (if refinement_steps == 0 {
            Some(4)
        } else {
            refinement_steps.checked_mul(2)
        }) else {
            return RepresentedOutcome::Undecided;
        };
        #[cfg(feature = "dispatch-trace")]
        if refinement_steps <= _hot_refinement_limit && next_steps > _hot_refinement_limit {
            hyperreal::dispatch_trace::record(
                "hypersolve",
                _trace_operation,
                "unbounded-cold-continuation",
            );
        }
        refinement_steps = next_steps;
    }
}

/// Constructs one exact affine image of already selected algebraic numbers.
/// Exact point witnesses and certified affine-related sources collapse before
/// elimination. Any remaining selected roots retain their exact isolators as
/// tensor axes; no rounded coordinate or approximate sheet choice is used.
pub fn represented_affine_coordinate(
    terms: &[(&AlgebraicRootRepresentation, &Real)],
    offset: &Real,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let mut affine_offset = offset.clone();
    let active = terms
        .iter()
        .filter_map(|(source, scale)| {
            if scale.zero_status() == ZeroKnowledge::Zero {
                return None;
            }
            if let Some(value) = source.exact_point_witness() {
                affine_offset = affine_offset.clone() + *scale * value;
                return None;
            }
            Some((*source, *scale))
        })
        .collect::<Vec<_>>();
    if active.is_empty() {
        return RepresentedOutcome::Decided(AlgebraicRootRepresentation::from_exact_value(
            &affine_offset,
        ));
    }
    let affine_image = |source: &AlgebraicRootRepresentation, scale: &Real, offset: &Real| {
        if scale.zero_status() == ZeroKnowledge::Zero {
            return RepresentedOutcome::Decided(AlgebraicRootRepresentation::from_exact_value(
                offset,
            ));
        }
        if scale == &Real::one() && offset.zero_status() == ZeroKnowledge::Zero {
            return RepresentedOutcome::Decided(source.clone());
        }
        let report = transform_algebraic_root_affine(
            source,
            scale.clone(),
            offset.clone(),
            crate::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicRootAffineTransformStatus::Transformed => RepresentedOutcome::Decided(
                report
                    .representation
                    .expect("a transformed affine root retains its representation"),
            ),
            AlgebraicRootAffineTransformStatus::Undecided => RepresentedOutcome::Undecided,
            AlgebraicRootAffineTransformStatus::InvalidEvidence
            | AlgebraicRootAffineTransformStatus::ZeroScale
            | AlgebraicRootAffineTransformStatus::InvalidTransformedEvidence => {
                RepresentedOutcome::Unsupported
            }
        }
    };
    if active.len() == 1 {
        return affine_image(active[0].0, active[0].1, &affine_offset);
    }
    let base = active[0].0;
    let mut combined_scale = active[0].1.clone();
    let mut combined_offset = affine_offset.clone();
    let mut all_affine = true;
    for (source, coefficient) in active.iter().skip(1) {
        let relation = if *source == base {
            Some(crate::AlgebraicRootAffineRelation {
                scale: Real::one(),
                offset: Real::zero(),
            })
        } else {
            algebraic_root_affine_relation(base, source)
        };
        let Some(relation) = relation else {
            all_affine = false;
            break;
        };
        combined_scale += *coefficient * relation.scale;
        combined_offset += *coefficient * relation.offset;
    }
    if all_affine {
        return affine_image(base, &combined_scale, &combined_offset);
    }
    let rank = active.len() + 1;
    let output_axis = rank - 1;
    let Some(mut relation) = DenseTensorPolynomial::from_axis_polynomial(
        rank,
        output_axis,
        &[(-affine_offset.clone()), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let mut sources = Vec::with_capacity(active.len());
    let mut scales = Vec::with_capacity(active.len());
    for (axis, (source, scale)) in active.iter().enumerate() {
        let Some(term) = DenseTensorPolynomial::from_axis_polynomial(
            rank,
            axis,
            &[Real::zero(), (*scale).clone()],
        ) else {
            return RepresentedOutcome::Unsupported;
        };
        let Some(next_relation) = relation.subtract(&term) else {
            return RepresentedOutcome::Unsupported;
        };
        relation = next_relation;
        sources.push((*source).clone());
        scales.push((*scale).clone());
    }
    represented_tensor_coordinate_refined(
        &relation,
        &sources,
        0,
        256,
        "represented-affine-image-separation",
        |refined_sources, _| {
            let mut interval = RealInterval {
                lower: affine_offset.clone(),
                upper: affine_offset.clone(),
            };
            for (source, scale) in refined_sources.iter().zip(&scales) {
                let source_interval = RealInterval {
                    lower: source.interval.lower.clone(),
                    upper: source.interval.upper.clone(),
                };
                let scale_interval = RealInterval {
                    lower: scale.clone(),
                    upper: scale.clone(),
                };
                let term_interval = source_interval.multiply(&scale_interval)?;
                interval = interval.add(&term_interval);
            }
            Some(interval)
        },
    )
}

/// Exact dense tensor interval with coefficient precision over represented algebraic roots.
pub fn dense_tensor_interval_with_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: Option<i32>,
) -> Option<RealInterval> {
    dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
        polynomial,
        sources,
        None,
        coefficient_precision,
    )
}

/// Exact dense tensor interval with coefficient precision and source witnesses over represented algebraic roots.
pub fn dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    source_real_witnesses: Option<&[Option<Real>]>,
    coefficient_precision: Option<i32>,
) -> Option<RealInterval> {
    let dimensions = polynomial.dimensions();
    if dimensions.len() != sources.len() + 1
        || dimensions.last() != Some(&1)
        || source_real_witnesses.is_some_and(|witnesses| witnesses.len() != sources.len())
    {
        return None;
    }
    if dense_tensor_is_stored_zero(polynomial) {
        return Some(RealInterval {
            lower: Real::zero(),
            upper: Real::zero(),
        });
    }
    let source_intervals = sources
        .iter()
        .enumerate()
        .map(|(index, source)| {
            let (lower, upper) = if let Some(value) =
                source_real_witnesses.and_then(|witnesses| witnesses[index].as_ref())
            {
                (value, value)
            } else {
                (&source.interval.lower, &source.interval.upper)
            };
            if let Some(precision) = coefficient_precision.filter(|_| dimensions[index] > 1) {
                // Keep the entire filtering calculation dyadic. Source charts
                // can have arbitrary rational endpoints or exact scalar
                // witnesses; multiplying them through a tensor needlessly
                // grows denominators or scalar expressions. Outward bounds
                // preserve every source value and leave exact replay intact.
                return Some(RealInterval {
                    lower: Real::new(lower.certified_dyadic_interval(precision)?[0].clone()),
                    upper: Real::new(upper.certified_dyadic_interval(precision)?[1].clone()),
                });
            }
            Some(RealInterval {
                lower: lower.clone(),
                upper: upper.clone(),
            })
        })
        .collect::<Option<Vec<_>>>()?;
    fn evaluate(
        polynomial: &DenseTensorPolynomial,
        dimensions: &[usize],
        source_intervals: &[RealInterval],
        coefficient_precision: Option<i32>,
        axis: usize,
        flat_prefix: usize,
    ) -> Option<RealInterval> {
        if axis == source_intervals.len() {
            let coefficient = polynomial.coefficients().get(flat_prefix)?;
            if coefficient
                .exact_rational_ref()
                .is_some_and(|value| value.is_zero())
            {
                return Some(RealInterval {
                    lower: Real::zero(),
                    upper: Real::zero(),
                });
            }
            if let Some(precision) = coefficient_precision {
                // A coefficient can be an exact but structurally opaque
                // cancellation. Requiring its sign would block the entire tensor
                // even when its certified magnitude is far too small to affect
                // the result. Dyadic bounds are exact enclosures, not an
                // approximate equality decision.
                let [lower, upper] = coefficient.certified_dyadic_interval(precision)?;
                return Some(RealInterval {
                    lower: Real::new(lower),
                    upper: Real::new(upper),
                });
            }
            return Some(RealInterval {
                lower: coefficient.clone(),
                upper: coefficient.clone(),
            });
        }
        let stride = dimensions[axis + 1..]
            .iter()
            .try_fold(1_usize, |stride, dimension| stride.checked_mul(*dimension))?;
        let degree = dimensions[axis].checked_sub(1)?;
        let mut value = evaluate(
            polynomial,
            dimensions,
            source_intervals,
            coefficient_precision,
            axis + 1,
            flat_prefix.checked_add(degree.checked_mul(stride)?)?,
        )?;
        for exponent in (0..degree).rev() {
            let coefficient = evaluate(
                polynomial,
                dimensions,
                source_intervals,
                coefficient_precision,
                axis + 1,
                flat_prefix.checked_add(exponent.checked_mul(stride)?)?,
            )?;
            value = value.multiply(&source_intervals[axis])?.add(&coefficient);
        }
        Some(value)
    }
    evaluate(
        polynomial,
        dimensions,
        &source_intervals,
        coefficient_precision,
        0,
        0,
    )
}

/// Exact dense tensor interval over represented algebraic roots.
pub fn dense_tensor_interval(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_tensor_interval_with_coefficient_precision(polynomial, sources, None)
}

/// Exact refined represented root over represented algebraic roots.
pub fn refined_represented_root(
    source: &AlgebraicRootRepresentation,
    refinement_steps: usize,
) -> AlgebraicRootRepresentation {
    if refinement_steps == 0 || source.interval.exact_root.is_some() {
        return source.clone();
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record("hypersolve", "represented-root-bounds", "refine");
    let report = refine_isolated_univariate_polynomial_interval(
        &source.polynomial_coefficients,
        &source.interval,
        RootIsolationConfig {
            policy: crate::PredicatePolicy::STRICT,
            max_interval_width: None,
            max_refinement_steps: refinement_steps,
        },
    );
    let Some(interval) = report.refined_interval else {
        return source.clone();
    };
    let mut refined = source.clone();
    refined.interval = interval;
    refined.validation =
        validate_algebraic_root_representation(&refined, crate::PredicatePolicy::STRICT);
    if refined.is_valid() {
        refined
    } else {
        source.clone()
    }
}

fn represented_dense_value_with_optional_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: Option<i32>,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let dimensions = polynomial.dimensions();
    if dimensions.len() != sources.len() + 1 || dimensions.last() != Some(&1) {
        return RepresentedOutcome::Unsupported;
    }
    let output_axis = dimensions.len() - 1;
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        dimensions.len(),
        output_axis,
        &[Real::zero(), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = output.subtract(polynomial) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(interval) = dense_tensor_interval_with_coefficient_precision(
        polynomial,
        sources,
        coefficient_precision,
    ) else {
        return RepresentedOutcome::Undecided;
    };
    #[cfg(test)]
    if relation.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!("tensor coordinate direct operation=represented-dense-value");
    }
    represented_tensor_coordinate(&relation, sources, &interval.lower, &interval.upper)
}

/// Exact represented dense value with coefficient precision over represented algebraic roots.
pub fn represented_dense_value_with_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: i32,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    #[cfg(test)]
    if polynomial.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!(
            "represented dense value caller=tuple-sign coefficient-precision={coefficient_precision}"
        );
    }
    represented_dense_value_with_optional_coefficient_precision(
        polynomial,
        sources,
        Some(coefficient_precision),
    )
}

fn represented_dense_value(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    #[cfg(test)]
    if polynomial.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!("represented dense value caller=vector-dot-cross");
    }
    represented_dense_value_with_optional_coefficient_precision(polynomial, sources, None)
}

/// Exact represented dense value refined over represented algebraic roots.
pub fn represented_dense_value_refined(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let dimensions = polynomial.dimensions();
    if dimensions.len() != sources.len() + 1 || dimensions.last() != Some(&1) {
        return RepresentedOutcome::Unsupported;
    }
    if sources.is_empty() {
        return RepresentedOutcome::Decided(AlgebraicRootRepresentation::from_exact_value(
            &polynomial.coefficients()[0],
        ));
    }
    let output_axis = dimensions.len() - 1;
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        dimensions.len(),
        output_axis,
        &[Real::zero(), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = output.subtract(polynomial) else {
        return RepresentedOutcome::Unsupported;
    };
    represented_tensor_coordinate_refined(
        &relation,
        sources,
        0,
        256,
        "represented-dense-image-separation",
        |refined, refinement_steps| {
            let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
            dense_tensor_interval_with_coefficient_precision(
                polynomial,
                refined,
                Some(-coefficient_bits),
            )
        },
    )
}

/// Proves equality of two selected algebraic numbers even when their defining
/// eliminants differ. This is STRICT construction evidence: a shared isolated
/// polynomial root or an exactly signed algebraic difference is required.
pub fn represented_roots_strictly_equal(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> bool {
    represented_strict_order(left, right) == Some(Ordering::Equal)
}

/// Exact represented affine tensor basis over represented algebraic roots.
pub fn represented_affine_tensor_basis(
    coordinates: &[AlgebraicRootRepresentation],
) -> Option<(Vec<AlgebraicRootRepresentation>, Vec<DenseTensorPolynomial>)> {
    let mut sources = Vec::<AlgebraicRootRepresentation>::new();
    let mut descriptions = Vec::with_capacity(coordinates.len());
    for coordinate in coordinates {
        if let Some(value) = coordinate.exact_point_witness() {
            descriptions.push((None, Real::zero(), value.clone()));
            continue;
        }
        let relation = sources.iter().enumerate().find_map(|(axis, source)| {
            let relation = if source == coordinate {
                crate::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else if let Some(relation) = algebraic_root_affine_relation(source, coordinate)
                && let (Some(scale), Some(offset)) = (
                    rational_tensor_constant(&relation.scale),
                    rational_tensor_constant(&relation.offset),
                )
            {
                // An irrational relation (for example sqrt(1/3) as
                // sqrt(2/3) * sqrt(1/2)) would move a field generator into
                // tensor coefficients; give that root its own axis instead.
                crate::AlgebraicRootAffineRelation { scale, offset }
            } else if represented_roots_strictly_equal(source, coordinate) {
                crate::AlgebraicRootAffineRelation {
                    scale: Real::one(),
                    offset: Real::zero(),
                }
            } else {
                return None;
            };
            Some((axis, relation))
        });
        if let Some((axis, relation)) = relation {
            descriptions.push((Some(axis), relation.scale, relation.offset));
        } else {
            let axis = sources.len();
            sources.push(coordinate.clone());
            descriptions.push((Some(axis), Real::one(), Real::zero()));
        }
    }

    let rank = sources.len() + 1;
    let constant = |value: &Real| {
        DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
    };
    let mut polynomials = Vec::with_capacity(descriptions.len());
    for (source, scale, offset) in descriptions {
        let polynomial = if let Some(axis) = source {
            DenseTensorPolynomial::from_axis_polynomial(rank, axis, &[offset, scale])?
        } else {
            constant(&offset)?
        };
        polynomials.push(polynomial);
    }
    Some((sources, polynomials))
}

/// Exact represented tensor nested interval over represented algebraic roots.
pub fn represented_tensor_nested_interval(
    retained: &DenseTensorPolynomial,
    candidate: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
) -> Option<RealInterval> {
    let retained = dense_tensor_interval(retained, sources)?;
    let candidate = dense_tensor_interval(candidate, sources)?;
    let radical = RealInterval {
        lower: signed_radical.interval.lower.clone(),
        upper: signed_radical.interval.upper.clone(),
    };
    Some(retained.add(&candidate.multiply(&radical)?))
}

/// Exact represented tensor nested value refined over represented algebraic roots.
#[allow(clippy::too_many_arguments)]
fn represented_tensor_nested_value_refined(
    retained: &DenseTensorPolynomial,
    candidate: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
    initial_refinement_steps: usize,
    hot_refinement_limit: usize,
    trace_operation: &'static str,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    if candidate
        .coefficients()
        .iter()
        .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
    {
        return represented_dense_value_refined(retained, sources);
    }
    let rank = sources.len() + 1;
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        rank,
        sources.len(),
        &[Real::zero(), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = output.subtract(retained).and_then(|residual| {
        residual
            .multiply(&residual)?
            .subtract(&candidate.multiply(candidate)?.multiply(discriminant)?)
    }) else {
        return RepresentedOutcome::Unsupported;
    };
    represented_tensor_coordinate_refined(
        &relation,
        sources,
        initial_refinement_steps,
        hot_refinement_limit,
        trace_operation,
        |refined_sources, refinement_steps| {
            let refined_radical = refined_represented_root(signed_radical, refinement_steps);
            represented_tensor_nested_interval(
                retained,
                candidate,
                refined_sources,
                &refined_radical,
            )
        },
    )
}

/// Exact represented strict order over represented algebraic roots.
pub fn represented_strict_order(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> Option<Ordering> {
    let report = compare_algebraic_root_representations_by_difference(
        left,
        right,
        AlgebraicRootRefinementComparisonConfig {
            policy: crate::PredicatePolicy::STRICT,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    matches!(
        report.comparison.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    )
    .then_some(report.comparison.ordering)
    .flatten()
}

/// Exact represented strict sign over represented algebraic roots.
pub fn represented_strict_sign(value: &AlgebraicRootRepresentation) -> Option<RealSign> {
    if strict_compare_reals(&value.interval.upper, &Real::zero()) == Some(Ordering::Less) {
        return Some(RealSign::Negative);
    }
    if strict_compare_reals(&value.interval.lower, &Real::zero()) == Some(Ordering::Greater) {
        return Some(RealSign::Positive);
    }
    represented_strict_order(
        value,
        &AlgebraicRootRepresentation::from_exact_value(&Real::zero()),
    )
    .map(|order| match order {
        Ordering::Less => RealSign::Negative,
        Ordering::Equal => RealSign::Zero,
        Ordering::Greater => RealSign::Positive,
    })
}

/// Exact represented ratio over represented algebraic roots.
pub fn represented_ratio(
    numerator: &AlgebraicRootRepresentation,
    denominator: &AlgebraicRootRepresentation,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let numerator_interval = RealInterval {
        lower: numerator.interval.lower.clone(),
        upper: numerator.interval.upper.clone(),
    };
    let denominator_interval = RealInterval {
        lower: denominator.interval.lower.clone(),
        upper: denominator.interval.upper.clone(),
    };
    let Some(interval) = numerator_interval.divide(&denominator_interval) else {
        return RepresentedOutcome::Undecided;
    };
    let Some(numerator_axis) =
        DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
    else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(denominator_axis) =
        DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
    else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(output) =
        DenseTensorPolynomial::from_axis_polynomial(3, 2, &[Real::zero(), Real::one()])
    else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = denominator_axis
        .multiply(&output)
        .and_then(|product| product.subtract(&numerator_axis))
    else {
        return RepresentedOutcome::Unsupported;
    };
    #[cfg(test)]
    if relation.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!("tensor coordinate direct operation=represented-ratio");
    }
    represented_tensor_coordinate(
        &relation,
        &[numerator.clone(), denominator.clone()],
        &interval.lower,
        &interval.upper,
    )
}

/// Materializes the exact dot and oriented-area products of two represented
/// vectors through one four-source tensor authority. Affine-related component
/// axes collapse in Hypersolve before any remaining resultant elimination.
pub fn represented_vector_dot_cross(
    first: &[AlgebraicRootRepresentation; 2],
    second: &[AlgebraicRootRepresentation; 2],
) -> RepresentedOutcome<[AlgebraicRootRepresentation; 2]> {
    let combine = |dot, cross| match (dot, cross) {
        (RepresentedOutcome::Decided(dot), RepresentedOutcome::Decided(cross)) => {
            RepresentedOutcome::Decided([dot, cross])
        }
        (RepresentedOutcome::Unsupported, _) | (_, RepresentedOutcome::Unsupported) => {
            RepresentedOutcome::Unsupported
        }
        _ => RepresentedOutcome::Undecided,
    };
    let exact_products = |first: &[AlgebraicRootRepresentation; 2],
                          second: &[AlgebraicRootRepresentation; 2]| {
        let [Some(first_x), Some(first_y), Some(second_x), Some(second_y)] = [
            first[0].exact_point_witness(),
            first[1].exact_point_witness(),
            second[0].exact_point_witness(),
            second[1].exact_point_witness(),
        ] else {
            return None;
        };
        Some([
            AlgebraicRootRepresentation::from_exact_value(
                &(first_x * second_x + first_y * second_y),
            ),
            AlgebraicRootRepresentation::from_exact_value(
                &(first_x * second_y - first_y * second_x),
            ),
        ])
    };
    if let Some(products) = exact_products(first, second) {
        return RepresentedOutcome::Decided(products);
    }
    if let [Some(second_x), Some(second_y)] = [
        second[0].exact_point_witness(),
        second[1].exact_point_witness(),
    ] {
        let negative_x = -second_x;
        return combine(
            represented_affine_coordinate(
                &[(&first[0], second_x), (&first[1], second_y)],
                &Real::zero(),
            ),
            represented_affine_coordinate(
                &[(&first[0], second_y), (&first[1], &negative_x)],
                &Real::zero(),
            ),
        );
    }
    if let [Some(first_x), Some(first_y)] = [
        first[0].exact_point_witness(),
        first[1].exact_point_witness(),
    ] {
        let negative_y = -first_y;
        return combine(
            represented_affine_coordinate(
                &[(&second[0], first_x), (&second[1], first_y)],
                &Real::zero(),
            ),
            represented_affine_coordinate(
                &[(&second[0], &negative_y), (&second[1], first_x)],
                &Real::zero(),
            ),
        );
    }
    let coordinates = [
        first[0].clone(),
        first[1].clone(),
        second[0].clone(),
        second[1].clone(),
    ];
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&coordinates) else {
        return RepresentedOutcome::Unsupported;
    };
    let [first_x, first_y, second_x, second_y]: [DenseTensorPolynomial; 4] = coordinates
        .try_into()
        .expect("the represented vector basis retains all four coordinates");
    let Some((dot, cross)) = (|| {
        Some((
            first_x
                .multiply(&second_x)?
                .add(&first_y.multiply(&second_y)?)?,
            first_x
                .multiply(&second_y)?
                .subtract(&first_y.multiply(&second_x)?)?,
        ))
    })() else {
        return RepresentedOutcome::Unsupported;
    };
    let dot = represented_dense_value(&dot, &sources);
    let cross = represented_dense_value(&cross, &sources);
    combine(dot, cross)
}

/// `represented_zero_offset_unit_scales` bit: the scale is `+1`.
pub const POSITIVE_UNIT_SCALE: u8 = 1;
/// `represented_zero_offset_unit_scales` bit: the scale is `-1`.
pub const NEGATIVE_UNIT_SCALE: u8 = 2;

/// Proves `right = sign * left` directly from already validated polynomial
/// and isolator evidence. Proportional defining polynomials describe the same
/// root set, while the equal or reflected one-root intervals select the same
/// sheet. This avoids rerunning a Sturm common-root proof for representations
/// produced by an exact identity or negation.
fn represented_structural_unit_scale(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    sign: i8,
) -> bool {
    if !left.is_valid()
        || !right.is_valid()
        || left.interval.distinct_root_count != 1
        || right.interval.distinct_root_count != 1
        || left.polynomial_coefficients.len() != right.polynomial_coefficients.len()
        || left.polynomial_coefficients.len() < 2
    {
        return false;
    }
    let (expected_lower, expected_upper) = if sign > 0 {
        (left.interval.lower.clone(), left.interval.upper.clone())
    } else {
        (-left.interval.upper.clone(), -left.interval.lower.clone())
    };
    if strict_compare_reals(&expected_lower, &right.interval.lower) != Some(Ordering::Equal)
        || strict_compare_reals(&expected_upper, &right.interval.upper) != Some(Ordering::Equal)
    {
        return false;
    }

    let left_leading = left.polynomial_coefficients.last().unwrap();
    let right_leading = right.polynomial_coefficients.last().unwrap();
    let degree = left.polynomial_coefficients.len() - 1;
    let transformed_leading = if sign < 0 && !degree.is_multiple_of(2) {
        -left_leading
    } else {
        left_leading.clone()
    };
    left.polynomial_coefficients
        .iter()
        .zip(&right.polynomial_coefficients)
        .enumerate()
        .all(|(power, (left_coefficient, right_coefficient))| {
            let transformed = if sign < 0 && !power.is_multiple_of(2) {
                -left_coefficient
            } else {
                left_coefficient.clone()
            };
            strict_compare_reals(
                &(transformed * right_leading),
                &(right_coefficient * &transformed_leading),
            ) == Some(Ordering::Equal)
        })
}

/// Returns every unit scale exactly certified by `right = scale * left`.
/// Zero admits both signs; retaining that ambiguity is necessary for axial
/// quarter turns, where either signed relation describes the zero component.
pub fn represented_zero_offset_unit_scales(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> u8 {
    if let (Some(left), Some(right)) = (left.exact_point_witness(), right.exact_point_witness()) {
        let mut scales = 0;
        if strict_compare_reals(left, right) == Some(Ordering::Equal) {
            scales |= POSITIVE_UNIT_SCALE;
        }
        if strict_compare_reals(&(-left), right) == Some(Ordering::Equal) {
            scales |= NEGATIVE_UNIT_SCALE;
        }
        return scales;
    }

    let mut scales = 0;
    if left == right || represented_structural_unit_scale(left, right, 1) {
        scales |= POSITIVE_UNIT_SCALE;
    }
    if represented_structural_unit_scale(left, right, -1) {
        scales |= NEGATIVE_UNIT_SCALE;
    }
    if scales != 0 {
        return scales;
    }

    if represented_roots_strictly_equal(left, right) {
        scales |= POSITIVE_UNIT_SCALE;
    }
    let reflected = crate::transform_algebraic_root_affine(
        left,
        Real::from(-1_i8),
        Real::zero(),
        crate::PredicatePolicy::STRICT,
    );
    if reflected
        .representation
        .as_ref()
        .is_some_and(|reflected| represented_roots_strictly_equal(reflected, right))
    {
        scales |= NEGATIVE_UNIT_SCALE;
    }
    if scales != 0 {
        return scales;
    }

    if let Some(relation) = algebraic_root_affine_relation(left, right)
        && strict_compare_reals(&relation.offset, &Real::zero()) == Some(Ordering::Equal)
    {
        if strict_compare_reals(&relation.scale, &Real::one()) == Some(Ordering::Equal) {
            scales |= POSITIVE_UNIT_SCALE;
        }
        if strict_compare_reals(&relation.scale, &Real::from(-1_i8)) == Some(Ordering::Equal) {
            scales |= NEGATIVE_UNIT_SCALE;
        }
    }
    scales
}

/// Collapses affine-related selected tensor axes before quotient reduction
/// or image projection, preserving their root correlation.
pub fn dense_substitute_affinely_related_sources(
    mut polynomial: DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<(DenseTensorPolynomial, Vec<AlgebraicRootRepresentation>)> {
    if polynomial.dimensions().len() != sources.len() {
        return None;
    }
    let mut sources = sources.to_vec();
    'next_relation: loop {
        for retained in 0..sources.len() {
            for removed in retained + 1..sources.len() {
                let relation = if sources[retained] == sources[removed]
                    || represented_roots_strictly_equal(&sources[retained], &sources[removed])
                {
                    Some(crate::AlgebraicRootAffineRelation {
                        scale: Real::one(),
                        offset: Real::zero(),
                    })
                } else {
                    algebraic_root_affine_relation(&sources[retained], &sources[removed])
                };
                let Some(relation) = relation else {
                    continue;
                };
                polynomial = polynomial.substitute_affine_axis(
                    retained,
                    removed,
                    &relation.scale,
                    &relation.offset,
                )?;
                sources.remove(removed);
                continue 'next_relation;
            }
        }
        break;
    }
    Some((polynomial, sources))
}

/// Returns the sign of every value in an interval when certified by STRICT
/// predicates or by more precision on a structurally nonzero endpoint.
pub fn dense_strict_interval_sign(value: &RealInterval) -> Option<RealSign> {
    // Hyperlimit's ordinary STRICT scalar predicate deliberately stops at its
    // fixed refinement budget. Interval endpoints can independently carry a
    // structural nonzero certificate, however, so asking Hyperreal for more
    // precision is still an exact sign proof rather than an equality policy.
    // This is substantially smaller than projecting a recursive algebraic
    // norm for tiny but already-known-nonzero endpoint values.
    let certified_nonzero_sign = |value: &Real| {
        (value.zero_status() == ZeroKnowledge::NonZero)
            .then(|| {
                value
                    .immediate_sign()
                    .or_else(|| value.certified_sign_until(-4096).sign())
            })
            .flatten()
            .filter(|sign| *sign != RealSign::Zero)
    };
    let upper_sign =
        strict_real_sign(&value.upper).or_else(|| certified_nonzero_sign(&value.upper));
    let lower_sign =
        strict_real_sign(&value.lower).or_else(|| certified_nonzero_sign(&value.lower));
    if upper_sign == Some(RealSign::Negative) {
        Some(RealSign::Negative)
    } else if lower_sign == Some(RealSign::Positive) {
        Some(RealSign::Positive)
    } else if value.lower.zero_status() == ZeroKnowledge::Zero
        && value.upper.zero_status() == ZeroKnowledge::Zero
    {
        Some(RealSign::Zero)
    } else {
        None
    }
}

/// Encloses a dense tensor polynomial at a represented root tuple.
pub fn dense_polynomial_value_interval(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_tensor_interval(&dense_tensor_with_output_axis(polynomial)?, sources)
}

/// Encloses a dense tensor polynomial at a represented root tuple, rounding
/// coefficient endpoints outward at `coefficient_precision`.
pub fn dense_polynomial_value_interval_with_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: i32,
) -> Option<RealInterval> {
    dense_tensor_interval_with_coefficient_precision(
        &dense_tensor_with_output_axis(polynomial)?,
        sources,
        Some(coefficient_precision),
    )
}

/// Encloses a two-radical expression on its positive square-root branches at
/// a represented root tuple, with optional real witnesses for the sources and
/// outward coefficient rounding at `coefficient_precision`.
pub fn dense_two_positive_square_root_interval_with_coefficient_precision(
    expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
    first_speed_squared: &DenseTensorPolynomial,
    second_speed_squared: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    source_real_witnesses: Option<&[Option<Real>]>,
    coefficient_precision: Option<i32>,
) -> Option<RealInterval> {
    let interval = |polynomial: &DenseTensorPolynomial| {
        dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
            &dense_tensor_with_output_axis(polynomial)?,
            sources,
            source_real_witnesses,
            coefficient_precision,
        )
    };
    let radicands = [first_speed_squared, second_speed_squared];
    let mut roots: [Option<RealInterval>; 2] = [None, None];
    let mut value = interval(&expression.rational)?;
    for (coefficient, mask) in [
        (&expression.first, 1),
        (&expression.second, 2),
        (&expression.product, 3),
    ] {
        if dense_tensor_is_stored_zero(coefficient) {
            continue;
        }
        let mut term = interval(coefficient)?;
        for (index, radicand) in radicands.iter().enumerate() {
            if mask & (1 << index) != 0 {
                let root = match &roots[index] {
                    Some(root) => root,
                    None => roots[index].insert(
                        interval(radicand)?.nonnegative_square_root(coefficient_precision)?,
                    ),
                };
                term = term.multiply(root)?;
            }
        }
        value = value.add(&term);
    }
    Some(value)
}

/// Encloses a two-radical expression on its positive square-root branches at
/// a represented root tuple.
pub fn dense_two_positive_square_root_interval(
    expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
    first_speed_squared: &DenseTensorPolynomial,
    second_speed_squared: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_two_positive_square_root_interval_with_coefficient_precision(
        expression,
        first_speed_squared,
        second_speed_squared,
        sources,
        None,
        None,
    )
}

/// Encloses `rational + radical*sqrt(radicand)` on the positive branch at a
/// represented root tuple, rounding coefficients outward at the given precision.
fn dense_positive_square_root_interval_with_coefficient_precision(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: Option<i32>,
) -> Option<RealInterval> {
    let interval = |polynomial: &DenseTensorPolynomial| match coefficient_precision {
        Some(precision) => dense_polynomial_value_interval_with_coefficient_precision(
            polynomial, sources, precision,
        ),
        None => dense_polynomial_value_interval(polynomial, sources),
    };
    let speed = interval(radicand)?.nonnegative_square_root(coefficient_precision)?;
    Some(interval(rational)?.add(&interval(radical)?.multiply(&speed)?))
}

/// Encloses `rational + radical*sqrt(radicand)` on the positive branch at a
/// represented root tuple.
pub fn dense_positive_square_root_interval(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_positive_square_root_interval_with_coefficient_precision(
        rational, radical, radicand, sources, None,
    )
}

/// Combines the signs of two terms on the same positive-root sheet: a zero
/// term defers to the other, equal signs agree, and opposite signs are undecided.
pub fn same_positive_root_sheet_signs(first: RealSign, second: RealSign) -> Option<RealSign> {
    match (first, second) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => Some(sign),
        (first, second) if first == second => Some(first),
        _ => None,
    }
}

/// The approximation protocol of a predicate evaluation context.
///
/// A context may select the `APPROXIMATE_512` terminal for an operation while
/// a strict pass temporarily forbids consuming it. Exact constructions consult
/// these facts and report every approximate decision they consume.
pub trait ApproximationPolicy {
    /// Whether the operation selected the `APPROXIMATE_512` terminal, even if
    /// a strict pass currently forbids consuming it.
    fn selects_approximate_512(&self) -> bool;
    /// Whether an approximate terminal decision may be consumed now.
    fn permits_approximate_512(&self) -> bool;
    /// Records that an approximate terminal decision was consumed.
    fn observe_approximate_512(&self);
}

/// The sign of a represented value: a certified STRICT sign, else an
/// `APPROXIMATE_512` comparison with zero when `policy` permits and records it.
pub fn represented_policy_sign(
    value: &AlgebraicRootRepresentation,
    policy: &impl ApproximationPolicy,
) -> RepresentedOutcome<RealSign> {
    if let Some(sign) = represented_strict_sign(value) {
        return RepresentedOutcome::Decided(sign);
    }
    if !policy.permits_approximate_512() {
        return RepresentedOutcome::Undecided;
    }
    let zero = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    let report = compare_algebraic_root_representations_with_refinement(
        value,
        &zero,
        AlgebraicRootRefinementComparisonConfig {
            policy: PredicatePolicy::APPROXIMATE_512,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    let Some(order) = matches!(
        report.comparison.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    )
    .then_some(report.comparison.ordering)
    .flatten() else {
        return RepresentedOutcome::Undecided;
    };
    policy.observe_approximate_512();
    RepresentedOutcome::Decided(match order {
        Ordering::Less => RealSign::Negative,
        Ordering::Equal => RealSign::Zero,
        Ordering::Greater => RealSign::Positive,
    })
}

/// Decides the sign of a reduced dense polynomial at a selected source tuple
/// by refining the source isolators.
///
/// `value` is `polynomial` with an appended unit output axis. Each refinement
/// first tries a certified tensor-interval sign. STRICT continues through
/// complete algebraic tensor images without a refinement limit; an
/// `APPROXIMATE_512` operation stops at 512 refinement steps, where it either
/// consumes the approximate terminal or, inside a strict pass, reports the
/// sign as undecided for the outer replay.
pub fn dense_tuple_sign_by_refinement(
    polynomial: &DenseTensorPolynomial,
    value: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &impl ApproximationPolicy,
) -> RepresentedOutcome<RealSign> {
    let mut previous = None;
    let mut refinement_steps = 0_usize;
    let next_refinement_steps = |steps: usize| match steps {
        0 => Some(4),
        4 => Some(8),
        8 => Some(16),
        16 => Some(32),
        32 => Some(64),
        64 => Some(128),
        128 => Some(256),
        256 => Some(512),
        steps => steps.checked_mul(2),
    };
    loop {
        let refined = sources
            .iter()
            .map(|source| refined_represented_root(source, refinement_steps))
            .collect::<Vec<_>>();
        let progressed = previous.as_ref() != Some(&refined);
        previous = Some(refined.clone());
        if !progressed && refinement_steps < 512 {
            refinement_steps = next_refinement_steps(refinement_steps)
                .expect("the bounded refinement schedule cannot overflow");
            continue;
        }
        if !progressed && !policy.permits_approximate_512() {
            return RepresentedOutcome::Undecided;
        }
        let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
        let coefficient_precision = -coefficient_bits;
        if let Some(sign) = dense_polynomial_value_interval_with_coefficient_precision(
            polynomial,
            &refined,
            coefficient_precision,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign)
        {
            return RepresentedOutcome::Decided(sign);
        }
        let bounded_terminal = policy.selects_approximate_512() && refinement_steps == 512;
        let approximate_terminal = policy.permits_approximate_512() && bounded_terminal;
        // APPROXIMATE_512 already performs the certified tensor interval test
        // above at every refinement, including its 512-bit terminal. Building
        // a global tensor image cannot strengthen that policy's terminal
        // equality interpretation and would duplicate an exact elimination in
        // both the preliminary strict pass and the outer approximate replay.
        // STRICT alone retains the complete algebraic-image authority.
        let represented = if policy.selects_approximate_512() {
            RepresentedOutcome::Undecided
        } else {
            represented_dense_value_with_coefficient_precision(
                value,
                &refined,
                coefficient_precision,
            )
        };
        if let RepresentedOutcome::Decided(represented) = &represented
            && let Some(sign) = represented_strict_sign(represented)
        {
            return RepresentedOutcome::Decided(sign);
        }
        if represented == RepresentedOutcome::Unsupported {
            return RepresentedOutcome::Unsupported;
        }
        if approximate_terminal {
            return match represented {
                RepresentedOutcome::Decided(represented) => {
                    represented_policy_sign(&represented, policy)
                }
                RepresentedOutcome::Unsupported
                | RepresentedOutcome::Undecided
                | RepresentedOutcome::Vanishes => {
                    policy.observe_approximate_512();
                    RepresentedOutcome::Decided(RealSign::Zero)
                }
            };
        }
        if bounded_terminal {
            // This is the preliminary certified pass of an APPROXIMATE_512
            // operation. Preserve its strict uncertainty so the outer policy
            // replay can consume the terminal; never continue this selected
            // policy into an unbounded exact promotion.
            return RepresentedOutcome::Undecided;
        }
        refinement_steps = match next_refinement_steps(refinement_steps) {
            Some(next) => next,
            None => return RepresentedOutcome::Unsupported,
        };
        assert!(
            !(policy.selects_approximate_512() && refinement_steps > 512),
            "APPROXIMATE_512 cannot refine past its terminal"
        );
    }
}

/// Certifies that a represented value is nonzero.
fn represented_value_nonzero(
    value: RepresentedOutcome<AlgebraicRootRepresentation>,
) -> RepresentedOutcome<()> {
    let value = match value {
        RepresentedOutcome::Decided(value) => value,
        uncertain => return uncertain.map(|_| ()),
    };
    match represented_strict_sign(&value) {
        Some(RealSign::Positive | RealSign::Negative) => RepresentedOutcome::Decided(()),
        Some(RealSign::Zero) => RepresentedOutcome::Vanishes,
        None => RepresentedOutcome::Undecided,
    }
}

/// Certifies that a dense tensor value is nonzero at a represented root tuple.
fn represented_dense_nonzero(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> RepresentedOutcome<()> {
    represented_value_nonzero(represented_dense_value_refined(polynomial, sources))
}

/// Materializes `(A + branch*B*sqrt(S)) / (C + branch*D*sqrt(S))`
/// from one retained tensor authority. The supplied signed radical interval
/// selects the authored square-root sheet; the exact squared relation remains
/// independent of that procedural branch choice.
pub fn represented_tensor_nested_ratio(
    numerator_retained: &DenseTensorPolynomial,
    numerator_candidate: &DenseTensorPolynomial,
    denominator_retained: &DenseTensorPolynomial,
    denominator_candidate: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let rank = sources.len() + 1;
    if [
        numerator_retained,
        numerator_candidate,
        denominator_retained,
        denominator_candidate,
        discriminant,
    ]
    .into_iter()
    .any(|polynomial| {
        polynomial.dimensions().len() != rank || polynomial.dimensions().last() != Some(&1)
    }) {
        return RepresentedOutcome::Unsupported;
    }
    // With no retained tensor axes this is exactly an ordinary Mobius image
    // of the already represented signed radical. Reuse the complete quotient
    // authority instead of maintaining a second transform loop.
    if sources.is_empty() {
        let (Some(numerator), Some(denominator)) = (
            DenseTensorPolynomial::from_axis_polynomial(
                2,
                0,
                &[
                    numerator_retained.coefficients()[0].clone(),
                    numerator_candidate.coefficients()[0].clone(),
                ],
            ),
            DenseTensorPolynomial::from_axis_polynomial(
                2,
                0,
                &[
                    denominator_retained.coefficients()[0].clone(),
                    denominator_candidate.coefficients()[0].clone(),
                ],
            ),
        ) else {
            return RepresentedOutcome::Unsupported;
        };
        return represented_tensor_ratio(
            &numerator,
            &denominator,
            std::slice::from_ref(signed_radical),
        );
    }
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        rank,
        sources.len(),
        &[Real::zero(), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = (|| {
        let retained = denominator_retained
            .multiply(&output)?
            .subtract(numerator_retained)?;
        let candidate = denominator_candidate
            .multiply(&output)?
            .subtract(numerator_candidate)?;
        retained
            .multiply(&retained)?
            .subtract(&candidate.multiply(&candidate)?.multiply(discriminant)?)
    })() else {
        return RepresentedOutcome::Unsupported;
    };
    for refinement_steps in [0, 4, 8, 16, 32, 64] {
        let refined_sources = sources
            .iter()
            .map(|source| refined_represented_root(source, refinement_steps))
            .collect::<Vec<_>>();
        let refined_radical = refined_represented_root(signed_radical, refinement_steps);
        let (Some(numerator), Some(denominator)) = (
            represented_tensor_nested_interval(
                numerator_retained,
                numerator_candidate,
                &refined_sources,
                &refined_radical,
            ),
            represented_tensor_nested_interval(
                denominator_retained,
                denominator_candidate,
                &refined_sources,
                &refined_radical,
            ),
        ) else {
            continue;
        };
        let Some(interval) = numerator.divide(&denominator) else {
            continue;
        };
        match represented_tensor_coordinate(
            &relation,
            &refined_sources,
            &interval.lower,
            &interval.upper,
        ) {
            decided @ RepresentedOutcome::Decided(_) => return decided,
            RepresentedOutcome::Unsupported => return RepresentedOutcome::Unsupported,
            RepresentedOutcome::Undecided | RepresentedOutcome::Vanishes => {}
        }
    }
    if let Some(uncertain) = represented_value_nonzero(represented_tensor_nested_value_refined(
        denominator_retained,
        denominator_candidate,
        discriminant,
        sources,
        signed_radical,
        128,
        64,
        "represented-nested-denominator-separation",
    ))
    .uncertain()
    {
        return uncertain;
    }
    represented_tensor_coordinate_refined(
        &relation,
        sources,
        128,
        64,
        "represented-nested-ratio-image-separation",
        |refined_sources, refinement_steps| {
            let refined_radical = refined_represented_root(signed_radical, refinement_steps);
            let numerator = represented_tensor_nested_interval(
                numerator_retained,
                numerator_candidate,
                refined_sources,
                &refined_radical,
            )?;
            let denominator = represented_tensor_nested_interval(
                denominator_retained,
                denominator_candidate,
                refined_sources,
                &refined_radical,
            )?;
            numerator.divide(&denominator)
        },
    )
}

/// Materializes one exact quotient of two retained tensor values.
///
/// The numerator and denominator stay in their common selected-root tensor
/// until the output relation is constructed.  This is important for
/// projective constructions such as a retained line-line intersection: first
/// eliminating the two values independently can discard the cancellation
/// which proves that the denominator is nonzero on the authored tuple.
pub fn represented_tensor_ratio(
    numerator: &DenseTensorPolynomial,
    denominator: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> RepresentedOutcome<AlgebraicRootRepresentation> {
    let rank = sources.len() + 1;
    if [numerator, denominator].into_iter().any(|polynomial| {
        polynomial.dimensions().len() != rank || polynomial.dimensions().last() != Some(&1)
    }) {
        return RepresentedOutcome::Unsupported;
    }
    // A rank-one tensor quotient is an ordinary rational function of one
    // selected algebraic root. Cancel its exact polynomial content before
    // invoking the general tensor-image eliminator. Recursive procedural
    // geometry commonly arrives as `L(alpha) * H(alpha) / H(alpha)`; exposing
    // the affine/Mobius image avoids manufacturing a high-degree resultant
    // for a value already carried by the source field.
    if sources.len() == 1
        && numerator.dimensions().len() == 2
        && numerator.dimensions()[1] == 1
        && denominator.dimensions().len() == 2
        && denominator.dimensions()[1] == 1
        && let Some(common) = greatest_common_divisor_univariate_polynomials_exact(
            numerator.coefficients(),
            denominator.coefficients(),
        )
        && let (Some(numerator), Some(denominator)) = (
            divide_univariate_polynomial_exact(numerator.coefficients(), &common),
            divide_univariate_polynomial_exact(denominator.coefficients(), &common),
        )
    {
        if common.len() > 1 {
            let Some(common) = DenseTensorPolynomial::from_axis_polynomial(2, 0, &common) else {
                return RepresentedOutcome::Unsupported;
            };
            if let Some(uncertain) = represented_dense_nonzero(&common, sources).uncertain() {
                return uncertain;
            }
        }
        if numerator.len() == 1
            && denominator.len() == 1
            && let Ok(value) = &numerator[0] / &denominator[0]
        {
            return RepresentedOutcome::Decided(AlgebraicRootRepresentation::from_exact_value(
                &value,
            ));
        }
        if numerator.len() <= 2 && denominator.len() <= 2 {
            let report = transform_algebraic_root_mobius(
                &sources[0],
                numerator.get(1).cloned().unwrap_or_else(Real::zero),
                numerator.first().cloned().unwrap_or_else(Real::zero),
                denominator.get(1).cloned().unwrap_or_else(Real::zero),
                denominator.first().cloned().unwrap_or_else(Real::zero),
                PredicatePolicy::STRICT,
            );
            if report.status == AlgebraicRootMobiusTransformStatus::Transformed
                && let Some(representation) = report.representation
            {
                return RepresentedOutcome::Decided(representation);
            }
        }
    }
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        rank,
        sources.len(),
        &[Real::zero(), Real::one()],
    ) else {
        return RepresentedOutcome::Unsupported;
    };
    let Some(relation) = denominator
        .multiply(&output)
        .and_then(|product| product.subtract(numerator))
    else {
        return RepresentedOutcome::Unsupported;
    };
    for refinement_steps in [0, 4, 8, 16, 32, 64, 128] {
        let refined_sources = sources
            .iter()
            .map(|source| refined_represented_root(source, refinement_steps))
            .collect::<Vec<_>>();
        let (Some(numerator), Some(denominator)) = (
            dense_tensor_interval(numerator, &refined_sources),
            dense_tensor_interval(denominator, &refined_sources),
        ) else {
            continue;
        };
        let Some(interval) = numerator.divide(&denominator) else {
            continue;
        };
        if let RepresentedOutcome::Decided(value) = represented_tensor_coordinate(
            &relation,
            &refined_sources,
            &interval.lower,
            &interval.upper,
        ) {
            return RepresentedOutcome::Decided(value);
        }
    }
    if let Some(uncertain) = represented_dense_nonzero(denominator, sources).uncertain() {
        return uncertain;
    }
    represented_tensor_coordinate_refined(
        &relation,
        sources,
        256,
        128,
        "represented-ratio-image-separation",
        |refined_sources, _| {
            let numerator = dense_tensor_interval(numerator, refined_sources)?;
            let denominator = dense_tensor_interval(denominator, refined_sources)?;
            numerator.divide(&denominator)
        },
    )
}

/// Orders a represented value against an exact real: a certified STRICT
/// order, else the policy sign of their exact difference.
pub fn represented_order_to_real(
    value: &AlgebraicRootRepresentation,
    target: &Real,
    policy: &impl ApproximationPolicy,
) -> RepresentedOutcome<Ordering> {
    if let Some(order) = represented_strict_order(
        value,
        &AlgebraicRootRepresentation::from_exact_value(target),
    ) {
        return RepresentedOutcome::Decided(order);
    }
    match represented_affine_coordinate(&[(value, &Real::one())], &(-target)) {
        RepresentedOutcome::Decided(difference) => represented_policy_sign(&difference, policy)
            .map(|sign| match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            }),
        uncertain => uncertain.map(|_| unreachable!("only a decided outcome maps its value")),
    }
}

/// Intersects the lines `a*x + b*y + c = 0` given by two coefficient triples
/// at a represented root tuple, keeping both coordinates as exact ratios of
/// one shared determinant. A vanishing or undecided determinant is reported
/// as undecided.
pub fn represented_projective_line_intersection(
    first: [DenseTensorPolynomial; 3],
    second: [DenseTensorPolynomial; 3],
    sources: &[AlgebraicRootRepresentation],
) -> RepresentedOutcome<[AlgebraicRootRepresentation; 2]> {
    let [first_a, first_b, first_c] = first;
    let [second_a, second_b, second_c] = second;
    let Some((x_numerator, y_numerator, denominator)) = (|| {
        let denominator = first_a
            .multiply(&second_b)?
            .subtract(&second_a.multiply(&first_b)?)?;
        let x_numerator = first_b
            .multiply(&second_c)?
            .subtract(&second_b.multiply(&first_c)?)?;
        let y_numerator = first_c
            .multiply(&second_a)?
            .subtract(&second_c.multiply(&first_a)?)?;
        Some((x_numerator, y_numerator, denominator))
    })() else {
        return RepresentedOutcome::Unsupported;
    };
    let x = represented_tensor_ratio(&x_numerator, &denominator, sources);
    let y = represented_tensor_ratio(&y_numerator, &denominator, sources);
    match (x, y) {
        (RepresentedOutcome::Decided(x), RepresentedOutcome::Decided(y)) => {
            RepresentedOutcome::Decided([x, y].map(|coordinate| {
                compact_algebraic_root_low_degree_witness(&coordinate).unwrap_or(coordinate)
            }))
        }
        (RepresentedOutcome::Unsupported, _) | (_, RepresentedOutcome::Unsupported) => {
            RepresentedOutcome::Unsupported
        }
        _ => RepresentedOutcome::Undecided,
    }
}
