//! Exact represented coordinates, ratios, tensor images and circle
//! predicates built from selected algebraic roots.

use super::*;

pub(super) fn represented_coordinate_interval(
    lower: &Real,
    upper: &Real,
) -> Option<IsolatedRootInterval> {
    match compare_reals(lower, upper, &CurveContext::STRICT)? {
        std::cmp::Ordering::Greater => None,
        std::cmp::Ordering::Equal => Some(IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: Some(lower.clone()),
            distinct_root_count: 1,
        }),
        std::cmp::Ordering::Less => Some(IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: None,
            distinct_root_count: 1,
        }),
    }
}

pub(super) fn represented_univariate_coordinate(
    coefficients: &[Real],
    lower: &Real,
    upper: &Real,
    provenance: &AlgebraicRootRepresentation,
) -> Classification<AlgebraicRootRepresentation> {
    if coefficients.len() <= 1 {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let Some(interval) = represented_coordinate_interval(lower, upper) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    let coefficients = if let Some(root) = interval.exact_root.as_ref() {
        if real_sign(&Real::eval_poly(coefficients, root), &CurveContext::STRICT)
            != Some(RealSign::Zero)
        {
            return Classification::Uncertain(UncertaintyReason::Predicate);
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
            return Classification::Uncertain(UncertaintyReason::Predicate);
        };
        let derivative_nonzero = compare_reals(
            &derivative_bounds.lower,
            &Real::zero(),
            &CurveContext::STRICT,
        ) == Some(std::cmp::Ordering::Greater)
            || compare_reals(
                &derivative_bounds.upper,
                &Real::zero(),
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less);
        if !derivative_nonzero {
            return Classification::Uncertain(UncertaintyReason::Predicate);
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
    representation.validation = validate_algebraic_root_representation(
        &representation,
        hypersolve::PredicatePolicy::STRICT,
    );
    if representation.is_valid() {
        Classification::Decided(representation)
    } else {
        Classification::Uncertain(UncertaintyReason::Unsupported)
    }
}

pub(super) fn represented_tensor_coordinate(
    relation: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    lower: &Real,
    upper: &Real,
) -> Classification<AlgebraicRootRepresentation> {
    let Some(interval) = represented_coordinate_interval(lower, upper) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    if sources.is_empty() {
        if relation.dimensions().len() != 1 {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let Some(root) = interval.exact_root.as_ref() else {
            return Classification::Uncertain(UncertaintyReason::Predicate);
        };
        return match real_sign(
            &Real::eval_poly(relation.coefficients(), root),
            &CurveContext::STRICT,
        ) {
            Some(RealSign::Zero) => {
                Classification::Decided(AlgebraicRootRepresentation::from_exact_value(root))
            }
            Some(RealSign::Negative | RealSign::Positive) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            None => Classification::Uncertain(UncertaintyReason::Predicate),
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
        AlgebraicTensorImageStatus::Transformed => Classification::Decided(
            report
                .representation
                .expect("a transformed tensor image retains its representation"),
        ),
        AlgebraicTensorImageStatus::NonIsolatingImageInterval
        | AlgebraicTensorImageStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
        AlgebraicTensorImageStatus::InvalidSourceEvidence
        | AlgebraicTensorImageStatus::InvalidRelationShape
        | AlgebraicTensorImageStatus::SourceSquareFreeFailed
        | AlgebraicTensorImageStatus::EliminationFailed
        | AlgebraicTensorImageStatus::ImageSquareFreeFailed
        | AlgebraicTensorImageStatus::InvalidTransformedEvidence => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

/// Refines selected source isolators until one exact tensor-image root is
/// separated. A repeated source/image state proves that further subdivision
/// cannot add evidence and remains an explicit predicate blocker; otherwise
/// no resource-shaped refinement ceiling changes the mathematical result.
pub(super) fn represented_tensor_coordinate_refined(
    relation: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    initial_refinement_steps: usize,
    _hot_refinement_limit: usize,
    _trace_operation: &'static str,
    mut image_interval: impl FnMut(&[AlgebraicRootRepresentation], usize) -> Option<RealInterval>,
) -> Classification<AlgebraicRootRepresentation> {
    let mut refinement_steps = initial_refinement_steps;
    let mut previous = None;
    #[cfg(feature = "dispatch-trace")]
    if initial_refinement_steps > _hot_refinement_limit {
        hyperreal::dispatch_trace::record(
            "hypercurve",
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
            if compare_reals(&interval.lower, &interval.upper, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
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
                decided @ Classification::Decided(_) => return decided,
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                }
                Classification::Uncertain(_) if unchanged => {
                    return Classification::Uncertain(UncertaintyReason::Predicate);
                }
                Classification::Uncertain(_) => {}
            }
            previous = Some((refined_sources, interval));
        }
        let Some(next_steps) = (if refinement_steps == 0 {
            Some(4)
        } else {
            refinement_steps.checked_mul(2)
        }) else {
            return Classification::Uncertain(UncertaintyReason::Predicate);
        };
        #[cfg(feature = "dispatch-trace")]
        if refinement_steps <= _hot_refinement_limit && next_steps > _hot_refinement_limit {
            hyperreal::dispatch_trace::record(
                "hypercurve",
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
pub(super) fn represented_affine_coordinate(
    terms: &[(&AlgebraicRootRepresentation, &Real)],
    offset: &Real,
) -> Classification<AlgebraicRootRepresentation> {
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
        return Classification::Decided(AlgebraicRootRepresentation::from_exact_value(
            &affine_offset,
        ));
    }
    let affine_image = |source: &AlgebraicRootRepresentation, scale: &Real, offset: &Real| {
        if scale.zero_status() == ZeroKnowledge::Zero {
            return Classification::Decided(AlgebraicRootRepresentation::from_exact_value(offset));
        }
        if scale == &Real::one() && offset.zero_status() == ZeroKnowledge::Zero {
            return Classification::Decided(source.clone());
        }
        let report = transform_algebraic_root_affine(
            source,
            scale.clone(),
            offset.clone(),
            hypersolve::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicRootAffineTransformStatus::Transformed => Classification::Decided(
                report
                    .representation
                    .expect("a transformed affine root retains its representation"),
            ),
            AlgebraicRootAffineTransformStatus::Undecided => {
                Classification::Uncertain(UncertaintyReason::Predicate)
            }
            AlgebraicRootAffineTransformStatus::InvalidEvidence
            | AlgebraicRootAffineTransformStatus::ZeroScale
            | AlgebraicRootAffineTransformStatus::InvalidTransformedEvidence => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
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
            Some(hypersolve::AlgebraicRootAffineRelation {
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut sources = Vec::with_capacity(active.len());
    let mut scales = Vec::with_capacity(active.len());
    for (axis, (source, scale)) in active.iter().enumerate() {
        let Some(term) = DenseTensorPolynomial::from_axis_polynomial(
            rank,
            axis,
            &[Real::zero(), (*scale).clone()],
        ) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(next_relation) = relation.subtract(&term) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
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

pub(super) fn represented_similarity_point(
    point: &[AlgebraicRootRepresentation; 2],
    transform: &Similarity2,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let (a, b, d, e, xoff, yoff) = transform.affine_components();
    let x = represented_affine_coordinate(&[(&point[0], a), (&point[1], b)], xoff);
    let y = represented_affine_coordinate(&[(&point[0], d), (&point[1], e)], yoff);
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => Classification::Decided([x, y]),
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

pub(super) fn represented_similarity_vector(
    vector: &[AlgebraicRootRepresentation; 2],
    transform: &Similarity2,
    output_scale: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let (a, b, d, e, _, _) = transform.affine_components();
    let [a, b, d, e] = [a, b, d, e].map(|coefficient| coefficient * output_scale);
    let x = represented_affine_coordinate(&[(&vector[0], &a), (&vector[1], &b)], &Real::zero());
    let y = represented_affine_coordinate(&[(&vector[0], &d), (&vector[1], &e)], &Real::zero());
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => Classification::Decided([x, y]),
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

pub(super) fn dense_tensor_interval_with_coefficient_precision(
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

pub(super) fn dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
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
    if BezierDenseTwoSquareRootExpression2::polynomial_is_stored_zero(polynomial) {
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

pub(super) fn dense_tensor_interval(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_tensor_interval_with_coefficient_precision(polynomial, sources, None)
}

pub(super) fn refined_represented_root(
    source: &AlgebraicRootRepresentation,
    refinement_steps: usize,
) -> AlgebraicRootRepresentation {
    if refinement_steps == 0 || source.interval.exact_root.is_some() {
        return source.clone();
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record("hypercurve", "represented-root-bounds", "refine");
    let report = refine_isolated_univariate_polynomial_interval(
        &source.polynomial_coefficients,
        &source.interval,
        RootIsolationConfig {
            policy: hypersolve::PredicatePolicy::STRICT,
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
        validate_algebraic_root_representation(&refined, hypersolve::PredicatePolicy::STRICT);
    if refined.is_valid() {
        refined
    } else {
        source.clone()
    }
}

pub(super) fn represented_point_bounds_refined(
    x: &AlgebraicRootRepresentation,
    y: &AlgebraicRootRepresentation,
    refinement_steps: usize,
) -> Classification<Aabb2> {
    if !x.is_valid() || !y.is_valid() {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let x = refined_represented_root(x, refinement_steps);
    let y = refined_represented_root(y, refinement_steps);
    Classification::Decided(Aabb2::new_unchecked(
        Point2::new(x.interval.lower, y.interval.lower),
        Point2::new(x.interval.upper, y.interval.upper),
    ))
}

pub(super) fn represented_dense_value_with_optional_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: Option<i32>,
) -> Classification<AlgebraicRootRepresentation> {
    let dimensions = polynomial.dimensions();
    if dimensions.len() != sources.len() + 1 || dimensions.last() != Some(&1) {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let output_axis = dimensions.len() - 1;
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        dimensions.len(),
        output_axis,
        &[Real::zero(), Real::one()],
    ) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(relation) = output.subtract(polynomial) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(interval) = dense_tensor_interval_with_coefficient_precision(
        polynomial,
        sources,
        coefficient_precision,
    ) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    #[cfg(test)]
    if relation.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!("tensor coordinate direct operation=represented-dense-value");
    }
    represented_tensor_coordinate(&relation, sources, &interval.lower, &interval.upper)
}

pub(super) fn represented_dense_value_with_coefficient_precision(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    coefficient_precision: i32,
) -> Classification<AlgebraicRootRepresentation> {
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

pub(super) fn represented_dense_value(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Classification<AlgebraicRootRepresentation> {
    #[cfg(test)]
    if polynomial.dimensions().len() == 4
        && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
    {
        eprintln!("represented dense value caller=vector-dot-cross");
    }
    represented_dense_value_with_optional_coefficient_precision(polynomial, sources, None)
}

pub(super) fn represented_dense_value_refined(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Classification<AlgebraicRootRepresentation> {
    let dimensions = polynomial.dimensions();
    if dimensions.len() != sources.len() + 1 || dimensions.last() != Some(&1) {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    if sources.is_empty() {
        return Classification::Decided(AlgebraicRootRepresentation::from_exact_value(
            &polynomial.coefficients()[0],
        ));
    }
    let output_axis = dimensions.len() - 1;
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        dimensions.len(),
        output_axis,
        &[Real::zero(), Real::one()],
    ) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(relation) = output.subtract(polynomial) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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

pub(super) fn represented_value_nonzero(
    value: Classification<AlgebraicRootRepresentation>,
) -> Classification<()> {
    let value = match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match represented_policy_sign(&value, &CurveContext::STRICT) {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Classification::Decided(())
        }
        Classification::Decided(RealSign::Zero) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn represented_dense_nonzero(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Classification<()> {
    represented_value_nonzero(represented_dense_value_refined(polynomial, sources))
}

/// Proves equality of two selected algebraic numbers even when their defining
/// eliminants differ. This is STRICT construction evidence: a shared isolated
/// polynomial root or an exactly signed algebraic difference is required.
pub(super) fn represented_roots_strictly_equal(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> bool {
    represented_strict_order(left, right) == Some(std::cmp::Ordering::Equal)
}

pub(super) fn represented_affine_tensor_basis(
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
                hypersolve::AlgebraicRootAffineRelation {
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
                hypersolve::AlgebraicRootAffineRelation { scale, offset }
            } else if represented_roots_strictly_equal(source, coordinate) {
                hypersolve::AlgebraicRootAffineRelation {
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

pub(super) fn represented_tensor_nested_interval(
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

#[allow(clippy::too_many_arguments)]
pub(super) fn represented_tensor_nested_value_refined(
    retained: &DenseTensorPolynomial,
    candidate: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
    initial_refinement_steps: usize,
    hot_refinement_limit: usize,
    trace_operation: &'static str,
) -> Classification<AlgebraicRootRepresentation> {
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(relation) = output.subtract(retained).and_then(|residual| {
        residual
            .multiply(&residual)?
            .subtract(&candidate.multiply(candidate)?.multiply(discriminant)?)
    }) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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

/// Materializes `(A + branch*B*sqrt(S)) / (C + branch*D*sqrt(S))`
/// from one retained tensor authority. The supplied signed radical interval
/// selects the authored square-root sheet; the exact squared relation remains
/// independent of that procedural branch choice.
pub(super) fn represented_tensor_nested_ratio(
    numerator_retained: &DenseTensorPolynomial,
    numerator_candidate: &DenseTensorPolynomial,
    denominator_retained: &DenseTensorPolynomial,
    denominator_candidate: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
) -> Classification<AlgebraicRootRepresentation> {
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
            return Classification::Uncertain(UncertaintyReason::Unsupported);
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
        #[cfg(test)]
        if relation.dimensions().len() == 4
            && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
        {
            eprintln!(
                "tensor coordinate direct operation=represented-nested-ratio steps={refinement_steps}"
            );
        }
        match represented_tensor_coordinate(
            &relation,
            &refined_sources,
            &interval.lower,
            &interval.upper,
        ) {
            decided @ Classification::Decided(_) => return decided,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            Classification::Uncertain(_) => {}
        }
    }
    if let Classification::Uncertain(reason) =
        represented_value_nonzero(represented_tensor_nested_value_refined(
            denominator_retained,
            denominator_candidate,
            discriminant,
            sources,
            signed_radical,
            128,
            64,
            "represented-nested-denominator-separation",
        ))
    {
        return Classification::Uncertain(reason);
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
pub(super) fn represented_tensor_ratio(
    numerator: &DenseTensorPolynomial,
    denominator: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Classification<AlgebraicRootRepresentation> {
    let rank = sources.len() + 1;
    if [numerator, denominator].into_iter().any(|polynomial| {
        polynomial.dimensions().len() != rank || polynomial.dimensions().last() != Some(&1)
    }) {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            if let Classification::Uncertain(reason) = represented_dense_nonzero(&common, sources) {
                return Classification::Uncertain(reason);
            }
        }
        if numerator.len() == 1
            && denominator.len() == 1
            && let Ok(value) = &numerator[0] / &denominator[0]
        {
            return Classification::Decided(AlgebraicRootRepresentation::from_exact_value(&value));
        }
        if numerator.len() <= 2 && denominator.len() <= 2 {
            let report = transform_algebraic_root_mobius(
                &sources[0],
                numerator.get(1).cloned().unwrap_or_else(Real::zero),
                numerator.first().cloned().unwrap_or_else(Real::zero),
                denominator.get(1).cloned().unwrap_or_else(Real::zero),
                denominator.first().cloned().unwrap_or_else(Real::zero),
                hypersolve::PredicatePolicy::STRICT,
            );
            if report.status == AlgebraicRootMobiusTransformStatus::Transformed
                && let Some(representation) = report.representation
            {
                return Classification::Decided(representation);
            }
        }
    }
    let Some(output) = DenseTensorPolynomial::from_axis_polynomial(
        rank,
        sources.len(),
        &[Real::zero(), Real::one()],
    ) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(relation) = denominator
        .multiply(&output)
        .and_then(|product| product.subtract(numerator))
    else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
        #[cfg(test)]
        if relation.dimensions().len() == 4
            && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
        {
            eprintln!(
                "tensor coordinate direct operation=represented-tensor-ratio steps={refinement_steps}"
            );
        }
        match represented_tensor_coordinate(
            &relation,
            &refined_sources,
            &interval.lower,
            &interval.upper,
        ) {
            Classification::Decided(value) => return Classification::Decided(value),
            Classification::Uncertain(_) => {}
        }
    }
    if let Classification::Uncertain(reason) = represented_dense_nonzero(denominator, sources) {
        return Classification::Uncertain(reason);
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

#[allow(clippy::too_many_arguments)]
pub(super) fn represented_tensor_circle_contact_location_parameter(
    unit_radial: &[DenseTensorPolynomial; 2],
    radial_retained: &[DenseTensorPolynomial; 2],
    radial_candidate: &[DenseTensorPolynomial; 2],
    common_denominator: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
    signed_radius: &Real,
    turn: &Real,
    radius_squared: &Real,
) -> CurveResult<
    Classification<
        Option<(
            BezierAlgebraicCuspSemicircleContactLocation2,
            BezierParameter2,
        )>,
    >,
> {
    let dot = |first: &[DenseTensorPolynomial; 2], second: &[DenseTensorPolynomial; 2]| {
        first[0]
            .multiply(&second[0])?
            .add(&first[1].multiply(&second[1])?)
    };
    let cross = |first: &[DenseTensorPolynomial; 2], second: &[DenseTensorPolynomial; 2]| {
        first[0]
            .multiply(&second[1])?
            .subtract(&first[1].multiply(&second[0])?)
    };
    let Some((dot_retained, dot_candidate, cross_retained, cross_candidate)) = (|| {
        let dot_retained = dot(unit_radial, radial_retained)?.scale(signed_radius)?;
        let dot_candidate = dot(unit_radial, radial_candidate)?.scale(signed_radius)?;
        let cross_scale = signed_radius * turn;
        let cross_retained = cross(unit_radial, radial_retained)?.scale(&cross_scale)?;
        let cross_candidate = cross(unit_radial, radial_candidate)?.scale(&cross_scale)?;
        Some((dot_retained, dot_candidate, cross_retained, cross_candidate))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let rank = sources.len() + 1;
    let Some(zero) =
        DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(&Real::zero()))
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let dot_value = represented_tensor_nested_ratio(
        &dot_retained,
        &dot_candidate,
        common_denominator,
        &zero,
        discriminant,
        sources,
        signed_radical,
    );
    let cross_value = represented_tensor_nested_ratio(
        &cross_retained,
        &cross_candidate,
        common_denominator,
        &zero,
        discriminant,
        sources,
        signed_radical,
    );
    let (Classification::Decided(dot_value), Classification::Decided(cross_value)) =
        (dot_value, cross_value)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let Some(cross_sign) = represented_strict_sign(&cross_value) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if cross_sign == RealSign::Negative {
        return Ok(Classification::Decided(None));
    }
    if cross_sign == RealSign::Zero {
        let Some(dot_sign) = represented_strict_sign(&dot_value) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        return match dot_sign {
            RealSign::Positive => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::Start,
                BezierParameter2::Exact(Real::zero()),
            )))),
            RealSign::Negative => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::End,
                BezierParameter2::Exact(Real::one()),
            )))),
            RealSign::Zero => Err(CurveError::Topology(
                "a represented nonzero circle contact had zero diameter coordinates".into(),
            )),
        };
    }
    let radial_complement =
        match represented_affine_coordinate(&[(&dot_value, &Real::one())], radius_squared) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if represented_strict_sign(&radial_complement) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some((parameter_denominator_retained, parameter_denominator_candidate)) = (|| {
        Some((
            common_denominator
                .scale(radius_squared)?
                .add(&dot_retained)?
                .add(&cross_retained)?,
            dot_candidate.add(&cross_candidate)?,
        ))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let parameter = match represented_tensor_nested_ratio(
        &cross_retained,
        &cross_candidate,
        &parameter_denominator_retained,
        &parameter_denominator_candidate,
        discriminant,
        sources,
        signed_radical,
    ) {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = match represented_strict_interior_bezier_parameter(&parameter)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some((
        BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        parameter,
    ))))
}

pub(super) fn represented_strict_order(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> Option<std::cmp::Ordering> {
    let report = compare_algebraic_root_representations_by_difference(
        left,
        right,
        AlgebraicRootRefinementComparisonConfig {
            policy: hypersolve::PredicatePolicy::STRICT,
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

pub(super) fn represented_strict_sign(value: &AlgebraicRootRepresentation) -> Option<RealSign> {
    if compare_reals(&value.interval.upper, &Real::zero(), &CurveContext::STRICT)
        == Some(std::cmp::Ordering::Less)
    {
        return Some(RealSign::Negative);
    }
    if compare_reals(&value.interval.lower, &Real::zero(), &CurveContext::STRICT)
        == Some(std::cmp::Ordering::Greater)
    {
        return Some(RealSign::Positive);
    }
    represented_strict_order(
        value,
        &AlgebraicRootRepresentation::from_exact_value(&Real::zero()),
    )
    .map(|order| match order {
        std::cmp::Ordering::Less => RealSign::Negative,
        std::cmp::Ordering::Equal => RealSign::Zero,
        std::cmp::Ordering::Greater => RealSign::Positive,
    })
}

/// Converts a root already proved to lie strictly between zero and one into a
/// Bezier parameter. A tensor image may initially isolate that root in a wider
/// rational interval; intersecting the isolator with the unit interval keeps
/// the same singleton root and avoids treating harmless bracket slack as an
/// invalid parameter.
pub(super) fn represented_strict_interior_bezier_parameter(
    representation: &AlgebraicRootRepresentation,
) -> CurveResult<Classification<BezierParameter2>> {
    if representation.exact_point_witness().is_some() {
        return BezierParameter2::from_algebraic_root_representation(
            representation,
            &CurveContext::STRICT,
        );
    }
    let mut clipped = representation.clone();
    let zero = Real::zero();
    let one = Real::one();
    match compare_reals(&clipped.interval.lower, &zero, &CurveContext::STRICT) {
        Some(std::cmp::Ordering::Less) => clipped.interval.lower = zero,
        Some(_) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
    }
    match compare_reals(&clipped.interval.upper, &one, &CurveContext::STRICT) {
        Some(std::cmp::Ordering::Greater) => clipped.interval.upper = one,
        Some(_) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
    }
    clipped.validation =
        validate_algebraic_root_representation(&clipped, hypersolve::PredicatePolicy::STRICT);
    if !clipped.is_valid() {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    BezierParameter2::from_algebraic_root_representation(&clipped, &CurveContext::STRICT)
}

pub(super) fn represented_policy_sign(
    value: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Classification<RealSign> {
    if let Some(sign) = represented_strict_sign(value) {
        return Classification::Decided(sign);
    }
    if !policy.permits_approximate_512() {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    }
    let zero = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    let report = compare_algebraic_root_representations_with_refinement(
        value,
        &zero,
        AlgebraicRootRefinementComparisonConfig {
            policy: hypersolve::PredicatePolicy::APPROXIMATE_512,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    let Some(order) = matches!(
        report.comparison.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    )
    .then_some(report.comparison.ordering)
    .flatten() else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    policy.observe_approximate_512();
    Classification::Decided(match order {
        std::cmp::Ordering::Less => RealSign::Negative,
        std::cmp::Ordering::Equal => RealSign::Zero,
        std::cmp::Ordering::Greater => RealSign::Positive,
    })
}

pub(super) fn represented_order_to_real(
    value: &AlgebraicRootRepresentation,
    target: &Real,
    policy: &CurveContext,
) -> Classification<std::cmp::Ordering> {
    if let Some(order) = represented_strict_order(
        value,
        &AlgebraicRootRepresentation::from_exact_value(target),
    ) {
        return Classification::Decided(order);
    }
    match represented_affine_coordinate(&[(value, &Real::one())], &(-target)) {
        Classification::Decided(difference) => {
            represented_policy_sign(&difference, policy).map(|sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            })
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn represented_ratio(
    numerator: &AlgebraicRootRepresentation,
    denominator: &AlgebraicRootRepresentation,
) -> Classification<AlgebraicRootRepresentation> {
    let numerator_interval = RealInterval {
        lower: numerator.interval.lower.clone(),
        upper: numerator.interval.upper.clone(),
    };
    let denominator_interval = RealInterval {
        lower: denominator.interval.lower.clone(),
        upper: denominator.interval.upper.clone(),
    };
    let Some(interval) = numerator_interval.divide(&denominator_interval) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    let Some(numerator_axis) =
        DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
    else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(denominator_axis) =
        DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
    else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(output) =
        DenseTensorPolynomial::from_axis_polynomial(3, 2, &[Real::zero(), Real::one()])
    else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Some(relation) = denominator_axis
        .multiply(&output)
        .and_then(|product| product.subtract(&numerator_axis))
    else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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

pub(super) fn represented_circle_contact_location_parameter_from_dot_cross(
    dot: AlgebraicRootRepresentation,
    oriented_cross: AlgebraicRootRepresentation,
    radius_squared: &Real,
) -> CurveResult<
    Classification<
        Option<(
            BezierAlgebraicCuspSemicircleContactLocation2,
            BezierParameter2,
        )>,
    >,
> {
    let Some(cross_sign) = represented_strict_sign(&oriented_cross) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if cross_sign == RealSign::Negative {
        return Ok(Classification::Decided(None));
    }
    if cross_sign == RealSign::Zero {
        let Some(dot_sign) = represented_strict_sign(&dot) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        return match dot_sign {
            RealSign::Positive => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::Start,
                BezierParameter2::Exact(Real::zero()),
            )))),
            RealSign::Negative => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::End,
                BezierParameter2::Exact(Real::one()),
            )))),
            RealSign::Zero => Err(CurveError::Topology(
                "a represented nonzero circle contact had zero diameter coordinates".into(),
            )),
        };
    }
    let radial_complement =
        match represented_affine_coordinate(&[(&dot, &Real::one())], radius_squared) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if represented_strict_sign(&radial_complement) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let denominator = match represented_affine_coordinate(
        &[(&dot, &Real::one()), (&oriented_cross, &Real::one())],
        radius_squared,
    ) {
        Classification::Decided(denominator) => denominator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if represented_strict_sign(&denominator) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let parameter = match represented_ratio(&oriented_cross, &denominator) {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = match represented_strict_interior_bezier_parameter(&parameter)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some((
        BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        parameter,
    ))))
}

pub(super) fn represented_circle_diameter_predicate_sign(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    curve: &RationalBezier2,
    curve_parameter: &BezierParameter2,
    radial_coefficient: &Real,
    parameter_denominator: &Real,
    cache: &BezierAlgebraicCuspSemicircleParameterCache2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let dot = if let Some(dot) =
        cache.cached_represented_diameter_coordinate(curve_parameter, policy)
    {
        dot
    } else {
        let point = match rational_point_evidence_at_parameter(curve, curve_parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point = match represented_point_evidence_coordinates(&point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let difference = |point: &AlgebraicRootRepresentation,
                          center: &AlgebraicRootRepresentation| {
            represented_affine_coordinate(
                &[(point, &Real::one()), (center, &Real::from(-1_i8))],
                &Real::zero(),
            )
        };
        let (Classification::Decided(dx), Classification::Decided(dy)) = (
            difference(&point[0], &frame.center[0]),
            difference(&point[1], &frame.center[1]),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let dot = match represented_vector_dot_cross(&frame.unit_radial, &[dx, dy]) {
            Classification::Decided([dot, _]) => dot,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        cache.retain_represented_diameter_coordinate(curve_parameter.clone(), dot.clone(), policy);
        dot
    };
    let diameter_scale = parameter_denominator * &frame.signed_radius;
    let radius_squared = &frame.signed_radius * &frame.signed_radius;
    let predicate = match represented_affine_coordinate(
        &[(&dot, &diameter_scale)],
        &(-radial_coefficient * radius_squared),
    ) {
        Classification::Decided(predicate) => predicate,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(represented_policy_sign(&predicate, policy))
}

/// Materializes the exact dot and oriented-area products of two represented
/// vectors through one four-source tensor authority. Affine-related component
/// axes collapse in Hypersolve before any remaining resultant elimination.
pub(super) fn represented_vector_dot_cross(
    first: &[AlgebraicRootRepresentation; 2],
    second: &[AlgebraicRootRepresentation; 2],
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let combine = |dot, cross| match (dot, cross) {
        (Classification::Decided(dot), Classification::Decided(cross)) => {
            Classification::Decided([dot, cross])
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
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
        return Classification::Decided(products);
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
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
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let dot = represented_dense_value(&dot, &sources);
    let cross = represented_dense_value(&cross, &sources);
    combine(dot, cross)
}

pub(super) const POSITIVE_UNIT_SCALE: u8 = 1;
pub(super) const NEGATIVE_UNIT_SCALE: u8 = 2;

/// Proves `right = sign * left` directly from already validated polynomial
/// and isolator evidence. Proportional defining polynomials describe the same
/// root set, while the equal or reflected one-root intervals select the same
/// sheet. This avoids rerunning a Sturm common-root proof for representations
/// produced by an exact identity or negation.
pub(super) fn represented_structural_unit_scale(
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
    if compare_reals(
        &expected_lower,
        &right.interval.lower,
        &CurveContext::STRICT,
    ) != Some(std::cmp::Ordering::Equal)
        || compare_reals(
            &expected_upper,
            &right.interval.upper,
            &CurveContext::STRICT,
        ) != Some(std::cmp::Ordering::Equal)
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
            compare_reals(
                &(transformed * right_leading),
                &(right_coefficient * &transformed_leading),
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Equal)
        })
}

/// Returns every unit scale exactly certified by `right = scale * left`.
/// Zero admits both signs; retaining that ambiguity is necessary for axial
/// quarter turns, where either signed relation describes the zero component.
pub(super) fn represented_zero_offset_unit_scales(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> u8 {
    if let (Some(left), Some(right)) = (left.exact_point_witness(), right.exact_point_witness()) {
        let mut scales = 0;
        if compare_reals(left, right, &CurveContext::STRICT) == Some(std::cmp::Ordering::Equal) {
            scales |= POSITIVE_UNIT_SCALE;
        }
        if compare_reals(&(-left), right, &CurveContext::STRICT) == Some(std::cmp::Ordering::Equal)
        {
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
    let reflected = hypersolve::transform_algebraic_root_affine(
        left,
        Real::from(-1_i8),
        Real::zero(),
        hypersolve::PredicatePolicy::STRICT,
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
        && compare_reals(&relation.offset, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
    {
        if compare_reals(&relation.scale, &Real::one(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
        {
            scales |= POSITIVE_UNIT_SCALE;
        }
        if compare_reals(&relation.scale, &Real::from(-1_i8), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
        {
            scales |= NEGATIVE_UNIT_SCALE;
        }
    }
    scales
}

/// Materializes dot and cross for two scaled unit circle radials while
/// retaining their circle correlation. Independently eliminated coordinates
/// can still certify an orthogonal signed permutation exactly. The
/// orientation-preserving permutations reduce to `(+-scale_product, 0)` or
/// `(0, +-scale_product)` without inventing independent coordinate sheets.
pub(super) fn represented_scaled_unit_radial_dot_cross(
    first: &[AlgebraicRootRepresentation; 2],
    second: &[AlgebraicRootRepresentation; 2],
    scale_product: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let direct_x = represented_zero_offset_unit_scales(&first[0], &second[0]);
    let direct_y = represented_zero_offset_unit_scales(&first[1], &second[1]);
    let zero = || AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    if direct_x & POSITIVE_UNIT_SCALE != 0 && direct_y & POSITIVE_UNIT_SCALE != 0 {
        return Classification::Decided([
            AlgebraicRootRepresentation::from_exact_value(scale_product),
            zero(),
        ]);
    }
    if direct_x & NEGATIVE_UNIT_SCALE != 0 && direct_y & NEGATIVE_UNIT_SCALE != 0 {
        return Classification::Decided([
            AlgebraicRootRepresentation::from_exact_value(&(-scale_product)),
            zero(),
        ]);
    }

    let second_x_from_first_y = represented_zero_offset_unit_scales(&first[1], &second[0]);
    let second_y_from_first_x = represented_zero_offset_unit_scales(&first[0], &second[1]);
    if second_x_from_first_y & NEGATIVE_UNIT_SCALE != 0
        && second_y_from_first_x & POSITIVE_UNIT_SCALE != 0
    {
        return Classification::Decided([
            zero(),
            AlgebraicRootRepresentation::from_exact_value(scale_product),
        ]);
    }
    if second_x_from_first_y & POSITIVE_UNIT_SCALE != 0
        && second_y_from_first_x & NEGATIVE_UNIT_SCALE != 0
    {
        return Classification::Decided([
            zero(),
            AlgebraicRootRepresentation::from_exact_value(&(-scale_product)),
        ]);
    }
    match represented_vector_dot_cross(first, second) {
        Classification::Decided([dot, cross]) => {
            let dot = represented_affine_coordinate(&[(&dot, scale_product)], &Real::zero());
            let cross = represented_affine_coordinate(&[(&cross, scale_product)], &Real::zero());
            match (dot, cross) {
                (Classification::Decided(dot), Classification::Decided(cross)) => {
                    Classification::Decided([dot, cross])
                }
                (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                _ => Classification::Uncertain(UncertaintyReason::Predicate),
            }
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn represented_circle_dot_cross_from_exact_radial(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    contact_radial: &[Real; 2],
    turn: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let x_scale = &frame.signed_radius * contact_radial[0].clone();
    let y_scale = &frame.signed_radius * contact_radial[1].clone();
    let dot = represented_affine_coordinate(
        &[
            (&frame.unit_radial[0], &x_scale),
            (&frame.unit_radial[1], &y_scale),
        ],
        &Real::zero(),
    );
    let cross_x_scale = &frame.signed_radius * contact_radial[1].clone();
    let cross_y_scale = -(&frame.signed_radius * contact_radial[0].clone());
    let cross = represented_affine_coordinate(
        &[
            (&frame.unit_radial[0], &cross_x_scale),
            (&frame.unit_radial[1], &cross_y_scale),
        ],
        &Real::zero(),
    );
    let (Classification::Decided(dot), Classification::Decided(cross)) = (dot, cross) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    let oriented_cross = match represented_affine_coordinate(&[(&cross, turn)], &Real::zero()) {
        Classification::Decided(cross) => cross,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    Classification::Decided([dot, oriented_cross])
}

pub(super) fn represented_exact_radial_linear_sign(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    x_scale: &Real,
    y_scale: &Real,
    offset: &Real,
) -> Option<RealSign> {
    let x_scale = x_scale * &frame.signed_radius;
    let y_scale = y_scale * &frame.signed_radius;
    for refinement_steps in [0, 2, 4, 8] {
        let x = refined_represented_root(&frame.unit_radial[0], refinement_steps);
        let y = refined_represented_root(&frame.unit_radial[1], refinement_steps);
        let interval = |source: &AlgebraicRootRepresentation, scale: &Real| {
            RealInterval {
                lower: source.interval.lower.clone(),
                upper: source.interval.upper.clone(),
            }
            .multiply(&RealInterval {
                lower: scale.clone(),
                upper: scale.clone(),
            })
        };
        let value = RealInterval {
            lower: offset.clone(),
            upper: offset.clone(),
        }
        .add(&interval(&x, &x_scale)?)
        .add(&interval(&y, &y_scale)?);
        if compare_reals(&value.upper, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Some(RealSign::Negative);
        }
        if compare_reals(&value.lower, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Some(RealSign::Positive);
        }
        if value.lower == Real::zero() && value.upper == Real::zero() {
            return Some(RealSign::Zero);
        }
    }
    None
}

pub(super) fn represented_circle_contact_location_from_exact_radial(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    contact_radial: &[Real; 2],
    radius_squared: &Real,
    turn: &Real,
) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
    let cross_x = &contact_radial[1] * turn;
    let cross_y = -(&contact_radial[0] * turn);
    match represented_exact_radial_linear_sign(frame, &cross_x, &cross_y, &Real::zero()) {
        Some(RealSign::Positive) => {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )));
        }
        Some(RealSign::Negative) => return Ok(Classification::Decided(None)),
        Some(RealSign::Zero) | None => {}
    }
    let [dot, cross] =
        match represented_circle_dot_cross_from_exact_radial(frame, contact_radial, turn) {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    Ok(
        represented_circle_contact_location_parameter_from_dot_cross(dot, cross, radius_squared)?
            .map(|contact| contact.map(|(location, _)| location)),
    )
}
