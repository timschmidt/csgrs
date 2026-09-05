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
use crate::algebraic_fiber::{
    AlgebraicFiberProjectionReport, AlgebraicFiberProjectionStatus,
    project_bivariate_fiber_at_algebraic_parameter,
};
use crate::curve_resultant::{
    BivariatePolynomial, CurveResultantParameter, primitive_common_fiber_component,
};
use crate::root_isolation::{
    IsolatedRootInterval, IsolatedRootRefinementStatus, RootIsolationConfig, polynomial_div_rem,
    polynomial_has_no_distinct_root_in_closed_interval,
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

fn squared_tag_difference_tensor(rank: usize, tag_axis: usize) -> Option<DenseTensorPolynomial> {
    if tag_axis == 0 || tag_axis >= rank {
        return None;
    }
    // Every nonparticipating axis has dimension one, so row-major storage for
    // (tag - source[0])^2 is the same 3 by 3 grid for every supported rank.
    let mut dimensions = Vec::new();
    dimensions.try_reserve_exact(rank).ok()?;
    dimensions.resize(rank, 1);
    dimensions[0] = 3;
    dimensions[tag_axis] = 3;
    let mut coefficients = vec![Real::zero(); 9];
    coefficients[2] = Real::one();
    coefficients[4] = Real::from(-2);
    coefficients[6] = Real::one();
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

fn primitive_first_parameter_component_from_flat_rows(
    retained_count: usize,
    fiber_count: usize,
    coefficients: Vec<Real>,
) -> Option<BivariatePolynomial> {
    if retained_count == 0
        || fiber_count == 0
        || retained_count.checked_mul(fiber_count)? != coefficients.len()
    {
        return None;
    }
    // The tensor is already owned and row-major in [retained, fiber] order.
    // Move its scalars into the primitive component's fiber-major input rather
    // than nesting them, cloning a transpose, and discarding the first grid.
    let mut fibers = Vec::new();
    fibers.try_reserve_exact(fiber_count).ok()?;
    for _ in 0..fiber_count {
        let mut fiber = Vec::new();
        fiber.try_reserve_exact(retained_count).ok()?;
        fibers.push(fiber);
    }
    let mut coefficients = coefficients.into_iter();
    for _ in 0..retained_count {
        for fiber in &mut fibers {
            fiber.push(coefficients.next()?);
        }
    }
    debug_assert!(coefficients.next().is_none());
    primitive_common_fiber_component(fibers, CurveResultantParameter::First)
}

/// Projects the final axis of one selected algebraic tensor after saturating
/// target-wide conjugate components.
///
/// Ordinary sequential norms can be identically zero when *any* foreign
/// source tuple makes `relation` vanish for every target value, even though
/// the authored selected tuple owns a nonzero polynomial. This construction
/// adds a retained tag `z` and eliminates the sum of squares
///
/// `relation(source, target)^2 + (z - source[0])^2`.
///
/// Every target-wide source component then contributes only retained-axis
/// content. Exact polynomial GCD and division remove that content before the
/// remaining bivariate relation is projected through the first selected root.
/// The resulting univariate polynomial is an enumerator: it can contain
/// foreign or complex-source candidates, and callers must replay every root
/// against the complete selected tuple. Construction and saturation are
/// always `STRICT`; no approximate predicate selects a factor or root.
///
/// The caller must first prove that the authored selected fiber is not
/// identically zero. A zero selected fiber is a component, not a finite root
/// set, and therefore has no univariate projection.
pub fn project_selected_tensor_fiber_via_tagged_norm(
    relation: &DenseTensorPolynomial,
    source_roots: &[AlgebraicRootRepresentation],
) -> AlgebraicFiberProjectionReport {
    let projection_report = |status| AlgebraicFiberProjectionReport {
        status,
        coefficients: Vec::new(),
    };
    if source_roots.is_empty() || relation.dimensions().len() != source_roots.len() + 1 {
        return projection_report(AlgebraicFiberProjectionStatus::InvalidEvidence);
    }
    if source_roots.iter().any(|source| {
        !source.is_valid()
            || validate_algebraic_root_representation(source, PredicatePolicy::STRICT).status
                != AlgebraicRootValidationStatus::Valid
    }) {
        return projection_report(AlgebraicFiberProjectionStatus::InvalidEvidence);
    }
    let mut constraints: Vec<Vec<Real>> = Vec::new();
    if constraints.try_reserve_exact(source_roots.len()).is_err() {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    }
    for (source_index, source) in source_roots.iter().enumerate() {
        // Conjugate selections commonly share one defining polynomial. Exact
        // structural equality is sufficient to reuse its proved square-free
        // constraint; a differently represented polynomial takes the full
        // independent path.
        if source_index > 0
            && source_roots[0].polynomial_coefficients == source.polynomial_coefficients
        {
            constraints.push(constraints[0].clone());
            continue;
        }
        let Some(constraint) = square_free_part(
            source.polynomial_coefficients.clone(),
            PredicatePolicy::STRICT,
        )
        .map(canonicalize_proven_rational_coefficients) else {
            return projection_report(AlgebraicFiberProjectionStatus::UnsupportedCoefficient);
        };
        constraints.push(constraint);
    }

    let source_count = source_roots.len();
    let rank = source_count + 2;
    let Some(lifted_relation) = relation.insert_independent_axis(source_count) else {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    };
    let Some(tag_difference_square) = squared_tag_difference_tensor(rank, source_count) else {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    };
    let Some(mut tagged_relation) = lifted_relation
        .multiply(&lifted_relation)
        .and_then(|square| square.add(&tag_difference_square))
    else {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    };

    for (axis, constraint) in constraints.iter().enumerate() {
        let Some(reduced) =
            tagged_relation.reduce_axis_modulo(axis, constraint, PredicatePolicy::STRICT)
        else {
            return projection_report(AlgebraicFiberProjectionStatus::UnsupportedCoefficient);
        };
        tagged_relation = reduced;
    }
    tagged_relation = canonicalize_proven_rational_tensor(tagged_relation);

    for (source_index, constraint) in constraints.iter().enumerate() {
        tagged_relation = if let Some(independent) = tagged_relation
            .remove_certified_independent_axis(0, PredicatePolicy::MAX_REFINEMENT_PRECISION)
        {
            independent
        } else {
            let elimination = resultant_tensor_polynomial_univariate_constraint(
                &tagged_relation,
                constraint,
                0,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            );
            if elimination.status != TensorConstraintResultantStatus::Constructed {
                return projection_report(AlgebraicFiberProjectionStatus::Undecided);
            }
            elimination
                .resultant
                .expect("a constructed tagged tensor resultant retains its polynomial")
        };
        tagged_relation = canonicalize_proven_rational_tensor(tagged_relation);
        for (axis, remaining_constraint) in constraints.iter().skip(source_index + 1).enumerate() {
            let Some(reduced) = tagged_relation.reduce_axis_modulo(
                axis,
                remaining_constraint,
                PredicatePolicy::STRICT,
            ) else {
                return projection_report(AlgebraicFiberProjectionStatus::UnsupportedCoefficient);
            };
            tagged_relation = reduced;
        }
        tagged_relation = canonicalize_proven_rational_tensor(tagged_relation);
    }

    let (dimensions, coefficients) = tagged_relation.into_parts();
    let [tag_count, target_count] = dimensions.as_slice() else {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    };
    let Some(primitive) =
        primitive_first_parameter_component_from_flat_rows(*tag_count, *target_count, coefficients)
    else {
        return projection_report(AlgebraicFiberProjectionStatus::Undecided);
    };
    let first_constraint = constraints
        .into_iter()
        .next()
        .expect("a nonempty selected source tuple retains its first constraint");
    let mut square_free_projection_root = None;
    if first_constraint.len() < source_roots[0].polynomial_coefficients.len() {
        // All preceding reductions already used this exact square-free
        // constraint. Reusing it for the final norm removes only repeated
        // carrier multiplicity and preserves the enumerated root set.
        let source = &source_roots[0];
        let mut root = AlgebraicRootRepresentation {
            constraint_index: source.constraint_index,
            symbol: source.symbol,
            interval_index: source.interval_index,
            polynomial_coefficients: first_constraint,
            interval: source.interval.clone(),
            kind: source.kind.clone(),
            validation: source.validation.clone(),
        };
        root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
        if !root.is_valid() {
            return projection_report(AlgebraicFiberProjectionStatus::Undecided);
        }
        square_free_projection_root = Some(root);
    }
    let projection_root = square_free_projection_root
        .as_ref()
        .unwrap_or(&source_roots[0]);
    let max_source_degree = projection_root
        .polynomial_coefficients
        .len()
        .saturating_sub(1);
    let mut projection = project_bivariate_fiber_at_algebraic_parameter(
        &primitive,
        CurveResultantParameter::First,
        projection_root,
        max_source_degree,
    );
    if projection.status == AlgebraicFiberProjectionStatus::Constructed {
        projection.coefficients =
            canonicalize_proven_rational_coefficients(projection.coefficients);
    }
    projection
}

/// Reduces a selected root to an exact rational or pure-quadratic point when a
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
        let positive = (-factor[0].clone()).sqrt().ok()?;
        let inside = |candidate: &Real| {
            matches!(
                compare_reals(&root.interval.lower, candidate, PredicatePolicy::STRICT,).value(),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
            ) && matches!(
                compare_reals(candidate, &root.interval.upper, PredicatePolicy::STRICT,).value(),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
            )
        };
        let negative = -positive.clone();
        let witness = match (inside(&negative), inside(&positive)) {
            (true, false) => negative,
            (false, true) => positive,
            (false, false) | (true, true) => continue,
        };
        let mut compact = root.clone();
        compact.polynomial_coefficients = factor;
        compact.interval = IsolatedRootInterval {
            lower: witness.clone(),
            upper: witness.clone(),
            exact_root: Some(witness.clone()),
            distinct_root_count: 1,
        };
        compact.kind = if witness.exact_rational_ref().is_some() {
            AlgebraicRootKind::ExactRationalWitness
        } else {
            AlgebraicRootKind::IsolatingInterval
        };
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
    let first_source = source_roots[0].clone();
    let mut source_roots = source_roots.to_vec();
    let mut relation = relation.clone();
    let has_unresolved_source = source_roots
        .iter()
        .any(|source| source.exact_point_witness().is_none());
    // Exact-value and affine substitutions remove a source axis just as
    // conclusively as a later resultant or certified independence proof.
    // Keep the public count faithful on every early-return path, not only
    // after full construction.
    let mut eliminated_source_count = 0;
    let mut source_index = 0;
    while source_index < source_roots.len() {
        let substitution_value =
            source_roots[source_index]
                .exact_point_witness()
                .and_then(|value| {
                    if has_unresolved_source {
                        Some(value.clone())
                    } else {
                        value.exact_rational_normal_form().map(Real::new)
                    }
                });
        if let Some(value) = substitution_value {
            let Some(substituted) = relation.substitute_axis_value(source_index, &value) else {
                return report(
                    AlgebraicTensorImageStatus::InvalidRelationShape,
                    eliminated_source_count,
                    None,
                    None,
                    "an exact tensor-image source axis could not be substituted",
                );
            };
            relation = substituted;
            source_roots.remove(source_index);
            eliminated_source_count += 1;
            continue;
        }
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
                    eliminated_source_count,
                    None,
                    None,
                    "affine-related tensor-image source axes could not be collapsed exactly",
                );
            };
            relation = diagonal;
            source_roots.remove(source_index);
            eliminated_source_count += 1;
        } else {
            source_index += 1;
        }
    }
    let mut constraints: Vec<Vec<Real>> = Vec::with_capacity(source_roots.len());
    for (source_index, source) in source_roots.iter().enumerate() {
        if let Some(shared_index) = source_roots[..source_index]
            .iter()
            .position(|prior| prior.polynomial_coefficients == source.polynomial_coefficients)
        {
            // Distinct selected conjugates can retain the same exact carrier
            // even after affine-related axes have been collapsed. Its proved
            // square-free constraint is independent of the selected interval.
            constraints.push(constraints[shared_index].clone());
            continue;
        }
        let Some(constraint) = square_free_part(
            source.polynomial_coefficients.clone(),
            PredicatePolicy::STRICT,
        ) else {
            return report(
                AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                eliminated_source_count,
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
        // A stored power bound strictly below the divisor degree is already
        // its own quotient-ring remainder. Avoid cloning (or zero-padding)
        // the complete dense tensor for that exact no-op.
        if relation
            .dimensions()
            .get(axis)
            .is_some_and(|dimension| *dimension < constraint.len())
        {
            continue;
        }
        let Some(reduced) = relation.reduce_axis_modulo(axis, constraint, PredicatePolicy::STRICT)
        else {
            return report(
                AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                eliminated_source_count,
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
                    elimination_count: eliminated_source_count,
                    failed_elimination: Some(elimination),
                    representation: None,
                    message: Some(message),
                };
            }
            elimination
                .resultant
                .expect("a constructed tensor resultant retains its polynomial")
        };
        eliminated_source_count += 1;
        relation = canonicalize_proven_rational_tensor(relation);
        for (axis, remaining_constraint) in constraints.iter().skip(source_index + 1).enumerate() {
            if relation
                .dimensions()
                .get(axis)
                .is_some_and(|dimension| *dimension < remaining_constraint.len())
            {
                continue;
            }
            let Some(reduced) =
                relation.reduce_axis_modulo(axis, remaining_constraint, PredicatePolicy::STRICT)
            else {
                return report(
                    AlgebraicTensorImageStatus::SourceSquareFreeFailed,
                    eliminated_source_count,
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
        debug_assert_eq!(eliminated_source_count, original_source_count);
        return report(
            AlgebraicTensorImageStatus::InvalidRelationShape,
            original_source_count,
            None,
            None,
            "tensor-image elimination did not leave one output axis",
        );
    }
    debug_assert_eq!(eliminated_source_count, original_source_count);
    let (_, relation_coefficients) = relation.into_parts();
    let Some(polynomial_coefficients) =
        square_free_part(relation_coefficients, PredicatePolicy::STRICT)
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
        crate::policy_division::reciprocal_with_policy(
            &polynomial_coefficients[1],
            PredicatePolicy::STRICT,
        )
        .map(|reciprocal| -polynomial_coefficients[0].clone() * reciprocal)
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
    // source tuples that are not the authored tuple. Factor the proposed root
    // and certify that its cofactor has no root in the supplied interval;
    // only then is the cardinal candidate the unique possible authored image.
    // Narrow cardinal intervals usually finish in one Horner enclosure, while
    // the exact Sturm fallback rejects intervals containing a foreign root.
    let exact_cardinal_root =
        exact_cardinal_root_in_interval(&polynomial_coefficients, image_interval).and_then(
            |root| {
                let factor = vec![-root.clone(), Real::one()];
                let (quotient, remainder) = polynomial_div_rem(
                    polynomial_coefficients.clone(),
                    &factor,
                    PredicatePolicy::STRICT,
                )?;
                let remainder_is_zero = remainder.iter().all(|coefficient| {
                    coefficient
                        .exact_rational_ref()
                        .is_some_and(|coefficient| coefficient.is_zero())
                });
                (remainder_is_zero
                    && polynomial_has_no_distinct_root_in_closed_interval(
                        &quotient,
                        &image_interval.lower,
                        &image_interval.upper,
                        PredicatePolicy::STRICT,
                    ) == Some(true))
                .then_some(root)
            },
        );
    let exact_rational_root = exact_linear_root.or(exact_cardinal_root);
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

fn canonicalize_proven_rational_coefficients(mut coefficients: Vec<Real>) -> Vec<Real> {
    for coefficient in &mut coefficients {
        // A rational-class Real has no arithmetic DAG to collapse. Preserve
        // its storage directly; Rational operations already consult the lazy
        // canonical-coordinate cache when numeric normalization is required.
        if coefficient.exact_rational_ref().is_some() {
            continue;
        }
        if let Some(rational) = coefficient.exact_rational_normal_form() {
            *coefficient = Real::new(rational);
        }
    }
    coefficients
}

fn canonicalize_proven_rational_tensor(polynomial: DenseTensorPolynomial) -> DenseTensorPolynomial {
    let (dimensions, coefficients) = polynomial.into_parts();
    let coefficients = canonicalize_proven_rational_coefficients(coefficients);
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

    fn exact_root(value: i64) -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: value.unsigned_abs() as usize,
            symbol: SymbolId(value.unsigned_abs() as u32),
            interval_index: 0,
            polynomial_coefficients: vec![real(-value), Real::one()],
            interval: IsolatedRootInterval {
                lower: real(value),
                upper: real(value),
                exact_root: Some(real(value)),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    fn exact_square_root(square: i64) -> AlgebraicRootRepresentation {
        let mut root = square_root(square);
        let value = real(square).sqrt().expect("a positive integer has a root");
        root.interval = IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value),
            distinct_root_count: 1,
        };
        root
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
        assert_eq!(
            compact.exact_point_witness(),
            Some(&(-((real(3) / real(4)).unwrap().sqrt().unwrap())))
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

    fn opposite_conjugate_cubic_fiber_relation() -> DenseTensorPolynomial {
        // (x - y) * (8 t^3 - 1). The selected tuple
        // (sqrt(2), -sqrt(2)) owns one cubic fiber, while the two foreign
        // tuples x=y make the ordinary sequential norm identically zero.
        let dimensions = vec![2, 2, 4];
        let mut coefficients = vec![Real::zero(); dimensions.iter().product()];
        for (exponents, coefficient) in [
            ([1, 0, 0], real(-1)),
            ([1, 0, 3], real(8)),
            ([0, 1, 0], Real::one()),
            ([0, 1, 3], real(-8)),
        ] {
            coefficients[flat_index(&dimensions, &exponents)] = coefficient;
        }
        DenseTensorPolynomial::try_new(dimensions, coefficients).unwrap()
    }

    #[test]
    fn direct_tag_difference_square_matches_tensor_arithmetic() {
        assert!(squared_tag_difference_tensor(3, 0).is_none());
        assert!(squared_tag_difference_tensor(3, 3).is_none());
        for rank in 3..=6 {
            let tag_axis = rank - 2;
            let tag = DenseTensorPolynomial::from_axis_polynomial(
                rank,
                tag_axis,
                &[Real::zero(), Real::one()],
            )
            .unwrap();
            let source =
                DenseTensorPolynomial::from_axis_polynomial(rank, 0, &[Real::zero(), Real::one()])
                    .unwrap();
            let difference = tag.subtract(&source).unwrap();
            assert_eq!(
                squared_tag_difference_tensor(rank, tag_axis).unwrap(),
                difference.multiply(&difference).unwrap()
            );
        }
    }

    #[test]
    fn flat_first_parameter_handoff_matches_public_primitive_component() {
        let rows = vec![
            vec![real(1), real(2), Real::zero(), real(1)],
            vec![Real::zero(), real(1), real(1), Real::zero()],
            vec![real(1), Real::zero(), Real::zero(), real(1)],
        ];
        let expected = crate::curve_resultant::primitive_bivariate_fiber_component_exact(
            &BivariatePolynomial::new(rows.clone()),
            CurveResultantParameter::First,
        );
        let flat = rows.into_iter().flatten().collect();
        assert_eq!(
            primitive_first_parameter_component_from_flat_rows(3, 4, flat),
            expected
        );
        assert!(primitive_first_parameter_component_from_flat_rows(0, 4, Vec::new()).is_none());
        assert!(
            primitive_first_parameter_component_from_flat_rows(2, 2, vec![Real::one()]).is_none()
        );
    }

    #[test]
    fn tagged_norm_projects_nonzero_selected_fiber_past_foreign_wide_components() {
        let positive = square_root(2);
        let mut negative = positive.clone();
        negative.constraint_index = 3;
        negative.symbol = SymbolId(3);
        negative.interval = IsolatedRootInterval {
            lower: real(-2),
            upper: real(-1),
            exact_root: None,
            distinct_root_count: 1,
        };
        negative.validation =
            validate_algebraic_root_representation(&negative, PredicatePolicy::STRICT);
        assert!(negative.is_valid());

        let relation = opposite_conjugate_cubic_fiber_relation();
        let sources = [positive, negative];
        let report = project_selected_tensor_fiber_via_tagged_norm(&relation, &sources);
        assert_eq!(report.status, AlgebraicFiberProjectionStatus::Constructed);
        assert!(report.coefficients.len() > 1);
        assert!(report.coefficients.iter().any(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_none_or(|coefficient| !coefficient.is_zero())
        }));
        let half = (Real::one() / real(2)).unwrap();
        let value = report
            .coefficients
            .iter()
            .rev()
            .fold(Real::zero(), |value, coefficient| {
                value * &half + coefficient
            });
        assert!(
            value
                .exact_rational_ref()
                .is_some_and(|value| value.is_zero())
        );

        let mut repeated_sources = sources;
        let repeated_sqrt_two = vec![
            real(-8),
            Real::zero(),
            real(12),
            Real::zero(),
            real(-6),
            Real::zero(),
            Real::one(),
        ];
        for source in &mut repeated_sources {
            source.polynomial_coefficients = repeated_sqrt_two.clone();
        }
        assert_eq!(
            project_selected_tensor_fiber_via_tagged_norm(&relation, &repeated_sources),
            report
        );
    }

    #[test]
    fn tagged_norm_reuses_a_square_free_exact_witness_carrier() {
        let relation = DenseTensorPolynomial::try_new(
            vec![2, 2],
            vec![Real::zero(), Real::one(), real(-1), Real::zero()],
        )
        .unwrap();
        let square_free = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![real(-1), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: Real::one(),
                exact_root: Some(Real::one()),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let mut repeated = square_free.clone();
        repeated.polynomial_coefficients = vec![real(-1), real(3), real(-3), Real::one()];

        let expected = project_selected_tensor_fiber_via_tagged_norm(
            &relation,
            std::slice::from_ref(&square_free),
        );
        assert_eq!(expected.status, AlgebraicFiberProjectionStatus::Constructed);
        assert_eq!(
            project_selected_tensor_fiber_via_tagged_norm(
                &relation,
                std::slice::from_ref(&repeated),
            ),
            expected
        );

        assert_eq!(
            project_selected_tensor_fiber_via_tagged_norm(&relation, &[]).status,
            AlgebraicFiberProjectionStatus::InvalidEvidence
        );
        let misshaped = DenseTensorPolynomial::try_new(vec![1, 1, 1], vec![Real::zero()]).unwrap();
        assert_eq!(
            project_selected_tensor_fiber_via_tagged_norm(
                &misshaped,
                std::slice::from_ref(&square_free),
            )
            .status,
            AlgebraicFiberProjectionStatus::InvalidEvidence
        );
        let mut invalid = square_free;
        invalid.validation.status = AlgebraicRootValidationStatus::InvalidPolynomial;
        assert_eq!(
            project_selected_tensor_fiber_via_tagged_norm(
                &relation,
                std::slice::from_ref(&invalid),
            )
            .status,
            AlgebraicFiberProjectionStatus::InvalidEvidence
        );
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
    fn tensor_image_substitutes_exact_source_axes_before_elimination() {
        let exact_sources = [exact_root(2), exact_root(3), exact_root(5)];
        let exact_report = represent_algebraic_tensor_image(
            &sum_relation(3, Real::zero()),
            &exact_sources,
            &IsolatedRootInterval {
                lower: real(9),
                upper: real(11),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(exact_report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(exact_report.elimination_count, 3);
        let exact_image = exact_report.representation.unwrap();
        assert_eq!(
            exact_image.polynomial_coefficients,
            vec![real(-10), Real::one()]
        );
        assert_eq!(exact_image.exact_point_witness(), Some(&real(10)));

        let mixed_report = represent_algebraic_tensor_image(
            &sum_relation(2, Real::zero()),
            &[exact_root(2), square_root(2)],
            &IsolatedRootInterval {
                lower: real(3),
                upper: real(4),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(mixed_report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(mixed_report.elimination_count, 2);
        assert_eq!(
            mixed_report.representation.unwrap().polynomial_coefficients,
            vec![real(2), real(-4), Real::one()]
        );
    }

    #[test]
    fn tensor_image_retains_carriers_for_a_complete_nonrational_point_tuple() {
        let report = represent_algebraic_tensor_image(
            &sum_relation(2, Real::zero()),
            &[exact_square_root(2), exact_square_root(3)],
            &IsolatedRootInterval {
                lower: real(3),
                upper: real(4),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        assert_eq!(report.elimination_count, 2);
        assert_eq!(
            report.representation.unwrap().polynomial_coefficients,
            vec![
                Real::one(),
                Real::zero(),
                real(-10),
                Real::zero(),
                Real::one()
            ]
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
    fn tensor_image_counts_collapsed_axes_before_a_later_elimination_failure() {
        let source = AlgebraicRootRepresentation {
            constraint_index: 2,
            symbol: SymbolId(2),
            interval_index: 0,
            polynomial_coefficients: vec![real(-2), Real::zero(), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: real(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let dimensions = vec![3, 3, 2];
        let mut coefficients = vec![Real::zero(); dimensions.iter().product()];
        coefficients[flat_index(&dimensions, &[1, 0, 0])] = real(-1);
        coefficients[flat_index(&dimensions, &[0, 1, 0])] = real(-1);
        coefficients[flat_index(&dimensions, &[0, 0, 1])] = Real::one();
        coefficients[flat_index(&dimensions, &[2, 0, 0])] = crate::test_support::terminal_zero();
        let relation = DenseTensorPolynomial::try_new(dimensions, coefficients).unwrap();

        let report = represent_algebraic_tensor_image(
            &relation,
            &[source.clone(), source],
            &IsolatedRootInterval {
                lower: real(2),
                upper: real(3),
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::EliminationFailed);
        assert_eq!(report.elimination_count, 1);
        assert_eq!(
            report.failed_elimination.unwrap().status,
            TensorConstraintResultantStatus::UndecidedCoefficient
        );
    }

    #[test]
    fn tensor_image_reuses_square_free_constraint_for_selected_conjugates() {
        let polynomial = vec![Real::one(), real(-3), Real::zero(), Real::one()];
        let source = |interval_index, lower, upper| AlgebraicRootRepresentation {
            constraint_index: 31,
            symbol: SymbolId(31),
            interval_index,
            polynomial_coefficients: polynomial.clone(),
            interval: IsolatedRootInterval {
                lower: real(lower),
                upper: real(upper),
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let sources = [source(0, -2, -1), source(1, 0, 1)];
        assert!(
            algebraic_root_affine_relation(&sources[0], &sources[1], PredicatePolicy::STRICT)
                .is_none()
        );
        let relation = sum_relation(2, Real::zero());
        let interval = IsolatedRootInterval {
            lower: real(-2),
            upper: real(-1),
            exact_root: None,
            distinct_root_count: 1,
        };
        let expected = represent_algebraic_tensor_image(&relation, &sources, &interval);
        assert_eq!(expected.status, AlgebraicTensorImageStatus::Transformed);

        let mut repeated = sources;
        for source in &mut repeated {
            source.polynomial_coefficients = vec![
                Real::one(),
                real(-6),
                real(9),
                real(2),
                real(-6),
                Real::zero(),
                Real::one(),
            ];
        }
        assert_eq!(
            represent_algebraic_tensor_image(&relation, &repeated, &interval),
            expected
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
    fn tensor_image_materializes_a_unique_cardinal_factor_of_a_nonlinear_eliminant() {
        let source = square_root(2);
        let source_axis =
            DenseTensorPolynomial::from_axis_polynomial(2, 0, &[Real::zero(), Real::one()])
                .unwrap();
        let output =
            DenseTensorPolynomial::from_axis_polynomial(2, 1, &[Real::zero(), Real::one()])
                .unwrap();
        let relation = output
            .multiply(&output.subtract(&source_axis).unwrap())
            .unwrap();
        let quarter = (Real::one() / real(4)).unwrap();
        let report = represent_algebraic_tensor_image(
            &relation,
            &[source],
            &IsolatedRootInterval {
                lower: -quarter.clone(),
                upper: quarter,
                exact_root: None,
                distinct_root_count: 1,
            },
        );
        assert_eq!(report.status, AlgebraicTensorImageStatus::Transformed);
        let representation = report.representation.unwrap();
        assert_eq!(
            representation.polynomial_coefficients,
            vec![Real::zero(), Real::one()]
        );
        assert_eq!(representation.interval.exact_root, Some(Real::zero()));
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
