//! Recursive quadratic field helpers: bases, embeddings, projections and values.

use super::*;

pub(super) fn recursive_quadratic_pair_base(
    sources: Vec<AlgebraicRootRepresentation>,
    discriminant: DenseTensorPolynomial,
) -> Option<BezierRecursiveQuadraticField2> {
    let one = DenseTensorPolynomial::try_new(vec![1; sources.len()], vec![Real::one()])?;
    BezierRecursiveQuadraticField2::base(sources, discriminant, one)
}

pub(super) fn recursive_quadratic_pair_value(
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    rational: DenseTensorPolynomial,
    radical: DenseTensorPolynomial,
    branch: i8,
) -> Option<BezierRecursiveQuadraticValue2> {
    let zero = DenseTensorPolynomial::zero(vec![1; base.sources.len()])?;
    BezierRecursiveQuadraticValue2::from_base(
        base.clone(),
        TwoSquareRootExpression {
            rational,
            first: radical.scale(&Real::from(branch))?,
            second: zero.clone(),
            product: zero,
        },
    )
}

pub(super) fn recursive_quadratic_rational_value(
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    rational: DenseTensorPolynomial,
) -> Option<BezierRecursiveQuadraticValue2> {
    BezierRecursiveQuadraticValue2::from_base(
        base.clone(),
        TwoSquareRootExpression::from_rational(rational)?,
    )
}

pub(super) fn recursive_quadratic_bases_equivalent(
    first: &BezierRecursiveQuadraticBaseFieldData2,
    second: &BezierRecursiveQuadraticBaseFieldData2,
) -> bool {
    first.sources == second.sources
        && first.first_speed_squared == second.first_speed_squared
        && first.second_speed_squared == second.second_speed_squared
}

pub(super) fn recursive_quadratic_source_union(
    first: &[AlgebraicRootRepresentation],
    second: &[AlgebraicRootRepresentation],
) -> (Vec<AlgebraicRootRepresentation>, Vec<usize>, Vec<usize>) {
    let canonical = |source: &AlgebraicRootRepresentation| {
        hypersolve::compact_algebraic_root_low_degree_witness(source)
            .unwrap_or_else(|| source.clone())
    };
    let mut sources = Vec::with_capacity(first.len().saturating_add(second.len()));
    let mut first_axes = Vec::with_capacity(first.len());
    let mut second_axes = Vec::with_capacity(second.len());
    for source in first {
        let source = canonical(source);
        let axis = sources
            .iter()
            .position(|candidate| candidate == &source)
            .unwrap_or_else(|| {
                sources.push(source);
                sources.len() - 1
            });
        first_axes.push(axis);
    }
    for source in second {
        let source = canonical(source);
        let axis = sources
            .iter()
            .position(|candidate| candidate == &source)
            .unwrap_or_else(|| {
                sources.push(source);
                sources.len() - 1
            });
        second_axes.push(axis);
    }
    (sources, first_axes, second_axes)
}

pub(super) fn recursive_rebase_value_preserving_base(
    value: &BezierRecursiveQuadraticValue2,
    source_base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    target_base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    axes: &[usize],
    embeddings: &[BezierRecursiveQuadraticExtensionEmbedding2],
) -> Option<BezierRecursiveQuadraticValue2> {
    match value.data.as_ref() {
        BezierRecursiveQuadraticValueData2::Base {
            field, expression, ..
        } if Arc::ptr_eq(field, source_base) => {
            let embed = |polynomial: &DenseTensorPolynomial| {
                dense_tensor_embed_axes(polynomial, target_base.sources.len(), axes).and_then(
                    |polynomial| {
                        dense_reduce_selected_tuple_relations(polynomial, &target_base.sources)
                    },
                )
            };
            BezierRecursiveQuadraticValue2::from_base(
                target_base.clone(),
                TwoSquareRootExpression {
                    rational: embed(&expression.rational)?,
                    first: embed(&expression.first)?,
                    second: embed(&expression.second)?,
                    product: embed(&expression.product)?,
                },
            )
        }
        BezierRecursiveQuadraticValueData2::Extension {
            field,
            retained,
            radical,
            ..
        } => {
            let target = embeddings
                .iter()
                .find(|embedding| Arc::ptr_eq(&embedding.source, field))?
                .target
                .clone();
            BezierRecursiveQuadraticValue2::from_extension(
                target,
                recursive_rebase_value_preserving_base(
                    retained,
                    source_base,
                    target_base,
                    axes,
                    embeddings,
                )?,
                recursive_rebase_value_preserving_base(
                    radical,
                    source_base,
                    target_base,
                    axes,
                    embeddings,
                )?,
            )
        }
        BezierRecursiveQuadraticValueData2::Base { .. } => None,
    }
}

pub(super) fn recursive_rebase_field_preserving_base(
    field: &BezierRecursiveQuadraticField2,
    target_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    axes: &[usize],
) -> Option<(
    BezierRecursiveQuadraticField2,
    Arc<BezierRecursiveQuadraticBaseFieldData2>,
    Vec<BezierRecursiveQuadraticExtensionEmbedding2>,
)> {
    let (source_base, source_path) = field.base_and_extension_path();
    if source_base.sources.len() != axes.len() {
        return None;
    }
    let mut target = BezierRecursiveQuadraticField2::Base(target_base.clone());
    let mut embeddings = Vec::with_capacity(source_path.len());
    for source in source_path {
        let radicand = recursive_rebase_value_preserving_base(
            &source.radicand,
            &source_base,
            &target_base,
            axes,
            &embeddings,
        )?;
        let target_field = target.extension(radicand)?;
        let BezierRecursiveQuadraticField2::Extension(target_extension) = &target_field else {
            unreachable!("replaying a recursive generator creates an extension")
        };
        embeddings.push(BezierRecursiveQuadraticExtensionEmbedding2 {
            source,
            target: target_extension.clone(),
        });
        target = target_field;
    }
    Some((target, source_base, embeddings))
}

pub(super) fn recursive_rebase_point_preserving_base(
    point: &BezierRecursiveQuadraticProjectivePoint2,
    source_base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    target_base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    axes: &[usize],
    embeddings: &[BezierRecursiveQuadraticExtensionEmbedding2],
) -> Option<BezierRecursiveQuadraticProjectivePoint2> {
    Some(BezierRecursiveQuadraticProjectivePoint2 {
        x: recursive_rebase_value_preserving_base(
            &point.x,
            source_base,
            target_base,
            axes,
            embeddings,
        )?,
        y: recursive_rebase_value_preserving_base(
            &point.y,
            source_base,
            target_base,
            axes,
            embeddings,
        )?,
        denominator: recursive_rebase_value_preserving_base(
            &point.denominator,
            source_base,
            target_base,
            axes,
            embeddings,
        )?,
    })
}

pub(super) fn recursive_quadratic_base_generator(
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    first: bool,
) -> Option<BezierRecursiveQuadraticValue2> {
    let dimensions = vec![1; base.sources.len()];
    let zero = DenseTensorPolynomial::zero(dimensions.clone())?;
    let one = DenseTensorPolynomial::try_new(dimensions, vec![Real::one()])?;
    BezierRecursiveQuadraticValue2::from_base(
        base.clone(),
        TwoSquareRootExpression {
            rational: zero.clone(),
            first: if first { one.clone() } else { zero.clone() },
            second: if first { zero.clone() } else { one },
            product: zero,
        },
    )
}

pub(super) struct BezierRecursiveQuadraticForeignBaseEmbedding2 {
    pub(super) source_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    pub(super) target_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    pub(super) axes: Vec<usize>,
    pub(super) first_root: BezierRecursiveQuadraticValue2,
    pub(super) second_root: BezierRecursiveQuadraticValue2,
    pub(super) extensions: Vec<BezierRecursiveQuadraticExtensionEmbedding2>,
}

/// Returns `sqrt(scale)` when `value == scale * reference` coefficientwise
/// and the scalar is strictly positive. Recursive field joins use this to
/// recognize the speed-squared polynomials carried through a similarity:
/// their positive roots differ only by this exact positive factor and must
/// not be adjoined as independent generators.
pub(super) fn dense_exact_positive_scale(
    value: &DenseTensorPolynomial,
    reference: &DenseTensorPolynomial,
) -> Option<Real> {
    if value.dimensions().len() != reference.dimensions().len() {
        return None;
    }
    let zero = Real::zero();
    let (pivot, reference_coefficient) =
        reference
            .coefficients()
            .iter()
            .enumerate()
            .find(|(_, coefficient)| {
                real_sign(coefficient, &CurveContext::STRICT)
                    .is_some_and(|sign| sign != RealSign::Zero)
            })?;
    let mut remaining = pivot;
    let mut exponents = vec![0_usize; reference.dimensions().len()];
    for axis in (0..reference.dimensions().len()).rev() {
        exponents[axis] = remaining % reference.dimensions()[axis];
        remaining /= reference.dimensions()[axis];
    }
    let value_coefficient = value.coefficient(&exponents).unwrap_or(&zero);
    let scale = (value_coefficient / reference_coefficient).ok()?;
    if real_sign(&scale, &CurveContext::STRICT) != Some(RealSign::Positive) {
        return None;
    }
    let difference = value.subtract(&reference.scale(&scale)?)?;
    if difference
        .coefficients()
        .iter()
        .any(|coefficient| real_sign(coefficient, &CurveContext::STRICT) != Some(RealSign::Zero))
    {
        return None;
    }
    Some(scale)
}

pub(super) fn dense_positive_square_root_scale(
    value: &DenseTensorPolynomial,
    reference: &DenseTensorPolynomial,
) -> Option<Real> {
    dense_exact_positive_scale(value, reference)?.sqrt().ok()
}

pub(super) fn recursive_foreign_base_root(
    polynomial: DenseTensorPolynomial,
    target_base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    field: BezierRecursiveQuadraticField2,
    prior: Option<(&DenseTensorPolynomial, &BezierRecursiveQuadraticValue2)>,
    policy: &CurveContext,
) -> CurveResult<
    Classification<
        Option<(
            BezierRecursiveQuadraticField2,
            BezierRecursiveQuadraticValue2,
        )>,
    >,
> {
    let Some(reduced) =
        dense_reduce_selected_tuple_relations(polynomial.clone(), &target_base.sources)
    else {
        return Ok(Classification::Decided(None));
    };
    for (reference, first) in [
        (&target_base.first_speed_squared, true),
        (&target_base.second_speed_squared, false),
    ] {
        if let Some(scale) = dense_positive_square_root_scale(&reduced, reference) {
            return Ok(Classification::Decided(
                recursive_quadratic_base_generator(target_base, first)
                    .and_then(|root| root.scale(&scale))
                    .map(|root| (field, root)),
            ));
        }
    }
    if let Some((prior_polynomial, prior_root)) = prior
        && let Some(prior_polynomial) =
            dense_reduce_selected_tuple_relations(prior_polynomial.clone(), &target_base.sources)
        && let Some(scale) = dense_positive_square_root_scale(&reduced, &prior_polynomial)
    {
        return Ok(Classification::Decided(
            prior_root.scale(&scale).map(|root| (field, root)),
        ));
    }
    // A positive generator can already be a polynomial in one retained
    // source axis. Raw and regularized PH tangents often differ by such a
    // factor. Replay its square and select its sign at the existing tuple;
    // the polynomial's authored sign is not the positive radical sheet.
    // Preserve the polynomial square before reduction by the selected source
    // relations: a reduced square need not be a square in the polynomial ring.
    let rank = polynomial.dimensions().len();
    if rank > 0
        && polynomial
            .dimensions()
            .iter()
            .filter(|degree| **degree > 1)
            .count()
            <= 1
    {
        let root = policy.bounded_exact_predicate_pass(|| -> CurveResult<Option<_>> {
            let strict = policy.strict_counterpart();
            let Classification::Decided(Some(root)) =
                polynomial_square_root(polynomial.coefficients(), &strict)?
            else {
                return Ok(None);
            };
            let axis = polynomial
                .dimensions()
                .iter()
                .position(|degree| *degree > 1)
                .unwrap_or(0);
            let Some(root) = DenseTensorPolynomial::from_axis_polynomial(rank, axis, &root)
                .and_then(|root| recursive_quadratic_rational_value(target_base, root))
                .and_then(|root| field.lift(&root))
            else {
                return Ok(None);
            };
            Ok(match root.sign(&strict)? {
                Classification::Decided(RealSign::Positive | RealSign::Zero) => Some(root),
                Classification::Decided(RealSign::Negative) => root.scale(&Real::from(-1_i8)),
                Classification::Uncertain(_) => None,
            })
        })?;
        if let Some(root) = root {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-field-generator",
                "retained-polynomial-square-root",
            );
            return Ok(Classification::Decided(Some((field, root))));
        }
    }
    let Some(radicand) = recursive_quadratic_rational_value(target_base, reduced) else {
        return Ok(Classification::Decided(None));
    };
    let Some(radicand) = field.lift(&radicand) else {
        return Ok(Classification::Decided(None));
    };
    // Selecting a field generator constructs exact reusable evidence. The
    // caller's approximate terminal cannot collapse an unresolved root to zero.
    match policy.strict_predicate_pass(|| radicand.sign(policy))? {
        Classification::Decided(RealSign::Positive) => {
            let Some(extension) = field.extension(radicand) else {
                return Ok(Classification::Decided(None));
            };
            let Some(root) = extension.element(
                field.constant(Real::zero()).ok_or_else(|| {
                    CurveError::Topology("a merged recursive field lost its zero".into())
                })?,
                field.constant(Real::one()).ok_or_else(|| {
                    CurveError::Topology("a merged recursive field lost its unit".into())
                })?,
            ) else {
                return Ok(Classification::Decided(None));
            };
            Ok(Classification::Decided(Some((extension, root))))
        }
        Classification::Decided(RealSign::Zero) => Ok(Classification::Decided(
            field.constant(Real::zero()).map(|root| (field, root)),
        )),
        Classification::Decided(RealSign::Negative) => Err(CurveError::Topology(
            "a recursive base retained a negative positive-root radicand".into(),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn recursive_embed_foreign_field(
    source: &BezierRecursiveQuadraticField2,
    target_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    axes: Vec<usize>,
    mut field: BezierRecursiveQuadraticField2,
    policy: &CurveContext,
) -> CurveResult<
    Classification<
        Option<(
            BezierRecursiveQuadraticField2,
            BezierRecursiveQuadraticForeignBaseEmbedding2,
        )>,
    >,
> {
    let (source_base, source_path) = source.base_and_extension_path();
    let embed_polynomial = |polynomial: &DenseTensorPolynomial| {
        dense_tensor_embed_axes(polynomial, target_base.sources.len(), &axes)
    };
    let Some(first_polynomial) = embed_polynomial(&source_base.first_speed_squared) else {
        return Ok(Classification::Decided(None));
    };
    let (next, first_root) = match recursive_foreign_base_root(
        first_polynomial.clone(),
        &target_base,
        field,
        None,
        policy,
    )? {
        Classification::Decided(Some(root)) => root,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    field = next;
    let Some(second_polynomial) = embed_polynomial(&source_base.second_speed_squared) else {
        return Ok(Classification::Decided(None));
    };
    let (next, second_root) = match recursive_foreign_base_root(
        second_polynomial,
        &target_base,
        field,
        Some((&first_polynomial, &first_root)),
        policy,
    )? {
        Classification::Decided(Some(root)) => root,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    field = next;
    let mut embedding = BezierRecursiveQuadraticForeignBaseEmbedding2 {
        source_base,
        target_base,
        axes,
        first_root,
        second_root,
        extensions: Vec::with_capacity(source_path.len()),
    };
    'source_extensions: for source in source_path {
        let Some(radicand) = embedding.value(&source.radicand, &field) else {
            return Ok(Classification::Decided(None));
        };
        let (_, candidates) = field.base_and_extension_path();
        for candidate in candidates {
            let Some(mapped_radicand) = embedding.value(&source.radicand, &candidate.parent) else {
                continue;
            };
            if mapped_radicand.is_stored_equivalent_to(&candidate.radicand) {
                embedding
                    .extensions
                    .push(BezierRecursiveQuadraticExtensionEmbedding2 {
                        source,
                        target: candidate,
                    });
                continue 'source_extensions;
            }
            let Some(difference) = mapped_radicand.subtract(&candidate.radicand) else {
                continue;
            };
            // Generator identity must remain exact under either query policy.
            let sign = match policy.strict_predicate_pass(|| difference.sign(policy))? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(_) => {
                    match policy.strict_predicate_pass(|| {
                        difference.sign_with_projected_zero_fallback(policy)
                    })? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            };
            if sign == RealSign::Zero {
                embedding
                    .extensions
                    .push(BezierRecursiveQuadraticExtensionEmbedding2 {
                        source,
                        target: candidate,
                    });
                continue 'source_extensions;
            }
        }
        let Some(target_field) = field.extension(radicand) else {
            return Ok(Classification::Decided(None));
        };
        let BezierRecursiveQuadraticField2::Extension(target) = &target_field else {
            unreachable!("embedding a recursive generator creates an extension")
        };
        embedding
            .extensions
            .push(BezierRecursiveQuadraticExtensionEmbedding2 {
                source,
                target: target.clone(),
            });
        field = target_field;
    }
    Ok(Classification::Decided(Some((field, embedding))))
}

pub(super) fn recursive_merge_projective_point_fields(
    field: &BezierRecursiveQuadraticField2,
    represented: &[BezierRecursiveQuadraticProjectivePoint2],
    point: &BezierRecursiveQuadraticProjectivePoint2,
    policy: &CurveContext,
) -> CurveResult<
    Classification<
        Option<(
            BezierRecursiveQuadraticField2,
            Vec<BezierRecursiveQuadraticProjectivePoint2>,
            BezierRecursiveQuadraticProjectivePoint2,
        )>,
    >,
> {
    let (first_base, _) = field.base_and_extension_path();
    let (second_base, _) = point.denominator.field().base_and_extension_path();
    let (sources, first_axes, second_axes) =
        recursive_quadratic_source_union(&first_base.sources, &second_base.sources);
    let embed_first = |polynomial: &DenseTensorPolynomial| {
        dense_tensor_embed_axes(polynomial, sources.len(), &first_axes)
            .and_then(|polynomial| dense_reduce_selected_tuple_relations(polynomial, &sources))
    };
    let (Some(first_speed_squared), Some(second_speed_squared)) = (
        embed_first(&first_base.first_speed_squared),
        embed_first(&first_base.second_speed_squared),
    ) else {
        return Ok(Classification::Decided(None));
    };
    let Some(target_base_field) =
        BezierRecursiveQuadraticField2::base(sources, first_speed_squared, second_speed_squared)
    else {
        return Ok(Classification::Decided(None));
    };
    let BezierRecursiveQuadraticField2::Base(target_base) = &target_base_field else {
        unreachable!("a merged recursive field begins at its dense base")
    };
    let Some((rebased_field, source_base, embeddings)) =
        recursive_rebase_field_preserving_base(field, target_base.clone(), &first_axes)
    else {
        return Ok(Classification::Decided(None));
    };
    let Some(mut rebased) = represented
        .iter()
        .map(|point| {
            recursive_rebase_point_preserving_base(
                point,
                &source_base,
                target_base,
                &first_axes,
                &embeddings,
            )
        })
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(Classification::Decided(None));
    };
    let (joined, embedding) = match recursive_embed_foreign_field(
        &point.denominator.field(),
        target_base.clone(),
        second_axes,
        rebased_field,
        policy,
    )? {
        Classification::Decided(Some(joined)) => joined,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let Some(point) = embedding.projective_point(point, &joined) else {
        return Ok(Classification::Decided(None));
    };
    let Some(lifted) = rebased
        .drain(..)
        .map(|point| point.lifted_to(&joined))
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(Classification::Decided(None));
    };
    Ok(Classification::Decided(Some((joined, lifted, point))))
}

pub(super) fn recursive_quadratic_polynomial_scale(
    polynomial: &[BezierRecursiveQuadraticValue2],
    scale: &BezierRecursiveQuadraticValue2,
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    polynomial
        .iter()
        .map(|coefficient| coefficient.multiply(scale))
        .collect()
}

pub(super) fn recursive_quadratic_polynomial_combine(
    first: &[BezierRecursiveQuadraticValue2],
    second: &[BezierRecursiveQuadraticValue2],
    subtract: bool,
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    let field = first.first().or_else(|| second.first())?.field();
    let mut result = Vec::with_capacity(first.len().max(second.len()));
    for index in 0..first.len().max(second.len()) {
        let first = first
            .get(index)
            .cloned()
            .or_else(|| field.constant(Real::zero()))?;
        let second = second
            .get(index)
            .cloned()
            .or_else(|| field.constant(Real::zero()))?;
        result.push(if subtract {
            first.subtract(&second)?
        } else {
            first.add(&second)?
        });
    }
    Some(result)
}

/// Converts a coefficient-field power polynomial to Bernstein form on the
/// authored unit domain.  All basis-change weights are exact rationals, so a
/// one-signed control sequence (zeros included) proves a strict sign at every
/// open-domain parameter without projecting the recursive field.
pub(super) fn recursive_quadratic_power_to_unit_bernstein(
    polynomial: &[BezierRecursiveQuadraticValue2],
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    let degree = polynomial.len().checked_sub(1)?;
    let field = polynomial.first()?.field();
    let integer = |value: usize| Some(Real::from(u64::try_from(value).ok()?));
    let mut degree_binomials = Vec::with_capacity(degree + 1);
    let mut binomial = Real::one();
    for index in 0..=degree {
        degree_binomials.push(binomial.clone());
        if index != degree {
            binomial = (binomial * integer(degree - index)? / integer(index + 1)?).ok()?;
        }
    }

    let mut controls = Vec::with_capacity(degree + 1);
    for index in 0..=degree {
        let mut value = field.constant(Real::zero())?;
        let mut row_binomial = Real::one();
        for power in 0..=index {
            let scale = (&row_binomial / &degree_binomials[power]).ok()?;
            value = value.add(&polynomial[power].scale(&scale)?)?;
            if power != index {
                row_binomial =
                    (row_binomial * integer(index - power)? / integer(power + 1)?).ok()?;
            }
        }
        controls.push(value);
    }
    Some(controls)
}

/// A nonzero one-signed Bernstein sequence is strict throughout `(0, 1)`:
/// every Bernstein basis function is positive there.  Structural zeros are
/// accepted at either endpoint, which is essential for adjacent carriers
/// whose common endpoint may be a stationary support contact.
pub(super) fn recursive_quadratic_open_unit_bernstein_sign(
    polynomial: &[BezierRecursiveQuadraticValue2],
) -> Option<RealSign> {
    let signs = recursive_quadratic_power_to_unit_bernstein(polynomial)?
        .into_iter()
        .map(|control| control.bounded_or_exact_real_witness_sign())
        .collect::<Vec<_>>();
    let mut retained = None;
    for sign in signs {
        let sign = sign?;
        match (retained, sign) {
            (None, RealSign::Positive | RealSign::Negative) => retained = Some(sign),
            (Some(expected), actual @ (RealSign::Positive | RealSign::Negative))
                if expected != actual =>
            {
                return None;
            }
            (_, RealSign::Zero) | (Some(_), _) => {}
        }
    }
    retained
}

/// Certifies that a retained-field polynomial has no root anywhere in the
/// closed unit interval. Endpoint controls must be strictly one-signed;
/// interior zeros are harmless because the endpoint Bernstein basis
/// functions remain positive throughout the open interval.
pub(super) fn recursive_quadratic_closed_unit_bernstein_sign(
    polynomial: &[BezierRecursiveQuadraticValue2],
) -> Option<RealSign> {
    let signs = recursive_quadratic_power_to_unit_bernstein(polynomial)?
        .into_iter()
        .map(|control| control.bounded_or_exact_real_witness_sign())
        .collect::<Option<Vec<_>>>()?;
    let sign = *signs.first()?;
    if !matches!(sign, RealSign::Positive | RealSign::Negative)
        || signs.last() != Some(&sign)
        || signs
            .iter()
            .any(|control| *control != sign && *control != RealSign::Zero)
    {
        return None;
    }
    Some(sign)
}

pub(super) fn recursive_quadratic_polynomial_multiply(
    first: &[BezierRecursiveQuadraticValue2],
    second: &[BezierRecursiveQuadraticValue2],
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    let field = first.first()?.field();
    if second.is_empty() {
        return None;
    }
    let count = first.len().checked_add(second.len())?.checked_sub(1)?;
    let mut result = (0..count)
        .map(|_| field.constant(Real::zero()))
        .collect::<Option<Vec<_>>>()?;
    for (first_power, first) in first.iter().enumerate() {
        for (second_power, second) in second.iter().enumerate() {
            let power = first_power.checked_add(second_power)?;
            result[power] = result[power].add(&first.multiply(second)?)?;
        }
    }
    Some(result)
}

/// Eliminates every recursively retained positive quadratic generator from a
/// polynomial over that field, leaving the selected dense base plus one free
/// parameter axis. The result is an enumerator and must be sheet-replayed.
pub(super) fn recursive_quadratic_polynomial_projection(
    mut coefficients: Vec<BezierRecursiveQuadraticValue2>,
) -> Option<(
    Arc<BezierRecursiveQuadraticBaseFieldData2>,
    DenseTensorPolynomial,
)> {
    loop {
        match coefficients.first()?.data.as_ref() {
            BezierRecursiveQuadraticValueData2::Extension { field, .. } => {
                let mut retained = Vec::with_capacity(coefficients.len());
                let mut radical = Vec::with_capacity(coefficients.len());
                for coefficient in &coefficients {
                    let BezierRecursiveQuadraticValueData2::Extension {
                        field: coefficient_field,
                        retained: coefficient_retained,
                        radical: coefficient_radical,
                        ..
                    } = coefficient.data.as_ref()
                    else {
                        return None;
                    };
                    if !Arc::ptr_eq(field, coefficient_field) {
                        return None;
                    }
                    retained.push(coefficient_retained.clone());
                    radical.push(coefficient_radical.clone());
                }
                while retained.len() > 1
                    && retained
                        .last()
                        .is_some_and(BezierRecursiveQuadraticValue2::is_structurally_zero)
                    && radical
                        .last()
                        .is_some_and(BezierRecursiveQuadraticValue2::is_structurally_zero)
                {
                    retained.pop();
                    radical.pop();
                }
                if radical
                    .iter()
                    .all(BezierRecursiveQuadraticValue2::is_structurally_zero)
                {
                    coefficients = retained;
                    continue;
                }
                if retained
                    .iter()
                    .all(BezierRecursiveQuadraticValue2::is_structurally_zero)
                {
                    // The retained radicand is strictly positive, hence
                    // `R(t) * sqrt(r) = 0` has exactly the zero set `R(t)=0`.
                    coefficients = radical;
                    continue;
                }
                let retained_squared =
                    recursive_quadratic_polynomial_multiply(&retained, &retained)?;
                let radical_squared = recursive_quadratic_polynomial_multiply(&radical, &radical)?;
                let radical_squared =
                    recursive_quadratic_polynomial_scale(&radical_squared, &field.radicand)?;
                coefficients = recursive_quadratic_polynomial_combine(
                    &retained_squared,
                    &radical_squared,
                    true,
                )?;
            }
            BezierRecursiveQuadraticValueData2::Base { field, .. } => {
                let mut expressions = Vec::with_capacity(coefficients.len());
                for coefficient in &coefficients {
                    let BezierRecursiveQuadraticValueData2::Base {
                        field: coefficient_field,
                        expression,
                        ..
                    } = coefficient.data.as_ref()
                    else {
                        return None;
                    };
                    if !Arc::ptr_eq(field, coefficient_field) {
                        return None;
                    }
                    expressions.push(expression);
                }
                let component_is_zero = |select: fn(
                    &TwoSquareRootExpression<DenseTensorPolynomial>,
                ) -> &DenseTensorPolynomial| {
                    expressions.iter().all(|expression| {
                        select(expression)
                            .coefficients()
                            .iter()
                            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
                    })
                };
                let component = |select: fn(
                    &TwoSquareRootExpression<DenseTensorPolynomial>,
                ) -> &DenseTensorPolynomial| {
                    let coefficients = expressions
                        .iter()
                        .map(|expression| select(expression))
                        .collect::<Vec<_>>();
                    dense_tensor_from_polynomial_coefficients(&coefficients)
                };
                let nonzero_components = [
                    component_is_zero(|expression| &expression.rational),
                    component_is_zero(|expression| &expression.first),
                    component_is_zero(|expression| &expression.second),
                    component_is_zero(|expression| &expression.product),
                ]
                .into_iter()
                .filter(|zero| !zero)
                .count();
                if nonzero_components <= 1 {
                    let selected = if !component_is_zero(|expression| &expression.rational) {
                        component(|expression| &expression.rational)
                    } else if !component_is_zero(|expression| &expression.first) {
                        component(|expression| &expression.first)
                    } else if !component_is_zero(|expression| &expression.second) {
                        component(|expression| &expression.second)
                    } else {
                        component(|expression| &expression.product)
                    }?;
                    return Some((field.clone(), selected));
                }
                let expression = TwoSquareRootExpression {
                    rational: component(|expression| &expression.rational)?,
                    first: component(|expression| &expression.first)?,
                    second: component(|expression| &expression.second)?,
                    product: component(|expression| &expression.product)?,
                };
                let first_speed_squared =
                    dense_tensor_with_output_axis(&field.first_speed_squared)?;
                let second_speed_squared =
                    dense_tensor_with_output_axis(&field.second_speed_squared)?;
                let projection = expression.projection(
                    &first_speed_squared,
                    &second_speed_squared,
                    &field.sources,
                )?;
                return Some((field.clone(), projection));
            }
        }
    }
}

pub(super) fn recursive_quadratic_real_polynomial(
    field: &BezierRecursiveQuadraticField2,
    coefficients: &[Real],
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    if coefficients.is_empty() {
        return Some(vec![field.constant(Real::zero())?]);
    }
    coefficients
        .iter()
        .cloned()
        .map(|coefficient| field.constant(coefficient))
        .collect()
}

pub(super) fn recursive_quadratic_polynomial_scale_real(
    polynomial: &[BezierRecursiveQuadraticValue2],
    scale: &Real,
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    polynomial
        .iter()
        .map(|coefficient| coefficient.scale(scale))
        .collect()
}

pub(super) fn recursive_quadratic_target_embedding(
    field: &BezierRecursiveQuadraticField2,
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    target: &AlgebraicRootRepresentation,
) -> Option<BezierRecursiveQuadraticTargetEmbedding2> {
    let (sources, source_axes, target_axes) =
        recursive_quadratic_source_union(&base.sources, std::slice::from_ref(target));
    let embed_base = |polynomial: &DenseTensorPolynomial| {
        let embedded = dense_tensor_embed_axes(polynomial, sources.len(), &source_axes)?;
        Some(dense_reduce_selected_tuple_relations(embedded.clone(), &sources).unwrap_or(embedded))
    };
    let target_field = BezierRecursiveQuadraticField2::base(
        sources.clone(),
        embed_base(&base.first_speed_squared)?,
        embed_base(&base.second_speed_squared)?,
    )?;
    let BezierRecursiveQuadraticField2::Base(target_base) = &target_field else {
        unreachable!("a target-augmented recursive field begins at its dense base")
    };
    let target_base = target_base.clone();
    let (field, source_base, extensions) =
        recursive_rebase_field_preserving_base(field, target_base.clone(), &source_axes)?;
    let target_axis = *target_axes
        .first()
        .expect("one target parameter retains one dense axis");
    let target_polynomial = DenseTensorPolynomial::from_axis_polynomial(
        sources.len(),
        target_axis,
        &[Real::zero(), Real::one()],
    )?;
    let target = recursive_quadratic_rational_value(&target_base, target_polynomial)?;
    Some(BezierRecursiveQuadraticTargetEmbedding2 {
        parameter: field.lift(&target)?,
        field,
        source_base,
        target_base,
        source_axes,
        target_axis,
        extensions,
    })
}

/// Certifies identity on the authored coefficient sheet. A formal norm can
/// vanish on a foreign conjugate without vanishing on the selected field.
pub(super) fn recursive_quadratic_polynomial_is_identically_zero(
    polynomial: &[BezierRecursiveQuadraticValue2],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    policy.strict_predicate_pass(|| {
        for coefficient in polynomial {
            match coefficient.sign(policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(true))
    })
}

pub(super) fn recursive_quadratic_parallel_candidate_evaluation(
    field: &BezierRecursiveQuadraticField2,
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    weight: &[BezierRecursiveQuadraticValue2],
    speed_squared: &[BezierRecursiveQuadraticValue2],
    unit_target_speed: bool,
    target_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierRecursiveQuadraticParallelEvaluation2>>> {
    let Some(embedding) = recursive_quadratic_target_embedding(
        field,
        base,
        &bezier_parameter_root_representation(target_parameter),
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let weight = embedding.polynomial_value(weight).ok_or_else(|| {
        CurveError::Topology("a recursive parallel weight exceeded its field budget".into())
    })?;
    match policy.strict_predicate_pass(|| weight.sign(policy))? {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    if unit_target_speed {
        let speed_field = embedding.field.clone();
        let speed = speed_field.constant(Real::one()).ok_or_else(|| {
            CurveError::Topology("a recursive rational target lost its exact unit speed".into())
        })?;
        return Ok(Classification::Decided(Some(
            BezierRecursiveQuadraticParallelEvaluation2 {
                embedding,
                speed_field,
                speed,
            },
        )));
    }
    let speed_squared = embedding.polynomial_value(speed_squared).ok_or_else(|| {
        CurveError::Topology("a recursive parallel speed exceeded its field budget".into())
    })?;
    match policy.strict_predicate_pass(|| speed_squared.sign(policy))? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "a recursive parallel target had negative speed squared".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let parent = embedding.field.clone();
    let speed_field = parent.extension(speed_squared).ok_or_else(|| {
        CurveError::Topology("a recursive parallel speed could not extend its target field".into())
    })?;
    let speed = speed_field
        .element(
            parent.constant(Real::zero()).ok_or_else(|| {
                CurveError::Topology("a recursive parallel speed lost its zero".into())
            })?,
            parent.constant(Real::one()).ok_or_else(|| {
                CurveError::Topology("a recursive parallel speed lost its unit".into())
            })?,
        )
        .ok_or_else(|| {
            CurveError::Topology("a recursive parallel speed root exceeded its field budget".into())
        })?;
    Ok(Classification::Decided(Some(
        BezierRecursiveQuadraticParallelEvaluation2 {
            embedding,
            speed_field,
            speed,
        },
    )))
}

pub(super) fn recursive_quadratic_polynomial_interval(
    coefficients: &[BezierRecursiveQuadraticValue2],
    target: &RealInterval,
    source_steps: usize,
    coefficient_precision: i32,
) -> Option<RealInterval> {
    let zero = Real::zero();
    let mut value = RealInterval {
        lower: zero.clone(),
        upper: zero,
    };
    for coefficient in coefficients.iter().rev() {
        value = value.multiply(target)?.add(
            &coefficient
                .interval_with_coefficient_precision(source_steps, Some(coefficient_precision))?,
        );
    }
    Some(value)
}

pub(super) fn recursive_quadratic_parameter_interval_sign(
    target_parameter: &BezierParameter2,
    interval_value: impl Fn(&RealInterval, usize, i32) -> Option<RealInterval>,
) -> Option<RealSign> {
    let strict = &CurveContext::STRICT;
    let mut target_refinement = BezierParameterRefinement2::new(target_parameter, strict);
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let target =
            bezier_parameter_root_representation(target_refinement.refine_to(target_steps));
        let source_steps = target_steps.saturating_add(64);
        let coefficient_bits = source_steps.max(64).min(i32::MAX as usize) as i32;
        let interval = RealInterval {
            lower: target.interval.lower,
            upper: target.interval.upper,
        };
        if let Some(sign) = interval_value(&interval, source_steps, -coefficient_bits)
            .as_ref()
            .and_then(dense_strict_interval_sign)
        {
            return Some(sign);
        }
    }
    None
}

pub(super) fn recursive_quadratic_parallel_expression_interval(
    expression: &BezierRecursiveQuadraticParallelExpression2,
    unit_target_speed: bool,
    target: &RealInterval,
    source_steps: usize,
    coefficient_precision: i32,
) -> Option<RealInterval> {
    let rational = recursive_quadratic_polynomial_interval(
        &expression.rational,
        target,
        source_steps,
        coefficient_precision,
    )?;
    let radical = recursive_quadratic_polynomial_interval(
        &expression.radical,
        target,
        source_steps,
        coefficient_precision,
    )?;
    let speed = if unit_target_speed {
        RealInterval {
            lower: Real::one(),
            upper: Real::one(),
        }
    } else {
        recursive_quadratic_polynomial_interval(
            &expression.speed_squared,
            target,
            source_steps,
            coefficient_precision,
        )?
        .nonnegative_square_root(Some(coefficient_precision))?
    };
    Some(rational.add(&radical.multiply(&speed)?))
}

/// Certifies authored-sheet incidence directly on the candidate box.
/// An exactly zero enclosure proves incidence, including multiple roots.
/// Otherwise, opposite strict signs on the target faces give an existence
/// proof for the fixed selected recursive tuple. Since the bracket contains
/// exactly one root of the complete norm projection, it must be the published
/// parameter. A strictly nonzero enclosure rejects a conjugate candidate
/// without constructing its field.
pub(super) fn recursive_quadratic_parallel_expression_root_by_interval(
    expression: &BezierRecursiveQuadraticParallelExpression2,
    unit_target_speed: bool,
    target_parameter: &BezierParameter2,
) -> Option<bool> {
    let strict = &CurveContext::STRICT;
    let mut target_refinement = BezierParameterRefinement2::new(target_parameter, strict);
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let target =
            bezier_parameter_root_representation(target_refinement.refine_to(target_steps));
        let source_steps = target_steps.saturating_add(64);
        let coefficient_bits = source_steps.max(64).min(i32::MAX as usize) as i32;
        let coefficient_precision = -coefficient_bits;
        let interval = RealInterval {
            lower: target.interval.lower.clone(),
            upper: target.interval.upper.clone(),
        };
        let expression_interval = |target| {
            recursive_quadratic_parallel_expression_interval(
                expression,
                unit_target_speed,
                target,
                source_steps,
                coefficient_precision,
            )
        };
        match expression_interval(&interval)
            .as_ref()
            .and_then(dense_strict_interval_sign)
        {
            Some(RealSign::Zero) => return Some(true),
            Some(RealSign::Positive | RealSign::Negative) => return Some(false),
            None => {}
        }
        let lower = RealInterval {
            lower: target.interval.lower.clone(),
            upper: target.interval.lower,
        };
        let upper = RealInterval {
            lower: target.interval.upper.clone(),
            upper: target.interval.upper,
        };
        let lower_sign = expression_interval(&lower)
            .as_ref()
            .and_then(dense_strict_interval_sign);
        let upper_sign = expression_interval(&upper)
            .as_ref()
            .and_then(dense_strict_interval_sign);
        if strict_signs_are_opposite(lower_sign, upper_sign) {
            return Some(true);
        }
    }
    None
}

pub(super) fn recursive_projective_algebraic_point_source(
    point: &RationalBezierAlgebraicPointImage2,
) -> Option<BezierRecursiveQuadraticProjectivePoint2> {
    let sources = vec![point.parameter().clone()];
    let one = DenseTensorPolynomial::try_new(vec![1], vec![Real::one()])?;
    let field = BezierRecursiveQuadraticField2::base(sources, one.clone(), one)?;
    recursive_projective_point_source_in_field(
        &field,
        &BezierRecursiveProjectivePointSource2::Algebraic(point.clone()),
    )
}

pub(super) fn recursive_projective_point_source(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierRecursiveProjectivePointSource2>>> {
    let source = match point {
        CurvePoint2(CurvePointData2::Endpoint(endpoint)) => {
            return match endpoint.resolve(policy)? {
                Classification::Decided(Some(point)) => {
                    recursive_projective_point_source(&point, policy)
                }
                Classification::Decided(None) => Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        CurvePoint2(CurvePointData2::Exact(point)) => {
            Some(BezierRecursiveProjectivePointSource2::Exact(point.clone()))
        }
        CurvePoint2(CurvePointData2::Algebraic(point)) => Some(
            BezierRecursiveProjectivePointSource2::Algebraic(point.clone()),
        ),
        CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
            return Ok(match point.recursive_projective_point(policy)? {
                Classification::Decided(point) => Classification::Decided(
                    point.map(BezierRecursiveProjectivePointSource2::Recursive),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
            return Ok(match point.recursive_projective_point(policy)? {
                Classification::Decided(point) => Classification::Decided(
                    point.map(BezierRecursiveProjectivePointSource2::Recursive),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
            return Ok(match point.recursive_projective_point(policy)? {
                Classification::Decided(point) => Classification::Decided(
                    point.map(BezierRecursiveProjectivePointSource2::Recursive),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
            // Cardinal materialization is only an accelerator here.  Do not
            // ask a composite source chord to rediscover axis alignment by
            // exact endpoint comparison before importing the procedural
            // normalized point; that can construct the same joined field the
            // recursive carrier is specifically meant to avoid.
            if point.data.source.certified_axis_direction().is_some()
                && let Some(cardinal) = point.strict_cardinal_point_evidence(policy)?
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-point",
                    "cardinal-displacement-canonicalized",
                );
                return recursive_projective_point_source(&cardinal, policy);
            }
            return Ok(match point.recursive_projective_point(policy)? {
                Classification::Decided(point) => Classification::Decided(
                    point.map(BezierRecursiveProjectivePointSource2::Recursive),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        CurvePoint2(CurvePointData2::Similarity(point)) => {
            if !policy.accepts_retained_policy(point.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let source = match recursive_projective_point_source(&point.data.source, policy)? {
                Classification::Decided(Some(source)) => source,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let source = match source {
                BezierRecursiveProjectivePointSource2::Exact(source) => {
                    BezierRecursiveProjectivePointSource2::Exact(
                        point.data.transform.transform_point(&source),
                    )
                }
                source @ (BezierRecursiveProjectivePointSource2::Algebraic(_)
                | BezierRecursiveProjectivePointSource2::Recursive(_)) => {
                    let source = match source {
                        BezierRecursiveProjectivePointSource2::Algebraic(source) => {
                            let Some(source) = recursive_projective_algebraic_point_source(&source)
                            else {
                                return Ok(Classification::Decided(None));
                            };
                            source
                        }
                        BezierRecursiveProjectivePointSource2::Recursive(source) => source,
                        BezierRecursiveProjectivePointSource2::Exact(_) => unreachable!(),
                    };
                    let (m00, m01, m10, m11, tx, ty) = point.data.transform.affine_components();
                    let Some(source) = source.transformed_affine(m00, m01, m10, m11, tx, ty) else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    BezierRecursiveProjectivePointSource2::Recursive(source)
                }
            };
            Some(source)
        }
        CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
            return Ok(match point.recursive_projective_point(policy)? {
                Classification::Decided(point) => Classification::Decided(
                    point.map(BezierRecursiveProjectivePointSource2::Recursive),
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
    };
    Ok(Classification::Decided(source))
}

/// Returns the homogeneous-denominator orientation already certified by a
/// retained point's construction.
///
/// Recursive chord, cusp, normalized-displacement, and analytic-parallel
/// points are published only after their denominators have been made
/// positive. Similarities preserve that denominator verbatim. A standalone
/// rational algebraic image is the sole variant whose authored weight may be
/// negative, so it reuses the image's exact selected-parameter predicate.
pub(super) fn recursive_projective_evidence_denominator_sign(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match point {
        CurvePoint2(CurvePointData2::Endpoint(endpoint)) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                recursive_projective_evidence_denominator_sign(&point, policy)
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::RealSign))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        CurvePoint2(CurvePointData2::Exact(_)) => Ok(Classification::Decided(RealSign::Positive)),
        CurvePoint2(CurvePointData2::Algebraic(point)) => point
            .predicate_evaluator(&policy.strict_counterpart())
            .map(|predicate| predicate.map(|predicate| predicate.denominator_sign())),
        CurvePoint2(CurvePointData2::Similarity(point)) => {
            if !policy.accepts_retained_policy(point.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            recursive_projective_evidence_denominator_sign(&point.data.source, policy)
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
            Ok(Classification::Decided(RealSign::Positive))
        }
    }
}

/// Imports all participating points together. A base-field merge must rebase
/// the frame and every earlier point, not just the newest incoming point.
pub(super) fn embed_recursive_projective_point_sources<const N: usize>(
    authority: BezierRecursiveCircleFrame2,
    sources: [BezierRecursiveProjectivePointSource2; N],
    policy: &CurveContext,
) -> CurveResult<
    Classification<
        Option<(
            BezierRecursiveCircleFrame2,
            [BezierRecursiveQuadraticProjectivePoint2; N],
        )>,
    >,
> {
    let mut field = authority.field;
    let mut represented = Vec::with_capacity(N + 2);
    represented.extend([authority.center, authority.support_center]);
    for source in sources {
        if let Some(point) = recursive_projective_point_source_in_field(&field, &source) {
            represented.push(point);
            continue;
        }
        let point = match source {
            BezierRecursiveProjectivePointSource2::Exact(_) => {
                return Ok(Classification::Decided(None));
            }
            BezierRecursiveProjectivePointSource2::Algebraic(algebraic) => {
                let Some(point) = recursive_projective_algebraic_point_source(&algebraic) else {
                    return Ok(Classification::Decided(None));
                };
                point
            }
            BezierRecursiveProjectivePointSource2::Recursive(point) => point,
        };
        let point_field = point.denominator.field();
        if let Some(lifted) = represented
            .iter()
            .map(|point| point.lifted_to(&point_field))
            .collect::<Option<Vec<_>>>()
        {
            field = point_field;
            represented = lifted;
            represented.push(point);
            continue;
        }
        let joined = match field.joined_with(&point_field, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some((joined, embeddings)) = joined
            && let Some(lifted) = represented
                .iter()
                .map(|point| point.lifted_to(&joined))
                .collect::<Option<Vec<_>>>()
            && let Some(point) = point.embedded_to(&joined, &embeddings)
        {
            field = joined;
            represented = lifted;
            represented.push(point);
            continue;
        }
        match recursive_merge_projective_point_fields(&field, &represented, &point, policy)? {
            Classification::Decided(Some((joined, lifted, point))) => {
                field = joined;
                represented = lifted;
                represented.push(point);
            }
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let points = represented.split_off(2).try_into().unwrap_or_else(|_| {
        unreachable!("a frame import preserves the number of participating points")
    });
    let [center, support_center] = represented
        .try_into()
        .unwrap_or_else(|_| unreachable!("a circle frame retains its center and radial anchor"));
    Ok(Classification::Decided(Some((
        BezierRecursiveCircleFrame2 {
            field,
            center,
            support_center,
            normal_denominator: authority.normal_denominator,
        },
        points,
    ))))
}

/// Embeds an exact, singly selected, or ancestral recursive point in an
/// existing quadratic tower. Algebraic points are admitted only when their
/// selected parameter is already one of the base axes; adding an unrelated
/// root would destroy the tower's correlation and belongs in the represented
/// tensor fallback instead.
pub(super) fn recursive_projective_point_source_in_field(
    field: &BezierRecursiveQuadraticField2,
    source: &BezierRecursiveProjectivePointSource2,
) -> Option<BezierRecursiveQuadraticProjectivePoint2> {
    match source {
        BezierRecursiveProjectivePointSource2::Exact(point) => {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: field.constant(point.x().clone())?,
                y: field.constant(point.y().clone())?,
                denominator: field.constant(Real::one())?,
            })
        }
        BezierRecursiveProjectivePointSource2::Algebraic(point) => {
            let (base, _) = field.base_and_extension_path();
            let axis = base
                .sources
                .iter()
                .position(|source| source == point.parameter())?;
            let (x, y, denominator) = point.retained_coordinate_polynomials()?;
            let value = |coefficients: &[Real]| {
                let polynomial = DenseTensorPolynomial::from_axis_polynomial(
                    base.sources.len(),
                    axis,
                    coefficients,
                )?;
                BezierRecursiveQuadraticValue2::from_base(
                    base.clone(),
                    TwoSquareRootExpression::from_rational(polynomial)?,
                )
            };
            BezierRecursiveQuadraticProjectivePoint2 {
                x: value(x)?,
                y: value(y)?,
                denominator: value(denominator)?,
            }
            .lifted_to(field)
        }
        BezierRecursiveProjectivePointSource2::Recursive(point) => point
            .lifted_to(field)
            .or_else(|| point.embedded_to_equivalent_field(field))
            .or_else(|| {
                // The selected axes may agree even when one frame stores a
                // polynomial speed as a radical. Reuse the existing foreign
                // base replay, accepting only an embedding in this very field.
                // New axes or generators belong to the explicit join path.
                let (source_base, _) = point.denominator.field().base_and_extension_path();
                let (target_base, _) = field.base_and_extension_path();
                let axes = source_base
                    .sources
                    .iter()
                    .map(|source| {
                        target_base
                            .sources
                            .iter()
                            .position(|target| target == source)
                    })
                    .collect::<Option<Vec<_>>>()?;
                let strict = CurveContext::STRICT;
                match strict
                    .bounded_exact_predicate_pass(|| {
                        recursive_embed_foreign_field(
                            &point.denominator.field(),
                            target_base,
                            axes,
                            field.clone(),
                            &strict,
                        )
                    })
                    .ok()?
                {
                    Classification::Decided(Some((target, embedding)))
                        if target.same_field(field) =>
                    {
                        embedding.projective_point(point, &target)
                    }
                    _ => None,
                }
            }),
    }
}

/// Recovers a finite parameter on a certified nonsingular quadratic conic.
/// The caller owns incidence on the target support. Homogeneous dual linear
/// forms stay in the point's existing recursive field; no new construction
/// root or global Cartesian projection is required. `None` excludes the
/// target's closed unit chart, including its affine infinity.
pub(crate) fn quadratic_conic_parameter_at_incident_point(
    point: &CurvePoint2,
    target: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CurveParameter2>>> {
    let controls = match target.quadratic_homogeneous_controls(policy)? {
        Classification::Decided(Some(controls)) => controls,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let point = match recursive_projective_evidence_points(&[point], policy)? {
        Classification::Decided(Some(mut points)) => points
            .pop()
            .expect("one conic inverse request retains one projective point"),
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let point = match positive_recursive_projective_point(point)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };

    let cross = |first: &[Real; 3], second: &[Real; 3]| {
        [
            &first[1] * &second[2] - &first[2] * &second[1],
            &first[2] * &second[0] - &first[0] * &second[2],
            &first[0] * &second[1] - &first[1] * &second[0],
        ]
    };
    let dual = [
        cross(&controls[1], &controls[2]),
        cross(&controls[2], &controls[0]),
        cross(&controls[0], &controls[1]),
    ];
    let coordinate = |linear: &[Real; 3]| {
        point
            .x
            .scale(&linear[0])?
            .add(&point.y.scale(&linear[1])?)?
            .add(&point.denominator.scale(&linear[2])?)
    };
    // On the incident conic the dual coordinates are proportional to
    // ((1-u)^2, 2u(1-u), u^2). Thus (b + 2c) / (2(a + b + c))
    // recovers every finite u, including both endpoints. The two separate
    // ratios b/(2a+b) and 2c/(b+2c) introduce avoidable endpoint base points
    // and require deciding a vanishing denominator before changing charts.
    // Combine the linear forms before importing them into the point field.
    let two = Real::from(2_i8);
    let numerator_linear = std::array::from_fn(|axis| &dual[1][axis] + &two * &dual[2][axis]);
    let denominator_linear =
        std::array::from_fn(|axis| &two * (&dual[0][axis] + &dual[1][axis] + &dual[2][axis]));
    let (Some(numerator), Some(denominator)) = (
        coordinate(&numerator_linear),
        coordinate(&denominator_linear),
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let strict = policy.strict_counterpart();
    let (numerator, denominator) = match denominator.sign(&strict)? {
        Classification::Decided(RealSign::Positive) => (numerator, denominator),
        Classification::Decided(RealSign::Negative) => (
            numerator.scale(&Real::from(-1_i8)).ok_or_else(|| {
                CurveError::Topology(
                    "a retained conic inverse exceeded its coefficient-field budget".into(),
                )
            })?,
            denominator.scale(&Real::from(-1_i8)).ok_or_else(|| {
                CurveError::Topology(
                    "a retained conic inverse exceeded its coefficient-field budget".into(),
                )
            })?,
        ),
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = BezierRecursiveQuadraticProjectiveScalar2 {
        numerator,
        denominator,
    };
    let zero_order = match parameter.order_to_real(&Real::zero(), &strict)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let one_order = match parameter.order_to_real(&Real::one(), &strict)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if zero_order == std::cmp::Ordering::Less || one_order == std::cmp::Ordering::Greater {
        return Ok(Classification::Decided(None));
    }
    if zero_order == std::cmp::Ordering::Equal {
        return Ok(Classification::Decided(Some(CurveParameter2::from(
            BezierParameter2::Exact(Real::zero()),
        ))));
    }
    if one_order == std::cmp::Ordering::Equal {
        return Ok(Classification::Decided(Some(CurveParameter2::from(
            BezierParameter2::Exact(Real::one()),
        ))));
    }
    if let Some(exact) = parameter.exact_real_value() {
        return Ok(Classification::Decided(Some(CurveParameter2::from(
            BezierParameter2::Exact(exact),
        ))));
    }
    Ok(BezierRecursiveProjectiveParameter2::new(parameter, policy)?
        .map(|parameter| Some(CurveParameter2::from_recursive_projective(parameter))))
}

/// Imports a fixed set of retained point evidences into their least shared
/// recursive tower. When no input owns a tower yet, the independently
/// selected algebraic parameters form a rational dense base with two trivial
/// positive generators. Exact and base-axis algebraic points then enter as
/// constants or native polynomials, while divergent descendants append only
/// their certified positive generators.
#[track_caller]
pub(super) fn recursive_projective_evidence_points(
    points: &[&CurvePoint2],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierRecursiveQuadraticProjectivePoint2>>>> {
    let mut sources = Vec::with_capacity(points.len());
    for point in points {
        match recursive_projective_point_source(point, policy)? {
            Classification::Decided(Some(source)) => sources.push(source),
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let mut field = if let Some(field) = sources.iter().find_map(|source| match source {
        BezierRecursiveProjectivePointSource2::Recursive(point) => Some(point.denominator.field()),
        BezierRecursiveProjectivePointSource2::Exact(_)
        | BezierRecursiveProjectivePointSource2::Algebraic(_) => None,
    }) {
        field
    } else {
        let mut base_sources = Vec::new();
        for source in &sources {
            let BezierRecursiveProjectivePointSource2::Algebraic(point) = source else {
                continue;
            };
            if !base_sources.contains(point.parameter()) {
                base_sources.push(point.parameter().clone());
            }
        }
        let Some(one) =
            DenseTensorPolynomial::try_new(vec![1; base_sources.len()], vec![Real::one()])
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = BezierRecursiveQuadraticField2::base(base_sources, one.clone(), one)
        else {
            return Ok(Classification::Decided(None));
        };
        field
    };
    let mut represented = Vec::with_capacity(sources.len());
    for source in &sources {
        if let Some(point) = recursive_projective_point_source_in_field(&field, source) {
            represented.push(point);
            continue;
        }
        let source_point = match source {
            BezierRecursiveProjectivePointSource2::Algebraic(point) => {
                let Some(point) = recursive_projective_algebraic_point_source(point) else {
                    return Ok(Classification::Decided(None));
                };
                point
            }
            BezierRecursiveProjectivePointSource2::Recursive(point) => point.clone(),
            BezierRecursiveProjectivePointSource2::Exact(_) => {
                unreachable!("exact points embed in every recursive field")
            }
        };
        let mut point = source_point;
        let (active_base, _) = field.base_and_extension_path();
        let (point_base, _) = point.denominator.field().base_and_extension_path();
        if !Arc::ptr_eq(&active_base, &point_base)
            && recursive_quadratic_bases_equivalent(&active_base, &point_base)
        {
            let Some(rebased) = point.rebased_to_equivalent_base(active_base) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            point = rebased;
        }
        let point_field = point.denominator.field();
        if let Some(lifted) = represented
            .iter()
            .map(|point| point.lifted_to(&point_field))
            .collect::<Option<Vec<_>>>()
        {
            field = point_field;
            represented = lifted;
            represented.push(point.clone());
            continue;
        }
        #[cfg(test)]
        let force_field_join = std::env::var_os("HYPERCURVE_DEBUG_FORCE_RECURSIVE_JOIN").is_some()
            && matches!(
                points,
                [
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(first)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(second)),
                ] if !first.shares_carrier(second)
                    && matches!(first.source_endpoint(), CurvePoint2(CurvePointData2::Exact(_)))
                    && first.source_endpoint().shares_storage(second.source_endpoint())
            );
        #[cfg(not(test))]
        let force_field_join = false;
        if policy.has_bounded_exact_predicate_budget() && !force_field_join {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
                && points.len() == 2
                && points.iter().all(|point| {
                    matches!(
                        point,
                        CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    )
                })
            {
                let (active_base, active_path) = field.base_and_extension_path();
                let (incoming_base, incoming_path) = point_field.base_and_extension_path();
                let shared_sources = active_base
                    .sources
                    .iter()
                    .filter(|source| incoming_base.sources.contains(source))
                    .count();
                eprintln!(
                    "recursive parallel tuple fields bases=({},{}) shared-sources={} extensions=({},{}) same-base={} equivalent-base={}",
                    active_base.sources.len(),
                    incoming_base.sources.len(),
                    shared_sources,
                    active_path.len(),
                    incoming_path.len(),
                    Arc::ptr_eq(&active_base, &incoming_base),
                    recursive_quadratic_bases_equivalent(&active_base, &incoming_base),
                );
            }
            // Divergent recursive descendants require a new exact field
            // compositum.  That is the unbounded cold promotion which the
            // APPROXIMATE_512 preliminary pass is expressly allowed to
            // decline: the enclosing complete kernel immediately replays
            // with its terminal policy enabled.  STRICT still reaches the
            // exact join below, and no approximate decision is consumed
            // here.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-evidence",
                "bounded-before-field-join",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let caller = std::panic::Location::caller();
            eprintln!(
                "recursive evidence field join caller={}:{} points={} selects-approximate={} permits-approximate={}",
                caller.file(),
                caller.line(),
                points.len(),
                policy.selects_approximate_512(),
                policy.permits_approximate_512(),
            );
        }
        let joined = match field.joined_with(&point_field, policy)? {
            Classification::Decided(joined) => joined,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some((joined, embeddings)) = joined else {
            match recursive_merge_projective_point_fields(&field, &represented, &point, policy)? {
                Classification::Decided(Some((joined, mut lifted, point))) => {
                    field = joined;
                    lifted.push(point);
                    represented = lifted;
                    continue;
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let Some(lifted) = represented
            .iter()
            .map(|point| point.lifted_to(&joined))
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(point) = point.embedded_to(&joined, &embeddings) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        field = joined;
        represented = lifted;
        represented.push(point);
    }
    Ok(Classification::Decided(Some(represented)))
}

/// Imports fixed affine points through their selected coordinate witnesses
/// instead of constructing a primitive compositum of divergent recursive
/// towers.  Every coordinate remains an exact selected algebraic number; the
/// dense base merely evaluates their authored isolated tuple directly.
pub(super) fn represented_projective_evidence_points(
    points: &[&CurvePoint2],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierRecursiveQuadraticProjectivePoint2>>>> {
    let mut represented = Vec::with_capacity(points.len().saturating_mul(2));
    for point in points {
        match represented_point_evidence_coordinates(point, policy)? {
            Classification::Decided(coordinates) => represented.extend(coordinates),
            Classification::Uncertain(reason) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    let kind = match point {
                        CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                        CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                        CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                        CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                        CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                        CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                        CurvePoint2(
                            CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_),
                        ) => "similarity",
                    };
                    eprintln!(
                        "represented projective evidence stage=coordinate kind={kind} reason={reason:?}"
                    );
                }
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
        return Ok(Classification::Decided(None));
    };
    let remove_output_axis = |polynomial: DenseTensorPolynomial| {
        polynomial.remove_certified_independent_axis(
            sources.len(),
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        )
    };
    let Some(coordinates) = coordinates
        .into_iter()
        .map(remove_output_axis)
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(Classification::Decided(None));
    };
    let Some(one) = DenseTensorPolynomial::try_new(vec![1; sources.len()], vec![Real::one()])
    else {
        return Ok(Classification::Decided(None));
    };
    let Some(field) = BezierRecursiveQuadraticField2::base(sources, one.clone(), one.clone())
    else {
        return Ok(Classification::Decided(None));
    };
    let BezierRecursiveQuadraticField2::Base(base) = &field else {
        unreachable!("a represented point import begins in its dense base field")
    };
    let Some(denominator) = recursive_quadratic_rational_value(base, one) else {
        return Ok(Classification::Decided(None));
    };
    let mut imported = Vec::with_capacity(points.len());
    for coordinates in coordinates.as_chunks::<2>().0 {
        let (Some(x), Some(y)) = (
            recursive_quadratic_rational_value(base, coordinates[0].clone()),
            recursive_quadratic_rational_value(base, coordinates[1].clone()),
        ) else {
            return Ok(Classification::Decided(None));
        };
        imported.push(BezierRecursiveQuadraticProjectivePoint2 {
            x,
            y,
            denominator: denominator.clone(),
        });
    }
    if imported.len() != points.len() {
        return Ok(Classification::Decided(None));
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "recursive-projective-evidence",
        "represented-flat-base",
    );
    Ok(Classification::Decided(Some(imported)))
}

pub(super) fn recursive_projective_polynomial_value(
    coefficients: &[BezierRecursiveQuadraticValue2],
    scalar: &BezierRecursiveQuadraticProjectiveScalar2,
) -> Option<BezierRecursiveQuadraticValue2> {
    let field = scalar.denominator.field();
    let mut coefficients = coefficients.iter().rev();
    let mut value = field.lift(coefficients.next()?)?;
    let mut denominator_power = field.constant(Real::one())?;
    for coefficient in coefficients {
        denominator_power = denominator_power.multiply(&scalar.denominator)?;
        value = value
            .multiply(&scalar.numerator)?
            .add(&field.lift(coefficient)?.multiply(&denominator_power)?)?;
    }
    Some(value)
}

/// Substitutes one recursive projective scalar for the first axis while
/// preserving a caller-selected common homogeneous degree. The returned
/// coefficients remain a polynomial in the independent second parameter.
pub(super) fn recursive_projective_bivariate_first_parameter_polynomial(
    polynomial: &BivariatePolynomial,
    parameter: &BezierRecursiveProjectiveParameter2,
    first_degree: usize,
) -> Option<Vec<BezierRecursiveQuadraticValue2>> {
    if bivariate_first_active_degree(polynomial) > first_degree {
        return None;
    }
    let coefficient_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(1)
        .max(1);
    (0..coefficient_count)
        .map(|second_power| {
            let first_coefficients = polynomial
                .coefficients
                .iter()
                .map(|row| row.get(second_power).cloned().unwrap_or_else(Real::zero))
                .collect::<Vec<_>>();
            parameter.homogeneous_polynomial_value(&first_coefficients, first_degree)
        })
        .collect()
}

/// Proves that a quadratic has two distinct real roots from exact interval-
/// separated values in its unit domain. A strict sign change is sufficient
/// for a positive discriminant by continuity; failure to find one is only a
/// declined fast certificate and leaves the complete discriminant sign path
/// authoritative.
#[derive(Clone, Debug)]
pub(super) struct BezierRecursiveQuadraticUnitCrossing2 {
    pub(super) start_sign: RealSign,
    pub(super) end_sign: RealSign,
    /// Present only when the quadratic orientation separated under a small
    /// bounded filter.  Opposite endpoint signs already prove one unique,
    /// simple unit root for a polynomial of degree at most two; they do not
    /// require expanding a deep retained field merely to orient its leading
    /// coefficient.
    pub(super) leading_sign: Option<RealSign>,
    pub(super) lower: Real,
    pub(super) upper: Real,
}

pub(super) fn recursive_quadratic_polynomial_strict_unit_crossing(
    field: &BezierRecursiveQuadraticField2,
    coefficients: &[BezierRecursiveQuadraticValue2],
) -> Option<BezierRecursiveQuadraticUnitCrossing2> {
    let [constant, _, quadratic] = coefficients else {
        return None;
    };
    let value_at = |parameter: &Real| {
        let parameter = field.constant(parameter.clone())?;
        let mut value = field.constant(Real::zero())?;
        for coefficient in coefficients.iter().rev() {
            value = value.multiply(&parameter)?.add(coefficient)?;
        }
        Some(value)
    };
    let start_sign = constant.bounded_or_exact_real_witness_sign();
    #[cfg(test)]
    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
        eprintln!(
            "strict unit crossing stage=start-sign sign={start_sign:?} exact-real-witness={}",
            constant
                .exact_real_value_with_retained_witnesses()
                .is_some(),
        );
        if start_sign.is_none() {
            for (index, coefficient) in coefficients.iter().enumerate() {
                let witness = coefficient.exact_real_value_with_retained_witnesses();
                let interval = coefficient.interval_with_coefficient_precision(512, Some(-512));
                eprintln!(
                    "strict unit crossing coefficient={index} structural-zero={} stored-zero={} witness-value={:?} witness-zero={:?} witness-immediate={:?} witness-certified={:?} interval={:?}",
                    coefficient.is_structurally_zero(),
                    coefficient.is_coefficientwise_stored_zero(),
                    witness.as_ref().map(Real::to_f64_lossy),
                    witness.as_ref().map(Real::zero_status),
                    witness.as_ref().and_then(Real::immediate_sign),
                    witness
                        .as_ref()
                        .and_then(|value| value.certified_sign_until(-512).sign()),
                    interval.map(|interval| (
                        interval.lower.to_f64_lossy(),
                        interval.upper.to_f64_lossy(),
                    )),
                );
            }
        }
    }
    let start_sign = start_sign?;
    let end_value = value_at(&Real::one())?;
    let end_sign = end_value.bounded_or_exact_real_witness_sign();
    #[cfg(test)]
    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
        eprintln!(
            "strict unit crossing stage=end-sign sign={end_sign:?} exact-real-witness={}",
            end_value
                .exact_real_value_with_retained_witnesses()
                .is_some(),
        );
    }
    let end_sign = end_sign?;
    if matches!(
        (start_sign, end_sign),
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive)
    ) {
        // Keep the radical-formula fast path when the orientation is cheap,
        // but never let this optional representation choice dominate the
        // exact opposite-sign bracket authority.
        let leading_sign = quadratic
            .bounded_interval_sign(0..=16)
            .filter(|sign| *sign != RealSign::Zero);
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!("strict unit crossing stage=leading-sign sign={leading_sign:?}");
        }
        let mut lower = Real::zero();
        let mut upper = Real::one();
        let mut lower_sign = start_sign;
        // Retain enough construction-time bits for downstream normalized
        // offset predicates. Four bits prove the unique crossing but leave a
        // dependency-wide Cartesian enclosure; twelve still costs only a
        // handful of low-degree retained-field interval signs and avoids a
        // later recursive norm for ordinary separated offset supports.
        for _refinement_index in 0..12 {
            let midpoint = ((&lower + &upper) / Real::from(2_i8)).ok()?;
            let Some(midpoint_value) = value_at(&midpoint) else {
                break;
            };
            let Some(midpoint_sign) = (if midpoint_value.is_structurally_zero() {
                Some(RealSign::Zero)
            } else {
                midpoint_value.bounded_or_exact_real_witness_sign()
            }) else {
                // This refinement is optional. The existing opposite-sign
                // bracket still certifies the unique root of degree at most two.
                break;
            };
            match midpoint_sign {
                RealSign::Zero => {
                    lower = midpoint.clone();
                    upper = midpoint;
                    break;
                }
                sign if sign == lower_sign => {
                    lower = midpoint;
                    lower_sign = sign;
                }
                _ => upper = midpoint,
            }
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "strict unit crossing stage=refinement-complete index={_refinement_index}"
                );
            }
        }
        Some(BezierRecursiveQuadraticUnitCrossing2 {
            start_sign,
            end_sign,
            leading_sign,
            lower,
            upper,
        })
    } else {
        None
    }
}

/// Solves one linear or quadratic polynomial directly in its retained
/// recursive coefficient field. `None` declines the fast path when the active
/// degree or a required STRICT sign cannot be certified; callers may then use
/// their complete projection authority. Every returned denominator is
/// strictly positive and the roots are ordered by their authored real value.
pub(super) fn recursive_quadratic_polynomial_projective_roots(
    field: &BezierRecursiveQuadraticField2,
    coefficients: &[BezierRecursiveQuadraticValue2],
    strict_unit_crossing: Option<&BezierRecursiveQuadraticUnitCrossing2>,
    policy: &CurveContext,
) -> CurveResult<Option<Vec<BezierRecursiveQuadraticProjectiveScalar2>>> {
    let strict = policy.strict_counterpart();
    match coefficients {
        [constant, linear] => {
            let (numerator, denominator) = match linear.sign(&strict)? {
                Classification::Decided(RealSign::Positive) => {
                    (constant.scale(&Real::from(-1_i8)), Some(linear.clone()))
                }
                Classification::Decided(RealSign::Negative) => {
                    (Some(constant.clone()), linear.scale(&Real::from(-1_i8)))
                }
                Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                    return Ok(None);
                }
            };
            let (Some(numerator), Some(denominator)) = (numerator, denominator) else {
                return Ok(None);
            };
            Ok(Some(vec![BezierRecursiveQuadraticProjectiveScalar2 {
                numerator,
                denominator,
            }]))
        }
        [constant, linear, quadratic] => {
            let quadratic_sign = if let Some(crossing) = strict_unit_crossing {
                let Some(sign) = crossing.leading_sign else {
                    return Ok(None);
                };
                sign
            } else {
                let quadratic_sign = quadratic.sign(&strict)?;
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "recursive quadratic roots stage=quadratic-sign result={quadratic_sign:?} exact-real-witness={}",
                        quadratic
                            .exact_real_value_with_retained_witnesses()
                            .is_some(),
                    );
                }
                match quadratic_sign {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                }
            };
            if strict_unit_crossing.is_none()
                && constant.bounded_or_exact_real_witness_sign() == Some(RealSign::Zero)
                && constant
                    .add(linear)
                    .and_then(|value| value.add(quadratic))
                    .and_then(|value| value.bounded_or_exact_real_witness_sign())
                    == Some(RealSign::Zero)
                && let (Some(zero), Some(one)) =
                    (field.constant(Real::zero()), field.constant(Real::one()))
            {
                // A nonzero quadratic vanishing at both native endpoints is
                // a*x*(x-1). Keep those exact scalars instead of adjoining
                // sqrt(a²) and later reconstructing 0 and 1 from that field.
                return Ok(Some(vec![
                    BezierRecursiveQuadraticProjectiveScalar2 {
                        numerator: zero,
                        denominator: one.clone(),
                    },
                    BezierRecursiveQuadraticProjectiveScalar2 {
                        numerator: one.clone(),
                        denominator: one,
                    },
                ]));
            }
            let discriminant = linear
                .square()
                .and_then(|linear_squared| {
                    quadratic
                        .multiply(constant)
                        .and_then(|product| product.scale(&Real::from(4_i8)))
                        .and_then(|product| linear_squared.subtract(&product))
                })
                .ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive quadratic discriminant exceeded its field budget".into(),
                    )
                })?;
            let discriminant_sign = if strict_unit_crossing.is_some() {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-polynomial-roots",
                    "strict-sign-change-discriminant",
                );
                RealSign::Positive
            } else {
                let discriminant_sign = discriminant.sign(&strict)?;
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "recursive quadratic roots stage=discriminant-sign result={discriminant_sign:?} exact-real-witness={}",
                        discriminant
                            .exact_real_value_with_retained_witnesses()
                            .is_some(),
                    );
                }
                match discriminant_sign {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(_) => return Ok(None),
                }
            };
            if discriminant_sign == RealSign::Negative {
                return Ok(Some(Vec::new()));
            }
            let root_field = if discriminant_sign == RealSign::Positive {
                field.extension(discriminant)
            } else {
                Some(field.clone())
            }
            .ok_or_else(|| {
                CurveError::Topology("a recursive quadratic root could not extend its field".into())
            })?;
            let lifted_linear = root_field.lift(linear).ok_or_else(|| {
                CurveError::Topology("a recursive linear term crossed fields".into())
            })?;
            let lifted_quadratic = root_field.lift(quadratic).ok_or_else(|| {
                CurveError::Topology("a recursive quadratic term crossed fields".into())
            })?;
            let denominator_negative = quadratic_sign == RealSign::Negative;
            let mut denominator = lifted_quadratic.scale(&Real::from(2_i8)).ok_or_else(|| {
                CurveError::Topology(
                    "a recursive quadratic denominator exceeded its field budget".into(),
                )
            })?;
            if denominator_negative {
                denominator = denominator.scale(&Real::from(-1_i8)).ok_or_else(|| {
                    CurveError::Topology(
                        "a recursive quadratic denominator exceeded its field budget".into(),
                    )
                })?;
            }
            let radical = if discriminant_sign == RealSign::Positive {
                root_field.element(
                    field.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology("a recursive root field lost its zero".into())
                    })?,
                    field.constant(Real::one()).ok_or_else(|| {
                        CurveError::Topology("a recursive root field lost its unit".into())
                    })?,
                )
            } else {
                root_field.constant(Real::zero())
            }
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive quadratic field lost its discriminant root".into(),
                )
            })?;
            let branches: &[i8] = if discriminant_sign == RealSign::Zero {
                &[0]
            } else {
                &[-1, 1]
            };
            let mut roots = Vec::with_capacity(branches.len());
            for branch in branches {
                let mut numerator = lifted_linear
                    .scale(&Real::from(-1_i8))
                    .and_then(|value| {
                        radical
                            .scale(&Real::from(*branch))
                            .and_then(|root| value.add(&root))
                    })
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive quadratic numerator exceeded its field budget".into(),
                        )
                    })?;
                if denominator_negative {
                    numerator = numerator.scale(&Real::from(-1_i8)).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive quadratic numerator exceeded its field budget".into(),
                        )
                    })?;
                }
                roots.push(BezierRecursiveQuadraticProjectiveScalar2 {
                    numerator,
                    denominator: denominator.clone(),
                });
            }
            if roots.len() == 2 && denominator_negative {
                roots.swap(0, 1);
            }
            Ok(Some(roots))
        }
        _ => Ok(None),
    }
}

pub(super) fn recursive_projective_polynomial_sign_at_parameter(
    field: &BezierRecursiveQuadraticField2,
    coefficients: &[BezierRecursiveQuadraticValue2],
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if let Some(parameter) = parameter.as_recursive_projective() {
        // A selected polynomial root is an exact parameter even when it has
        // no explicit projective scalar. Signs consume its retained root
        // authority directly instead of requiring a field-valued image.
        return parameter.recursive_polynomial_sign(coefficients, policy);
    }
    let Some(parameter) = parameter.as_bezier_parameter() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let base = field.base_and_extension_path().0;
    recursive_quadratic_target_embedding(
        field,
        &base,
        &bezier_parameter_root_representation(parameter),
    )
    .and_then(|embedding| embedding.polynomial_value(coefficients))
    .ok_or_else(|| {
        CurveError::Topology(
            "a recursive polynomial predicate exceeded its retained field budget".into(),
        )
    })?
    .sign(policy)
}

pub(super) enum BezierRecursiveOrderedFieldError2 {
    Curve(CurveError),
    Uncertain,
}

pub(super) struct BezierRecursiveOrderedFieldContext2 {
    pub(super) field: BezierRecursiveQuadraticField2,
    pub(super) policy: CurveContext,
}

impl OrderedFieldPolynomialContext<BezierRecursiveQuadraticValue2>
    for BezierRecursiveOrderedFieldContext2
{
    type Error = BezierRecursiveOrderedFieldError2;

    fn constant(&mut self, value: &Real) -> Result<BezierRecursiveQuadraticValue2, Self::Error> {
        self.field.constant(value.clone()).ok_or_else(|| {
            BezierRecursiveOrderedFieldError2::Curve(CurveError::Topology(
                "a recursive polynomial isolator lost its coefficient-field constant".into(),
            ))
        })
    }

    fn add(
        &mut self,
        left: &BezierRecursiveQuadraticValue2,
        right: &BezierRecursiveQuadraticValue2,
    ) -> Result<BezierRecursiveQuadraticValue2, Self::Error> {
        left.add(right).ok_or_else(|| {
            BezierRecursiveOrderedFieldError2::Curve(CurveError::Topology(
                "a recursive polynomial isolator crossed coefficient fields".into(),
            ))
        })
    }

    fn multiply(
        &mut self,
        left: &BezierRecursiveQuadraticValue2,
        right: &BezierRecursiveQuadraticValue2,
    ) -> Result<BezierRecursiveQuadraticValue2, Self::Error> {
        left.multiply(right).ok_or_else(|| {
            BezierRecursiveOrderedFieldError2::Curve(CurveError::Topology(
                "a recursive polynomial product exceeded its coefficient field".into(),
            ))
        })
    }

    fn scale(
        &mut self,
        value: &BezierRecursiveQuadraticValue2,
        scale: &Real,
    ) -> Result<BezierRecursiveQuadraticValue2, Self::Error> {
        value.scale(scale).ok_or_else(|| {
            BezierRecursiveOrderedFieldError2::Curve(CurveError::Topology(
                "a recursive polynomial isolator exceeded its coefficient field".into(),
            ))
        })
    }

    fn normalize_positive_scale(&mut self, coefficients: &mut [BezierRecursiveQuadraticValue2]) {
        BezierRecursiveQuadraticValue2::normalize_positive_scale(coefficients);
    }

    fn sign(
        &mut self,
        value: &BezierRecursiveQuadraticValue2,
    ) -> Result<std::cmp::Ordering, Self::Error> {
        // The shared isolator and remainder engine consume these signs as
        // exact algebraic evidence, including polynomial degree decisions.
        match self
            .policy
            .strict_predicate_pass(|| value.sign(&self.policy))
            .map_err(BezierRecursiveOrderedFieldError2::Curve)?
        {
            Classification::Decided(RealSign::Negative) => Ok(std::cmp::Ordering::Less),
            Classification::Decided(RealSign::Zero) => Ok(std::cmp::Ordering::Equal),
            Classification::Decided(RealSign::Positive) => Ok(std::cmp::Ordering::Greater),
            Classification::Uncertain(_) => Err(BezierRecursiveOrderedFieldError2::Uncertain),
        }
    }

    fn sign_if_separated(
        &mut self,
        value: &BezierRecursiveQuadraticValue2,
    ) -> Result<Option<std::cmp::Ordering>, Self::Error> {
        if value.is_coefficientwise_stored_zero() || value.is_structurally_zero() {
            return Ok(Some(std::cmp::Ordering::Equal));
        }
        Ok(value
            .bounded_or_exact_real_witness_sign()
            .map(|sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            }))
    }
}

/// Keeps simple roots in the already-selected recursive coefficient field.
/// The shared Hypersolve Bernstein engine is division-free; an unresolved
/// repeated root merely declines so the complete dense projection below can
/// remain the cold authority.
pub(super) fn recursive_quadratic_polynomial_local_parameters(
    field: &BezierRecursiveQuadraticField2,
    coefficients: &[BezierRecursiveQuadraticValue2],
    bounds: [&Real; 2],
    policy: &CurveContext,
) -> CurveResult<Option<Vec<CurveParameter2>>> {
    let mut context = BezierRecursiveOrderedFieldContext2 {
        field: field.clone(),
        policy: *policy,
    };
    let authority = Arc::new(BezierRecursivePolynomialParameterAuthority2::new(
        field.clone(),
        coefficients.to_vec(),
    ));
    let report = match isolate_ordered_field_polynomial_roots(
        authority.coefficients.clone(),
        bounds[0],
        bounds[1],
        OrderedFieldRootIsolationConfig {
            max_subdivision_depth: 512,
            refinement_steps: 8,
        },
        &mut context,
    ) {
        Ok(report) => report,
        Err(BezierRecursiveOrderedFieldError2::Uncertain) => {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("recursive polynomial local isolation stage=sign-uncertain");
            }
            return Ok(None);
        }
        Err(BezierRecursiveOrderedFieldError2::Curve(error)) => return Err(error),
    };
    #[cfg(test)]
    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
        eprintln!(
            "recursive polynomial local isolation stage=complete status={:?} subdivisions={}",
            report.status, report.subdivision_steps,
        );
    }
    match report.status {
        OrderedFieldRootIsolationStatus::Isolated => {}
        OrderedFieldRootIsolationStatus::CompleteFallbackRequired
        | OrderedFieldRootIsolationStatus::IdenticallyZero => return Ok(None),
        OrderedFieldRootIsolationStatus::InvalidInterval => {
            return Err(CurveError::InvalidBezierRange);
        }
    }
    let retained = report
        .intervals
        .into_iter()
        .map(|root| {
            if let Some(value) = root.exact_root {
                CurveParameter2::from(BezierParameter2::Exact(value))
            } else {
                CurveParameter2::from_recursive_projective(BezierRecursiveProjectiveParameter2 {
                    data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                        projection: Arc::default(),
                        authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                            authority: authority.clone(),
                            endpoint_signs: std::array::from_fn(|_| OnceLock::new()),
                        },
                        lower: root.lower,
                        upper: root.upper,
                        refinement_steps: 0,
                        identity: None,
                        line_branch: 0,
                        policy: policy.retained_object_policy(),
                    }),
                })
            }
        })
        .collect();
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "recursive-polynomial-roots",
        "local-bernstein",
    );
    Ok(Some(retained))
}

/// Isolates every root in an original-parameter domain over a retained
/// recursive quadratic field. Linear and quadratic roots remain projective
/// values in that same tower. Higher degrees use the shared dense projection
/// only as an enumerator and replay every candidate on the authored recursive
/// sheet before publishing it.
pub(super) fn recursive_projective_polynomial_parameters_with_crossing(
    field: &BezierRecursiveQuadraticField2,
    mut coefficients: Vec<BezierRecursiveQuadraticValue2>,
    strict_unit_crossing: Option<BezierRecursiveQuadraticUnitCrossing2>,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    while coefficients
        .last()
        .is_some_and(BezierRecursiveQuadraticValue2::is_structurally_zero)
    {
        coefficients.pop();
    }
    let Some(first) = coefficients.first() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    };
    if !field.same_field(&first.field())
        || coefficients
            .iter()
            .any(|coefficient| !field.same_field(&coefficient.field()))
    {
        return Err(CurveError::Topology(
            "a recursive polynomial crossed retained coefficient fields".into(),
        ));
    }
    if coefficients.len() == 1 {
        return Ok(match first.sign(policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                Classification::Decided(Vec::new())
            }
            Classification::Decided(RealSign::Zero) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        });
    }

    let unit_domain = matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit());
    debug_assert!(strict_unit_crossing.is_none() || unit_domain);
    if unit_domain && recursive_quadratic_closed_unit_bernstein_sign(&coefficients).is_some() {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-polynomial-roots",
            "compact-witness-bernstein-no-root",
        );
        return Ok(Classification::Decided(Vec::new()));
    }

    if let Some(crossing) = strict_unit_crossing.as_ref()
        && crossing.leading_sign.is_none()
    {
        let authority = Arc::new(BezierRecursivePolynomialParameterAuthority2::new(
            field.clone(),
            coefficients.clone(),
        ));
        let parameter = BezierRecursiveProjectiveParameter2 {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: Arc::default(),
                authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                    authority,
                    endpoint_signs: if crossing.lower == crossing.upper {
                        [RealSign::Zero; 2]
                    } else {
                        [crossing.start_sign, crossing.end_sign]
                    }
                    .map(OnceLock::from),
                },
                lower: crossing.lower.clone(),
                upper: crossing.upper.clone(),
                refinement_steps: 0,
                identity: None,
                line_branch: 0,
                policy: policy.retained_object_policy(),
            }),
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-polynomial-roots",
            "opposite-sign-bracket",
        );
        return Ok(Classification::Decided(vec![
            CurveParameter2::from_recursive_projective(parameter),
        ]));
    }

    if let Some(scalars) = recursive_quadratic_polynomial_projective_roots(
        field,
        &coefficients,
        strict_unit_crossing.as_ref(),
        policy,
    )? {
        let scalars = match (strict_unit_crossing, scalars.as_slice()) {
            (Some(crossing), [_, _]) => {
                // For `a*t^2+b*t+c`, the sign strictly between the two
                // ordered roots is `-sign(a)`. If zero starts between the
                // roots, the upper root is the unique unit crossing;
                // otherwise the lower root is. The opposite endpoint sign
                // proves that selected root lies strictly before one.
                let between_sign = product_sign(
                    crossing
                        .leading_sign
                        .expect("direct quadratic roots retain a leading orientation"),
                    RealSign::Negative,
                );
                let selected = usize::from(crossing.start_sign == between_sign);
                vec![(
                    scalars[selected].clone(),
                    Some((crossing.lower, crossing.upper)),
                )]
            }
            _ => scalars.into_iter().map(|scalar| (scalar, None)).collect(),
        };
        let mut retained = Vec::with_capacity(scalars.len());
        for (scalar, certified_bounds) in scalars {
            let has_certified_bounds = certified_bounds.is_some();
            let parameter = match BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                scalar,
                certified_bounds,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if !has_certified_bounds {
                match domain.contains_parameter(
                    &CurveParameter2::from_recursive_projective(parameter.clone()),
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let scalar = parameter
                .projective_scalar()
                .expect("a direct recursive polynomial root owns its scalar");
            // The linear/quadratic solver constructs roots in this exact
            // retained field after certifying its nonzero leading term and
            // discriminant. That construction is already the root proof.
            // Evaluating the equation again would discard the formula's
            // cancellation identity inside reconstructed scalar products.
            retained.push(scalar.exact_real_value().map_or_else(
                || CurveParameter2::from_recursive_projective(parameter),
                |value| CurveParameter2::from(BezierParameter2::Exact(value)),
            ));
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-polynomial-roots",
            "direct-projective",
        );
        return Ok(Classification::Decided(retained));
    }

    if let SelectedThirdAxisDomain2::Finite(range) = domain {
        let (_, bounds) = match CurveParameterDomain2::new(range, None).finite_envelope(policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if let Some(candidates) =
            recursive_quadratic_polynomial_local_parameters(field, &coefficients, bounds, policy)?
        {
            if unit_domain {
                return Ok(Classification::Decided(candidates));
            }
            let mut retained = Vec::with_capacity(candidates.len());
            for candidate in candidates {
                match domain.contains_parameter(&candidate, policy)? {
                    Classification::Decided(true) => retained.push(candidate),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            return Ok(Classification::Decided(retained));
        }
    }

    let Some((base, projection)) = recursive_quadratic_polynomial_projection(coefficients.clone())
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    if !recursive_quadratic_bases_equivalent(&base, &field.base_and_extension_path().0) {
        return Err(CurveError::Topology(
            "a recursive polynomial projection changed its retained base".into(),
        ));
    }
    let candidates =
        match selected_dense_last_axis_parameters(&projection, &base.sources, domain, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let candidate = CurveParameter2::from(candidate);
        match recursive_projective_polynomial_sign_at_parameter(
            field,
            &coefficients,
            &candidate,
            policy,
        )? {
            Classification::Decided(RealSign::Zero) => retained.push(candidate),
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "recursive-polynomial-roots",
        "projected-replay",
    );
    Ok(Classification::Decided(retained))
}

pub(super) fn recursive_projective_polynomial_parameters(
    field: &BezierRecursiveQuadraticField2,
    coefficients: Vec<BezierRecursiveQuadraticValue2>,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    let strict_unit_crossing = (matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit()))
        .then(|| recursive_quadratic_polynomial_strict_unit_crossing(field, &coefficients))
        .flatten();
    recursive_projective_polynomial_parameters_with_crossing(
        field,
        coefficients,
        strict_unit_crossing,
        domain,
        policy,
    )
}

/// Isolates every rational-curve parameter whose chosen coordinate equals one
/// retained recursive projective point coordinate. Collinear range clipping
/// uses the chord's certified monotone axis, so this single coordinate
/// equation is sufficient and avoids constructing independent Cartesian
/// images for a deep endpoint.
pub(super) fn recursive_projective_point_rational_axis_parameters(
    point: &CurvePoint2,
    source: &RationalBezier2,
    axis: Axis2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    if range == &CurveParameterRange2::unit()
        && let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point
        && let Some(parameters) =
            point.zero_distance_rational_source_parameters_for_axis(source, axis, policy)?
    {
        #[cfg(feature = "dispatch-trace")]
        if matches!(&parameters, Classification::Decided(_)) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-collinear-endpoint",
                "zero-distance-rational-source",
            );
        }
        return Ok(parameters
            .map(|parameters| Some(parameters.into_iter().map(CurveParameter2::from).collect())));
    }
    let points = match recursive_projective_evidence_points(&[point], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let [point]: [BezierRecursiveQuadraticProjectivePoint2; 1] = points
        .try_into()
        .expect("a recursive endpoint projection retains one authored point");
    let point = match positive_recursive_projective_point(point)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let field = point.denominator.field();
    let source = source.homogeneous_power_basis()?;
    let source_axis = match axis {
        Axis2::X => &source.x_numerator,
        Axis2::Y => &source.y_numerator,
    };
    let point_axis = match axis {
        Axis2::X => &point.x,
        Axis2::Y => &point.y,
    };
    let Some((mut equation, mut weight)) = (|| {
        let source_axis = recursive_quadratic_real_polynomial(&field, source_axis)?;
        let weight = recursive_quadratic_real_polynomial(&field, &source.weight)?;
        let equation = recursive_quadratic_polynomial_combine(
            &recursive_quadratic_polynomial_scale(&source_axis, &point.denominator)?,
            &recursive_quadratic_polynomial_scale(&weight, point_axis)?,
            true,
        )?;
        Some((equation, weight))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    while equation
        .last()
        .is_some_and(BezierRecursiveQuadraticValue2::is_structurally_zero)
    {
        equation.pop();
    }
    while weight
        .last()
        .is_some_and(BezierRecursiveQuadraticValue2::is_structurally_zero)
    {
        weight.pop();
    }
    let candidates = match recursive_projective_polynomial_parameters(
        &field,
        equation,
        SelectedThirdAxisDomain2::Finite(range),
        policy,
    )? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let weight_sign = policy.strict_predicate_pass(|| {
            recursive_projective_polynomial_sign_at_parameter(&field, &weight, &candidate, policy)
        })?;
        match weight_sign {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                retained.push(candidate)
            }
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "algebraic-chord-collinear-endpoint",
        "recursive-projective",
    );
    Ok(Classification::Decided(Some(retained)))
}

pub(super) fn normalize_recursive_contact_frame(
    frame: BezierRecursiveQuadraticChordContactFrame2,
) -> CurveResult<Classification<BezierRecursiveQuadraticChordContactFrame2>> {
    if !frame.field.same_field(&frame.point.denominator.field())
        || !frame.field.same_field(&frame.center.denominator.field())
    {
        return Err(CurveError::Topology(
            "a recursive contact frame crossed retained coefficient fields".into(),
        ));
    }
    let point = match positive_recursive_projective_point(frame.point)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let center = match positive_recursive_projective_point(frame.center)? {
        Classification::Decided(center) => center,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        BezierRecursiveQuadraticChordContactFrame2 {
            field: frame.field,
            point,
            center,
        },
    ))
}

pub(super) fn orient_recursive_projective_point_positive(
    mut point: BezierRecursiveQuadraticProjectivePoint2,
    denominator_sign: RealSign,
) -> CurveResult<BezierRecursiveQuadraticProjectivePoint2> {
    match denominator_sign {
        RealSign::Positive => Ok(point),
        RealSign::Negative => {
            let negative = Real::from(-1_i8);
            point.x = point.x.scale(&negative).ok_or_else(|| {
                CurveError::Topology(
                    "a recursive point normalization exceeded its field budget".into(),
                )
            })?;
            point.y = point.y.scale(&negative).ok_or_else(|| {
                CurveError::Topology(
                    "a recursive point normalization exceeded its field budget".into(),
                )
            })?;
            point.denominator = point.denominator.scale(&negative).ok_or_else(|| {
                CurveError::Topology(
                    "a recursive point normalization exceeded its field budget".into(),
                )
            })?;
            Ok(point)
        }
        RealSign::Zero => Err(CurveError::Topology(
            "a recursive projective point retained a zero denominator".into(),
        )),
    }
}

pub(super) fn positive_recursive_projective_point(
    point: BezierRecursiveQuadraticProjectivePoint2,
) -> CurveResult<Classification<BezierRecursiveQuadraticProjectivePoint2>> {
    if point.denominator.is_structurally_zero() {
        return Err(CurveError::Topology(
            "a recursive projective point retained a zero denominator".into(),
        ));
    }
    // Recursive projective constructors certify a nonzero homogeneous
    // denominator before publishing the point. Normalization needs only its
    // orientation, so exact interval separation is complete; constructing a
    // selected-root norm here would redundantly solve denominator equality.
    match point.denominator.sign_with_nonzero_certificate()? {
        Classification::Decided(sign) => {
            orient_recursive_projective_point_positive(point, sign).map(Classification::Decided)
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn selected_trivariate_third_axis_is_identically_zero(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if trivariate_structurally_zero(polynomial, policy) {
        return Ok(Classification::Decided(true));
    }
    let (_, _, third_count) = polynomial.dimensions();
    for third in 0..third_count {
        let coefficient = BivariatePolynomial::new(
            polynomial
                .coefficients
                .iter()
                .map(|rows| {
                    rows.iter()
                        .map(|row| row.get(third).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect(),
        );
        let sign = if let Some(sign) = bivariate_parameter_pair_strict_sign_by_refinement(
            &coefficient,
            first_parameter,
            second_parameter,
            policy,
        )? {
            Classification::Decided(sign)
        } else {
            signed_bivariate_at_parameter_pair(
                &coefficient,
                first_parameter,
                second_parameter,
                policy,
            )?
        };
        match sign {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(true))
}

#[derive(Clone, Copy)]
pub(super) enum SelectedThirdAxisDomain2<'a> {
    Finite(&'a CurveParameterRange2),
    AffineLine,
    IncidentRay {
        anchor: &'a Real,
        direction: BezierParameterRayDirection2,
        barrier: Option<&'a BezierParameter2>,
    },
}

impl SelectedThirdAxisDomain2<'_> {
    pub(super) fn contains_parameter(
        self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match self {
            Self::Finite(range) => {
                CurveParameterDomain2::new(range, None).contains_finite_parameter(parameter, policy)
            }
            Self::AffineLine => Ok(Classification::Decided(true)),
            Self::IncidentRay {
                anchor,
                direction,
                barrier,
            } => {
                let wanted = match direction {
                    BezierParameterRayDirection2::Increasing => std::cmp::Ordering::Greater,
                    BezierParameterRayDirection2::Decreasing => std::cmp::Ordering::Less,
                };
                match parameter.cmp_by_refinement(&CurveParameter2::from(anchor.clone()), policy)? {
                    Classification::Decided(order) if order != wanted => {
                        return Ok(Classification::Decided(false));
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                if let Some(barrier) = barrier {
                    return Ok(parameter
                        .cmp_by_refinement(&CurveParameter2::from(barrier.clone()), policy)?
                        .map(|order| order == wanted.reverse()));
                }
                Ok(Classification::Decided(true))
            }
        }
    }

    pub(super) fn polynomial_is_nonzero(
        self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Self::Finite(range) = self {
            return polynomial_is_nonzero_on_parameter_range(coefficients, range, policy);
        }
        let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .isolate(&polynomial, policy)?
            .map(|roots| roots.is_empty()))
    }

    pub(super) fn strict_sample(self, policy: &CurveContext) -> CurveResult<Classification<Real>> {
        policy.strict_predicate_pass(|| match self {
            Self::Finite(range) => range.strict_interior_scalar(policy),
            Self::AffineLine => Ok(Classification::Decided(Real::zero())),
            Self::IncidentRay {
                anchor,
                direction,
                barrier,
            } => {
                let anchor_parameter = BezierParameter2::Exact(anchor.clone());
                match (direction, barrier) {
                    (BezierParameterRayDirection2::Increasing, Some(barrier)) => {
                        anchor_parameter.strict_scalar_between_ordered(barrier, policy)
                    }
                    (BezierParameterRayDirection2::Decreasing, Some(barrier)) => {
                        barrier.strict_scalar_between_ordered(&anchor_parameter, policy)
                    }
                    (BezierParameterRayDirection2::Increasing, None) => {
                        Ok(Classification::Decided(anchor + Real::one()))
                    }
                    (BezierParameterRayDirection2::Decreasing, None) => {
                        Ok(Classification::Decided(anchor - Real::one()))
                    }
                }
            }
        })
    }

    pub(super) fn isolate(
        self,
        polynomial: &BezierParameterPolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        match self {
            Self::Finite(range) => {
                CurveParameterDomain2::new(range, None).finite_roots(polynomial, policy)
            }
            Self::AffineLine => {
                let zero = Real::zero();
                let mut decreasing = match polynomial.isolate_incident_ray_roots(
                    &zero,
                    BezierParameterRayDirection2::Decreasing,
                    policy,
                )? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                decreasing.reverse();
                match real_sign(&polynomial.evaluate(&zero), policy) {
                    Some(RealSign::Zero) => {
                        decreasing.push(BezierParameter2::Exact(zero.clone()));
                    }
                    Some(RealSign::Negative | RealSign::Positive) => {}
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
                let increasing = match polynomial.isolate_incident_ray_roots(
                    &zero,
                    BezierParameterRayDirection2::Increasing,
                    policy,
                )? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                decreasing.extend(increasing);
                Ok(Classification::Decided(decreasing))
            }
            Self::IncidentRay {
                anchor,
                direction,
                barrier,
            } => {
                let parameters =
                    match polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
                        Classification::Decided(parameters) => parameters,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                retain_parameters_before_incident_barrier(parameters, barrier, direction, policy)
            }
        }
    }
}

pub(super) fn strict_sample_for_parallel_domain(
    range: &CurveParameterRange2,
    incident: Option<&BezierParallelIncidentDomain2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    if let Some(incident) = incident {
        return (SelectedThirdAxisDomain2::IncidentRay {
            anchor: incident.anchor(),
            direction: incident.direction(),
            barrier: incident.barrier(),
        })
        .strict_sample(policy);
    }
    SelectedThirdAxisDomain2::Finite(range).strict_sample(policy)
}

/// Eliminates two retained selected roots and isolates every candidate on the
/// requested third-axis domain. The projection is enumeration only; callers
/// must replay their unsquared authored predicate at each returned triple
/// before admitting topology.
pub(super) fn selected_trivariate_third_axis_parameters(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    selected_trivariate_third_axis_parameters_with_resultant_limit(
        polynomial,
        first_parameter,
        second_parameter,
        domain,
        MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        policy,
    )
}

pub(super) fn selected_trivariate_third_axis_parameters_with_resultant_limit(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    domain: SelectedThirdAxisDomain2<'_>,
    max_resultant_degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let bounded = selected_trivariate_third_axis_parameters_bounded(
        polynomial,
        first_parameter,
        second_parameter,
        domain,
        max_resultant_degree,
        policy,
    )?;
    if !matches!(
        bounded,
        Classification::Uncertain(UncertaintyReason::Unsupported)
    ) {
        return Ok(bounded);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "selected-trivariate-projection",
        "rank-independent-tensor-fallback",
    );
    let Some(polynomial) = polynomial.to_dense_polynomial() else {
        return Ok(bounded);
    };
    let sources = selected_parameter_representations([first_parameter, second_parameter]);
    selected_dense_last_axis_parameters(&polynomial, &sources, domain, policy)
}

pub(super) fn selected_trivariate_third_axis_parameters_bounded(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    domain: SelectedThirdAxisDomain2<'_>,
    max_resultant_degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let defining = |parameter: &BezierParameter2| match parameter {
        BezierParameter2::Exact(parameter) => vec![-parameter.clone(), Real::one()],
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().coefficients().to_vec(),
    };
    // Resultants only need the quotient-ring representative at the two
    // already selected roots.  Reducing those axes before interpolation can
    // turn a high-degree squared geometric projection into a small tensor,
    // especially for the ubiquitous rational-line case.
    let first_defining = defining(first_parameter);
    let second_defining_coefficients = defining(second_parameter);
    let mut reduced = polynomial.clone();
    for (axis, coefficients) in [
        (0_usize, first_defining.as_slice()),
        (1_usize, second_defining_coefficients.as_slice()),
    ] {
        let dimensions = reduced.dimensions();
        let count = [dimensions.0, dimensions.1, dimensions.2][axis];
        let degree = coefficients.len().saturating_sub(1);
        if count > degree {
            let Some(next) = trivariate_reduce_axis_mod_defining(reduced, axis, coefficients)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            reduced = next;
        }
    }
    let polynomial = &reduced;
    let first_axis_is_independent = polynomial.dimensions().0 == 1;
    let config = CurveIntersectionResultantConfig {
        min_precision: hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        max_resultant_degree,
    };
    // Quotient reduction commonly leaves the first selected axis linear.  Its
    // constrained resultant has a direct homogeneous evaluation
    // `B^n Q(-A/B)`, which is exact even when the generic interpolation grid
    // meets a vanishing leading coefficient.  It also builds a substantially
    // smaller Real DAG than interpolating a Sylvester determinant.
    let direct_first_projection = if first_axis_is_independent {
        // The selected root remains valid evidence, but this relation no
        // longer depends on its axis. Eliminating it would only raise the
        // surviving polynomial to the constraint degree.
        polynomial
            .coefficients
            .first()
            .cloned()
            .map(BivariatePolynomial::new)
    } else {
        trivariate_linear_axis_coefficients(polynomial, 0)
            .and_then(|(constant, linear, remaining)| {
                (remaining == [1, 2]).then_some((constant, linear))
            })
            .and_then(|(constant, linear)| {
                bivariate_linear_root_resultant(&constant, &linear, &first_defining)
            })
    };
    let first_projection = if let Some(projection) = direct_first_projection {
        projection
    } else {
        let report = resultant_trivariate_polynomial_univariate_constraint(
            polynomial,
            &first_defining,
            TrivariatePolynomialAxis::First,
            config,
        );
        match report.status {
            TrivariateConstraintResultantStatus::Constructed => report
                .resultant
                .expect("a constructed constrained resultant retains its polynomial"),
            TrivariateConstraintResultantStatus::UndecidedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            TrivariateConstraintResultantStatus::EmptyPolynomial
            | TrivariateConstraintResultantStatus::InvalidConstraint
            | TrivariateConstraintResultantStatus::DegreeBoundExceeded
            | TrivariateConstraintResultantStatus::ResultantError
            | TrivariateConstraintResultantStatus::InterpolationDivisionFailed => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
    };
    let coefficients = if let Some(coefficients) =
        bivariate_quadratic_constraint_resultant(&first_projection, &second_defining_coefficients)
    {
        coefficients
    } else {
        let second_defining = BivariatePolynomial::new(
            second_defining_coefficients
                .iter()
                .cloned()
                .map(|coefficient| vec![coefficient])
                .collect(),
        );
        let projection = resultant_bivariate_polynomial_system(
            &first_projection,
            &second_defining,
            CurveResultantParameter::Second,
            config,
        );
        match projection.status {
            CurveIntersectionResultantStatus::Constructed => projection.resultant_coefficients,
            CurveIntersectionResultantStatus::UndecidedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            CurveIntersectionResultantStatus::EmptyCoordinatePolynomial
            | CurveIntersectionResultantStatus::DegreeBoundExceeded
            | CurveIntersectionResultantStatus::ResultantError
            | CurveIntersectionResultantStatus::InterpolationDivisionFailed
            | CurveIntersectionResultantStatus::InvalidHomogeneousWeight => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
    };
    match polynomial_coefficients_are_identically_zero(&coefficients, policy) {
        Classification::Decided(true) => {
            return Ok(
                match selected_trivariate_third_axis_is_identically_zero(
                    polynomial,
                    first_parameter,
                    second_parameter,
                    policy,
                )? {
                    Classification::Decided(true) => {
                        Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero)
                    }
                    Classification::Decided(false) => {
                        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    // Once the first selected axis is absent, this eliminant is only a local
    // candidate carrier. Repeated radical-norm factors add no roots; exact
    // authored-sheet replay below still owns admission. Preserve the existing
    // global schedule for genuinely two-field projections.
    let coefficients = if first_axis_is_independent {
        let coefficients =
            hypersolve::square_free_part(coefficients.clone(), hypersolve::PredicatePolicy::STRICT)
                .unwrap_or(coefficients);
        // Resultant interpolation can retain a large arithmetic DAG even when
        // a coefficient has a bounded exact rational normal form. Collapse
        // only those proven rationals before local fiber isolation; all other
        // exact scalar fields remain untouched.
        coefficients
            .into_iter()
            .map(|coefficient| {
                coefficient
                    .exact_rational_normal_form()
                    .map(Real::new)
                    .unwrap_or(coefficient)
            })
            .collect::<Vec<_>>()
    } else {
        coefficients
    };
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if matches!(domain, SelectedThirdAxisDomain2::Finite(range) if range == &CurveParameterRange2::unit())
        && first_axis_is_independent
        && polynomial.degree() > MAX_DIRECT_SELECTED_PAIR_NORM_ISOLATION_DEGREE
        && let BezierParameter2::Algebraic(second_parameter) = second_parameter
    {
        let selected = algebraic_selected_parameters_from_norm(
            &first_projection,
            &parameter_representation(second_parameter, policy),
            polynomial,
            &CurveParameterRange2::unit(),
            policy,
        )?;
        return Ok(selected.map(|projection| match projection {
            ResultantParameterProjection::Empty => {
                BezierAlgebraicFiberProjection2::Parameters(Vec::new())
            }
            ResultantParameterProjection::Parameters(parameters)
            | ResultantParameterProjection::SelectedParameters(parameters) => {
                BezierAlgebraicFiberProjection2::Parameters(parameters)
            }
            ResultantParameterProjection::Degenerate => BezierAlgebraicFiberProjection2::Degenerate,
        }));
    }
    domain
        .isolate(&polynomial, policy)
        .map(|isolated| isolated.map(BezierAlgebraicFiberProjection2::Parameters))
}

/// Gives an ordinary nonzero selected tuple a small exact-box opportunity to
/// separate before correlation machinery is constructed.
pub(super) fn trivariate_parameter_triple_bounded_box_sign(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    maximum_steps: usize,
) -> CurveResult<Option<RealSign>> {
    let strict = &CurveContext::STRICT;
    let mut first_refinement = BezierParameterRefinement2::new(first_parameter, strict);
    let mut second_refinement = BezierParameterRefinement2::new(second_parameter, strict);
    let mut third_refinement = BezierParameterRefinement2::new(third_parameter, strict);
    let dimensions = polynomial.dimensions();
    let is_multi_affine = dimensions.0 <= 2 && dimensions.1 <= 2 && dimensions.2 <= 2;
    for target_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        if target_steps > maximum_steps {
            break;
        }
        let first = first_refinement.refine_to(target_steps);
        let second = second_refinement.refine_to(target_steps);
        let third = third_refinement.refine_to(target_steps);
        let sign = if is_multi_affine {
            trivariate_multi_affine_parameter_box_strict_sign(
                polynomial, first, second, third, strict,
            )
        } else {
            trivariate_unit_cube_strict_bernstein_sign(
                trivariate_restrict_to_parameter_box(polynomial, first, second, third),
                strict,
            )?
        };
        if sign.is_some() {
            return Ok(sign);
        }
    }
    Ok(None)
}

/// Enumerates one exact third-axis norm projection and retains only roots
/// accepted by the caller's authored-sheet replay.  This is shared by the
/// one- and two-radical selected-pair kernels so candidate correlation,
/// transverse box certification, and even-root subresultants have one owner.
pub(super) fn selected_projected_trivariate_third_axis_parameters(
    projection: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
    mut replay: impl FnMut(&BezierParameter2, bool) -> CurveResult<Classification<RealSign>>,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let candidates = match selected_trivariate_third_axis_parameters(
        projection,
        first_parameter,
        second_parameter,
        domain,
        policy,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
            candidates
        }
        Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero,
            ));
        }
        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::Degenerate,
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let has_box_root = projected_selected_trivariate_candidate_has_box_root(
            projection,
            first_parameter,
            second_parameter,
            &candidate,
            8,
        )?;
        if !has_box_root
            && trivariate_parameter_triple_bounded_box_sign(
                projection,
                first_parameter,
                second_parameter,
                &candidate,
                64,
            )?
            .is_some()
        {
            continue;
        }
        let projected_root_certified = if has_box_root {
            true
        } else {
            match projected_selected_trivariate_candidate_has_subresultant_root(
                projection,
                first_parameter,
                second_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(true) => true,
                Classification::Decided(false) => continue,
                Classification::Uncertain(_) => false,
            }
        };
        match replay(&candidate, projected_root_certified)? {
            Classification::Decided(RealSign::Zero) => retained.push(candidate),
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(retained),
    ))
}
