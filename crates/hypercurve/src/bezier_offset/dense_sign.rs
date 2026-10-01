//! Exact and certified-interval signs of dense tensor polynomials at
//! selected source tuples, including sums of positive square roots.

use super::*;

/// Collapses affine-related selected tensor axes before quotient reduction
/// or image projection, preserving their root correlation.
pub(super) fn dense_substitute_affinely_related_sources(
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
                    Some(hypersolve::AlgebraicRootAffineRelation {
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

pub(super) fn dense_strict_interval_sign(value: &RealInterval) -> Option<RealSign> {
    let strict = &CurveContext::STRICT;
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
        real_sign(&value.upper, strict).or_else(|| certified_nonzero_sign(&value.upper));
    let lower_sign =
        real_sign(&value.lower, strict).or_else(|| certified_nonzero_sign(&value.lower));
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

pub(super) fn dense_polynomial_value_interval(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_tensor_interval(&dense_tensor_with_output_axis(polynomial)?, sources)
}

pub(super) fn dense_polynomial_value_interval_with_coefficient_precision(
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

#[track_caller]
pub(super) fn dense_polynomial_tuple_sign(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let Some(polynomial) = try_clone_dense_tensor(polynomial) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    dense_polynomial_tuple_sign_owned(polynomial, sources, policy)
}

#[track_caller]
pub(super) fn dense_polynomial_tuple_sign_owned(
    polynomial: DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let Some((polynomial, sources)) =
        dense_substitute_affinely_related_sources(polynomial, sources)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(polynomial) = dense_reduce_selected_tuple_relations(polynomial, &sources)
        .map(DenseTensorPolynomial::compact_coefficients)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    match polynomial_coefficients_are_identically_zero(
        polynomial.coefficients(),
        &CurveContext::STRICT,
    ) {
        Classification::Decided(true) => {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    // Point substitution is a proof attempt, not a replacement for the
    // retained equations. An undecided scalar/field query falls through to
    // the original selected tuple and its image replay.
    if sources.len() > 1
        && let Some(sign) = hypersolve::sign_at_selected_tuple(&polynomial, &sources)
    {
        return Ok(Classification::Decided(match sign {
            std::cmp::Ordering::Less => RealSign::Negative,
            std::cmp::Ordering::Equal => RealSign::Zero,
            std::cmp::Ordering::Greater => RealSign::Positive,
        }));
    }
    if let [source] = sources.as_slice() {
        let result =
            match BezierParameter2::from_algebraic_root_representation_unbounded(source, policy)? {
                Classification::Decided(parameter) => {
                    signed_coefficients_at_parameter(polynomial.coefficients(), &parameter, policy)?
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            };
        if result.is_decided() || policy.has_bounded_exact_predicate_budget() {
            #[cfg(feature = "dispatch-trace")]
            if result.is_decided() {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "dense-polynomial-tuple-sign",
                    "retained-univariate-parameter",
                );
            }
            return Ok(result);
        }
    }
    let Some(value) = dense_tensor_with_output_axis(&polynomial) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
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
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
        let coefficient_precision = -coefficient_bits;
        if let Some(sign) = dense_polynomial_value_interval_with_coefficient_precision(
            &polynomial,
            &refined,
            coefficient_precision,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign)
        {
            return Ok(Classification::Decided(sign));
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
            Classification::Uncertain(UncertaintyReason::Predicate)
        } else {
            #[cfg(test)]
            if value.dimensions().len() == 4
                && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            {
                let caller = std::panic::Location::caller();
                eprintln!(
                    "dense tuple image replay caller={}:{} steps={refinement_steps} selects-approximate={} permits-approximate={}",
                    caller.file(),
                    caller.line(),
                    policy.selects_approximate_512(),
                    policy.permits_approximate_512(),
                );
            }
            represented_dense_value_with_coefficient_precision(
                &value,
                &refined,
                coefficient_precision,
            )
        };
        if let Classification::Decided(represented) = &represented
            && let Some(sign) = represented_strict_sign(represented)
        {
            return Ok(Classification::Decided(sign));
        }
        if matches!(
            represented,
            Classification::Uncertain(UncertaintyReason::Unsupported)
        ) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }

        if approximate_terminal {
            return match represented {
                Classification::Decided(represented) => {
                    Ok(represented_policy_sign(&represented, policy))
                }
                Classification::Uncertain(_) => {
                    policy.observe_approximate_512();
                    Ok(Classification::Decided(RealSign::Zero))
                }
            };
        }
        if bounded_terminal {
            // This is the preliminary certified pass of an APPROXIMATE_512
            // operation.  Preserve its strict uncertainty so the outer policy
            // replay can consume the terminal; never continue this selected
            // policy into an unbounded exact promotion.
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        refinement_steps = match next_refinement_steps(refinement_steps) {
            Some(next) => next,
            None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        };
        if policy.selects_approximate_512() && refinement_steps > 512 {
            unreachable!("APPROXIMATE_512 cannot refine past its terminal")
        }
    }
}

pub(super) fn dense_two_positive_square_root_interval_with_coefficient_precision(
    expression: &BezierDenseTwoSquareRootExpression2,
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
        if BezierDenseTwoSquareRootExpression2::polynomial_is_stored_zero(coefficient) {
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

pub(super) fn dense_two_positive_square_root_interval(
    expression: &BezierDenseTwoSquareRootExpression2,
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

pub(super) fn dense_positive_square_root_interval_with_coefficient_precision(
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

pub(super) fn dense_positive_square_root_interval(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<RealInterval> {
    dense_positive_square_root_interval_with_coefficient_precision(
        rational, radical, radicand, sources, None,
    )
}

pub(super) fn same_positive_root_sheet_signs(
    first: RealSign,
    second: RealSign,
) -> Option<RealSign> {
    match (first, second) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => Some(sign),
        (first, second) if first == second => Some(first),
        _ => None,
    }
}

pub(super) fn combined_sign_uncertainty(
    first: &Classification<RealSign>,
    second: &Classification<RealSign>,
) -> UncertaintyReason {
    match (first, second) {
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            UncertaintyReason::Unsupported
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => *reason,
        _ => UncertaintyReason::Predicate,
    }
}

/// Combines signs for A+B*sqrt(S), with S strictly positive and magnitude
/// A^2-B^2*S. A zero magnitude alone does not select the cancelling sheet.
pub(super) fn positive_root_sum_sign_from_components(
    rational_sign: Classification<RealSign>,
    radical_sign: Classification<RealSign>,
    magnitude_sign: Classification<RealSign>,
) -> Classification<RealSign> {
    match magnitude_sign {
        Classification::Decided(RealSign::Positive) => rational_sign,
        Classification::Decided(RealSign::Negative) => radical_sign,
        Classification::Decided(RealSign::Zero) => match (&rational_sign, &radical_sign) {
            (Classification::Decided(rational_sign), Classification::Decided(radical_sign)) => {
                Classification::Decided(
                    same_positive_root_sheet_signs(*rational_sign, *radical_sign)
                        .unwrap_or(RealSign::Zero),
                )
            }
            _ => {
                Classification::Uncertain(combined_sign_uncertainty(&rational_sign, &radical_sign))
            }
        },
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

#[track_caller]
pub(super) fn dense_positive_square_root_sum_sign(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let reduce = |polynomial| dense_reduce_selected_tuple_relations(polynomial, sources);
    let Some((rational, radical, radicand)) = (|| {
        Some((
            reduce(rational.clone())?,
            reduce(radical.clone())?,
            reduce(radicand.clone())?,
        ))
    })() else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "dense-positive-root-sign-blocker",
            "input-reduction",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let rational_sign = dense_polynomial_tuple_sign(&rational, sources, policy)?;
    let radical_sign = dense_polynomial_tuple_sign(&radical, sources, policy)?;
    if let (Classification::Decided(rational_sign), Classification::Decided(radical_sign)) =
        (&rational_sign, &radical_sign)
        && let Some(sign) = same_positive_root_sheet_signs(*rational_sign, *radical_sign)
    {
        return Ok(Classification::Decided(sign));
    }
    let Some(magnitude) = (|| {
        let rational_squared = reduce(rational.multiply(&rational)?)?;
        let radical_squared = reduce(radical.multiply(&radical)?)?;
        let radical_squared = reduce(radical_squared.multiply(&radicand)?)?;
        reduce(rational_squared.subtract(&radical_squared)?)
    })() else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "dense-positive-root-sign-blocker",
            "magnitude-construction",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let magnitude_sign = dense_polynomial_tuple_sign(&magnitude, sources, policy)?;
    Ok(positive_root_sum_sign_from_components(
        rational_sign,
        radical_sign,
        magnitude_sign,
    ))
}

#[track_caller]
pub(super) fn dense_two_positive_square_root_sum_sign(
    expression: &BezierDenseTwoSquareRootExpression2,
    first_speed_squared: &DenseTensorPolynomial,
    second_speed_squared: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let reduce = |polynomial| dense_reduce_selected_tuple_relations(polynomial, sources);
    macro_rules! reduce_input {
        ($value:expr, $path:literal) => {
            match reduce($value) {
                Some(value) => value,
                None => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "dense-two-root-sign-blocker",
                        $path,
                    );
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            }
        };
    }
    let expression = BezierDenseTwoSquareRootExpression2 {
        rational: reduce_input!(expression.rational.clone(), "rational-input-reduction"),
        first: reduce_input!(expression.first.clone(), "first-input-reduction"),
        second: reduce_input!(expression.second.clone(), "second-input-reduction"),
        product: reduce_input!(expression.product.clone(), "product-input-reduction"),
    };
    let first_speed_squared =
        reduce_input!(first_speed_squared.clone(), "first-speed-input-reduction");
    let second_speed_squared =
        reduce_input!(second_speed_squared.clone(), "second-speed-input-reduction");
    let retained = dense_positive_square_root_sum_sign(
        &expression.rational,
        &expression.first,
        &first_speed_squared,
        sources,
        policy,
    )?;
    let candidate = dense_positive_square_root_sum_sign(
        &expression.second,
        &expression.product,
        &first_speed_squared,
        sources,
        policy,
    )?;
    if let (Classification::Decided(retained), Classification::Decided(candidate)) =
        (&retained, &candidate)
        && let Some(sign) = same_positive_root_sheet_signs(*retained, *candidate)
    {
        return Ok(Classification::Decided(sign));
    }
    let Some(norm_rational) = (|| {
        let rational_squared = reduce(expression.rational.multiply(&expression.rational)?)?;
        let first_squared = reduce(expression.first.multiply(&expression.first)?)?;
        let first_squared = reduce(first_squared.multiply(&first_speed_squared)?)?;
        let retained_squared = reduce(rational_squared.add(&first_squared)?)?;
        let second_squared = reduce(expression.second.multiply(&expression.second)?)?;
        let product_squared = reduce(expression.product.multiply(&expression.product)?)?;
        let product_squared = reduce(product_squared.multiply(&first_speed_squared)?)?;
        let candidate_squared = reduce(second_squared.add(&product_squared)?)?;
        let candidate_squared = reduce(candidate_squared.multiply(&second_speed_squared)?)?;
        reduce(retained_squared.subtract(&candidate_squared)?)
    })() else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "dense-two-root-sign-blocker",
            "norm-rational-construction",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(norm_radical) = (|| {
        let retained_product = reduce(expression.rational.multiply(&expression.first)?)?;
        let candidate_product = reduce(expression.second.multiply(&expression.product)?)?;
        let candidate_product = reduce(candidate_product.multiply(&second_speed_squared)?)?;
        reduce(
            retained_product
                .subtract(&candidate_product)?
                .scale(&Real::from(2_i8))?,
        )
    })() else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "dense-two-root-sign-blocker",
            "norm-radical-construction",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let norm_sign = dense_positive_square_root_sum_sign(
        &norm_rational,
        &norm_radical,
        &first_speed_squared,
        sources,
        policy,
    )?;
    Ok(match norm_sign {
        Classification::Decided(RealSign::Positive) => retained,
        Classification::Decided(RealSign::Negative) => candidate,
        Classification::Decided(RealSign::Zero) => match (&retained, &candidate) {
            (Classification::Decided(retained), Classification::Decided(candidate)) => {
                Classification::Decided(
                    same_positive_root_sheet_signs(*retained, *candidate).unwrap_or(RealSign::Zero),
                )
            }
            _ => Classification::Uncertain(combined_sign_uncertainty(&retained, &candidate)),
        },
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// Selects the authored `(+sqrt(first), +sqrt(second))` sheet after the
/// complete two-radical norm was independently certified zero at `sources`.
///
/// Write the value as `u + v sqrt(second)`, where both `u` and `v` are
/// one-root values over `sqrt(first)`.  If their signs oppose, the sign of
/// `u^2 - v^2 second` selects the dominant term.  The supplied norm-zero
/// certificate makes that magnitude itself a one-root sheet-selection
/// problem, so its two polynomial component signs decide exact equality
/// without reconstructing the eliminated tensor coordinate.
pub(super) fn dense_two_positive_square_root_sum_sign_at_projected_zero(
    expression: &BezierDenseTwoSquareRootExpression2,
    first_speed_squared: &DenseTensorPolynomial,
    second_speed_squared: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let reduce = |polynomial| dense_reduce_selected_tuple_relations(polynomial, sources);
    let Some((expression, first_speed_squared, second_speed_squared)) = (|| {
        Some((
            BezierDenseTwoSquareRootExpression2 {
                rational: reduce(expression.rational.clone())?,
                first: reduce(expression.first.clone())?,
                second: reduce(expression.second.clone())?,
                product: reduce(expression.product.clone())?,
            },
            reduce(first_speed_squared.clone())?,
            reduce(second_speed_squared.clone())?,
        ))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let structurally_zero = |polynomial: &DenseTensorPolynomial| {
        polynomial
            .coefficients()
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
    };
    let nonzero_components = [
        &expression.rational,
        &expression.first,
        &expression.second,
        &expression.product,
    ]
    .into_iter()
    .filter(|component| !structurally_zero(component))
    .count();
    if nonzero_components <= 1 {
        // The projection authority preserves the sole component verbatim;
        // positive radicals cannot change its zero set.
        return Ok(Classification::Decided(RealSign::Zero));
    }
    let one_root_sign = |rational: &DenseTensorPolynomial, radical: &DenseTensorPolynomial| {
        dense_positive_square_root_sum_sign(
            rational,
            radical,
            &first_speed_squared,
            sources,
            policy,
        )
    };
    let retained = match one_root_sign(&expression.rational, &expression.first)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidate = match one_root_sign(&expression.second, &expression.product)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if let Some(sign) = same_positive_root_sheet_signs(retained, candidate) {
        return Ok(Classification::Decided(sign));
    }
    let Some((norm_rational, norm_radical)) = (|| {
        let rational_squared = reduce(expression.rational.multiply(&expression.rational)?)?;
        let first_squared = reduce(expression.first.multiply(&expression.first)?)?;
        let retained_squared =
            reduce(rational_squared.add(&first_squared.multiply(&first_speed_squared)?)?)?;
        let second_squared = reduce(expression.second.multiply(&expression.second)?)?;
        let product_squared = reduce(expression.product.multiply(&expression.product)?)?;
        let candidate_squared =
            reduce(second_squared.add(&product_squared.multiply(&first_speed_squared)?)?)?;
        let norm_rational = reduce(
            retained_squared.subtract(&candidate_squared.multiply(&second_speed_squared)?)?,
        )?;
        let retained_product = reduce(expression.rational.multiply(&expression.first)?)?;
        let candidate_product = reduce(expression.second.multiply(&expression.product)?)?;
        let norm_radical = reduce(
            retained_product
                .subtract(&candidate_product.multiply(&second_speed_squared)?)?
                .scale(&Real::from(2_i8))?,
        )?;
        Some((norm_rational, norm_radical))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let norm_rational = match dense_polynomial_tuple_sign(&norm_rational, sources, policy)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let norm_radical = match dense_polynomial_tuple_sign(&norm_radical, sources, policy)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let norm = match (norm_rational, norm_radical) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
        (first, second) if first == second => first,
        _ => RealSign::Zero,
    };
    Ok(Classification::Decided(match norm {
        RealSign::Positive => retained,
        RealSign::Negative => candidate,
        RealSign::Zero => RealSign::Zero,
    }))
}
