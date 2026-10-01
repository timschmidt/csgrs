//! Exact sign, content, substitution and factorization helpers for
//! bivariate, trivariate and quadrivariate parameter polynomials.

use super::*;

#[inline]
pub(super) fn trivariate_structurally_zero(
    polynomial: &TrivariatePolynomial,
    policy: &CurveContext,
) -> bool {
    for rows in &polynomial.coefficients {
        for row in rows {
            for coefficient in row {
                if real_sign(coefficient, policy) != Some(RealSign::Zero) {
                    return false;
                }
            }
        }
    }
    true
}

/// Returns the quotient-ring representative at two retained selected roots.
/// This is exact for every value of the untouched third axis and is therefore
/// suitable for maps that must later evaluate many target parameters.
pub(super) fn trivariate_reduce_parameter_pair_relations(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
) -> Option<TrivariatePolynomial> {
    let parameters = [first, second];
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1];
    let degrees = parameters.map(|parameter| match parameter {
        BezierParameter2::Exact(_) => 1,
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().degree(),
    });
    let reduce_axes: [bool; 2] = std::array::from_fn(|axis| counts[axis] > degrees[axis]);
    if !reduce_axes.into_iter().any(std::convert::identity) {
        return None;
    }
    let mut reduced = polynomial.clone();
    for (axis, parameter) in parameters.into_iter().enumerate() {
        if !reduce_axes[axis] {
            continue;
        }
        match parameter {
            BezierParameter2::Exact(parameter) => {
                let defining = [-parameter.clone(), Real::one()];
                reduced = trivariate_reduce_axis_mod_defining(reduced, axis, &defining)?;
            }
            BezierParameter2::Algebraic(parameter) => {
                reduced = trivariate_reduce_axis_mod_defining(
                    reduced,
                    axis,
                    parameter.polynomial().coefficients(),
                )?;
            }
        }
    }
    Some(reduced)
}

/// Returns a smaller tensor only when at least one selected-root relation
/// removes powers. Sequential reductions commute at the selected root tuple.
pub(super) fn trivariate_reduce_selected_root_relations(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> Option<TrivariatePolynomial> {
    let parameters = [first, second, third];
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let degrees = parameters.map(|parameter| match parameter {
        BezierParameter2::Exact(_) => 1,
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().degree(),
    });
    let reduce_axes: [bool; 3] = std::array::from_fn(|axis| counts[axis] > degrees[axis]);
    if !reduce_axes.into_iter().any(std::convert::identity) {
        return None;
    }
    let mut reduced = polynomial.clone();
    for (axis, parameter) in parameters.into_iter().enumerate() {
        if !reduce_axes[axis] {
            continue;
        }
        match parameter {
            BezierParameter2::Exact(parameter) => {
                let defining = [-parameter.clone(), Real::one()];
                reduced = trivariate_reduce_axis_mod_defining(reduced, axis, &defining)?;
            }
            BezierParameter2::Algebraic(parameter) => {
                reduced = trivariate_reduce_axis_mod_defining(
                    reduced,
                    axis,
                    parameter.polynomial().coefficients(),
                )?;
            }
        }
    }
    Some(reduced)
}

/// Removes separable univariate tensor content and returns its selected sign.
/// A zero content factor proves the original tensor zero immediately.
pub(super) fn trivariate_strip_axis_contents(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<(TrivariatePolynomial, Option<RealSign>)>> {
    let parameters = [first, second, third];
    let mut reduced = polynomial.clone();
    let mut factor_sign = Some(RealSign::Positive);
    let mut changed = false;
    for (axis, parameter) in parameters.into_iter().enumerate() {
        let Some(content) = trivariate_axis_content(&reduced, axis) else {
            continue;
        };
        changed = true;
        match signed_coefficients_at_parameter(&content, parameter, &CurveContext::STRICT)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Some((reduced, Some(RealSign::Zero))));
            }
            Classification::Decided(sign) => {
                factor_sign = factor_sign.map(|accumulator| product_sign(accumulator, sign));
            }
            Classification::Uncertain(_) => factor_sign = None,
        }
        let Some(quotient) = trivariate_divide_axis_content(&reduced, axis, &content) else {
            return Ok(None);
        };
        reduced = quotient;
    }
    Ok(changed.then_some((reduced, factor_sign)))
}

pub(super) fn parameter_bounds(parameter: &BezierParameter2) -> (&Real, &Real) {
    match parameter {
        BezierParameter2::Exact(parameter) => (parameter, parameter),
        BezierParameter2::Algebraic(parameter) => {
            (parameter.interval().start(), parameter.interval().end())
        }
    }
}

/// Returns `(scale, offset)` only after exact evidence proves
/// `second = scale * first + offset`.
pub(super) fn exact_parameter_affine_relation(
    first: &BezierParameter2,
    second: &BezierParameter2,
) -> Option<(Real, Real)> {
    match (first, second) {
        (BezierParameter2::Exact(first), BezierParameter2::Exact(second)) => {
            Some((Real::one(), second - first))
        }
        (BezierParameter2::Algebraic(first), BezierParameter2::Algebraic(second)) => {
            let first = parameter_representation(first, &CurveContext::STRICT);
            let second = parameter_representation(second, &CurveContext::STRICT);
            if let Some(difference) = hypersolve::translated_algebraic_root_difference(
                &first,
                &second,
                hypersolve::PredicatePolicy::STRICT,
            ) {
                return Some((Real::one(), -difference));
            }
            let relation = hypersolve::algebraic_root_affine_relation(&first, &second)?;
            Some((relation.scale, relation.offset))
        }
        (BezierParameter2::Exact(_), BezierParameter2::Algebraic(_))
        | (BezierParameter2::Algebraic(_), BezierParameter2::Exact(_)) => None,
    }
}

/// Proves `result = left op right` for the three selected parameter values.
///
/// Hypersolve constructs the arithmetic image as an exact represented root;
/// the separate difference comparison then proves that the image is the
/// selected result root rather than a foreign conjugate.
pub(super) fn exact_parameter_binary_relation(
    left: &BezierParameter2,
    right: &BezierParameter2,
    result: &BezierParameter2,
    operation: hypersolve::AlgebraicRootArithmeticOp,
) -> bool {
    let represent = |parameter: &BezierParameter2| match parameter {
        BezierParameter2::Exact(parameter) => {
            AlgebraicRootRepresentation::from_exact_value(parameter)
        }
        BezierParameter2::Algebraic(parameter) => {
            certified_parameter_representation(parameter, &CurveContext::STRICT)
        }
    };
    let left = represent(left);
    let right = represent(right);
    let result = represent(result);
    let arithmetic = hypersolve::arithmetic_algebraic_root_representations(
        &left,
        Some(&right),
        operation,
        hypersolve::PredicatePolicy::STRICT,
    );
    if !crate::bezier_algebraic_image::algebraic_arithmetic_succeeded(&arithmetic.status) {
        return false;
    }
    let Some(candidate) = arithmetic.result_representation.or_else(|| {
        arithmetic
            .exact_result
            .as_ref()
            .map(AlgebraicRootRepresentation::from_exact_value)
    }) else {
        return false;
    };
    let comparison = hypersolve::compare_algebraic_root_representations_by_difference(
        &candidate,
        &result,
        hypersolve::AlgebraicRootRefinementComparisonConfig {
            policy: hypersolve::PredicatePolicy::STRICT,
            ..hypersolve::AlgebraicRootRefinementComparisonConfig::default()
        },
    )
    .comparison;
    matches!(
        comparison.status,
        hypersolve::AlgebraicRootComparisonStatus::Compared
            | hypersolve::AlgebraicRootComparisonStatus::SameRepresentation
    ) && comparison.ordering == Some(std::cmp::Ordering::Equal)
}

#[derive(Clone, Copy)]
pub(super) enum TrivariateParameterBinaryRelation2 {
    Sum,
    Product,
}

impl TrivariateParameterBinaryRelation2 {
    const fn operation(self) -> hypersolve::AlgebraicRootArithmeticOp {
        match self {
            Self::Sum => hypersolve::AlgebraicRootArithmeticOp::Add,
            Self::Product => hypersolve::AlgebraicRootArithmeticOp::Multiply,
        }
    }
}

/// Exact interval rejection keeps unrelated root triples out of resultant
/// construction. Bezier parameters lie in `[0, 1]`, so endpoint products are
/// ordered without a four-corner enclosure.
pub(super) fn parameter_binary_relation_may_overlap(
    left: &BezierParameter2,
    right: &BezierParameter2,
    result: &BezierParameter2,
    relation: TrivariateParameterBinaryRelation2,
) -> bool {
    let (left_lower, left_upper) = parameter_bounds(left);
    let (right_lower, right_upper) = parameter_bounds(right);
    let (result_lower, result_upper) = parameter_bounds(result);
    if matches!(relation, TrivariateParameterBinaryRelation2::Product)
        && [left_lower, left_upper, right_lower, right_upper]
            .into_iter()
            .any(|bound| {
                matches!(
                    compare_reals(bound, &Real::zero(), &CurveContext::STRICT),
                    Some(std::cmp::Ordering::Less) | None
                )
            })
    {
        return true;
    }
    let (image_lower, image_upper) = match relation {
        TrivariateParameterBinaryRelation2::Sum => {
            (left_lower + right_lower, left_upper + right_upper)
        }
        TrivariateParameterBinaryRelation2::Product => {
            (left_lower * right_lower, left_upper * right_upper)
        }
    };
    compare_reals(&image_upper, result_lower, &CurveContext::STRICT)
        != Some(std::cmp::Ordering::Less)
        && compare_reals(result_upper, &image_lower, &CurveContext::STRICT)
            != Some(std::cmp::Ordering::Less)
}

pub(super) fn trivariate_binary_related_parameter_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<RealSign>> {
    let parameters = [first, second, third];
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let root_degree = |axis: usize| match parameters[axis] {
        BezierParameter2::Exact(_) => 1,
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().degree(),
    };
    let mut candidates = [(0, 1, 2), (0, 2, 1), (1, 2, 0)];
    candidates.sort_by_key(|(left, right, product)| {
        (
            root_degree(*left).saturating_mul(root_degree(*right)),
            counts[*left]
                .saturating_add(counts[*product])
                .saturating_mul(counts[*right].saturating_add(counts[*product])),
        )
    });
    for (left_axis, right_axis, result_axis) in candidates {
        for relation in [
            TrivariateParameterBinaryRelation2::Sum,
            TrivariateParameterBinaryRelation2::Product,
        ] {
            if !parameter_binary_relation_may_overlap(
                parameters[left_axis],
                parameters[right_axis],
                parameters[result_axis],
                relation,
            ) || !exact_parameter_binary_relation(
                parameters[left_axis],
                parameters[right_axis],
                parameters[result_axis],
                relation.operation(),
            ) {
                continue;
            }
            let reduced = match relation {
                TrivariateParameterBinaryRelation2::Sum => {
                    trivariate_substitute_sum_axis(polynomial, left_axis, right_axis, result_axis)
                }
                TrivariateParameterBinaryRelation2::Product => trivariate_substitute_product_axis(
                    polynomial,
                    left_axis,
                    right_axis,
                    result_axis,
                ),
            };
            let Some(reduced) = reduced else {
                continue;
            };
            if let Classification::Decided(sign) = signed_bivariate_at_parameter_pair_exact_first(
                &reduced,
                parameters[left_axis],
                parameters[right_axis],
            )? {
                return Ok(Some(sign));
            }
        }
    }
    Ok(None)
}

pub(super) fn trivariate_affinely_related_parameter_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    let parameters = [first, second, third];
    for (retained_axis, substituted_axis) in [(0, 1), (0, 2), (1, 2)] {
        let Some((scale, offset)) = exact_parameter_affine_relation(
            parameters[retained_axis],
            parameters[substituted_axis],
        ) else {
            continue;
        };
        let remaining_axis = 3 - retained_axis - substituted_axis;
        let Some(reduced) = trivariate_substitute_affine_axis(
            polynomial,
            retained_axis,
            substituted_axis,
            &scale,
            &offset,
        ) else {
            continue;
        };
        if let Classification::Decided(sign) = signed_bivariate_at_parameter_pair(
            &reduced,
            parameters[retained_axis],
            parameters[remaining_axis],
            policy,
        )? {
            return Ok(Some(sign));
        }
    }
    Ok(None)
}

#[cold]
#[inline(never)]
pub(super) fn bivariate_remove_common_factors(
    mut equations: [BivariatePolynomial; 2],
) -> [BivariatePolynomial; 2] {
    let axis_report =
        extract_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1]);
    if axis_report.status == BivariatePolynomialAxisFactorStatus::Reduced
        && let Some(reduced) = axis_report.reduced_equations
    {
        equations = reduced;
    }
    loop {
        let degree = bivariate_storage_bidegree_sum(&equations[0])
            .saturating_add(bivariate_storage_bidegree_sum(&equations[1]));
        let mut next = None;
        for retained in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &equations[0],
                &equations[1],
                retained,
                CurveIntersectionResultantConfig {
                    min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                    max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                },
            );
            if !matches!(
                report.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ) {
                continue;
            }
            let Some(candidate) = report.reduced_equations else {
                continue;
            };
            let candidate_degree = bivariate_storage_bidegree_sum(&candidate[0])
                .saturating_add(bivariate_storage_bidegree_sum(&candidate[1]));
            if candidate_degree < degree {
                next = Some(candidate);
                break;
            }
        }
        let Some(reduced) = next else {
            return equations;
        };
        equations = reduced;
    }
}

/// Removes coefficient content only when the raw rational-function factor is
/// not already an exact polynomial divisor. Whole-tensor division remains the
/// authority in either case.
#[cold]
#[inline(never)]
pub(super) fn trivariate_normalize_and_divide_linear_axis_factor(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    raw: [BivariatePolynomial; 2],
) -> Option<([BivariatePolynomial; 2], TrivariatePolynomial)> {
    if let Some(quotient) = trivariate_divide_linear_axis_factor(polynomial, axis, &raw) {
        return Some((raw, quotient));
    }
    let primitive = bivariate_remove_common_factors(raw);
    let quotient = trivariate_divide_linear_axis_factor(polynomial, axis, &primitive)?;
    Some((primitive, quotient))
}

/// Splits a quadratic tensor axis when its bivariate discriminant is an exact
/// square, then verifies each recovered factor by exact tensor division.
#[cold]
#[inline(never)]
pub(super) fn trivariate_quadratic_axis_factorizations(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<Vec<(TrivariatePolynomial, TrivariatePolynomial)>> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let [constant, linear, quadratic]: [BivariatePolynomial; 3] = coefficients.try_into().ok()?;
    let linear_square = try_bivariate_multiply(&linear, &linear)?;
    let constant_quadratic = try_bivariate_multiply(&constant, &quadratic)?;
    let discriminant = bivariate_subtract(
        &linear_square,
        &bivariate_scale(constant_quadratic, &Real::from(4_i8)),
    );
    let square_root = bivariate_exact_square_root(&discriminant)?;
    let doubled_quadratic = bivariate_scale(quadratic, &Real::from(2_i8));
    let mut factorizations: Vec<(TrivariatePolynomial, TrivariatePolynomial)> =
        Vec::with_capacity(2);
    for constant in [
        bivariate_add(&linear, &square_root),
        bivariate_subtract(&linear, &square_root),
    ] {
        let raw = [constant, doubled_quadratic.clone()];
        let Some((factor_coefficients, quotient)) =
            trivariate_normalize_and_divide_linear_axis_factor(polynomial, axis, raw)
        else {
            continue;
        };
        let factor =
            trivariate_from_axis_bivariate_coefficients(&factor_coefficients, axis, remaining)?;
        if factorizations.iter().any(|(existing, _)| {
            existing.coefficients == factor.coefficients
                || existing.coefficients == quotient.coefficients
        }) {
            continue;
        }
        factorizations.push((factor, quotient));
    }
    (!factorizations.is_empty()).then_some(factorizations)
}

/// Splits a cubic tensor axis when the cubic has a repeated linear factor over
/// the exact bivariate fraction field.
///
/// For `a*x^3+b*x^2+c*x+d`, a non-triple repeated root is
/// `(9*a*d-b*c)/(2*(b^2-3*a*c))`; a triple root is `-b/(3*a)`.
/// These expressions only propose polynomial coefficient pairs. Common
/// bivariate content is removed when necessary, and exact division of the full
/// tensor is the final authority, so neither a vanishing invariant nor a
/// rational-function candidate can create a false factor.
#[cold]
#[inline(never)]
pub(super) fn trivariate_repeated_cubic_axis_factorizations(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<Vec<(TrivariatePolynomial, TrivariatePolynomial)>> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let [constant, linear, quadratic, cubic]: [BivariatePolynomial; 4] =
        coefficients.try_into().ok()?;
    let _ = bivariate_exact_nonzero_metadata(&cubic)??;
    if cubic_specialization_rejects_repeated_factor([&constant, &linear, &quadratic, &cubic]) {
        return None;
    }

    let quadratic_square = try_bivariate_multiply(&quadratic, &quadratic)?;
    let cubic_linear = try_bivariate_multiply(&cubic, &linear)?;
    let delta_zero = bivariate_subtract(
        &quadratic_square,
        &bivariate_scale(cubic_linear.clone(), &Real::from(3_i8)),
    );
    let raw = if bivariate_exact_nonzero_metadata(&delta_zero)?.is_none() {
        // Delta-one vanishes exactly for a triple root once delta-zero does.
        // Reject x^3+d and related square-free cubics before component work.
        let quadratic_cube = try_bivariate_multiply(&quadratic_square, &quadratic)?;
        let cubic_quadratic_linear = try_bivariate_multiply(&cubic_linear, &quadratic)?;
        let cubic_square = try_bivariate_multiply(&cubic, &cubic)?;
        let cubic_square_constant = try_bivariate_multiply(&cubic_square, &constant)?;
        let delta_one = bivariate_add(
            &bivariate_subtract(
                &bivariate_scale(quadratic_cube, &Real::from(2_i8)),
                &bivariate_scale(cubic_quadratic_linear, &Real::from(9_i8)),
            ),
            &bivariate_scale(cubic_square_constant, &Real::from(27_i8)),
        );
        if bivariate_exact_nonzero_metadata(&delta_one)?.is_some() {
            return None;
        }
        drop(delta_zero);
        drop(constant);
        drop(linear);
        [quadratic, bivariate_scale(cubic, &Real::from(3_i8))]
    } else {
        let quadratic_linear = try_bivariate_multiply(&quadratic, &linear)?;
        let cubic_constant = try_bivariate_multiply(&cubic, &constant)?;
        let factor_constant = bivariate_subtract(
            &quadratic_linear,
            &bivariate_scale(cubic_constant, &Real::from(9_i8)),
        );
        let factor_linear = bivariate_scale(delta_zero, &Real::from(2_i8));
        // At x=-factor_constant/factor_linear, the derivative numerator
        // must vanish for a repeated root. This cheap structural rejection
        // keeps square-free cubics out of general component extraction.
        let constant_square = try_bivariate_multiply(&factor_constant, &factor_constant)?;
        let constant_linear = try_bivariate_multiply(&factor_constant, &factor_linear)?;
        let linear_square = try_bivariate_multiply(&factor_linear, &factor_linear)?;
        let derivative_remainder = bivariate_add(
            &bivariate_subtract(
                &bivariate_scale(
                    try_bivariate_multiply(&cubic, &constant_square)?,
                    &Real::from(3_i8),
                ),
                &bivariate_scale(
                    try_bivariate_multiply(&quadratic, &constant_linear)?,
                    &Real::from(2_i8),
                ),
            ),
            &try_bivariate_multiply(&linear, &linear_square)?,
        );
        if bivariate_exact_nonzero_metadata(&derivative_remainder)?.is_some() {
            return None;
        }
        drop((constant, linear, quadratic, cubic));
        [factor_constant, factor_linear]
    };
    drop((quadratic_square, cubic_linear));
    let (factor_coefficients, quotient) =
        trivariate_normalize_and_divide_linear_axis_factor(polynomial, axis, raw)?;
    trivariate_divide_linear_axis_factor(&quotient, axis, &factor_coefficients)?;
    let factor =
        trivariate_from_axis_bivariate_coefficients(&factor_coefficients, axis, remaining)?;
    Some(vec![(factor, quotient)])
}

pub(super) fn trivariate_rational_multi_affine_factor_from_scale(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    remaining: [usize; 2],
    anchor_factor: &BivariatePolynomial,
    top_factor: &BivariatePolynomial,
    scale: &Real,
    anchor: &Real,
    lift_coordinate: usize,
) -> Option<(TrivariatePolynomial, TrivariatePolynomial)> {
    let raw = rational_multi_affine_lift_factor_coefficients(
        anchor_factor,
        top_factor,
        scale,
        anchor,
        lift_coordinate,
    )?;
    let (factor_coefficients, quotient) =
        trivariate_normalize_and_divide_linear_axis_factor(polynomial, axis, raw)?;
    let factor =
        trivariate_from_axis_bivariate_coefficients(&factor_coefficients, axis, remaining)?;
    Some((factor, quotient))
}

/// Recovers one exact rational multi-affine factor from a resource-bounded
/// tensor in `axis`. Specializations only propose bilinear slice factors.
/// Hypersolve exact division proves each slice, derives the inter-slice scale
/// from the translated first-order coefficient or an exact two-anchor
/// projective alignment for repeated factors, and finally proves the complete
/// trivariate factor. Cubic through octic tensors retain exhaustive proposal
/// enumeration; higher degrees receive bounded first-factor passes. Unsupported
/// coefficient towers, exhausted proposal budgets, or degenerate slices make
/// no claim.
#[cold]
#[inline(never)]
pub(super) fn trivariate_rational_multi_affine_axis_factorizations(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<Vec<(TrivariatePolynomial, TrivariatePolynomial)>> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    if !(4..=MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS).contains(&coefficients.len()) {
        return None;
    }
    let exhaustive = coefficients.len() <= MAX_EXHAUSTIVE_MULTI_AFFINE_COEFFICIENTS;
    // Try the first proved slice factors before enumerating every divisor. The
    // exhaustive pass remains authoritative in its measured-safe envelope.
    // Higher degrees get a capped first-Taylor pass and, when needed, a capped
    // repeated-factor alignment pass.
    for (maximum_factorizations, maximum_proposals, align_anchors, enabled) in [
        (1, MAX_FIRST_BILINEAR_FACTOR_PROPOSALS, false, true),
        (
            MAX_BOUNDED_BILINEAR_FACTORIZATIONS,
            MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS,
            false,
            !exhaustive,
        ),
        (usize::MAX, usize::MAX, false, exhaustive),
        (
            MAX_BOUNDED_BILINEAR_FACTORIZATIONS,
            MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS,
            true,
            !exhaustive,
        ),
        (usize::MAX, usize::MAX, true, exhaustive),
    ] {
        if !enabled {
            continue;
        }
        for lift_coordinate in 0..2 {
            let lift_degree = trivariate_axis_lift_degree(&coefficients, lift_coordinate)?;
            if lift_degree == 0 {
                continue;
            }
            let top_slice =
                trivariate_axis_lift_power_slice(&coefficients, lift_coordinate, lift_degree)?;
            let top_factorizations = bivariate_bilinear_factorizations_bounded(
                &top_slice,
                maximum_factorizations,
                maximum_proposals,
            );
            if top_factorizations.is_empty() {
                continue;
            }
            for anchor in [1_i8, 0, -1, 2].map(Real::from) {
                let anchor_slice = trivariate_axis_lift_taylor_slice(
                    &coefficients,
                    lift_coordinate,
                    &anchor,
                    false,
                )?;
                let first_taylor_slice = trivariate_axis_lift_taylor_slice(
                    &coefficients,
                    lift_coordinate,
                    &anchor,
                    true,
                )?;
                for (anchor_factor, anchor_quotient) in bivariate_bilinear_factorizations_bounded(
                    &anchor_slice,
                    maximum_factorizations,
                    maximum_proposals,
                ) {
                    for (top_factor, _) in &top_factorizations {
                        if !align_anchors {
                            if let Some(scale) = rational_multi_affine_lift_scale(
                                &first_taylor_slice,
                                &anchor_factor,
                                top_factor,
                                &anchor_quotient,
                            ) && let Some(factorization) =
                                trivariate_rational_multi_affine_factor_from_scale(
                                    polynomial,
                                    axis,
                                    remaining,
                                    &anchor_factor,
                                    top_factor,
                                    &scale,
                                    &anchor,
                                    lift_coordinate,
                                )
                            {
                                return Some(vec![factorization]);
                            }
                            continue;
                        }
                        for other_anchor in [1_i8, 0, -1, 2].map(Real::from) {
                            if other_anchor == anchor {
                                continue;
                            }
                            let other_slice = trivariate_axis_lift_taylor_slice(
                                &coefficients,
                                lift_coordinate,
                                &other_anchor,
                                false,
                            )?;
                            let anchor_delta = &other_anchor - &anchor;
                            for (other_factor, _) in bivariate_bilinear_factorizations_bounded(
                                &other_slice,
                                maximum_factorizations,
                                maximum_proposals,
                            ) {
                                let Some(scale) = rational_multi_affine_lift_scale_from_anchor_pair(
                                    &anchor_factor,
                                    &other_factor,
                                    top_factor,
                                    &anchor_delta,
                                ) else {
                                    continue;
                                };
                                let Some(factorization) =
                                    trivariate_rational_multi_affine_factor_from_scale(
                                        polynomial,
                                        axis,
                                        remaining,
                                        &anchor_factor,
                                        top_factor,
                                        &scale,
                                        &anchor,
                                        lift_coordinate,
                                    )
                                else {
                                    continue;
                                };
                                return Some(vec![factorization]);
                            }
                        }
                    }
                }
            }
        }
    }
    None
}

pub(super) fn signed_bivariate_at_parameter_pair_exact_first(
    polynomial: &BivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
) -> CurveResult<Classification<RealSign>> {
    let policy = &CurveContext::STRICT;
    let direct = signed_bivariate_at_parameter_pair(polynomial, first, second, policy)?;
    if matches!(direct, Classification::Decided(_)) {
        return Ok(direct);
    }
    if let (BezierParameter2::Algebraic(first), BezierParameter2::Algebraic(second)) =
        (first, second)
        && bivariate_parameter_pair_is_exact_common_root(polynomial, first, second)
    {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    if let Some(sign) =
        bivariate_parameter_pair_strict_sign_by_refinement(polynomial, first, second, policy)?
    {
        return Ok(Classification::Decided(sign));
    }
    Ok(direct)
}

pub(super) fn signed_bivariate_at_parameter_pair_refinement_first(
    polynomial: &BivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
) -> CurveResult<Classification<RealSign>> {
    let policy = &CurveContext::STRICT;
    if let Some(sign) =
        bivariate_parameter_pair_strict_sign_by_refinement(polynomial, first, second, policy)?
    {
        return Ok(Classification::Decided(sign));
    }
    signed_bivariate_at_parameter_pair(polynomial, first, second, policy)
}

pub(super) fn trivariate_linear_axis_resultant_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<RealSign>> {
    let parameters = [first, second, third];
    let root_degree = |axis: usize| match parameters[axis] {
        BezierParameter2::Exact(_) => 0,
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().degree(),
    };
    let mut axes = [0, 1, 2];
    axes.sort_by_key(|axis| std::cmp::Reverse(root_degree(*axis)));
    for axis in axes {
        let BezierParameter2::Algebraic(axis_parameter) = parameters[axis] else {
            continue;
        };
        let Some((constant, linear, remaining)) =
            trivariate_linear_axis_coefficients(polynomial, axis)
        else {
            continue;
        };
        let first_remaining = parameters[remaining[0]];
        let second_remaining = parameters[remaining[1]];

        // If the linear coefficient itself vanishes at the retained pair, the
        // full trivariate value is exactly its constant coefficient.
        if signed_bivariate_at_parameter_pair_refinement_first(
            &linear,
            first_remaining,
            second_remaining,
        )? == Classification::Decided(RealSign::Zero)
        {
            if let Classification::Decided(sign) =
                signed_bivariate_at_parameter_pair_refinement_first(
                    &constant,
                    first_remaining,
                    second_remaining,
                )?
            {
                return Ok(Some(sign));
            }
            continue;
        }

        let Some(resultant) = bivariate_linear_root_resultant(
            &constant,
            &linear,
            axis_parameter.polynomial().coefficients(),
        ) else {
            continue;
        };
        if signed_bivariate_at_parameter_pair_exact_first(
            &resultant,
            first_remaining,
            second_remaining,
        )? != Classification::Decided(RealSign::Zero)
        {
            continue;
        }

        // A zero resultant proves the unique root of the retained linear
        // polynomial is some root of the axis defining polynomial. Strictly
        // opposite endpoint signs place that root inside this selected
        // one-root isolator, proving it is the authored root rather than a
        // foreign conjugate.
        let (lower, upper) = parameter_bounds(parameters[axis]);
        let lower_sign = {
            let mut value = constant.clone();
            if bivariate_add_scaled_assign(&mut value, &linear, lower).is_none() {
                return Ok(None);
            }
            signed_bivariate_at_parameter_pair_refinement_first(
                &value,
                first_remaining,
                second_remaining,
            )?
        };
        let upper_sign = {
            let mut value = constant.clone();
            if bivariate_add_scaled_assign(&mut value, &linear, upper).is_none() {
                return Ok(None);
            }
            signed_bivariate_at_parameter_pair_refinement_first(
                &value,
                first_remaining,
                second_remaining,
            )?
        };
        if matches!(
            (lower_sign, upper_sign),
            (
                Classification::Decided(RealSign::Negative),
                Classification::Decided(RealSign::Positive)
            ) | (
                Classification::Decided(RealSign::Positive),
                Classification::Decided(RealSign::Negative)
            )
        ) {
            return Ok(Some(RealSign::Zero));
        }
    }
    Ok(None)
}

pub(super) fn trivariate_restrict_to_parameter_box(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> TrivariatePolynomial {
    let (first_start, first_end) = parameter_bounds(first);
    let (second_start, second_end) = parameter_bounds(second);
    let (third_start, third_end) = parameter_bounds(third);

    trivariate_restrict_to_box_bounds(
        polynomial,
        [
            (first_start, first_end),
            (second_start, second_end),
            (third_start, third_end),
        ],
    )
}

pub(super) fn trivariate_unit_cube_strict_bernstein_sign(
    polynomial: TrivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    let (a_count, b_count, c_count) = polynomial.dimensions();
    if a_count == 0 || b_count == 0 || c_count == 0 {
        return Ok(None);
    }
    let mut controls = polynomial.coefficients;
    for rows in &mut controls {
        for row in rows {
            *row = power_to_bernstein_coefficients(row, c_count - 1)?;
        }
    }
    for rows in &mut controls {
        let first_row = rows.first().cloned().unwrap_or_default();
        for (c, _) in first_row.iter().enumerate() {
            let coefficients = rows.iter().map(|row| row[c].clone()).collect::<Vec<_>>();
            for (row, coefficient) in rows
                .iter_mut()
                .zip(power_to_bernstein_coefficients(&coefficients, b_count - 1)?)
            {
                row[c] = coefficient;
            }
        }
    }
    let mut strict_sign = None;
    for (b, row) in controls[0].iter().enumerate() {
        for (c, _) in row.iter().enumerate() {
            let coefficients = (0..a_count)
                .map(|a| controls[a][b][c].clone())
                .collect::<Vec<_>>();
            for control in power_to_bernstein_coefficients(&coefficients, a_count - 1)? {
                let Some(sign @ (RealSign::Negative | RealSign::Positive)) =
                    real_sign(&control, policy)
                else {
                    return Ok(None);
                };
                match strict_sign {
                    Some(previous) if previous != sign => return Ok(None),
                    Some(_) => {}
                    None => strict_sign = Some(sign),
                }
            }
        }
    }
    Ok(strict_sign)
}

pub(super) fn trivariate_multi_affine_parameter_box_strict_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
    policy: &CurveContext,
) -> Option<RealSign> {
    debug_assert!({
        let dimensions = polynomial.dimensions();
        dimensions.0 <= 2 && dimensions.1 <= 2 && dimensions.2 <= 2
    });
    let (first_start, first_end) = parameter_bounds(first);
    let (second_start, second_end) = parameter_bounds(second);
    let (third_start, third_end) = parameter_bounds(third);
    let bounds = [
        [first_start, first_end],
        [second_start, second_end],
        [third_start, third_end],
    ];
    let coefficients: [[[Real; 2]; 2]; 2] = std::array::from_fn(|first| {
        std::array::from_fn(|second| {
            std::array::from_fn(|third| {
                polynomial
                    .coefficients
                    .get(first)
                    .and_then(|rows| rows.get(second))
                    .and_then(|row| row.get(third))
                    .cloned()
                    .unwrap_or_else(Real::zero)
            })
        })
    });
    let affine = |constant: &Real, linear: &Real, parameter: &Real| constant + linear * parameter;
    let third_values: [[[Real; 2]; 2]; 2] = std::array::from_fn(|first| {
        std::array::from_fn(|second| {
            std::array::from_fn(|third| {
                affine(
                    &coefficients[first][second][0],
                    &coefficients[first][second][1],
                    bounds[2][third],
                )
            })
        })
    });
    let second_values: [[[Real; 2]; 2]; 2] = std::array::from_fn(|first| {
        std::array::from_fn(|second| {
            std::array::from_fn(|third| {
                affine(
                    &third_values[first][0][third],
                    &third_values[first][1][third],
                    bounds[1][second],
                )
            })
        })
    });
    let mut strict_sign = None;
    for first_parameter in &bounds[0] {
        for (constant_row, linear_row) in second_values[0].iter().zip(&second_values[1]) {
            for (constant, linear) in constant_row.iter().zip(linear_row) {
                let control = affine(constant, linear, first_parameter);
                let Some(sign @ (RealSign::Negative | RealSign::Positive)) =
                    real_sign(&control, policy)
                else {
                    return None;
                };
                match strict_sign {
                    Some(previous) if previous != sign => return None,
                    Some(_) => {}
                    None => strict_sign = Some(sign),
                }
            }
        }
    }
    strict_sign
}

pub(super) fn trivariate_existing_symbolic_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<RealSign>> {
    if trivariate_structurally_zero(polynomial, &CurveContext::STRICT) {
        return Ok(Some(RealSign::Zero));
    }
    if let Some(sign) =
        trivariate_unit_cube_strict_bernstein_sign(polynomial.clone(), &CurveContext::STRICT)?
    {
        return Ok(Some(sign));
    }
    if let Some(sign) = trivariate_affinely_related_parameter_sign(
        polynomial,
        first,
        second,
        third,
        &CurveContext::STRICT,
    )? {
        return Ok(Some(sign));
    }
    if let Some(sign) = trivariate_linear_axis_resultant_sign(polynomial, first, second, third)? {
        return Ok(Some(sign));
    }
    trivariate_binary_related_parameter_sign(polynomial, first, second, third)
}

pub(super) fn trivariate_factored_component_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
    remaining_splits: usize,
) -> CurveResult<Option<RealSign>> {
    if let Some(sign) = trivariate_existing_symbolic_sign(polynomial, first, second, third)? {
        return Ok(Some(sign));
    }
    if remaining_splits == 0 {
        return Ok(None);
    }
    trivariate_bounded_factor_sign_with_budget(polynomial, first, second, third, remaining_splits)
}

pub(super) fn trivariate_bounded_factor_sign_with_budget(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
    remaining_splits: usize,
) -> CurveResult<Option<RealSign>> {
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    // A denser exact high-degree leading slice is less likely to lose
    // candidate factors at the bounded specializations. This is scheduling
    // only: an undecidable coefficient gives no preference and cannot create
    // a claim.
    let leading_nonzero_count = |axis: usize| {
        if counts[axis] < 6 {
            return 0;
        }
        let mut count = 0_usize;
        for (first, rows) in polynomial.coefficients.iter().enumerate() {
            for (second, row) in rows.iter().enumerate() {
                for (third, coefficient) in row.iter().enumerate() {
                    if [first, second, third][axis] + 1 != counts[axis] {
                        continue;
                    }
                    match real_sign(coefficient, &CurveContext::STRICT) {
                        Some(RealSign::Zero) => {}
                        Some(RealSign::Negative | RealSign::Positive) => count += 1,
                        None => return 0,
                    }
                }
            }
        }
        count
    };
    let leading_nonzero_counts: [usize; 3] = std::array::from_fn(leading_nonzero_count);
    let mut axes = [0, 1, 2];
    axes.sort_by_key(|axis| {
        (
            counts[*axis],
            std::cmp::Reverse(leading_nonzero_counts[*axis]),
        )
    });
    for axis in axes {
        let factorizations = match counts[axis] {
            3 => trivariate_quadratic_axis_factorizations(polynomial, axis),
            4 => trivariate_repeated_cubic_axis_factorizations(polynomial, axis)
                .or_else(|| trivariate_rational_multi_affine_axis_factorizations(polynomial, axis)),
            5..=MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS => {
                trivariate_rational_multi_affine_axis_factorizations(polynomial, axis)
            }
            _ => None,
        };
        let Some(factorizations) = factorizations else {
            continue;
        };
        for (factor, quotient) in factorizations {
            let next_budget = remaining_splits.saturating_sub(1);
            let factor_sign =
                trivariate_factored_component_sign(&factor, first, second, third, next_budget)?;
            if factor_sign == Some(RealSign::Zero) {
                return Ok(factor_sign);
            }
            let quotient_sign =
                trivariate_factored_component_sign(&quotient, first, second, third, next_budget)?;
            if quotient_sign == Some(RealSign::Zero) {
                return Ok(quotient_sign);
            }
            if let (Some(factor_sign), Some(quotient_sign)) = (factor_sign, quotient_sign) {
                return Ok(Some(product_sign(factor_sign, quotient_sign)));
            }
        }
    }
    Ok(None)
}

/// Replays the bounded exact quadratic, repeated-cubic, and rational
/// multi-affine factor authorities. The tensor degree supplies the useful
/// split budget, capped at the largest balanced multi-affine product admitted
/// by the existing control-count ceiling.
#[cold]
#[inline(never)]
pub(super) fn trivariate_bounded_factor_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<RealSign>> {
    let dimensions = polynomial.dimensions();
    let total_degree = dimensions
        .0
        .saturating_sub(1)
        .saturating_add(dimensions.1.saturating_sub(1))
        .saturating_add(dimensions.2.saturating_sub(1));
    trivariate_bounded_factor_sign_with_budget(
        polynomial,
        first,
        second,
        third,
        total_degree.min(MAX_TRIVARIATE_EXACT_FACTOR_SPLITS),
    )
}

/// Preserves authored products that quotient-ring reduction may expand across
/// a defining-polynomial boundary. Separable contents and bounded coupled
/// factors are both replayed on the original tensor before this path makes an
/// exact claim.
#[cold]
#[inline(never)]
pub(super) fn trivariate_content_and_bounded_factor_sign(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
) -> CurveResult<Option<RealSign>> {
    if let Some(sign) = trivariate_bounded_factor_sign(polynomial, first, second, third)? {
        return Ok(Some(sign));
    }
    let Some((primitive, factor_sign)) =
        trivariate_strip_axis_contents(polynomial, first, second, third)?
    else {
        return Ok(None);
    };
    if factor_sign == Some(RealSign::Zero)
        || trivariate_structurally_zero(&primitive, &CurveContext::STRICT)
    {
        return Ok(Some(RealSign::Zero));
    }
    let Some(sign) = trivariate_bounded_factor_sign(&primitive, first, second, third)? else {
        return Ok(None);
    };
    if sign == RealSign::Zero {
        return Ok(Some(sign));
    }
    Ok(factor_sign.map(|factor_sign| product_sign(factor_sign, sign)))
}

pub(super) fn trivariate_exceeds_bounded_symbolic_schedule(
    polynomial: &TrivariatePolynomial,
) -> bool {
    let dimensions = polynomial.dimensions();
    [dimensions.0, dimensions.1, dimensions.2]
        .into_iter()
        .any(|count| count > MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS)
        || dimensions
            .0
            .checked_mul(dimensions.1)
            .and_then(|count| count.checked_mul(dimensions.2))
            .is_none_or(|count| count > MAX_TRIVARIATE_BOUNDED_FAST_PATH_CONTROLS)
}

pub(super) fn selected_parameter_representations<const N: usize>(
    parameters: [&BezierParameter2; N],
) -> [AlgebraicRootRepresentation; N] {
    let strict = CurveContext::STRICT;
    parameters.map(|parameter| match parameter {
        BezierParameter2::Exact(parameter) => {
            AlgebraicRootRepresentation::from_exact_value(parameter)
        }
        BezierParameter2::Algebraic(parameter) => parameter_representation(parameter, &strict),
    })
}

#[cold]
pub(super) fn trivariate_parameter_triple_sign_by_refinement(
    polynomial: &TrivariatePolynomial,
    first: &BezierParameter2,
    second: &BezierParameter2,
    third: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if trivariate_structurally_zero(polynomial, policy) {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    let parameters = [first, second, third];
    for (axis, parameter) in parameters.into_iter().enumerate() {
        let BezierParameter2::Exact(parameter) = parameter else {
            continue;
        };
        let Some((specialized, remaining_axes)) =
            trivariate_specialize_axis_bivariate(polynomial, axis, parameter)
        else {
            continue;
        };
        return signed_bivariate_at_parameter_pair(
            &specialized,
            parameters[remaining_axes[0]],
            parameters[remaining_axes[1]],
            policy,
        );
    }
    if trivariate_exceeds_bounded_symbolic_schedule(polynomial) {
        let Some(polynomial) = polynomial.to_dense_polynomial() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let sources = selected_parameter_representations(parameters);
        return dense_polynomial_tuple_sign_owned(polynomial, &sources, policy);
    }
    let mut first_refinement = BezierParameterRefinement2::new(first, policy);
    let mut second_refinement = BezierParameterRefinement2::new(second, policy);
    let mut third_refinement = BezierParameterRefinement2::new(third, policy);
    let dimensions = polynomial.dimensions();
    let is_multi_affine = dimensions.0 <= 2 && dimensions.1 <= 2 && dimensions.2 <= 2;
    for target_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let first_refined = first_refinement.refine_to(target_steps);
        let second_refined = second_refinement.refine_to(target_steps);
        let third_refined = third_refinement.refine_to(target_steps);
        let strict_sign = if is_multi_affine {
            trivariate_multi_affine_parameter_box_strict_sign(
                polynomial,
                first_refined,
                second_refined,
                third_refined,
                policy,
            )
        } else {
            trivariate_unit_cube_strict_bernstein_sign(
                trivariate_restrict_to_parameter_box(
                    polynomial,
                    first_refined,
                    second_refined,
                    third_refined,
                ),
                policy,
            )?
        };
        if let Some(sign) = strict_sign {
            return Ok(Classification::Decided(sign));
        }
        // Let common nonzero boxes separate before constructing a field
        // relation, but do not grow a correlated zero to the 512-bit terminal
        // before exact symbolic substitution. Eight bisections preserve the
        // mapped-cusp nonzero fast path while bounding the zero-path work.
        if target_steps == 8
            && let Some(sign) = trivariate_affinely_related_parameter_sign(
                polynomial, first, second, third, policy,
            )?
        {
            return Ok(Classification::Decided(sign));
        }
        if target_steps == 8
            && let Some(sign) =
                trivariate_linear_axis_resultant_sign(polynomial, first, second, third)?
        {
            return Ok(Classification::Decided(sign));
        }
        if target_steps == 8 {
            let original_dimensions = polynomial.dimensions();
            let original_cubic_factor_attempted = [
                original_dimensions.0,
                original_dimensions.1,
                original_dimensions.2,
            ]
            .contains(&4);
            // Degree-three defining-polynomial reduction can preserve the
            // selected value while expanding an authored repeated factor.
            // Give an original cubic axis one exact replay first, and record
            // the attempt so a failed square-free tensor is not repeated.
            if original_cubic_factor_attempted
                && let Some(sign) =
                    trivariate_content_and_bounded_factor_sign(polynomial, first, second, third)?
            {
                return Ok(Classification::Decided(sign));
            }
            let reduced =
                trivariate_reduce_selected_root_relations(polynomial, first, second, third);
            if let Some(reduced) = reduced.as_ref() {
                // Quotient-ring reduction changes the tensor away from the
                // roots, but preserves its value at this selected root tuple
                // exactly. A strict sign over the selected box is therefore
                // still valid.
                if trivariate_structurally_zero(reduced, &CurveContext::STRICT) {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                let restricted = trivariate_restrict_to_parameter_box(
                    reduced,
                    first_refinement.refine_to(target_steps),
                    second_refinement.refine_to(target_steps),
                    third_refinement.refine_to(target_steps),
                );
                if let Some(sign) =
                    trivariate_unit_cube_strict_bernstein_sign(restricted, &CurveContext::STRICT)?
                {
                    return Ok(Classification::Decided(sign));
                }
                if let Some(sign) = trivariate_affinely_related_parameter_sign(
                    reduced,
                    first,
                    second,
                    third,
                    &CurveContext::STRICT,
                )? {
                    return Ok(Classification::Decided(sign));
                }
                if let Some(sign) =
                    trivariate_linear_axis_resultant_sign(reduced, first, second, third)?
                {
                    return Ok(Classification::Decided(sign));
                }
            }
            let symbolic = reduced.as_ref().unwrap_or(polynomial);
            if let Some(sign) =
                trivariate_binary_related_parameter_sign(symbolic, first, second, third)?
            {
                return Ok(Classification::Decided(sign));
            }
            let stripped = trivariate_strip_axis_contents(symbolic, first, second, third)?;
            if let Some((primitive, factor_sign)) = &stripped {
                if *factor_sign == Some(RealSign::Zero)
                    || trivariate_structurally_zero(primitive, &CurveContext::STRICT)
                {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                let restricted = trivariate_restrict_to_parameter_box(
                    primitive,
                    first_refinement.refine_to(target_steps),
                    second_refinement.refine_to(target_steps),
                    third_refinement.refine_to(target_steps),
                );
                let primitive_sign = if let Some(sign) =
                    trivariate_unit_cube_strict_bernstein_sign(restricted, &CurveContext::STRICT)?
                {
                    Some(sign)
                } else if let Some(sign) = trivariate_affinely_related_parameter_sign(
                    primitive,
                    first,
                    second,
                    third,
                    &CurveContext::STRICT,
                )? {
                    Some(sign)
                } else if let Some(sign) =
                    trivariate_linear_axis_resultant_sign(primitive, first, second, third)?
                {
                    Some(sign)
                } else {
                    trivariate_binary_related_parameter_sign(primitive, first, second, third)?
                };
                if primitive_sign == Some(RealSign::Zero) {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                if let (Some(primitive_sign), Some(factor_sign)) = (primitive_sign, *factor_sign) {
                    return Ok(Classification::Decided(product_sign(
                        primitive_sign,
                        factor_sign,
                    )));
                }
            }
            let factor_source = stripped
                .as_ref()
                .map(|(primitive, _)| primitive)
                .unwrap_or(symbolic);
            // A quadratic exact-square discriminant or a cubic repeated-root
            // invariant can propose a linear-axis factor. Split only after
            // exact factor-content removal and whole-tensor division have
            // replayed it; recursively replay the bounded quadratic quotient.
            if let Some(sign) = trivariate_bounded_factor_sign(factor_source, first, second, third)?
            {
                if sign == RealSign::Zero {
                    return Ok(Classification::Decided(sign));
                }
                let removed_sign = stripped
                    .as_ref()
                    .map_or(Some(RealSign::Positive), |(_, factor_sign)| *factor_sign);
                if let Some(removed_sign) = removed_sign {
                    return Ok(Classification::Decided(product_sign(removed_sign, sign)));
                }
            }
            // Reducing powers modulo a selected defining polynomial can
            // obscure an authored factorization even though it preserves the
            // value at the root tuple. Replay the exact original product only
            // after the smaller reduced tensor has exhausted its authorities.
            if reduced.is_some()
                && !original_cubic_factor_attempted
                && let Some(sign) =
                    trivariate_content_and_bounded_factor_sign(polynomial, first, second, third)?
            {
                return Ok(Classification::Decided(sign));
            }
        }
    }
    if policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Ok(Classification::Decided(RealSign::Zero))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }
}

pub(super) fn quadrivariate_structurally_zero(polynomial: &QuadrivariatePolynomial2) -> bool {
    polynomial
        .coefficients
        .iter()
        .all(|coefficient| real_sign(coefficient, &CurveContext::STRICT) == Some(RealSign::Zero))
}

pub(super) fn quadrivariate_specialize_axis_trivariate(
    polynomial: &QuadrivariatePolynomial2,
    axis: usize,
    value: &Real,
) -> Option<(TrivariatePolynomial, [usize; 3])> {
    if axis >= 4 {
        return None;
    }
    let remaining = match axis {
        0 => [1, 2, 3],
        1 => [0, 2, 3],
        2 => [0, 1, 3],
        3 => [0, 1, 2],
        _ => unreachable!(),
    };
    let dimensions = remaining.map(|remaining| polynomial.dimensions[remaining]);
    let mut coefficients = try_zero_trivariate_coefficients(dimensions)?;
    let mut fiber = Vec::new();
    fiber.try_reserve_exact(polynomial.dimensions[axis]).ok()?;
    for (first, planes) in coefficients.iter_mut().enumerate() {
        for (second, row) in planes.iter_mut().enumerate() {
            for (third, coefficient) in row.iter_mut().enumerate() {
                let retained = [first, second, third];
                fiber.clear();
                for power in 0..polynomial.dimensions[axis] {
                    let mut exponents = [0; 4];
                    exponents[axis] = power;
                    for retained_axis in 0..3 {
                        exponents[remaining[retained_axis]] = retained[retained_axis];
                    }
                    fiber.push(
                        polynomial
                            .coefficient(exponents)
                            .cloned()
                            .unwrap_or_else(Real::zero),
                    );
                }
                *coefficient = Real::eval_poly(&fiber, value);
            }
        }
    }
    Some((TrivariatePolynomial { coefficients }, remaining))
}

pub(super) fn quadrivariate_parameter_tuple_sign_by_refinement(
    polynomial: &QuadrivariatePolynomial2,
    parameters: [&BezierParameter2; 4],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if quadrivariate_structurally_zero(polynomial) {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    for (axis, parameter) in parameters.into_iter().enumerate() {
        let BezierParameter2::Exact(value) = parameter else {
            continue;
        };
        let Some((specialized, remaining)) =
            quadrivariate_specialize_axis_trivariate(polynomial, axis, value)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        return trivariate_parameter_triple_sign_by_refinement(
            &specialized,
            parameters[remaining[0]],
            parameters[remaining[1]],
            parameters[remaining[2]],
            policy,
        );
    }
    let Some(polynomial) = polynomial.to_dense_polynomial() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let sources = selected_parameter_representations(parameters);
    dense_polynomial_tuple_sign_owned(polynomial, &sources, policy)
}
