//! Exact parameter images between rational Bezier charts.
//!
//! Overlap correspondences, conic parameter maps and rational map image
//! polynomials name one curve's parameter on another curve's chart without
//! materializing a global projection unless a candidate requires it.

use super::*;

pub(super) fn complete_rational_bezier_image_overlap(
    reversed: bool,
) -> Classification<RationalBezierSharedComponentReplay> {
    Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
        RationalBezierIntersectionOverlap2 {
            first_range: BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            second_range: if reversed {
                BezierParameterRange2::from_exact(Real::one(), Real::zero())
            } else {
                BezierParameterRange2::from_exact(Real::zero(), Real::one())
            },
            orientation: if reversed {
                CurveOverlapOrientation2::Reversed
            } else {
                CurveOverlapOrientation2::Same
            },
            endpoint_inclusion: [true, true],
        },
    ))
}

pub(super) fn endpoint_projective_parameter_image(
    parameter: &BezierParameter2,
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let (numerator, denominator) = endpoint_projective_parameter_coefficients(
        second_to_first_scale,
        reversed,
        first_to_second,
    );
    projective_parameter_image(parameter, &numerator, &denominator, range, policy)
}

pub(super) fn endpoint_projective_parameter_coefficients(
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
) -> ([Real; 2], [Real; 2]) {
    let one = Real::one();
    let zero = Real::zero();
    match (reversed, first_to_second) {
        (false, true) => (
            [zero, one.clone()],
            [second_to_first_scale.clone(), &one - second_to_first_scale],
        ),
        (false, false) => (
            [Real::zero(), second_to_first_scale.clone()],
            [one.clone(), second_to_first_scale - &one],
        ),
        (true, _) => (
            [
                second_to_first_scale.clone(),
                -second_to_first_scale.clone(),
            ],
            [second_to_first_scale.clone(), &one - second_to_first_scale],
        ),
    }
}

pub(super) fn range_projective_parameter_image(
    parameter: &BezierParameter2,
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let Some((numerator, denominator)) = range_projective_parameter_coefficients(
        first_range,
        second_range,
        second_to_first_scale,
        reversed,
        first_to_second,
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let target_range = if first_to_second {
        second_range
    } else {
        first_range
    };
    projective_parameter_image(
        parameter,
        &numerator,
        &denominator,
        &CurveParameterRange2::from_bezier_range(target_range.clone()),
        policy,
    )
}

pub(super) fn range_projective_parameter_coefficients(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
) -> Option<([Real; 2], [Real; 2])> {
    let (Some(first_start), Some(first_end)) =
        (first_range.start().scalar(), first_range.end().scalar())
    else {
        return None;
    };
    let (second_start, second_end) = if reversed {
        (second_range.end().scalar(), second_range.start().scalar())
    } else {
        (second_range.start().scalar(), second_range.end().scalar())
    };
    let (Some(second_start), Some(second_end)) = (second_start, second_end) else {
        return None;
    };
    let second_span = second_end - second_start;
    let denominator = [
        second_to_first_scale * first_end - first_start,
        Real::one() - second_to_first_scale,
    ];
    let aligned_numerator = if reversed {
        [second_to_first_scale * first_end, -second_to_first_scale]
    } else {
        [-first_start.clone(), Real::one()]
    };
    let numerator = [
        second_start * &denominator[0] + &second_span * &aligned_numerator[0],
        second_start * &denominator[1] + second_span * &aligned_numerator[1],
    ];
    Some(if first_to_second {
        (numerator, denominator)
    } else {
        let inverse_numerator = [numerator[0].clone(), -denominator[0].clone()];
        let inverse_denominator = [-numerator[1].clone(), denominator[1].clone()];
        (inverse_numerator, inverse_denominator)
    })
}

pub(super) fn projective_parameter_image(
    parameter: &BezierParameter2,
    numerator: &[Real; 2],
    denominator: &[Real; 2],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let mapped = match CurveParameter2::from(parameter.clone()).projective_image_unbounded(
        numerator,
        denominator,
        policy,
    )? {
        Classification::Decided(mapped) => mapped,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match CurveParameterDomain2::new(range, None).contains_finite_parameter(&mapped, policy)? {
        Classification::Decided(true) => mapped
            .promoted_bezier_parameter_complete(policy)
            .map(|result| result.map(Some)),
        Classification::Decided(false) => Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn overlap_parameter_on_curve(
    source: &RationalBezier2,
    target: &RationalBezier2,
    source_parameter: &BezierParameter2,
    mut unresolved: Option<UncertaintyReason>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let has_conic_parameter_frame = target.degree() == 2
        || target
            .data
            .lineage
            .root
            .quadratic_conic_parameter_frame
            .get()
            .is_some();
    let target_is_conic = if has_conic_parameter_frame {
        match target.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(_)) => true,
            Classification::Decided(None) => false,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                false
            }
        }
    } else {
        false
    };
    if target_is_conic {
        match conic_parameter_map(target, source, policy)? {
            Classification::Decided(parameter_map) => {
                let root = parameter_root_representation(source_parameter, policy);
                match conic_parameter_candidate(
                    &root.polynomial_coefficients,
                    &parameter_map.primary,
                    policy,
                )? {
                    Classification::Decided(primary) => {
                        return conic_parameter_from_curve_parameter(
                            &parameter_map,
                            &primary,
                            &root.polynomial_coefficients,
                            source_parameter,
                            false,
                            policy,
                        );
                    }
                    Classification::Uncertain(reason) => unresolved = Some(reason),
                }
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }

    for axis in [Axis2::X, Axis2::Y] {
        let graph = match target.polynomial_graph(axis, policy)? {
            Classification::Decided(Some(graph)) => graph,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                continue;
            }
        };
        let basis = source.homogeneous_power_basis()?;
        let coordinate = match axis {
            Axis2::X => &basis.x_numerator,
            Axis2::Y => &basis.y_numerator,
        };
        let numerator = subtract_power_polynomials(
            coordinate,
            &scale_power_polynomial(&basis.weight, &graph.origin),
        );
        let denominator = scale_power_polynomial(&basis.weight, &graph.scale);
        let root = parameter_root_representation(source_parameter, policy);
        let candidate = match conic_parameter_candidate(
            &root.polynomial_coefficients,
            &(numerator, denominator),
            policy,
        )? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                continue;
            }
        };
        return conic_parameter_from_candidates(&[candidate], source_parameter, policy);
    }

    if let Some(parameter) = source_parameter.scalar() {
        match source.point_at_classified(parameter, policy) {
            Classification::Decided(point) => {
                return Ok(unique_point_incidence_parameter(target, &point, policy));
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }
    for axis in [Axis2::X, Axis2::Y] {
        if !target.has_certified_injective_axis_on(axis, policy) {
            continue;
        }
        match overlap_parameter_through_injective_axis(
            source,
            target,
            source_parameter,
            axis,
            policy,
        )? {
            Classification::Decided(parameter) => {
                return Ok(Classification::Decided(parameter));
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }
    Ok(Classification::Uncertain(
        unresolved.unwrap_or(UncertaintyReason::Unsupported),
    ))
}

pub(super) fn overlap_parameter_through_injective_axis(
    source: &RationalBezier2,
    target: &RationalBezier2,
    source_parameter: &BezierParameter2,
    axis: Axis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let source_basis = source.homogeneous_power_basis()?;
    let source_coordinate = match axis {
        Axis2::X => &source_basis.x_numerator,
        Axis2::Y => &source_basis.y_numerator,
    };
    let source_root = parameter_root_representation(source_parameter, policy);
    let coordinate_map = AlgebraicRootRationalMap::new(
        &source_root.polynomial_coefficients,
        source_coordinate,
        &source_basis.weight,
        policy.predicate_policy(),
    );
    let coordinate_image = coordinate_map.transform(&source_root);
    if coordinate_image.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(coordinate_image) = coordinate_image.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };

    let target_basis = target.homogeneous_power_basis()?;
    let target_coordinate = match axis {
        Axis2::X => &target_basis.x_numerator,
        Axis2::Y => &target_basis.y_numerator,
    };
    let Some(preimage_coefficients) = rational_map_preimage_polynomial(
        &coordinate_image.polynomial_coefficients,
        target_coordinate,
        &target_basis.weight,
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let polynomial = match BezierParameterPolynomial::try_new_power_basis_with_policy(
        preimage_coefficients,
        policy,
    ) {
        Ok(Classification::Decided(polynomial)) => polynomial,
        Ok(Classification::Uncertain(reason)) => {
            return Ok(Classification::Uncertain(reason));
        }
        Err(CurveError::InvalidBezierPolynomial) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        Err(error) => return Err(error),
    };
    let parameters = match polynomial.isolate_unit_interval_roots_with_policy(policy)? {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut matched = None;
    let mut unresolved = false;
    for parameter in parameters {
        let Some(replay) = target.candidate_point_replay(&parameter, policy)? else {
            unresolved = true;
            continue;
        };
        let candidate_coordinate = match axis {
            Axis2::X => &replay.x,
            Axis2::Y => &replay.y,
        };
        match algebraic_coordinates_equal(coordinate_image, candidate_coordinate, policy) {
            Some(true) if matched.is_none() => matched = Some(parameter),
            Some(true) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Some(false) => {}
            None => unresolved = true,
        }
    }
    if let Some(parameter) = matched {
        return match parameter.promote_represented_exact_point_with_policy(policy)? {
            Classification::Decided(parameter) => Ok(Classification::Decided(Some(parameter))),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
    }
    Ok(if unresolved {
        Classification::Uncertain(UncertaintyReason::Predicate)
    } else {
        Classification::Decided(None)
    })
}

pub(super) fn rational_map_preimage_polynomial(
    image_polynomial: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let degree = image_polynomial.len().checked_sub(1)?;
    let numerator_powers = power_polynomial_sequence(numerator, degree)?;
    let denominator_powers = power_polynomial_sequence(denominator, degree)?;
    let mut preimage = vec![Real::zero()];
    for (power, coefficient) in image_polynomial.iter().enumerate() {
        let term = multiply_power_polynomials(
            &numerator_powers[power],
            &denominator_powers[degree - power],
        )?;
        add_scaled_power_polynomial(&mut preimage, &term, coefficient);
    }
    Some(preimage)
}

pub(super) struct ConicParameterMap2 {
    pub(super) primary: (Vec<Real>, Vec<Real>),
    pub(super) coordinates: [Vec<Real>; 3],
    pub(super) range_start: Real,
    pub(super) range_span: Real,
}

pub(super) struct ConicParameterCandidate2 {
    pub(super) map: AlgebraicRootRationalMap,
    pub(super) numerator: Vec<Real>,
    pub(super) denominator: Vec<Real>,
    pub(super) image_polynomial: OnceLock<Option<BezierParameterPolynomial>>,
    pub(super) image_parameters: OnceLock<CurveResult<Classification<Vec<BezierParameter2>>>>,
    pub(super) quotient_matrices: OnceLock<Option<QuotientRingRationalMapMatrices>>,
    pub(super) quotient_power: OnceLock<Option<Vec<Real>>>,
}

pub(super) fn conic_parameter_map(
    conic: &RationalBezier2,
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<ConicParameterMap2>> {
    let controls = quadratic_conic_parameter_frame(conic);
    let first = homogeneous_control_vector(&controls[0]);
    let middle = homogeneous_control_vector(&controls[1]);
    let last = homogeneous_control_vector(&controls[2]);
    let lambda_0 = cross3(&middle, &last);
    if is_zero(&dot3(&first, &lambda_0), policy) != Some(false) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let lambda_1 = cross3(&last, &first);
    let lambda_2 = cross3(&first, &middle);
    let basis = curve.homogeneous_power_basis()?;
    let coordinate_0 = homogeneous_linear_form(basis, &lambda_0);
    let coordinate_1 = homogeneous_linear_form(basis, &lambda_1);
    let coordinate_2 = homogeneous_linear_form(basis, &lambda_2);
    let two = Real::from(2_i8);
    let twice_coordinate_2 = scale_power_polynomial(&coordinate_2, &two);
    let coordinate_sum = add_power_polynomials(
        &add_power_polynomials(&coordinate_0, &coordinate_1),
        &coordinate_2,
    );
    let right_numerator = add_power_polynomials(&coordinate_1, &twice_coordinate_2);
    let range = conic.source_parameter_range();
    let range_start = range.start().clone();
    let span = range.end() - range.start();
    let primary = localize_conic_parameter_candidate(
        right_numerator,
        scale_power_polynomial(&coordinate_sum, &two),
        &range_start,
        &span,
    );
    Ok(Classification::Decided(ConicParameterMap2 {
        primary,
        coordinates: [coordinate_0, coordinate_1, coordinate_2],
        range_start,
        range_span: span,
    }))
}

pub(super) fn conic_parameter_from_curve_parameter(
    parameter_map: &ConicParameterMap2,
    primary_candidate: &ConicParameterCandidate2,
    source_polynomial: &[Real],
    curve_parameter: &BezierParameter2,
    prefer_exact_image_polynomial: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if prefer_exact_image_polynomial {
        match real_coefficient_rational_image_parameter(curve_parameter, primary_candidate, policy)?
        {
            Classification::Decided(Some(parameter)) => {
                return Ok(Classification::Decided(Some(parameter)));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
    }
    let primary = conic_parameter_from_candidates(
        std::slice::from_ref(primary_candidate),
        curve_parameter,
        policy,
    )?;
    let primary_absent = match primary {
        Classification::Decided(Some(parameter)) => {
            return Ok(Classification::Decided(Some(parameter)));
        }
        // For an algebraic source, the retained image route reports `None`
        // only after proving the primary image is disjoint from the target
        // interval. Every nonsingular conic chart represents the same
        // parameter, so rebuilding the two fallback charts cannot recover an
        // in-range value. Exact-source evaluation also uses `None` for a
        // chart pole and must retain the fallback search below.
        Classification::Decided(None) if curve_parameter.scalar().is_none() => {
            return Ok(Classification::Decided(None));
        }
        Classification::Decided(None) => true,
        Classification::Uncertain(_) => false,
    };

    let [coordinate_0, coordinate_1, coordinate_2] = &parameter_map.coordinates;
    let two = Real::from(2_i8);
    let twice_coordinate_0 = scale_power_polynomial(coordinate_0, &two);
    let twice_coordinate_2 = scale_power_polynomial(coordinate_2, &two);
    let fallback_candidate_polynomials = [
        localize_conic_parameter_candidate(
            coordinate_1.clone(),
            add_power_polynomials(&twice_coordinate_0, coordinate_1),
            &parameter_map.range_start,
            &parameter_map.range_span,
        ),
        localize_conic_parameter_candidate(
            twice_coordinate_2.clone(),
            add_power_polynomials(coordinate_1, &twice_coordinate_2),
            &parameter_map.range_start,
            &parameter_map.range_span,
        ),
    ];
    let mut fallback_candidates = Vec::with_capacity(fallback_candidate_polynomials.len());
    for candidate in &fallback_candidate_polynomials {
        match conic_parameter_candidate(source_polynomial, candidate, policy)? {
            Classification::Decided(candidate) => fallback_candidates.push(candidate),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    match conic_parameter_from_candidates(&fallback_candidates, curve_parameter, policy)? {
        Classification::Decided(Some(parameter)) => Ok(Classification::Decided(Some(parameter))),
        Classification::Decided(None) => Ok(Classification::Decided(None)),
        Classification::Uncertain(_) if primary_absent => Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn localize_conic_parameter_candidate(
    numerator: Vec<Real>,
    denominator: Vec<Real>,
    range_start: &Real,
    range_span: &Real,
) -> (Vec<Real>, Vec<Real>) {
    (
        subtract_power_polynomials(
            &numerator,
            &scale_power_polynomial(&denominator, range_start),
        ),
        scale_power_polynomial(&denominator, range_span),
    )
}

pub(super) fn conic_parameter_from_candidates(
    candidates: &[ConicParameterCandidate2],
    curve_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    // A conic parameter is retained construction evidence. Exhaust every
    // exact map and image-root path under STRICT; APPROXIMATE_512 may still
    // resolve later equality predicates, but it cannot select this scalar.
    let strict = policy.strict_counterpart();
    if curve_parameter.scalar().is_some() {
        // An implicit conic can meet the other curve's projective extension at
        // a parameter where one rational chart has a zero denominator. Try
        // every chart and treat a chart's exact pole as absence, not global
        // predicate uncertainty.
        let mut uncertain = None;
        for candidate in candidates {
            match real_coefficient_rational_image_parameter(curve_parameter, candidate, &strict)? {
                Classification::Decided(Some(parameter)) => {
                    return Ok(Classification::Decided(Some(parameter)));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => uncertain = Some(reason),
            }
        }
        return Ok(uncertain.map_or(Classification::Decided(None), Classification::Uncertain));
    }

    let refinement_steps = [2, 4, 8, 16, 32, 64, 128];
    let mut certified_absent = vec![false; candidates.len()];
    let mut refinement = BezierParameterRefinement2::new(curve_parameter, &strict);
    for max_refinement_steps in refinement_steps {
        let refined_curve_parameter = refinement.refine_to(max_refinement_steps);
        let root = parameter_root_representation(refined_curve_parameter, &strict);
        for (candidate_index, candidate) in candidates.iter().enumerate() {
            if certified_absent[candidate_index] {
                continue;
            }
            match rational_image_parameter(&root, candidate, &strict)? {
                Classification::Decided(Some(parameter)) => {
                    return Ok(Classification::Decided(Some(parameter)));
                }
                Classification::Decided(None) => certified_absent[candidate_index] = true,
                Classification::Uncertain(_) => {}
            }
        }
        if certified_absent.iter().all(|absent| *absent) {
            return Ok(Classification::Decided(None));
        }
    }

    // The direct retained-root transform above is only an allocation-saving
    // schedule. Each unresolved chart now enters the one complete global
    // image authority exactly once; that authority owns all further source
    // refinement and correlated image-root selection.
    let mut uncertainty = None;
    for (candidate_index, candidate) in candidates.iter().enumerate() {
        if certified_absent[candidate_index] {
            continue;
        }
        match real_coefficient_rational_image_parameter(curve_parameter, candidate, &strict)? {
            Classification::Decided(Some(parameter)) => {
                return Ok(Classification::Decided(Some(parameter)));
            }
            Classification::Decided(None) => certified_absent[candidate_index] = true,
            Classification::Uncertain(reason) => uncertainty = Some(reason),
        }
    }
    Ok(if certified_absent.iter().all(|absent| *absent) {
        Classification::Decided(None)
    } else {
        Classification::Uncertain(uncertainty.unwrap_or(UncertaintyReason::Predicate))
    })
}

pub(super) fn rational_map_image_polynomial(
    source_polynomial: &[Real],
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> Option<BezierParameterPolynomial> {
    if let Some(coefficients) = quotient_ring_rational_map_image_polynomial(
        source_polynomial,
        numerator,
        denominator,
        policy,
    ) && let Ok(Classification::Decided(polynomial)) =
        BezierParameterPolynomial::try_new_power_basis_with_policy(coefficients, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "quotient-ring-resultant",
        );
        return Some(polynomial);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "conic-rational-image-fallback",
        "sampled-bareiss-resultant",
    );
    let source_degree = source_polynomial.len().checked_sub(1)?;
    let mut samples = Vec::with_capacity(source_degree + 1);
    for sample in 0..=source_degree {
        let value = Real::from(i64::try_from(sample).ok()?);
        let relation =
            subtract_power_polynomials(numerator, &scale_power_polynomial(denominator, &value));
        let resultant = resultant_univariate_polynomials(
            source_polynomial,
            &relation,
            RATIONAL_INTERSECTION_RESULTANT_PRECISION,
        )
        .ok()?
        .resultant;
        samples.push(resultant);
    }
    let coefficients = hypersolve::curve_resultant::interpolate_integer_node_samples(&samples)?;
    match BezierParameterPolynomial::try_new_power_basis_with_policy(coefficients, policy).ok()? {
        Classification::Decided(polynomial) => Some(polynomial),
        Classification::Uncertain(_) => None,
    }
}

pub(super) fn quotient_ring_rational_map_image_coefficients(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let matrices = quotient_ring_rational_map_matrices(source, numerator, denominator)?;
    determinant_linear_power_polynomial(&matrices.numerator, &matrices.denominator, matrices.degree)
}

pub(super) fn quotient_ring_rational_map_image_polynomial(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> Option<Vec<Real>> {
    match trim_power_polynomial(
        quotient_ring_rational_map_image_coefficients(source, numerator, denominator)?,
        policy,
    ) {
        Classification::Decided(polynomial) => Some(polynomial),
        Classification::Uncertain(_) => None,
    }
}

pub(super) fn determinant_linear_power_polynomial(
    constants: &[Real],
    negative_linear_coefficients: &[Real],
    degree: usize,
) -> Option<Vec<Real>> {
    let matrix_entries = degree.checked_mul(degree)?;
    if constants.len() != matrix_entries || negative_linear_coefficients.len() != matrix_entries {
        return None;
    }
    // The determinant of multiplication by n(x) - y*d(x) in R[x]/(source)
    // is its exact norm, hence the required resultant up to one nonzero scale.
    // Subset expansion visits each partial column set once and keeps the matrix
    // entries as linear polynomials in y.
    let state_count = 1_usize.checked_shl(u32::try_from(degree).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![Real::one()]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == degree {
            continue;
        }
        let Some(partial) = partials[mask].take() else {
            continue;
        };
        for column in 0..degree {
            let column_bit = 1_usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry_index = row * degree + column;
            let negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            let next = partials[mask | column_bit]
                .get_or_insert_with(|| vec![Real::zero(); partial.len() + 1]);
            for (power, coefficient) in partial.iter().enumerate() {
                let constant = coefficient * &constants[entry_index];
                let linear = coefficient * &negative_linear_coefficients[entry_index];
                if negative {
                    next[power] -= constant;
                    next[power + 1] += linear;
                } else {
                    next[power] += constant;
                    next[power + 1] -= linear;
                }
            }
        }
    }
    partials.pop()?
}

pub(super) fn locally_certified_rational_image_parameter(
    source_parameter: &BezierParameter2,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Option<Classification<Option<BezierParameter2>>>> {
    let Some(matrices) = candidate
        .quotient_matrices
        .get_or_init(|| {
            let BezierParameter2::Algebraic(source) = source_parameter else {
                return None;
            };
            quotient_ring_rational_map_matrices(
                source.polynomial().coefficients(),
                &candidate.numerator,
                &candidate.denominator,
            )
        })
        .as_ref()
    else {
        return Ok(None);
    };
    // Interval subset expansion is exponential in the quotient degree and
    // repeatedly reduces widening dyadic rationals. Above degree six the
    // shared exact determinant/image-root authority is both smaller-work and
    // complete, so do not attempt this local accelerator first.
    if matrices.degree > 6 {
        return Ok(None);
    }
    let mut refinement = BezierParameterRefinement2::new(source_parameter, policy);
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256] {
        let refined = refinement.refine_to(refinement_steps);
        let source_interval = match refined.known_interval_with_policy(policy)? {
            Classification::Decided(interval) => RealInterval {
                lower: interval.start().clone(),
                upper: interval.end().clone(),
            },
            Classification::Uncertain(_) => continue,
        };
        let Some(image_interval) = evaluate_rational_map_interval(
            &candidate.numerator,
            &candidate.denominator,
            &source_interval,
        ) else {
            continue;
        };
        if compare_reals(&image_interval.upper, &Real::zero(), policy) == Some(Ordering::Less)
            || compare_reals(&image_interval.lower, &Real::one(), policy) == Some(Ordering::Greater)
        {
            return Ok(Some(Classification::Decided(None)));
        }
        if !matches!(
            compare_reals(&image_interval.lower, &Real::zero(), policy),
            Some(Ordering::Greater | Ordering::Equal)
        ) || !matches!(
            compare_reals(&image_interval.upper, &Real::one(), policy),
            Some(Ordering::Less | Ordering::Equal)
        ) || compare_reals(&image_interval.lower, &image_interval.upper, policy)
            != Some(Ordering::Less)
        {
            continue;
        }

        let enclosure_precision = match refinement_steps {
            0..=4 => -4,
            5..=8 => -6,
            9..=16 => -8,
            17..=32 => -12,
            33..=64 => -16,
            65..=128 => -24,
            _ => -32,
        };
        let Some(lower_enclosure) = image_interval
            .lower
            .certified_rational_interval(enclosure_precision)
        else {
            continue;
        };
        let Some(upper_enclosure) = image_interval
            .upper
            .certified_rational_interval(enclosure_precision)
        else {
            continue;
        };
        let lower = if lower_enclosure[0] < HyperRational::zero() {
            HyperRational::zero()
        } else {
            lower_enclosure[0].clone()
        };
        let upper = if upper_enclosure[1] > HyperRational::one() {
            HyperRational::one()
        } else {
            upper_enclosure[1].clone()
        };
        if lower >= upper {
            continue;
        }

        // Localize `det(M_N - u M_D)` to the rational target enclosure and
        // propagate certified dyadic coefficient intervals through the small
        // determinant. This avoids first materializing a combinatorial `Real`
        // expression merely to ask for the signs of its Bernstein controls.
        let Some((signs, _leading_power_sign)) = [-16, -32, -64, -128, -256, -512]
            .into_iter()
            .find_map(|precision| {
                determinant_local_bernstein_signs_from_enclosures(
                    matrices, &lower, &upper, precision,
                )
            })
        else {
            continue;
        };
        let first_sign = signs[0];
        let last_sign = signs[signs.len() - 1];
        if first_sign == RealSign::Zero || last_sign == RealSign::Zero {
            continue;
        }
        let mut previous = None;
        let mut variations = 0_usize;
        for sign in signs {
            if sign == RealSign::Zero {
                continue;
            }
            if previous.is_some_and(|previous| previous != sign) {
                variations += 1;
            }
            previous = Some(sign);
        }
        if variations != 1 {
            continue;
        }

        let interval = match BezierParameterInterval::try_new_with_policy(
            Real::new(lower),
            Real::new(upper),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(_) => continue,
        };
        let Some(global_power) = candidate
            .quotient_power
            .get_or_init(|| {
                determinant_linear_power_polynomial(
                    &matrices.numerator,
                    &matrices.denominator,
                    matrices.degree,
                )
            })
            .as_ref()
        else {
            continue;
        };
        let Some(parameter) = BezierAlgebraicParameter2::from_certified_simple_power_basis(
            global_power.clone(),
            interval,
        ) else {
            continue;
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "local-bernstein-resultant",
        );
        return Ok(Some(Classification::Decided(Some(
            BezierParameter2::Algebraic(parameter),
        ))));
    }
    Ok(None)
}

pub(super) fn real_coefficient_rational_image_parameter(
    source_parameter: &BezierParameter2,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let strict = policy.strict_counterpart();
    if let Some(source) = source_parameter.scalar() {
        let numerator = Real::eval_poly(&candidate.numerator, source);
        let denominator = Real::eval_poly(&candidate.denominator, source);
        match is_zero(&denominator, &strict) {
            Some(true) => return Ok(Classification::Decided(None)),
            Some(false) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let value = (numerator / denominator)?;
        return match in_closed_unit_interval(&value, &strict) {
            Some(true) => Ok(Classification::Decided(Some(BezierParameter2::Exact(
                value,
            )))),
            Some(false) => Ok(Classification::Decided(None)),
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        };
    }

    let BezierParameter2::Algebraic(source_algebraic) = source_parameter else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if let Some(result) =
        locally_certified_rational_image_parameter(source_parameter, candidate, &strict)?
    {
        return Ok(result);
    }
    let Some(image_polynomial) = candidate.image_polynomial.get_or_init(|| {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "construct-image-polynomial",
        );
        rational_map_image_polynomial(
            source_algebraic.polynomial().coefficients(),
            &candidate.numerator,
            &candidate.denominator,
            &strict,
        )
    }) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let image_parameters = match candidate
        .image_parameters
        .get_or_init(|| image_polynomial.isolate_unit_interval_roots_with_policy(&strict))
    {
        Ok(Classification::Decided(parameters)) => parameters,
        Ok(Classification::Uncertain(reason)) => {
            return Ok(Classification::Uncertain(*reason));
        }
        Err(error) => return Err(error.clone()),
    };
    if image_parameters.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let mut refinement = BezierParameterRefinement2::new(source_parameter, &strict);
    let mut image_refinements = image_parameters
        .iter()
        .map(|parameter| BezierParameterRefinement2::new(parameter, &strict))
        .collect::<Vec<_>>();
    let mut refinement_steps = 0_usize;
    let mut denominator_sign = None;
    let mut excluded_endpoints = [false; 2];
    loop {
        let refined = refinement.refine_to(refinement_steps);
        let source_interval = match refined.known_interval_with_policy(&strict)? {
            Classification::Decided(interval) => RealInterval {
                lower: interval.start().clone(),
                upper: interval.end().clone(),
            },
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let image_interval = evaluate_rational_map_interval(
            &candidate.numerator,
            &candidate.denominator,
            &source_interval,
        );
        if let Some(image_interval) = image_interval {
            if compare_reals(&image_interval.upper, &Real::zero(), &strict) == Some(Ordering::Less)
                || compare_reals(&image_interval.lower, &Real::one(), &strict)
                    == Some(Ordering::Greater)
            {
                return Ok(Classification::Decided(None));
            }

            let inside = [
                matches!(
                    compare_reals(&image_interval.lower, &Real::zero(), &strict),
                    Some(Ordering::Greater | Ordering::Equal)
                ),
                matches!(
                    compare_reals(&image_interval.upper, &Real::one(), &strict),
                    Some(Ordering::Less | Ordering::Equal)
                ),
            ];
            // This root inventory is complete on the unit interval. Once the
            // image enclosure lies there, a single possible owner identifies
            // the image. A scalar witness must not hide another overlapping
            // algebraic root merely because its carrier has a wider interval.
            if inside == [true; 2] {
                let mut possible = image_refinements.iter_mut().filter_map(|refinement| {
                    let parameter = refinement.refine_to(0);
                    if !image_parameter_may_meet_map_interval(parameter, &image_interval, &strict) {
                        return None;
                    }
                    // A deflated isolator may still cover another image root.
                    // Refine possible competitors as well as the source; only
                    // narrowing the source cannot separate such retained boxes.
                    let steps = if parameter.scalar().is_some() {
                        0
                    } else {
                        refinement_steps.saturating_sub(64)
                    };
                    let parameter = refinement.refine_to(steps);
                    image_parameter_may_meet_map_interval(parameter, &image_interval, &strict)
                        .then_some(parameter)
                });
                if let Some(parameter) = possible.next()
                    && possible.next().is_none()
                {
                    return Ok(Classification::Decided(Some(parameter.clone())));
                }
            } else if refinement_steps >= 64 {
                // An exact endpoint can straddle the unit boundary forever.
                // The map enclosure already proves a nonzero denominator;
                // replay N=0 or N-D=0 at the retained source to own that point.
                for endpoint in 0..2 {
                    if inside[endpoint] || excluded_endpoints[endpoint] {
                        continue;
                    }
                    let coefficients = if endpoint == 0 {
                        candidate.numerator.clone()
                    } else {
                        subtract_power_polynomials(&candidate.numerator, &candidate.denominator)
                    };
                    match signed_coefficients_at_parameter(
                        &coefficients,
                        source_parameter,
                        &strict,
                    )? {
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Decided(Some(BezierParameter2::Exact(
                                Real::from(endpoint as i8),
                            ))));
                        }
                        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                            excluded_endpoints[endpoint] = true
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
            }
        }
        let next = next_rational_image_refinement(refinement_steps)?;
        if next > 64 {
            if denominator_sign.is_none() {
                denominator_sign = Some(signed_coefficients_at_parameter(
                    &candidate.denominator,
                    source_parameter,
                    &strict,
                )?);
            }
            match denominator_sign
                .as_ref()
                .expect("the denominator sign was retained")
            {
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
            }
            #[cfg(feature = "dispatch-trace")]
            if refinement_steps <= 64 {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "conic-rational-image-separation",
                    "unbounded-cold-continuation",
                );
            }
        }
        refinement_steps = next;
    }
}

pub(super) fn image_parameter_may_meet_map_interval(
    parameter: &BezierParameter2,
    image_interval: &RealInterval,
    policy: &CurveContext,
) -> bool {
    // Both a retained isolator and a learned scalar enclose the same root.
    // Only certified disjointness excludes a candidate; an unavailable
    // comparison must remain possible while the source enclosure refines.
    let Ok(Classification::Decided(interval)) = parameter.known_interval_with_policy(policy) else {
        return true;
    };
    !matches!(
        compare_reals(interval.end(), &image_interval.lower, policy),
        Some(Ordering::Less)
    ) && !matches!(
        compare_reals(&image_interval.upper, interval.start(), policy),
        Some(Ordering::Less)
    )
}

pub(super) fn next_rational_image_refinement(current: usize) -> CurveResult<usize> {
    if current == 0 {
        Ok(2)
    } else {
        current.checked_mul(2).ok_or_else(|| {
            CurveError::Topology("conic rational-image refinement depth overflow".into())
        })
    }
}

/// Encloses a rational map `numerator / denominator` over a parameter
/// interval. Enclosures use only certified STRICT order decisions; the
/// consuming predicate applies the caller's policy.
pub(super) fn evaluate_rational_map_interval(
    numerator: &[Real],
    denominator: &[Real],
    parameter: &RealInterval,
) -> Option<RealInterval> {
    RealInterval::evaluate_power_basis(numerator, parameter)?
        .divide(&RealInterval::evaluate_power_basis(denominator, parameter)?)
}

/// Retains the exact affine contact point or its construction blocker.
/// An undefined point must not be confused with absent contact evidence.
pub(crate) fn exact_contact_point_evidence(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => Ok(curve
            .point_at_affine_classified(parameter, policy)
            .map(CurvePoint2::from)),
        BezierParameter2::Algebraic(parameter) => Ok(curve
            .point_at_algebraic_parameter(parameter, policy)?
            .map(CurvePoint2::from)),
    }
}

pub(crate) fn rational_parameter_image_matches(
    source: &BezierParameter2,
    target: &BezierParameter2,
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    match source {
        BezierParameter2::Exact(source) => {
            let denominator = Real::eval_poly(denominator, source);
            match real_sign(&denominator, policy) {
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Some(RealSign::Positive | RealSign::Negative) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let image =
                Real::eval_poly(numerator, source) * denominator.inverse_ref_assuming_nonzero()?;
            BezierParameter2::Exact(image).same_value(target, policy)
        }
        BezierParameter2::Algebraic(source) => {
            let candidate = match conic_parameter_candidate(
                source.polynomial().coefficients(),
                &(numerator.to_vec(), denominator.to_vec()),
                policy,
            )? {
                Classification::Decided(candidate) => candidate,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            {
                let target_interval = match target.known_interval_with_policy(policy)? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let evidence = candidate.map.transform_in_interval(
                    &parameter_representation(source, policy),
                    &AlgebraicPolynomialValueInterval {
                        lower: target_interval.start().clone(),
                        upper: target_interval.end().clone(),
                    },
                );
                if evidence.status == AlgebraicRootRationalImageStatus::ImageIntervalDisjoint {
                    return Ok(Classification::Decided(false));
                }
                if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                let Some(image) = evidence.representation.as_ref() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                };
                Ok(algebraic_coordinates_equal(
                    image,
                    &parameter_root_representation(target, policy),
                    policy,
                )
                .map_or(
                    Classification::Uncertain(UncertaintyReason::Predicate),
                    Classification::Decided,
                ))
            }
        }
    }
}

/// Operation-scoped exact rational map with policy-isolated algebraic proof caches.
pub(crate) struct RationalParameterImageMap2 {
    pub(super) coefficients: (Vec<Real>, Vec<Real>),
    pub(super) candidates: Vec<(Vec<Real>, ConicParameterCandidate2)>,
    pub(super) policy: CurveContext,
}

impl RationalParameterImageMap2 {
    pub(crate) fn new(numerator: Vec<Real>, denominator: Vec<Real>, policy: &CurveContext) -> Self {
        Self {
            coefficients: (numerator, denominator),
            candidates: Vec::new(),
            policy: *policy,
        }
    }

    pub(crate) fn image(
        &mut self,
        source: &BezierParameter2,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let strict_policy = self.policy.strict_counterpart();
        if let Some(source) = source.scalar() {
            let strict = exact_rational_parameter_image(
                source,
                &self.coefficients.0,
                &self.coefficients.1,
                true,
                &strict_policy,
            )?;
            if strict.is_decided() || !self.policy.permits_approximate_512() {
                return Ok(strict);
            }
            return exact_rational_parameter_image(
                source,
                &self.coefficients.0,
                &self.coefficients.1,
                true,
                &self.policy,
            );
        }
        let BezierParameter2::Algebraic(source) = source else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let source_polynomial = source.polynomial().coefficients();
        let candidate_index = if let Some(index) = self
            .candidates
            .iter()
            .position(|(polynomial, _)| polynomial == source_polynomial)
        {
            index
        } else {
            let candidate = match conic_parameter_candidate(
                source_polynomial,
                &self.coefficients,
                &strict_policy,
            )? {
                Classification::Decided(candidate) => candidate,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            self.candidates
                .push((source_polynomial.to_vec(), candidate));
            self.candidates.len() - 1
        };
        real_coefficient_rational_image_parameter(
            &BezierParameter2::Algebraic(source.clone()),
            &self.candidates[candidate_index].1,
            &strict_policy,
        )
    }

    /// Maps to any finite affine parameter. The caller owns the projective
    /// cell and placement checks; unlike [`Self::image`], this does not clip
    /// the image to the authored unit segment.
    pub(crate) fn image_unbounded(
        &self,
        source: &BezierParameter2,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let strict_policy = self.policy.strict_counterpart();
        let strict = rational_parameter_image_unbounded(
            source,
            &self.coefficients.0,
            &self.coefficients.1,
            &strict_policy,
        )?;
        if strict.is_decided() || !self.policy.permits_approximate_512() {
            return Ok(strict);
        }
        rational_parameter_image_unbounded(
            source,
            &self.coefficients.0,
            &self.coefficients.1,
            &self.policy,
        )
    }
}

pub(super) fn exact_rational_parameter_image(
    source: &Real,
    numerator: &[Real],
    denominator: &[Real],
    unit_domain: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let denominator_value = Real::eval_poly(denominator, source);
    match real_sign(&denominator_value, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let value =
        Real::eval_poly(numerator, source) * denominator_value.inverse_ref_assuming_nonzero()?;
    if !unit_domain {
        return Ok(Classification::Decided(Some(BezierParameter2::Exact(
            value,
        ))));
    }
    match in_closed_unit_interval(&value, policy) {
        Some(true) => Ok(Classification::Decided(Some(BezierParameter2::Exact(
            value,
        )))),
        Some(false) => Ok(Classification::Decided(None)),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

pub(super) fn rational_parameter_image_unbounded(
    source: &BezierParameter2,
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let BezierParameter2::Algebraic(source) = source else {
        return exact_rational_parameter_image(
            source
                .scalar()
                .expect("a non-algebraic Bezier parameter is exact"),
            numerator,
            denominator,
            false,
            policy,
        );
    };
    let map = AlgebraicRootRationalMap::new(
        source.polynomial().coefficients(),
        numerator,
        denominator,
        policy.predicate_policy(),
    );
    let evidence = map.transform(&parameter_representation(source, policy));
    if evidence.status == AlgebraicRootRationalImageStatus::CertifiedZeroDenominator {
        return Ok(Classification::Decided(None));
    }
    if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(representation) = evidence.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    BezierParameter2::from_algebraic_root_representation_unbounded(representation, policy)
        .map(|parameter| parameter.map(Some))
}

pub(super) fn conic_parameter_candidate(
    source_polynomial: &[Real],
    candidate: &(Vec<Real>, Vec<Real>),
    policy: &CurveContext,
) -> CurveResult<Classification<ConicParameterCandidate2>> {
    let strict = policy.strict_counterpart();
    let mut numerator = match trim_power_polynomial(candidate.0.clone(), &strict) {
        Classification::Decided(numerator) => numerator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut denominator = match trim_power_polynomial(candidate.1.clone(), &strict) {
        Classification::Decided(denominator) => denominator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if numerator
        .iter()
        .chain(&denominator)
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
        && let Some(scale) = numerator
            .iter()
            .chain(&denominator)
            .find(|coefficient| is_zero(coefficient, &strict) == Some(false))
            .cloned()
    {
        let normalized_numerator = numerator
            .iter()
            .map(|coefficient| coefficient.clone() / scale.clone())
            .collect::<Result<Vec<_>, _>>()?;
        let normalized_denominator = denominator
            .iter()
            .map(|coefficient| coefficient.clone() / scale.clone())
            .collect::<Result<Vec<_>, _>>()?;
        if normalized_numerator
            .iter()
            .chain(&normalized_denominator)
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
        {
            numerator = normalized_numerator;
            denominator = normalized_denominator;
        }
    }
    {
        Ok(Classification::Decided(ConicParameterCandidate2 {
            map: AlgebraicRootRationalMap::new(
                source_polynomial,
                &numerator,
                &denominator,
                strict.predicate_policy(),
            ),
            numerator,
            denominator,
            image_polynomial: OnceLock::new(),
            image_parameters: OnceLock::new(),
            quotient_matrices: OnceLock::new(),
            quotient_power: OnceLock::new(),
        }))
    }
}

pub(super) fn rational_image_parameter(
    source: &AlgebraicRootRepresentation,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let zero = Real::zero();
    let one = Real::one();
    let evidence = candidate.map.transform_in_interval(
        source,
        &AlgebraicPolynomialValueInterval {
            lower: zero.clone(),
            upper: one.clone(),
        },
    );
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "conic-rational-image",
        match evidence.status {
            AlgebraicRootRationalImageStatus::Transformed => "transformed",
            AlgebraicRootRationalImageStatus::ImageIntervalDisjoint => "interval-disjoint",
            AlgebraicRootRationalImageStatus::InvalidEvidence => "invalid-evidence",
            AlgebraicRootRationalImageStatus::InvalidNumeratorPolynomial => "invalid-numerator",
            AlgebraicRootRationalImageStatus::InvalidDenominatorPolynomial => "invalid-denominator",
            AlgebraicRootRationalImageStatus::CertifiedZeroDenominator => "zero-denominator",
            AlgebraicRootRationalImageStatus::DenominatorMayContainZero => {
                "denominator-may-contain-zero"
            }
            AlgebraicRootRationalImageStatus::NumeratorImageFailed => "numerator-image-failed",
            AlgebraicRootRationalImageStatus::DenominatorImageFailed => "denominator-image-failed",
            AlgebraicRootRationalImageStatus::QuotientConstructionFailed => {
                "quotient-construction-failed"
            }
            AlgebraicRootRationalImageStatus::InvalidTransformedEvidence => {
                "invalid-transformed-evidence"
            }
            AlgebraicRootRationalImageStatus::Undecided => "undecided",
        },
    );
    if evidence.status == AlgebraicRootRationalImageStatus::ImageIntervalDisjoint {
        return Ok(Classification::Decided(None));
    }
    if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(representation) = evidence.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let lower_zero = compare_reals(&representation.interval.lower, &zero, policy);
    let upper_zero = compare_reals(&representation.interval.upper, &zero, policy);
    let lower_one = compare_reals(&representation.interval.lower, &one, policy);
    let upper_one = compare_reals(&representation.interval.upper, &one, policy);
    let (Some(lower_zero), Some(upper_zero), Some(lower_one), Some(upper_one)) =
        (lower_zero, upper_zero, lower_one, upper_one)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    };
    if upper_zero == Ordering::Less || lower_one == Ordering::Greater {
        return Ok(Classification::Decided(None));
    }
    if lower_zero == Ordering::Less || upper_one == Ordering::Greater {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    match BezierParameter2::from_algebraic_root_representation(representation, policy) {
        Ok(Classification::Decided(parameter)) => Ok(Classification::Decided(Some(parameter))),
        Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(CurveError::InvalidBezierParameter) => {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
        Err(error) => Err(error),
    }
}
