//! Exact parameter correspondence at algebraic cusp-semicircle overlaps.
//!
//! Cusp endpoints map one carrier's parameter onto rational, parallel and
//! selected-fiber parameters of another; complementary-cut and diameter
//! relations decide which mapped parameter a shared overlap retains.

use super::*;

pub(super) fn parallel_overlap_parameter_for_exact_cusp(
    map: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    parameter: &Real,
    range: &CurveParameterRange2,
    map_reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    let map_range = if map_reversed {
        CurveParameterRange2::new_validated(
            range
                .start()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
            range
                .end()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
        )
    } else {
        range.clone()
    };
    let map = &map.data;
    let Some((cusp_parameter, _, diameter, radius_squared_denominator, speed_squared)) =
        map.one_field_system()
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let cusp_map_parameter = cusp_parameter;
    let BezierParameter2::Algebraic(cusp_parameter) = cusp_map_parameter else {
        return Err(CurveError::Topology(
            "cusp/parallel map lost its retained cusp root".into(),
        ));
    };
    let one_minus = Real::one() - parameter;
    let denominator = &one_minus * &one_minus + parameter * parameter;
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Err(CurveError::Topology(
                "semicircle inverse-map denominator was not positive".into(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
    let predicate = BezierAlgebraicCuspTwoTermExpression2 {
        rational: bivariate_scaled_difference(
            &diameter.rational,
            &denominator,
            radius_squared_denominator,
            &radial_coefficient,
        ),
        radical: bivariate_scale(diameter.radical.clone(), &denominator),
    };
    let incidence = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&predicate.rational, &predicate.rational),
            speed_squared,
        ),
        &bivariate_multiply(&predicate.radical, &predicate.radical),
    );
    let incidence = match reduce_algebraic_cusp_bivariate(incidence, cusp_parameter, policy)? {
        Classification::Decided(incidence) => incidence,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let cusp_root = parameter_representation(cusp_parameter, policy);
    let quotient = algebraic_selected_quotient_ring_fiber_projection_with_max_degree(
        &incidence,
        &cusp_root,
        MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
        &map_range,
        policy,
    )?;
    // The quotient norm includes candidates contributed by conjugate cusp
    // roots. The unsquared replay below evaluates every candidate at the
    // selected cusp root, so it rejects both those foreign candidates and
    // the opposite radical branch in one exact predicate. Running the
    // general selected-fiber membership pass first would duplicate that
    // proof and can dominate reversal-heavy clipping.
    let candidates = match quotient {
        Classification::Decided(ResultantParameterProjection::Empty) => Vec::new(),
        Classification::Decided(
            ResultantParameterProjection::Parameters(candidates)
            | ResultantParameterProjection::SelectedParameters(candidates),
        ) => candidates,
        Classification::Decided(ResultantParameterProjection::Degenerate)
        | Classification::Uncertain(_) => {
            match selected_fiber_parameters(
                &incidence,
                &BezierParameter2::Algebraic(cusp_parameter.clone()),
                &map_range,
                policy,
            )? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero
                    | BezierAlgebraicFiberProjection2::Degenerate,
                ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    };
    retain_unique_overlap_parameter(
        curve_region_parameters_from_bezier(candidates),
        range,
        map_reversed,
        true,
        policy,
        |map_parameter| {
            let map_parameter = map_parameter.as_bezier_parameter().ok_or_else(|| {
                CurveError::Topology("a parallel inverse produced a non-Bezier candidate".into())
            })?;
            algebraic_cusp_correlated_radical_sum_sign(
                &incidence,
                &predicate,
                speed_squared,
                cusp_map_parameter,
                map_parameter,
                policy,
            )
        },
    )
}

pub(super) fn promote_curve_region_bezier_parameter(
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParameter2>> {
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return Ok(Classification::Decided(parameter.clone()));
    }
    policy.strict_predicate_pass(|| parameter.promoted_bezier_parameter_complete(policy))
}

pub(super) fn rational_mapped_cusp_scalar_value(
    map: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    contact: &BezierAlgebraicCuspSemicircleRationalMapContact2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
        cusp_parameter,
        diameter,
        radius_squared_denominator,
        ..
    } = &map.data.system
    else {
        // Projecting this pair-radial angular value would require eliminating
        // both selected source roots. Keep its exact mapped representation
        // until a caller needs that scalar projection.
        return Ok(Classification::Decided(None));
    };
    let other_parameter =
        match promote_curve_region_bezier_parameter(&contact.other_parameter, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if let Some(value) = map
        .data
        .parameter_cache
        .cached_scalar_value(&other_parameter, policy)
    {
        return Ok(Classification::Decided(value));
    }
    let represented_other_parameter = match other_parameter
        .clone()
        .promote_represented_exact_point_with_policy(policy)?
    {
        Classification::Decided(BezierParameter2::Exact(parameter)) => parameter,
        Classification::Decided(BezierParameter2::Algebraic(_)) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radius_squared_denominator =
        bivariate_specialize_second(radius_squared_denominator, &represented_other_parameter);
    let denominator = [Real::one(), Real::from(-2_i8), Real::from(2_i8)];
    let radial = [Real::one(), Real::from(-2_i8)];
    let incidence = match diameter {
        BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(diameter) => {
            let diameter = bivariate_specialize_second(diameter, &represented_other_parameter);
            bivariate_subtract(
                &bivariate_tensor_product(&diameter, &denominator),
                &bivariate_tensor_product(&radius_squared_denominator, &radial),
            )
        }
        BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
            coordinate,
            speed_squared,
        } => {
            let diameter_rational =
                bivariate_specialize_second(&coordinate.rational, &represented_other_parameter);
            let diameter_radical =
                bivariate_specialize_second(&coordinate.radical, &represented_other_parameter);
            let speed_squared =
                bivariate_specialize_second(speed_squared, &represented_other_parameter);
            let rational = bivariate_subtract(
                &bivariate_tensor_product(&diameter_rational, &denominator),
                &bivariate_tensor_product(&radius_squared_denominator, &radial),
            );
            let radical = bivariate_tensor_product(&diameter_radical, &denominator);
            bivariate_subtract(
                &bivariate_multiply(
                    &bivariate_multiply(&rational, &rational),
                    &bivariate_tensor_product(&speed_squared, &[Real::one()]),
                ),
                &bivariate_multiply(&radical, &radical),
            )
        }
    };
    let result =
        mapped_cusp_scalar_value_from_incidence(incidence, cusp_parameter, policy, |parameter| {
            map.mapped_contact_order_to_real(contact, parameter, policy)
        })?;
    if let Classification::Decided(value) = &result {
        map.data
            .parameter_cache
            .retain_scalar_value(other_parameter, value.clone(), policy);
    }
    Ok(result)
}

pub(super) fn parallel_mapped_cusp_scalar_value(
    map: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    contact: &BezierAlgebraicCuspSemicircleParallelContact2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    if let Some(value) = map
        .data
        .parameter_cache
        .cached_scalar_value(&contact.parallel_parameter, policy)
    {
        return Ok(Classification::Decided(value));
    }
    let Some((cusp_parameter, _, diameter, radius_squared_denominator, speed_squared)) =
        map.data.one_field_system()
    else {
        // A pair-radial map remains an exact procedural scalar. An independent
        // scalar witness would require a second three-axis projection with
        // the compact cusp parameter as its target; retain the exact map
        // until that projection is needed.
        return Ok(Classification::Decided(None));
    };
    let other_parameter = match contact
        .parallel_parameter
        .clone()
        .promote_represented_exact_point_with_policy(policy)?
    {
        Classification::Decided(BezierParameter2::Exact(parameter)) => parameter,
        Classification::Decided(BezierParameter2::Algebraic(_)) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let diameter_rational = bivariate_specialize_second(&diameter.rational, &other_parameter);
    let diameter_radical = bivariate_specialize_second(&diameter.radical, &other_parameter);
    let radius_squared_denominator =
        bivariate_specialize_second(radius_squared_denominator, &other_parameter);
    let speed_squared = bivariate_specialize_second(speed_squared, &other_parameter);
    let denominator = [Real::one(), Real::from(-2_i8), Real::from(2_i8)];
    let rational = bivariate_subtract(
        &bivariate_tensor_product(&diameter_rational, &denominator),
        &bivariate_tensor_product(
            &radius_squared_denominator,
            &[Real::one(), Real::from(-2_i8)],
        ),
    );
    let radical = bivariate_tensor_product(&diameter_radical, &denominator);
    let incidence = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&rational, &rational),
            &bivariate_tensor_product(&speed_squared, &[Real::one()]),
        ),
        &bivariate_multiply(&radical, &radical),
    );
    let result =
        mapped_cusp_scalar_value_from_incidence(incidence, cusp_parameter, policy, |parameter| {
            map.contact_order_to_real(contact, parameter, policy)
        })?;
    if let Classification::Decided(value) = &result {
        map.data.parameter_cache.retain_scalar_value(
            contact.parallel_parameter.clone(),
            value.clone(),
            policy,
        );
    }
    Ok(result)
}

pub(super) fn mapped_cusp_scalar_value_from_incidence(
    incidence: BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    policy: &CurveContext,
    mut order_to_real: impl FnMut(&Real) -> CurveResult<Classification<std::cmp::Ordering>>,
) -> CurveResult<Classification<Option<Real>>> {
    let BezierParameter2::Algebraic(cusp_parameter) = cusp_parameter else {
        return Ok(Classification::Decided(None));
    };
    let incidence = match reduce_algebraic_cusp_bivariate(incidence, cusp_parameter, policy)? {
        Classification::Decided(incidence) => incidence,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidates = match algebraic_selected_reduced_fiber_parameters(
        &incidence,
        cusp_parameter,
        &crate::CurveParameterRange2::unit(),
        policy,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
            candidates
        }
        Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero
            | BezierAlgebraicFiberProjection2::Degenerate,
        ) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut retained = None;
    for candidate in candidates {
        let candidate = match candidate.promote_represented_exact_point_with_policy(policy)? {
            Classification::Decided(BezierParameter2::Exact(candidate)) => candidate,
            Classification::Decided(BezierParameter2::Algebraic(_)) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match order_to_real(&candidate)? {
            Classification::Decided(std::cmp::Ordering::Equal) if retained.is_none() => {
                retained = Some(candidate);
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(CurveError::Topology(
                    "mapped cusp cut had multiple scalar value witnesses".into(),
                ));
            }
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(retained))
}

/// Tests same and complemented native parameters by the original unsquared
/// diameter coordinate. This is a per-cut proof: an isolated equality is
/// sufficient and cannot be mistaken for a global map equivalence.
pub(super) fn rational_parallel_parameter_orientation_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    source_parameter: &BezierParameter2,
    source_is_rational: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CurveOverlapOrientation2>>> {
    if !policy.accepts_retained_policy(rational.data.policy)
        || !policy.accepts_retained_policy(parallel.data.policy)
    {
        return Err(CurveError::Topology(
            "cross-map cusp parameterization comparison used a different predicate policy".into(),
        ));
    }
    let Some((rational_cusp_parameter, _, _, _)) = rational.data.one_field_system() else {
        return Ok(Classification::Decided(None));
    };
    let Some((parallel_cusp_parameter, _, _, _, _)) = parallel.data.one_field_system() else {
        return Ok(Classification::Decided(None));
    };
    match rational_cusp_parameter.same_value(parallel_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let mut uncertain = None;
    for orientation in [
        CurveOverlapOrientation2::Same,
        CurveOverlapOrientation2::Reversed,
    ] {
        let rational_parameter =
            if source_is_rational || orientation == CurveOverlapOrientation2::Same {
                source_parameter.clone()
            } else {
                source_parameter.unit_complement()
            };
        match rational_parallel_parameters_match_at_cut(
            rational,
            parallel,
            &rational_parameter,
            orientation,
            policy,
        )? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(Some(orientation)));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => uncertain = Some(reason),
        }
    }
    Ok(uncertain.map_or(Classification::Decided(None), Classification::Uncertain))
}

pub(super) fn rational_parallel_parameters_match_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    rational_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    rational_parallel_diameter_relation_at_cut(
        rational,
        parallel,
        rational_parameter,
        orientation,
        false,
        policy,
    )
}

pub(super) fn algebraic_cusp_independent_two_radical_sum_is_zero(
    rational: &BivariatePolynomial,
    first_radical: &BivariatePolynomial,
    first_radicand: &BivariatePolynomial,
    second_radical: &BivariatePolynomial,
    second_radicand: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
) -> CurveResult<Classification<bool>> {
    let exact = &CurveContext::STRICT;
    let sign = |polynomial: &BivariatePolynomial| {
        signed_bivariate_at_parameter_pair(polynomial, cusp_parameter, other_parameter, exact)
    };
    let rational_sign = match sign(rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let first_sign = match sign(first_radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let second_sign = match sign(second_radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    for radicand in [first_radicand, second_radicand] {
        match sign(radicand)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let signs = [rational_sign, first_sign, second_sign];
    let mut nonzero = [0_usize; 3];
    let mut nonzero_count = 0_usize;
    for (index, sign) in signs.into_iter().enumerate() {
        if sign != RealSign::Zero {
            nonzero[nonzero_count] = index;
            nonzero_count += 1;
        }
    }
    match nonzero_count {
        0 => return Ok(Classification::Decided(true)),
        1 => return Ok(Classification::Decided(false)),
        2 => {
            let first = nonzero[0];
            let second = nonzero[1];
            if signs[first] == signs[second] {
                return Ok(Classification::Decided(false));
            }
        }
        3 if rational_sign == first_sign && first_sign == second_sign => {
            return Ok(Classification::Decided(false));
        }
        3 => {}
        _ => unreachable!("the radical sum has exactly three terms"),
    }

    // Multiplication by both positive square roots changes
    //
    //     A + B/sqrt(S) + C/sqrt(T)
    //
    // into X+Y+Z, where X=A*sqrt(S*T), Y=B*sqrt(T), and
    // Z=C*sqrt(S). Their signs are the coefficient signs above and their
    // squares are ordinary bivariate polynomials. Exact cancellation of two
    // terms is equality of their squares with opposite signs. With three
    // terms, isolate the uniquely signed term L from the two same-signed
    // terms M,N and certify
    //
    //     D=L^2-M^2-N^2 > 0,   D^2=4*M^2*N^2.
    //
    // The sign precondition rejects every conjugate introduced by squaring.
    let square = |index| match index {
        0 => bivariate_multiply(
            &bivariate_multiply(rational, rational),
            &bivariate_multiply(first_radicand, second_radicand),
        ),
        1 => bivariate_multiply(
            &bivariate_multiply(first_radical, first_radical),
            second_radicand,
        ),
        2 => bivariate_multiply(
            &bivariate_multiply(second_radical, second_radical),
            first_radicand,
        ),
        _ => unreachable!("the radical sum has exactly three terms"),
    };
    if nonzero_count == 2 {
        return Ok(
            match sign(&bivariate_subtract(
                &square(nonzero[0]),
                &square(nonzero[1]),
            ))? {
                Classification::Decided(RealSign::Zero) => Classification::Decided(true),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    let squares = [square(0), square(1), square(2)];
    let odd = if rational_sign != first_sign && rational_sign != second_sign {
        0
    } else if first_sign != rational_sign && first_sign != second_sign {
        1
    } else {
        2
    };
    let [first_same, second_same] = match odd {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!(),
    };
    let magnitude_difference = bivariate_subtract(
        &bivariate_subtract(&squares[odd], &squares[first_same]),
        &squares[second_same],
    );
    match sign(&magnitude_difference)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(false));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let conjugate_residual = bivariate_subtract(
        &bivariate_multiply(&magnitude_difference, &magnitude_difference),
        &bivariate_scale(
            bivariate_multiply(&squares[first_same], &squares[second_same]),
            &Real::from(4_i8),
        ),
    );
    Ok(match sign(&conjugate_residual)? {
        Classification::Decided(RealSign::Zero) => Classification::Decided(true),
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Classification::Decided(false)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

pub(super) fn parallel_parameters_are_complementary_at_cut(
    first: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    second: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    shared_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    construction_policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !construction_policy.accepts_retained_policy(first.data.policy)
        || !construction_policy.accepts_retained_policy(second.data.policy)
    {
        return Err(CurveError::Topology(
            "parallel cusp diameter comparison used a different predicate policy".into(),
        ));
    }

    // This relation creates reusable axis evidence, so every equality is
    // proved under STRICT even when the enclosing construction was authored
    // with APPROXIMATE_512. The construction policy above validates the
    // retained maps; it is not permission to turn a terminal equality into a
    // cardinal chord certificate.
    let exact = &CurveContext::STRICT;
    let Some((first_cusp, _, first_diameter, first_radius, first_speed)) =
        first.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    let Some((second_cusp, _, second_diameter, second_radius, second_speed)) =
        second.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    match first_cusp.same_value(second_cusp, exact)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    // Each directed diameter coordinate has the form
    //
    //     (A + B/sqrt(S)) / D,       D > 0.
    //
    // Cross-multiply the positive denominators. Equal speed roots reduce to
    // the cheaper two-term radical signer; otherwise the exact two-radical
    // norm identity below rejects every conjugate introduced by squaring. A
    // zero sum is precisely the unit-complement relation on the selected
    // semicircle.
    let second_speed_squared = bivariate_orient_second_parameter(second_speed, orientation);
    let second_radius_squared_denominator =
        bivariate_orient_second_parameter(second_radius, orientation);
    let second_diameter_rational =
        bivariate_orient_second_parameter(&second_diameter.rational, orientation);
    let second_diameter_radical =
        bivariate_orient_second_parameter(&second_diameter.radical, orientation);
    let common_denominator = first_radius == second_radius_squared_denominator.as_ref();
    let rational = if common_denominator {
        bivariate_add(&first_diameter.rational, second_diameter_rational.as_ref())
    } else {
        bivariate_add(
            &bivariate_multiply(
                &first_diameter.rational,
                second_radius_squared_denominator.as_ref(),
            ),
            &bivariate_multiply(second_diameter_rational.as_ref(), first_radius),
        )
    };
    let first_radical = if common_denominator {
        Cow::Borrowed(&first_diameter.radical)
    } else {
        Cow::Owned(bivariate_multiply(
            &first_diameter.radical,
            second_radius_squared_denominator.as_ref(),
        ))
    };
    let second_radical = if common_denominator {
        Cow::Borrowed(second_diameter_radical.as_ref())
    } else {
        Cow::Owned(bivariate_multiply(
            second_diameter_radical.as_ref(),
            first_radius,
        ))
    };
    let speeds_equal = first_speed == second_speed_squared.as_ref() || {
        let speed_difference = bivariate_subtract(first_speed, second_speed_squared.as_ref());
        matches!(
            signed_bivariate_at_parameter_pair(
                &speed_difference,
                first_cusp,
                shared_parameter,
                exact,
            )?,
            Classification::Decided(RealSign::Zero)
        )
    };
    if speeds_equal {
        let expression = BezierAlgebraicCuspTwoTermExpression2 {
            rational,
            radical: bivariate_add(first_radical.as_ref(), second_radical.as_ref()),
        };
        return Ok(
            match algebraic_cusp_independent_radical_sum_sign(
                &expression,
                first_speed,
                first_cusp,
                shared_parameter,
                exact,
            )? {
                Classification::Decided(RealSign::Zero) => Classification::Decided(true),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    algebraic_cusp_independent_two_radical_sum_is_zero(
        &rational,
        first_radical.as_ref(),
        first_speed,
        second_radical.as_ref(),
        second_speed_squared.as_ref(),
        first_cusp,
        shared_parameter,
    )
}

pub(super) fn scaled_oriented_rational_diameter(
    diameter: &BezierAlgebraicCuspSemicircleRationalDiameter2,
    scale: &BivariatePolynomial,
    orientation: CurveOverlapOrientation2,
    negate: bool,
) -> (
    BivariatePolynomial,
    Option<(BivariatePolynomial, BivariatePolynomial)>,
) {
    let signed = |polynomial: BivariatePolynomial| {
        if negate {
            bivariate_scale(polynomial, &Real::from(-1_i8))
        } else {
            polynomial
        }
    };
    match diameter {
        BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(coordinate) => {
            let coordinate = bivariate_orient_second_parameter(coordinate, orientation);
            (signed(bivariate_multiply(coordinate.as_ref(), scale)), None)
        }
        BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
            coordinate,
            speed_squared,
        } => {
            let rational = bivariate_orient_second_parameter(&coordinate.rational, orientation);
            let radical = bivariate_orient_second_parameter(&coordinate.radical, orientation);
            let speed_squared =
                bivariate_orient_second_parameter(speed_squared, orientation).into_owned();
            (
                signed(bivariate_multiply(rational.as_ref(), scale)),
                Some((
                    signed(bivariate_multiply(radical.as_ref(), scale)),
                    speed_squared,
                )),
            )
        }
    }
}

pub(super) fn independent_diameter_sum_is_zero(
    rational: BivariatePolynomial,
    first_radical: Option<(BivariatePolynomial, BivariatePolynomial)>,
    second_radical: Option<(BivariatePolynomial, BivariatePolynomial)>,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let zero = |sign| match sign {
        Classification::Decided(RealSign::Zero) => Classification::Decided(true),
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Classification::Decided(false)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    };
    match (first_radical, second_radical) {
        (None, None) => Ok(zero(signed_bivariate_at_parameter_pair(
            &rational,
            cusp_parameter,
            other_parameter,
            policy,
        )?)),
        (Some((radical, speed_squared)), None) | (None, Some((radical, speed_squared))) => {
            Ok(zero(algebraic_cusp_independent_radical_sum_sign(
                &BezierAlgebraicCuspTwoTermExpression2 { rational, radical },
                &speed_squared,
                cusp_parameter,
                other_parameter,
                policy,
            )?))
        }
        (
            Some((first_radical, first_speed_squared)),
            Some((second_radical, second_speed_squared)),
        ) => {
            let speeds_equal = first_speed_squared == second_speed_squared || {
                let difference = bivariate_subtract(&first_speed_squared, &second_speed_squared);
                matches!(
                    signed_bivariate_at_parameter_pair(
                        &difference,
                        cusp_parameter,
                        other_parameter,
                        &CurveContext::STRICT,
                    )?,
                    Classification::Decided(RealSign::Zero)
                )
            };
            if speeds_equal {
                return Ok(zero(algebraic_cusp_independent_radical_sum_sign(
                    &BezierAlgebraicCuspTwoTermExpression2 {
                        rational,
                        radical: bivariate_add(&first_radical, &second_radical),
                    },
                    &first_speed_squared,
                    cusp_parameter,
                    other_parameter,
                    &CurveContext::STRICT,
                )?));
            }
            algebraic_cusp_independent_two_radical_sum_is_zero(
                &rational,
                &first_radical,
                &first_speed_squared,
                &second_radical,
                &second_speed_squared,
                cusp_parameter,
                other_parameter,
            )
        }
    }
}

pub(super) fn rational_parameters_are_complementary_at_cut(
    first: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    second: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    shared_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !policy.accepts_retained_policy(first.data.policy)
        || !policy.accepts_retained_policy(second.data.policy)
    {
        return Err(CurveError::Topology(
            "rational cusp diameter comparison used a different predicate policy".into(),
        ));
    }
    let (
        Some((first_cusp_parameter, _, first_diameter, first_radius)),
        Some((second_cusp_parameter, _, second_diameter, second_radius)),
    ) = (
        first.data.one_field_system(),
        second.data.one_field_system(),
    )
    else {
        return Ok(Classification::Decided(false));
    };
    match first_cusp_parameter.same_value(second_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let oriented_second_radius = bivariate_orient_second_parameter(second_radius, orientation);
    let (first_rational, first_radical) = scaled_oriented_rational_diameter(
        first_diameter,
        oriented_second_radius.as_ref(),
        CurveOverlapOrientation2::Same,
        false,
    );
    let (second_rational, second_radical) =
        scaled_oriented_rational_diameter(second_diameter, first_radius, orientation, false);
    independent_diameter_sum_is_zero(
        bivariate_add(&first_rational, &second_rational),
        first_radical,
        second_radical,
        first_cusp_parameter,
        shared_parameter,
        policy,
    )
}

pub(super) fn rational_parallel_diameter_relation_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    rational_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    opposite: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !policy.accepts_retained_policy(rational.data.policy)
        || !policy.accepts_retained_policy(parallel.data.policy)
    {
        return Err(CurveError::Topology(
            "cross-map cusp diameter comparison used a different predicate policy".into(),
        ));
    }
    let Some((rational_cusp_parameter, _, rational_diameter, rational_radius)) =
        rational.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    let Some((parallel_cusp_parameter, _, parallel_diameter, parallel_radius, parallel_speed)) =
        parallel.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    match rational_cusp_parameter.same_value(parallel_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let parallel_radius_squared_denominator =
        bivariate_orient_second_parameter(parallel_radius, orientation);
    let parallel_diameter = BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
        coordinate: parallel_diameter.clone(),
        speed_squared: parallel_speed.clone(),
    };
    let (parallel_rational, parallel_radical) =
        scaled_oriented_rational_diameter(&parallel_diameter, rational_radius, orientation, false);
    let (rational_term, rational_radical) = scaled_oriented_rational_diameter(
        rational_diameter,
        parallel_radius_squared_denominator.as_ref(),
        CurveOverlapOrientation2::Same,
        !opposite,
    );
    independent_diameter_sum_is_zero(
        bivariate_add(&parallel_rational, &rational_term),
        parallel_radical,
        rational_radical,
        rational_cusp_parameter,
        rational_parameter,
        policy,
    )
}

pub(super) fn bivariate_orient_second_parameter<'a>(
    polynomial: &'a BivariatePolynomial,
    orientation: CurveOverlapOrientation2,
) -> Cow<'a, BivariatePolynomial> {
    if orientation == CurveOverlapOrientation2::Same {
        Cow::Borrowed(polynomial)
    } else {
        Cow::Owned(bivariate_complement_second_parameter(polynomial))
    }
}

pub(super) fn retain_unique_overlap_parameter<F>(
    candidates: Vec<CurveParameter2>,
    range: &CurveParameterRange2,
    map_reversed: bool,
    include_boundaries: bool,
    policy: &CurveContext,
    mut predicate_sign: F,
) -> CurveResult<Classification<CurveParameter2>>
where
    F: FnMut(&CurveParameter2) -> CurveResult<Classification<RealSign>>,
{
    let mut retained = None;
    for map_parameter in candidates {
        let sign = match predicate_sign(&map_parameter)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if sign != RealSign::Zero {
            continue;
        }
        let candidate = if map_reversed {
            map_parameter.unit_complement().ok_or_else(|| {
                CurveError::Topology(
                    "a mapped overlap candidate had no scalar unit-complement".into(),
                )
            })?
        } else {
            map_parameter
        };
        let mut in_range = match CurveParameterDomain2::new(range, None)
            .contains_finite_parameter(&candidate, policy)?
        {
            Classification::Decided(in_range) => in_range,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if in_range && !include_boundaries {
            for endpoint in [range.start(), range.end()] {
                match candidate.same_value(endpoint, policy)? {
                    Classification::Decided(true) => in_range = false,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        if in_range && retained.replace(candidate).is_some() {
            return Err(CurveError::Topology(
                "one cusp cut mapped to multiple parameters in one regular overlap cell".into(),
            ));
        }
    }
    match retained {
        Some(parameter) => Ok(Classification::Decided(parameter)),
        None => Err(CurveError::Topology(
            "cusp cut had no parameter on its published mapped overlap".into(),
        )),
    }
}

pub(super) fn curve_region_parameters_from_bezier(
    parameters: Vec<BezierParameter2>,
) -> Vec<CurveParameter2> {
    parameters.into_iter().map(CurveParameter2::from).collect()
}

pub(super) fn retain_direct_overlap_parameter(
    parameter: CurveParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    match CurveParameterDomain2::new(range, None).contains_finite_parameter(&parameter, policy)? {
        Classification::Decided(true) => Ok(Classification::Decided(parameter)),
        Classification::Decided(false) => Err(CurveError::Topology(
            "cusp cut had no parameter on its published mapped overlap".into(),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn overlap_parameter_is_in_range(
    parameter: &CurveParameter2,
    range: &BezierParameterRange2,
    include_boundaries: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let start_parameter = CurveParameter2::from(range.start().clone());
    let end_parameter = CurveParameter2::from(range.end().clone());
    let orientation = match start_parameter.cmp_by_refinement(&end_parameter, policy)? {
        Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
        Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
        Classification::Decided(std::cmp::Ordering::Equal) => {
            return Err(CurveError::DegenerateOverlapRange);
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let start = match parameter.cmp_by_refinement(&start_parameter, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match parameter.cmp_by_refinement(&end_parameter, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(match orientation {
        std::cmp::Ordering::Less => {
            (start.is_gt() || (include_boundaries && start.is_eq()))
                && (end.is_lt() || (include_boundaries && end.is_eq()))
        }
        std::cmp::Ordering::Greater => {
            (start.is_lt() || (include_boundaries && start.is_eq()))
                && (end.is_gt() || (include_boundaries && end.is_eq()))
        }
        std::cmp::Ordering::Equal => unreachable!("the overlap range is positive-length"),
    }))
}

pub(super) fn bezier_parameter_is_in_curve_region_range(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    include_boundaries: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let parameter = CurveParameter2::from(parameter.clone());
    let start = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match parameter.cmp_by_refinement(range.end(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        (start.is_gt() || (include_boundaries && start.is_eq()))
            && (end.is_lt() || (include_boundaries && end.is_eq())),
    ))
}

pub(super) fn mapped_parameters_for_cusp_endpoint(
    contacts: impl Iterator<
        Item = (
            BezierAlgebraicCuspSemicircleContactLocation2,
            CurveParameter2,
        ),
    >,
    overlaps: Vec<BezierAlgebraicCuspSemicircleMappedOverlap2>,
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    let expected_location = if parameter == &Real::zero() {
        BezierAlgebraicCuspSemicircleContactLocation2::Start
    } else if parameter == &Real::one() {
        BezierAlgebraicCuspSemicircleContactLocation2::End
    } else {
        return Err(CurveError::InvalidCurveParameter);
    };
    let parameter = BezierAlgebraicCuspSemicircleParameter2::Exact(parameter.clone());
    let mut candidates: Vec<CurveParameter2> = contacts
        .filter_map(|(location, other)| (location == expected_location).then_some(other))
        .collect();
    for overlap in overlaps {
        let after_start = match parameter.cmp_by_refinement(&overlap.cusp_start, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => false,
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                true
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_end = match parameter.cmp_by_refinement(&overlap.cusp_end, policy)? {
            Classification::Decided(std::cmp::Ordering::Greater) => false,
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Less) => true,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !after_start || !before_end {
            continue;
        }
        let candidate = match overlap.other_parameter_for_cusp(&parameter, policy)? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut duplicate = false;
        for existing in &candidates {
            match existing.same_value(&candidate, policy)? {
                Classification::Decided(true) => {
                    duplicate = true;
                    break;
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if !duplicate {
            candidates.push(candidate);
        }
    }
    Ok(Classification::Decided(candidates))
}

pub(super) fn rational_parameters_for_cusp_endpoint(
    source: &BezierAlgebraicCuspSemicircle2,
    parameter: &Real,
    target: &RationalBezier2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    match source.rational_intersections(target, range, policy)? {
        Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
            contacts,
            overlaps,
        }) => mapped_parameters_for_cusp_endpoint(
            contacts
                .into_iter()
                .map(|contact| (contact.location, contact.other_parameter)),
            overlaps,
            parameter,
            policy,
        ),
        Classification::Decided(
            BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection
            | BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { .. },
        ) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn parallel_parameters_for_cusp_endpoint(
    source: &BezierAlgebraicCuspSemicircle2,
    parameter: &Real,
    target: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    match source.parallel_intersections(target, range, None, policy)? {
        Classification::Decided(BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
            contacts,
            overlaps,
        }) => mapped_parameters_for_cusp_endpoint(
            contacts.into_iter().map(|contact| {
                (
                    contact.location,
                    CurveParameter2::from(contact.parallel_parameter),
                )
            }),
            overlaps,
            parameter,
            policy,
        ),
        Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
            | BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection
            | BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { .. }
            | BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(_),
        ) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

/// Projects one bivariate relation through a compact selected source scalar.
/// Every local image candidate is replayed against the authored source/image
/// pair on the requested finite range, so conjugate roots introduced by
/// elimination never become geometry. Selected endpoints keep their policy
/// identity and perform the final exact clipping.
pub(super) fn selected_fiber_polynomial_relation_parameters(
    source: &BezierAlgebraicSelectedFiberParameter2,
    relation: &BivariatePolynomial,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
    policy.strict_predicate_pass(|| {
        source.validate_policy(policy)?;
        let image = match source.retained_polynomial_image_relation(relation, policy)? {
            Classification::Decided(Some(image)) => image,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(factor) = image.identically_zero_source_factor {
            match source.predicate_sign(&factor, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if image.identically_zero_image_relation {
            return Ok(Classification::Decided(None));
        }
        let Some(image_relation) = image.relation else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let candidates = match selected_fiber_parameters_in_range(
            &image_relation,
            &source.data.authority.data.retained_parameter,
            range,
            policy,
        )? {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let sign =
                algebraic_selected_fiber_pair_projected_root(source, &candidate, relation, policy)?;
            match sign {
                Classification::Decided(true) => retained.push(candidate),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(Some(retained)))
    })
}

/// Maps one interior point between exact carriers of the same selected circle
/// by their tangent line. On a circle, a tangent line identifies only the
/// point and its antipode; one published semicircle overlap range contains at
/// most one of those interior points. `range` is expressed in the target's
/// own chart, before any reversal in the overlap correspondence.
pub(super) fn mapped_circle_tangent_parameter_candidates(
    source_parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    source_tangent: &[Vec<Real>; 2],
    target_tangent: &[Vec<Real>; 2],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    let incidence = bivariate_subtract(
        &bivariate_outer_product(&source_tangent[0], &target_tangent[1]),
        &bivariate_outer_product(&source_tangent[1], &target_tangent[0]),
    );
    let candidates = match source_parameter {
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(parameter) => {
            // Tangent equations can carry radical coefficients from a
            // transported chamfer. Keep the direct resultant first for an
            // algebraic source: eager quotient-ring reduction can expand
            // those coefficients before the small projection is available.
            let projection = selected_fiber_parameters(&incidence, parameter, range, policy)?;
            match projection {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    parameters,
                )) => Classification::Decided(parameters),
                Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero
                    | BezierAlgebraicFiberProjection2::Degenerate,
                ) => Classification::Uncertain(UncertaintyReason::Unsupported),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(parameter) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "mapped-circle-tangent-inverse",
                "selected-fiber-local-image",
            );
            return Ok(
                match selected_fiber_polynomial_relation_parameters(
                    parameter, &incidence, range, policy,
                )? {
                    Classification::Decided(Some(parameters)) => Classification::Decided(
                        parameters
                            .into_iter()
                            .map(CurveParameter2::from_selected_fiber)
                            .collect(),
                    ),
                    Classification::Decided(None) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
    };
    Ok(candidates.map(curve_region_parameters_from_bezier))
}
