//! Bivariate parameter-pair replay and polynomial helpers.

use super::*;

pub(super) fn parallel_pair_equation_system_with_tangent_fields(
    first: &BezierParallel2,
    second: &BezierParallel2,
    first_tangent_field: Option<&BezierAnalyticParallelTangentField2>,
    second_tangent_field: Option<&BezierAnalyticParallelTangentField2>,
    unit_domain: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParallelPairEquationSystem2>>> {
    let first_distance_sign = match real_sign(first.distance(), policy) {
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Some(RealSign::Zero) => {
            return Err(CurveError::Topology(
                "zero-distance parallel bypassed its exact rational authority".to_owned(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let second_distance_sign = match real_sign(second.distance(), policy) {
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Some(RealSign::Zero) => {
            return Err(CurveError::Topology(
                "zero-distance parallel bypassed its exact rational authority".to_owned(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let first_source = first.source_power_basis()?;
    let second_source = second.source_power_basis()?;
    if unit_domain {
        for source in [&first_source, &second_source] {
            if let Classification::Uncertain(reason) =
                BezierParallel2::certify_finite_source(source, policy)?
            {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let first_differential = first.differential()?;
    let second_differential = second.differential()?;
    let (first_tangent_x, first_tangent_y) = first_tangent_field
        .map(|field| (&field.x[..], &field.y[..]))
        .unwrap_or((&first_differential.tangent_x, &first_differential.tangent_y));
    let (second_tangent_x, second_tangent_y) = second_tangent_field
        .map(|field| (&field.x[..], &field.y[..]))
        .unwrap_or((
            &second_differential.tangent_x,
            &second_differential.tangent_y,
        ));
    if unit_domain {
        for (tangent_x, tangent_y) in [
            (first_tangent_x, first_tangent_y),
            (second_tangent_x, second_tangent_y),
        ] {
            if let Classification::Uncertain(reason) =
                BezierParallel2::certify_regular_tangent_field(tangent_x, tangent_y, policy)?
            {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    // Outside the authored unit chart, regularity belongs to the requested
    // domains. Exact replay excludes zero weights and undefined normals at
    // each candidate; component cells partition those same predicates.
    if unit_domain
        && let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) =
            (first.conservative_bounds()?, second.conservative_bounds()?)
        && matches!(
            first_bounds.overlaps_with_policy(&second_bounds, policy),
            Classification::Decided(false)
        )
    {
        return Ok(Classification::Decided(None));
    }

    let unit_weight = [Real::one()];
    let first_weight = first_source.weight.unwrap_or(&unit_weight);
    let second_weight = second_source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_parameter_difference(
        first_weight,
        second_source.x_numerator,
        first_source.x_numerator,
        second_weight,
    );
    let delta_y = bivariate_parameter_difference(
        first_weight,
        second_source.y_numerator,
        first_source.y_numerator,
        second_weight,
    );
    let first_speed_squared = polynomial_add(
        &polynomial_multiply(first_tangent_x, first_tangent_x),
        &polynomial_multiply(first_tangent_y, first_tangent_y),
    );
    let second_speed_squared = polynomial_add(
        &polynomial_multiply(second_tangent_x, second_tangent_x),
        &polynomial_multiply(second_tangent_y, second_tangent_y),
    );
    let tangent_cross = bivariate_subtract(
        &bivariate_outer_product(first_tangent_x, second_tangent_y),
        &bivariate_outer_product(first_tangent_y, second_tangent_x),
    );
    let tangent_dot = bivariate_add(
        &bivariate_outer_product(first_tangent_x, second_tangent_x),
        &bivariate_outer_product(first_tangent_y, second_tangent_y),
    );
    let second_tangent_x = bivariate_outer_product(&unit_weight, second_tangent_x);
    let second_tangent_y = bivariate_outer_product(&unit_weight, second_tangent_y);
    let first_projection = bivariate_add(
        &bivariate_multiply(&delta_x, &second_tangent_x),
        &bivariate_multiply(&delta_y, &second_tangent_y),
    );
    let second_projection = bivariate_add(
        &bivariate_multiply_first_parameter(&delta_x, first_tangent_x),
        &bivariate_multiply_first_parameter(&delta_y, first_tangent_y),
    );
    let first_normal_projection = bivariate_subtract(
        &bivariate_multiply_first_parameter(&delta_y, first_tangent_x),
        &bivariate_multiply_first_parameter(&delta_x, first_tangent_y),
    );
    let weight = bivariate_outer_product(first_weight, second_weight);
    let weight_squared = bivariate_multiply(&weight, &weight);
    let cross_weight_squared = bivariate_multiply(
        &bivariate_multiply(&tangent_cross, &tangent_cross),
        &weight_squared,
    );
    let first_equation = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_outer_product(&first_speed_squared, &unit_weight),
            &bivariate_multiply(&first_projection, &first_projection),
        ),
        &bivariate_scale(
            cross_weight_squared.clone(),
            &(first.distance() * first.distance()),
        ),
    );
    let second_equation = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_outer_product(&unit_weight, &second_speed_squared),
            &bivariate_multiply(&second_projection, &second_projection),
        ),
        &bivariate_scale(
            cross_weight_squared,
            &(second.distance() * second.distance()),
        ),
    );
    let squared_delta = bivariate_add(
        &bivariate_multiply(&delta_x, &delta_x),
        &bivariate_multiply(&delta_y, &delta_y),
    );
    let distance_square_sum =
        first.distance() * first.distance() + second.distance() * second.distance();
    let norm_residual = bivariate_subtract(
        &squared_delta,
        &bivariate_scale(weight_squared.clone(), &distance_square_sum),
    );
    let speed_product = bivariate_outer_product(&first_speed_squared, &second_speed_squared);
    let norm_equation = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&norm_residual, &norm_residual),
            &speed_product,
        ),
        &bivariate_scale(
            bivariate_multiply(
                &bivariate_multiply(&tangent_dot, &tangent_dot),
                &bivariate_multiply(&weight_squared, &weight_squared),
            ),
            &(Real::from(4_u8)
                * (first.distance() * first.distance())
                * (second.distance() * second.distance())),
        ),
    );
    Ok(Classification::Decided(Some(
        BezierParallelPairEquationSystem2 {
            first_equation,
            second_equation,
            norm_equation,
            first_projection,
            second_projection,
            tangent_cross,
            tangent_dot,
            norm_residual,
            first_normal_projection,
            first_distance: first.distance().clone(),
            second_distance: second.distance().clone(),
            first_distance_sign,
            second_distance_sign,
            weight_product: weight,
        },
    )))
}

pub(super) fn signed_bivariate_for_replay(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    replay: BivariateParameterPairReplay,
    parameter_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match replay {
        BivariateParameterPairReplay::Rejected => unreachable!("rejected pair reached sign replay"),
        BivariateParameterPairReplay::Direct => signed_bivariate_at_parameter_pair(
            polynomial,
            first_parameter,
            second_parameter,
            policy,
        ),
        BivariateParameterPairReplay::LinearLift(axis, map_index) => {
            let (report_index, retained_parameter) = match axis {
                CurveResultantParameter::First => (0, first_parameter),
                CurveResultantParameter::Second => (1, second_parameter),
            };
            signed_bivariate_on_parameter_lift(
                polynomial,
                retained_parameter,
                axis,
                &parameter_lifts[report_index]
                    .as_ref()
                    .expect("a lifted replay retains its report")
                    .maps[map_index],
                policy,
            )
        }
    }
}

pub(super) fn signed_bivariate_for_replay_or_parameter_box(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    replay: BivariateParameterPairReplay,
    parameter_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if let Some(sign) = bivariate_parameter_pair_strict_sign_by_refinement(
        polynomial,
        first_parameter,
        second_parameter,
        policy,
    )? {
        return Ok(Classification::Decided(sign));
    }
    let replayed = signed_bivariate_for_replay(
        polynomial,
        first_parameter,
        second_parameter,
        replay,
        parameter_lifts,
        policy,
    )?;
    Ok(replayed)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn signed_bivariate_for_either_replay(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    first_replay: BivariateParameterPairReplay,
    first_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    second_replay: BivariateParameterPairReplay,
    second_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if let Some(sign) = bivariate_parameter_pair_strict_sign_by_refinement(
        polynomial,
        first_parameter,
        second_parameter,
        policy,
    )? {
        return Ok(Classification::Decided(sign));
    }
    let first = signed_bivariate_for_replay(
        polynomial,
        first_parameter,
        second_parameter,
        first_replay,
        first_lifts,
        policy,
    )?;
    if matches!(first, Classification::Decided(_)) {
        return Ok(first);
    }
    let second = signed_bivariate_for_replay(
        polynomial,
        first_parameter,
        second_parameter,
        second_replay,
        second_lifts,
        policy,
    )?;
    Ok(second)
}

pub(super) fn multiply_nonzero_signs(first: RealSign, second: RealSign) -> RealSign {
    debug_assert!(first != RealSign::Zero && second != RealSign::Zero);
    if first == second {
        RealSign::Positive
    } else {
        RealSign::Negative
    }
}

pub(super) fn parallel_pair_component_selected_at(
    system: &BezierParallelPairEquationSystem2,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    parallel_pair_selected_branch(
        system,
        first_parameter,
        second_parameter,
        BivariateParameterPairReplay::Direct,
        &[None, None],
        BivariateParameterPairReplay::Direct,
        &[None, None],
        true,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn parallel_pair_selected_branch(
    system: &BezierParallelPairEquationSystem2,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    first_replay: BivariateParameterPairReplay,
    first_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    second_replay: BivariateParameterPairReplay,
    second_lifts: &[Option<CurveIntersectionParameterLiftReport>; 2],
    radicals_determine_separation: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let sign = |polynomial: &BivariatePolynomial| {
        signed_bivariate_for_either_replay(
            polynomial,
            first_parameter,
            second_parameter,
            first_replay,
            first_lifts,
            second_replay,
            second_lifts,
            policy,
        )
    };
    let weight_sign = match sign(&system.weight_product)? {
        Classification::Decided(RealSign::Zero) => return Ok(Classification::Decided(false)),
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let tangent_cross = match sign(&system.tangent_cross)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    // Both products vanish exactly when at least one source tangent is zero.
    // A nonzero displacement has no selected normal there. Read the dot sign
    // only on this branch, and retain it for the later norm decision.
    let parallel_tangent_dot = if tangent_cross == RealSign::Zero {
        match sign(&system.tangent_dot)? {
            Classification::Decided(RealSign::Zero) => return Ok(Classification::Decided(false)),
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    } else {
        None
    };
    let first_projection = match sign(&system.first_projection)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second_projection = match sign(&system.second_projection)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if tangent_cross == RealSign::Zero {
        if first_projection != RealSign::Zero || second_projection != RealSign::Zero {
            return Err(CurveError::Topology(
                "tangent-parallel candidate violated its squared projection equations".to_owned(),
            ));
        }
    } else {
        let expected_first = multiply_nonzero_signs(
            multiply_nonzero_signs(system.first_distance_sign, tangent_cross),
            weight_sign,
        );
        let expected_second = multiply_nonzero_signs(
            multiply_nonzero_signs(system.second_distance_sign, tangent_cross),
            weight_sign,
        );
        if first_projection == RealSign::Zero || second_projection == RealSign::Zero {
            return Err(CurveError::Topology(
                "nonparallel candidate lost a radical projection mate".to_owned(),
            ));
        }
        if first_projection != expected_first || second_projection != expected_second {
            return Ok(Classification::Decided(false));
        }
    }

    if radicals_determine_separation && tangent_cross != RealSign::Zero {
        return Ok(Classification::Decided(true));
    }

    let tangent_dot = match parallel_tangent_dot {
        Some(sign) => sign,
        None => match sign(&system.tangent_dot)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
    };
    let norm_residual = match sign(&system.norm_residual)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if tangent_dot == RealSign::Zero {
        if norm_residual != RealSign::Zero {
            return Ok(Classification::Decided(false));
        }
    } else {
        let distance_product =
            multiply_nonzero_signs(system.first_distance_sign, system.second_distance_sign);
        let expected_norm = match multiply_nonzero_signs(distance_product, tangent_dot) {
            RealSign::Positive => RealSign::Negative,
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => unreachable!("nonzero sign product returned zero"),
        };
        if norm_residual == RealSign::Zero || norm_residual != expected_norm {
            return Ok(Classification::Decided(false));
        }
    }

    if tangent_cross != RealSign::Zero {
        return Ok(Classification::Decided(true));
    }
    let selected_normal_distance = if tangent_dot == RealSign::Positive {
        &system.first_distance - &system.second_distance
    } else {
        &system.first_distance + &system.second_distance
    };
    let selected_normal_sign = match real_sign(&selected_normal_distance, policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    if selected_normal_sign == RealSign::Zero {
        return Ok(Classification::Decided(true));
    }
    let first_normal_projection = match sign(&system.first_normal_projection)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if first_normal_projection == RealSign::Zero {
        return Err(CurveError::Topology(
            "nonzero tangent-parallel normal separation replayed as zero".to_owned(),
        ));
    }
    Ok(Classification::Decided(
        multiply_nonzero_signs(first_normal_projection, weight_sign) == selected_normal_sign,
    ))
}

pub(super) fn parallel_rational_intersection_equations(
    source: &BezierParallelPowerBasisRef<'_>,
    differential: &BezierParallelDifferential2,
    distance: &Real,
    other: &RationalParametricCurve2,
) -> (BivariatePolynomial, BivariatePolynomial) {
    let unit_weight = [Real::one()];
    let source_weight = source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_parameter_difference(
        source_weight,
        &other.x_numerator,
        source.x_numerator,
        &other.weight,
    );
    let delta_y = bivariate_parameter_difference(
        source_weight,
        &other.y_numerator,
        source.y_numerator,
        &other.weight,
    );
    let orthogonality = bivariate_add(
        &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_x),
        &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_y),
    );
    let squared_delta = bivariate_add(
        &bivariate_multiply(&delta_x, &delta_x),
        &bivariate_multiply(&delta_y, &delta_y),
    );
    let weighted_distance_squared = bivariate_scale(
        bivariate_outer_product(
            &polynomial_multiply(source_weight, source_weight),
            &polynomial_multiply(&other.weight, &other.weight),
        ),
        &(distance * distance),
    );
    (
        orthogonality,
        bivariate_subtract(&squared_delta, &weighted_distance_squared),
    )
}

pub(super) fn parallel_rational_selected_branch(
    source: &BezierParallelPowerBasisRef<'_>,
    differential: &BezierParallelDifferential2,
    distance: &Real,
    other: &RationalParametricCurve2,
) -> BivariatePolynomial {
    let unit_weight = [Real::one()];
    let source_weight = source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_parameter_difference(
        source_weight,
        &other.x_numerator,
        source.x_numerator,
        &other.weight,
    );
    let delta_y = bivariate_parameter_difference(
        source_weight,
        &other.y_numerator,
        source.y_numerator,
        &other.weight,
    );
    let orientation = bivariate_subtract(
        &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_x),
        &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_y),
    );
    bivariate_scale(
        bivariate_multiply(
            &orientation,
            &bivariate_outer_product(source_weight, &other.weight),
        ),
        distance,
    )
}

pub(super) fn rational_pair_defined_selector(
    first_weight: &[Real],
    second_weight: &[Real],
) -> BivariatePolynomial {
    bivariate_outer_product(
        &polynomial_multiply(first_weight, first_weight),
        &polynomial_multiply(second_weight, second_weight),
    )
}

pub(super) fn parallel_rational_component_branch(
    source: &BezierParallelPowerBasisRef<'_>,
    differential: &BezierParallelDifferential2,
    distance: &Real,
    other: &RationalParametricCurve2,
    distance_sign: RealSign,
) -> BivariatePolynomial {
    if distance_sign == RealSign::Zero {
        let unit_weight = [Real::one()];
        rational_pair_defined_selector(source.weight.unwrap_or(&unit_weight), &other.weight)
    } else {
        parallel_rational_selected_branch(source, differential, distance, other)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum BivariateParameterPairReplay {
    Rejected,
    Direct,
    LinearLift(CurveResultantParameter, usize),
}

#[derive(Default)]
pub(crate) struct BivariateParameterPairReplayCache {
    pub(in crate::bezier_offset) parameter_lifts: [Option<CurveIntersectionParameterLiftReport>; 2],
}

pub(super) fn replay_bivariate_parameter_pair(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
    parameter_lifts: &mut [Option<CurveIntersectionParameterLiftReport>; 2],
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    match bivariate_pair_satisfies_system(first, second, first_parameter, second_parameter, policy)?
    {
        Classification::Decided(true) => {
            return Ok(Classification::Decided(
                BivariateParameterPairReplay::Direct,
            ));
        }
        Classification::Decided(false) => {
            return Ok(Classification::Decided(
                BivariateParameterPairReplay::Rejected,
            ));
        }
        Classification::Uncertain(_) => {}
    }

    for (axis, retained_parameter, fiber_parameter) in [
        (
            CurveResultantParameter::First,
            first_parameter,
            second_parameter,
        ),
        (
            CurveResultantParameter::Second,
            second_parameter,
            first_parameter,
        ),
    ] {
        match parameter_pair_matches_specialized_fiber(
            first,
            second,
            axis,
            retained_parameter,
            fiber_parameter,
            policy,
        )? {
            Classification::Decided(replay) => return Ok(Classification::Decided(replay)),
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(_) => {}
        }
    }

    let first_lifts = parameter_lifts[0].get_or_insert_with(|| {
        linear_parameter_lifts_bivariate_polynomial_system_complete(
            first,
            second,
            CurveResultantParameter::First,
            config,
        )
    });
    match parameter_pair_matches_linear_lift(
        first_lifts,
        first_parameter,
        second_parameter,
        policy,
    )? {
        Classification::Decided(replay) => return Ok(Classification::Decided(replay)),
        Classification::Uncertain(_) => {}
    }

    let second_lifts = parameter_lifts[1].get_or_insert_with(|| {
        linear_parameter_lifts_bivariate_polynomial_system_complete(
            first,
            second,
            CurveResultantParameter::Second,
            config,
        )
    });
    match parameter_pair_matches_linear_lift(
        second_lifts,
        second_parameter,
        first_parameter,
        policy,
    )? {
        Classification::Decided(replay) => Ok(Classification::Decided(replay)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

/// Replays one Cartesian pair drawn from the two resultant projections of an
/// exact bivariate system. The cache shares its exact linear subresultant lifts
/// across every pair from the same system.
pub(crate) fn replay_projected_bivariate_parameter_pair(
    equations: &[BivariatePolynomial; 2],
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
    cache: &mut BivariateParameterPairReplayCache,
) -> CurveResult<Classification<bool>> {
    Ok(replay_bivariate_parameter_pair(
        &equations[0],
        &equations[1],
        first_parameter,
        second_parameter,
        policy,
        config,
        &mut cache.parameter_lifts,
    )?
    .map(|replay| replay != BivariateParameterPairReplay::Rejected))
}

pub(super) fn replay_parallel_rational_contact_pair(
    orthogonality: &BivariatePolynomial,
    distance_relation: &BivariatePolynomial,
    selected_branch: &BivariatePolynomial,
    parallel_parameter: &BezierParameter2,
    rational_parameter: &BezierParameter2,
    config: CurveIntersectionResultantConfig,
    parameter_lifts: &mut [Option<CurveIntersectionParameterLiftReport>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BivariateParameterPairReplay>>> {
    let replay = match replay_bivariate_parameter_pair(
        orthogonality,
        distance_relation,
        parallel_parameter,
        rational_parameter,
        policy,
        config,
        parameter_lifts,
    )? {
        Classification::Decided(BivariateParameterPairReplay::Rejected) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Decided(replay) => replay,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(
        match signed_bivariate_for_replay_or_parameter_box(
            selected_branch,
            parallel_parameter,
            rational_parameter,
            replay,
            parameter_lifts,
            policy,
        )? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(Some(replay)),
            Classification::Decided(RealSign::Negative) => Classification::Decided(None),
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "parallel branch vanished at a regular rational contact".into(),
                ));
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn bivariate_pair_satisfies_system(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let first_sign =
        signed_bivariate_at_parameter_pair(first, first_parameter, second_parameter, policy)?;
    if matches!(
        first_sign,
        Classification::Decided(RealSign::Positive | RealSign::Negative)
    ) {
        return Ok(Classification::Decided(false));
    }
    let mut second_sign =
        signed_bivariate_at_parameter_pair(second, first_parameter, second_parameter, policy)?;
    if matches!(
        second_sign,
        Classification::Decided(RealSign::Positive | RealSign::Negative)
    ) {
        return Ok(Classification::Decided(false));
    }
    if matches!(first_sign, Classification::Decided(RealSign::Zero))
        && matches!(second_sign, Classification::Uncertain(_))
        && let Classification::Decided(Some(reduced)) =
            reduce_bivariate_by_single_axis_equation(first, second, policy)?
    {
        second_sign = signed_bivariate_at_parameter_pair(
            &reduced,
            first_parameter,
            second_parameter,
            policy,
        )?;
        if matches!(
            second_sign,
            Classification::Decided(RealSign::Positive | RealSign::Negative)
        ) {
            return Ok(Classification::Decided(false));
        }
    }
    let mut first_sign = first_sign;
    if matches!(second_sign, Classification::Decided(RealSign::Zero))
        && matches!(first_sign, Classification::Uncertain(_))
        && let Classification::Decided(Some(reduced)) =
            reduce_bivariate_by_single_axis_equation(second, first, policy)?
    {
        first_sign = signed_bivariate_at_parameter_pair(
            &reduced,
            first_parameter,
            second_parameter,
            policy,
        )?;
        if matches!(
            first_sign,
            Classification::Decided(RealSign::Positive | RealSign::Negative)
        ) {
            return Ok(Classification::Decided(false));
        }
    }
    match (first_sign, second_sign) {
        (Classification::Decided(RealSign::Zero), Classification::Decided(RealSign::Zero)) => {
            Ok(Classification::Decided(true))
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Ok(Classification::Uncertain(reason))
        }
        _ => unreachable!("nonzero bivariate signs returned above"),
    }
}

pub(super) fn reduce_bivariate_by_single_axis_equation(
    vanishing: &BivariatePolynomial,
    target: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BivariatePolynomial>>> {
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        let coefficients = match bivariate_single_axis_coefficients(vanishing, axis, policy)? {
            Classification::Decided(Some(coefficients)) => coefficients,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let modulus = match polynomial_from_coefficients(coefficients, policy)? {
            Classification::Decided(Some(modulus)) if modulus.degree() > 0 => modulus,
            Classification::Decided(_) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        return Ok(bivariate_reduce_axis(target, &modulus, axis, policy)?.map(Some));
    }
    Ok(Classification::Decided(None))
}

pub(super) fn parameter_pair_matches_linear_lift(
    report: &CurveIntersectionParameterLiftReport,
    retained_parameter: &BezierParameter2,
    lifted_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    if report.status != CurveIntersectionParameterLiftStatus::Constructed
        || report.retained_parameter == report.lifted_parameter
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let mut blocker = UncertaintyReason::Predicate;
    for (map_index, map) in report.maps.iter().enumerate() {
        match crate::rational_bezier_general::rational_parameter_image_matches(
            retained_parameter,
            lifted_parameter,
            &map.numerator_coefficients,
            &map.denominator_coefficients,
            policy,
        )? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(
                    BivariateParameterPairReplay::LinearLift(report.retained_parameter, map_index),
                ));
            }
            Classification::Decided(false) => {
                return Ok(Classification::Decided(
                    BivariateParameterPairReplay::Rejected,
                ));
            }
            Classification::Uncertain(reason) => {
                blocker = reason;
                continue;
            }
        }
    }
    Ok(Classification::Uncertain(blocker))
}

pub(super) fn parameter_pair_matches_specialized_fiber(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_axis: CurveResultantParameter,
    retained_parameter: &BezierParameter2,
    fiber_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    let (
        BezierParameter2::Algebraic(retained_parameter),
        BezierParameter2::Algebraic(fiber_parameter),
    ) = (retained_parameter, fiber_parameter)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        first,
        second,
        retained_axis,
        &parameter_representation(retained_parameter, policy),
        fiber_parameter.interval().start(),
        fiber_parameter.interval().end(),
        policy.predicate_policy(),
    );
    if report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    Ok(match report.status {
        AlgebraicFiberRootCountStatus::Counted => match report.distinct_root_count {
            Some(0) => Classification::Decided(BivariateParameterPairReplay::Rejected),
            Some(_) => Classification::Decided(BivariateParameterPairReplay::Direct),
            None => Classification::Uncertain(UncertaintyReason::Predicate),
        },
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        AlgebraicFiberRootCountStatus::EndpointRoot
        | AlgebraicFiberRootCountStatus::InvalidEvidence
        | AlgebraicFiberRootCountStatus::InvalidInterval
        | AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        | AlgebraicFiberRootCountStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    })
}

pub(super) fn signed_bivariate_on_parameter_lift(
    polynomial: &BivariatePolynomial,
    retained_parameter: &BezierParameter2,
    retained_axis: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let (cleared, lifted_degree) =
        bivariate_on_parameter_lift_cleared(polynomial, retained_axis, map);
    let cleared_sign = match signed_coefficients_at_parameter(&cleared, retained_parameter, policy)?
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if cleared_sign == RealSign::Zero || lifted_degree.is_multiple_of(2) {
        return Ok(Classification::Decided(cleared_sign));
    }
    let denominator_sign = match signed_coefficients_at_parameter(
        &map.denominator_coefficients,
        retained_parameter,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if denominator_sign == RealSign::Zero {
        return Err(CurveError::Topology(
            "selected bivariate parameter lift denominator vanished".to_owned(),
        ));
    }
    Ok(Classification::Decided(
        match (cleared_sign, denominator_sign) {
            (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
                RealSign::Positive
            }
            (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
                RealSign::Negative
            }
            (RealSign::Zero, _) | (_, RealSign::Zero) => {
                unreachable!("zero signs returned before multiplication")
            }
        },
    ))
}

pub(super) fn bivariate_parameter_pair_is_exact_common_root(
    polynomial: &BivariatePolynomial,
    first: &BezierAlgebraicParameter2,
    second: &BezierAlgebraicParameter2,
) -> bool {
    let policy = &CurveContext::STRICT;
    let attempt = |retained_axis| {
        let (retained, fiber_lower, fiber_upper, defining) = match retained_axis {
            CurveResultantParameter::First => (
                parameter_representation(first, policy),
                second.interval().start(),
                second.interval().end(),
                BivariatePolynomial::new(vec![second.polynomial().coefficients().to_vec()]),
            ),
            CurveResultantParameter::Second => (
                parameter_representation(second, policy),
                first.interval().start(),
                first.interval().end(),
                BivariatePolynomial::new(
                    first
                        .polynomial()
                        .coefficients()
                        .iter()
                        .map(|coefficient| vec![coefficient.clone()])
                        .collect(),
                ),
            ),
        };
        let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
            polynomial,
            &defining,
            retained_axis,
            &retained,
            fiber_lower,
            fiber_upper,
            hypersolve::PredicatePolicy::STRICT,
        );
        (report.status == AlgebraicFiberRootCountStatus::Counted
            && report.certainty == PredicateCertainty::Exact)
            .then_some(report.distinct_root_count.is_some_and(|count| count > 0))
    };
    let orientations = if first.polynomial().degree() <= second.polynomial().degree() {
        [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ]
    } else {
        [
            CurveResultantParameter::Second,
            CurveResultantParameter::First,
        ]
    };
    orientations.into_iter().find_map(attempt).unwrap_or(false)
}

pub(crate) fn signed_bivariate_at_parameter_pair(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match (first_parameter, second_parameter) {
        (BezierParameter2::Exact(first), BezierParameter2::Exact(second)) => {
            match real_sign(
                &Real::eval_poly(&bivariate_specialize_first(polynomial, first), second),
                policy,
            ) {
                Some(sign) => Ok(Classification::Decided(sign)),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        (BezierParameter2::Exact(first), second) => signed_coefficients_at_parameter(
            &bivariate_specialize_first(polynomial, first),
            second,
            policy,
        ),
        (first, BezierParameter2::Exact(second)) => signed_coefficients_at_parameter(
            &bivariate_specialize_second(polynomial, second),
            first,
            policy,
        ),
        (
            first @ BezierParameter2::Algebraic(first_algebraic),
            second @ BezierParameter2::Algebraic(second_algebraic),
        ) => {
            // Most predicates at independently retained algebraic parameters
            // are nonzero and separate under a small number of exact interval
            // refinements. Prove that cheap case before attempting structural
            // reduction or concluding that the two-field value is unknown.
            if let Some(sign) = bivariate_parameter_pair_strict_sign_by_refinement(
                polynomial,
                first,
                second,
                &CurveContext::STRICT,
            )? {
                return Ok(Classification::Decided(sign));
            }
            let mut blocker = UncertaintyReason::Predicate;
            match bivariate_single_axis_coefficients(
                polynomial,
                CurveResultantParameter::First,
                policy,
            )? {
                Classification::Decided(Some(coefficients)) => {
                    return signed_coefficients_at_parameter(&coefficients, first, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            match bivariate_single_axis_coefficients(
                polynomial,
                CurveResultantParameter::Second,
                policy,
            )? {
                Classification::Decided(Some(coefficients)) => {
                    return signed_coefficients_at_parameter(&coefficients, second, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            match signed_rank_one_bivariate_at_parameter_pair(polynomial, first, second, policy)? {
                Classification::Decided(Some(sign)) => {
                    return Ok(Classification::Decided(sign));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            if matches!(
                first.same_value(second, policy)?,
                Classification::Decided(true)
            ) {
                return signed_coefficients_at_parameter(
                    &bivariate_substitute_second_equal_first(polynomial),
                    first,
                    policy,
                );
            }
            let complemented = first.clone().unit_complement();
            if matches!(
                complemented.same_value(second, policy)?,
                Classification::Decided(true)
            ) {
                return signed_coefficients_at_parameter(
                    &bivariate_substitute_second_equal_one_minus_first(polynomial),
                    first,
                    policy,
                );
            }
            let reduced = match bivariate_reduce_parameter_polynomials(
                polynomial,
                first_algebraic.polynomial(),
                second_algebraic.polynomial(),
                policy,
            )? {
                Classification::Decided(reduced) => Some(reduced),
                Classification::Uncertain(reason) => {
                    blocker = reason;
                    None
                }
            };
            if let Some(reduced) = reduced.as_ref() {
                match bivariate_single_axis_coefficients(
                    reduced,
                    CurveResultantParameter::First,
                    policy,
                )? {
                    Classification::Decided(Some(coefficients)) => {
                        return signed_coefficients_at_parameter(&coefficients, first, policy);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => blocker = reason,
                }
                match bivariate_single_axis_coefficients(
                    reduced,
                    CurveResultantParameter::Second,
                    policy,
                )? {
                    Classification::Decided(Some(coefficients)) => {
                        return signed_coefficients_at_parameter(&coefficients, second, policy);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => blocker = reason,
                }
            }
            if let Some((scale, offset)) = exact_parameter_affine_relation(first, second) {
                return signed_coefficients_at_parameter(
                    &bivariate_substitute_second_equal_affine_first(polynomial, &scale, &offset),
                    first,
                    policy,
                );
            }
            Ok(Classification::Uncertain(blocker))
        }
    }
}

pub(super) fn signed_rank_one_bivariate_at_parameter_pair(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RealSign>>> {
    let column_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut pivot = None;
    let mut unknown = false;
    for (row_index, row) in polynomial.coefficients.iter().enumerate() {
        for column_index in 0..column_count {
            let coefficient = row.get(column_index).cloned().unwrap_or_else(Real::zero);
            match real_sign(&coefficient, policy) {
                Some(RealSign::Zero) => {}
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    pivot = Some((row_index, column_index, coefficient, sign));
                    break;
                }
                None => unknown = true,
            }
        }
        if pivot.is_some() {
            break;
        }
    }
    let Some((pivot_row_index, pivot_column_index, pivot_value, pivot_sign)) = pivot else {
        return Ok(if unknown {
            Classification::Uncertain(UncertaintyReason::RealSign)
        } else {
            Classification::Decided(Some(RealSign::Zero))
        });
    };
    let pivot_row = &polynomial.coefficients[pivot_row_index];
    for row in &polynomial.coefficients {
        let column_value = row
            .get(pivot_column_index)
            .cloned()
            .unwrap_or_else(Real::zero);
        for column_index in 0..column_count {
            let coefficient = row.get(column_index).cloned().unwrap_or_else(Real::zero);
            let row_value = pivot_row
                .get(column_index)
                .cloned()
                .unwrap_or_else(Real::zero);
            match real_sign(
                &(coefficient * &pivot_value - &column_value * row_value),
                policy,
            ) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(None));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
    }

    let first_sign = signed_coefficients_at_parameter(
        &polynomial
            .coefficients
            .iter()
            .map(|row| {
                row.get(pivot_column_index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
            })
            .collect::<Vec<_>>(),
        first_parameter,
        policy,
    )?;
    let second_sign = signed_coefficients_at_parameter(
        &(0..column_count)
            .map(|column_index| {
                pivot_row
                    .get(column_index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
            })
            .collect::<Vec<_>>(),
        second_parameter,
        policy,
    )?;
    Ok(match (first_sign, second_sign) {
        (Classification::Decided(RealSign::Zero), _)
        | (_, Classification::Decided(RealSign::Zero)) => {
            Classification::Decided(Some(RealSign::Zero))
        }
        (
            Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)),
            Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)),
        ) => Classification::Decided(Some(
            if (first == second) == (pivot_sign == RealSign::Positive) {
                RealSign::Positive
            } else {
                RealSign::Negative
            },
        )),
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Classification::Uncertain(reason)
        }
    })
}

pub(super) fn bivariate_reduce_parameter_polynomials(
    polynomial: &BivariatePolynomial,
    first: &BezierParameterPolynomial,
    second: &BezierParameterPolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    let first_reduced =
        match bivariate_reduce_axis(polynomial, first, CurveResultantParameter::First, policy)? {
            Classification::Decided(reduced) => reduced,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    bivariate_reduce_axis(
        &first_reduced,
        second,
        CurveResultantParameter::Second,
        policy,
    )
}

pub(super) fn bivariate_reduce_axis(
    polynomial: &BivariatePolynomial,
    modulus: &BezierParameterPolynomial,
    axis: CurveResultantParameter,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    let axis_degree = match axis {
        CurveResultantParameter::First => polynomial.coefficients.len().saturating_sub(1),
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| row.len().saturating_sub(1))
            .max()
            .unwrap_or(0),
    };
    if modulus.degree() > axis_degree {
        return Ok(Classification::Decided(polynomial.clone()));
    }
    match axis {
        CurveResultantParameter::First => {
            let second_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            let mut columns = Vec::with_capacity(second_count);
            for second_power in 0..second_count {
                let coefficients = polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.get(second_power).cloned().unwrap_or_else(Real::zero))
                    .collect();
                match modulus.reduce_power_basis(coefficients, policy)? {
                    Classification::Decided(coefficients) => columns.push(coefficients),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let first_count = columns.iter().map(Vec::len).max().unwrap_or(0);
            let coefficients = (0..first_count)
                .map(|first_power| {
                    columns
                        .iter()
                        .map(|column| column.get(first_power).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect();
            Ok(Classification::Decided(BivariatePolynomial::new(
                coefficients,
            )))
        }
        CurveResultantParameter::Second => {
            let mut coefficients = Vec::with_capacity(polynomial.coefficients.len());
            for row in &polynomial.coefficients {
                match modulus.reduce_power_basis(row.clone(), policy)? {
                    Classification::Decided(row) => coefficients.push(row),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(BivariatePolynomial::new(
                coefficients,
            )))
        }
    }
}

pub(super) fn bivariate_single_axis_coefficients(
    polynomial: &BivariatePolynomial,
    axis: CurveResultantParameter,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut unknown = false;
    match axis {
        CurveResultantParameter::First => {
            for row in &polynomial.coefficients {
                for coefficient in row.iter().skip(1) {
                    match real_sign(coefficient, policy) {
                        Some(RealSign::Zero) => {}
                        Some(RealSign::Positive | RealSign::Negative) => {
                            return Ok(Classification::Decided(None));
                        }
                        None => unknown = true,
                    }
                }
            }
            if unknown {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            Ok(Classification::Decided(Some(
                polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
                    .collect(),
            )))
        }
        CurveResultantParameter::Second => {
            for row in polynomial.coefficients.iter().skip(1) {
                for coefficient in row {
                    match real_sign(coefficient, policy) {
                        Some(RealSign::Zero) => {}
                        Some(RealSign::Positive | RealSign::Negative) => {
                            return Ok(Classification::Decided(None));
                        }
                        None => unknown = true,
                    }
                }
            }
            if unknown {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            Ok(Classification::Decided(Some(
                polynomial.coefficients.first().cloned().unwrap_or_default(),
            )))
        }
    }
}

/// `true` when every root on the closed range is an endpoint.
///
/// An identically zero polynomial is an interior singularity. An uncertain
/// endpoint comparison stays uncertain instead of dropping a one-sided frame.
pub(super) fn polynomial_roots_touch_only_range_endpoints(
    coefficients: &[Real],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let roots = match CurveParameterDomain2::new(range, None).finite_roots(&polynomial, policy)? {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if roots.is_empty() {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    for root in roots {
        let root = CurveParameter2::from(root);
        let at_start = root.cmp_by_refinement(range.start(), policy)?;
        let at_end = root.cmp_by_refinement(range.end(), policy)?;
        match (at_start, at_end) {
            (Classification::Decided(std::cmp::Ordering::Equal), _)
            | (_, Classification::Decided(std::cmp::Ordering::Equal)) => {}
            (
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
            ) => return Ok(Classification::Decided(false)),
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(true))
}
