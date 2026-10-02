//! Selected-fiber root predicates, parameters and signs.

use super::*;

/// Selected-fiber counterpart of
/// [`algebraic_selected_square_root_polynomial_is_identically_zero`].
///
/// The authored expression has center parameter `u` on its first axis and
/// candidate parameter `v` on its second. Each `v` coefficient is lifted into
/// the compact `F(alpha,u)=0` authority and signed there, so neither `u` nor
/// its positive speed root is globally materialized.
pub(super) fn selected_fiber_square_root_polynomial_is_identically_zero(
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    center: &BezierAlgebraicSelectedFiberParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let speed = bivariate_specialize_second(radicand, &Real::zero());
    if *radicand != bivariate_outer_product(&speed, &[Real::one()]) {
        return Err(CurveError::Topology(
            "a selected-fiber center speed unexpectedly depended on the target parameter".into(),
        ));
    }
    let speed = bivariate_outer_product(&[Real::one()], &speed);
    match center.predicate_sign(&speed, policy)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "a selected-fiber center squared speed was negative".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let coefficient_count = expression
        .rational
        .coefficients
        .iter()
        .chain(&expression.radical.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    for power in 0..coefficient_count {
        let coefficient = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_outer_product(
                &[Real::one()],
                &bivariate_second_parameter_coefficient(&expression.rational, power),
            ),
            radical: bivariate_outer_product(
                &[Real::one()],
                &bivariate_second_parameter_coefficient(&expression.radical, power),
            ),
        };
        match center.square_root_sum_sign(&coefficient, &speed, policy)? {
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

pub(super) enum SelectedParallelEquationProjection2 {
    Candidates(Vec<BezierAlgebraicSelectedFiberParameter2>),
    IdenticallyZero,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn selected_parallel_normal_positive_dimensional_projection(
    circle: &BezierParallelTwoNormalExpression2,
    squared_branch: &BezierAlgebraicCuspTwoTermExpression2,
    center_speed_squared: &BivariatePolynomial,
    candidate_speed_squared: &BivariatePolynomial,
    center_parameter: &BezierAlgebraicParameter2,
    range: &CurveParameterRange2,
    incident: Option<&BezierParallelIncidentDomain2>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierSelectedParallelNormalPositiveProjection2>> {
    let project = |equation: &BivariatePolynomial|
     -> CurveResult<Classification<SelectedParallelEquationProjection2>> {
        let mut parameters = match selected_fiber_parameters_in_range(equation, center_parameter, range, policy)? {
            Classification::Decided(Some(parameters)) => parameters,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(
                    SelectedParallelEquationProjection2::IdenticallyZero,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(incident) = incident {
            match selected_fiber_parameters_on_incident_ray(
                equation,
                center_parameter,
                &incident.anchor,
                incident.direction,
                incident.barrier.as_ref(),
                policy,
            )? {
                Classification::Decided(Some(exterior)) => {
                    for parameter in exterior {
                        match CurveParameterDomain2::new(range, None).contains_finite_parameter(
                            &CurveParameter2::from_selected_fiber(parameter.clone()), policy,
                        )? {
                            Classification::Decided(true) => {},
                            Classification::Decided(false) => parameters.push(parameter),
                            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                        }
                    }
                },
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        SelectedParallelEquationProjection2::IdenticallyZero,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(
            SelectedParallelEquationProjection2::Candidates(parameters),
        ))
    };
    let selected_expression_is_zero = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
        algebraic_selected_square_root_polynomial_is_identically_zero(
            &expression.rational,
            &expression.radical,
            center_speed_squared,
            center_parameter,
            policy,
        )
    };
    parallel_normal_positive_dimensional_projection(
        circle,
        squared_branch,
        center_speed_squared,
        candidate_speed_squared,
        range,
        &project,
        &selected_expression_is_zero,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn parallel_normal_positive_dimensional_projection(
    circle: &BezierParallelTwoNormalExpression2,
    squared_branch: &BezierAlgebraicCuspTwoTermExpression2,
    center_speed_squared: &BivariatePolynomial,
    candidate_speed_squared: &BivariatePolynomial,
    finite_range: &CurveParameterRange2,
    project: &impl Fn(
        &BivariatePolynomial,
    ) -> CurveResult<Classification<SelectedParallelEquationProjection2>>,
    selected_expression_is_zero: &impl Fn(
        &BezierAlgebraicCuspTwoTermExpression2,
    ) -> CurveResult<Classification<bool>>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierSelectedParallelNormalPositiveProjection2>> {
    let strict = policy.strict_counterpart();
    let finish_projection = |projection: SelectedParallelEquationProjection2| match projection {
        SelectedParallelEquationProjection2::Candidates(parameters) => Some(
            BezierSelectedParallelNormalPositiveProjection2::Candidates(parameters),
        ),
        SelectedParallelEquationProjection2::IdenticallyZero => None,
    };
    let project_nonzero_branch =
        |first: &BivariatePolynomial,
         second: &BivariatePolynomial|
         -> CurveResult<Classification<BezierSelectedParallelNormalPositiveProjection2>> {
            for equation in [first, second] {
                let projection = match project(equation)? {
                    Classification::Decided(projection) => projection,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match projection {
                    SelectedParallelEquationProjection2::Candidates(parameters) => {
                        return Ok(Classification::Decided(
                            BezierSelectedParallelNormalPositiveProjection2::Candidates(parameters),
                        ));
                    }
                    SelectedParallelEquationProjection2::IdenticallyZero => {}
                }
            }
            Ok(Classification::Decided(
                BezierSelectedParallelNormalPositiveProjection2::Degenerate,
            ))
        };

    let candidate_speed = bivariate_specialize_first(candidate_speed_squared, &Real::zero());
    if *candidate_speed_squared != bivariate_outer_product(&[Real::one()], &candidate_speed) {
        return Err(CurveError::Topology(
            "a target speed unexpectedly depended on the selected center parameter".into(),
        ));
    }
    match polynomial_square_root(&candidate_speed, policy)? {
        Classification::Decided(Some(mut speed)) => {
            // A polynomial square root has two sheets. Select the positive
            // sheet on the actual regular finite cell: an affine carrier edit
            // may move that cell across a speed zero lying outside its domain,
            // so the historical sign-at-zero convention is insufficient.
            let sample = match policy
                .strict_predicate_pass(|| finite_range.strict_interior_scalar(policy))?
            {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match real_sign(&Real::eval_poly(&speed, &sample), &strict) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Negative) => {
                    speed = polynomial_scale(&speed, &Real::from(-1_i8));
                }
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let speed = bivariate_outer_product(&[Real::one()], &speed);
            let collapsed = BezierAlgebraicCuspTwoTermExpression2 {
                rational: bivariate_add(
                    &circle.rational,
                    &bivariate_multiply(&circle.candidate, &speed),
                ),
                radical: bivariate_add(
                    &circle.center,
                    &bivariate_multiply(&circle.product, &speed),
                ),
            };
            match selected_expression_is_zero(&collapsed)? {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(
                        BezierSelectedParallelNormalPositiveProjection2::CoincidentCircleComponent,
                    ));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let norm = bivariate_subtract(
                &bivariate_multiply(&collapsed.rational, &collapsed.rational),
                &bivariate_multiply(
                    &bivariate_multiply(&collapsed.radical, &collapsed.radical),
                    center_speed_squared,
                ),
            );
            let projection = match project(&norm)? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Some(result) = finish_projection(projection) {
                return Ok(Classification::Decided(result));
            }
            // The norm itself can vanish on the opposite center-speed sheet.
            // Selected zeros then require both coefficients to vanish, so
            // either nonzero coefficient is a complete projection.
            project_nonzero_branch(&collapsed.rational, &collapsed.radical)
        }
        Classification::Decided(None) => {
            let selected_norm_is_zero = match selected_expression_is_zero(squared_branch)? {
                Classification::Decided(is_zero) => is_zero,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if !selected_norm_is_zero {
                // The final norm vanished because the opposite center-speed
                // conjugate owns the component. On the selected sheet roots
                // are common zeros of the once-squared coefficient pair.
                return project_nonzero_branch(&squared_branch.rational, &squared_branch.radical);
            }

            // The selected once-squared branch is
            // `(L + M sqrt(T)) (L - M sqrt(T))`. Since T is not a polynomial
            // square, the authored positive target-speed component is zero
            // exactly when both one-center-radical coefficient polynomials L
            // and M vanish. If only the opposite target sheet owns the
            // component, selected contacts are their common switching zeros.
            let center_term = BezierAlgebraicCuspTwoTermExpression2 {
                rational: circle.rational.clone(),
                radical: circle.center.clone(),
            };
            let target_term = BezierAlgebraicCuspTwoTermExpression2 {
                rational: circle.candidate.clone(),
                radical: circle.product.clone(),
            };
            let center_term_is_zero = match selected_expression_is_zero(&center_term)? {
                Classification::Decided(is_zero) => is_zero,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let target_term_is_zero = match selected_expression_is_zero(&target_term)? {
                Classification::Decided(is_zero) => is_zero,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if center_term_is_zero && target_term_is_zero {
                return Ok(Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::CoincidentCircleComponent,
                ));
            }
            let selected_term = if center_term_is_zero {
                &target_term
            } else {
                &center_term
            };
            let norm = bivariate_subtract(
                &bivariate_multiply(&selected_term.rational, &selected_term.rational),
                &bivariate_multiply(
                    &bivariate_multiply(&selected_term.radical, &selected_term.radical),
                    center_speed_squared,
                ),
            );
            let projection = match project(&norm)? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Some(result) = finish_projection(projection) {
                return Ok(Classification::Decided(result));
            }
            project_nonzero_branch(&selected_term.rational, &selected_term.radical)
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn selected_parameter_fiber_parameters(
    incidence: &BivariatePolynomial,
    parameter: &BezierParameter2,
    max_resultant_degree: usize,
    max_quotient_degree: usize,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => {
            let coefficients = bivariate_specialize_first(incidence, parameter);
            let polynomial = match polynomial_from_coefficients(coefficients, policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        BezierAlgebraicFiberProjection2::IdenticallyZero,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(
                isolate_polynomial_roots_in_range_envelope(&polynomial, range, policy)?
                    .map(BezierAlgebraicFiberProjection2::Parameters),
            )
        }
        BezierParameter2::Algebraic(parameter) => {
            algebraic_selected_reduced_fiber_parameters_with_resultant_limit(
                incidence,
                parameter,
                max_resultant_degree,
                max_quotient_degree,
                range,
                policy,
            )
        }
    }
}

/// Projects onto the requested finite range while preserving the selected
/// source fiber. Exact source scalars specialize directly; algebraic sources
/// retain the direct-resultant-first schedule and its exact replay fallback.
pub(crate) fn selected_fiber_parameters(
    incidence: &BivariatePolynomial,
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    match parameter {
        BezierParameter2::Exact(_) => selected_parameter_fiber_parameters(
            incidence,
            parameter,
            MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
            range,
            policy,
        ),
        BezierParameter2::Algebraic(parameter) => {
            algebraic_selected_fiber_parameters_with_resultant_limit(
                incidence,
                parameter,
                MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                range,
                policy,
            )
        }
    }
}

pub(super) fn algebraic_selected_fiber_parameters_with_incident_ray(
    incidence: &BivariatePolynomial,
    cusp: &BezierAlgebraicParameter2,
    domain: CurveParameterDomain2<'_>,
    max_resultant_degree: usize,
    max_quotient_degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let cusp_root = parameter_representation(cusp, policy);
    let report = project_bivariate_fiber_at_algebraic_parameter(
        incidence,
        CurveResultantParameter::First,
        &cusp_root,
        max_quotient_degree,
    );
    let coefficients = match report.status {
        AlgebraicFiberProjectionStatus::Constructed => report.coefficients,
        AlgebraicFiberProjectionStatus::InvalidEvidence => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberProjectionStatus::UnsupportedCoefficient
        | AlgebraicFiberProjectionStatus::Undecided => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-incident-fiber-projection",
                "general-resultant-fallback",
            );
            let defining = BivariatePolynomial::new(
                cusp.polynomial()
                    .coefficients()
                    .iter()
                    .map(|coefficient| vec![coefficient.clone()])
                    .collect(),
            );
            let report = resultant_bivariate_polynomial_system(
                &defining,
                incidence,
                CurveResultantParameter::Second,
                CurveIntersectionResultantConfig {
                    min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                    max_resultant_degree,
                },
            );
            match report.status {
                CurveIntersectionResultantStatus::Constructed => report.resultant_coefficients,
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
        }
    };
    if coefficients
        .iter()
        .all(|coefficient| real_sign(coefficient, policy) == Some(RealSign::Zero))
    {
        return Ok(Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero,
        ));
    }
    if coefficients
        .iter()
        .all(|coefficient| real_sign(coefficient, policy).is_none())
    {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    }
    let coefficients =
        hypersolve::square_free_part(coefficients.clone(), hypersolve::PredicatePolicy::STRICT)
            .unwrap_or(coefficients);
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidates = match selected_axis_parameters_in_domain(domain, policy, |axis| {
        Ok(axis
            .isolate(&polynomial, policy)?
            .map(BezierAlgebraicFiberProjection2::Parameters))
    })? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
            parameters
        }
        Classification::Decided(_) => unreachable!("a nonzero polynomial has isolated roots"),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let cusp_parameter = BezierParameter2::Algebraic(cusp.clone());
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let belongs = match algebraic_selected_fiber_contains_parameter(
            incidence,
            &cusp_parameter,
            &candidate,
            policy,
        )? {
            Classification::Decided(belongs) => belongs,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if belongs {
            retained.push(candidate);
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(retained),
    ))
}

/// Isolates one selected bivariate fiber between represented affine bounds.
///
/// Seeds the finite selected fiber shared by scalar and circle-component
/// authorities. Borrow the incidence and consume the caller's refinement
/// parameter; seeding adds no retained polynomial copies.
pub(super) fn selected_fiber_root_intervals_in_interval(
    incidence: &BivariatePolynomial,
    retained_parameter: BezierParameter2,
    lower: &Real,
    upper: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<IsolatedRootInterval>>>> {
    let refined_retained = retained_parameter.refined_isolating_interval(64, &CurveContext::STRICT);
    let retained_root = match refined_retained {
        BezierParameter2::Algebraic(parameter) => parameter_representation(&parameter, policy),
        BezierParameter2::Exact(parameter) => {
            AlgebraicRootRepresentation::from_exact_value(&parameter)
        }
    };
    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter_complete(
        incidence,
        CurveResultantParameter::First,
        &retained_root,
        lower,
        upper,
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 512,
            refinement_steps: 8,
        },
        policy.predicate_policy(),
    );
    if report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    Ok(match report.status {
        AlgebraicFiberRootIsolationStatus::Isolated => {
            Classification::Decided(Some(report.intervals))
        }
        AlgebraicFiberRootIsolationStatus::NoRoots => Classification::Decided(Some(Vec::new())),
        AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => Classification::Decided(None),
        AlgebraicFiberRootIsolationStatus::InvalidEvidence
        | AlgebraicFiberRootIsolationStatus::InvalidInterval => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        AlgebraicFiberRootIsolationStatus::DepthLimit
        | AlgebraicFiberRootIsolationStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    })
}

/// The retained first-axis parameter stays in its local algebraic field and
/// all second-axis roots share one compact authority. This is the bounded
/// counterpart of [`selected_fiber_parameters_on_incident_ray`]; together the
/// two cover a ray whose authored endpoint is itself algebraic without ever
/// turning an isolating bound into construction evidence.
pub(super) fn selected_fiber_parameters_in_interval(
    incidence: &BivariatePolynomial,
    retained_parameter: &BezierAlgebraicParameter2,
    lower: &Real,
    upper: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
    let roots = match selected_fiber_root_intervals_in_interval(
        incidence,
        BezierParameter2::Algebraic(retained_parameter.clone()),
        lower,
        upper,
        policy,
    )? {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let authority = BezierAlgebraicSelectedFiberAuthority2::new(
        incidence.clone(),
        retained_parameter.clone(),
        policy,
    );
    Ok(Classification::Decided(Some(
        roots
            .into_iter()
            .map(|root| authority.parameter(root))
            .collect(),
    )))
}

/// Isolates a selected fiber on either orientation of one finite parameter
/// range. Algebraic endpoints contribute only outward isolation bounds; exact
/// selected comparisons perform the final closed-range filtering.
pub(super) fn selected_fiber_parameters_in_range(
    incidence: &BivariatePolynomial,
    retained_parameter: &BezierAlgebraicParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
    // Exact decisions keep the caller's evidence identity. Replacing the
    // context with its strict counterpart would detach retained endpoints.
    policy.strict_predicate_pass(|| {
        let domain = CurveParameterDomain2::new(range, None);
        if let [univariate] = incidence.coefficients.as_slice() {
            let polynomial = match polynomial_from_coefficients(univariate.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let roots = match domain.finite_roots(&polynomial, policy)? {
                Classification::Decided(roots) => roots,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let authority = BezierAlgebraicSelectedFiberAuthority2::new(
                incidence.clone(),
                retained_parameter.clone(),
                policy,
            );
            return Ok(Classification::Decided(Some(
                roots
                    .into_iter()
                    .map(|root| {
                        authority.parameter(match root {
                            BezierParameter2::Exact(root) => IsolatedRootInterval {
                                lower: root.clone(),
                                upper: root.clone(),
                                exact_root: Some(root),
                                distinct_root_count: 1,
                            },
                            BezierParameter2::Algebraic(root) => IsolatedRootInterval {
                                lower: root.interval().start().clone(),
                                upper: root.interval().end().clone(),
                                exact_root: None,
                                distinct_root_count: 1,
                            },
                        })
                    })
                    .collect(),
            )));
        }
        let (_, [lower, upper]) = match domain.finite_envelope(policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let candidates = match selected_fiber_parameters_in_interval(
            incidence,
            retained_parameter,
            lower,
            upper,
            policy,
        )? {
            Classification::Decided(Some(candidates)) => candidates,
            decided @ Classification::Decided(None) => return Ok(decided),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            match domain.contains_finite_parameter(
                &CurveParameter2::from_selected_fiber(candidate.clone()),
                policy,
            )? {
                Classification::Decided(true) => retained.push(candidate),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(Some(retained)))
    })
}

/// Isolates one selected bivariate fiber on an open incident ray without
/// constructing its degree-multiplied norm. Hypersolve sees the compact chart
/// `u in (0, 1)` while every returned scalar is transported back to the
/// original affine parameter and shares the original incidence authority.
pub(super) fn selected_fiber_parameters_on_incident_ray(
    incidence: &BivariatePolynomial,
    retained_parameter: &BezierAlgebraicParameter2,
    anchor: &Real,
    direction: BezierParameterRayDirection2,
    barrier: Option<&BezierParameter2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
    let Some(compact_incidence) = bivariate_compose_incident_parameter(
        incidence,
        CurveResultantParameter::Second,
        anchor,
        direction,
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    };
    let refined_retained = BezierParameter2::Algebraic(retained_parameter.clone())
        .refined_isolating_interval(64, &CurveContext::STRICT);
    let retained_root = match refined_retained {
        BezierParameter2::Algebraic(parameter) => parameter_representation(&parameter, policy),
        BezierParameter2::Exact(parameter) => {
            AlgebraicRootRepresentation::from_exact_value(&parameter)
        }
    };
    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter_complete(
        &compact_incidence,
        CurveResultantParameter::First,
        &retained_root,
        &Real::zero(),
        &Real::one(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 512,
            refinement_steps: 8,
        },
        hypersolve::PredicatePolicy::STRICT,
    );
    let roots = match report.status {
        AlgebraicFiberRootIsolationStatus::Isolated => report.intervals,
        AlgebraicFiberRootIsolationStatus::NoRoots => Vec::new(),
        AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => {
            return Ok(Classification::Decided(None));
        }
        AlgebraicFiberRootIsolationStatus::InvalidEvidence
        | AlgebraicFiberRootIsolationStatus::InvalidInterval => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        AlgebraicFiberRootIsolationStatus::DepthLimit
        | AlgebraicFiberRootIsolationStatus::Undecided => {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
    };
    let compact_authority = BezierAlgebraicSelectedFiberAuthority2::new(
        compact_incidence,
        retained_parameter.clone(),
        policy,
    );
    let authority = BezierAlgebraicSelectedFiberAuthority2::new(
        incidence.clone(),
        retained_parameter.clone(),
        policy,
    );
    let zero = Real::zero();
    let one = Real::one();
    let map = |compact: &Real| -> CurveResult<Real> {
        let distance = (compact / (&one - compact))?;
        Ok(match direction {
            BezierParameterRayDirection2::Decreasing => anchor - distance,
            BezierParameterRayDirection2::Increasing => anchor + distance,
        })
    };
    let mut parameters = Vec::with_capacity(roots.len());
    for root in roots {
        let compact = compact_authority.parameter(root);
        if compact.order_to_real(&zero, policy)?
            != Classification::Decided(std::cmp::Ordering::Greater)
            || compact.order_to_real(&one, policy)?
                != Classification::Decided(std::cmp::Ordering::Less)
        {
            continue;
        }
        let mut refinement_steps = 0_usize;
        let compact = loop {
            let refined = match compact.refined(refinement_steps, policy)? {
                Classification::Decided(refined) => refined,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if compare_reals(&refined.root().upper, &one, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                break refined;
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("incident selected-fiber refinement overflow".into())
                })?;
        };
        let first = map(&compact.root().lower)?;
        let second = map(&compact.root().upper)?;
        let (lower, upper) = match direction {
            BezierParameterRayDirection2::Decreasing => (second, first),
            BezierParameterRayDirection2::Increasing => (first, second),
        };
        let mapped = authority.parameter(IsolatedRootInterval {
            lower,
            upper,
            exact_root: compact.root().exact_root.as_ref().map(&map).transpose()?,
            distinct_root_count: compact.root().distinct_root_count,
        });
        if let Some(barrier) = barrier {
            let ordering = match mapped.cmp_bezier_parameter(barrier, policy)? {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before = match direction {
                BezierParameterRayDirection2::Decreasing => ordering == std::cmp::Ordering::Greater,
                BezierParameterRayDirection2::Increasing => ordering == std::cmp::Ordering::Less,
            };
            if !before {
                continue;
            }
        }
        parameters.push(mapped);
    }
    Ok(Classification::Decided(Some(parameters)))
}

pub(super) fn algebraic_selected_fiber_parameters_with_resultant_limit(
    incidence: &BivariatePolynomial,
    cusp: &BezierAlgebraicParameter2,
    max_resultant_degree: usize,
    max_quotient_degree: usize,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let (_, [cell_lower, cell_upper]) =
        match CurveParameterDomain2::new(range, None).finite_envelope(policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let cusp_root = parameter_representation(cusp, policy);
    let defining = BivariatePolynomial::new(
        cusp.polynomial()
            .coefficients()
            .iter()
            .map(|coefficient| vec![coefficient.clone()])
            .collect(),
    );
    let report = resultant_bivariate_polynomial_system(
        &defining,
        incidence,
        CurveResultantParameter::Second,
        CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree,
        },
    );
    let projection = if report.status == CurveIntersectionResultantStatus::DegreeBoundExceeded {
        match algebraic_selected_quotient_ring_fiber_projection_with_max_degree(
            incidence,
            &cusp_root,
            max_quotient_degree,
            range,
            policy,
        )? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else {
        match resultant_parameter_polynomial(report, policy)? {
            Classification::Decided(Some(polynomial)) => {
                match isolate_polynomial_roots_in_range_envelope(&polynomial, range, policy)? {
                    Classification::Decided(parameters) if parameters.is_empty() => {
                        ResultantParameterProjection::Empty
                    }
                    Classification::Decided(parameters) => {
                        ResultantParameterProjection::Parameters(parameters)
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Classification::Decided(None) => ResultantParameterProjection::Degenerate,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    };
    let candidates = match projection {
        ResultantParameterProjection::Empty => Vec::new(),
        ResultantParameterProjection::Parameters(parameters) => parameters,
        ResultantParameterProjection::SelectedParameters(parameters) => {
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::Parameters(parameters),
            ));
        }
        ResultantParameterProjection::Degenerate => {
            let full_fiber = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
                incidence,
                CurveResultantParameter::First,
                &cusp_root,
                cell_lower,
                cell_upper,
                policy.predicate_policy(),
            );
            if full_fiber.certainty == PredicateCertainty::Approximate {
                policy.observe_approximate_512();
            }
            if full_fiber.status == AlgebraicFiberRootCountStatus::IdenticallyZeroFiber {
                return Ok(Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero,
                ));
            }
            if full_fiber.status == AlgebraicFiberRootCountStatus::Counted
                && full_fiber.distinct_root_count == Some(0)
            {
                return Ok(Classification::Decided(
                    BezierAlgebraicFiberProjection2::Parameters(Vec::new()),
                ));
            }
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::Degenerate,
            ));
        }
    };
    // Projection isolators are ordered and contain every selected-fiber root.
    // Midpoints of certified rootless gaps give disjoint cells whose boundary
    // variations are shared by the batch local-field Sturm count.
    let mut cell_boundaries = Vec::with_capacity(candidates.len() + 1);
    cell_boundaries.push(cell_lower.clone());
    for pair in candidates.windows(2) {
        let left = match &pair[0] {
            BezierParameter2::Exact(parameter) => parameter,
            BezierParameter2::Algebraic(parameter) => parameter.interval().end(),
        };
        let right = match &pair[1] {
            BezierParameter2::Exact(parameter) => parameter,
            BezierParameter2::Algebraic(parameter) => parameter.interval().start(),
        };
        match compare_reals(left, right, policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Some(std::cmp::Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "selected-field projection isolators are not ordered".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        cell_boundaries.push(((left + right) / Real::from(2_i8)).map_err(|_| {
            CurveError::Topology("selected-field projection midpoint division failed".into())
        })?);
    }
    cell_boundaries.push(cell_upper.clone());
    let candidate_intervals = candidates
        .iter()
        .enumerate()
        .filter_map(|(index, candidate)| match candidate {
            BezierParameter2::Algebraic(_) => {
                Some((&cell_boundaries[index], &cell_boundaries[index + 1]))
            }
            BezierParameter2::Exact(_) => None,
        })
        .collect::<Vec<_>>();
    let candidate_reports = count_bivariate_fiber_roots_at_algebraic_parameter_intervals(
        incidence,
        CurveResultantParameter::First,
        &cusp_root,
        &candidate_intervals,
        policy.predicate_policy(),
    );
    drop(candidate_intervals);
    let cusp_parameter = BezierParameter2::Algebraic(cusp.clone());
    let mut report_index = 0_usize;
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let batched_membership = if matches!(&candidate, BezierParameter2::Algebraic(_)) {
            let report = candidate_reports.get(report_index).ok_or_else(|| {
                CurveError::Topology("selected-field candidate report mismatch".into())
            })?;
            report_index += 1;
            if report.certainty == PredicateCertainty::Approximate {
                policy.observe_approximate_512();
            }
            match report.status {
                AlgebraicFiberRootCountStatus::Counted => match report.distinct_root_count {
                    Some(0) => Some(false),
                    Some(1) => Some(true),
                    Some(_) | None => None,
                },
                AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
                | AlgebraicFiberRootCountStatus::EndpointRoot
                | AlgebraicFiberRootCountStatus::InvalidEvidence
                | AlgebraicFiberRootCountStatus::InvalidInterval
                | AlgebraicFiberRootCountStatus::UnsupportedCoefficient
                | AlgebraicFiberRootCountStatus::Undecided => None,
            }
        } else {
            None
        };
        let belongs = match match batched_membership {
            Some(belongs) => Classification::Decided(belongs),
            None => algebraic_selected_fiber_contains_parameter(
                incidence,
                &cusp_parameter,
                &candidate,
                policy,
            )?,
        } {
            Classification::Decided(belongs) => belongs,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if belongs {
            retained.push(candidate);
        }
    }
    if report_index != candidate_reports.len() {
        return Err(CurveError::Topology(
            "selected-field candidate report mismatch".into(),
        ));
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(retained),
    ))
}

/// Isolates every root in a represented outer envelope of one finite range.
///
/// Algebraic range endpoints contribute only their certified outward bounds;
/// callers retain the original endpoints for final exact membership tests.
/// Direct interval isolation preserves every root and multiplicity while
/// avoiding both out-of-range components and coefficient-inflating affine
/// composition.
pub(super) fn isolate_polynomial_roots_in_range_envelope(
    polynomial: &BezierParameterPolynomial,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<BezierParameter2>>> {
    let (_, [lower, upper]) =
        match CurveParameterDomain2::new(range, None).finite_envelope(policy)? {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    // Preserve the exact nonrational low-degree and Bernstein authorities
    // when the finite envelope is the unit interval. General Sturm replay
    // alone need not recognize the same correlated coefficient identities.
    if lower == &Real::zero() && upper == &Real::one() {
        return polynomial.isolate_unit_interval_roots(policy);
    }
    polynomial.isolate_interval_roots(lower, upper, policy)
}

pub(super) fn algebraic_selected_quotient_ring_fiber_projection_with_max_degree(
    incidence: &BivariatePolynomial,
    cusp_root: &hypersolve::AlgebraicRootRepresentation,
    max_quotient_degree: usize,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<ResultantParameterProjection>> {
    let report = project_bivariate_fiber_at_algebraic_parameter(
        incidence,
        CurveResultantParameter::First,
        cusp_root,
        max_quotient_degree,
    );
    match report.status {
        AlgebraicFiberProjectionStatus::Constructed => {}
        AlgebraicFiberProjectionStatus::InvalidEvidence => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberProjectionStatus::UnsupportedCoefficient => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        AlgebraicFiberProjectionStatus::Undecided => {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
    }
    if report
        .coefficients
        .iter()
        .all(|coefficient| real_sign(coefficient, policy) == Some(RealSign::Zero))
    {
        return Ok(Classification::Decided(
            ResultantParameterProjection::Degenerate,
        ));
    }
    if report
        .coefficients
        .iter()
        .all(|coefficient| real_sign(coefficient, policy).is_none())
    {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    }
    // This projection is used only as a root carrier: multiplicity belongs to
    // the incidence evidence and its value/sign is never a geometric
    // predicate. Reduce repeated factors exactly before constructing the
    // scalar field so squared eliminants do not inflate every later replay.
    // STRICT owns the reduction even under APPROXIMATE_512; if the exact GCD
    // cannot be certified, retaining the original norm is still exact.
    let coefficients = hypersolve::square_free_part(
        report.coefficients.clone(),
        hypersolve::PredicatePolicy::STRICT,
    )
    .unwrap_or(report.coefficients);
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if polynomial.degree() > MAX_DIRECT_SELECTED_NORM_ISOLATION_DEGREE {
        return algebraic_selected_parameters_from_norm(
            incidence, cusp_root, polynomial, range, policy,
        );
    }
    let parameters = isolate_polynomial_roots_in_range_envelope(&polynomial, range, policy)?;
    match parameters {
        Classification::Decided(parameters) if parameters.is_empty() => {
            Ok(Classification::Decided(ResultantParameterProjection::Empty))
        }
        Classification::Decided(parameters) => Ok(Classification::Decided(
            ResultantParameterProjection::Parameters(parameters),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

/// Isolates only roots in the selected algebraic fiber, then certifies one
/// distinct quotient-norm root in each interval. Foreign conjugate roots never
/// enter the high-degree norm's global Sturm sequence, and a repeated norm root
/// is retained without being mislabeled as simple.
pub(super) fn algebraic_selected_parameters_from_norm(
    incidence: &BivariatePolynomial,
    retained_root: &hypersolve::AlgebraicRootRepresentation,
    norm: BezierParameterPolynomial,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<ResultantParameterProjection>> {
    let (_, [lower, upper]) =
        match CurveParameterDomain2::new(range, None).finite_envelope(policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let mut refinement_steps = 8_usize;
    loop {
        let report = isolate_bivariate_fiber_roots_at_algebraic_parameter_complete(
            incidence,
            CurveResultantParameter::First,
            retained_root,
            lower,
            upper,
            AlgebraicFiberRootIsolationConfig {
                max_subdivision_depth: 256,
                refinement_steps,
            },
            policy.predicate_policy(),
        );
        if report.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        let intervals = match report.status {
            AlgebraicFiberRootIsolationStatus::Isolated => report.intervals,
            AlgebraicFiberRootIsolationStatus::NoRoots => {
                return Ok(Classification::Decided(ResultantParameterProjection::Empty));
            }
            AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => {
                return Ok(Classification::Decided(
                    ResultantParameterProjection::Degenerate,
                ));
            }
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberRootIsolationStatus::InvalidEvidence
            | AlgebraicFiberRootIsolationStatus::InvalidInterval => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberRootIsolationStatus::DepthLimit
            | AlgebraicFiberRootIsolationStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };
        let mut parameters = Vec::with_capacity(intervals.len());
        let mut retry = false;
        for interval in intervals {
            if let Some(root) = interval.exact_root {
                match real_sign(&norm.evaluate(&root), policy) {
                    Some(RealSign::Zero) => {}
                    Some(RealSign::Positive | RealSign::Negative) => {
                        return Err(CurveError::Topology(
                            "selected fiber root was absent from its quotient norm".into(),
                        ));
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
                // The root and its finite interval are certified above;
                // exterior parameters do not belong to the unit constructor.
                parameters.push(BezierParameter2::Exact(root));
                continue;
            }
            let exact_singleton = hypersolve::polynomial_has_one_distinct_root_in_open_interval(
                norm.coefficients(),
                &interval.lower,
                &interval.upper,
                hypersolve::PredicatePolicy::STRICT,
            );
            let singleton = match exact_singleton {
                Some(singleton) => Some(singleton),
                None if policy.permits_approximate_512() => {
                    let singleton = hypersolve::polynomial_has_one_distinct_root_in_open_interval(
                        norm.coefficients(),
                        &interval.lower,
                        &interval.upper,
                        hypersolve::PredicatePolicy::APPROXIMATE_512,
                    );
                    if singleton.is_some() {
                        policy.observe_approximate_512();
                    }
                    singleton
                }
                None => None,
            };
            let parameter_interval = match BezierParameterInterval::try_new(
                interval.lower,
                interval.upper,
                &CurveContext::STRICT,
            )? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            // The open-interval proof excludes both bounds, but the norm may
            // also vanish there on another sheet. Remove those factors before
            // the fallback count or publication of a native isolator: both
            // require non-root endpoints, including when refinement is needed.
            let polynomial = match norm.clone().without_roots_at(
                &[parameter_interval.start(), parameter_interval.end()],
                policy,
            )? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if singleton != Some(true) {
                match polynomial.root_count_in_interval(&parameter_interval, policy)? {
                    Classification::Decided(0) => {
                        return Err(CurveError::Topology(
                            "a selected fiber root was absent from its quotient norm".into(),
                        ));
                    }
                    Classification::Decided(1) => {}
                    Classification::Decided(_) => {
                        retry = true;
                        break;
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            parameters.push(match polynomial.coefficients() {
                [constant, slope] => BezierParameter2::Exact((-constant / slope)?),
                _ => BezierParameter2::Algebraic(
                    BezierAlgebraicParameter2::from_certified_singleton(
                        polynomial,
                        parameter_interval,
                    ),
                ),
            });
        }
        if !retry {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-fiber-projection",
                "local-field-norm-isolation",
            );
            return Ok(Classification::Decided(
                ResultantParameterProjection::SelectedParameters(parameters),
            ));
        }
        #[cfg(feature = "dispatch-trace")]
        if refinement_steps >= 128 {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-fiber-norm-isolation",
                "unbounded-cold-continuation",
            );
        }
        refinement_steps = refinement_steps.checked_mul(2).ok_or_else(|| {
            CurveError::Topology("selected-fiber norm refinement depth overflow".into())
        })?;
    }
}

/// Projects a polynomial already reduced in one retained selected-root field,
/// then rejects norm roots contributed only by conjugate roots. The same
/// unsquared polynomial is both incidence and predicate, so one correlated
/// replay proves selected-fiber membership without the general resultant plus
/// batched Sturm pass. Any degenerate or undecided quotient falls back to that
/// complete authority.
pub(super) fn algebraic_selected_reduced_fiber_parameters(
    incidence: &BivariatePolynomial,
    cusp: &BezierAlgebraicParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    algebraic_selected_reduced_fiber_parameters_with_resultant_limit(
        incidence,
        cusp,
        MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
        range,
        policy,
    )
}

pub(super) fn algebraic_selected_reduced_fiber_parameters_with_resultant_limit(
    incidence: &BivariatePolynomial,
    cusp: &BezierAlgebraicParameter2,
    max_resultant_degree: usize,
    max_quotient_degree: usize,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let cusp_root = parameter_representation(cusp, policy);
    let projection = algebraic_selected_quotient_ring_fiber_projection_with_max_degree(
        incidence,
        &cusp_root,
        max_quotient_degree,
        range,
        policy,
    )?;
    let candidates = match projection {
        Classification::Decided(ResultantParameterProjection::Empty) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-fiber-projection",
                "quotient-ring",
            );
            Vec::new()
        }
        Classification::Decided(ResultantParameterProjection::Parameters(candidates)) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-fiber-projection",
                "quotient-ring",
            );
            candidates
        }
        Classification::Decided(ResultantParameterProjection::SelectedParameters(candidates)) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-fiber-projection",
                "local-field-norm-isolation",
            );
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::Parameters(candidates),
            ));
        }
        Classification::Decided(ResultantParameterProjection::Degenerate)
        | Classification::Uncertain(_) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-selected-fiber-projection",
                "general-resultant-fallback",
            );
            return algebraic_selected_fiber_parameters_with_resultant_limit(
                incidence,
                cusp,
                max_resultant_degree,
                max_quotient_degree,
                range,
                policy,
            );
        }
    };
    let cusp_parameter = BezierParameter2::Algebraic(cusp.clone());
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        match algebraic_selected_correlated_predicate_sign(
            incidence,
            incidence,
            &cusp_parameter,
            &candidate,
            policy,
        )? {
            Classification::Decided(RealSign::Zero) => retained.push(candidate),
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Uncertain(_) => {
                return algebraic_selected_fiber_parameters_with_resultant_limit(
                    incidence,
                    cusp,
                    max_resultant_degree,
                    max_quotient_degree,
                    range,
                    policy,
                );
            }
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(retained),
    ))
}

pub(super) fn algebraic_selected_fiber_contains_parameter(
    incidence: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if cusp_parameter.scalar().is_some() || other_parameter.scalar().is_some() {
        return Ok(
            match signed_bivariate_at_parameter_pair(
                incidence,
                cusp_parameter,
                other_parameter,
                policy,
            )? {
                Classification::Decided(sign) => Classification::Decided(sign == RealSign::Zero),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    let mut cusp_refinement = BezierParameterRefinement2::new(cusp_parameter, policy);
    let mut other_refinement = BezierParameterRefinement2::new(other_parameter, policy);
    let mut steps = 0_usize;
    loop {
        let cusp = cusp_refinement.refine_to(steps).clone();
        let other = other_refinement.refine_to(steps).clone();
        let (BezierParameter2::Algebraic(cusp), BezierParameter2::Algebraic(other)) = (cusp, other)
        else {
            return algebraic_selected_fiber_contains_parameter(
                incidence,
                cusp_refinement.refine_to(steps),
                other_refinement.refine_to(steps),
                policy,
            );
        };
        let report = count_bivariate_fiber_roots_at_algebraic_parameter(
            incidence,
            CurveResultantParameter::First,
            &parameter_representation(&cusp, policy),
            other.interval().start(),
            other.interval().end(),
            policy.predicate_policy(),
        );
        if report.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        match report.status {
            AlgebraicFiberRootCountStatus::Counted => match report.distinct_root_count {
                Some(0) => return Ok(Classification::Decided(false)),
                Some(1) => return Ok(Classification::Decided(true)),
                Some(_) | None => {}
            },
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            AlgebraicFiberRootCountStatus::EndpointRoot => {}
            AlgebraicFiberRootCountStatus::InvalidEvidence
            | AlgebraicFiberRootCountStatus::InvalidInterval => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberRootCountStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        }
        steps = steps
            .checked_mul(2)
            .and_then(|steps| steps.checked_add(1))
            .ok_or_else(|| CurveError::Topology("selected-field refinement overflow".into()))?;
    }
}

pub(crate) fn algebraic_selected_correlated_predicate_sign(
    incidence: &BivariatePolynomial,
    predicate: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    // Equal retained parameter evidence denotes one selected field, not two
    // independent roots of the same defining polynomial.  Collapse that
    // diagonal before interval or common-fiber replay so exact zero predicates
    // cannot be mistaken for an identically-zero independent fiber.
    if cusp_parameter == other_parameter {
        let diagonal = bivariate_substitute_second_equal_first(predicate);
        return match cusp_parameter {
            BezierParameter2::Exact(parameter) => {
                Ok(
                    real_sign(&Real::eval_poly(&diagonal, parameter), policy).map_or(
                        Classification::Uncertain(UncertaintyReason::RealSign),
                        Classification::Decided,
                    ),
                )
            }
            BezierParameter2::Algebraic(_) => {
                signed_coefficients_at_parameter(&diagonal, cusp_parameter, policy)
            }
        };
    }
    if let Classification::Decided(sign) =
        signed_bivariate_at_parameter_pair(predicate, cusp_parameter, other_parameter, policy)?
    {
        return Ok(Classification::Decided(sign));
    }
    if !matches!(cusp_parameter, BezierParameter2::Algebraic(_))
        || !matches!(other_parameter, BezierParameter2::Algebraic(_))
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }

    let mut cusp_refinement = BezierParameterRefinement2::new(cusp_parameter, policy);
    let mut other_refinement = BezierParameterRefinement2::new(other_parameter, policy);
    let mut steps = 0_usize;
    loop {
        let cusp = cusp_refinement.refine_to(steps).clone();
        let other = other_refinement.refine_to(steps).clone();
        if let Some(sign) = bivariate_parameter_box_strict_sign(predicate, &cusp, &other, policy)? {
            return Ok(Classification::Decided(sign));
        }
        // Nonzero predicates usually separate after a handful of exact
        // bisections. Give the division-free box certificate that bounded
        // opportunity before constructing a local-field GCD solely to decide
        // whether the correlated value is zero.
        if steps < 31 {
            steps = steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("selected-field sign refinement overflow".into())
                })?;
            continue;
        }
        let (BezierParameter2::Algebraic(cusp), BezierParameter2::Algebraic(other)) =
            (&cusp, &other)
        else {
            return signed_bivariate_at_parameter_pair(predicate, &cusp, &other, policy);
        };
        let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
            incidence,
            predicate,
            CurveResultantParameter::First,
            &parameter_representation(cusp, policy),
            other.interval().start(),
            other.interval().end(),
            policy.predicate_policy(),
        );
        if zero_report.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        match zero_report.status {
            AlgebraicFiberRootCountStatus::Counted
                if zero_report
                    .distinct_root_count
                    .is_some_and(|count| count > 0) =>
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
                return Ok(Classification::Decided(RealSign::Zero));
            }
            AlgebraicFiberRootCountStatus::Counted
            | AlgebraicFiberRootCountStatus::EndpointRoot => {}
            AlgebraicFiberRootCountStatus::InvalidEvidence
            | AlgebraicFiberRootCountStatus::InvalidInterval => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberRootCountStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        }
        steps = steps
            .checked_mul(2)
            .and_then(|steps| steps.checked_add(1))
            .ok_or_else(|| {
                CurveError::Topology("selected-field sign refinement overflow".into())
            })?;
    }
}

/// Refines one exact root retained directly in a selected algebraic fiber.
///
/// The returned interval continues to name a root of `incidence(t, u)` over
/// the already selected `t = alpha`; it is deliberately not converted into a
/// univariate norm whose degree multiplies by the degree of `alpha`.
pub(super) fn selected_fiber_parameter_at_exact_retained(
    incidence: &BivariatePolynomial,
    retained: &Real,
    root: &IsolatedRootInterval,
) -> CurveResult<Classification<BezierParameter2>> {
    if let Some(exact) = &root.exact_root {
        return Ok(Classification::Decided(BezierParameter2::Exact(
            exact.clone(),
        )));
    }
    let polynomial = match polynomial_from_coefficients(
        bivariate_specialize_first(incidence, retained),
        &CurveContext::STRICT,
    )? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => {
            return Err(CurveError::Topology(
                "a selected-fiber singleton became an identically-zero univariate incidence".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let interval = match BezierParameterInterval::try_new(
        root.lower.clone(),
        root.upper.clone(),
        &CurveContext::STRICT,
    )? {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, &CurveContext::STRICT)?
            .map(BezierParameter2::Algebraic),
    )
}

pub(super) fn algebraic_selected_fiber_root_interval_refined(
    authority: &BezierAlgebraicSelectedFiberAuthority2,
    root: &IsolatedRootInterval,
    refinement_steps: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<IsolatedRootInterval>> {
    if refinement_steps == 0 || root.exact_root.is_some() {
        return Ok(Classification::Decided(root.clone()));
    }
    let incidence = &authority.data.incidence;
    let retained_parameter = authority.retained_parameter_refined(refinement_steps.max(64));
    let BezierParameter2::Algebraic(retained_parameter) = retained_parameter else {
        let BezierParameter2::Exact(retained_parameter) = retained_parameter else {
            unreachable!()
        };
        let parameter =
            match selected_fiber_parameter_at_exact_retained(incidence, &retained_parameter, root)?
            {
                Classification::Decided(parameter) => {
                    parameter.refined_isolating_interval(refinement_steps, &CurveContext::STRICT)
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        return Ok(Classification::Decided(match parameter {
            BezierParameter2::Exact(root) => IsolatedRootInterval {
                lower: root.clone(),
                upper: root.clone(),
                exact_root: Some(root),
                distinct_root_count: 1,
            },
            BezierParameter2::Algebraic(root) => IsolatedRootInterval {
                lower: root.interval().start().clone(),
                upper: root.interval().end().clone(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }));
    };
    let retained = parameter_representation(&retained_parameter, policy);
    // All roots and point images sharing this incidence reuse the same exact
    // coefficient field and lazy Sturm authority during refinement.
    let report = (|| {
        let mut cache = authority
            .data
            .root_refiner
            .lock()
            .expect("selected-fiber refinement cache mutex poisoned");
        if cache.is_none() {
            *cache = Some(Box::new(hypersolve::AlgebraicFiberRootRefiner::try_new(
                incidence,
                CurveResultantParameter::First,
                &retained,
                hypersolve::PredicatePolicy::STRICT,
            )?));
        }
        Ok::<_, hypersolve::AlgebraicFiberRootIsolationReport>(
            cache
                .as_mut()
                .expect("an admitted selected-fiber refiner")
                .refine(root, refinement_steps),
        )
    })()
    .unwrap_or_else(|report| report);
    match report.status {
        AlgebraicFiberRootIsolationStatus::Isolated if report.intervals.len() == 1 => {
            Ok(Classification::Decided(
                report
                    .intervals
                    .into_iter()
                    .next()
                    .expect("a singleton selected-fiber report has one interval"),
            ))
        }
        AlgebraicFiberRootIsolationStatus::Isolated => Err(CurveError::Topology(
            "a certified selected-fiber singleton refined to multiple roots".into(),
        )),
        AlgebraicFiberRootIsolationStatus::NoRoots => Err(CurveError::Topology(
            "a certified selected-fiber singleton disappeared during refinement".into(),
        )),
        AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => Err(CurveError::Topology(
            "a selected-fiber contact incidence became identically zero during refinement".into(),
        )),
        AlgebraicFiberRootIsolationStatus::InvalidEvidence
        | AlgebraicFiberRootIsolationStatus::InvalidInterval => {
            Err(CurveError::InvalidBezierAlgebraicParameter)
        }
        AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        AlgebraicFiberRootIsolationStatus::DepthLimit
        | AlgebraicFiberRootIsolationStatus::Undecided => {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }
}

/// Signs a polynomial at one root retained locally over `Q(alpha)`.
///
/// Strict Bernstein boxes decide every separated nonzero value. Exact
/// common-fiber counting decides zero without a primitive element. Only the
/// final 512-step equality query may consume APPROXIMATE_512; STRICT continues
/// exact isolation without treating that schedule as a mathematical bound.
pub(super) fn algebraic_selected_fiber_root_predicate_sign(
    authority: &BezierAlgebraicSelectedFiberAuthority2,
    predicate: &BivariatePolynomial,
    root: &IsolatedRootInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if matches!(bivariate_exact_nonzero_metadata(predicate), Some(None)) {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    let incidence = &authority.data.incidence;
    if predicate == incidence || divide_bivariate_polynomial_exact(predicate, incidence).is_some() {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    let retained_parameter = authority.retained_parameter_refined(64);
    if predicate.coefficients.iter().all(|row| row.len() <= 1) {
        return signed_coefficients_at_parameter(
            &predicate
                .coefficients
                .iter()
                .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
                .collect::<Vec<_>>(),
            &retained_parameter,
            policy,
        );
    }
    // Any refinement stage may discover a represented retained root. Share
    // its exact univariate dispatch with the initial 64-step fast path.
    let sign_at_exact_retained = |retained_value: &Real, selected_root: &IsolatedRootInterval| {
        if let Some(exact_root) = &selected_root.exact_root {
            return Ok(real_sign(
                &bivariate_evaluate_exact(predicate, retained_value, exact_root),
                policy,
            )
            .map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        let parameter = match selected_fiber_parameter_at_exact_retained(
            incidence,
            retained_value,
            selected_root,
        )? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        signed_coefficients_at_parameter(
            &bivariate_specialize_first(predicate, retained_value),
            &parameter,
            policy,
        )
    };
    if let BezierParameter2::Exact(retained_value) = &retained_parameter {
        return sign_at_exact_retained(retained_value, root);
    }
    if let Some(exact_root) = &root.exact_root
        && let Classification::Decided(sign) = signed_coefficients_at_parameter(
            &bivariate_specialize_second(predicate, exact_root),
            &retained_parameter,
            &CurveContext::STRICT,
        )?
    {
        return Ok(Classification::Decided(sign));
    }

    let BezierParameter2::Algebraic(refined_retained) = &retained_parameter else {
        unreachable!("a selected-fiber root has an algebraic retained parameter")
    };
    // A selected fiber may retain the source diagonal u=alpha. If alpha is
    // strictly inside this singleton's isolator, both scalars are the same root;
    // replay the predicate on that diagonal instead of refining a zero box.
    // Cache the coefficient identity on the shared incidence authority.
    if *authority
        .data
        .incidence_has_parameter_diagonal
        .get_or_init(|| {
            bivariate_substitute_second_equal_first(incidence)
                .iter()
                .all(|coefficient| {
                    real_sign(coefficient, &CurveContext::STRICT) == Some(RealSign::Zero)
                })
        })
        && matches!(
            retained_parameter.cmp_by_refinement(
                &BezierParameter2::Exact(root.lower.clone()),
                &CurveContext::STRICT,
            )?,
            Classification::Decided(std::cmp::Ordering::Greater)
        )
        && matches!(
            retained_parameter.cmp_by_refinement(
                &BezierParameter2::Exact(root.upper.clone()),
                &CurveContext::STRICT,
            )?,
            Classification::Decided(std::cmp::Ordering::Less)
        )
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-fiber-predicate-sign",
            "retained-diagonal-identity",
        );
        return signed_coefficients_at_parameter(
            &bivariate_substitute_second_equal_first(predicate),
            &retained_parameter,
            policy,
        );
    }
    let retained_root = parameter_representation(refined_retained, policy);
    let mut latest = root.clone();
    let mut refinement_steps = 0_usize;
    let mut exact_nonzero = false;
    let next_refinement_steps = |steps: usize| {
        if steps == 0 {
            Ok(2)
        } else {
            steps.checked_mul(2).ok_or_else(|| {
                CurveError::Topology("selected-fiber predicate refinement overflow".into())
            })
        }
    };
    loop {
        if refinement_steps != 0 {
            latest = match algebraic_selected_fiber_root_interval_refined(
                authority,
                root,
                refinement_steps,
                policy,
            )? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        if let Some(exact_root) = &latest.exact_root {
            return signed_coefficients_at_parameter(
                &bivariate_specialize_second(predicate, exact_root),
                &retained_parameter,
                policy,
            );
        }
        let retained_refined = match authority.retained_parameter_refined(refinement_steps.max(64))
        {
            BezierParameter2::Algebraic(parameter) => parameter,
            BezierParameter2::Exact(value) => return sign_at_exact_retained(&value, &latest),
        };
        let restricted = predicate.substitute_affine(
            &(retained_refined.interval().end() - retained_refined.interval().start()),
            retained_refined.interval().start(),
            &(&latest.upper - &latest.lower),
            &latest.lower,
        );
        if let Some(sign) =
            bivariate_unit_square_strict_bernstein_sign(&restricted, &CurveContext::STRICT)?
        {
            return Ok(Classification::Decided(sign));
        }

        // A speculative geometric pass may use the retained singleton box,
        // but must yield before constructing or traversing a fiber Sturm
        // sequence. The complete caller can first try correlated point or
        // parameter evidence, then replay this predicate without that bound.
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        // Full replay may certify signs that have no immediate scalar fact.
        // Reuse the restricted polynomial: its exact affine substitution
        // retains cancellations within this box that direct Horner bounds
        // on the original coordinates can lose before common-root replay.
        let unit = RealInterval {
            lower: Real::zero(),
            upper: Real::one(),
        };
        if let Some(enclosure) =
            RealInterval::evaluate_bivariate_power_basis(&restricted, &unit, &unit)
                .and_then(|interval| interval.strict_nonzero_sign())
        {
            return Ok(Classification::Decided(enclosure));
        }
        // A zero predicate cannot acquire a strict interval sign. Replay its
        // common-root certificate before spending more on interval refinement;
        // once nonzero is proved, retain that fact while refining its sign.
        if !exact_nonzero {
            let predicate_policy = if refinement_steps == 512 && policy.permits_approximate_512() {
                policy.predicate_policy()
            } else {
                hypersolve::PredicatePolicy::STRICT
            };
            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                incidence,
                predicate,
                CurveResultantParameter::First,
                &retained_root,
                &latest.lower,
                &latest.upper,
                predicate_policy,
            );
            exact_nonzero = zero_report.certainty == PredicateCertainty::Exact
                && zero_report.status == AlgebraicFiberRootCountStatus::Counted
                && zero_report.distinct_root_count == Some(0);
            if zero_report.certainty == PredicateCertainty::Approximate {
                policy.observe_approximate_512();
            }
            match zero_report.status {
                AlgebraicFiberRootCountStatus::Counted
                    if zero_report
                        .distinct_root_count
                        .is_some_and(|count| count > 0) =>
                {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                AlgebraicFiberRootCountStatus::Counted
                | AlgebraicFiberRootCountStatus::EndpointRoot => {}
                AlgebraicFiberRootCountStatus::InvalidEvidence
                | AlgebraicFiberRootCountStatus::InvalidInterval => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
                    // Keep the existing interval opportunity when the algebraic
                    // replay cannot admit these coefficients.
                    if refinement_steps >= 32 {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                }
                AlgebraicFiberRootCountStatus::Undecided => {
                    if refinement_steps == 512 && policy.permits_approximate_512() {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                }
            }
        }
        if refinement_steps == 512 && policy.permits_approximate_512() && !exact_nonzero {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        #[cfg(feature = "dispatch-trace")]
        if refinement_steps == 512 {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-fiber-predicate-sign",
                "unbounded-cold-continuation",
            );
        }
        refinement_steps = next_refinement_steps(refinement_steps)?;
    }
}

pub(super) fn validate_selected_fiber_pair_base(
    first: &BezierAlgebraicSelectedFiberParameter2,
    second: &BezierAlgebraicSelectedFiberParameter2,
) -> CurveResult<()> {
    if first.data.authority.data.retained_parameter == second.data.authority.data.retained_parameter
    {
        Ok(())
    } else {
        Err(CurveError::Topology(
            "a selected-fiber pair lost its shared retained base".into(),
        ))
    }
}

/// Signs `predicate(u, v)` for two roots retained over the same selected
/// algebraic base parameter.
///
/// Separated values use only the two compact fiber boxes. Exact zero is
/// decided by a local subresultant GCD over the selected `(alpha,u)` field;
/// neither selected scalar is promoted to a degree-multiplied global norm.
pub(super) fn algebraic_selected_fiber_pair_predicate_sign(
    first: &BezierAlgebraicSelectedFiberParameter2,
    second: &BezierAlgebraicSelectedFiberParameter2,
    predicate: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    first.validate_policy(policy)?;
    second.validate_policy(policy)?;
    if policy.permits_approximate_512() {
        match policy.strict_predicate_pass(|| {
            algebraic_selected_fiber_pair_predicate_sign(first, second, predicate, policy)
        })? {
            decided @ Classification::Decided(_) => return Ok(decided),
            Classification::Uncertain(_) => {}
        }
    }
    if matches!(bivariate_exact_nonzero_metadata(predicate), Some(None)) {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    if first == second {
        return first.predicate_sign(
            &bivariate_outer_product(
                &[Real::one()],
                &bivariate_substitute_second_equal_first(predicate),
            ),
            policy,
        );
    }
    match (first.represented_value(), second.represented_value()) {
        (Some(first), Some(second)) => {
            return Ok(
                real_sign(&bivariate_evaluate_exact(predicate, first, second), policy).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    Classification::Decided,
                ),
            );
        }
        (Some(first), None) => {
            return second.predicate_sign(
                &bivariate_outer_product(
                    &[Real::one()],
                    &bivariate_specialize_first(predicate, first),
                ),
                policy,
            );
        }
        (None, Some(second)) => {
            return first.predicate_sign(
                &bivariate_outer_product(
                    &[Real::one()],
                    &bivariate_specialize_second(predicate, second),
                ),
                policy,
            );
        }
        (None, None) => {}
    }
    validate_selected_fiber_pair_base(first, second)?;

    let strict = policy;
    let mut previous = None;
    let mut certified_nonzero = false;
    let mut steps = 0_usize;
    loop {
        let refined_first = match first.refined(steps.saturating_add(64), strict)? {
            Classification::Decided(first) => first,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let refined_second = match second.refined(steps, strict)? {
            Classification::Decided(second) => second,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let stalled = previous.as_ref().is_some_and(|(old_first, old_second)| {
            old_first == &refined_first && old_second == &refined_second
        });
        previous = Some((refined_first.clone(), refined_second.clone()));
        let restricted = predicate.substitute_affine(
            &(&refined_first.root().upper - &refined_first.root().lower),
            &refined_first.root().lower,
            &(&refined_second.root().upper - &refined_second.root().lower),
            &refined_second.root().lower,
        );
        if let Some(sign) = bivariate_unit_square_strict_bernstein_sign(&restricted, strict)? {
            return Ok(Classification::Decided(sign));
        }
        if !policy.has_bounded_exact_predicate_budget()
            && let Some(sign) = RealInterval::evaluate_bivariate_power_basis(
                predicate,
                &RealInterval {
                    lower: refined_first.root().lower.clone(),
                    upper: refined_first.root().upper.clone(),
                },
                &RealInterval {
                    lower: refined_second.root().lower.clone(),
                    upper: refined_second.root().upper.clone(),
                },
            )
            .and_then(|interval| interval.strict_nonzero_sign())
        {
            return Ok(Classification::Decided(sign));
        }
        // Replay equality before refining a zero product box. An uncertain
        // early replay retains the interval opportunity; a proved nonzero
        // value refines until its sign separates, even beyond 512 in STRICT.
        if !certified_nonzero && (steps == 0 || stalled || steps == 512) {
            match algebraic_selected_fiber_pair_projected_root_via_subresultants(
                first, second, predicate, None, policy,
            )? {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Decided(false) => certified_nonzero = true,
                Classification::Uncertain(_) if steps == 0 && !stalled => {}
                Classification::Uncertain(UncertaintyReason::Predicate)
                    if policy.permits_approximate_512() =>
                {
                    policy.observe_approximate_512();
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        steps = if steps == 0 {
            2
        } else {
            steps.checked_mul(2).ok_or_else(|| {
                CurveError::Topology("selected-fiber pair sign refinement overflow".into())
            })?
        };
    }
}

/// Rebinds an equation known to vanish at a selected fiber root.
///
/// Differentiation can introduce foreign critical roots inside the original
/// isolator even though that interval contains only one root of the old
/// equation. Narrow the trusted old singleton until the new equation also has
/// exactly one root there before publishing a new authority. All isolation is
/// STRICT because the result becomes construction evidence.
pub(super) fn selected_fiber_rebind_incidence(
    parameter: &BezierAlgebraicSelectedFiberParameter2,
    incidence: BivariatePolynomial,
    strict: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
    let retained = &parameter.data.authority.data.retained_parameter;
    if parameter.data.root.exact_root.is_some() {
        return Ok(Classification::Decided(
            BezierAlgebraicSelectedFiberAuthority2::new(incidence, retained.clone(), strict)
                .parameter(parameter.data.root.clone()),
        ));
    }
    for steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let trusted = match parameter.refined(steps, strict)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if trusted.data.root.exact_root.is_some() {
            return Ok(Classification::Decided(
                BezierAlgebraicSelectedFiberAuthority2::new(incidence, retained.clone(), strict)
                    .parameter(trusted.data.root.clone()),
            ));
        }
        let retained_refined = BezierParameter2::Algebraic(retained.clone())
            .refined_isolating_interval(steps.max(64), strict);
        let retained_root = match retained_refined {
            BezierParameter2::Algebraic(parameter) => parameter_representation(&parameter, strict),
            BezierParameter2::Exact(parameter) => {
                AlgebraicRootRepresentation::from_exact_value(&parameter)
            }
        };
        let report = isolate_bivariate_fiber_roots_at_algebraic_parameter_complete(
            &incidence,
            CurveResultantParameter::First,
            &retained_root,
            &trusted.data.root.lower,
            &trusted.data.root.upper,
            AlgebraicFiberRootIsolationConfig {
                max_subdivision_depth: 512,
                refinement_steps: 8,
            },
            hypersolve::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicFiberRootIsolationStatus::Isolated if report.intervals.len() == 1 => {
                return Ok(Classification::Decided(
                    BezierAlgebraicSelectedFiberAuthority2::new(
                        incidence,
                        retained.clone(),
                        strict,
                    )
                    .parameter(
                        report
                            .intervals
                            .into_iter()
                            .next()
                            .expect("a singleton deflated fiber has one isolator"),
                    ),
                ));
            }
            AlgebraicFiberRootIsolationStatus::Isolated => {}
            AlgebraicFiberRootIsolationStatus::NoRoots
            | AlgebraicFiberRootIsolationStatus::IdenticallyZeroFiber => {
                return Err(CurveError::Topology(
                    "a certified fiber root disappeared during local multiplicity deflation".into(),
                ));
            }
            AlgebraicFiberRootIsolationStatus::InvalidEvidence
            | AlgebraicFiberRootIsolationStatus::InvalidInterval => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            AlgebraicFiberRootIsolationStatus::DepthLimit
            | AlgebraicFiberRootIsolationStatus::Undecided => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Predicate))
}

/// Returns a local authority in which the retained fiber root is simple.
///
/// Repeated differentiation is local multiplicity deflation: the last
/// equation whose derivative is nonzero at the selected root still vanishes
/// there and crosses its isolator. Every identity test and authority rebind is
/// STRICT because this equation becomes construction evidence for a later
/// product-box proof.
pub(super) fn selected_fiber_simple_parameter(
    parameter: &BezierAlgebraicSelectedFiberParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>> {
    let strict = policy.strict_counterpart();
    let mut incidence = parameter.data.authority.data.incidence.clone();
    // Norm projections commonly duplicate every factor. Peeling an exact
    // bivariate square preserves the complete root set and avoids asking the
    // local Sturm engine to rediscover that structural multiplicity.
    while let Some(square_root) = bivariate_exact_square_root(&incidence) {
        let old_count = incidence
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default();
        let new_count = square_root
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default();
        if new_count >= old_count {
            break;
        }
        incidence = square_root;
    }
    let mut parameter = BezierAlgebraicSelectedFiberAuthority2::new(
        incidence,
        parameter.data.authority.data.retained_parameter.clone(),
        &strict,
    )
    .parameter(parameter.data.root.clone());
    loop {
        let derivative = bivariate_parameter_derivative(
            &parameter.data.authority.data.incidence,
            CurveResultantParameter::Second,
        );
        if matches!(bivariate_exact_nonzero_metadata(&derivative), Some(None)) {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        match parameter.predicate_sign(&derivative, &strict)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(parameter));
            }
            Classification::Decided(RealSign::Zero) => {
                parameter = match selected_fiber_rebind_incidence(&parameter, derivative, &strict)?
                {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
}

/// Certifies one root of a trivariate equation in the selected
/// `(alpha,u,v)` product isolator.
///
/// The caller supplies locally simple source and image authorities; the base
/// is reduced here as the remaining simple constraint. Strict opposite signs
/// on all three coordinate-face pairs then give a Poincare--Miranda existence
/// proof. A strict Bernstein sign on the complete box instead proves the
/// selected tuple is not a root.
pub(super) fn algebraic_selected_fiber_pair_trivariate_root(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    incidence: &TrivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let strict = *policy;
    let retained = &source.data.authority.data.retained_parameter;
    validate_selected_fiber_pair_base(source, image)?;
    let base = match selected_parameter_simple_constraint(
        &BezierParameter2::Algebraic(retained.clone()),
        &strict,
    )? {
        Classification::Decided(base) => base,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_incidence = source.data.authority.data.incidence.clone();
    let mut previous = None;
    let mut steps = 0_usize;
    let next_steps = |steps: usize| {
        if steps == 0 {
            Ok(2)
        } else {
            steps.checked_mul(2).ok_or_else(|| {
                CurveError::Topology("selected-fiber root correlation overflow".into())
            })
        }
    };
    loop {
        // After the co-refined hot attempt, grow a triangular enclosure: the
        // base box becomes narrower than the source box, which becomes
        // narrower than the image box. Equal base/source refinements can make
        // an exact relation such as `u-alpha` touch zero at box corners
        // forever. Growing both gaps handles arbitrarily steep branches.
        let source_steps = steps.saturating_mul(2).saturating_add(64);
        let alpha_steps = source_steps.saturating_add(steps);
        let alpha = BezierParameter2::Algebraic(retained.clone())
            .refined_isolating_interval(alpha_steps, &strict);
        let BezierParameter2::Algebraic(alpha) = alpha else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let source = match source.refined(source_steps, &strict)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let image = match image.refined(steps, &strict)? {
            Classification::Decided(image) => image,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if previous
            .as_ref()
            .is_some_and(|(old_alpha, old_source, old_image)| {
                old_alpha == &alpha && old_source == &source && old_image == &image
            })
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        previous = Some((alpha.clone(), source.clone(), image.clone()));

        let defining_lower = Real::eval_poly(&base, alpha.interval().start());
        let defining_upper = Real::eval_poly(&base, alpha.interval().end());
        if !strict_signs_are_opposite(
            real_sign(&defining_lower, &strict),
            real_sign(&defining_upper, &strict),
        ) {
            steps = next_steps(steps)?;
            continue;
        }
        let restricted_source = source_incidence.substitute_affine(
            &(alpha.interval().end() - alpha.interval().start()),
            alpha.interval().start(),
            &(&source.root().upper - &source.root().lower),
            &source.root().lower,
        );
        let source_lower = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_second(&restricted_source, &Real::zero()),
            &strict,
        )?;
        let source_upper = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_second(&restricted_source, &Real::one()),
            &strict,
        )?;
        if !strict_signs_are_opposite(source_lower, source_upper) {
            steps = next_steps(steps)?;
            continue;
        }

        let restricted = trivariate_restrict_to_box_bounds(
            incidence,
            [
                (alpha.interval().start(), alpha.interval().end()),
                (&source.root().lower, &source.root().upper),
                (&image.root().lower, &image.root().upper),
            ],
        );
        if trivariate_unit_cube_strict_bernstein_sign(restricted.clone(), &strict)?.is_some() {
            return Ok(Classification::Decided(false));
        }
        let Some((coefficients, [0, 1])) = trivariate_axis_bivariate_coefficients(&restricted, 2)
        else {
            steps = next_steps(steps)?;
            continue;
        };
        let Some(lower) = coefficients.first() else {
            steps = next_steps(steps)?;
            continue;
        };
        let upper = coefficients
            .iter()
            .skip(1)
            .fold(lower.clone(), |sum, coefficient| {
                bivariate_add(&sum, coefficient)
            });
        let face_sign = |polynomial: &BivariatePolynomial| -> CurveResult<Option<RealSign>> {
            if let Some(sign) = bivariate_unit_square_strict_bernstein_sign(polynomial, &strict)? {
                return Ok(Some(sign));
            }
            let unit = RealInterval {
                lower: Real::zero(),
                upper: Real::one(),
            };
            Ok(
                RealInterval::evaluate_bivariate_power_basis(polynomial, &unit, &unit)
                    .and_then(|interval| interval.strict_nonzero_sign()),
            )
        };
        let lower = face_sign(lower)?;
        let upper = face_sign(&upper)?;
        if strict_signs_are_opposite(lower, upper) {
            return Ok(Classification::Decided(true));
        }
        steps = next_steps(steps)?;
    }
}

/// Correlates a projected image root without promoting either selected fiber.
///
/// The image constraint is locally deflated to a simple root. Exact
/// trivariate subresultants of that constraint and the authored incidence then
/// recover their first nonzero GCD over the selected `(alpha,u)` field. Since
/// that GCD divides the simple image constraint, the product-box proof above
/// decides both tangent membership and separation.
pub(super) fn algebraic_selected_fiber_pair_projected_root_via_subresultants(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    projected_incidence: &BivariatePolynomial,
    known_incidence: Option<&BivariatePolynomial>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    policy.strict_predicate_pass(|| {
        algebraic_selected_fiber_pair_projected_root_via_subresultants_strict(
            source,
            image,
            projected_incidence,
            known_incidence,
            policy,
        )
    })
}

pub(super) fn algebraic_selected_fiber_pair_projected_root_via_subresultants_strict(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    projected_incidence: &BivariatePolynomial,
    known_incidence: Option<&BivariatePolynomial>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let strict = *policy;
    validate_selected_fiber_pair_base(source, image)?;
    let source_result = selected_fiber_simple_parameter(source, &strict)?;
    let source = match source_result {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let image_result = selected_fiber_simple_parameter(image, &strict)?;
    let image = match image_result {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (image_incidence, axes) = match known_incidence {
        Some(incidence) => (incidence, [1, 2]),
        None => (&image.data.authority.data.incidence, [0, 2]),
    };
    let Some(mut image_incidence) = trivariate_from_bivariate_axes(image_incidence, axes) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(mut projected_incidence) = trivariate_from_bivariate_axes(projected_incidence, [1, 2])
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let specialize_degree = |polynomial: &TrivariatePolynomial|
     -> CurveResult<Classification<Option<TrivariatePolynomial>>> {
        let Some((mut coefficients, [0, 1])) =
            trivariate_axis_bivariate_coefficients(polynomial, 2)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        while coefficients.len() > 1 {
            match source.predicate_sign(
                coefficients
                    .last()
                    .expect("a nonempty coefficient list has a last value"),
                &strict,
            )? {
                Classification::Decided(RealSign::Zero) => {
                    coefficients.pop();
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => break,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        match source.predicate_sign(
            coefficients
                .first()
                .expect("a retained coefficient list is nonempty"),
            &strict,
        )? {
            Classification::Decided(RealSign::Zero) if coefficients.len() == 1 => {
                Ok(Classification::Decided(None))
            }
            Classification::Decided(_) => Ok(Classification::Decided(
                trivariate_from_axis_bivariate_coefficients(&coefficients, 2, [0, 1]),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    };
    image_incidence = match specialize_degree(&image_incidence)? {
        Classification::Decided(Some(incidence)) => incidence,
        Classification::Decided(None) => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    projected_incidence = match specialize_degree(&projected_incidence)? {
        Classification::Decided(Some(incidence)) => incidence,
        Classification::Decided(None) => return Ok(Classification::Decided(true)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };

    let first_degree = image_incidence.dimensions().2.saturating_sub(1);
    let second_degree = projected_incidence.dimensions().2.saturating_sub(1);
    let maximum_order = first_degree.min(second_degree);
    if maximum_order == 0 {
        // The simple selected-image equation is nonconstant. A specialized
        // constant authored equation was already certified nonzero above, so
        // it cannot vanish at the selected pair.
        return Ok(Classification::Decided(false));
    }
    let Some((image_coefficients, [0, 1])) =
        trivariate_axis_bivariate_coefficients(&image_incidence, 2)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some((projected_coefficients, [0, 1])) =
        trivariate_axis_bivariate_coefficients(&projected_incidence, 2)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let retained_root =
        parameter_representation(&source.data.authority.data.retained_parameter, &strict);
    for order in 0..=maximum_order {
        let coefficients = match hypersolve::subresultant_in_algebraic_fiber(
            &image_coefficients,
            &projected_coefficients,
            order,
            &source.data.authority.data.incidence,
            &retained_root,
        ) {
            Ok(coefficients) => coefficients,
            Err(hypersolve::AlgebraicFiberSubresultantError::Undecided) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Err(hypersolve::AlgebraicFiberSubresultantError::InvalidEvidence) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        };
        let mut any_nonzero = false;
        let mut uncertainty = None;
        for coefficient in &coefficients {
            let coefficient_sign = source.predicate_sign(coefficient, &strict)?;
            match coefficient_sign {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    any_nonzero = true;
                    break;
                }
                Classification::Uncertain(reason) => uncertainty = Some(reason),
            }
        }
        if order == 0 {
            if any_nonzero {
                return Ok(Classification::Decided(false));
            }
            if let Some(reason) = uncertainty {
                return Ok(Classification::Uncertain(reason));
            }
            continue;
        }
        if !any_nonzero {
            if let Some(reason) = uncertainty {
                return Ok(Classification::Uncertain(reason));
            }
            continue;
        }
        let Some(gcd) = trivariate_from_axis_bivariate_coefficients(&coefficients, 2, [0, 1])
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-fiber-pair-correlation",
            "local-subresultant-gcd",
        );
        return algebraic_selected_fiber_pair_trivariate_root(&source, &image, &gcd, &strict);
    }
    Err(CurveError::Topology(
        "a selected-fiber subresultant sequence lost its nonzero terminal polynomial".into(),
    ))
}

/// Correlates one projected image root with the particular selected source
/// fiber root that authored it.
///
/// The square system `P(alpha)=0`, `F(alpha,u)=0`, `G(u,v)=0` receives a
/// division-free Poincare--Miranda certificate whenever the contact is
/// transverse.  Since `v` is already a singleton root of the exact local norm
/// of `G`, any `G` root in its box is that selected image. Multiple roots use
/// a local trivariate subresultant GCD before the complete predicate fallback.
pub(super) fn algebraic_selected_fiber_pair_projected_root(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    projected_incidence: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    source.validate_policy(policy)?;
    image.validate_policy(policy)?;
    if policy.permits_approximate_512() {
        match policy.strict_predicate_pass(|| {
            algebraic_selected_fiber_pair_projected_root(source, image, projected_incidence, policy)
        })? {
            decided @ Classification::Decided(_) => return Ok(decided),
            Classification::Uncertain(_) => {}
        }
    }
    if source.represented_value().is_some() || image.represented_value().is_some() {
        return Ok(algebraic_selected_fiber_pair_predicate_sign(
            source,
            image,
            projected_incidence,
            policy,
        )?
        .map(|sign| sign == RealSign::Zero));
    }
    validate_selected_fiber_pair_base(source, image)?;

    if !policy.permits_approximate_512() {
        let strict = policy;
        let retained = source.data.authority.data.retained_parameter.clone();
        let box_source = match selected_fiber_simple_parameter(source, strict)? {
            Classification::Decided(parameter) => Some(parameter),
            Classification::Uncertain(_) => None,
        };
        let box_image = match selected_fiber_simple_parameter(image, strict)? {
            Classification::Decided(parameter) => Some(parameter),
            Classification::Uncertain(_) => None,
        };
        let mut previous = None;
        // This is only the transverse hot path. Tangencies and severely
        // conditioned boxes continue through the exact local subresultant
        // authority below, so do not spend the terminal predicate schedule
        // trying to turn an even contact into an opposite-face certificate.
        for steps in [0_usize, 2, 4, 8, 16, 32] {
            let (Some(box_source), Some(box_image)) = (&box_source, &box_image) else {
                break;
            };
            let source_steps = steps.saturating_mul(2).saturating_add(64);
            let alpha_steps = source_steps.saturating_add(steps);
            let alpha = BezierParameter2::Algebraic(retained.clone())
                .refined_isolating_interval(alpha_steps, strict);
            let BezierParameter2::Algebraic(alpha) = alpha else {
                break;
            };
            let source = match box_source.refined(source_steps, strict)? {
                Classification::Decided(source) => source,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let image = match box_image.refined(steps, strict)? {
                Classification::Decided(image) => image,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if previous
                .as_ref()
                .is_some_and(|(old_alpha, old_source, old_image)| {
                    old_alpha == &alpha && old_source == &source && old_image == &image
                })
            {
                break;
            }
            previous = Some((alpha.clone(), source.clone(), image.clone()));

            let restricted_incidence = projected_incidence.substitute_affine(
                &(&source.root().upper - &source.root().lower),
                &source.root().lower,
                &(&image.root().upper - &image.root().lower),
                &image.root().lower,
            );
            if bivariate_unit_square_strict_bernstein_sign(&restricted_incidence, strict)?.is_some()
            {
                return Ok(Classification::Decided(false));
            }

            if RealInterval::evaluate_bivariate_power_basis(
                projected_incidence,
                &RealInterval {
                    lower: source.root().lower.clone(),
                    upper: source.root().upper.clone(),
                },
                &RealInterval {
                    lower: image.root().lower.clone(),
                    upper: image.root().upper.clone(),
                },
            )
            .and_then(|interval| interval.strict_nonzero_sign())
            .is_some()
            {
                return Ok(Classification::Decided(false));
            }

            let defining_lower =
                Real::eval_poly(alpha.polynomial().coefficients(), alpha.interval().start());
            let defining_upper =
                Real::eval_poly(alpha.polynomial().coefficients(), alpha.interval().end());
            if !strict_signs_are_opposite(
                real_sign(&defining_lower, strict),
                real_sign(&defining_upper, strict),
            ) {
                continue;
            }

            let source_incidence = source.data.authority.data.incidence.substitute_affine(
                &(alpha.interval().end() - alpha.interval().start()),
                alpha.interval().start(),
                &(&source.root().upper - &source.root().lower),
                &source.root().lower,
            );
            let source_lower = univariate_unit_interval_strict_bernstein_sign(
                &bivariate_specialize_second(&source_incidence, &Real::zero()),
                strict,
            )?;
            let source_upper = univariate_unit_interval_strict_bernstein_sign(
                &bivariate_specialize_second(&source_incidence, &Real::one()),
                strict,
            )?;
            if !strict_signs_are_opposite(source_lower, source_upper) {
                continue;
            }

            let image_lower = univariate_unit_interval_strict_bernstein_sign(
                &bivariate_specialize_second(&restricted_incidence, &Real::zero()),
                strict,
            )?;
            let image_upper = univariate_unit_interval_strict_bernstein_sign(
                &bivariate_specialize_second(&restricted_incidence, &Real::one()),
                strict,
            )?;
            if strict_signs_are_opposite(image_lower, image_upper) {
                return Ok(Classification::Decided(true));
            }
        }
    }
    match algebraic_selected_fiber_pair_projected_root_via_subresultants(
        source,
        image,
        projected_incidence,
        None,
        policy,
    )? {
        decided @ Classification::Decided(_) => Ok(decided),
        Classification::Uncertain(UncertaintyReason::Predicate)
            if policy.permits_approximate_512() =>
        {
            policy.observe_approximate_512();
            Ok(Classification::Decided(true))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn algebraic_selected_fiber_pair_square_root_sum_sign(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    projected_magnitude_zero: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let sign = |polynomial: &BivariatePolynomial| {
        algebraic_selected_fiber_pair_predicate_sign(source, image, polynomial, policy)
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    if projected_magnitude_zero {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    let magnitude = bivariate_subtract(
        &bivariate_multiply(&expression.rational, &expression.rational),
        &bivariate_multiply(
            &bivariate_multiply(&expression.radical, &expression.radical),
            radicand,
        ),
    );
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

pub(super) fn collapse_two_normal_polynomial_speeds(
    expression: &BezierParallelTwoNormalExpression2,
    source_speed: &[Real],
    image_speed: &[Real],
) -> BivariatePolynomial {
    let source_speed = bivariate_outer_product(source_speed, &[Real::one()]);
    let image_speed = bivariate_outer_product(&[Real::one()], image_speed);
    let speed_product = bivariate_multiply(&source_speed, &image_speed);
    bivariate_add(
        &bivariate_add(
            &bivariate_multiply(&expression.product, &speed_product),
            &bivariate_multiply(&expression.center, &source_speed),
        ),
        &bivariate_add(
            &bivariate_multiply(&expression.candidate, &image_speed),
            &expression.rational,
        ),
    )
}

pub(super) fn positive_polynomial_speed_at(
    speed_squared: &[Real],
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut speed = match polynomial_square_root(speed_squared, policy)? {
        Classification::Decided(Some(speed)) => speed,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    // A polynomial square root has either sign. Select the positive speed
    // at the actual retained parameter under its original policy. A caller
    // using this throughout a cell separately proves that no speed zero
    // separates its parameters.
    match policy.strict_predicate_pass(|| parameter.polynomial_sign(&speed, policy))? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Negative) => {
            speed = polynomial_scale(&speed, &Real::from(-1_i8))
        }
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    Ok(Classification::Decided(Some(speed)))
}

pub(super) fn algebraic_selected_fiber_pair_two_normal_sum_sign(
    source: &BezierAlgebraicSelectedFiberParameter2,
    image: &BezierAlgebraicSelectedFiberParameter2,
    expression: &BezierParallelTwoNormalExpression2,
    source_speed_squared: &BivariatePolynomial,
    image_speed_squared: &BivariatePolynomial,
    projected_root_certified: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let source_speed = bivariate_specialize_second(source_speed_squared, &Real::zero());
    let image_speed = bivariate_specialize_first(image_speed_squared, &Real::zero());
    if *source_speed_squared == bivariate_outer_product(&source_speed, &[Real::one()])
        && *image_speed_squared == bivariate_outer_product(&[Real::one()], &image_speed)
        && let (
            Classification::Decided(Some(source_speed)),
            Classification::Decided(Some(image_speed)),
        ) = (
            positive_polynomial_speed_at(
                &source_speed,
                &CurveParameter2::from_selected_fiber(source.clone()),
                policy,
            )?,
            positive_polynomial_speed_at(
                &image_speed,
                &CurveParameter2::from_selected_fiber(image.clone()),
                policy,
            )?,
        )
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-fiber-two-normal-sign",
            "polynomial-speed-collapse",
        );
        let collapsed =
            collapse_two_normal_polynomial_speeds(expression, &source_speed, &image_speed);
        return algebraic_selected_fiber_pair_predicate_sign(source, image, &collapsed, policy);
    }
    let square_root_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2,
                            projected_magnitude_zero| {
        algebraic_selected_fiber_pair_square_root_sum_sign(
            source,
            image,
            expression,
            source_speed_squared,
            projected_magnitude_zero,
            policy,
        )
    };
    let source_term = BezierAlgebraicCuspTwoTermExpression2 {
        rational: expression.rational.clone(),
        radical: expression.center.clone(),
    };
    let image_term = BezierAlgebraicCuspTwoTermExpression2 {
        rational: expression.candidate.clone(),
        radical: expression.product.clone(),
    };
    let source_sign = match square_root_sign(&source_term, false)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let image_sign = match square_root_sign(&image_term, false)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (source_sign, image_sign) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let source_square = bivariate_add(
        &bivariate_multiply(&expression.rational, &expression.rational),
        &bivariate_multiply(
            &bivariate_multiply(&expression.center, &expression.center),
            source_speed_squared,
        ),
    );
    let image_square = bivariate_multiply(
        image_speed_squared,
        &bivariate_add(
            &bivariate_multiply(&expression.candidate, &expression.candidate),
            &bivariate_multiply(
                &bivariate_multiply(&expression.product, &expression.product),
                source_speed_squared,
            ),
        ),
    );
    let magnitude = BezierAlgebraicCuspTwoTermExpression2 {
        rational: bivariate_subtract(&source_square, &image_square),
        radical: bivariate_scale(
            bivariate_subtract(
                &bivariate_multiply(&expression.rational, &expression.center),
                &bivariate_multiply(
                    image_speed_squared,
                    &bivariate_multiply(&expression.candidate, &expression.product),
                ),
            ),
            &Real::from(2_u8),
        ),
    };
    Ok(
        match square_root_sign(&magnitude, projected_root_certified)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(source_sign),
            Classification::Decided(RealSign::Negative) => Classification::Decided(image_sign),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn algebraic_selected_fiber_root_radical_sum_sign(
    authority: &BezierAlgebraicSelectedFiberAuthority2,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    speed_squared: &BivariatePolynomial,
    root: &IsolatedRootInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let incidence = &authority.data.incidence;
    let retained = &authority.data.retained_parameter;
    let sign = |polynomial: &BivariatePolynomial| {
        algebraic_selected_fiber_root_predicate_sign(authority, polynomial, root, policy)
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    let magnitude = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&expression.rational, &expression.rational),
            speed_squared,
        ),
        &bivariate_multiply(&expression.radical, &expression.radical),
    );
    let magnitude = match bivariate_reduce_axis(
        &magnitude,
        retained.polynomial(),
        CurveResultantParameter::First,
        policy,
    )? {
        Classification::Decided(magnitude) => magnitude,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if magnitude == *incidence {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// Signs `A + B*sqrt(S)` at one root retained in `Q(alpha)`.
///
/// This is the positive-speed counterpart of
/// [`algebraic_selected_fiber_root_radical_sum_sign`], whose expression uses
/// `B/sqrt(S)`. Both keep equality on the local selected-fiber authority and
/// therefore share the same STRICT/APPROXIMATE_512 terminal.
pub(super) fn algebraic_selected_fiber_root_square_root_sum_sign(
    authority: &BezierAlgebraicSelectedFiberAuthority2,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    root: &IsolatedRootInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let incidence = &authority.data.incidence;
    let retained = &authority.data.retained_parameter;
    let sign = |polynomial: &BivariatePolynomial| {
        algebraic_selected_fiber_root_predicate_sign(authority, polynomial, root, policy)
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    let magnitude = bivariate_subtract(
        &bivariate_multiply(&expression.rational, &expression.rational),
        &bivariate_multiply(
            &bivariate_multiply(&expression.radical, &expression.radical),
            radicand,
        ),
    );
    let magnitude = match bivariate_reduce_axis(
        &magnitude,
        retained.polynomial(),
        CurveResultantParameter::First,
        policy,
    )? {
        Classification::Decided(magnitude) => magnitude,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if magnitude == *incidence {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// Signs `A*sqrt(S*T)+B*sqrt(S)+C*sqrt(T)+E` at one selected-fiber root.
///
/// Grouping the expression as `L + M*sqrt(T)`, with
/// `L=E+B*sqrt(S)` and `M=C+A*sqrt(S)`, needs only three calls to the
/// one-radical signer. Opposite term signs are distinguished by the exact
/// local predicate `L^2-M^2*T`; no primitive element or global norm is built.
pub(super) fn algebraic_selected_fiber_root_two_normal_sum_sign(
    authority: &BezierAlgebraicSelectedFiberAuthority2,
    expression: &BezierParallelTwoNormalExpression2,
    center_speed_squared: &BivariatePolynomial,
    candidate_speed_squared: &BivariatePolynomial,
    root: &IsolatedRootInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let square_root_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
        algebraic_selected_fiber_root_square_root_sum_sign(
            authority,
            expression,
            center_speed_squared,
            root,
            policy,
        )
    };
    let center_term = BezierAlgebraicCuspTwoTermExpression2 {
        rational: expression.rational.clone(),
        radical: expression.center.clone(),
    };
    let candidate_term = BezierAlgebraicCuspTwoTermExpression2 {
        rational: expression.candidate.clone(),
        radical: expression.product.clone(),
    };
    let center_sign = match square_root_sign(&center_term)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidate_sign = match square_root_sign(&candidate_term)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (center_sign, candidate_sign) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let center_square = bivariate_add(
        &bivariate_multiply(&expression.rational, &expression.rational),
        &bivariate_multiply(
            &bivariate_multiply(&expression.center, &expression.center),
            center_speed_squared,
        ),
    );
    let candidate_square = bivariate_multiply(
        candidate_speed_squared,
        &bivariate_add(
            &bivariate_multiply(&expression.candidate, &expression.candidate),
            &bivariate_multiply(
                &bivariate_multiply(&expression.product, &expression.product),
                center_speed_squared,
            ),
        ),
    );
    let magnitude = BezierAlgebraicCuspTwoTermExpression2 {
        rational: bivariate_subtract(&center_square, &candidate_square),
        radical: bivariate_scale(
            bivariate_subtract(
                &bivariate_multiply(&expression.rational, &expression.center),
                &bivariate_multiply(
                    candidate_speed_squared,
                    &bivariate_multiply(&expression.candidate, &expression.product),
                ),
            ),
            &Real::from(2_u8),
        ),
    };
    Ok(match square_root_sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(center_sign),
        Classification::Decided(RealSign::Negative) => Classification::Decided(candidate_sign),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

pub(super) fn algebraic_cusp_correlated_radical_sum_sign(
    incidence: &BivariatePolynomial,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    speed_squared: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let sign = |polynomial: &BivariatePolynomial| {
        algebraic_selected_correlated_predicate_sign(
            incidence,
            polynomial,
            cusp_parameter,
            other_parameter,
            policy,
        )
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let magnitude = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&expression.rational, &expression.rational),
            speed_squared,
        ),
        &bivariate_multiply(&expression.radical, &expression.radical),
    );
    let magnitude = if let BezierParameter2::Algebraic(cusp) = cusp_parameter {
        match bivariate_reduce_axis(
            &magnitude,
            cusp.polynomial(),
            CurveResultantParameter::First,
            policy,
        )? {
            Classification::Decided(magnitude) => magnitude,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else {
        magnitude
    };
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// Signs `A + B/sqrt(S)` for independently authored parameter evidence.
///
/// Unlike the correlated variant, this helper never treats a squared support
/// equation as a relation selecting the second parameter. It is used by
/// positive-dimensional circle replay at authored range endpoints, where the
/// support equation is identically zero and therefore carries no root identity.
pub(super) fn algebraic_cusp_independent_radical_sum_sign(
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    speed_squared: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let sign = |polynomial: &BivariatePolynomial| {
        signed_bivariate_at_parameter_pair(polynomial, cusp_parameter, other_parameter, policy)
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let magnitude = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&expression.rational, &expression.rational),
            speed_squared,
        ),
        &bivariate_multiply(&expression.radical, &expression.radical),
    );
    let magnitude = if let BezierParameter2::Algebraic(cusp) = cusp_parameter {
        match bivariate_reduce_axis(
            &magnitude,
            cusp.polynomial(),
            CurveResultantParameter::First,
            policy,
        )? {
            Classification::Decided(magnitude) => magnitude,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else {
        magnitude
    };
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}
