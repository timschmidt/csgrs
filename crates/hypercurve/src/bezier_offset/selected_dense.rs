//! Projection of dense selected-tuple polynomials onto their last axis
//! and isolation of the resulting candidate parameters.

use super::*;

/// Eliminates every already-selected source axis and retains the exact
/// univariate projection on the final axis.
pub(super) fn selected_dense_last_axis_projection(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> CurveResult<Classification<DenseTensorPolynomial>> {
    if polynomial.dimensions().len() != sources.len() + 1 {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let mut projection = polynomial.clone();
    for (source_index, source) in sources.iter().enumerate() {
        let Some(reduced) = projection.reduce_axis_modulo(
            0,
            &source.polynomial_coefficients,
            hypersolve::PredicatePolicy::STRICT,
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        projection = reduced.compact_coefficients();
        // Quotient-ring reduction preserves the constraint's nominal axis
        // width even when every positive-power coefficient cancels. Certify
        // and remove such an axis before constructing a resultant; recursive
        // curve fields frequently retain several selected roots that the
        // final sparse incidence no longer uses.
        if let Some(reduced) = projection.remove_certified_independent_axis(
            0,
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        ) {
            projection = reduced;
            continue;
        }
        if projection.dimensions()[0] == 1 {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let report = hypersolve::resultant_tensor_polynomial_univariate_constraint(
            &projection,
            &source.polynomial_coefficients,
            0,
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        );
        projection = match report.status {
            hypersolve::TensorConstraintResultantStatus::Constructed => report
                .resultant
                .expect("a constructed tensor projection retains its polynomial"),
            hypersolve::TensorConstraintResultantStatus::UndecidedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            hypersolve::TensorConstraintResultantStatus::InvalidAxis
            | hypersolve::TensorConstraintResultantStatus::InvalidConstraint
            | hypersolve::TensorConstraintResultantStatus::DimensionOverflow
            | hypersolve::TensorConstraintResultantStatus::AllocationFailed
            | hypersolve::TensorConstraintResultantStatus::ResultantError
            | hypersolve::TensorConstraintResultantStatus::InterpolationDivisionFailed => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        for (axis, retained_source) in sources[source_index + 1..].iter().enumerate() {
            let Some(reduced) = projection.reduce_axis_modulo(
                axis,
                &retained_source.polynomial_coefficients,
                hypersolve::PredicatePolicy::STRICT,
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            projection = reduced;
        }
    }
    if projection.dimensions().len() != 1 {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    Ok(Classification::Decided(projection))
}

/// Certifies that one projected last-axis candidate belongs to the selected
/// represented source tuple rather than to conjugates introduced by
/// sequential resultants.
///
/// The source defining polynomials occupy their own coordinate axes. On a
/// refined product isolator they already have opposite signs on their two
/// corresponding faces. If the authored tensor also has uniform opposite
/// signs on the candidate's two parameter faces, Poincare--Miranda proves a
/// common zero in that box. Every box isolates exactly one root on each axis,
/// so that zero is necessarily the authored tuple. Uniform equal face signs
/// together with a strictly signed parameter derivative prove that the
/// conjugate candidate has no authored zero. Multiple/even roots simply
/// decline both certificates and retain the complete exact sign fallback.
#[derive(Clone, Debug)]
pub(super) struct BezierDenseSelectedCandidateBox2 {
    pub(super) sources: Vec<AlgebraicRootRepresentation>,
    pub(super) candidate: AlgebraicRootRepresentation,
}

#[derive(Clone, Debug)]
pub(super) enum BezierDenseCandidateBoxIncidence2 {
    Root(BezierDenseSelectedCandidateBox2),
    Disjoint(BezierDenseSelectedCandidateBox2),
}

pub(super) fn projected_selected_dense_candidate_box_incidence(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    candidate: &BezierParameter2,
    maximum_steps: usize,
    source_lead: usize,
) -> Option<BezierDenseCandidateBoxIncidence2> {
    if polynomial.dimensions().len() != sources.len() + 1 {
        return None;
    }
    let BezierParameter2::Algebraic(_) = candidate else {
        return None;
    };
    let derivative = dense_last_axis_derivative(polynomial)?;
    let strict = &CurveContext::STRICT;
    let retained_parameter = |root: &AlgebraicRootRepresentation| {
        match BezierParameter2::from_algebraic_root_representation_unbounded(root, strict).ok()? {
            Classification::Decided(parameter) => Some(parameter),
            Classification::Uncertain(_) => None,
        }
    };
    let source_parameters = sources
        .iter()
        .map(retained_parameter)
        .collect::<Option<Vec<_>>>()?;
    let mut source_refinements = source_parameters
        .iter()
        .map(|source| BezierParameterRefinement2::new(source, strict))
        .collect::<Vec<_>>();
    let mut candidate_refinement = BezierParameterRefinement2::new(candidate, strict);
    let square_free_defining = |root: &AlgebraicRootRepresentation| {
        hypersolve::square_free_part(
            root.polynomial_coefficients.clone(),
            hypersolve::PredicatePolicy::STRICT,
        )
        .unwrap_or_else(|| root.polynomial_coefficients.clone())
    };
    let source_defining = sources.iter().map(square_free_defining).collect::<Vec<_>>();
    let defining_face_signs = |root: &AlgebraicRootRepresentation, coefficients: &[Real]| {
        if root.interval.lower == root.interval.upper {
            return None;
        }
        let lower = real_sign(
            &Real::eval_poly(coefficients, &root.interval.lower),
            &CurveContext::STRICT,
        )?;
        let upper = real_sign(
            &Real::eval_poly(coefficients, &root.interval.upper),
            &CurveContext::STRICT,
        )?;
        matches!(
            (lower, upper),
            (RealSign::Negative, RealSign::Positive) | (RealSign::Positive, RealSign::Negative)
        )
        .then_some((lower, upper))
    };

    let mut previous = None;
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        if target_steps > maximum_steps {
            break;
        }
        let source_steps = target_steps.saturating_add(source_lead);
        let refined_sources = source_refinements
            .iter_mut()
            .map(|source| bezier_parameter_root_representation(source.refine_to(source_steps)))
            .collect::<Vec<_>>();
        let refined_candidate =
            bezier_parameter_root_representation(candidate_refinement.refine_to(target_steps));
        if previous
            .as_ref()
            .is_some_and(|(old_sources, old_candidate)| {
                old_sources == &refined_sources && old_candidate == &refined_candidate
            })
        {
            break;
        }
        previous = Some((refined_sources.clone(), refined_candidate.clone()));
        let source_defining_ok = !refined_sources
            .iter()
            .zip(&source_defining)
            .any(|(source, defining)| defining_face_signs(source, defining).is_none());
        let (Some(lower), Some(upper)) = (
            dense_specialize_last_axis(polynomial, &refined_candidate.interval.lower),
            dense_specialize_last_axis(polynomial, &refined_candidate.interval.upper),
        ) else {
            return None;
        };
        let coefficient_bits = source_steps.max(64).min(i32::MAX as usize) as i32;
        let face_sign = |face: &DenseTensorPolynomial| {
            dense_tensor_interval_with_coefficient_precision(
                face,
                &refined_sources,
                Some(-coefficient_bits),
            )
            .as_ref()
            .and_then(dense_strict_interval_sign)
        };
        let face_signs = (face_sign(&lower), face_sign(&upper));
        if matches!(
            face_signs,
            (Some(RealSign::Negative), Some(RealSign::Positive))
                | (Some(RealSign::Positive), Some(RealSign::Negative))
        ) && source_defining_ok
        {
            return Some(BezierDenseCandidateBoxIncidence2::Root(
                BezierDenseSelectedCandidateBox2 {
                    sources: refined_sources,
                    candidate: refined_candidate,
                },
            ));
        }
        let mut selected = refined_sources.clone();
        selected.push(refined_candidate.clone());
        let selected_sign = dense_polynomial_value_interval_with_coefficient_precision(
            polynomial,
            &selected,
            -coefficient_bits,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign);
        if let Some(sign) = selected_sign {
            let certificate = BezierDenseSelectedCandidateBox2 {
                sources: refined_sources,
                candidate: refined_candidate,
            };
            return Some(if sign == RealSign::Zero {
                BezierDenseCandidateBoxIncidence2::Root(certificate)
            } else {
                BezierDenseCandidateBoxIncidence2::Disjoint(certificate)
            });
        }
        if matches!(
            face_signs,
            (Some(RealSign::Negative), Some(RealSign::Negative))
                | (Some(RealSign::Positive), Some(RealSign::Positive))
        ) && dense_polynomial_value_interval_with_coefficient_precision(
            &derivative,
            &selected,
            -coefficient_bits,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign)
        .is_some_and(|sign| sign != RealSign::Zero)
        {
            return Some(BezierDenseCandidateBoxIncidence2::Disjoint(
                BezierDenseSelectedCandidateBox2 {
                    sources: refined_sources,
                    candidate: refined_candidate,
                },
            ));
        }
    }
    None
}

#[derive(Debug)]
pub(super) enum BezierSelectedDenseLastAxisUnivariate2 {
    IdenticallyZero,
    Empty,
    Polynomial {
        polynomial: BezierParameterPolynomial,
        square_free: bool,
    },
}

#[derive(Debug)]
pub(super) struct BezierRepresentedCenterParallelSchedule2 {
    pub(super) univariate: BezierSelectedDenseLastAxisUnivariate2,
    pub(super) unit_interval: OnceLock<BezierAlgebraicFiberProjection2>,
}

/// Eliminates every already-selected source axis into one reusable exact
/// univariate authority. Isolation domains must not repeat this projection or
/// its potentially expensive square-free normalization.
pub(super) fn selected_dense_last_axis_univariate(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<BezierSelectedDenseLastAxisUnivariate2>> {
    let projection = match selected_dense_last_axis_projection(polynomial, sources)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut coefficients = projection
        .coefficients()
        .iter()
        .cloned()
        .map(|coefficient| {
            coefficient
                .exact_rational_normal_form()
                .map(Real::new)
                .unwrap_or(coefficient)
        })
        .collect::<Vec<_>>();
    let exact_rational_zero = coefficients
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()
        .map(|coefficients| {
            coefficients
                .into_iter()
                .all(|coefficient| coefficient.is_zero())
        });
    let structural_zero = if let Some(zero) = exact_rational_zero {
        Classification::Decided(zero)
    } else {
        polynomial_coefficients_are_identically_zero(&coefficients, policy)
    };
    match structural_zero {
        Classification::Decided(true) => {
            match selected_dense_last_axis_active_degree(polynomial, sources, policy)? {
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        BezierSelectedDenseLastAxisUnivariate2::IdenticallyZero,
                    ));
                }
                Classification::Decided(Some(0)) => {
                    return Ok(Classification::Decided(
                        BezierSelectedDenseLastAxisUnivariate2::Empty,
                    ));
                }
                Classification::Decided(Some(_)) => {
                    // A foreign conjugate tuple owns a target-wide component,
                    // but the authored selected tuple has a finite nonzero
                    // fiber. Tagging every source tuple turns those complete
                    // components into retained-axis content; Hypersolve
                    // saturates that content exactly before projecting the
                    // target. This is the single rank-independent completion
                    // path for every dense selected system.
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!(
                            "tagged tensor projection begin dimensions={:?} sources={}",
                            polynomial.dimensions(),
                            sources.len()
                        );
                    }
                    let tagged = policy.strict_predicate_pass(|| {
                        project_selected_tensor_fiber_via_tagged_norm(polynomial, sources)
                    });
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("tagged tensor projection end status={:?}", tagged.status);
                    }
                    match tagged.status {
                        AlgebraicFiberProjectionStatus::Constructed => {
                            coefficients = tagged.coefficients;
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "selected-dense-degenerate",
                                "tagged-norm-selected-fiber",
                            );
                        }
                        AlgebraicFiberProjectionStatus::InvalidEvidence => {
                            return Err(CurveError::InvalidBezierAlgebraicParameter);
                        }
                        AlgebraicFiberProjectionStatus::UnsupportedCoefficient
                        | AlgebraicFiberProjectionStatus::Undecided => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let (coefficients, square_free) = match hypersolve::square_free_part(
        coefficients.clone(),
        hypersolve::PredicatePolicy::STRICT,
    ) {
        Some(coefficients) => (coefficients, true),
        None => (coefficients, false),
    };
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        BezierSelectedDenseLastAxisUnivariate2::Polynomial {
            polynomial,
            square_free,
        },
    ))
}

pub(super) fn isolate_selected_dense_last_axis_univariate(
    univariate: &BezierSelectedDenseLastAxisUnivariate2,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let BezierSelectedDenseLastAxisUnivariate2::Polynomial {
        polynomial,
        square_free,
    } = univariate
    else {
        return Ok(Classification::Decided(match univariate {
            BezierSelectedDenseLastAxisUnivariate2::IdenticallyZero => {
                BezierAlgebraicFiberProjection2::IdenticallyZero
            }
            BezierSelectedDenseLastAxisUnivariate2::Empty => {
                BezierAlgebraicFiberProjection2::Parameters(Vec::new())
            }
            BezierSelectedDenseLastAxisUnivariate2::Polynomial { .. } => unreachable!(),
        }));
    };
    let isolated = match (*square_free, domain) {
        (true, SelectedThirdAxisDomain2::Finite(range))
            if range == &CurveParameterRange2::unit() =>
        {
            polynomial.isolate_square_free_unit_interval_roots(policy)?
        }
        (
            true,
            SelectedThirdAxisDomain2::IncidentRay {
                anchor,
                direction,
                barrier,
            },
        ) => {
            let parameters = match polynomial
                .isolate_square_free_incident_ray_roots(anchor, direction, policy)?
            {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            retain_parameters_before_incident_barrier(parameters, barrier, direction, policy)?
        }
        _ => domain.isolate(polynomial, policy)?,
    };
    Ok(isolated.map(BezierAlgebraicFiberProjection2::Parameters))
}

/// Eliminates every already-selected source axis and isolates all candidates
/// on the final curve-parameter axis.
///
/// The resultant is only an enumerator. Sequential elimination can include
/// conjugate tuples or degree-drop factors, so callers must replay the
/// original tensor at every returned parameter before admitting topology.
pub(super) fn selected_dense_last_axis_parameters(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let univariate = match selected_dense_last_axis_univariate(polynomial, sources, policy)? {
        Classification::Decided(univariate) => univariate,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    isolate_selected_dense_last_axis_univariate(&univariate, domain, policy)
}

/// Projects one selected dense fiber once, then isolates its finite span and
/// optional incident ray from that same exact univariate authority.
pub(super) fn selected_dense_last_axis_parameters_with_incident_domain(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    domain: CurveParameterDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let univariate = match selected_dense_last_axis_univariate(polynomial, sources, policy)? {
        Classification::Decided(univariate) => univariate,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    selected_dense_last_axis_univariate_parameters_with_incident_domain(&univariate, domain, policy)
}

pub(super) fn selected_dense_last_axis_univariate_parameters_with_incident_domain(
    univariate: &BezierSelectedDenseLastAxisUnivariate2,
    domain: CurveParameterDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    selected_axis_parameters_in_domain(domain, policy, |domain| {
        isolate_selected_dense_last_axis_univariate(univariate, domain, policy)
    })
}

/// Returns the exact degree of a dense target polynomial after fixing every
/// selected source root. `None` means the complete selected fiber is zero.
///
/// Sequential resultants can vanish because a different conjugate source
/// tuple owns a target-wide component. This local coefficient replay does not
/// isolate positive-degree roots, but it can prove the important constant
/// nonzero case immediately and without admitting a conjugate component.
pub(super) fn selected_dense_last_axis_active_degree(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<usize>>> {
    if polynomial.dimensions().len() != sources.len() + 1 {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let target_count = *polynomial
        .dimensions()
        .last()
        .expect("a selected dense polynomial has a target axis");
    let strict = policy.strict_counterpart();
    for target_power in (0..target_count).rev() {
        let Some(coefficient) = dense_last_axis_coefficient(polynomial, target_power) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let value = match represented_dense_value_refined(&coefficient, sources) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match represented_policy_sign(&value, &strict) {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(Some(target_power)));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(None))
}

/// Uses a binary64 solve only to locate the at-most-two roots of one selected
/// quadratic `P + sqrt(Q) R`. Each published parameter is independently
/// certified as a simple root of the exact projection inside a
/// rational interval. Failure to certify every numeric guide returns `None`
/// so the complete global isolator remains the authority.
pub(super) fn selected_dense_guided_quadratic_parameters(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    projection: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> CurveResult<Classification<Option<Vec<BezierParameter2>>>> {
    let approximate_coefficients = |polynomial: &DenseTensorPolynomial| -> Option<Vec<f64>> {
        let dimensions = polynomial.dimensions();
        if dimensions.len() != sources.len() + 1 {
            return None;
        }
        let target_count = *dimensions.last()?;
        let source_values = sources
            .iter()
            .map(|source| refined_represented_root(source, 64))
            .map(|source| {
                Some(
                    (source.interval.lower.to_f64_lossy()?
                        + source.interval.upper.to_f64_lossy()?)
                        * 0.5,
                )
            })
            .collect::<Option<Vec<_>>>()?;
        let mut result = vec![0.0_f64; target_count];
        for (flat_index, coefficient) in polynomial.coefficients().iter().enumerate() {
            let mut remaining = flat_index;
            let mut exponents = vec![0_usize; dimensions.len()];
            for axis in (0..dimensions.len()).rev() {
                exponents[axis] = remaining % dimensions[axis];
                remaining /= dimensions[axis];
            }
            let mut value = coefficient.to_f64_lossy()?;
            for (source, exponent) in source_values.iter().zip(&exponents[..sources.len()]) {
                value *= source.powi((*exponent).try_into().ok()?);
            }
            result[exponents[sources.len()]] += value;
        }
        result
            .iter()
            .all(|value| value.is_finite())
            .then_some(result)
    };
    let Some(mut roots) = (|| {
        let rational = approximate_coefficients(rational)?;
        let radical = approximate_coefficients(radical)?;
        let radicand = approximate_coefficients(radicand)?;
        let speed_squared = *radicand.first()?;
        (speed_squared.is_finite() && speed_squared > 0.0).then_some(())?;
        let speed = speed_squared.sqrt();
        let coefficient_count = rational.len().max(radical.len());
        let mut coefficients = (0..coefficient_count)
            .map(|index| {
                rational.get(index).copied().unwrap_or(0.0)
                    + speed * radical.get(index).copied().unwrap_or(0.0)
            })
            .collect::<Vec<_>>();
        while coefficients.len() > 1
            && coefficients
                .last()
                .is_some_and(|coefficient| *coefficient == 0.0)
        {
            coefficients.pop();
        }
        let scale = coefficients
            .iter()
            .map(|coefficient| coefficient.abs())
            .fold(0.0_f64, f64::max);
        (scale.is_finite() && scale > 0.0).then_some(())?;
        let tolerance = scale * 1.0e-12;
        let c = coefficients.first().copied().unwrap_or(0.0);
        let b = coefficients.get(1).copied().unwrap_or(0.0);
        let a = coefficients.get(2).copied().unwrap_or(0.0);
        let mut roots = Vec::with_capacity(2);
        if a.abs() <= tolerance {
            (b.abs() > tolerance).then_some(())?;
            roots.push(-c / b);
        } else {
            let discriminant = b * b - 4.0 * a * c;
            let discriminant_tolerance = (b * b).abs().max((4.0 * a * c).abs()).max(1.0) * 1.0e-12;
            (discriminant >= -discriminant_tolerance).then_some(())?;
            let square_root = discriminant.max(0.0).sqrt();
            roots.push((-b - square_root) / (2.0 * a));
            if square_root > discriminant_tolerance.sqrt() {
                roots.push((-b + square_root) / (2.0 * a));
            }
        }
        roots.iter().all(|root| root.is_finite()).then_some(roots)
    })() else {
        return Ok(Classification::Decided(None));
    };
    roots.sort_by(|first, second| first.total_cmp(second));
    roots.dedup_by(|first, second| {
        (*first - *second).abs() <= 1.0e-11 * first.abs().max(second.abs()).max(1.0)
    });
    if roots.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let univariate = match selected_dense_last_axis_projection(projection, sources)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(_reason) => {
            return Ok(Classification::Decided(None));
        }
    };
    let coefficients = univariate
        .coefficients()
        .iter()
        .cloned()
        .map(|coefficient| {
            coefficient
                .exact_rational_normal_form()
                .map(Real::new)
                .unwrap_or(coefficient)
        })
        .collect::<Vec<_>>();
    let Some(coefficients) =
        hypersolve::square_free_part(coefficients, hypersolve::PredicatePolicy::STRICT)
    else {
        return Ok(Classification::Decided(None));
    };
    const GUIDE_BITS: u32 = 44;
    let denominator_integer = 1_i128 << GUIDE_BITS;
    let denominator = Real::from(denominator_integer);
    let mut parameters = Vec::with_capacity(roots.len());
    for root in roots {
        let scaled = root * denominator_integer as f64;
        if !scaled.is_finite() || scaled.abs() > i128::MAX as f64 {
            return Ok(Classification::Decided(None));
        }
        let center = scaled.round() as i128;
        let mut parameter = None;
        for radius in [
            4_i128, 16, 64, 256, 1_024, 4_096, 16_384, 65_536, 262_144, 1_048_576, 4_194_304,
        ] {
            let (Ok(lower), Ok(upper)) = (
                Real::from(center.saturating_sub(radius)) / &denominator,
                Real::from(center.saturating_add(radius)) / &denominator,
            ) else {
                continue;
            };
            let lower_sign = real_sign(
                &Real::eval_poly(&coefficients, &lower),
                &CurveContext::STRICT,
            );
            let upper_sign = real_sign(
                &Real::eval_poly(&coefficients, &upper),
                &CurveContext::STRICT,
            );
            if !matches!(
                (lower_sign, upper_sign),
                (Some(RealSign::Negative), Some(RealSign::Positive))
                    | (Some(RealSign::Positive), Some(RealSign::Negative))
            ) {
                continue;
            }
            let singleton = hypersolve::polynomial_has_one_distinct_root_in_open_interval(
                &coefficients,
                &lower,
                &upper,
                hypersolve::PredicatePolicy::STRICT,
            );
            if singleton != Some(true) {
                continue;
            }
            let interval =
                match BezierParameterInterval::try_new(lower, upper, &CurveContext::STRICT) {
                    Ok(Classification::Decided(interval)) => interval,
                    Ok(Classification::Uncertain(_)) => continue,
                    Err(error) => return Err(error),
                };
            parameter = BezierAlgebraicParameter2::from_certified_simple_power_basis(
                coefficients.clone(),
                interval,
            )
            .map(BezierParameter2::Algebraic);
            if parameter.is_some() {
                break;
            }
        }
        let Some(parameter) = parameter else {
            return Ok(Classification::Decided(None));
        };
        parameters.push(parameter);
    }
    Ok(Classification::Decided(Some(parameters)))
}
