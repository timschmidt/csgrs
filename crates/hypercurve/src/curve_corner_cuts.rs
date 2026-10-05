//! Corner cut placement, chamfer cuts and corner materialization.

use super::*;

pub(super) fn compare_corner_parameter(
    left: &Real,
    right: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    crate::classify::compare_reals(left, right, policy).ok_or_else(|| {
        ExactCurveError::blocked(operation, family, crate::UncertaintyReason::Ordering)
    })
}

pub(super) fn corner_parameter_placement(
    parameter: &Real,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero_order = compare_corner_parameter(parameter, &Real::zero(), operation, family, policy)?;
    let one_order = compare_corner_parameter(parameter, &Real::one(), operation, family, policy)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

pub(super) fn bezier_corner_parameter_placement(
    parameter: &BezierParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let compare = |boundary: &BezierParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|result| match result {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let zero_order = compare(&zero)?;
    let one_order = compare(&one)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

pub(super) fn curve_region_corner_parameter_placement(
    parameter: &CurveParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
    let one = CurveParameter2::from(BezierParameter2::Exact(Real::one()));
    let compare = |boundary: &CurveParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|result| match result {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let zero_order = compare(&zero)?;
    let one_order = compare(&one)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

pub(super) fn decided_parallel_point(
    parallel: &BezierParallel2,
    parameter: &Real,
    source_point: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    let point = if source_point {
        parallel.source_point_at(parameter, policy)
    } else {
        parallel
            .point_at(parameter, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    };
    match point {
        Classification::Decided(point) => Ok(point),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

pub(super) fn bezier_parallel_source_point_evidence(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if let Some(parameter) = parameter.scalar() {
        return match parallel.source_point_at(parameter, policy) {
            Classification::Decided(point) => Ok(point.into()),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        };
    }
    let rational_source = bezier_parallel_rational_source(parallel, operation, family)?;
    match crate::rational_bezier_general::exact_contact_point_evidence(
        &rational_source,
        parameter,
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(point) => return Ok(point),
        Classification::Uncertain(crate::UncertaintyReason::Boundary) => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Boundary,
            ));
        }
        Classification::Uncertain(_) => {}
    }
    {
        // A selected fiber can have non-rational coefficients even though the
        // source curve is rational. Keep the source point in the same procedural
        // normalized-frame carrier used by analytic parallels instead of rejecting
        // an exact parameter merely because a one-field coordinate image was not
        // profitable to materialize.
        Ok(CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
            parallel.with_distance(Real::zero()),
            parameter.clone(),
            policy,
        )))
    }
}

pub(super) fn curve_region_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    source_point: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if source_point {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return bezier_parallel_source_point_evidence(
                parallel, parameter, operation, family, policy,
            );
        }
        return analytic_parallel_point_evidence(
            &parallel.with_distance(Real::zero()),
            parameter,
            operation,
            family,
            policy,
        );
    }
    analytic_parallel_point_evidence(parallel, parameter, operation, family, policy)
}

pub(super) fn bezier_parallel_rational_source(
    parallel: &BezierParallel2,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<RationalBezier2> {
    match parallel.source() {
        crate::BezierParallelSource2::Quadratic(curve) => {
            RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(curve.clone()))
        }
        crate::BezierParallelSource2::Cubic(curve) => {
            RationalBezier2::try_from_subcurve(&BezierSubcurve2::Cubic(curve.clone()))
        }
        crate::BezierParallelSource2::Rational(curve) => Ok(curve.clone()),
    }
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn corner_chamfer_cuts(
    carrier: ExactCornerCarrier2<'_>,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    logical_run: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    match carrier {
        ExactCornerCarrier2::Line(source) => line_chamfer_cuts(
            source,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::PromotedLine(curve) => line_chamfer_cuts(
            curve
                .retained_exact_line_image()
                .expect("a promoted-line carrier retains its exact line image"),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::Arc(arc) => arc_chamfer_cuts(
            ExactCornerArc2::Native(arc.clone()),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::RetainedRationalArc(arc) => arc_chamfer_cuts(
            ExactCornerArc2::RetainedRational(arc),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::Bezier(source) => bezier_chamfer_cuts(
            ExactCornerBezier2::Direct(source),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::NativeBezierSpan(fragment) => bezier_chamfer_cuts(
            ExactCornerBezier2::NativeSpan(fragment),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AlgebraicChord(chord) => algebraic_chord_chamfer_cuts(
            chord,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AnalyticParallel(fragment) => analytic_parallel_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::SelectedFiber(fragment) => selected_fiber_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AlgebraicCusp(fragment) => algebraic_cusp_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            logical_run,
            operation,
            family,
            policy,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn algebraic_cusp_chamfer_cuts(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    logical_run: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    fragment
        .validate_policy(policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let start_endpoint = !previous;
    let corner_parameter = fragment.endpoint_parameter(start_endpoint).clone();
    let corner = match fragment
        .endpoint_point_evidence(start_endpoint, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from_algebraic_cusp(corner_parameter)),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let mut cuts = CornerCuts2::default();
    for (outward, placement) in [
        (false, CornerPlacement2::Trim),
        (true, CornerPlacement2::Extension),
    ] {
        if outward && mode != CurveCornerMode2::TrimOrExtend {
            continue;
        }
        let cut = match fragment
            .endpoint_chord_setback_cut(start_endpoint, setback, outward, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(Some(cut)) => Some(cut),
            Classification::Decided(None) if logical_run && !outward => {
                match fragment
                    .endpoint_chord_setback_support_cut(start_endpoint, setback, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
                {
                    Classification::Decided(cut) => cut,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(operation, family, reason));
                    }
                }
            }
            Classification::Decided(None) => None,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        let Some((parameter, point, complementary)) = cut else {
            continue;
        };
        let parameter = if complementary {
            CurveParameter2::from_algebraic_cusp_complement(parameter)
        } else {
            CurveParameter2::from_algebraic_cusp(parameter)
        };
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn algebraic_chord_chamfer_cuts(
    chord: &crate::BezierAlgebraicChord2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    chord
        .validate_policy(policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let corner_parameter = if previous {
        chord.end_parameter()
    } else {
        chord.start_parameter()
    };
    let corner = if previous { chord.end() } else { chord.start() };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from_algebraic_chord(corner_parameter)),
                point: corner.clone(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let interior_distance = if previous {
        -setback.clone()
    } else {
        setback.clone()
    };
    let mut cuts = CornerCuts2::default();
    let distances = if mode == CurveCornerMode2::TrimOrExtend {
        [Some(interior_distance.clone()), Some(-interior_distance)]
    } else {
        [Some(interior_distance), None]
    };
    for distance in distances.into_iter().flatten() {
        // Unit-tangent displacement is construction evidence for support
        // incidence. General endpoint fields remain separate behind one lazy
        // normalized expression; only finite-domain placement is a predicate.
        let point = match chord
            .endpoint_at_signed_tangent_distance(previous, distance, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        if let Some(cut) = algebraic_chord_corner_cut_from_support_point(
            chord, point, previous, mode, operation, family, policy,
        )? {
            cuts.push(cut);
        }
    }
    Ok(cuts)
}

pub(super) fn analytic_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if let Some(parameter) = parameter
        .as_bezier_parameter()
        .and_then(BezierParameter2::scalar)
    {
        return decided_parallel_point(parallel, parameter, false, operation, family, policy)
            .map(Into::into);
    }
    crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
        parallel.clone(),
        parameter,
        Real::zero(),
        policy,
    )
    .map(CurvePoint2::from)
    .ok_or_else(|| {
        ExactCurveError::blocked(operation, family, crate::UncertaintyReason::Unsupported)
    })
}

pub(super) fn retained_parallel_corner_parameter_placement(
    parameter: &CurveParameter2,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &BezierParameter2| {
        parameter
            .cmp_by_refinement(&CurveParameter2::from(boundary.clone()), policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|ordering| match ordering {
                Classification::Decided(ordering) => Ok(ordering),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(fragment.range().start())?;
    let end_order = compare(fragment.range().end())?;
    Ok(retained_parallel_corner_orders_placement(
        start_order,
        end_order,
        fragment,
        previous,
        mode,
    ))
}

pub(super) fn retained_parallel_corner_orders_placement(
    start_order: std::cmp::Ordering,
    end_order: std::cmp::Ordering,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
) -> Option<CornerPlacement2> {
    if start_order.is_gt() && end_order.is_lt() {
        return Some(CornerPlacement2::Trim);
    }
    if mode != CurveCornerMode2::TrimOrExtend {
        return None;
    }
    let extends_toward_higher_parameter = previous != fragment.is_reversed();
    ((extends_toward_higher_parameter && end_order.is_gt())
        || (!extends_toward_higher_parameter && start_order.is_lt()))
    .then_some(CornerPlacement2::Extension)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn selected_fiber_corner_parameter_placement(
    parameter: &CurveParameter2,
    fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &CurveParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|ordering| match ordering {
                Classification::Decided(ordering) => Ok(ordering),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(fragment.range().start())?;
    let end_order = compare(fragment.range().end())?;
    if start_order.is_gt() && end_order.is_lt() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode != CurveCornerMode2::TrimOrExtend {
        return Ok(None);
    }
    let extends_toward_higher_parameter = previous != fragment.is_reversed();
    Ok(((extends_toward_higher_parameter && end_order.is_gt())
        || (!extends_toward_higher_parameter && start_order.is_lt()))
    .then_some(CornerPlacement2::Extension))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn selected_fiber_chamfer_cuts(
    fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner_parameter = if previous != fragment.is_reversed() {
        fragment.range().end()
    } else {
        fragment.range().start()
    };
    let corner = if previous {
        fragment.end_point().clone()
    } else {
        fragment.start_point().clone()
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(corner_parameter.clone()),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let parallel = fragment.parallel_carrier();
    let direction = if previous != fragment.is_reversed() {
        crate::BezierParameterRayDirection2::Increasing
    } else {
        crate::BezierParameterRayDirection2::Decreasing
    };
    let parameters = match parallel
        .fixed_distance_incidence(
            &parallel,
            corner_parameter,
            setback,
            fragment.range(),
            (mode == CurveCornerMode2::TrimOrExtend).then_some(direction),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = selected_fiber_corner_parameter_placement(
            &parameter, fragment, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point =
            analytic_parallel_point_evidence(&parallel, &parameter, operation, family, policy)?;
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn analytic_parallel_chamfer_cuts(
    fragment: &crate::BezierParallelFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner_parameter = match (previous, fragment.is_reversed()) {
        (true, false) | (false, true) => fragment.range().end(),
        (true, true) | (false, false) => fragment.range().start(),
    };
    let corner = analytic_parallel_point_evidence(
        fragment.parallel(),
        &corner_parameter.clone().into(),
        operation,
        family,
        policy,
    )?;
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from(corner_parameter.clone())),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let direction = if previous != fragment.is_reversed() {
        crate::BezierParameterRayDirection2::Increasing
    } else {
        crate::BezierParameterRayDirection2::Decreasing
    };
    let parameters = match fragment
        .parallel()
        .fixed_distance_incidence(
            fragment.parallel(),
            &CurveParameter2::from(corner_parameter.clone()),
            setback,
            &crate::CurveParameterRange2::new_validated(
                fragment.range().start().clone().into(),
                fragment.range().end().clone().into(),
            ),
            (mode == CurveCornerMode2::TrimOrExtend).then_some(direction),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = retained_parallel_corner_parameter_placement(
            &parameter, fragment, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point = analytic_parallel_point_evidence(
            fragment.parallel(),
            &parameter,
            operation,
            family,
            policy,
        )?;
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn bezier_chamfer_cuts(
    source: ExactCornerBezier2<'_>,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner = source.corner(previous);
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(source.curve_parameter(
                    &if previous { Real::one() } else { Real::zero() }.into(),
                    operation,
                    family,
                    policy,
                )?),
                point: corner.clone().into(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }

    let radius_squared = setback * setback;
    let parallel = exact_corner_bezier_parallel(source, Real::zero(), operation, family)?;
    let mut parameters = match parallel
        .source_circle_incidence(
            corner,
            &radius_squared,
            &crate::CurveParameterRange2::unit(),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    if mode == CurveCornerMode2::TrimOrExtend {
        let (anchor, direction) = if previous {
            (Real::one(), crate::BezierParameterRayDirection2::Increasing)
        } else {
            (
                Real::zero(),
                crate::BezierParameterRayDirection2::Decreasing,
            )
        };
        let exterior = match parallel
            .source_circle_incidence_on_incident_ray(
                corner,
                &radius_squared,
                &anchor,
                direction,
                policy,
            )
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        parameters.extend(exterior);
    }
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = bezier_corner_parameter_placement(
            &parameter, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point = bezier_parallel_source_point_evidence(
            &parallel, &parameter, operation, family, policy,
        )?;
        let parameter =
            Some(source.curve_parameter(&parameter.into(), operation, family, policy)?);
        cuts.push(CornerCut2 {
            parameter,
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn arc_chamfer_cuts(
    arc: ExactCornerArc2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let support = arc.support();
    validate_exact_corner_arc_support(support, operation, family, policy)?;
    let corner = match &arc {
        ExactCornerArc2::Native(_) => CurvePoint2::from(
            if previous {
                support.end()
            } else {
                support.start()
            }
            .clone(),
        ),
        ExactCornerArc2::RetainedRational(arc) => if previous {
            arc.fragment.end_point()
        } else {
            arc.fragment.start_point()
        }
        .clone(),
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(arc.corner_parameter(previous, operation, family, policy)?),
                point: corner.clone(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }

    let points = if let Some(corner) = corner.coordinates() {
        circular_setback_points(support, corner, setback, operation, family, policy)?
            .map(|point| point.map(CurvePoint2::from))
    } else {
        // A setback on a known circle is a rotation of the incident radius.
        // Keep the selected endpoint field instead of adjoining another root
        // merely to solve two circles that already share this radial frame.
        let cosine = Real::one()
            - (setback * setback / (Real::from(2) * support.radius_squared()))
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
        let sine_squared = Real::one() - &cosine * &cosine;
        let sign = match crate::classify::real_sign(&sine_squared, policy) {
            Some(sign) => sign,
            None => {
                return Err(ExactCurveError::blocked(
                    operation,
                    family,
                    crate::UncertaintyReason::RealSign,
                ));
            }
        };
        if sign == RealSign::Negative {
            return Ok(CornerCuts2::default());
        }
        let sine = sine_squared
            .sqrt()
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
        let rotate =
            |sine: Real| -> ExactCurveResult<CurvePoint2> {
                let center = support.center();
                let complement = Real::one() - &cosine;
                let transform = crate::Similarity2::try_from_real_affine(
                    cosine.clone(),
                    -&sine,
                    sine.clone(),
                    cosine.clone(),
                    center.x() * &complement + center.y() * &sine,
                    center.y() * complement - center.x() * sine,
                )
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
                Ok(crate::bezier_offset::BezierSimilarityPoint2::new(
                    corner.clone(),
                    transform,
                    policy,
                )
                .into())
            };
        [
            Some(rotate(sine.clone())?),
            if sign == RealSign::Zero {
                None
            } else {
                Some(rotate(-sine)?)
            },
        ]
    };
    let mut cuts = CornerCuts2::default();
    for point in points.into_iter().flatten() {
        let cut = match &arc {
            ExactCornerArc2::RetainedRational(arc) => {
                arc.cut_at_incident_point(point, previous, mode, false, operation, family, policy)?
            }
            ExactCornerArc2::Native(_) => arc_corner_cut_from_incident_point(
                &arc,
                point
                    .coordinates()
                    .expect("native circular setback")
                    .clone(),
                previous,
                mode,
                operation,
                family,
                policy,
            )?,
        };
        if let Some(cut) = cut {
            cuts.push(cut)
        }
    }
    Ok(cuts)
}

/// The circle relation owns these exact contact points. Chart inverses may
/// recover source parameters without constructing a second point authority.
pub(super) fn circular_setback_points(
    support: &CircularArc2,
    corner: &Point2,
    setback: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<[Option<Point2>; 2]> {
    let setback_squared = setback * setback;
    let relation = crate::intersect::circle_relation_from_supports(
        support.center(),
        support.radius_squared_ref(),
        corner,
        &setback_squared,
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    Ok(match relation {
        crate::CircleCircleRelation::Disjoint => [None, None],
        crate::CircleCircleRelation::Tangent { point } => [Some(point), None],
        crate::CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => [Some(first_point), Some(second_point)],
        crate::CircleCircleRelation::Coincident => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
        crate::CircleCircleRelation::Uncertain { reason } => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    })
}

#[allow(clippy::too_many_arguments)]
pub(super) fn arc_corner_cut_from_incident_point(
    arc: &ExactCornerArc2,
    point: Point2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    if let ExactCornerArc2::RetainedRational(arc) = arc {
        return arc.cut_at_incident_point(
            point.into(),
            previous,
            mode,
            false,
            operation,
            family,
            policy,
        );
    }
    // The chamfer circle relation or fillet offset/contact construction has
    // already certified source-support incidence. Re-expanding the radical
    // construction through `contains_point` would ask Hyperreal to rediscover
    // that equality and can block STRICT on an otherwise exact square-root
    // representation. Only sweep membership is a new predicate here.
    let support = arc.support();
    match support.contains_sweep_point(&point, policy) {
        Classification::Decided(true) => {
            let sweep_fraction = match support
                .sweep_fraction_for_incident_point(&point, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(operation, family, reason));
                }
            };
            if corner_parameter_placement(
                &sweep_fraction,
                previous,
                CurveCornerMode2::TrimOnly,
                operation,
                family,
                policy,
            )? == Some(CornerPlacement2::Trim)
            {
                return Ok(Some(CornerCut2 {
                    parameter: exact_corner_parameter(sweep_fraction),
                    point: point.into(),
                    placement: CornerPlacement2::Trim,
                }));
            }
        }
        Classification::Decided(false) if mode == CurveCornerMode2::TrimOrExtend => {
            if arc_extension_contains_corner(support, &point, previous, operation, family, policy)?
            {
                return Ok(Some(CornerCut2 {
                    parameter: Some(arc.corner_parameter(previous, operation, family, policy)?),
                    point: point.into(),
                    placement: CornerPlacement2::Extension,
                }));
            }
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    }
    Ok(None)
}

pub(crate) fn arc_fillet_cut_from_incident_point(
    arc: &ExactCornerArc2,
    point: CurvePoint2,
    deferred_arc_contact: bool,
    previous: bool,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    if let ExactCornerArc2::RetainedRational(arc) = arc {
        return arc.cut_at_incident_point(
            point,
            previous,
            domain.mode(),
            matches!(domain, FilletContactDomain2::SourceChart(_)),
            CurveOperation2::Fillet,
            family,
            policy,
        );
    }
    use crate::segment::ArcSweepPointLocation2;

    let mode = domain.mode();
    let support = arc.support();
    match support
        .strict_incident_point_evidence_location(&point, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(ArcSweepPointLocation2::Interior) => {
            let parameter = if deferred_arc_contact {
                Some(arc.corner_parameter(previous, CurveOperation2::Fillet, family, policy)?)
            } else {
                None
            };
            Ok(Some(CornerCut2 {
                parameter,
                point,
                placement: CornerPlacement2::Trim,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Endpoint) => {
            if matches!(domain, FilletContactDomain2::AuthoredCurve(_)) {
                return Ok(None);
            }
            let at_start = match point.same_point(&support.start().clone().into(), policy) {
                Classification::Decided(at_start) => at_start,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    ));
                }
            };
            let parameter: CurveParameter2 =
                if at_start { Real::zero() } else { Real::one() }.into();
            let corner = arc.corner_parameter(previous, CurveOperation2::Fillet, family, policy)?;
            let placement =
                domain.with_boundary_contact(None, &parameter, || corner, family, policy)?;
            Ok(placement.map(|placement| CornerCut2 {
                point,
                parameter: Some(parameter),
                placement,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Outside)
            if mode == CurveCornerMode2::TrimOrExtend =>
        {
            // On-circle points outside the closed authored sweep lie in its
            // incident circular complement. Both authored endpoints were
            // excluded above, so no additional angular reconstruction is needed.
            Ok(Some(CornerCut2 {
                parameter: Some(arc.corner_parameter(
                    previous,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?),
                point,
                placement: CornerPlacement2::Extension,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Outside) => Ok(None),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

pub(super) fn validate_exact_corner_arc_support(
    arc: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match crate::classify::real_sign(arc.radius_squared_ref(), policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::ZeroRadiusArc,
            ));
        }
        Some(RealSign::Negative) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::RadiusMismatch,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    }
    if !arc.endpoints_on_stored_circle_are_certified() {
        for endpoint in [arc.start(), arc.end()] {
            let radius_delta = endpoint.distance_squared(arc.center()) - arc.radius_squared_ref();
            match crate::classify::is_zero(&radius_delta, policy) {
                Some(true) => {}
                Some(false) => {
                    return Err(ExactCurveError::invalid(
                        operation,
                        family,
                        CurveError::RadiusMismatch,
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        operation,
                        family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            }
        }
    }
    Ok(())
}

pub(super) fn exact_corner_arc_radius(
    arc: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    validate_exact_corner_arc_support(arc, operation, family, policy)?;
    arc.radius_squared()
        .sqrt()
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))
}

pub(super) fn arc_extension_contains_corner(
    arc: &CircularArc2,
    point: &Point2,
    previous: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let extended = if previous {
        CircularArc2::new_with_certified_radius(
            arc.start().clone(),
            point.clone(),
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )
    } else {
        CircularArc2::new_with_certified_radius(
            point.clone(),
            arc.end().clone(),
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )
    };
    let retained_corner = if previous { arc.end() } else { arc.start() };
    match extended.contains_sweep_point(retained_corner, policy) {
        Classification::Decided(contains) => Ok(contains),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn line_chamfer_cuts(
    line: &LineSeg2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: exact_corner_parameter(if previous {
                    Real::one()
                } else {
                    Real::zero()
                }),
                point: if previous {
                    line.end().clone()
                } else {
                    line.start().clone()
                }
                .into(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let (dx, dy) = line.delta();
    let (_, _, length) = line_unit_direction(&dx, &dy, operation, family, policy)?;
    let ratio = (setback / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
    let interior_parameter = if previous {
        Real::one() - &ratio
    } else {
        ratio.clone()
    };
    let mut cuts = CornerCuts2::default();
    let interior_after_zero = compare_corner_parameter(
        &interior_parameter,
        &Real::zero(),
        operation,
        family,
        policy,
    )?;
    let interior_before_one =
        compare_corner_parameter(&interior_parameter, &Real::one(), operation, family, policy)?;
    if interior_after_zero == std::cmp::Ordering::Greater
        && interior_before_one == std::cmp::Ordering::Less
    {
        cuts.push(CornerCut2 {
            point: line.point_at(interior_parameter.clone()).into(),
            parameter: exact_corner_parameter(interior_parameter),
            placement: CornerPlacement2::Trim,
        });
    }
    if mode == CurveCornerMode2::TrimOrExtend {
        let extension_parameter = if previous {
            Real::one() + ratio
        } else {
            -ratio
        };
        cuts.push(CornerCut2 {
            point: line.point_at(extension_parameter.clone()).into(),
            parameter: exact_corner_parameter(extension_parameter),
            placement: CornerPlacement2::Extension,
        });
    }
    Ok(cuts)
}

pub(super) enum MaterializedCornerSide2 {
    One(Curve2),
    SplineExtension {
        source: Curve2,
        extension: Curve2,
        previous: bool,
    },
}

impl MaterializedCornerSide2 {
    pub(super) const fn extra_curve_count(&self) -> usize {
        match self {
            Self::One(_) => 0,
            Self::SplineExtension { .. } => 1,
        }
    }

    pub(super) fn append_to(self, curves: &mut Vec<Curve2>) {
        match self {
            Self::One(curve) => curves.push(curve),
            Self::SplineExtension {
                source,
                extension,
                previous: true,
            } => curves.extend([source, extension]),
            Self::SplineExtension {
                source,
                extension,
                previous: false,
            } => curves.extend([extension, source]),
        }
    }
}

pub(super) enum MaterializedCornerBody2 {
    One(Curve2),
    Two(Curve2, Curve2),
    Three(Curve2, Curve2, Curve2),
}

impl MaterializedCornerBody2 {
    pub(super) fn from_spline_sides(
        next: MaterializedCornerSide2,
        previous: MaterializedCornerSide2,
    ) -> Self {
        match (next, previous) {
            (
                MaterializedCornerSide2::SplineExtension {
                    extension,
                    previous: false,
                    ..
                },
                MaterializedCornerSide2::One(previous),
            ) => Self::Two(extension, previous),
            (
                MaterializedCornerSide2::One(next),
                MaterializedCornerSide2::SplineExtension {
                    extension,
                    previous: true,
                    ..
                },
            ) => Self::Two(next, extension),
            (
                MaterializedCornerSide2::SplineExtension {
                    source,
                    extension: next_extension,
                    previous: false,
                },
                MaterializedCornerSide2::SplineExtension {
                    extension: previous_extension,
                    previous: true,
                    ..
                },
            ) => Self::Three(next_extension, source, previous_extension),
            _ => unreachable!("corner sides retain their incident traversal direction"),
        }
    }

    pub(super) const fn curve_count(&self) -> usize {
        match self {
            Self::One(_) => 1,
            Self::Two(_, _) => 2,
            Self::Three(_, _, _) => 3,
        }
    }

    pub(super) fn append_to(self, curves: &mut Vec<Curve2>) {
        match self {
            Self::One(curve) => curves.push(curve),
            Self::Two(first, second) => curves.extend([first, second]),
            Self::Three(first, second, third) => curves.extend([first, second, third]),
        }
    }
}

pub(super) fn materialize_single_curve_corner_body(
    curve: &Curve2,
    previous: &CornerCut2,
    next: &CornerCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<MaterializedCornerBody2> {
    if previous.placement == CornerPlacement2::Extension
        || next.placement == CornerPlacement2::Extension
    {
        if matches!(
            curve.geometry(),
            Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_))
        ) {
            return Ok(MaterializedCornerBody2::from_spline_sides(
                materialize_corner_side(curve, next, false, operation, policy)?,
                materialize_corner_side(curve, previous, true, operation, policy)?,
            ));
        }
        let start = next.exact_parameter().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        let end = previous.exact_parameter().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        return materialize_affine_corner_subcurve(curve, start, end, operation, policy)
            .map(MaterializedCornerBody2::One);
    }

    let parameter = |cut: &CornerCut2| {
        cut.exact_parameter().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })
    };
    let (start, end) = if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
        (
            materialized_arc_cut_parameter(curve, arc, next, operation, policy)?,
            materialized_arc_cut_parameter(curve, arc, previous, operation, policy)?,
        )
    } else {
        (parameter(next)?, parameter(previous)?)
    };
    curve
        .subcurve_raw(start, end, policy)
        .map_err(|error| remap_operation(error, operation))
        .map(MaterializedCornerBody2::One)
}

pub(super) fn materialized_arc_cut_parameter(
    curve: &Curve2,
    arc: &CircularArc2,
    cut: &CornerCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    let point = cut.exact_point().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    // Deferred circle contacts carry endpoint markers, not source parameters.
    // The certified point supplies the actual chart coordinate for lineage.
    match arc
        .parameter_at_incident_point(point, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(parameter) => Ok(parameter),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, curve.family(), reason))
        }
    }
}

pub(super) fn materialize_corner_side(
    curve: &Curve2,
    cut: &CornerCut2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<MaterializedCornerSide2> {
    if cut.placement != CornerPlacement2::Extension
        || !matches!(
            curve.geometry(),
            Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_))
        )
    {
        return materialize_corner_cut(curve, cut, previous, operation, policy)
            .map(MaterializedCornerSide2::One);
    }

    let parameter = cut.exact_parameter().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    cut.exact_point().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    let fragments = curve.native_bezier_fragments_for_operation(policy, operation)?;
    let fragment = if previous {
        fragments.last()
    } else {
        fragments.first()
    }
    .ok_or_else(|| {
        ExactCurveError::invalid(
            operation,
            curve.family(),
            CurveError::Topology(
                "spline corner extension did not retain an incident native span".into(),
            ),
        )
    })?;
    let (span_start, span_end) = fragment.parameter_range();
    let local_parameter = ((parameter - span_start) / (span_end - span_start))
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause.into()))?;
    let (start, end) = if previous {
        (Real::one(), local_parameter)
    } else {
        (local_parameter, Real::zero())
    };
    let extension = match fragment
        .native_curve()
        .subcurve_between_affine_exact(&start, &end, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(extension) => Curve2::from(extension),
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, curve.family(), reason));
        }
    };
    Ok(MaterializedCornerSide2::SplineExtension {
        source: curve.clone(),
        extension,
        previous,
    })
}

pub(super) fn materialize_affine_corner_subcurve(
    curve: &Curve2,
    start: &Real,
    end: &Real,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Curve2> {
    let source = match curve.geometry() {
        Some(CurveGeometry2::QuadraticBezier(source)) => BezierSubcurve2::Quadratic(source.clone()),
        Some(CurveGeometry2::CubicBezier(source)) => BezierSubcurve2::Cubic(source.clone()),
        Some(CurveGeometry2::RationalQuadraticBezier(source)) => {
            BezierSubcurve2::RationalQuadratic(source.clone())
        }
        Some(CurveGeometry2::RationalBezier(source)) => BezierSubcurve2::Rational(source.clone()),
        _ => {
            return Err(ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            ));
        }
    };
    match source
        .subcurve_between_affine_exact(start, end, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(curve) => Ok(Curve2::from(curve)),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, curve.family(), reason))
        }
    }
}

pub(super) fn materialize_corner_cut(
    curve: &Curve2,
    cut: &CornerCut2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Curve2> {
    match cut.placement {
        CornerPlacement2::Trim => {
            if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
                let point = cut.exact_point().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                // The carrier solver already certified the strict parameter
                // placement and the circle kernel certified `cut.point` on
                // the support. Retain that exact point as the fragment
                // endpoint instead of evaluating an algebraically equivalent
                // rational parameter and then asking path connectivity to
                // rediscover the equality.
                let parameter = materialized_arc_cut_parameter(curve, arc, cut, operation, policy)?;
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start().clone(), parameter)
                } else {
                    (parameter, domain.end().clone())
                };
                let lineage = curve
                    .lineage_subrange(&start, &end)
                    .map_err(|error| remap_operation(error, operation))?;
                let constructor = if arc.endpoints_on_stored_circle_are_certified() {
                    CircularArc2::new_with_certified_radius
                } else {
                    CircularArc2::new_unchecked_with_radius
                };
                let trimmed = if previous {
                    constructor(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    constructor(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                };
                Ok(Curve2::from_geometry_with_lineage(
                    CurveGeometry2::CircularArc(trimmed),
                    lineage,
                ))
            } else {
                let parameter = cut.exact_parameter().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start().clone(), parameter.clone())
                } else {
                    (parameter.clone(), domain.end().clone())
                };
                curve
                    .subcurve_raw(start, end, policy)
                    .map_err(|error| remap_operation(error, operation))
            }
        }
        CornerPlacement2::Corner => Ok(curve.clone()),
        CornerPlacement2::Extension => {
            let point = cut.exact_point().ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    curve.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            if let Some(line) = exact_linear_corner_line(curve) {
                let extended = if previous {
                    LineSeg2::try_new(line.start().clone(), point.clone())
                } else {
                    LineSeg2::try_new(point.clone(), line.end().clone())
                }
                .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?;
                Ok(match curve.geometry() {
                    Some(CurveGeometry2::QuadraticBezier(source))
                        if source.retained_exact_line_image().is_some() =>
                    {
                        Curve2::from(QuadraticBezier2::from_line_segment(extended))
                    }
                    _ => Curve2::from(extended),
                })
            } else if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
                Ok(Curve2::from(if previous {
                    CircularArc2::new_with_certified_radius(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    CircularArc2::new_with_certified_radius(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                }))
            } else if let Some(arc) = retained_rational_arc_support(curve, operation, policy)? {
                Ok(Curve2::from(if previous {
                    CircularArc2::new_with_certified_radius(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    CircularArc2::new_with_certified_radius(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                }))
            } else if let Some(parameter) = cut.exact_parameter() {
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start(), parameter)
                } else {
                    (parameter, domain.end())
                };
                materialize_affine_corner_subcurve(curve, start, end, operation, policy)
            } else {
                Err(ExactCurveError::blocked(
                    operation,
                    curve.family(),
                    crate::UncertaintyReason::Unsupported,
                ))
            }
        }
    }
}

pub(super) fn certify_closed_path(
    path: &CurvePath2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match validate_closed_curve_path_connectivity(path, policy)
        .map_err(|error| remap_operation(error, operation))?
    {
        Classification::Decided(()) => Ok(()),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            path.data.curves[0].family(),
            reason,
        )),
    }
}

pub(super) fn validate_strict_split_parameter(
    domain_start: &Real,
    parameter: &Real,
    domain_end: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match (
        crate::classify::compare_reals(domain_start, parameter, policy),
        crate::classify::compare_reals(parameter, domain_end, policy),
    ) {
        (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            crate::UncertaintyReason::Ordering,
        )),
    }
}

pub(super) fn validate_subcurve_range(
    domain_start: &Real,
    start: &Real,
    end: &Real,
    domain_end: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match (
        crate::classify::compare_reals(domain_start, start, policy),
        crate::classify::compare_reals(start, end, policy),
        crate::classify::compare_reals(end, domain_end, policy),
    ) {
        (
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            Some(std::cmp::Ordering::Less),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
        ) => Ok(()),
        (Some(_), Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            crate::UncertaintyReason::Ordering,
        )),
    }
}
