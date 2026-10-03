//! Exact offset bands, joins and tangent relations.

use super::*;

pub(super) fn exact_offset_join_band_semantics(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<(bool, bool)>> {
    let endpoint_reason = match previous.offset_end.same_point(&next.offset_start, policy) {
        Classification::Decided(true) => {
            // A span that collapses to a point intentionally has no tangent.
            // Its adjacent offset spans already meet at the retained center,
            // so no corner band exists and no synthetic tangent is needed.
            return Ok(Classification::Decided((true, true)));
        }
        Classification::Decided(false) => None,
        Classification::Uncertain(reason) => Some(reason),
    };
    let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
    else {
        return Ok(Classification::Uncertain(
            endpoint_reason.unwrap_or(UncertaintyReason::Unsupported),
        ));
    };
    // Inner and outer joins are a property of the source path's turn. With
    // source vertex V, the offset ends are V + d*n_previous and
    // V + d*n_next, so their orientation about V is the sign of
    // d^2 * cross(n_previous, n_next): the source turn for either offset
    // side. Offset tangents can run against their source where the offset
    // exceeds the curvature radius, so they decide only when the offset ends
    // are collinear with V.
    let turn = match offset_end_source_turn(previous, next, policy)? {
        Some(turn) => turn,
        None => match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
            Classification::Decided(turn) => turn,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        },
    };
    let distance = match real_sign(distance, policy) {
        Some(distance) => distance,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    Ok(Classification::Decided((
        exact_sign_product(turn, distance) == RealSign::Positive,
        turn == RealSign::Positive,
    )))
}

/// Signs `cross(previous.offset_end - V, next.offset_start - V)` for the
/// shared source vertex `V`, or `None` when it is zero or undecided.
fn offset_end_source_turn(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    let vertex = &previous.source_end;
    if let (Some(vertex), Some(first), Some(second)) = (
        vertex.coordinates(),
        previous.offset_end.coordinates(),
        next.offset_start.coordinates(),
    ) {
        let cross = Real::diff_of_products(
            &(first.x() - vertex.x()),
            &(second.y() - vertex.y()),
            &(first.y() - vertex.y()),
            &(second.x() - vertex.x()),
        );
        return Ok(match real_sign(&cross, policy) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => Some(sign),
            Some(RealSign::Zero) | None => None,
        });
    }
    let Classification::Decided(radial) =
        crate::BezierAlgebraicChord2::try_new(vertex.clone(), previous.offset_end.clone(), policy)?
    else {
        return Ok(None);
    };
    Ok(
        match radial.oriented_support_side(&next.offset_start, policy)? {
            Classification::Decided(crate::classify::LineSide::Left) => Some(RealSign::Positive),
            Classification::Decided(crate::classify::LineSide::Right) => Some(RealSign::Negative),
            Classification::Decided(crate::classify::LineSide::On)
            | Classification::Uncertain(_) => None,
        },
    )
}

pub(super) fn exact_offset_spans_form_reversal(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
    else {
        return Ok(Classification::Decided(false));
    };
    match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Ok(Classification::Decided(false))
        }
        Classification::Decided(RealSign::Zero) => Ok(curve_tangents_are_opposite(
            previous_tangent,
            next_tangent,
            policy,
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn exact_offset_band_connector(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &CurvePoint2,
    to: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    append_exact_algebraic_line_join(fragments, from, to, None, None, true, [false; 2], policy)
}

/// Builds the band between one source span's two parallel offsets.
///
/// A supplied start or end vertex splits that end connector at the retained
/// source vertex: the connector joins `vertex - s*n` to `vertex + s*n`, so the
/// vertex is its exact midpoint. Callers supply only certified corners, where
/// the adjacent bands' connectors cross transversally; keeping the retained
/// vertex there, rather than a recomputed crossing, preserves its identity
/// with the region boundary.
pub(super) fn exact_offset_span_band_loop(
    opposite: &ExactOffsetSpan2,
    span: &ExactOffsetSpan2,
    start_vertex: Option<&CurvePoint2>,
    end_vertex: Option<&CurvePoint2>,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveRegionBoundaryLoop2>> {
    // Both halves of a split connector must be native lines through
    // represented points: algebraic halves would be collinear chords whose
    // continuation through the vertex the arrangement cannot certify.
    let split_at_vertices = !opposite.fragments.is_empty() && !span.fragments.is_empty();
    let represented = |points: [&CurvePoint2; 3]| {
        points
            .into_iter()
            .all(|point| point.coordinates().is_some())
    };
    let start_vertex = start_vertex.filter(|vertex| {
        split_at_vertices && represented([&span.offset_start, vertex, &opposite.offset_start])
    });
    let end_vertex = end_vertex.filter(|vertex| {
        split_at_vertices && represented([&opposite.offset_end, vertex, &span.offset_end])
    });
    let mut fragments = Vec::with_capacity(
        opposite
            .fragments
            .len()
            .saturating_add(span.fragments.len())
            .saturating_add(2),
    );
    fragments.extend(opposite.fragments.iter().cloned());
    let end_vertices: Vec<&CurvePoint2> = match end_vertex {
        Some(vertex) => vec![&opposite.offset_end, vertex, &span.offset_end],
        None => vec![&opposite.offset_end, &span.offset_end],
    };
    for pair in end_vertices.windows(2) {
        match exact_offset_band_connector(&mut fragments, pair[0], pair[1], policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for fragment in span.fragments.iter().rev() {
        fragments.push(fragment.reversed()?);
    }
    let start_vertices: Vec<&CurvePoint2> = match start_vertex {
        Some(vertex) => vec![&span.offset_start, vertex, &opposite.offset_start],
        None => vec![&span.offset_start, &opposite.offset_start],
    };
    for pair in start_vertices.windows(2) {
        match exact_offset_band_connector(&mut fragments, pair[0], pair[1], policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(fragments, None, policy)
        .map(Classification::Decided)
}

pub(super) fn exact_offset_corner_band_loop(
    source_vertex: &CurvePoint2,
    previous_offset_end: &CurvePoint2,
    join_fragments: Vec<BezierSplitFragment2>,
    next_offset_start: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveRegionBoundaryLoop2>> {
    if join_fragments.is_empty() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let mut fragments = Vec::with_capacity(join_fragments.len().saturating_add(2));
    match exact_offset_band_connector(&mut fragments, source_vertex, previous_offset_end, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    fragments.extend(join_fragments);
    match exact_offset_band_connector(&mut fragments, next_offset_start, source_vertex, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(fragments, None, policy)
        .map(Classification::Decided)
}

pub(super) fn exact_offset_span_runs_from_open_path(
    source_fragments: &[BezierSplitFragment2],
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let mut spans = Vec::with_capacity(source_fragments.len());
    let mut processed = 0;
    while processed < source_fragments.len() {
        let (run, consumed) = match exact_offset_span_from_source_run(
            source_fragments,
            processed,
            source_fragments.len() - processed,
            distance,
            policy,
        )? {
            Classification::Decided(run) => run,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        spans.extend(run);
        processed += consumed;
    }
    Ok(Classification::Decided(spans))
}

pub(super) fn exact_offset_corner_band(
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(CurveRegionBoundaryLoop2, bool)>>> {
    let (inner_join, filled_side_is_left) =
        match exact_offset_join_band_semantics(previous, next, distance, policy)? {
            Classification::Decided(semantics) => semantics,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if inner_join {
        return Ok(Classification::Decided(None));
    }

    let mut join_fragments = Vec::new();
    match append_exact_offset_join(&mut join_fragments, previous, next, distance, style, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    if join_fragments.is_empty() {
        return Ok(Classification::Decided(None));
    }
    exact_offset_corner_band_loop(
        &previous.source_end,
        &previous.offset_end,
        join_fragments,
        &next.offset_start,
        policy,
    )
    .map(|loop_| loop_.map(|loop_| Some((loop_, filled_side_is_left))))
}

pub(super) fn regularized_exact_offset_band_arrangement(
    boundary_loops: Vec<CurveRegionBoundaryLoop2>,
    filled_sides_are_left: Vec<bool>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    if boundary_loops.is_empty() || boundary_loops.len() != filled_sides_are_left.len() {
        return Err(curve_region_edit_error(
            CurveOperation2::Offset,
            CurveError::Topology("exact offset produced inconsistent boundary bands".into()),
        ));
    }
    let band_count = boundary_loops.len();
    let mut band = CurveRegion2::new(boundary_loops)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    {
        let data = band.data_mut_for_construction();
        data.certified_loop_roles = Some(shared_all_material_curve_region_loop_roles(band_count));
        data.state = CurveRegionState2::Authored(Arc::from(vec![FillRule::NonZero; band_count]));
    }
    band = band
        .with_certified_filled_side_is_left(filled_sides_are_left)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    band.regularized_region_raw(policy)
        .map_err(|error| error.with_operation(CurveOperation2::Offset))
}

pub(super) fn exact_round_path_cap_band(
    center: &Point2,
    distance: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionBoundaryLoop2> {
    let positive = center.translated(distance.clone(), Real::zero());
    let negative = center.translated(-distance.clone(), Real::zero());
    let first =
        CircularArc2::try_from_center(positive.clone(), negative.clone(), center.clone(), false)
            .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    let second = CircularArc2::try_from_center(negative, positive, center.clone(), false)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    let contour = Contour2::try_new_with_fill_rule(
        vec![Segment2::Arc(first), Segment2::Arc(second)],
        FillRule::NonZero,
    )
    .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?;
    curve_region_boundary_loop_from_native_material_contour(contour, policy)
        .map_err(|error| error.with_operation(CurveOperation2::Offset))
}

pub(super) fn exact_path_endpoint_unit_tangent(
    path: &CurvePath2,
    at_start: bool,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<(Real, Real)>> {
    let curve = if at_start {
        &path.curves()[0]
    } else {
        path.curves().last().expect("a curve path is nonempty")
    };
    let parameter = if at_start {
        curve.parameter_domain().start()
    } else {
        curve.parameter_domain().end()
    };
    let Some(parameter) = parameter.scalar() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let max_order = match curve.geometry() {
        None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Some(CurveGeometry2::Line(_)) | Some(CurveGeometry2::CircularArc(_)) => 1,
        Some(CurveGeometry2::QuadraticBezier(_))
        | Some(CurveGeometry2::RationalQuadraticBezier(_)) => 2,
        Some(CurveGeometry2::CubicBezier(_)) => 3,
        Some(CurveGeometry2::RationalBezier(curve)) => curve.degree(),
        Some(CurveGeometry2::PolynomialBSpline(curve)) => curve.degree(),
        Some(CurveGeometry2::Nurbs(curve)) => curve.degree(),
    };
    let derivatives = curve
        .derivatives_at_side_with_policy(
            parameter,
            max_order,
            if at_start {
                CurveParameterSide2::Right
            } else {
                CurveParameterSide2::Left
            },
            policy,
        )
        .map_err(|error| error.with_operation(CurveOperation2::Offset))?;
    for (index, derivative) in derivatives.into_iter().enumerate() {
        let length_squared = derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
        match real_sign(&length_squared, policy) {
            Some(RealSign::Positive) => {
                let length = length_squared.sqrt().map_err(|cause| {
                    curve_region_edit_error(CurveOperation2::Offset, cause.into())
                })?;
                let orientation = if !at_start && (index + 1).is_multiple_of(2) {
                    -Real::one()
                } else {
                    Real::one()
                };
                return Ok(Classification::Decided((
                    ((derivative.dx() * &orientation) / &length).map_err(|cause| {
                        curve_region_edit_error(CurveOperation2::Offset, cause.into())
                    })?,
                    ((derivative.dy() * orientation) / length).map_err(|cause| {
                        curve_region_edit_error(CurveOperation2::Offset, cause.into())
                    })?,
                )));
            }
            Some(RealSign::Zero) => {}
            Some(RealSign::Negative) => {
                return Err(curve_region_edit_error(
                    CurveOperation2::Offset,
                    CurveError::Topology(
                        "curve endpoint derivative squared norm was certified negative".into(),
                    ),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
}

pub(super) fn exact_line_stroke_band(
    line: LineSeg2,
    distance: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<CurveRegionBoundaryLoop2>> {
    let source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line));
    let left = match exact_offset_spans_from_materialized_curve(&source, distance, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(mut spans) if spans.len() == 1 => {
            spans.pop().expect("a line offset produces one exact span")
        }
        Classification::Decided(_) => {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology("a line offset split into multiple source branches".into()),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let right_distance = -distance.clone();
    let right = match exact_offset_spans_from_materialized_curve(&source, &right_distance, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(mut spans) if spans.len() == 1 => {
            spans.pop().expect("a line offset produces one exact span")
        }
        Classification::Decided(_) => {
            return Err(curve_region_edit_error(
                CurveOperation2::Offset,
                CurveError::Topology("a line offset split into multiple source branches".into()),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let boundary = match exact_offset_span_band_loop(&right, &left, None, None, policy)
        .map_err(|cause| curve_region_edit_error(CurveOperation2::Offset, cause))?
    {
        Classification::Decided(boundary) => boundary,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(boundary))
}

pub(super) fn exact_offset_parallel_endpoint(
    span: &ExactOffsetSpan2,
    at_start: bool,
) -> Option<(&BezierParallel2, Real, bool)> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    match fragment? {
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let parameter = if at_start {
                analytic_parallel_traversal_start(fragment)
            } else {
                analytic_parallel_traversal_end(fragment)
            }
            .scalar()?
            .clone();
            Some((fragment.parallel(), parameter, fragment.is_reversed()))
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let parallel = fragment.analytic_parallel()?;
            let parameter = if at_start != fragment.is_reversed() {
                fragment.range().start()
            } else {
                fragment.range().end()
            }
            .as_bezier_parameter()?
            .scalar()?
            .clone();
            Some((parallel, parameter, fragment.is_reversed()))
        }
        BezierSplitFragment2::Materialized { .. }
        | BezierSplitFragment2::RetainedBezier { .. }
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
    }
}

pub(super) fn exact_offset_parallel_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    point: &Point2,
) -> Option<crate::rational_bezier::RationalQuadraticParallelCircleContact2> {
    let (parallel, parameter, _) = exact_offset_parallel_endpoint(span, at_start)?;
    Some(
        crate::rational_bezier::RationalQuadraticParallelCircleContact2 {
            parallel: parallel.clone(),
            parameter,
            point: point.clone(),
            eliminant_root_multiplicity: 2,
        },
    )
}

pub(super) fn exact_offset_parallel_line_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    line_endpoint: BezierEndpoint,
) -> Option<BezierParallelLineTangentContact2> {
    let (parallel, parameter, reversed) = exact_offset_parallel_endpoint(span, at_start)?;
    Some(BezierParallelLineTangentContact2::new(
        parallel.clone(),
        parameter,
        line_endpoint,
        reversed,
    ))
}

pub(super) fn exact_offset_line_tangent_contact(
    span: &ExactOffsetSpan2,
    at_start: bool,
    point: &Point2,
) -> Option<crate::rational_bezier::RationalQuadraticCircleTangentContact2> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(curve),
        ..
    } = fragment?
    else {
        return None;
    };
    Some(
        crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
            line: curve.retained_exact_line_image()?.clone(),
            point: point.clone(),
        },
    )
}

/// Reuses the retained fillet-circle constructor for a round offset join
/// anchored by one algebraic analytic-parallel endpoint.
///
/// The anchor's original carrier locates the source vertex, while the exact
/// difference between its composed and original parallel distances is the
/// authored round radius. Selected circular and chord companions reuse the
/// certified normal-contact parameters used by fillets. Tangent relations
/// select the chart before the common retained circle publication.
pub(super) fn append_retained_parallel_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<()>>> {
    let previous_fragment = previous.fragments.last()?;
    let next_fragment = next.fragments.first()?;
    let (
        anchor_is_previous,
        parallel,
        source_parallel,
        parameter,
        selected_source_parameter,
        source_direction,
        companion,
        companion_at_start,
    ) = match (
        &previous.end_tangent,
        previous_fragment,
        &next.start_tangent,
        next_fragment,
    ) {
        (
            Some(CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                parameter,
                selected_source_parameter,
                source_direction,
                ..
            }),
            BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_),
            _,
            companion @ (BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::AlgebraicChord(_)),
        ) => (
            true,
            parallel,
            source_parallel,
            parameter,
            selected_source_parameter.as_ref(),
            *source_direction,
            companion,
            true,
        ),
        (
            _,
            companion @ (BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::AlgebraicChord(_)),
            Some(CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                parameter,
                selected_source_parameter,
                source_direction,
                ..
            }),
            BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_),
        ) => (
            false,
            parallel,
            source_parallel,
            parameter,
            selected_source_parameter.as_ref(),
            *source_direction,
            companion,
            false,
        ),
        _ => return None,
    };
    let (anchor_tangent, companion_tangent) = if anchor_is_previous {
        (previous.end_tangent.as_ref()?, next.start_tangent.as_ref()?)
    } else {
        (next.start_tangent.as_ref()?, previous.end_tangent.as_ref()?)
    };
    if matches!(companion, BezierSplitFragment2::AlgebraicChord(_))
        && !matches!(companion_tangent, CurveTangent2::AlgebraicChord(_))
    {
        return None;
    }
    Some((|| {
        let radial_distance = parallel.distance() - source_parallel.distance();
        match is_zero(
            &(&radial_distance * &radial_distance - distance * distance),
            policy,
        ) {
            Some(true) => {}
            Some(false) => {
                return Err(CurveError::Topology(
                    "a retained round-join frame disagreed with its offset distance".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let fillet_clockwise = if anchor_is_previous {
            clockwise
        } else {
            !clockwise
        };
        let fillet = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
            source_parallel.clone(),
            parameter.clone().into(),
            radial_distance,
            fillet_clockwise,
            policy,
        )? {
            Classification::Decided(Some(fillet)) => fillet,
            Classification::Decided(None) => {
                return Err(CurveError::Topology(
                    "a nonzero retained round join collapsed".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let anchor_cross_companion =
            match curve_tangent_cross_sign(anchor_tangent, companion_tangent, policy) {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let sweep_halves = match (fillet_clockwise, anchor_cross_companion) {
            (false, RealSign::Positive) | (true, RealSign::Negative) => 1_u8,
            (false, RealSign::Negative) | (true, RealSign::Positive) => 2_u8,
            (_, RealSign::Zero) => {
                match curve_tangents_are_opposite(anchor_tangent, companion_tangent, policy) {
                    Classification::Decided(true) => 1_u8,
                    Classification::Decided(false) => {
                        return Err(CurveError::Topology(
                            "distinct round-join endpoints retained the same oriented tangent"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let terminal_circle = if sweep_halves == 2 {
            fillet.complementary_half()
        } else {
            fillet.clone()
        };
        let other_point = if anchor_is_previous {
            next.offset_start.clone()
        } else {
            previous.offset_end.clone()
        };
        let contact_parameter = if anchor_cross_companion == RealSign::Zero {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        } else {
            let terminal_radial_sign = match real_sign(terminal_circle.radial_distance(), policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a retained round terminal circle collapsed".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let distance_sign = match real_sign(distance, policy) {
                Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Some(RealSign::Zero) => return Ok(Classification::Decided(())),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            };
            let radial_product_sign = exact_sign_product(
                exact_sign_product(terminal_radial_sign, source_direction),
                distance_sign,
            );
            let contact = match companion {
                BezierSplitFragment2::AlgebraicCuspSemicircle(companion) => terminal_circle
                    .certified_selected_circular_tangent_contact_parameter(
                        companion.clone(),
                        companion_at_start,
                        parallel.clone(),
                        selected_source_parameter.map_or_else(
                            || CurveParameter2::from(parameter.clone()),
                            |parameter| CurveParameter2::from_selected_fiber(parameter.clone()),
                        ),
                        source_direction,
                        radial_product_sign,
                        other_point,
                        policy,
                    )?,
                BezierSplitFragment2::AlgebraicChord(_) => {
                    let CurveTangent2::AlgebraicChord(chord) = companion_tangent else {
                        unreachable!("retained chord tangent checked above")
                    };
                    terminal_circle.certified_selected_chord_parallel_normal_contact_parameter(
                        chord.clone(),
                        other_point,
                        distance.clone(),
                        anchor_cross_companion,
                        policy,
                    )?
                }
                _ => unreachable!("retained round-join companion checked above"),
            };
            match contact {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let exact_zero =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
        let exact_one =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
        let mut inserted = if sweep_halves == 1 {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero,
                    contact_parameter,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
            ]
        } else {
            vec![
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    fillet,
                    exact_zero.clone(),
                    exact_one,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    terminal_circle,
                    exact_zero,
                    contact_parameter,
                    false,
                    policy,
                )
                .with_certified_tangent_endpoints(),
            ]
        };
        if !anchor_is_previous {
            inserted = inserted
                .into_iter()
                .rev()
                .map(|fragment| fragment.reversed())
                .collect();
        }
        fragments.extend(
            inserted
                .into_iter()
                .map(BezierSplitFragment2::AlgebraicCuspSemicircle),
        );
        Ok(Classification::Decided(()))
    })())
}

pub(super) fn append_exact_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    // Both endpoints were constructed from this vertex with the same signed
    // left-normal distance. The already-certified outer turn therefore fixes
    // both the traversal orientation and the fact that this is the minor arc;
    // do not recompute a potentially wide radical radial cross-product.
    let clockwise = match real_sign(distance, policy) {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative) => false,
        Some(RealSign::Zero) => return Ok(Classification::Decided(())),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    if sweep_kind == crate::arc_bezier::ArcSweepKind::Minor
        && let Some(result) = append_retained_parallel_round_join(
            fragments, previous, next, distance, clockwise, policy,
        )
    {
        return result;
    }
    if let (Some(previous_offset_end), Some(next_offset_start), Some(center)) = (
        previous.offset_end.coordinates(),
        next.offset_start.coordinates(),
        previous.source_end.coordinates(),
    ) {
        let radius_squared = distance * distance;
        let arc = CircularArc2::new_with_certified_radius_and_sweep(
            previous_offset_end.clone(),
            next_offset_start.clone(),
            center.clone(),
            radius_squared.clone(),
            clockwise,
            sweep_kind,
        );
        let decomposition = match arc.rational_bezier_decomposition_with_policy(policy) {
            Ok(Classification::Decided(decomposition)) => decomposition,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        let mut parallel_contacts = [
            exact_offset_parallel_tangent_contact(previous, false, previous_offset_end),
            exact_offset_parallel_tangent_contact(next, true, next_offset_start),
        ]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
        if sweep_kind == crate::arc_bezier::ArcSweepKind::Semicircle
            && parallel_contacts.len() == 2
            && parallel_contacts[0].parallel.source() == parallel_contacts[1].parallel.source()
            && parallel_contacts[0].parameter == parallel_contacts[1].parameter
        {
            // A semicircle between the two limiting sides of one analytic cusp is
            // centered at the cusp parallel. Each neighboring parallel therefore
            // has this circle as its osculating circle, certifying one additional
            // eliminant factor beyond ordinary tangency.
            for contact in &mut parallel_contacts {
                contact.eliminant_root_multiplicity = 3;
            }
        }
        let tangent_contacts = parallel_contacts
            .into_iter()
            .map(crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel)
            .chain(
                [
                    exact_offset_line_tangent_contact(previous, false, previous_offset_end),
                    exact_offset_line_tangent_contact(next, true, next_offset_start),
                ]
                .into_iter()
                .flatten(),
            )
            .collect::<Vec<_>>();
        let circular_conic = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
            center: center.clone(),
            radius_squared,
            tangent_contacts: (!tangent_contacts.is_empty()).then(|| Arc::from(tangent_contacts)),
        });
        fragments.extend(decomposition.spans().iter().map(|span| {
            let curve = span.curve().clone().with_retained_conic_provenance(
                span.curve().retained_implicit_quadratic_conic().cloned(),
                Some(Arc::clone(&circular_conic)),
            );
            materialized_offset_fragment(BezierSubcurve2::RationalQuadratic(curve))
        }));
        return Ok(Classification::Decided(()));
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let (
        Some(CurveTangent2::AlgebraicChord(anchor)),
        Some(CurveTangent2::AlgebraicChord(chord)),
    ) = (previous.end_tangent.as_ref(), next.start_tangent.as_ref())
    {
        return append_selected_chord_pair_round_join(
            fragments, previous, next, distance, clockwise, sweep_kind, anchor, chord, policy,
        );
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let (Some(previous_tangent), Some(next_tangent)) =
        (previous.end_tangent.as_ref(), next.start_tangent.as_ref())
    {
        if let CurveTangent2::SelectedCircularEndpoint {
            fragment, at_start, ..
        } = previous_tangent
            && let CurveTangent2::AlgebraicChord(chord) = next_tangent
        {
            return append_selected_circle_chord_round_join(
                fragments, previous, next, distance, clockwise, sweep_kind, chord, fragment,
                *at_start, false, policy,
            );
        }
        if let CurveTangent2::AlgebraicChord(chord) = previous_tangent
            && let CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            } = next_tangent
        {
            return append_selected_circle_chord_round_join(
                fragments, previous, next, distance, clockwise, sweep_kind, chord, fragment,
                *at_start, true, policy,
            );
        }
    }

    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) {
        let normal_join = match (previous.end_tangent.as_ref(), next.start_tangent.as_ref()) {
            (
                Some(CurveTangent2::RepresentedDirection(direction)),
                Some(CurveTangent2::AlgebraicChord(chord)),
            ) => Some((direction, chord, false)),
            (
                Some(CurveTangent2::AlgebraicChord(chord)),
                Some(CurveTangent2::RepresentedDirection(direction)),
            ) => Some((direction, chord, true)),
            _ => None,
        };
        if let Some((direction, chord, reversed)) = normal_join {
            let unit_direction = match crate::direction::UnitDirection2::from_direction(direction)?
            {
                Classification::Decided(direction) => direction,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let represented =
                match crate::BezierAlgebraicChord2::from_unit_direction(&unit_direction, policy)? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let (previous_direction, next_direction) = if reversed {
                (chord, &represented)
            } else {
                (&represented, chord)
            };
            return append_selected_chord_pair_round_join(
                fragments,
                previous,
                next,
                distance,
                clockwise,
                sweep_kind,
                previous_direction,
                next_direction,
                policy,
            );
        }
    }

    // A retained one-field center plus the previous exact unit tangent defines
    // the same selected circular carrier used by analytic cusp and fillet
    // paths. A regular outer corner retains its first half; crossing a local
    // collapse radius retains the complete semicircle.
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let Some(CurveTangent2::RepresentedDirection((tangent_x, tangent_y))) =
        previous.end_tangent.as_ref()
    {
        let unit_residual = tangent_x * tangent_x + tangent_y * tangent_y - Real::one();
        if is_zero(&unit_residual, &CurveContext::STRICT) == Some(true) {
            let semicircle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
                &previous.source_end,
                (-tangent_y.clone(), tangent_x.clone()),
                distance.clone(),
                clockwise,
                policy,
            )? {
                Classification::Decided(Some(semicircle)) => semicircle,
                Classification::Decided(None) => return Ok(Classification::Decided(())),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let quadrant_minor = match next.start_tangent.as_ref() {
                Some(CurveTangent2::RepresentedDirection((next_x, next_y))) => {
                    is_zero(
                        &(tangent_x * next_x + tangent_y * next_y),
                        &CurveContext::STRICT,
                    ) == Some(true)
                }
                _ => false,
            };
            let end_parameter = match sweep_kind {
                crate::arc_bezier::ArcSweepKind::Minor if quadrant_minor => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        (Real::one() / Real::from(2_i8))?,
                    )
                }
                crate::arc_bezier::ArcSweepKind::Minor => {
                    // The selected carrier starts at the previous offset
                    // endpoint, but a general corner need not turn through a
                    // quadrant. Intersect its exact endpoint chord with that
                    // carrier and reuse the existing circle/chord authority
                    // to name the actual minor-arc cut.
                    let (endpoint_chord, endpoint_chord_parameter) = if let Some(
                        BezierSplitFragment2::AlgebraicChord(chord),
                    ) = next.fragments.first()
                    {
                        (chord.clone(), chord.start_parameter())
                    } else {
                        let chord = match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                                previous.offset_end.clone(),
                                next.offset_start.clone(),
                                policy,
                            )? {
                                Classification::Decided(chord) => chord,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                        let endpoint = chord.end_parameter();
                        (chord, endpoint)
                    };
                    let contacts = match semicircle.chord_intersections(&endpoint_chord, policy)? {
                        Classification::Decided(contacts) if contacts.is_empty() => {
                            return Err(CurveError::Topology(
                                "an authored round-join endpoint missed its selected circle".into(),
                            ));
                        }
                        Classification::Decided(contacts) => contacts,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let mut endpoint_parameter = None;
                    let mut uncertainty = None;
                    for contact in contacts {
                        match contact
                            .chord_parameter
                            .cmp_by_refinement(&endpoint_chord_parameter, policy)?
                        {
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                if endpoint_parameter.replace(contact.cusp_parameter).is_some() {
                                    return Err(CurveError::Topology(
                                        "an authored round-join endpoint had duplicate selected-circle contacts"
                                            .into(),
                                    ));
                                }
                            }
                            Classification::Decided(_) => {}
                            Classification::Uncertain(reason) => {
                                uncertainty.get_or_insert(reason);
                            }
                        }
                    }
                    match endpoint_parameter {
                        Some(parameter) => parameter,
                        None => {
                            if let Some(reason) = uncertainty {
                                return Ok(Classification::Uncertain(reason));
                            }
                            return Err(CurveError::Topology(
                                "an authored round-join endpoint was absent from its selected-circle contact set"
                                    .into(),
                            ));
                        }
                    }
                }
                crate::arc_bezier::ArcSweepKind::Semicircle => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
                crate::arc_bezier::ArcSweepKind::Major
                | crate::arc_bezier::ArcSweepKind::FullCircle => {
                    unreachable!("the retained unit-frame round path admits at most one semicircle")
                }
            };
            let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                semicircle,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                end_parameter,
                false,
                policy,
            )? {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)]
            {
                match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(CurveError::Topology(
                            "retained selected semicircle join missed its certified endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
                fragment.with_certified_tangent_endpoints(),
            ));
            return Ok(Classification::Decided(()));
        }
    }
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Minor | crate::arc_bezier::ArcSweepKind::Semicircle
    ) && let Some((previous_tangent, next_tangent)) = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref())
        && let (Some(previous_support), Some(next_support)) = (
            exact_offset_retained_tangent_support(previous_tangent, &previous.offset_end, policy),
            exact_offset_retained_tangent_support(next_tangent, &next.offset_start, policy),
        )
    {
        let anchor = match previous_support? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let companion = match next_support? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        // Both supports retain their oriented endpoint directions. The common
        // chord-normal frame also covers parallel cusp joins whose exact
        // endpoints remain procedural, without a separate circular carrier.
        return append_selected_chord_pair_round_join(
            fragments, previous, next, distance, clockwise, sweep_kind, &anchor, &companion, policy,
        );
    }
    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn append_selected_circle_chord_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    chord: &crate::BezierAlgebraicChord2,
    selected_circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
    selected_at_start: bool,
    anchor_is_previous: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    // Use the algebraic chord as the local unit frame. Its offset endpoint is
    // parameter zero on the join circle; reversing the selected half when the
    // chord follows the circular span restores the requested boundary order.
    // The other authored endpoint is then selected by the same authoritative
    // selected-circle/chord kernel used everywhere else.
    let (other_point, fillet_clockwise, reversed) = if anchor_is_previous {
        (&next.offset_start, clockwise, false)
    } else {
        (&previous.offset_end, !clockwise, true)
    };
    let center = if anchor_is_previous {
        chord.end().clone()
    } else {
        chord.start().clone()
    };
    let fillet = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
        center,
        chord.clone(),
        distance.clone(),
        fillet_clockwise,
        policy,
    )? {
        Classification::Decided(Some(fillet)) => fillet,
        Classification::Decided(None) => return Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_parameter = match sweep_kind {
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        }
        crate::arc_bezier::ArcSweepKind::Minor => {
            let tangent = match selected_circle.endpoint_tangent_chord(selected_at_start, policy)? {
                Classification::Decided(Some(tangent)) => tangent,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match fillet.certified_chord_normal_contact_parameter(
                crate::bezier_offset::BezierSelectedChordNormalAnchor2::RetainedChord(
                    chord.clone(),
                ),
                tangent,
                other_point.clone(),
                distance.clone(),
                true,
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle => {
            unreachable!("an exact offset round join admits at most one semicircle")
        }
    };
    let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        fillet,
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
        end_parameter,
        reversed,
        policy,
    )? {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)] {
        if at_start == anchor_is_previous {
            // Parameter zero was constructed from this exact chord endpoint
            // and its signed left-normal offset. That shared carrier is the
            // incidence certificate; comparing its independently wrapped
            // center point would only rebuild the same selected fields.
            continue;
        }
        match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "a retained selected-circle/chord round join missed its certified endpoint"
                        .into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        "selected-circle-chord-round",
    );
    fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
        fragment.with_certified_tangent_endpoints(),
    ));
    Ok(Classification::Decided(()))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn append_selected_chord_pair_round_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    clockwise: bool,
    sweep_kind: crate::arc_bezier::ArcSweepKind,
    anchor: &crate::BezierAlgebraicChord2,
    chord: &crate::BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let represented_unit_tangent = anchor.certified_unit_tangent().or_else(|| {
        anchor
            .certified_axis_direction()
            .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
    });
    let chord_frame = || {
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
            previous.source_end.clone(),
            anchor.clone(),
            distance.clone(),
            clockwise,
            policy,
        )
    };
    let (selected, represented_frame) = if let Some((tangent_x, tangent_y)) =
        represented_unit_tangent.as_ref()
    {
        let represented = crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_certified_unit_normal(
            &previous.source_end,
            (-tangent_y.clone(), tangent_x.clone()),
            distance.clone(),
            clockwise,
            policy,
        )?;
        match represented {
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                // A represented circle frame owns one algebraic center field.
                // Exact and other retained centers instead share the anchor's
                // general chord-normal frame without projecting coordinates.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "selected-chord-pair-round-chord-frame-fallback",
                );
                (chord_frame()?, false)
            }
            represented => (represented, true),
        }
    } else {
        (chord_frame()?, false)
    };
    let semicircle = match selected {
        Classification::Decided(Some(semicircle)) => semicircle,
        Classification::Decided(None) => return Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match sweep_kind {
        crate::arc_bezier::ArcSweepKind::Minor => {
            let selected = match (represented_frame, represented_unit_tangent) {
                (true, Some(anchor_tangent)) => semicircle
                    .certified_chord_normal_contact_parameter(
                        crate::bezier_offset::BezierSelectedChordNormalAnchor2::Represented(
                            anchor_tangent,
                        ),
                        chord.clone(),
                        next.offset_start.clone(),
                        distance.clone(),
                        false,
                        policy,
                    )?,
                (false, _) => semicircle.certified_chord_normal_contact_parameter(
                    crate::bezier_offset::BezierSelectedChordNormalAnchor2::RetainedChord(
                        anchor.clone(),
                    ),
                    chord.clone(),
                    next.offset_start.clone(),
                    distance.clone(),
                    false,
                    policy,
                )?,
                (true, None) => unreachable!("a represented frame has a represented tangent"),
            };
            match selected {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
        }
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle => {
            unreachable!("an exact offset round join admits at most one semicircle")
        }
    };
    let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        semicircle,
        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
        end,
        false,
        policy,
    )? {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for (at_start, expected) in [(true, &previous.offset_end), (false, &next.offset_start)] {
        match fragment.certify_and_cache_authored_endpoint(at_start, expected, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "retained chord-pair round join missed its certified endpoint".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        "selected-chord-pair-round",
    );
    fragments.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
        fragment.with_certified_tangent_endpoints(),
    ));
    Ok(Classification::Decided(()))
}

pub(super) fn append_exact_line_join_with_parallel_tangencies(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &Point2,
    to: &Point2,
    parallel_tangent_contacts: Vec<BezierParallelLineTangentContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match is_zero(&from.distance_squared(to), policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => {
            let line = LineSeg2::try_new(from.clone(), to.clone())?;
            let curve = if parallel_tangent_contacts.is_empty() {
                QuadraticBezier2::from_line_segment(line)
            } else {
                QuadraticBezier2::from_line_segment_with_parallel_tangent_contacts(
                    line,
                    parallel_tangent_contacts,
                )
            };
            fragments.push(materialized_offset_fragment(BezierSubcurve2::Quadratic(
                curve,
            )));
            Ok(Classification::Decided(()))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

pub(super) fn append_exact_miter_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    limit: Option<&Real>,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let represented_vector_frame = matches!(
        (&previous.end_tangent, &next.start_tangent),
        (
            Some(CurveTangent2::RepresentedDirection(_)),
            Some(CurveTangent2::RepresentedDirection(_))
        )
    ) && previous.offset_end.coordinates().is_some()
        && next.offset_start.coordinates().is_some()
        && previous.source_end.coordinates().is_some();
    if !represented_vector_frame
        && let Some(result) = append_retained_support_miter_join(
            fragments,
            previous,
            next,
            distance,
            limit,
            turn_sign,
            distance_sign,
            policy,
        )
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "retained-support",
        );
        return result;
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-miter",
        "represented-vector-fallback",
    );
    let (
        Some(CurveTangent2::RepresentedDirection(previous_tangent)),
        Some(CurveTangent2::RepresentedDirection(next_tangent)),
    ) = (&previous.end_tangent, &next.start_tangent)
    else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "non-vector-tangent",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let (Some(previous_offset_end), Some(next_offset_start), Some(source_vertex)) = (
        previous.offset_end.coordinates(),
        next.offset_start.coordinates(),
        previous.source_end.coordinates(),
    ) else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-miter",
            "non-represented-point",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let denominator = if offset_vectors_are_structurally_opposite(previous_tangent, next_tangent) {
        Real::zero()
    } else {
        offset_vector_cross(previous_tangent, next_tangent)
    };
    let denominator_sign = real_sign(&denominator, policy);
    let Some(RealSign::Positive | RealSign::Negative) = denominator_sign else {
        return match denominator_sign {
            Some(RealSign::Zero) => append_exact_line_join_with_parallel_tangencies(
                fragments,
                previous_offset_end,
                next_offset_start,
                // Equal tangents make the endpoints coincide; opposite
                // tangents connect across their normals. Neither surviving
                // line is an analytic-parallel tangent leg.
                Vec::new(),
                policy,
            ),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            Some(RealSign::Positive | RealSign::Negative) => unreachable!(),
        };
    };
    let denominator_reciprocal = denominator.inverse_ref_assuming_nonzero()?;
    let delta = next_offset_start.delta_from(previous_offset_end);
    let numerator = &delta.0 * &next_tangent.1 - &delta.1 * &next_tangent.0;
    let parameter = numerator * denominator_reciprocal;
    let miter = previous_offset_end.translated(
        &previous_tangent.0 * &parameter,
        &previous_tangent.1 * parameter,
    );
    if let Some(limit) = limit {
        let miter_distance_squared = miter.distance_squared(source_vertex);
        let maximum_squared = distance * distance * limit * limit;
        match compare_reals(&miter_distance_squared, &maximum_squared, policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Some(std::cmp::Ordering::Greater) => {
                return append_exact_line_join_with_parallel_tangencies(
                    fragments,
                    previous_offset_end,
                    next_offset_start,
                    // A rejected miter becomes the transverse bevel joining
                    // the two offset endpoints, not either tangent ray.
                    Vec::new(),
                    policy,
                );
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    let previous_contacts =
        exact_offset_parallel_line_tangent_contact(previous, false, BezierEndpoint::Start)
            .into_iter()
            .collect::<Vec<_>>();
    match append_exact_line_join_with_parallel_tangencies(
        fragments,
        previous_offset_end,
        &miter,
        previous_contacts,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let next_contacts = exact_offset_parallel_line_tangent_contact(next, true, BezierEndpoint::End)
        .into_iter()
        .collect::<Vec<_>>();
    append_exact_line_join_with_parallel_tangencies(
        fragments,
        &miter,
        next_offset_start,
        next_contacts,
        policy,
    )
}

pub(super) fn exact_retained_parallel_represented_tangent(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(Real, Real)>>> {
    let Some(parameter) = parameter.scalar() else {
        return Ok(Classification::Decided(None));
    };
    let tangent = match parallel.source_tangent_at(parameter, policy)? {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some(match source_direction {
        RealSign::Positive => tangent,
        RealSign::Negative => (-tangent.0, -tangent.1),
        RealSign::Zero => {
            return Err(CurveError::Topology(
                "a retained parallel tangent had zero traversal direction".into(),
            ));
        }
    })))
}

pub(super) fn exact_offset_retained_tangent_support(
    tangent: &CurveTangent2,
    endpoint: &CurvePoint2,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<crate::BezierAlgebraicChord2>>> {
    match tangent {
        CurveTangent2::RepresentedDirection(vector) => Some((|| {
            let displaced = match crate::BezierAlgebraicChord2::translated_endpoint(
                endpoint, &vector.0, &vector.1, policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::RetainedParallel {
            parallel,
            parameter,
            source_direction,
            ..
        } => Some((|| {
            match exact_retained_parallel_represented_tangent(
                parallel,
                parameter,
                *source_direction,
                policy,
            )? {
                Classification::Decided(Some(vector)) => {
                    let displaced = match crate::BezierAlgebraicChord2::translated_endpoint(
                        endpoint, &vector.0, &vector.1, policy,
                    )? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    return crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        endpoint.clone(),
                        displaced,
                        policy,
                    );
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let tangent_distance = Real::from(match source_direction {
                RealSign::Positive => 1_i8,
                RealSign::Negative => -1_i8,
                RealSign::Zero => {
                    return Err(CurveError::Topology(
                        "a retained miter tangent had zero traversal direction".into(),
                    ));
                }
            });
            let displaced = CurvePoint2::from(
                crate::BezierAnalyticParallelPoint2::new_with_tangent_distance(
                    parallel.clone(),
                    parameter.clone(),
                    tangent_distance,
                    policy,
                ),
            );
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::AlgebraicChord(chord) => Some(Ok(Classification::Decided(chord.clone()))),
        CurveTangent2::SelectedCircularEndpoint {
            fragment, at_start, ..
        }
        | CurveTangent2::ChordContact {
            fragment, at_start, ..
        } => Some((|| {
            let displaced = match fragment.endpoint_tangent_support_point(*at_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                endpoint.clone(),
                displaced,
                policy,
            )
        })()),
        CurveTangent2::CircularPoint { .. } => None,
    }
}

pub(super) fn exact_offset_span_retained_tangent_support(
    span: &ExactOffsetSpan2,
    at_start: bool,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<(crate::BezierAlgebraicChord2, bool)>>> {
    let fragment = if at_start {
        span.fragments.first()
    } else {
        span.fragments.last()
    };
    if let Some(BezierSplitFragment2::AlgebraicChord(chord)) = fragment {
        // The offset span itself is the authoritative finite subset of this
        // tangent support. Reuse it so a later miter intersection retains the
        // same support identity used by Boolean regularization.
        return Some(Ok(Classification::Decided((chord.clone(), !at_start))));
    }
    // A carrier may collapse its finite offset image while still retaining an
    // authored endpoint tangent for the adjacent inner join.  The tangent is
    // the supporting-line authority; absence of an emitted finite fragment
    // must not suppress it before the complete retained miter path runs.
    let tangent = if at_start {
        span.start_tangent.as_ref()
    } else {
        span.end_tangent.as_ref()
    }?;
    let endpoint = if at_start {
        &span.offset_start
    } else {
        &span.offset_end
    };
    exact_offset_retained_tangent_support(tangent, endpoint, policy)
        .map(|result| result.map(|classification| classification.map(|chord| (chord, false))))
}

pub(super) fn append_retained_support_miter_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    limit: Option<&Real>,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> Option<CurveResult<Classification<()>>> {
    let previous_support = exact_offset_span_retained_tangent_support(previous, false, policy)?;
    let next_support = exact_offset_span_retained_tangent_support(next, true, policy)?;
    Some((|| {
        let previous_support = match previous_support {
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "previous-support",
                );
                return Err(error);
            }
            Ok(classification) => match classification {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-miter-blocker",
                        "previous-support",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let next_support = match next_support {
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "next-support",
                );
                return Err(error);
            }
            Ok(classification) => match classification {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-miter-blocker",
                        "next-support",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let (previous_support, previous_anchor_at_end) = previous_support;
        let (next_support, next_anchor_at_end) = next_support;
        let displacement_sign = exact_sign_product(turn_sign, distance_sign);
        let first_order = match displacement_sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Negative => std::cmp::Ordering::Greater,
            RealSign::Zero => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
        let second_order = first_order.reverse();
        let support_intersection = match previous_support
            .supporting_line_intersection_with_certified_anchor_orders(
                &next_support,
                previous_anchor_at_end,
                first_order,
                next_anchor_at_end,
                second_order,
                policy,
            ) {
            Ok(intersection) => intersection,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "support-intersection",
                );
                return Err(error);
            }
        };
        let miter = match support_intersection {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter",
                    "parallel-supports",
                );
                return append_exact_algebraic_line_join(
                    fragments,
                    &previous.offset_end,
                    &next.offset_start,
                    None,
                    None,
                    true,
                    [false; 2],
                    policy,
                );
            }
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-blocker",
                    "support-intersection",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(limit) = limit {
            let maximum_squared = distance * distance * limit * limit;
            let strict_policy = policy.strict_counterpart();
            let within_limit = match crate::bezier_offset::algebraic_point_distance_squared_at_most(
                &miter,
                &previous.source_end,
                &maximum_squared,
                &strict_policy,
            ) {
                Classification::Decided(within_limit) => Classification::Decided(within_limit),
                Classification::Uncertain(_) => {
                    if let (
                        Some(CurveTangent2::RepresentedDirection(previous_tangent)),
                        Some(CurveTangent2::RepresentedDirection(next_tangent)),
                    ) = (&previous.end_tangent, &next.start_tangent)
                    {
                        // For unit traversal tangents u and v, the squared distance
                        // from their shared source vertex to the equal-distance
                        // offset-line intersection is d²·2/(1 + u·v).  Comparing
                        // 2 <= limit²·(1 + u·v) therefore decides the authored miter
                        // limit without adjoining either retained endpoint field.
                        let previous_norm_squared = &previous_tangent.0 * &previous_tangent.0
                            + &previous_tangent.1 * &previous_tangent.1;
                        let next_norm_squared =
                            &next_tangent.0 * &next_tangent.0 + &next_tangent.1 * &next_tangent.1;
                        match (
                            is_zero(&(previous_norm_squared - Real::one()), &strict_policy),
                            is_zero(&(next_norm_squared - Real::one()), &strict_policy),
                        ) {
                            (Some(true), Some(true)) => {
                                let one_plus_dot = Real::one()
                                    + &previous_tangent.0 * &next_tangent.0
                                    + &previous_tangent.1 * &next_tangent.1;
                                match real_sign(&one_plus_dot, &strict_policy) {
                                    Some(RealSign::Positive) => {
                                        let threshold = limit * limit * one_plus_dot;
                                        match compare_reals(
                                            &Real::from(2_i8),
                                            &threshold,
                                            &strict_policy,
                                        ) {
                                            Some(
                                                std::cmp::Ordering::Less
                                                | std::cmp::Ordering::Equal,
                                            ) => Classification::Decided(true),
                                            Some(std::cmp::Ordering::Greater) => {
                                                Classification::Decided(false)
                                            }
                                            None => Classification::Uncertain(
                                                UncertaintyReason::Ordering,
                                            ),
                                        }
                                    }
                                    Some(RealSign::Zero) => {
                                        Classification::Uncertain(UncertaintyReason::Boundary)
                                    }
                                    Some(RealSign::Negative) => {
                                        return Err(CurveError::Topology(
                                            "unit miter tangents had a dot product below minus one"
                                                .into(),
                                        ));
                                    }
                                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                                }
                            }
                            (Some(false), _) | (_, Some(false)) => {
                                Classification::Uncertain(UncertaintyReason::Unsupported)
                            }
                            _ => Classification::Uncertain(UncertaintyReason::RealSign),
                        }
                    } else {
                        Classification::Uncertain(UncertaintyReason::Ordering)
                    }
                }
            };
            let within_limit = match within_limit {
                Classification::Uncertain(reason) if policy.permits_approximate_512() => {
                    match crate::bezier_offset::algebraic_point_distance_squared_at_most(
                        &miter,
                        &previous.source_end,
                        &maximum_squared,
                        policy,
                    ) {
                        decided @ Classification::Decided(_) => decided,
                        Classification::Uncertain(_) => Classification::Uncertain(reason),
                    }
                }
                classification => classification,
            };
            match within_limit {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return append_exact_algebraic_line_join(
                        fragments,
                        &previous.offset_end,
                        &next.offset_start,
                        None,
                        None,
                        true,
                        [
                            previous
                                .end_tangent
                                .as_ref()
                                .is_some_and(exact_offset_tangent_is_selected_circle),
                            next.start_tangent
                                .as_ref()
                                .is_some_and(exact_offset_tangent_is_selected_circle),
                        ],
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let previous_contacts: Vec<_> =
            exact_offset_parallel_line_tangent_contact(previous, false, BezierEndpoint::Start)
                .into_iter()
                .collect();
        let previous_leg = match append_retained_support_miter_leg(
            fragments,
            &previous_support,
            previous.offset_end.clone(),
            miter.clone(),
            previous_contacts,
            policy,
        ) {
            Ok(classification) => classification,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "previous-leg",
                );
                return Err(error);
            }
        };
        match previous_leg {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-blocker",
                    "previous-leg",
                );
                return Ok(Classification::Uncertain(reason));
            }
        }
        let next_contacts: Vec<_> =
            exact_offset_parallel_line_tangent_contact(next, true, BezierEndpoint::End)
                .into_iter()
                .collect();
        let next_leg = match append_retained_support_miter_leg(
            fragments,
            &next_support,
            miter,
            next.offset_start.clone(),
            next_contacts,
            policy,
        ) {
            Ok(classification) => classification,
            Err(error) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-miter-error",
                    "next-leg",
                );
                return Err(error);
            }
        };
        #[cfg(feature = "dispatch-trace")]
        if matches!(next_leg, Classification::Uncertain(_)) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-miter-blocker",
                "next-leg",
            );
        }
        Ok(next_leg)
    })())
}

pub(super) fn append_retained_support_miter_leg(
    fragments: &mut Vec<BezierSplitFragment2>,
    support: &crate::BezierAlgebraicChord2,
    from: CurvePoint2,
    to: CurvePoint2,
    parallel_tangent_contacts: Vec<BezierParallelLineTangentContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match support.chord_between_certified_support_points(from, to, policy)? {
        Classification::Decided(Some(chord)) => {
            let chord = chord.with_parallel_tangent_contacts(parallel_tangent_contacts);
            fragments.push(retained_chord_fragment(chord));
            Ok(Classification::Decided(()))
        }
        Classification::Decided(None) => Ok(Classification::Decided(())),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn offset_vector_cross(first: &(Real, Real), second: &(Real, Real)) -> Real {
    &first.0 * &second.1 - &first.1 * &second.0
}

pub(super) const fn exact_sign_product(first: RealSign, second: RealSign) -> RealSign {
    match (first, second) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            RealSign::Positive
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            RealSign::Negative
        }
    }
}

pub(super) const fn exact_sign_reverse(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
        RealSign::Positive => RealSign::Negative,
    }
}

pub(super) fn exact_circular_tangent_cross_vector(
    point: &CurvePoint2,
    circle: &crate::rational_bezier::RationalQuadraticCircle2,
    clockwise: bool,
    vector: &(Real, Real),
    policy: &CurveContext,
) -> Classification<RealSign> {
    {
        match algebraic_chord_point_linear_order_to_exact(
            point,
            &circle.center,
            &vector.0,
            &vector.1,
            policy,
        ) {
            Ok(Classification::Decided(order)) => {
                let radial_projection_sign = match order {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                };
                let orientation = if clockwise {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                Classification::Decided(exact_sign_product(radial_projection_sign, orientation))
            }
            Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }
}

pub(super) fn exact_algebraic_chord_parallel_factor(
    reference: &crate::BezierAlgebraicChord2,
    candidate: &crate::BezierAlgebraicChord2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    if let (Some(first), Some(second)) = (
        reference.certified_unit_tangent(),
        candidate.certified_unit_tangent(),
    ) {
        let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
        // The represented-unit-vector comparison is only a fast path.  Keep
        // it certified so an unresolved symbolic cancellation can fall
        // through to the retained chord relation below instead of consuming
        // APPROXIMATE_512 before that exact certificate is consulted.
        match policy.strict_predicate_pass(|| real_sign(&cross, policy)) {
            Some(RealSign::Zero) => {
                let dot = Real::dot2_refs([&first.0, &first.1], [&second.0, &second.1]);
                match policy.strict_predicate_pass(|| real_sign(&dot, policy)) {
                    Some(sign @ (RealSign::Negative | RealSign::Positive)) => {
                        return Classification::Decided(sign);
                    }
                    Some(RealSign::Zero) => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    None => {}
                }
            }
            Some(RealSign::Negative | RealSign::Positive) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            None => {}
        }
    }
    match reference.tangent_cross_sign(candidate, policy) {
        Ok(Classification::Decided(RealSign::Zero)) => {}
        Ok(Classification::Decided(RealSign::Negative | RealSign::Positive)) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match reference.tangent_dot_sign(candidate, policy) {
        Ok(Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive))) => {
            Classification::Decided(sign)
        }
        Ok(Classification::Decided(RealSign::Zero)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

pub(super) fn exact_algebraic_chord_vector_factor(
    reference: &crate::BezierAlgebraicChord2,
    candidate: &(Real, Real),
    policy: &CurveContext,
) -> Classification<RealSign> {
    match reference.tangent_cross_vector_sign(candidate, policy) {
        Ok(Classification::Decided(RealSign::Zero)) => {}
        Ok(Classification::Decided(RealSign::Negative | RealSign::Positive)) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match reference.tangent_dot_vector_sign(candidate, policy) {
        Ok(Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive))) => {
            Classification::Decided(sign)
        }
        Ok(Classification::Decided(RealSign::Zero)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

pub(super) fn exact_algebraic_chord_retained_parallel_relation(
    chord: &crate::BezierAlgebraicChord2,
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    cross: bool,
    policy: &CurveContext,
) -> Classification<RealSign> {
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        if cross {
            "curve-region-exact-offset-tangent-cross"
        } else {
            "curve-region-exact-offset-tangent-dot"
        },
        "algebraic-chord-retained-parallel",
    );
    let represented = chord.certified_unit_tangent().or_else(|| {
        chord
            .certified_axis_direction()
            .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
    });
    let relation = if let Some(tangent) = represented {
        match parallel.vector_tangent_cross_and_dot_signs(
            &parameter.clone().into(),
            &tangent.0,
            &tangent.1,
            policy,
        ) {
            Ok(Classification::Decided((cross_sign, dot_sign))) => {
                if cross {
                    cross_sign
                } else {
                    dot_sign
                }
            }
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    } else {
        let zero = Real::zero();
        let one = Real::one();
        match chord.tangent_cross_dot_parallel_linear_combination_sign(
            parallel,
            &parameter.clone().into(),
            if cross { &one } else { &zero },
            if cross { &zero } else { &one },
            policy,
        ) {
            Ok(Classification::Decided(relation)) => relation,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    };
    let parallel_scale =
        match parallel.parallel_derivative_scale_sign(&parameter.clone().into(), policy) {
            Ok(Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative))) => sign,
            Ok(Classification::Decided(RealSign::Zero)) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    Classification::Decided(exact_sign_product(
        relation,
        exact_sign_product(source_direction, parallel_scale),
    ))
}

pub(super) fn exact_retained_parallel_tangent_cross_and_dot_vector(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    source_direction: RealSign,
    vector: &(Real, Real),
    policy: &CurveContext,
) -> Classification<(RealSign, RealSign)> {
    match exact_retained_parallel_represented_tangent(parallel, parameter, source_direction, policy)
    {
        Ok(Classification::Decided(Some(tangent))) => {
            let cross = offset_vector_cross(&tangent, vector);
            let dot = &tangent.0 * &vector.0 + &tangent.1 * &vector.1;
            return match (real_sign(&cross, policy), real_sign(&dot, policy)) {
                (Some(cross), Some(dot)) => Classification::Decided((cross, dot)),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            };
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    let (vector_cross_parallel, vector_dot_parallel) = match parallel
        .vector_tangent_cross_and_dot_signs(&parameter.clone().into(), &vector.0, &vector.1, policy)
    {
        Ok(Classification::Decided(signs)) => signs,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let parallel_scale =
        match parallel.parallel_derivative_scale_sign(&parameter.clone().into(), policy) {
            Ok(Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative))) => sign,
            Ok(Classification::Decided(RealSign::Zero)) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    let factor = exact_sign_product(source_direction, parallel_scale);
    Classification::Decided((
        exact_sign_reverse(exact_sign_product(vector_cross_parallel, factor)),
        exact_sign_product(vector_dot_parallel, factor),
    ))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn exact_retained_parallel_tangent_pair_cross_and_dot(
    first_parallel: &BezierParallel2,
    first_parameter: &BezierParameter2,
    first_direction: RealSign,
    second_parallel: &BezierParallel2,
    second_parameter: &BezierParameter2,
    second_direction: RealSign,
    policy: &CurveContext,
) -> Classification<(RealSign, RealSign)> {
    let first_represented = match exact_retained_parallel_represented_tangent(
        first_parallel,
        first_parameter,
        first_direction,
        policy,
    ) {
        Ok(Classification::Decided(tangent)) => tangent,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let second_represented = match exact_retained_parallel_represented_tangent(
        second_parallel,
        second_parameter,
        second_direction,
        policy,
    ) {
        Ok(Classification::Decided(tangent)) => tangent,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if let (Some(first), Some(second)) = (first_represented, second_represented) {
        let cross = offset_vector_cross(&first, &second);
        let dot = &first.0 * &second.0 + &first.1 * &second.1;
        return match (real_sign(&cross, policy), real_sign(&dot, policy)) {
            (Some(cross), Some(dot)) => Classification::Decided((cross, dot)),
            _ => Classification::Uncertain(UncertaintyReason::RealSign),
        };
    }
    match first_parallel.source_tangent_pair_cross_and_dot_signs(
        first_parameter,
        second_parallel,
        second_parameter,
        policy,
    ) {
        Ok(Classification::Decided((cross, dot))) => {
            let factor = exact_sign_product(first_direction, second_direction);
            Classification::Decided((
                exact_sign_product(cross, factor),
                exact_sign_product(dot, factor),
            ))
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn exact_selected_circle_retained_parallel_tangent_cross_and_dot(
    source_fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    _parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &BezierParameter2,
    selected_source_parameter: Option<
        &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2,
    >,
    source_direction: RealSign,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    let mut cross_uncertainty = None;
    let cross = match fragment.endpoint_tangent_cross_retained_parallel(
        at_start,
        source_parallel,
        parameter,
        source_direction,
        policy,
    ) {
        Ok(Classification::Decided(cross)) => cross,
        Ok(Classification::Uncertain(reason)) => {
            // This is an optional local tangent predicate. Independently
            // represented source carriers can make it inconclusive even when
            // the authored carriers retain an exact positive-length overlap;
            // try that stronger topology certificate below before blocking.
            cross_uncertainty = Some(reason);
            None
        }
        Err(_) => {
            cross_uncertainty = Some(UncertaintyReason::Unsupported);
            None
        }
    };
    if let Some(cross @ (RealSign::Negative | RealSign::Positive)) = cross {
        return Classification::Decided((cross, None));
    }
    if cross == Some(RealSign::Zero) {
        let zero = Real::zero();
        let one = Real::one();
        match fragment.endpoint_tangent_cross_dot_linear_combination_retained_parallel(
            at_start,
            source_parallel,
            parameter,
            source_direction,
            &zero,
            &one,
            policy,
        ) {
            Ok(Classification::Decided(Some(dot))) => {
                return Classification::Decided((RealSign::Zero, Some(dot)));
            }
            Ok(Classification::Decided(None)) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }
    match fragment.endpoint_tangent_dot_retained_parallel_source_overlap(
        source_fragment,
        at_start,
        source_parallel,
        source_range,
        parameter,
        selected_source_parameter,
        source_direction,
        policy,
    ) {
        Ok(Classification::Decided(Some(dot))) => {
            Classification::Decided((RealSign::Zero, Some(dot)))
        }
        Ok(Classification::Decided(None)) => {
            Classification::Uncertain(cross_uncertainty.unwrap_or(UncertaintyReason::Unsupported))
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn exact_selected_circle_pair_tangent_cross_and_dot(
    first_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    match first.endpoint_pair_tangent_cross_and_dot(first_start, second, second_start, policy) {
        Ok(Classification::Decided(Some(relation))) => {
            return Classification::Decided(relation);
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match first.endpoint_pair_tangent_cross_and_dot_source_circle(
        first_source,
        first_start,
        second,
        second_source,
        second_start,
        policy,
    ) {
        Ok(Classification::Decided(Some((cross, dot)))) => {
            Classification::Decided((cross, Some(dot)))
        }
        Ok(Classification::Decided(None)) => {
            let fallback = exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
                first,
                first_start,
                second,
                second_start,
                policy,
            );
            match fallback {
                Classification::Decided(Some((cross, dot))) => {
                    Classification::Decided((cross, dot))
                }
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

/// General exact tangent relation for selected-circle endpoints whose
/// construction maps do not share one specialized pair authority.
///
/// Each endpoint can already publish a certified-distinct point one unit
/// along its traversal tangent. Retaining those two witness chords lets the
/// complete chord predicates sign cross and dot without adjoining either
/// circle center field or materializing a tangent vector.
pub(super) fn exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<Option<(RealSign, Option<RealSign>)>> {
    let first = match first.endpoint_tangent_chord(first_start, policy) {
        Ok(Classification::Decided(Some(chord))) => chord,
        Ok(Classification::Decided(None)) => return Classification::Decided(None),
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let second = match second.endpoint_tangent_chord(second_start, policy) {
        Ok(Classification::Decided(Some(chord))) => chord,
        Ok(Classification::Decided(None)) => return Classification::Decided(None),
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let cross = match first.tangent_cross_sign(&second, policy) {
        Ok(Classification::Decided(cross)) => cross,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if cross != RealSign::Zero {
        return Classification::Decided(Some((cross, None)));
    }
    match first.tangent_dot_sign(&second, policy) {
        Ok(Classification::Decided(dot)) => Classification::Decided(Some((cross, Some(dot)))),
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

/// Signs two retained circular traversal tangents from their current endpoint
/// witnesses when no shared source-frame specialization applies.
///
/// Offset construction can give the same circular carrier either an ordinary
/// endpoint tangent or a chord-contact tangent. Replay shared pair provenance
/// first; unrelated representations still publish exact unit-step tangent
/// chords for the general predicate.
pub(super) fn exact_current_selected_circle_pair_tangent_cross_and_dot(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    first_start: bool,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second_start: bool,
    policy: &CurveContext,
) -> Classification<(RealSign, Option<RealSign>)> {
    match first.endpoint_pair_tangent_cross_and_dot(first_start, second, second_start, policy) {
        Ok(Classification::Decided(Some(relation))) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-pair-tangent",
                "retained-pair",
            );
            return Classification::Decided(relation);
        }
        Ok(Classification::Decided(None)) => {}
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    }
    match exact_selected_circle_pair_tangent_cross_and_dot_by_chords(
        first,
        first_start,
        second,
        second_start,
        policy,
    ) {
        Classification::Decided(Some(relation)) => Classification::Decided(relation),
        Classification::Decided(None) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn exact_offset_tangent_relation_is_opposite(
    relation: Classification<(RealSign, Option<RealSign>)>,
) -> Classification<bool> {
    match relation {
        Classification::Decided((RealSign::Negative | RealSign::Positive, _)) => {
            Classification::Decided(false)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Negative))) => {
            Classification::Decided(true)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Positive))) => {
            Classification::Decided(false)
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Zero))) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        Classification::Decided((RealSign::Zero, None)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn curve_tangent_cross_sign(
    first: &CurveTangent2,
    second: &CurveTangent2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    match (first, second) {
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-vector",
            );
            match real_sign(&offset_vector_cross(first, second), policy) {
                Some(sign) => Classification::Decided(sign),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::RepresentedDirection(vector),
        ) => exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        )
        .map(|(cross, _)| cross),
        (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        )
        .map(|(cross, _)| exact_sign_reverse(cross)),
        (
            CurveTangent2::RetainedParallel {
                parallel: first_parallel,
                parameter: first_parameter,
                source_direction: first_direction,
                ..
            },
            CurveTangent2::RetainedParallel {
                parallel: second_parallel,
                parameter: second_parameter,
                source_direction: second_direction,
                ..
            },
        ) => exact_retained_parallel_tangent_pair_cross_and_dot(
            first_parallel,
            first_parameter,
            *first_direction,
            second_parallel,
            second_parameter,
            *second_direction,
            policy,
        )
        .map(|(cross, _)| cross),
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::AlgebraicChord(second)) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-algebraic-chord",
            );
            if let Some(sign) = first.tangent_cross_sign_with_shared_endpoint(second, policy) {
                return match sign {
                    Ok(sign) => sign,
                    Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
                };
            }
            match first.tangent_cross_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            true,
            policy,
        ),
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::AlgebraicChord(chord),
        ) => exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            true,
            policy,
        )
        .map(exact_sign_reverse),
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::RepresentedDirection(second)) => {
            match first.tangent_cross_vector_sign(second, policy) {
                Ok(sign) => sign,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (CurveTangent2::RepresentedDirection(first), CurveTangent2::AlgebraicChord(second)) => {
            match second.tangent_cross_vector_sign(first, policy) {
                Ok(sign) => sign.map(exact_sign_reverse),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
            CurveTangent2::RepresentedDirection(second),
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, second, policy),
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
        ) => exact_circular_tangent_cross_vector(point, circle, *clockwise, first, policy)
            .map(exact_sign_reverse),
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
        ) => match first.certified_unit_tangent() {
            Some(tangent) => {
                exact_circular_tangent_cross_vector(point, circle, *clockwise, &tangent, policy)
                    .map(exact_sign_reverse)
            }
            None => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        (
            CurveTangent2::CircularPoint {
                point,
                circle,
                clockwise,
            },
            CurveTangent2::AlgebraicChord(second),
        ) => match second.certified_unit_tangent() {
            Some(tangent) => {
                exact_circular_tangent_cross_vector(point, circle, *clockwise, &tangent, policy)
            }
            None => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-endpoint-vector",
            );
            match fragment.endpoint_tangent_cross_vector(*at_start, second, policy) {
                Ok(cross) => cross,
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-selected-circle-endpoint",
            );
            match fragment.endpoint_tangent_cross_vector(*at_start, first, policy) {
                Ok(cross) => cross.map(exact_sign_reverse),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-retained-parallel",
            );
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "retained-parallel-selected-circle",
            );
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            )
            .map(|(cross, _)| exact_sign_reverse(cross))
        }
        (
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
            CurveTangent2::RepresentedDirection(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "circle-chord-contact-vector",
            );
            match exact_algebraic_chord_vector_factor(chord, second, policy) {
                Classification::Decided(factor) => {
                    Classification::Decided(exact_sign_product(*circle_cross_chord, factor))
                }
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_vector(*at_start, second, policy)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "vector-circle-chord-contact",
            );
            match exact_algebraic_chord_vector_factor(chord, first, policy) {
                Classification::Decided(factor) => Classification::Decided(exact_sign_reverse(
                    exact_sign_product(*circle_cross_chord, factor),
                )),
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_vector(*at_start, first, policy)
                    .map(|cross| cross.map(exact_sign_reverse))
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
            CurveTangent2::AlgebraicChord(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "circle-chord-contact-algebraic-chord",
            );
            match exact_algebraic_chord_parallel_factor(chord, second, policy) {
                Classification::Decided(factor) => {
                    Classification::Decided(exact_sign_product(*circle_cross_chord, factor))
                }
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_algebraic_chord(*at_start, second, true, policy)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::ChordContact {
                fragment,
                at_start,
                chord,
                circle_cross_chord,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-circle-chord-contact",
            );
            match exact_algebraic_chord_parallel_factor(chord, first, policy) {
                Classification::Decided(factor) => Classification::Decided(exact_sign_reverse(
                    exact_sign_product(*circle_cross_chord, factor),
                )),
                Classification::Uncertain(_) => fragment
                    .endpoint_tangent_cross_algebraic_chord(*at_start, first, true, policy)
                    .map(|cross| cross.map(exact_sign_reverse))
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::AlgebraicChord(second),
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-algebraic-chord",
            );
            fragment
                .endpoint_tangent_cross_algebraic_chord_from_source(
                    source_fragment,
                    *at_start,
                    second,
                    true,
                    policy,
                )
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        (
            CurveTangent2::AlgebraicChord(first),
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "algebraic-chord-selected-circle",
            );
            fragment
                .endpoint_tangent_cross_algebraic_chord_from_source(
                    source_fragment,
                    *at_start,
                    first,
                    true,
                    policy,
                )
                .map(|cross| cross.map(exact_sign_reverse))
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: first_source,
                fragment: first_fragment,
                at_start: first_start,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: second_source,
                fragment: second_fragment,
                at_start: second_start,
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-pair-contact",
            );
            exact_selected_circle_pair_tangent_cross_and_dot(
                first_source,
                first_fragment,
                *first_start,
                second_source,
                second_fragment,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        )
        | (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::SelectedCircularEndpoint {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-chord-contact",
            );
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (
            CurveTangent2::ChordContact {
                chord: first,
                circle_cross_chord: RealSign::Zero,
                ..
            },
            CurveTangent2::ChordContact {
                chord: second,
                circle_cross_chord: RealSign::Zero,
                ..
            },
        ) => exact_algebraic_chord_parallel_factor(first, second, policy).map(|_| RealSign::Zero),
        (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "chord-contact-pair",
            );
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            )
            .map(|(cross, _)| cross)
        }
        (CurveTangent2::ChordContact { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::ChordContact { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::SelectedCircularEndpoint { .. }, CurveTangent2::CircularPoint { .. })
        | (CurveTangent2::CircularPoint { .. }, CurveTangent2::SelectedCircularEndpoint { .. })
        | (CurveTangent2::RetainedParallel { .. }, _)
        | (_, CurveTangent2::RetainedParallel { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

pub(super) fn curve_tangents_are_opposite(
    first: &CurveTangent2,
    second: &CurveTangent2,
    policy: &CurveContext,
) -> Classification<bool> {
    match curve_tangent_cross_sign(first, second, policy) {
        Classification::Decided(RealSign::Negative | RealSign::Positive) => {
            return Classification::Decided(false);
        }
        Classification::Decided(RealSign::Zero) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match (first, second) {
        (
            CurveTangent2::RepresentedDirection(first),
            CurveTangent2::RepresentedDirection(second),
        ) => {
            if offset_vectors_are_structurally_opposite(first, second) {
                return Classification::Decided(true);
            }
            let dot = &first.0 * &second.0 + &first.1 * &second.1;
            match real_sign(&dot, policy) {
                Some(RealSign::Negative) => Classification::Decided(true),
                Some(RealSign::Positive) => Classification::Decided(false),
                Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::RepresentedDirection(vector),
        )
        | (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        ) => match exact_retained_parallel_tangent_cross_and_dot_vector(
            parallel,
            parameter,
            *source_direction,
            vector,
            policy,
        ) {
            Classification::Decided((_, RealSign::Negative)) => Classification::Decided(true),
            Classification::Decided((_, RealSign::Positive)) => Classification::Decided(false),
            Classification::Decided((_, RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::RetainedParallel {
                parallel: first_parallel,
                parameter: first_parameter,
                source_direction: first_direction,
                ..
            },
            CurveTangent2::RetainedParallel {
                parallel: second_parallel,
                parameter: second_parameter,
                source_direction: second_direction,
                ..
            },
        ) => match exact_retained_parallel_tangent_pair_cross_and_dot(
            first_parallel,
            first_parameter,
            *first_direction,
            second_parallel,
            second_parameter,
            *second_direction,
            policy,
        ) {
            Classification::Decided((_, RealSign::Negative)) => Classification::Decided(true),
            Classification::Decided((_, RealSign::Positive)) => Classification::Decided(false),
            Classification::Decided((_, RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (CurveTangent2::AlgebraicChord(first), CurveTangent2::AlgebraicChord(second)) => {
            match first.tangent_dot_sign(second, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
        )
        | (
            CurveTangent2::RetainedParallel {
                parallel,
                parameter,
                source_direction,
                ..
            },
            CurveTangent2::AlgebraicChord(chord),
        ) => match exact_algebraic_chord_retained_parallel_relation(
            chord,
            parallel,
            parameter,
            *source_direction,
            false,
            policy,
        ) {
            Classification::Decided(RealSign::Negative) => Classification::Decided(true),
            Classification::Decided(RealSign::Positive) => Classification::Decided(false),
            Classification::Decided(RealSign::Zero) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (CurveTangent2::AlgebraicChord(chord), CurveTangent2::RepresentedDirection(vector))
        | (CurveTangent2::RepresentedDirection(vector), CurveTangent2::AlgebraicChord(chord)) => {
            match chord.tangent_dot_vector_sign(vector, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: first_source,
                fragment: first_fragment,
                at_start: first_start,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment: second_source,
                fragment: second_fragment,
                at_start: second_start,
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_selected_circle_pair_tangent_cross_and_dot(
                first_source,
                first_fragment,
                *first_start,
                second_source,
                second_fragment,
                *second_start,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        )
        | (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::SelectedCircularEndpoint {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
        )
        | (
            CurveTangent2::RetainedParallel {
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter,
                source_direction,
            },
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_selected_circle_retained_parallel_tangent_cross_and_dot(
                source_fragment,
                fragment,
                *at_start,
                parallel,
                source_parallel,
                source_range,
                parameter,
                selected_source_parameter.as_ref(),
                *source_direction,
                policy,
            ),
        ),
        (
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
            CurveTangent2::RepresentedDirection(vector),
        )
        | (
            CurveTangent2::RepresentedDirection(vector),
            CurveTangent2::SelectedCircularEndpoint {
                fragment, at_start, ..
            },
        ) => {
            let perpendicular = (-vector.1.clone(), vector.0.clone());
            match fragment.endpoint_tangent_cross_vector(*at_start, &perpendicular, policy) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
            CurveTangent2::AlgebraicChord(chord),
        )
        | (
            CurveTangent2::AlgebraicChord(chord),
            CurveTangent2::SelectedCircularEndpoint {
                source_fragment,
                fragment,
                at_start,
            },
        ) => {
            match fragment.endpoint_tangent_dot_algebraic_chord_from_source(
                source_fragment,
                *at_start,
                chord,
                policy,
            ) {
                Ok(Classification::Decided(RealSign::Negative)) => Classification::Decided(true),
                Ok(Classification::Decided(RealSign::Positive)) => Classification::Decided(false),
                Ok(Classification::Decided(RealSign::Zero)) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        (
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
            CurveTangent2::AlgebraicChord(candidate),
        )
        | (
            CurveTangent2::AlgebraicChord(candidate),
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
        ) => match exact_algebraic_chord_parallel_factor(chord, candidate, policy) {
            Classification::Decided(factor) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-tangent-dot",
                    "selected-chord-normal-algebraic-chord",
                );
                match exact_sign_product(*circle_dot_chord, factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
            CurveTangent2::RepresentedDirection(candidate),
        )
        | (
            CurveTangent2::RepresentedDirection(candidate),
            CurveTangent2::ChordContact {
                chord,
                circle_dot_chord: Some(circle_dot_chord),
                ..
            },
        ) => match exact_algebraic_chord_vector_factor(chord, candidate, policy) {
            Classification::Decided(factor) => {
                match exact_sign_product(*circle_dot_chord, factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                chord: first_chord,
                circle_dot_chord: Some(first_dot),
                ..
            },
            CurveTangent2::ChordContact {
                chord: second_chord,
                circle_dot_chord: Some(second_dot),
                ..
            },
        ) => match exact_algebraic_chord_parallel_factor(first_chord, second_chord, policy) {
            Classification::Decided(factor) => {
                match exact_sign_product(exact_sign_product(*first_dot, *second_dot), factor) {
                    RealSign::Negative => Classification::Decided(true),
                    RealSign::Positive => Classification::Decided(false),
                    RealSign::Zero => Classification::Uncertain(UncertaintyReason::Boundary),
                }
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
        (
            CurveTangent2::ChordContact {
                fragment: first,
                at_start: first_start,
                ..
            },
            CurveTangent2::ChordContact {
                fragment: second,
                at_start: second_start,
                ..
            },
        ) => exact_offset_tangent_relation_is_opposite(
            exact_current_selected_circle_pair_tangent_cross_and_dot(
                first,
                *first_start,
                second,
                *second_start,
                policy,
            ),
        ),
        (CurveTangent2::CircularPoint { .. }, _) | (_, CurveTangent2::CircularPoint { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        (CurveTangent2::ChordContact { .. }, CurveTangent2::RepresentedDirection(_))
        | (CurveTangent2::RepresentedDirection(_), CurveTangent2::ChordContact { .. })
        | (CurveTangent2::ChordContact { .. }, CurveTangent2::AlgebraicChord(_))
        | (CurveTangent2::AlgebraicChord(_), CurveTangent2::ChordContact { .. })
        | (CurveTangent2::RetainedParallel { .. }, _)
        | (_, CurveTangent2::RetainedParallel { .. }) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

pub(super) fn offset_vectors_are_structurally_opposite(
    first: &(Real, Real),
    second: &(Real, Real),
) -> bool {
    (&first.0 + &second.0).zero_status() == hyperreal::ZeroKnowledge::Zero
        && (&first.1 + &second.1).zero_status() == hyperreal::ZeroKnowledge::Zero
}

pub(super) struct RetainedDeferredArcContact2 {
    pub(super) source_parameter: CurveParameter2,
    pub(super) source_at_start: bool,
    pub(super) source_at_end: bool,
    pub(super) point: CurvePoint2,
    pub(super) fillet_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    pub(super) fillet_half: u8,
}

pub(super) struct RetainedDeferredArcFilletResult2 {
    pub(super) fillet_fragments: Vec<BezierSplitFragment2>,
    pub(super) arc_replacement: Option<Vec<BezierSplitFragment2>>,
}
