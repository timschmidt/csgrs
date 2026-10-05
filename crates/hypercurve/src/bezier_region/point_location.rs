//! Exact point location against retained and native boundary loops.
//!
//! Exact points use native line/arc loops or retained-fragment ray winding;
//! algebraic points use certified ray sign hulls over their predicate
//! evaluator. Both paths classify boundary contacts before parity.

use super::*;

pub(super) fn native_loop_sample_point(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Point2> {
    let Some(fragment) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let half = match Real::one() / Real::from(2_i8) {
        Ok(half) => half,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    subcurve_point_at(fragment, half, policy)
}

pub(super) fn retained_loop_sample_point_evidence(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if boundary_loop.fragments().is_empty() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let mut last_reason = UncertaintyReason::Unsupported;
    for fragment in boundary_loop.fragments() {
        let candidate = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                subcurve_point_at(curve, half.clone(), policy).map(CurvePoint2::from)
            }
            BezierSplitFragment2::RetainedBezier {
                start,
                end,
                source_curve,
                ..
            } => match start.strict_scalar_between(end, policy)? {
                Classification::Decided(parameter) => {
                    subcurve_point_at(source_curve, parameter, policy).map(CurvePoint2::from)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            BezierSplitFragment2::AnalyticParallel(fragment) => fragment
                .representative_point(policy)?
                .map(CurvePoint2::from),
            BezierSplitFragment2::SelectedFiber(fragment) => fragment
                .representative_point(policy)?
                .map(CurvePoint2::from),
            BezierSplitFragment2::AlgebraicChord(chord) => chord.representative_point(policy)?,
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                match fragment.representative_point()? {
                    Classification::Decided(point) => {
                        Classification::Decided(CurvePoint2::from(point))
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        };
        match candidate {
            Classification::Decided(point) => return Ok(Classification::Decided(point)),
            Classification::Uncertain(reason) => last_reason = reason,
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

pub(crate) fn classify_point_evidence_against_retained_loop(
    region: &CurveRegion2,
    loop_index: usize,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let boundary_loop = region.data.boundary_loops.get(loop_index).ok_or_else(|| {
        CurveError::Topology("retained loop classification index is out of bounds".into())
    })?;
    let direct = match point {
        CurvePoint2(CurvePointData2::Exact(point)) => Some(classify_point_against_retained_loop(
            boundary_loop,
            point,
            policy,
        )?),
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            Some(match point.predicate_evaluator(policy)? {
                Classification::Decided(predicate) => {
                    if let Classification::Decided(bounds) =
                        retained_loop_query_bounds(boundary_loop, policy)
                        && algebraic_point_is_decided_outside_bounds(&predicate, &bounds, policy)?
                    {
                        Classification::Decided(ContourPointLocation::Outside)
                    } else {
                        classify_algebraic_point_against_retained_loop(
                            boundary_loop,
                            &predicate,
                            FillRule::EvenOdd,
                            true,
                            policy,
                        )?
                    }
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
    };
    match direct {
        Some(decided @ Classification::Decided(_)) => Ok(decided),
        Some(Classification::Uncertain(_)) | None => {
            crate::curve_region_boolean::classify_retained_point_evidence_against_loop_by_probe(
                region,
                loop_index,
                point.clone(),
                policy,
            )
        }
    }
}

pub(super) fn subcurve_control_hull_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::RationalQuadratic(curve) => {
            if curve.common_nonzero_weight_sign(policy).is_none() {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            Aabb2::from_points(curve.control_points())
        }
        BezierSubcurve2::Rational(_) => {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
    };
    match bounds {
        Classification::Decided(bounds) => bounds.contains_point(point, policy),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn algebraic_point_is_decided_outside_bounds(
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    bounds: &Aabb2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    for (use_x, minimum, maximum) in [
        (true, bounds.min_x(), bounds.max_x()),
        (false, bounds.min_y(), bounds.max_y()),
    ] {
        if matches!(
            point.coordinate_order_to_real(use_x, minimum, policy)?,
            Classification::Decided(std::cmp::Ordering::Less)
        ) || matches!(
            point.coordinate_order_to_real(use_x, maximum, policy)?,
            Classification::Decided(std::cmp::Ordering::Greater)
        ) {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn classify_algebraic_point_against_line_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let mut winding = 0_i32;
    for fragment in boundary_loop.fragments() {
        let line = match retained_line_fragment_segment(fragment, policy)? {
            Classification::Decided(line) => line,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start_order = match point.coordinate_order_to_real(false, line.start().y(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = match point.coordinate_order_to_real(false, line.end().y(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if matches!(
            (start_order, end_order),
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
        ) {
            continue;
        }
        let side = match point.oriented_line_side(line.start(), line.end(), policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if side == LineSide::On {
            let start_x = match point.coordinate_order_to_real(true, line.start().x(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let end_x = match point.coordinate_order_to_real(true, line.end().x(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if !matches!(
                (start_x, end_x),
                (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                    | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
            ) {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            continue;
        }
        if start_order != std::cmp::Ordering::Less
            && end_order == std::cmp::Ordering::Less
            && side == LineSide::Left
        {
            winding += 1;
        } else if start_order == std::cmp::Ordering::Less
            && end_order != std::cmp::Ordering::Less
            && side == LineSide::Right
        {
            winding -= 1;
        }
    }
    Ok(Classification::Decided(winding_location(
        winding, fill_rule,
    )))
}

#[derive(Clone)]
pub(super) struct AlgebraicRayHomogeneousControl2 {
    pub(super) x: Real,
    pub(super) y: Real,
    pub(super) weight: Real,
}

pub(super) struct AlgebraicRayRationalFragment2 {
    pub(super) curve: RationalBezier2,
    pub(super) retained_range: Option<CurveParameterRange2>,
    pub(super) reversed: bool,
}

pub(super) enum AlgebraicRayRetainedFragment2 {
    Rational(AlgebraicRayRationalFragment2),
    AnalyticParallel(crate::bezier_offset::BezierParallelAlgebraicRay2),
    AlgebraicChord(crate::BezierAlgebraicChord2),
    AlgebraicCusp(crate::bezier_offset::BezierAlgebraicCuspSemicircleAlgebraicRay2),
}

#[derive(Default)]
pub(super) struct AlgebraicRaySignHull2 {
    pub(super) negative: bool,
    pub(super) zero: bool,
    pub(super) positive: bool,
    pub(super) first: Option<RealSign>,
    pub(super) last: Option<RealSign>,
}

impl AlgebraicRaySignHull2 {
    fn include(&mut self, sign: RealSign) {
        match sign {
            RealSign::Negative => self.negative = true,
            RealSign::Zero => self.zero = true,
            RealSign::Positive => self.positive = true,
        }
        self.first.get_or_insert(sign);
        self.last = Some(sign);
    }
}

pub(super) fn classify_algebraic_point_against_retained_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    certify_boundary: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    match classify_algebraic_point_against_line_loop(boundary_loop, point, fill_rule, policy)? {
        decided @ Classification::Decided(_) => return Ok(decided),
        Classification::Uncertain(UncertaintyReason::Unsupported) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    classify_algebraic_point_against_retained_loop_with_cusps(
        boundary_loop,
        point,
        fill_rule,
        certify_boundary,
        policy,
    )
}

pub(super) fn classify_algebraic_point_against_retained_loop_with_cusps(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    fill_rule: FillRule,
    certify_boundary: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let fragments = match prepare_algebraic_ray_retained_fragments(boundary_loop, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };

    if certify_boundary {
        for fragment in &fragments {
            let contains = match fragment {
                AlgebraicRayRetainedFragment2::Rational(fragment) => {
                    algebraic_point_on_rational_fragment(fragment, point, policy)?
                }
                AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                    fragment.contains_point(point, None, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                    fragment.contains_algebraic_point(point, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                    fragment.contains_point(point, policy)?
                }
            };
            match contains {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(ContourPointLocation::Boundary));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    // There are at most two endpoint-collinear slopes per retained fragment.
    // Testing 2n+1 exact integer slopes therefore finds a nondegenerate ray
    // whenever the promised off-boundary query is distinct from every vertex.
    let candidate_count = fragments.len().saturating_mul(2).saturating_add(1);
    let mut last_reason = UncertaintyReason::Predicate;
    for slope in 0..candidate_count {
        let Ok(slope) = u64::try_from(slope) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let direction_x = Real::one();
        let direction_y = Real::from(slope);
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let mut admissible = match algebraic_ray_retained_fragments_admit_direction(
            &fragments, point, &side_x, &side_y, policy,
        )? {
            Classification::Decided(admissible) => admissible,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                false
            }
        };
        if !admissible {
            continue;
        }

        let winding = match algebraic_ray_retained_fragments_winding(
            &fragments,
            point,
            &direction_x,
            &direction_y,
            None,
            false,
            policy,
        )? {
            Classification::Decided(winding) => winding,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                admissible = false;
                0
            }
        };
        if !admissible {
            continue;
        }
        return Ok(Classification::Decided(winding_location(
            winding, fill_rule,
        )));
    }
    Ok(Classification::Uncertain(last_reason))
}

pub(super) fn prepare_algebraic_ray_retained_fragments(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<AlgebraicRayRetainedFragment2>>> {
    let mut fragments = Vec::with_capacity(boundary_loop.fragments().len());
    for fragment in boundary_loop.fragments() {
        match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                if chord.exact_line().is_some() {
                    let fragment = match retained_fragment_algebraic_ray_curve(fragment, policy)? {
                        Classification::Decided(fragment) => fragment,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    fragments.push(AlgebraicRayRetainedFragment2::Rational(fragment));
                    continue;
                }
                chord.validate_policy(policy)?;
                fragments.push(AlgebraicRayRetainedFragment2::AlgebraicChord(chord.clone()));
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                let evaluator = match fragment.algebraic_ray_evaluator(policy)? {
                    Classification::Decided(evaluator) => evaluator,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                fragments.push(AlgebraicRayRetainedFragment2::AlgebraicCusp(evaluator));
            }
            _ => match retained_fragment_algebraic_ray_curve(fragment, policy)? {
                Classification::Decided(fragment) => {
                    fragments.push(AlgebraicRayRetainedFragment2::Rational(fragment));
                }
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    let fragment =
                        match retained_fragment_analytic_algebraic_ray_curve(fragment, policy)? {
                            Classification::Decided(fragment) => fragment,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    fragments.push(AlgebraicRayRetainedFragment2::AnalyticParallel(fragment));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        }
    }
    Ok(Classification::Decided(fragments))
}

pub(super) fn algebraic_ray_retained_fragments_admit_direction(
    fragments: &[AlgebraicRayRetainedFragment2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    side_x: &Real,
    side_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    for fragment in fragments {
        match fragment {
            // Rational winding owns finite endpoints spatially and can use
            // a vertex ray without reconstructing its endpoint images.
            AlgebraicRayRetainedFragment2::Rational(_) => {}
            AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                match fragment.endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                match fragment.algebraic_ray_endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                match fragment.endpoint_side_signs(point, side_x, side_y, policy)? {
                    Classification::Decided(signs)
                        if signs.into_iter().all(|sign| sign != RealSign::Zero) => {}
                    Classification::Decided(_) => return Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }
    Ok(Classification::Decided(true))
}

pub(super) fn algebraic_ray_retained_fragments_winding(
    fragments: &[AlgebraicRayRetainedFragment2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    skipped_fragment: Option<usize>,
    skip_incident_origin_contacts: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    if let Some(fragment_index) = skipped_fragment {
        match fragments.get(fragment_index) {
            Some(
                AlgebraicRayRetainedFragment2::AlgebraicCusp(_)
                | AlgebraicRayRetainedFragment2::AlgebraicChord(_),
            ) => {}
            Some(
                AlgebraicRayRetainedFragment2::Rational(_)
                | AlgebraicRayRetainedFragment2::AnalyticParallel(_),
            ) => {
                return Err(CurveError::Topology(
                    "an algebraic side ray can skip only a retained algebraic source".into(),
                ));
            }
            None => {
                return Err(CurveError::Topology(
                    "the algebraic side-ray source fragment is missing".into(),
                ));
            }
        }
    }
    let mut winding = 0_i32;
    for fragment in fragments {
        let delta = match fragment {
            AlgebraicRayRetainedFragment2::Rational(fragment) => {
                if skip_incident_origin_contacts {
                    match algebraic_point_rational_curve_ray_winding_skipping_incident_origin(
                        fragment,
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => {
                            algebraic_point_rational_curve_ray_winding(
                                fragment,
                                point,
                                direction_x,
                                direction_y,
                                policy,
                            )?
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    algebraic_point_rational_curve_ray_winding(
                        fragment,
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?
                }
            }
            AlgebraicRayRetainedFragment2::AnalyticParallel(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicChord(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.algebraic_forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment
                            .algebraic_forward_ray_winding_delta(
                                point,
                                direction_x,
                                direction_y,
                                policy,
                            )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.algebraic_forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?
                }
            }
            AlgebraicRayRetainedFragment2::AlgebraicCusp(fragment) => {
                if skip_incident_origin_contacts {
                    match fragment.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => Classification::Decided(delta),
                        Classification::Decided(None) => fragment.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            false,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                } else {
                    fragment.forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        false,
                        policy,
                    )?
                }
            }
        };
        let delta = match delta {
            Classification::Decided(delta) => delta,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        winding = winding.checked_add(delta).ok_or_else(|| {
            CurveError::Topology("algebraic ray winding exceeds the region counter".into())
        })?;
    }
    Ok(Classification::Decided(winding))
}

pub(super) fn retained_fragment_algebraic_ray_endpoints(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<[CurvePoint2; 2]> {
    let endpoint = |start_endpoint| -> CurveResult<_> {
        match curve_fragment_endpoint_point(fragment, start_endpoint, policy)? {
            Classification::Decided(Some(point)) => Ok(point),
            Classification::Decided(None) | Classification::Uncertain(_) => {
                Err(CurveError::Topology(
                    "a retained algebraic-ray fragment lost exact endpoint evidence".into(),
                ))
            }
        }
    };
    Ok([endpoint(true)?, endpoint(false)?])
}

pub(super) fn retained_fragment_analytic_algebraic_ray_curve(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::bezier_offset::BezierParallelAlgebraicRay2>> {
    let (parallel, range, reversed) = match fragment {
        BezierSplitFragment2::AnalyticParallel(fragment) => (
            fragment.parallel().clone(),
            CurveParameterRange2::from_bezier_range(fragment.range().clone()),
            fragment.is_reversed(),
        ),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(parallel) = fragment.analytic_parallel() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            (
                parallel.clone(),
                fragment.range().clone(),
                fragment.is_reversed(),
            )
        }
        _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
    };
    crate::bezier_offset::BezierParallelAlgebraicRay2::try_new(
        parallel,
        range,
        reversed,
        retained_fragment_algebraic_ray_endpoints(fragment, policy)?,
        policy,
    )
}

pub(super) fn retained_fragment_algebraic_ray_curve(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<AlgebraicRayRationalFragment2>> {
    match retained_line_fragment_segment(fragment, policy)? {
        Classification::Decided(line) => {
            return Ok(Classification::Decided(AlgebraicRayRationalFragment2 {
                curve: RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                ))?,
                retained_range: None,
                reversed: false,
            }));
        }
        Classification::Uncertain(UncertaintyReason::Unsupported) => {}
        // A straight parallel cut at algebraic parameters has no explicit
        // endpoint pair; its retained rational component below keeps the
        // exact range instead.
        Classification::Uncertain(UncertaintyReason::Boundary)
            if matches!(fragment, BezierSplitFragment2::AnalyticParallel(_)) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let (curve, retained_range, reversed) = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            (RationalBezier2::try_from_subcurve(curve)?, None, false)
        }
        BezierSplitFragment2::RetainedBezier {
            start,
            end,
            reversed,
            source_curve: curve,
            ..
        } => (
            RationalBezier2::try_from_subcurve(curve)?,
            Some(CurveParameterRange2::new_validated(
                CurveParameter2::from(start.clone()),
                CurveParameter2::from(end.clone()),
            )),
            *reversed,
        ),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let curve = match fragment
                .parallel()
                .exact_rational_parallel_component(policy)?
            {
                Classification::Decided(Some(curve)) => curve,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                curve,
                Some(CurveParameterRange2::from_bezier_range(
                    fragment.range().clone(),
                )),
                fragment.is_reversed(),
            )
        }
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(curve) = fragment.rational_curve() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            (
                curve.clone(),
                Some(fragment.range().clone()),
                fragment.is_reversed(),
            )
        }
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    // Native subdivision is an accelerator only inside its authored unit
    // domain. Exterior intervals retain the source and their exact bounds.
    let (curve, retained_range) = if let Some(range) = retained_range {
        if let Some((start, end)) = range.scalar_endpoints()
            && policy.strict_predicate_pass(|| {
                crate::classify::in_closed_unit_interval(start, policy) == Some(true)
                    && crate::classify::in_closed_unit_interval(end, policy) == Some(true)
            })
        {
            let curve = match curve.subcurve_between_exact_with_policy(start, end, policy)? {
                Classification::Decided(curve) => curve,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (curve, None)
        } else {
            (curve, Some(range))
        }
    } else {
        (curve, None)
    };
    Ok(Classification::Decided(AlgebraicRayRationalFragment2 {
        curve,
        retained_range,
        reversed,
    }))
}

pub(super) fn algebraic_point_rational_curve_linear_equation(
    curve: &RationalBezier2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
) -> CurveResult<BivariatePolynomial> {
    let power = curve.homogeneous_power_basis()?;
    let (query_x, query_y, query_weight) = point.coordinate_polynomials();
    let first_count = query_x.len().max(query_y.len()).max(query_weight.len());
    let second_count = power
        .x_numerator
        .len()
        .max(power.y_numerator.len())
        .max(power.weight.len());
    let query_linear = (0..first_count)
        .map(|index| {
            x_factor * query_x.get(index).cloned().unwrap_or_else(Real::zero)
                + y_factor * query_y.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect::<Vec<_>>();
    let curve_linear = (0..second_count)
        .map(|index| {
            x_factor
                * power
                    .x_numerator
                    .get(index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
                + y_factor
                    * power
                        .y_numerator
                        .get(index)
                        .cloned()
                        .unwrap_or_else(Real::zero)
        })
        .collect::<Vec<_>>();
    Ok(BivariatePolynomial::new(
        (0..first_count)
            .map(|first_power| {
                (0..second_count)
                    .map(|second_power| {
                        query_weight
                            .get(first_power)
                            .cloned()
                            .unwrap_or_else(Real::zero)
                            * &curve_linear[second_power]
                            - &query_linear[first_power]
                                * power
                                    .weight
                                    .get(second_power)
                                    .cloned()
                                    .unwrap_or_else(Real::zero)
                    })
                    .collect()
            })
            .collect(),
    ))
}

pub(super) fn algebraic_point_on_rational_curve(
    curve: &RationalBezier2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let weight_sign = match curve.denominator_sign(&CurveParameterRange2::unit()) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(_) => RealSign::Zero,
    };
    if weight_sign != RealSign::Zero {
        let controls = curve
            .homogeneous_controls()
            .iter()
            .map(|control| AlgebraicRayHomogeneousControl2 {
                x: control.x().clone(),
                y: control.y().clone(),
                weight: control.weight().clone(),
            })
            .collect::<Vec<_>>();
        for (factor_x, factor_y) in [(Real::one(), Real::zero()), (Real::zero(), Real::one())] {
            if let Classification::Decided(hull) = algebraic_ray_control_sign_hull(
                &controls,
                point,
                &factor_x,
                &factor_y,
                weight_sign,
                policy,
            )? && !hull.zero
                && (hull.negative ^ hull.positive)
            {
                return Ok(Classification::Decided(false));
            }
        }
    }
    let x =
        algebraic_point_rational_curve_linear_equation(curve, point, &Real::one(), &Real::zero())?;
    let y =
        algebraic_point_rational_curve_linear_equation(curve, point, &Real::zero(), &Real::one())?;
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &x,
        &y,
        CurveResultantParameter::First,
        point.retained_root(),
        &Real::zero(),
        &Real::one(),
        policy.predicate_policy(),
    );
    Ok(match report.status {
        AlgebraicFiberRootCountStatus::Counted => {
            Classification::Decided(report.distinct_root_count.unwrap_or(0) != 0)
        }
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
        | AlgebraicFiberRootCountStatus::EndpointRoot => Classification::Decided(true),
        AlgebraicFiberRootCountStatus::UnsupportedCoefficient => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        AlgebraicFiberRootCountStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
        AlgebraicFiberRootCountStatus::InvalidEvidence => {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        AlgebraicFiberRootCountStatus::InvalidInterval => {
            return Err(CurveError::Topology(
                "algebraic boundary incidence received an invalid unit interval".into(),
            ));
        }
    })
}

pub(super) fn algebraic_point_on_rational_fragment(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let Some(range) = fragment.retained_range.as_ref() else {
        return algebraic_point_on_rational_curve(&fragment.curve, point, policy);
    };
    let x = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        &Real::one(),
        &Real::zero(),
    )?;
    let y = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        &Real::zero(),
        &Real::one(),
    )?;
    let mut identically_zero_count = 0_usize;
    let mut last_reason = UncertaintyReason::Predicate;
    for (incidence, predicate) in [(&x, &y), (&y, &x)] {
        let parameters = match selected_fiber_parameters(
            incidence,
            point.retained_parameter(),
            range,
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                identically_zero_count += 1;
                continue;
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => continue,
            Classification::Uncertain(reason) => {
                last_reason = reason;
                continue;
            }
        };
        for parameter in parameters {
            match retained_curve_region_parameter_contains(&parameter, range, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match algebraic_selected_correlated_predicate_sign(
                incidence,
                predicate,
                point.retained_parameter(),
                &parameter,
                policy,
            )? {
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(true));
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        return Ok(Classification::Decided(false));
    }
    if identically_zero_count == 2 {
        Ok(Classification::Decided(true))
    } else {
        Ok(Classification::Uncertain(last_reason))
    }
}

pub(super) fn algebraic_point_rational_curve_ray_winding(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    if fragment.retained_range.is_some() {
        return algebraic_point_retained_rational_curve_ray_winding(
            fragment,
            point,
            direction_x,
            direction_y,
            false,
            policy,
        );
    }
    let weight_sign = match fragment
        .curve
        .denominator_sign(&CurveParameterRange2::unit())
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let controls = fragment
        .curve
        .homogeneous_controls()
        .iter()
        .map(|control| AlgebraicRayHomogeneousControl2 {
            x: control.x().clone(),
            y: control.y().clone(),
            weight: control.weight().clone(),
        })
        .collect::<Vec<_>>();
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let half = (Real::one() / Real::from(2_i8))?;
    let mut stack = vec![controls];
    let mut winding = 0_i32;
    while let Some(controls) = stack.pop() {
        let side = algebraic_ray_control_sign_hull(
            &controls,
            point,
            &side_x,
            &side_y,
            weight_sign,
            policy,
        )?;
        let side = match side {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !side.zero && (!side.negative || !side.positive) {
            continue;
        }
        let ahead = match algebraic_ray_control_sign_hull(
            &controls,
            point,
            direction_x,
            direction_y,
            weight_sign,
            policy,
        )? {
            Classification::Decided(ahead) => ahead,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if (side.first == Some(RealSign::Zero) && ahead.first == Some(RealSign::Zero))
            || (side.last == Some(RealSign::Zero) && ahead.last == Some(RealSign::Zero))
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if !ahead.positive {
            continue;
        }
        if !side.negative && !side.positive {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if !ahead.negative {
            // Every interior point lies strictly ahead. Spatial ownership
            // makes the sum of all contacts telescope to the endpoint signs,
            // including zero endpoints. The same rule cancels artificial
            // subdivision endpoints without a separate midpoint contact.
            let delta = spatial_ray_winding_delta(
                side.first == Some(RealSign::Positive),
                side.last == Some(RealSign::Positive),
                false,
                false,
            );
            winding = winding.checked_add(delta).ok_or_else(|| {
                CurveError::Topology("algebraic ray winding exceeds the curve counter".into())
            })?;
            continue;
        }
        let (left, right) = split_algebraic_ray_controls_at_half(&controls, &half);
        stack.push(right);
        stack.push(left);
    }
    Ok(Classification::Decided(if fragment.reversed {
        winding.checked_neg().ok_or_else(|| {
            CurveError::Topology("algebraic ray winding reversal overflowed".into())
        })?
    } else {
        winding
    }))
}

pub(super) fn algebraic_point_rational_curve_ray_winding_skipping_incident_origin(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<i32>>> {
    match algebraic_point_on_rational_fragment(fragment, point, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    if fragment.retained_range.is_some() {
        return algebraic_point_retained_rational_curve_ray_winding(
            fragment,
            point,
            direction_x,
            direction_y,
            true,
            policy,
        )
        .map(|delta| delta.map(Some));
    }
    let retained = AlgebraicRayRationalFragment2 {
        curve: fragment.curve.clone(),
        retained_range: Some(CurveParameterRange2::from_bezier_range(
            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
        )),
        reversed: fragment.reversed,
    };
    algebraic_point_retained_rational_curve_ray_winding(
        &retained,
        point,
        direction_x,
        direction_y,
        true,
        policy,
    )
    .map(|delta| delta.map(Some))
}

pub(super) fn algebraic_point_retained_rational_curve_ray_winding(
    fragment: &AlgebraicRayRationalFragment2,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    direction_x: &Real,
    direction_y: &Real,
    skip_incident_origin: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let range = fragment
        .retained_range
        .as_ref()
        .expect("retained algebraic ray winding requires a retained range");
    let weight_sign = match fragment.curve.denominator_sign(range) {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let incidence =
        algebraic_point_rational_curve_linear_equation(&fragment.curve, point, &side_x, &side_y)?;

    if let BezierParameter2::Algebraic(retained) = point.retained_parameter()
        && bivariate_fiber_strict_sign_on_parameter_range(&incidence, retained, range, policy)?
            .is_some()
    {
        return Ok(Classification::Decided(0));
    }

    let parameters =
        match selected_fiber_parameters(&incidence, point.retained_parameter(), range, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let ahead = algebraic_point_rational_curve_linear_equation(
        &fragment.curve,
        point,
        direction_x,
        direction_y,
    )?;
    let denominator_sign = product_sign(point.denominator_sign(), weight_sign);
    let mut winding = 0_i32;
    for parameter in parameters {
        let [start, end] = match retained_curve_region_parameter_orders(&parameter, range, policy)?
        {
            Classification::Decided(orders) => orders,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        use std::cmp::Ordering::{Equal, Greater, Less};
        if start == Less || end == Greater || (start == Equal && end == Equal) {
            continue;
        }

        let ahead_sign = match algebraic_selected_correlated_predicate_sign(
            &incidence,
            &ahead,
            point.retained_parameter(),
            &parameter,
            policy,
        )? {
            Classification::Decided(sign) => product_sign(sign, denominator_sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let at_origin = match ahead_sign {
            RealSign::Negative => continue,
            RealSign::Zero => {
                if !skip_incident_origin {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                true
            }
            RealSign::Positive => false,
        };

        let mut derivative = incidence.clone();
        let mut derivative_order = 0_usize;
        let (before_positive, after_positive) = loop {
            derivative_order += 1;
            derivative = algebraic_ray_bivariate_second_derivative(&derivative);
            let derivative_sign = match algebraic_selected_correlated_predicate_sign(
                &incidence,
                &derivative,
                point.retained_parameter(),
                &parameter,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if derivative_sign != RealSign::Zero {
                let after_positive =
                    product_sign(derivative_sign, denominator_sign) == RealSign::Positive;
                let before_positive = if derivative_order.is_multiple_of(2) {
                    after_positive
                } else {
                    !after_positive
                };
                break (before_positive, after_positive);
            }
            if derivative.coefficients.iter().all(|row| row.len() <= 1) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        };
        if at_origin {
            if before_positive == after_positive {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            continue;
        }
        let delta = spatial_ray_winding_delta(
            before_positive,
            after_positive,
            start == Equal,
            end == Equal,
        );
        winding = winding.checked_add(delta).ok_or_else(|| {
            CurveError::Topology("algebraic ray winding exceeds the curve counter".into())
        })?;
    }
    Ok(Classification::Decided(if fragment.reversed {
        winding.checked_neg().ok_or_else(|| {
            CurveError::Topology("algebraic ray winding reversal overflowed".into())
        })?
    } else {
        winding
    }))
}

pub(super) fn algebraic_ray_bivariate_second_derivative(
    polynomial: &BivariatePolynomial,
) -> BivariatePolynomial {
    BivariatePolynomial::new(
        polynomial
            .coefficients
            .iter()
            .map(|row| {
                if row.len() <= 1 {
                    return vec![Real::zero()];
                }
                row.iter()
                    .enumerate()
                    .skip(1)
                    .map(|(power, coefficient)| coefficient * Real::from(power as u64))
                    .collect()
            })
            .collect(),
    )
}

pub(super) fn algebraic_ray_control_sign_hull(
    controls: &[AlgebraicRayHomogeneousControl2],
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
    weight_sign: RealSign,
    policy: &CurveContext,
) -> CurveResult<Classification<AlgebraicRaySignHull2>> {
    let mut hull = AlgebraicRaySignHull2::default();
    for control in controls {
        match point.homogeneous_linear_difference_sign(
            &control.x,
            &control.y,
            &control.weight,
            x_factor,
            y_factor,
            weight_sign,
            policy,
        )? {
            Classification::Decided(sign) => hull.include(sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(hull))
}

pub(super) fn split_algebraic_ray_controls_at_half(
    controls: &[AlgebraicRayHomogeneousControl2],
    half: &Real,
) -> (
    Vec<AlgebraicRayHomogeneousControl2>,
    Vec<AlgebraicRayHomogeneousControl2>,
) {
    let mut level = controls.to_vec();
    let mut left = Vec::with_capacity(level.len());
    let mut right = Vec::with_capacity(level.len());
    left.push(level[0].clone());
    right.push(level[level.len() - 1].clone());
    for next_len in (1..level.len()).rev() {
        for index in 0..next_len {
            level[index] = AlgebraicRayHomogeneousControl2 {
                x: Real::dot2_refs([&level[index].x, &level[index + 1].x], [half, half]),
                y: Real::dot2_refs([&level[index].y, &level[index + 1].y], [half, half]),
                weight: Real::dot2_refs(
                    [&level[index].weight, &level[index + 1].weight],
                    [half, half],
                ),
            };
        }
        left.push(level[0].clone());
        right.push(level[next_len - 1].clone());
    }
    right.reverse();
    (left, right)
}

pub(super) fn classify_point_against_native_loop_after_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    classify_point_against_native_loop_after_bounds_with_fill_rule(
        boundary_loop,
        point,
        FillRule::EvenOdd,
        policy,
    )
}

pub(super) fn classify_point_against_native_loop_after_bounds_with_fill_rule(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    for fragment in boundary_loop.fragments() {
        if matches!(
            subcurve_control_hull_contains_point(fragment, point, policy),
            Classification::Decided(false)
        ) {
            continue;
        }
        match subcurve_contains_point(fragment, point, policy) {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) | Classification::Uncertain(_) => {}
        }
    }
    let rays = ray_candidates(point);
    let mut last_reason = UncertaintyReason::Boundary;
    for ray in rays {
        match classify_point_with_ray(boundary_loop, point, &ray, fill_rule, policy)? {
            Classification::Decided(location) => {
                return Ok(Classification::Decided(location));
            }
            Classification::Uncertain(reason) => last_reason = reason,
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

pub(super) fn classify_point_against_retained_loops(
    boundary_loops: &[CurveRegionBoundaryLoop2],
    point: &Point2,
    policy: &CurveContext,
    roles: Option<&[CurveRegionLoopRole]>,
    fill_rules: Option<&[FillRule]>,
) -> CurveResult<Classification<RegionPointLocation>> {
    if roles.is_some_and(|roles| roles.len() != boundary_loops.len())
        || fill_rules.is_some_and(|rules| rules.len() != boundary_loops.len())
    {
        return Err(CurveError::Topology(
            "retained region loop semantics are inconsistent with boundary loops".into(),
        ));
    }
    let mut inside = false;
    let mut signed_depth = 0_i32;
    for (index, boundary_loop) in boundary_loops.iter().enumerate() {
        let fill_rule = fill_rules.map_or(FillRule::EvenOdd, |rules| rules[index]);
        match classify_point_against_retained_loop_with_fill_rule(
            boundary_loop,
            point,
            fill_rule,
            policy,
        )? {
            Classification::Decided(ContourPointLocation::Inside) => {
                if let Some(roles) = roles {
                    signed_depth += match roles[index] {
                        CurveRegionLoopRole::Material => 1,
                        CurveRegionLoopRole::Hole => -1,
                    };
                } else {
                    inside = !inside;
                }
            }
            Classification::Decided(ContourPointLocation::Outside) => {}
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Ok(Classification::Decided(RegionPointLocation::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let inside = roles.map_or(inside, |_| signed_depth > 0);
    Ok(Classification::Decided(if inside {
        RegionPointLocation::Inside
    } else {
        RegionPointLocation::Outside
    }))
}

pub(super) fn classify_point_against_retained_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    classify_point_against_retained_loop_with_fill_rule(
        boundary_loop,
        point,
        FillRule::EvenOdd,
        policy,
    )
}

pub(super) fn classify_point_against_retained_loop_with_fill_rule(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    if policy.strict_predicate_pass(|| {
        matches!(
            retained_loop_query_bounds(boundary_loop, policy),
            Classification::Decided(bounds)
                if bounds.contains_point(point, &CurveContext::STRICT)
                    == Classification::Decided(false)
        )
    }) {
        return Ok(Classification::Decided(ContourPointLocation::Outside));
    }
    for (fragment, evaluator) in boundary_loop
        .fragments()
        .iter()
        .zip(boundary_loop.rational_evaluators()?)
    {
        if let BezierSplitFragment2::Materialized { curve, .. } = fragment
            && matches!(
                subcurve_control_hull_contains_point(curve, point, policy),
                Classification::Decided(false)
            )
        {
            continue;
        }
        match policy.strict_predicate_pass(|| {
            retained_fragment_contains_point(fragment, evaluator.as_ref(), point, policy)
        })? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(ContourPointLocation::Boundary));
            }
            Classification::Decided(false) | Classification::Uncertain(_) => {}
        }
    }
    let mut last_reason = UncertaintyReason::Boundary;
    let rays = ray_candidates(point);
    // A ray that meets an unresolved endpoint is only one possible witness.
    // Exhaust every exact ray before permitting any terminal interpretation.
    for approximate in [false, true] {
        if approximate && !policy.permits_approximate_512() {
            break;
        }
        for ray in &rays {
            let classify =
                || classify_point_with_retained_ray(boundary_loop, point, ray, fill_rule, policy);
            let result = if approximate {
                classify()
            } else {
                policy.strict_predicate_pass(classify)
            }?;
            match result {
                Classification::Decided(location) => {
                    return Ok(Classification::Decided(location));
                }
                Classification::Uncertain(reason) => last_reason = reason,
            }
        }
    }
    Ok(Classification::Uncertain(last_reason))
}

pub(super) fn retained_fragment_contains_point(
    fragment: &BezierSplitFragment2,
    evaluator: Option<&RationalBezier2>,
    point: &Point2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            Ok(subcurve_contains_point(curve, point, policy))
        }
        BezierSplitFragment2::RetainedBezier { start, end, .. } => {
            let Some(evaluator) = evaluator else {
                return Err(CurveError::Topology(
                    "retained algebraic source evaluator cache is incomplete".into(),
                ));
            };
            let range =
                CurveParameterRange2::new_validated(start.clone().into(), end.clone().into());
            Ok(evaluator
                .point_incidence_on_range(point, &range, policy)?
                .map(|incidence| match incidence {
                    RationalBezierPointIncidence2::EntireCurve => true,
                    RationalBezierPointIncidence2::Parameters(parameters) => !parameters.is_empty(),
                }))
        }
        BezierSplitFragment2::AnalyticParallel(fragment) => fragment.parallel().contains_point(
            point,
            &CurveParameterRange2::from_bezier_range(fragment.range().clone()),
            policy,
        ),
        BezierSplitFragment2::SelectedFiber(fragment) => {
            if let Some(curve) = fragment.rational_curve() {
                Ok(curve
                    .point_incidence_on_range(point, fragment.range(), policy)?
                    .map(|incidence| match incidence {
                        RationalBezierPointIncidence2::EntireCurve => true,
                        RationalBezierPointIncidence2::Parameters(parameters) => {
                            !parameters.is_empty()
                        }
                    }))
            } else {
                fragment
                    .analytic_parallel()
                    .expect("a selected-fiber source is rational or analytic")
                    .contains_point(point, fragment.range(), policy)
            }
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            chord.contains_point(&CurvePoint2::from(point.clone()), policy)
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            fragment.contains_point(&CurvePoint2::from(point.clone()), policy)
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct RetainedRayOriginContact<'a> {
    pub(super) fragment_index: Option<usize>,
    pub(super) parameter: Option<&'a CurveParameter2>,
    pub(super) crossing_direction: BezierLineCrossingDirection,
    pub(super) tangent_contacts:
        Option<&'a [crate::rational_bezier::RationalQuadraticCircleTangentContact2]>,
}

impl RetainedRayOriginContact<'_> {
    // The caller certifies the crossing in boundary traversal order. The
    // supporting-line solvers consume increasing source-parameter order.
    fn parameter_crossing_direction(&self, reversed: bool) -> BezierLineCrossingDirection {
        match (self.crossing_direction, reversed) {
            (BezierLineCrossingDirection::NegativeToPositive, true) => {
                BezierLineCrossingDirection::PositiveToNegative
            }
            (BezierLineCrossingDirection::PositiveToNegative, true) => {
                BezierLineCrossingDirection::NegativeToPositive
            }
            (direction, false) => direction,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum RetainedRayWinding {
    Winding(i32),
    Boundary,
}

pub(super) fn retained_circle_tangent_contacts(
    fragment: &BezierSplitFragment2,
) -> Option<&[crate::rational_bezier::RationalQuadraticCircleTangentContact2]> {
    let circle = match fragment {
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::RationalQuadratic(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::Rational(curve),
            ..
        }
        | BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::Rational(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::RationalQuadratic(curve),
            ..
        } => curve.retained_circular_conic(),
        BezierSplitFragment2::SelectedFiber(fragment) => fragment
            .rational_curve()
            .and_then(RationalBezier2::retained_circular_conic),
        BezierSplitFragment2::Materialized { .. }
        | BezierSplitFragment2::RetainedBezier { .. }
        | BezierSplitFragment2::AnalyticParallel(_)
        | BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
    }?;
    circle.tangent_contacts.as_deref()
}

pub(super) fn classify_point_with_retained_ray(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    Ok(
        match classify_point_with_retained_ray_skipping_origin(
            boundary_loop,
            point,
            ray,
            None,
            policy,
        )? {
            Classification::Decided(RetainedRayWinding::Winding(winding)) => {
                Classification::Decided(winding_location(winding, fill_rule))
            }
            Classification::Decided(RetainedRayWinding::Boundary) => {
                Classification::Decided(ContourPointLocation::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn classify_point_with_retained_ray_skipping_origin(
    boundary_loop: &CurveRegionBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    skipped_origin: Option<RetainedRayOriginContact<'_>>,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedRayWinding>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    let mut source_origin_contact_was_skipped = false;
    let cached = boundary_loop.fragment_query_bounds(policy);
    let cached_bounds = cached.map(|(bounds, _)| bounds);
    // A closed loop has zero winding at any point strictly outside its
    // certified bounding box, so its fragments need no ray query. The source
    // loop is excluded because the origin lies on it.
    if skipped_origin.is_none_or(|origin| origin.fragment_index.is_none())
        && let Some((_, Some(loop_box))) = cached
        && let (Some([x_low, x_high]), Some([y_low, y_high])) = (
            certified_f64_enclosure(point.x()),
            certified_f64_enclosure(point.y()),
        )
        && (x_high < loop_box[0]
            || x_low > loop_box[1]
            || y_high < loop_box[2]
            || y_low > loop_box[3])
    {
        return Ok(Classification::Decided(RetainedRayWinding::Winding(0)));
    }
    let approximate_ray = match (
        certified_f64_enclosure(point.x()),
        certified_f64_enclosure(point.y()),
        certified_f64_enclosure(direction_x),
        certified_f64_enclosure(direction_y),
    ) {
        (Some(x), Some(y), Some(dx), Some(dy)) => Some(([x, y], [dx, dy])),
        _ => None,
    };
    for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
        let is_source_fragment =
            skipped_origin.is_some_and(|origin| origin.fragment_index == Some(fragment_index));
        if !is_source_fragment
            && let (Some(bounds), Some((origin, direction))) = (cached_bounds, approximate_ray)
            && let Some(approximate) = bounds[fragment_index].approximate
            && f64_box_certainly_misses_forward_ray(approximate, origin, direction)
        {
            continue;
        }
        let computed;
        let bounds = match cached_bounds {
            Some(bounds) => &bounds[fragment_index].exact,
            None => {
                computed = retained_fragment_query_bounds(fragment, policy);
                &computed
            }
        };
        if let Classification::Decided(bounds) = bounds
            && !retained_bounds_may_intersect_forward_ray(bounds, point, direction_x, direction_y)
            && !is_source_fragment
        {
            continue;
        }
        let exact_parallel_curve = match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) => match fragment
                .parallel()
                .exact_rational_parallel_component(policy)
            {
                Ok(Classification::Decided(Some(curve))) => Some(BezierSubcurve2::Rational(curve)),
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
            },
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
            BezierSplitFragment2::SelectedFiber(_) => None,
        };
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            let source_contact = skipped_origin.and_then(|origin| {
                (origin.fragment_index == Some(fragment_index))
                    .then_some(origin)
                    .and_then(|origin| {
                        Some((
                            origin.parameter?.as_algebraic_chord()?,
                            origin.crossing_direction,
                        ))
                    })
            });
            let result = match source_contact {
                Some((parameter, crossing_direction)) => {
                    let result = chord.forward_ray_winding_delta_skipping_origin(
                        point,
                        direction_x,
                        direction_y,
                        parameter,
                        crossing_direction,
                        policy,
                    )?;
                    if matches!(result, Classification::Decided(_)) {
                        source_origin_contact_was_skipped = true;
                    }
                    result
                }
                None if skipped_origin.is_some() => {
                    match chord.forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                        Classification::Decided(Some(delta)) => {
                            if skipped_origin
                                .is_some_and(|origin| origin.fragment_index == Some(fragment_index))
                            {
                                source_origin_contact_was_skipped = true;
                            }
                            Classification::Decided(delta)
                        }
                        Classification::Decided(None) => chord.forward_ray_winding_delta(
                            point,
                            direction_x,
                            direction_y,
                            policy,
                        )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                }
                None => chord.forward_ray_winding_delta(point, direction_x, direction_y, policy)?,
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            continue;
        }
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            let source_contact = skipped_origin.and_then(|origin| {
                (origin.fragment_index == Some(fragment_index))
                    .then_some(origin)
                    .and_then(|origin| {
                        Some((
                            origin.parameter?.as_algebraic_cusp()?,
                            origin.crossing_direction,
                        ))
                    })
            });
            let result = match source_contact {
                Some((parameter, crossing_direction)) => {
                    let result = fragment.forward_ray_winding_delta_skipping_origin(
                        point,
                        direction_x,
                        direction_y,
                        parameter,
                        crossing_direction,
                        policy,
                    )?;
                    if matches!(result, Classification::Decided(_)) {
                        source_origin_contact_was_skipped = true;
                    }
                    result
                }
                None if skipped_origin.is_some() => match fragment
                    .forward_ray_winding_delta_skipping_incident_origin(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )? {
                    Classification::Decided(Some(delta)) => {
                        if skipped_origin
                            .is_some_and(|origin| origin.fragment_index == Some(fragment_index))
                        {
                            source_origin_contact_was_skipped = true;
                        }
                        Classification::Decided(delta)
                    }
                    Classification::Decided(None) => fragment.forward_ray_winding_delta(
                        point,
                        direction_x,
                        direction_y,
                        policy,
                    )?,
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
                None => {
                    fragment.forward_ray_winding_delta(point, direction_x, direction_y, policy)?
                }
            };
            match result {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            continue;
        }
        let procedural_parallel = match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) if exact_parallel_curve.is_none() => {
                Some((
                    fragment.parallel(),
                    fragment.is_reversed(),
                    Some(fragment.range()),
                    None,
                ))
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                fragment.analytic_parallel().map(|parallel| {
                    (
                        parallel,
                        fragment.is_reversed(),
                        None,
                        Some(fragment.range()),
                    )
                })
            }
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        };
        if let Some((parallel, reversed, ordinary_range, selected_range)) = procedural_parallel {
            let regular_range = selected_range.cloned().unwrap_or_else(|| {
                CurveParameterRange2::from_bezier_range(
                    ordinary_range
                        .expect("an ordinary analytic fragment retains its range")
                        .clone(),
                )
            });
            let certified_origin_parameter =
                skipped_origin.and_then(|origin| {
                    if origin.fragment_index == Some(fragment_index) {
                        return origin
                            .parameter
                            .and_then(CurveParameter2::as_bezier_parameter)
                            .and_then(BezierParameter2::scalar);
                    }
                    let source_fragment_index = origin.fragment_index?;
                    let fragment_count = boundary_loop.fragments().len();
                    let adjacent = fragment_count > 1
                        && ((source_fragment_index + 1) % fragment_count == fragment_index
                            || (fragment_index + 1) % fragment_count == source_fragment_index);
                    adjacent.then_some(())?;
                    origin.tangent_contacts?.iter().find_map(|contact| {
                        match contact {
                    crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(
                        contact,
                    ) if contact.parallel == *parallel && contact.point == *point => {
                        Some(&contact.parameter)
                    }
                    crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(_)
                    | crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
                        ..
                    } => None,
                }
                    })
                });
            let certified_crossing = skipped_origin.and_then(|origin| {
                certified_origin_parameter
                    .map(|parameter| (parameter, origin.parameter_crossing_direction(reversed)))
            });
            let relation = match parallel.relation_to_supporting_line_on_regular_range(
                &ray.line,
                &regular_range,
                certified_crossing,
                policy,
            )? {
                Classification::Decided(relation) => relation,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match relation {
                BezierLineContactRelation::ControlHullDisjoint { .. }
                | BezierLineContactRelation::NoContact => {}
                BezierLineContactRelation::OnSupportingLine => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BezierLineContactRelation::Contacts { contacts } => {
                    let sole_crossing_contact = contacts.len() == 1
                        && contacts[0].kind() == BezierLineContactKind::Crossing;
                    for contact in contacts {
                        let retained = retained_curve_region_parameter_contains(
                            contact.parameter(),
                            &regular_range,
                            policy,
                        )?;
                        match retained {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                        if let Some(origin) = skipped_origin
                            && certified_origin_parameter.is_some()
                        {
                            let certified_parameter = BezierParameter2::Exact(
                                certified_origin_parameter
                                    .expect("a certified origin parameter was selected")
                                    .clone(),
                            );
                            let is_origin = retained_parameters_equal(
                                contact.parameter(),
                                &certified_parameter,
                                policy,
                            )?;
                            match is_origin {
                                Classification::Decided(true) => {
                                    if contact.kind() != BezierLineContactKind::Crossing {
                                        return Ok(Classification::Uncertain(
                                            UncertaintyReason::Boundary,
                                        ));
                                    }
                                    if origin.fragment_index == Some(fragment_index) {
                                        if source_origin_contact_was_skipped {
                                            return Ok(Classification::Uncertain(
                                                UncertaintyReason::Boundary,
                                            ));
                                        }
                                        source_origin_contact_was_skipped = true;
                                    }
                                    continue;
                                }
                                Classification::Decided(false) => {}
                                Classification::Uncertain(reason) => {
                                    // The exact representative lies on this
                                    // transverse source ray. A complete
                                    // singleton crossing is therefore the
                                    // origin even when its independently
                                    // isolated parameter cannot be ordered
                                    // against the represented witness.
                                    if sole_crossing_contact {
                                        if origin.fragment_index == Some(fragment_index) {
                                            source_origin_contact_was_skipped = true;
                                        }
                                        continue;
                                    }
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        match parallel.supporting_line_parameter_order_on_regular_range(
                            contact.parameter(),
                            &ray.line,
                            &regular_range,
                            policy,
                        )? {
                            Classification::Decided(std::cmp::Ordering::Greater) => {
                                match retained_line_contact_winding_delta(
                                    &contact,
                                    Some(&regular_range),
                                    reversed,
                                    policy,
                                )? {
                                    Classification::Decided(delta) => winding += delta,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                }
                            }
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                if let Some(origin) = skipped_origin
                                    && contact.kind() == BezierLineContactKind::Crossing
                                {
                                    if origin.fragment_index == Some(fragment_index) {
                                        if source_origin_contact_was_skipped {
                                            return Ok(Classification::Uncertain(
                                                UncertaintyReason::Boundary,
                                            ));
                                        }
                                        source_origin_contact_was_skipped = true;
                                    }
                                    continue;
                                }
                                return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                            }
                            Classification::Decided(std::cmp::Ordering::Less) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
            }
            continue;
        }
        let selected_curve = match fragment {
            BezierSplitFragment2::SelectedFiber(fragment) => fragment
                .rational_curve()
                .map(|curve| BezierSubcurve2::Rational(curve.clone())),
            _ => None,
        };
        let (curve, range, reversed) = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => (curve, None, false),
            BezierSplitFragment2::RetainedBezier {
                reversed,
                start,
                end,
                source_curve: curve,
                ..
            } => (
                curve,
                Some(CurveParameterRange2::new_validated(
                    CurveParameter2::from(start.clone()),
                    CurveParameter2::from(end.clone()),
                )),
                *reversed,
            ),
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => (
                exact_parallel_curve
                    .as_ref()
                    .expect("exact analytic component was selected above"),
                Some(CurveParameterRange2::from_bezier_range(
                    fragment.range().clone(),
                )),
                fragment.is_reversed(),
            ),
            BezierSplitFragment2::SelectedFiber(fragment) => (
                selected_curve
                    .as_ref()
                    .expect("an analytic selected-fiber fragment was handled procedurally"),
                Some(fragment.range().clone()),
                fragment.is_reversed(),
            ),
        };
        let unit = CurveParameterRange2::unit();
        let active_range = range.as_ref().unwrap_or(&unit);
        let unit_covers_range = matches!(
            crate::bezier_split::CurveParameterDomain2::new(&unit, None)
                .contains_finite_range(active_range, policy),
            Ok(Classification::Decided(true))
        );
        if unit_covers_range
            && !subcurve_control_hull_may_be_ahead(curve, point, direction_x, direction_y, policy)
        {
            continue;
        }
        let control_hull_order = unit_covers_range
            .then(|| {
                subcurve_control_hull_strict_order(curve, point, direction_x, direction_y, policy)
            })
            .flatten();
        let certified_source_crossing = skipped_origin.and_then(|origin| {
            (origin.fragment_index == Some(fragment_index))
                .then(|| {
                    origin
                        .parameter
                        .and_then(CurveParameter2::as_bezier_parameter)
                        .and_then(BezierParameter2::scalar)
                        .map(|parameter| (parameter, origin.parameter_crossing_direction(reversed)))
                })
                .flatten()
        });
        let certified_circle_relation =
            certified_source_crossing.and_then(|(parameter, crossing_direction)| {
                let retained_circle = match curve {
                    BezierSubcurve2::RationalQuadratic(curve) => {
                        curve.retained_circular_conic().is_some()
                    }
                    BezierSubcurve2::Rational(curve) => curve.retained_circular_conic().is_some(),
                    BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => false,
                };
                (unit_covers_range && retained_circle).then(|| {
                    RationalBezier2::try_from_subcurve(curve).map(|curve| {
                        curve.relation_to_line_with_certified_crossing(
                            &ray.line,
                            parameter,
                            crossing_direction,
                            policy,
                        )
                    })
                })
            });
        let relation = match certified_circle_relation.transpose() {
            Ok(Some(relation)) => relation,
            Ok(None) if !unit_covers_range => RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.relation_to_line_on_range(&ray.line, active_range, policy))
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            Ok(None) => subcurve_relation_to_line_with_contacts(
                curve,
                &ray.line,
                Some((direction_x, direction_y)),
                policy,
            ),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let relation = if unit_covers_range
            && active_range != &unit
            && matches!(relation, Classification::Uncertain(_))
        {
            // The unused part of the unit chart may contain a pole even
            // though this retained range is finite.
            RationalBezier2::try_from_subcurve(curve)
                .map(|curve| curve.relation_to_line_on_range(&ray.line, active_range, policy))
                .unwrap_or(relation)
        } else {
            relation
        };
        let relation = match relation {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {}
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => {
                let sole_crossing_contact =
                    contacts.len() == 1 && contacts[0].kind() == BezierLineContactKind::Crossing;
                for contact in contacts {
                    if let Some(origin) = skipped_origin
                        && origin.fragment_index == Some(fragment_index)
                    {
                        let retained_origin_parameter = || {
                            origin
                                .parameter
                                .and_then(CurveParameter2::as_bezier_parameter)
                                .map_or(
                                    Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                                    |parameter| {
                                        retained_parameters_equal(
                                            contact.parameter(),
                                            parameter,
                                            policy,
                                        )
                                    },
                                )
                        };
                        let is_origin =
                            if let Some(line_parameter) = contact.supporting_line_parameter() {
                                let strict_order = policy.strict_predicate_pass(|| {
                                    compare_reals(line_parameter, &Real::zero(), policy)
                                });
                                if let Some(order) = strict_order {
                                    Classification::Decided(order == std::cmp::Ordering::Equal)
                                } else {
                                    match policy.strict_predicate_pass(retained_origin_parameter)? {
                                        decided @ Classification::Decided(_) => decided,
                                        uncertain @ Classification::Uncertain(_)
                                            if sole_crossing_contact =>
                                        {
                                            uncertain
                                        }
                                        Classification::Uncertain(_) => {
                                            compare_reals(line_parameter, &Real::zero(), policy)
                                                .map(|order| {
                                                    Classification::Decided(
                                                        order == std::cmp::Ordering::Equal,
                                                    )
                                                })
                                                .map_or_else(retained_origin_parameter, Ok)?
                                        }
                                    }
                                }
                            } else {
                                // A sole crossing and the certified source
                                // incidence identify the origin even when its
                                // reconstructed scalar equality is undecided.
                                match policy.strict_predicate_pass(retained_origin_parameter)? {
                                    decided @ Classification::Decided(_) => decided,
                                    uncertain @ Classification::Uncertain(_)
                                        if sole_crossing_contact =>
                                    {
                                        uncertain
                                    }
                                    Classification::Uncertain(_) => retained_origin_parameter()?,
                                }
                            };
                        match is_origin {
                            Classification::Decided(true) => {
                                if source_origin_contact_was_skipped
                                    || contact.kind() != BezierLineContactKind::Crossing
                                {
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Boundary,
                                    ));
                                }
                                source_origin_contact_was_skipped = true;
                                continue;
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(_) if sole_crossing_contact => {
                                source_origin_contact_was_skipped = true;
                                continue;
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    let ahead = if let Some(line_parameter) = contact.supporting_line_parameter() {
                        compare_reals(line_parameter, &Real::zero(), policy)
                            .map(Classification::Decided)
                            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                    } else {
                        match contact.parameter() {
                            BezierParameter2::Exact(parameter) => {
                                if let Some(order) = control_hull_order {
                                    Classification::Decided(order)
                                } else {
                                    let contact_point =
                                        match subcurve_point_at(curve, parameter.clone(), policy) {
                                            Classification::Decided(point) => point,
                                            Classification::Uncertain(reason) => {
                                                return Ok(Classification::Uncertain(reason));
                                            }
                                        };
                                    let delta_x = contact_point.x() - point.x();
                                    let delta_y = contact_point.y() - point.y();
                                    let projection = Real::dot2_refs(
                                        [&delta_x, &delta_y],
                                        [direction_x, direction_y],
                                    );
                                    compare_reals(&projection, &Real::zero(), policy)
                                        .map(Classification::Decided)
                                        .unwrap_or(Classification::Uncertain(
                                            UncertaintyReason::RealSign,
                                        ))
                                }
                            }
                            BezierParameter2::Algebraic(parameter) => {
                                algebraic_contact_order_along_ray(
                                    curve,
                                    parameter,
                                    point,
                                    direction_x,
                                    direction_y,
                                    policy,
                                )?
                            }
                        }
                    };
                    let retained = retained_curve_region_parameter_contains(
                        contact.parameter(),
                        range.as_ref().unwrap_or(&CurveParameterRange2::unit()),
                        policy,
                    )?;
                    match retained {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            match retained_line_contact_winding_delta(
                                &contact,
                                range.as_ref(),
                                reversed,
                                policy,
                            )? {
                                Classification::Decided(delta) => winding += delta,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            if skipped_origin.is_some()
                                && contact.kind() == BezierLineContactKind::Crossing
                            {
                                continue;
                            }
                            return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                        }
                        Classification::Decided(std::cmp::Ordering::Less) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
    }
    if skipped_origin.is_some_and(|origin| origin.fragment_index.is_some())
        && !source_origin_contact_was_skipped
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    Ok(Classification::Decided(RetainedRayWinding::Winding(
        winding,
    )))
}

pub(super) fn retained_parameters_equal(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    first
        .cmp_by_refinement(second, policy)
        .map(|order| order.map(|order| order == std::cmp::Ordering::Equal))
}

pub(super) fn retained_curve_region_parameter_orders(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<[std::cmp::Ordering; 2]>> {
    let parameter = CurveParameter2::from(parameter.clone());
    let start = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(parameter
        .cmp_by_refinement(range.end(), policy)?
        .map(|end| [start, end]))
}

pub(super) fn retained_curve_region_parameter_contains(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    Ok(
        retained_curve_region_parameter_orders(parameter, range, policy)?.map(|[start, end]| {
            start != std::cmp::Ordering::Less && end != std::cmp::Ordering::Greater
        }),
    )
}

pub(super) fn rationalize_retained_subcurve(
    curve: &BezierSubcurve2,
) -> CurveResult<RationalBezier2> {
    RationalBezier2::try_from_subcurve(curve)
}

pub(super) fn native_loop_bounds(
    boundary_loop: &BezierBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let Some(first) = boundary_loop.fragments().first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut bounds = match subcurve_query_bounds(first, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for fragment in &boundary_loop.fragments()[1..] {
        let fragment_bounds = match subcurve_query_bounds(fragment, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        bounds = match bounds.union(&fragment_bounds) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
    }
    Classification::Decided(bounds)
}

pub(super) fn retained_loop_query_bounds(
    boundary_loop: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let mut fragments = boundary_loop.fragments().iter();
    let Some(first) = fragments.next() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut bounds = match retained_fragment_query_bounds(first, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for fragment in fragments {
        let fragment_bounds = match retained_fragment_query_bounds(fragment, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        bounds = match bounds.union(&fragment_bounds) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
    }
    Classification::Decided(bounds)
}

pub(super) fn retained_loops_have_pairwise_disjoint_bounds(
    boundary_loops: &[CurveRegionBoundaryLoop2],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut bounds = Vec::<Aabb2>::with_capacity(boundary_loops.len());
    for boundary_loop in boundary_loops {
        let current = match retained_loop_query_bounds(boundary_loop, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-disjoint-loop-roles-blocker",
                    "loop-bounds",
                );
                return Classification::Uncertain(reason);
            }
        };
        for previous in &bounds {
            match previous.overlaps(&current, policy) {
                Classification::Decided(false) => {}
                Classification::Decided(true) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-disjoint-loop-roles-blocker",
                        "overlapping-bounds",
                    );
                    return Classification::Decided(false);
                }
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-disjoint-loop-roles-blocker",
                        "bounds-overlap",
                    );
                    return Classification::Uncertain(reason);
                }
            }
        }
        bounds.push(current);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-disjoint-loop-roles",
        "pairwise-disjoint",
    );
    Classification::Decided(true)
}

pub(crate) fn retained_fragment_query_bounds(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
        return chord
            .conservative_local_bounds_refined(0, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let range = if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
        CurveParameterRange2::unit()
    } else {
        fragment.curve_region_parameter_range()
    };
    crate::curve_support::CurveSupport2::from_fragment(fragment)
        .certified_outer_bounds(&range, 0, policy)
}

/// Conservatively rejects a retained fragment box from an exact forward ray.
///
/// A linear projection reaches its extrema at box corners. The box is
/// disjoint when every corner is strictly on one side of the supporting line,
/// or strictly behind the ray origin. Only STRICT comparisons may prune a
/// fragment; unresolved signs retain it for the complete curve predicate.
pub(super) fn retained_bounds_may_intersect_forward_ray(
    bounds: &Aabb2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
) -> bool {
    let side_x = -direction_y.clone();
    let side_y = direction_x.clone();
    let zero = Real::zero();
    let mut all_side_negative = true;
    let mut all_side_positive = true;
    let mut all_behind = true;
    for (x, y) in [
        (bounds.min().x(), bounds.min().y()),
        (bounds.min().x(), bounds.max().y()),
        (bounds.max().x(), bounds.min().y()),
        (bounds.max().x(), bounds.max().y()),
    ] {
        let delta_x = x - origin.x();
        let delta_y = y - origin.y();
        let side = Real::dot2_refs([&delta_x, &delta_y], [&side_x, &side_y]);
        let forward = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        match compare_reals(&side, &zero, &CurveContext::STRICT) {
            Some(std::cmp::Ordering::Less) => all_side_positive = false,
            Some(std::cmp::Ordering::Greater) => all_side_negative = false,
            Some(std::cmp::Ordering::Equal) | None => {
                all_side_negative = false;
                all_side_positive = false;
            }
        }
        if compare_reals(&forward, &zero, &CurveContext::STRICT) != Some(std::cmp::Ordering::Less) {
            all_behind = false;
        }
    }
    !(all_side_negative || all_side_positive || all_behind)
}

pub(super) fn classify_point_with_ray(
    boundary_loop: &BezierBoundaryLoop2,
    point: &Point2,
    ray: &BezierRay2,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let direction_x = &ray.direction_x;
    let direction_y = &ray.direction_y;
    let mut winding = 0_i32;
    for fragment in boundary_loop.fragments() {
        if !subcurve_control_hull_may_be_ahead(fragment, point, direction_x, direction_y, policy) {
            continue;
        }
        let control_hull_order =
            subcurve_control_hull_strict_order(fragment, point, direction_x, direction_y, policy);
        let relation = match subcurve_relation_to_line_with_contacts(
            fragment,
            &ray.line,
            Some((direction_x, direction_y)),
            policy,
        ) {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {}
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => {
                for contact in contacts {
                    let ahead = if let Some(line_parameter) = contact.supporting_line_parameter() {
                        compare_reals(line_parameter, &Real::zero(), policy)
                            .map(Classification::Decided)
                            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
                    } else {
                        match contact.parameter() {
                            BezierParameter2::Exact(parameter) => {
                                if let Some(order) = control_hull_order {
                                    Classification::Decided(order)
                                } else {
                                    let contact_point = match subcurve_point_at(
                                        fragment,
                                        parameter.clone(),
                                        policy,
                                    ) {
                                        Classification::Decided(point) => point,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    let delta_x = contact_point.x() - point.x();
                                    let delta_y = contact_point.y() - point.y();
                                    let projection = Real::dot2_refs(
                                        [&delta_x, &delta_y],
                                        [direction_x, direction_y],
                                    );
                                    compare_reals(&projection, &Real::zero(), policy)
                                        .map(Classification::Decided)
                                        .unwrap_or(Classification::Uncertain(
                                            UncertaintyReason::RealSign,
                                        ))
                                }
                            }
                            BezierParameter2::Algebraic(parameter) => {
                                algebraic_contact_order_along_ray(
                                    fragment,
                                    parameter,
                                    point,
                                    direction_x,
                                    direction_y,
                                    policy,
                                )?
                            }
                        }
                    };
                    match ahead {
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            match retained_line_contact_winding_delta(
                                &contact, None, false, policy,
                            )? {
                                Classification::Decided(delta) => winding += delta,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            return Ok(Classification::Decided(ContourPointLocation::Boundary));
                        }
                        Classification::Decided(std::cmp::Ordering::Less) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        }
    }

    Ok(Classification::Decided(winding_location(
        winding, fill_rule,
    )))
}

pub(super) fn control_points_may_be_ahead<'a>(
    controls: impl IntoIterator<Item = &'a Point2>,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> bool {
    controls.into_iter().any(|control| {
        let delta_x = control.x() - origin.x();
        let delta_y = control.y() - origin.y();
        let projection = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        policy.strict_predicate_pass(|| real_sign(&projection, policy)) != Some(RealSign::Negative)
    })
}

pub(super) fn control_points_strict_order<'a>(
    controls: impl IntoIterator<Item = &'a Point2>,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    let mut order = None;
    for control in controls {
        let delta_x = control.x() - origin.x();
        let delta_y = control.y() - origin.y();
        let projection = Real::dot2_refs([&delta_x, &delta_y], [direction_x, direction_y]);
        let current = match policy.strict_predicate_pass(|| real_sign(&projection, policy))? {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => return None,
            RealSign::Positive => std::cmp::Ordering::Greater,
        };
        if order.is_some_and(|order| order != current) {
            return None;
        }
        order = Some(current);
    }
    order
}

pub(super) fn subcurve_control_hull_may_be_ahead(
    curve: &BezierSubcurve2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> bool {
    match curve {
        BezierSubcurve2::Quadratic(curve) => control_points_may_be_ahead(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::Cubic(curve) => control_points_may_be_ahead(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.common_nonzero_weight_sign(policy).is_none()
                || control_points_may_be_ahead(
                    curve.control_points(),
                    origin,
                    direction_x,
                    direction_y,
                    policy,
                )
        }
        BezierSubcurve2::Rational(_) => true,
    }
}

pub(super) fn subcurve_control_hull_strict_order(
    curve: &BezierSubcurve2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> Option<std::cmp::Ordering> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => control_points_strict_order(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::Cubic(curve) => control_points_strict_order(
            curve.control_points(),
            origin,
            direction_x,
            direction_y,
            policy,
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.common_nonzero_weight_sign(policy).and_then(|_| {
                control_points_strict_order(
                    curve.control_points(),
                    origin,
                    direction_x,
                    direction_y,
                    policy,
                )
            })
        }
        BezierSubcurve2::Rational(_) => None,
    }
}

/// Counts a contact from the strict ray side occupied inside the finite
/// source interval. A start owns its positive after-side; an end owns its
/// positive before-side. Reversal negates the contribution, not ownership.
/// One-sided endpoint contacts retain `tangent_side` even when the supporting
/// carrier cannot certify a continuation outside its domain.
pub(super) fn retained_line_contact_winding_delta(
    contact: &BezierLineContact,
    range: Option<&CurveParameterRange2>,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<i32>> {
    let unit = CurveParameterRange2::unit();
    let range = range.unwrap_or(&unit);
    let [start, end] =
        match retained_curve_region_parameter_orders(contact.parameter(), range, policy)? {
            Classification::Decided(orders) => orders,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    use std::cmp::Ordering::{Equal, Greater, Less};
    if start == Less || end == Greater || (start == Equal && end == Equal) {
        return Ok(Classification::Decided(0));
    }
    let at_start = start == Equal;
    let at_end = end == Equal;
    let (before_positive, after_positive) = match contact.kind() {
        BezierLineContactKind::Crossing => match contact.crossing_direction() {
            Some(BezierLineCrossingDirection::NegativeToPositive) => (false, true),
            Some(BezierLineCrossingDirection::PositiveToNegative) => (true, false),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        },
        BezierLineContactKind::Tangent if !at_start && !at_end => {
            return Ok(Classification::Decided(0));
        }
        BezierLineContactKind::Tangent => match contact.tangent_side() {
            Some(LineSide::Left) => (true, true),
            Some(LineSide::Right) => (false, false),
            Some(LineSide::On) | None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
        },
    };
    let delta = spatial_ray_winding_delta(before_positive, after_positive, at_start, at_end);
    Ok(Classification::Decided(if reversed {
        -delta
    } else {
        delta
    }))
}

/// Counts only the strict positive side inside the finite source interval.
/// Artificial cut endpoints cancel when adjacent pieces are added.
pub(super) fn spatial_ray_winding_delta(
    before_positive: bool,
    after_positive: bool,
    at_start: bool,
    at_end: bool,
) -> i32 {
    i32::from(!at_end && after_positive) - i32::from(!at_start && before_positive)
}

pub(super) fn winding_location(winding: i32, fill_rule: FillRule) -> ContourPointLocation {
    let inside = match fill_rule {
        FillRule::NonZero => winding != 0,
        FillRule::EvenOdd => winding.rem_euclid(2) != 0,
    };
    if inside {
        ContourPointLocation::Inside
    } else {
        ContourPointLocation::Outside
    }
}

pub(super) fn algebraic_contact_order_along_ray(
    curve: &BezierSubcurve2,
    parameter: &crate::BezierAlgebraicParameter2,
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    let (use_x, origin_coordinate, direction_sign) = match real_sign(direction_x, policy) {
        Some(RealSign::Positive) => (true, origin.x(), RealSign::Positive),
        Some(RealSign::Negative) => (true, origin.x(), RealSign::Negative),
        Some(RealSign::Zero) => match real_sign(direction_y, policy) {
            Some(RealSign::Positive) => (false, origin.y(), RealSign::Positive),
            Some(RealSign::Negative) => (false, origin.y(), RealSign::Negative),
            Some(RealSign::Zero) => return Err(CurveError::ZeroLengthLine),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        },
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let image = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.point_at_algebraic_parameter(parameter, policy),
        BezierSubcurve2::Cubic(curve) => curve.point_at_algebraic_parameter(parameter, policy),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.point_at_algebraic_parameter(parameter, policy)
        }
        BezierSubcurve2::Rational(curve) => curve.point_at_algebraic_parameter(parameter, policy),
    }?;
    let ordering = match image {
        Classification::Decided(image) => {
            image.coordinate_order_to_real(use_x, origin_coordinate, policy)?
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(ordering.map(|ordering| {
        if direction_sign == RealSign::Negative {
            ordering.reverse()
        } else {
            ordering
        }
    }))
}

pub(super) struct BezierRay2 {
    pub(super) line: LineSeg2,
    pub(super) direction_x: Real,
    pub(super) direction_y: Real,
}

pub(super) fn ray_candidates(point: &Point2) -> Vec<BezierRay2> {
    let one = Real::one();
    let two = Real::from(2_i8);
    let directions = [
        (-one.clone(), Real::zero()),
        (Real::zero(), -one.clone()),
        (-one.clone(), -two.clone()),
        (-two.clone(), -one.clone()),
        (-one.clone(), one.clone()),
        (one.clone(), Real::zero()),
        (Real::zero(), one.clone()),
        (one.clone(), two.clone()),
        (two, one.clone()),
        (one.clone(), -one),
    ];
    directions
        .into_iter()
        .map(|(direction_x, direction_y)| {
            let endpoint = Point2::new(point.x() + &direction_x, point.y() + &direction_y);
            BezierRay2 {
                line: LineSeg2::try_new(point.clone(), endpoint)
                    .expect("fixed exact ray directions are nonzero"),
                direction_x,
                direction_y,
            }
        })
        .collect()
}

/// Returns a conservative outer box for point-query rejection.
///
/// Tight extrema are unnecessary here: polynomial control hulls contain their
/// entire curves, as do rational control hulls after a common nonzero weight
/// sign is certified.
pub(super) fn subcurve_query_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::Cubic(curve) => Aabb2::from_points(curve.control_points()),
        BezierSubcurve2::RationalQuadratic(curve)
            if curve.common_nonzero_weight_sign(policy).is_some() =>
        {
            Aabb2::from_points(curve.control_points())
        }
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    }
}

pub(super) fn subcurve_point_at(
    curve: &BezierSubcurve2,
    parameter: Real,
    policy: &CurveContext,
) -> Classification<Point2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::Cubic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::RationalQuadratic(curve) => curve.point_at_with_policy(parameter, policy),
        // The owning fragment supplies the domain; retained endpoints and
        // interior samples may lie outside the author's unit interval.
        BezierSubcurve2::Rational(curve) => curve.point_at_affine_classified(&parameter, policy),
    }
}

pub(super) fn subcurve_contains_point(
    curve: &BezierSubcurve2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.contains_point_with_policy(point, policy),
        BezierSubcurve2::Cubic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        )
        .map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            |curve| curve.contains_point_classified(point, policy),
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.contains_point_with_policy(point, policy)
        }
        BezierSubcurve2::Rational(curve) => curve.contains_point_classified(point, policy),
    }
}

pub(super) fn subcurve_relation_to_line_with_contacts(
    curve: &BezierSubcurve2,
    line: &LineSeg2,
    direction: Option<(&Real, &Real)>,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => direction.map_or_else(
            || curve.relation_to_line_with_contacts(line, policy),
            |(direction_x, direction_y)| {
                let original = curve.relation_to_line_with_contacts(line, policy);
                match original {
                    Classification::Decided(_) => original,
                    Classification::Uncertain(_) => {
                        exact_polynomial_line_contact_relation_from_direction(
                            &curve.control_points(),
                            line.start(),
                            direction_x,
                            direction_y,
                            policy,
                        )
                    }
                }
            },
        ),
        BezierSubcurve2::Cubic(curve) => direction.map_or_else(
            || curve.relation_to_line_with_contacts(line, policy),
            |(direction_x, direction_y)| {
                let original = curve.relation_to_line_with_contacts(line, policy);
                match original {
                    Classification::Decided(_) => original,
                    Classification::Uncertain(_) => {
                        exact_polynomial_line_contact_relation_from_direction(
                            &curve.control_points(),
                            line.start(),
                            direction_x,
                            direction_y,
                            policy,
                        )
                    }
                }
            },
        ),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.relation_to_line_with_contacts(line, policy)
        }
        BezierSubcurve2::Rational(curve) => {
            if curve.retained_circular_conic().is_some() {
                match curve.materialized_quadratic_representative(policy) {
                    Ok(Classification::Decided(Some(quadratic))) => {
                        return quadratic.relation_to_line_with_contacts(line, policy);
                    }
                    Ok(Classification::Decided(None)) => {}
                    Ok(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                }
            }
            curve.relation_to_line_with_contacts(line, policy)
        }
    }
}
