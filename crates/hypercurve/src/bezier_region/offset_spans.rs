//! Exact offset spans from materialized, parallel and cusp sources.

use super::*;

pub(crate) enum CurveTangent2 {
    /// Represented traversal direction. Its magnitude is arbitrary; metric
    /// constructions must request a certified unit direction explicitly.
    RepresentedDirection((Real, Real)),
    /// Traversal tangent of an analytic parallel at an algebraic parameter.
    /// `source_direction` is the nonzero orientation relative to the
    /// parallel source's homogeneous tangent numerator.
    RetainedParallel {
        parallel: BezierParallel2,
        source_parallel: BezierParallel2,
        source_range: CurveParameterRange2,
        parameter: BezierParameter2,
        /// Original compact selected-fiber scalar, when this tangent came
        /// from a selected region fragment. Keeping its one-word authority
        /// avoids re-proving equality against the promoted global root.
        selected_source_parameter:
            Option<crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2>,
        source_direction: RealSign,
    },
    AlgebraicChord(crate::BezierAlgebraicChord2),
    CircularPoint {
        point: CurvePoint2,
        circle: Arc<crate::rational_bezier::RationalQuadraticCircle2>,
        clockwise: bool,
    },
    SelectedCircularEndpoint {
        /// Source carrier before the concentric offset.  Smooth carrier
        /// switches can need its exact overlap map after the two offset
        /// endpoint images have moved into independent selected fields.
        source_fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
    },
    ChordContact {
        fragment: crate::BezierAlgebraicCuspSemicircleFragment2,
        at_start: bool,
        chord: crate::BezierAlgebraicChord2,
        circle_cross_chord: RealSign,
        circle_dot_chord: Option<RealSign>,
    },
}

impl CurveTangent2 {
    /// Retains a boundary's traversal direction in the same geometric chart
    /// used by offsets. No offset construction or coordinate projection is
    /// needed to compare endpoint directions in an arrangement.
    pub(crate) fn at_boundary_endpoint(
        fragment: &BezierSplitFragment2,
        at_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let (parallel, range, reversed) = match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                return Ok(Classification::Decided(Self::AlgebraicChord(chord.clone())));
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(circle) => {
                return Ok(
                    match selected_circle_endpoint_tangent(circle, circle, at_start, policy)? {
                        Classification::Decided(Some(tangent)) => Classification::Decided(tangent),
                        Classification::Decided(None) => {
                            Classification::Uncertain(UncertaintyReason::Unsupported)
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    },
                );
            }
            BezierSplitFragment2::Materialized { curve, .. } => {
                // A polynomial Bezier's endpoint derivative is n times its
                // first or last control difference; no power basis is needed.
                let polynomial_endpoint = |controls: &[&Point2]| {
                    let degree = Real::from(controls.len() as i64 - 1);
                    let (from, to) = if at_start {
                        (controls[0], controls[1])
                    } else {
                        (controls[controls.len() - 2], controls[controls.len() - 1])
                    };
                    crate::CurveDerivative2::new(
                        (to.x() - from.x()) * &degree,
                        (to.y() - from.y()) * &degree,
                    )
                };
                let derivative = match curve {
                    BezierSubcurve2::Quadratic(curve) => {
                        polynomial_endpoint(&curve.control_points())
                    }
                    BezierSubcurve2::Cubic(curve) => polynomial_endpoint(&curve.control_points()),
                    BezierSubcurve2::RationalQuadratic(_) | BezierSubcurve2::Rational(_) => {
                        let curve = RationalBezier2::try_from_subcurve(curve)?;
                        let parameter = if at_start { Real::zero() } else { Real::one() };
                        match curve.derivative_at_classified(&parameter, policy) {
                            Classification::Decided(derivative) => derivative,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                };
                if derivative.zero_status() != hyperreal::ZeroKnowledge::NonZero
                    && ![derivative.dx(), derivative.dy()]
                        .into_iter()
                        .any(|component| is_zero(component, policy) == Some(false))
                {
                    // Stationary endpoints require a one-sided higher-order
                    // direction. A zero derivative cannot order face branches.
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                return Ok(Classification::Decided(Self::RepresentedDirection((
                    derivative.dx().clone(),
                    derivative.dy().clone(),
                ))));
            }
            BezierSplitFragment2::RetainedBezier {
                source_curve,
                start,
                end,
                reversed,
                ..
            } => (
                RationalBezier2::try_from_subcurve(source_curve)?.parallel_left(Real::zero())?,
                CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
                    start.clone(),
                    end.clone(),
                )),
                *reversed,
            ),
            BezierSplitFragment2::AnalyticParallel(fragment) => (
                fragment.parallel().clone(),
                CurveParameterRange2::from_bezier_range(fragment.range().clone()),
                fragment.is_reversed(),
            ),
            BezierSplitFragment2::SelectedFiber(fragment) => (
                fragment.parallel_carrier(),
                fragment.range().clone(),
                fragment.is_reversed(),
            ),
        };
        let range = if matches!(fragment, BezierSplitFragment2::SelectedFiber(_)) {
            let analysis = match parallel.singularity_analysis(&range, policy)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let mut ranges = match analysis.regular_subranges(policy)? {
                Classification::Decided(ranges) => ranges,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if at_start != reversed {
                ranges.remove(0)
            } else {
                ranges
                    .pop()
                    .expect("a regular partition retains the source range")
            }
        } else {
            range
        };
        let scale = match retained_parallel_range_scale_sign(&parallel, &range, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(scale) => scale,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let parameter = if at_start != reversed {
            range.start()
        } else {
            range.end()
        };
        exact_parallel_region_endpoint_tangent(
            &parallel, &parallel, &range, parameter, scale, reversed, policy,
        )
    }

    /// Orders outgoing directions around an incoming traversal with its filled
    /// face on the left. Equal directions retain their higher-order ambiguity.
    pub(crate) fn compare_filled_left_turn(
        &self,
        first: &Self,
        second: &Self,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        let half = |candidate| match curve_tangent_cross_sign(self, candidate, policy) {
            Classification::Decided(RealSign::Positive) => Classification::Decided((0_u8, false)),
            Classification::Decided(RealSign::Negative) => Classification::Decided((1, false)),
            Classification::Decided(RealSign::Zero) => {
                curve_tangents_are_opposite(self, candidate, policy)
                    .map(|opposite| (u8::from(opposite), true))
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        let ((first_half, first_collinear), (second_half, second_collinear)) =
            match (half(first), half(second)) {
                (Classification::Decided(first), Classification::Decided(second)) => {
                    (first, second)
                }
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
            };
        if first_half != second_half {
            return Classification::Decided(first_half.cmp(&second_half));
        }
        // In either half, a ray parallel to the reference is the last in
        // clockwise order. Reuse that certified relation instead of joining
        // the two candidate fields to rediscover their determinant sign.
        // Two such rays in the same half have the same direction.
        if first_collinear || second_collinear {
            return Classification::Decided(first_collinear.cmp(&second_collinear));
        }
        curve_tangent_cross_sign(first, second, policy).map(|sign| match sign {
            RealSign::Positive => std::cmp::Ordering::Greater,
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
        })
    }
}

pub(super) fn exact_offset_tangent_is_selected_circle(tangent: &CurveTangent2) -> bool {
    matches!(
        tangent,
        CurveTangent2::SelectedCircularEndpoint { .. } | CurveTangent2::ChordContact { .. }
    )
}

/// Certifies one monotone bevel coordinate directly from incident tangent
/// component signs. Unit normalization preserves each component sign; when
/// the two signs differ, the corresponding normal-component difference has a
/// strict sign independent of either speed magnitude.
pub(super) fn exact_offset_bevel_parameter_axis(
    previous: &CurveTangent2,
    next: &CurveTangent2,
    turn_sign: RealSign,
    distance_sign: RealSign,
    policy: &CurveContext,
) -> Option<(crate::Axis2, bool)> {
    let (CurveTangent2::AlgebraicChord(previous), CurveTangent2::AlgebraicChord(next)) =
        (previous, next)
    else {
        return None;
    };
    let component_difference = |first, second| match (first, second) {
        (RealSign::Negative, RealSign::Zero | RealSign::Positive)
        | (RealSign::Zero, RealSign::Positive) => Some(RealSign::Positive),
        (RealSign::Positive, RealSign::Zero | RealSign::Negative)
        | (RealSign::Zero, RealSign::Negative) => Some(RealSign::Negative),
        _ => None,
    };
    let mut resolved_signs = [(None, None); 2];
    for (axis_index, tangent_axis) in [crate::Axis2::X, crate::Axis2::Y].into_iter().enumerate() {
        let (previous_certified, next_certified) = (
            previous.certified_tangent_axis_sign(tangent_axis),
            next.certified_tangent_axis_sign(tangent_axis),
        );
        let component_sign = |chord: &crate::BezierAlgebraicChord2, certified: Option<RealSign>| {
            certified.or_else(|| match chord.tangent_axis_sign(tangent_axis, policy) {
                Ok(Classification::Decided(sign)) => Some(sign),
                Ok(Classification::Uncertain(_)) | Err(_) => None,
            })
        };
        let (Some(previous_sign), Some(next_sign)) = (
            component_sign(previous, previous_certified),
            component_sign(next, next_certified),
        ) else {
            continue;
        };
        resolved_signs[axis_index] = (Some(previous_sign), Some(next_sign));
        let difference = component_difference(previous_sign, next_sign);
        let Some(difference) = difference else {
            continue;
        };
        let (axis, normal_difference) = match tangent_axis {
            // N_y = T_x / |T|.
            crate::Axis2::X => (crate::Axis2::Y, difference),
            // N_x = -T_y / |T|.
            crate::Axis2::Y => (crate::Axis2::X, exact_sign_reverse(difference)),
        };
        let sign = exact_sign_product(distance_sign, normal_difference);
        if sign == RealSign::Zero {
            return None;
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-bevel-axis",
            "separated-tangent-components",
        );
        return Some((axis, sign == RealSign::Positive));
    }
    let [
        (Some(previous_x), Some(next_x)),
        (Some(previous_y), Some(next_y)),
    ] = resolved_signs
    else {
        return None;
    };
    if previous_x == next_x
        && previous_y == next_y
        && previous_x != RealSign::Zero
        && previous_y != RealSign::Zero
        && turn_sign != RealSign::Zero
    {
        // Both unit tangents lie in one open quadrant. That quadrant is
        // narrower than pi, so the cross sign fixes their angular order and
        // component monotonicity without comparing either normalized
        // magnitude: d(T_x)/d(theta)=-T_y.
        let difference_x = exact_sign_product(turn_sign, exact_sign_reverse(previous_y));
        let sign = exact_sign_product(distance_sign, difference_x);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-bevel-axis",
            "same-quadrant-turn",
        );
        return Some((crate::Axis2::Y, sign == RealSign::Positive));
    }
    None
}

pub(super) fn retained_chord_fragment(chord: crate::BezierAlgebraicChord2) -> BezierSplitFragment2 {
    BezierSplitFragment2::AlgebraicChord(chord)
}

pub(super) fn append_exact_algebraic_line_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    from: &crate::CurvePoint2,
    to: &crate::CurvePoint2,
    certified_direction: Option<BezierAlgebraicChordAxisDirection2>,
    certified_parameter_axis: Option<(crate::Axis2, bool)>,
    certified_distinct: bool,
    certified_circle_transverse_endpoints: [bool; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let endpoint_equality = if certified_direction.is_some()
        || certified_parameter_axis.is_some()
        || certified_distinct
    {
        Classification::Decided(false)
    } else {
        from.same_point(to, policy)
    };
    match endpoint_equality {
        Classification::Decided(true) => Ok(Classification::Decided(())),
        Classification::Decided(false) => {
            if let (Some(from), Some(to)) = (from.coordinates(), to.coordinates()) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-line-join",
                    "represented-endpoints",
                );
                let line = LineSeg2::try_new(from.clone(), to.clone())?;
                fragments.push(materialized_offset_fragment(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line),
                )));
                return Ok(Classification::Decided(()));
            }
            let chord = if let Some(direction) = certified_direction {
                crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                    from.clone(),
                    to.clone(),
                    direction,
                    policy,
                )
            } else if let Some((axis, coordinate_increases)) = certified_parameter_axis {
                crate::BezierAlgebraicChord2::from_certified_monotone_axis_endpoints(
                    from.clone(),
                    to.clone(),
                    axis,
                    coordinate_increases,
                    policy,
                )
            } else {
                let chord = if certified_distinct {
                    crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        from.clone(),
                        to.clone(),
                        policy,
                    )?
                } else {
                    crate::BezierAlgebraicChord2::try_new(from.clone(), to.clone(), policy)?
                };
                match chord {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let chord = chord
                .with_certified_circle_transverse_endpoints(certified_circle_transverse_endpoints);
            fragments.push(retained_chord_fragment(chord));
            Ok(Classification::Decided(()))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn exact_circular_algebraic_endpoint_tangent(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    point: &CurvePoint2,
    circle: &Arc<crate::rational_bezier::RationalQuadraticCircle2>,
    clockwise: bool,
    reversed: bool,
    policy: &CurveContext,
) -> Classification<Option<CurveTangent2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => curve
            .derivative_at_classified(parameter, policy)
            .map(|derivative| {
                let tangent = (derivative.dx().clone(), derivative.dy().clone());
                Some(CurveTangent2::RepresentedDirection(if reversed {
                    (-tangent.0, -tangent.1)
                } else {
                    tangent
                }))
            }),
        BezierParameter2::Algebraic(_) => {
            Classification::Decided(Some(CurveTangent2::CircularPoint {
                point: point.clone(),
                circle: Arc::clone(circle),
                clockwise: clockwise != reversed,
            }))
        }
    }
}

pub(super) fn exact_offset_spans_from_algebraic_endpoint_images(
    reversed: bool,
    start: &BezierParameter2,
    end: &BezierParameter2,
    source: &BezierSubcurve2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let general_offset = || {
        let parallel = retained_subcurve_parallel(source, Real::zero())?;
        let fragment = crate::BezierParallelFragment2::from_certified_range(
            parallel,
            BezierParameterRange2::new_validated(start.clone(), end.clone()),
            reversed,
        );
        exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Analytic(&fragment),
            distance,
            policy,
        )
    };
    let BezierSubcurve2::RationalQuadratic(source_curve) = source else {
        return general_offset();
    };
    // Circular recognition selects a smaller native carrier, so it must be a
    // STRICT certificate.  An unresolved or noncircular conic simply rejoins
    // the complete analytic-parallel path below.
    let source_arc = match crate::arc_bezier::rational_quadratic_circular_arc(
        source_curve,
        &CurveContext::STRICT,
    )? {
        Classification::Decided(Some(arc)) => arc,
        Classification::Decided(None) | Classification::Uncertain(_) => return general_offset(),
    };
    let source_subcurve = BezierSubcurve2::RationalQuadratic(source_curve.clone());
    let source_rational = RationalBezier2::try_from_subcurve(&source_subcurve)?;
    let (traversal_start, traversal_end) = if reversed { (end, start) } else { (start, end) };
    let source_end = match crate::rational_bezier_general::exact_contact_point_evidence(
        &source_rational,
        traversal_end,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let carrier_distance = if reversed {
        -distance
    } else {
        distance.clone()
    };
    let radial_scale = source_arc.left_offset_radius_scale(&carrier_distance)?;
    match real_sign(&radial_scale, policy) {
        Some(RealSign::Zero) => {
            let center = CurvePoint2::from(source_arc.center().clone());
            return Ok(Classification::Decided(vec![ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end,
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }]));
        }
        Some(RealSign::Positive | RealSign::Negative) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let scale_point = |point: &Point2| {
        let (x, y) = point.delta_from(source_arc.center());
        source_arc
            .center()
            .translated(&x * &radial_scale, &y * &radial_scale)
    };
    let radius_squared = source_arc.radius_squared_ref() * &radial_scale * &radial_scale;
    let two = Real::from(2_i8);
    let implicit = Arc::new([
        Real::one(),
        Real::zero(),
        Real::one(),
        -(&two * source_arc.center().x()),
        -(&two * source_arc.center().y()),
        source_arc.center().x() * source_arc.center().x()
            + source_arc.center().y() * source_arc.center().y()
            - &radius_squared,
    ]);
    let offset_circle = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
        center: source_arc.center().clone(),
        radius_squared,
        tangent_contacts: None,
    });
    let offset_curve =
        crate::RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
            scale_point(source_curve.start()),
            scale_point(source_curve.control()),
            scale_point(source_curve.end()),
            source_curve.start_weight().clone(),
            source_curve.control_weight().clone(),
            source_curve.end_weight().clone(),
            source_curve.common_nonzero_weight_sign(policy),
            Some(implicit),
            Some(Arc::clone(&offset_circle)),
        )?;
    let offset_subcurve = BezierSubcurve2::RationalQuadratic(offset_curve);
    let endpoint_image = |parameter: &BezierParameter2| -> CurveResult<_> {
        match parameter {
            BezierParameter2::Exact(_) => Ok(Classification::Decided(None)),
            BezierParameter2::Algebraic(parameter) => {
                BezierAlgebraicEndpointImage2::from_source_curve(
                    &offset_subcurve,
                    parameter,
                    policy,
                )
                .map(|image| image.map(Some))
            }
        }
    };
    let start_image = match endpoint_image(start)? {
        Classification::Decided(image) => image,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_image = match endpoint_image(end)? {
        Classification::Decided(image) => image,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_rational = RationalBezier2::try_from_subcurve(&offset_subcurve)?;
    let offset_start = match crate::rational_bezier_general::exact_contact_point_evidence(
        &offset_rational,
        traversal_start,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_end = match crate::rational_bezier_general::exact_contact_point_evidence(
        &offset_rational,
        traversal_end,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let start_tangent = match exact_circular_algebraic_endpoint_tangent(
        &offset_rational,
        traversal_start,
        &offset_start,
        &offset_circle,
        source_arc.is_clockwise(),
        reversed,
        policy,
    ) {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end_tangent = match exact_circular_algebraic_endpoint_tangent(
        &offset_rational,
        traversal_end,
        &offset_end,
        &offset_circle,
        source_arc.is_clockwise(),
        reversed,
        policy,
    ) {
        Classification::Decided(tangent) => tangent,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(vec![ExactOffsetSpan2 {
        fragments: vec![BezierSplitFragment2::RetainedBezier {
            reversed,
            start: start.clone(),
            end: end.clone(),
            source_curve: offset_subcurve,
            start_image,
            end_image,
        }],
        source_end,
        offset_start,
        offset_end,
        start_tangent,
        end_tangent,
    }]))
}

pub(super) fn exact_offset_spans_from_source_singular_parallel(
    curve: &BezierSubcurve2,
    parallel: &BezierParallel2,
    analysis: &crate::BezierParallelSingularityAnalysis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let mut source_boundaries = vec![(zero.clone(), false)];
    for singularity in analysis.source_singularities() {
        let after_zero = match singularity.cmp_by_refinement(&zero, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_one = match singularity.cmp_by_refinement(&one, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match (after_zero, before_one) {
            (std::cmp::Ordering::Equal, _) => source_boundaries[0].1 = true,
            (_, std::cmp::Ordering::Equal) => {}
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                if let Some((previous, _)) = source_boundaries.last() {
                    match singularity.cmp_by_refinement(previous, policy)? {
                        Classification::Decided(std::cmp::Ordering::Greater) => {}
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(std::cmp::Ordering::Less) => {
                            return Err(CurveError::Topology(
                                "source singularities were not ordered".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                source_boundaries.push((singularity.clone(), true));
            }
            _ => {
                return Err(CurveError::Topology(
                    "source singularity escaped the unit parameter domain".into(),
                ));
            }
        }
    }
    let end_is_singular = analysis.source_singularities().iter().any(|singularity| {
        matches!(
            singularity.cmp_by_refinement(&one, policy),
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        )
    });
    source_boundaries.push((one, end_is_singular));

    let source_rational = RationalBezier2::try_from_subcurve(curve)?;
    let mut spans = Vec::with_capacity(source_boundaries.len().saturating_sub(1));
    for branch in source_boundaries.windows(2) {
        let source_range =
            BezierParameterRange2::new_validated(branch[0].0.clone(), branch[1].0.clone());
        let mut boundaries = vec![(branch[0].0.clone(), branch[0].1)];
        for cusp in analysis.parallel_cusps() {
            let after_start = match cusp.cmp_by_refinement(source_range.start(), policy)? {
                Classification::Decided(order) => order.is_gt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before_end = match cusp.cmp_by_refinement(source_range.end(), policy)? {
                Classification::Decided(order) => order.is_lt(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after_start && before_end {
                boundaries.push((cusp.clone(), false));
            }
        }
        boundaries.push((branch[1].0.clone(), branch[1].1));
        let ranges = boundaries
            .windows(2)
            .map(|pair| BezierParameterRange2::new_validated(pair[0].0.clone(), pair[1].0.clone()))
            .collect::<Vec<_>>();
        let start_scale = match parallel.regular_fragment_derivative_scale_sign(
            ranges
                .first()
                .expect("a source branch has one regular offset range"),
            policy,
        )? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_scale = match parallel.regular_fragment_derivative_scale_sign(
            ranges
                .last()
                .expect("a source branch has one regular offset range"),
            policy,
        )? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let endpoint = |parameter: &BezierParameter2,
                        singular: bool,
                        scale: RealSign|
         -> CurveResult<Classification<(CurvePoint2, CurveTangent2)>> {
            if singular {
                return parallel
                    .regular_source_point_and_tangent_support(
                        parallel,
                        &parameter.clone().into(),
                        &CurveParameterRange2::from_bezier_range(source_range.clone()),
                        scale,
                        policy,
                    )
                    .map(|frame| {
                        frame
                            .map(|(point, tangent)| (point, CurveTangent2::AlgebraicChord(tangent)))
                    });
            }
            let point = match exact_parallel_point_evidence(parallel, parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let Some(parameter) = parameter.scalar() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let tangent = match parallel.derivative_at(parameter, policy)? {
                Classification::Decided(derivative) => CurveTangent2::RepresentedDirection((
                    derivative.dx().clone(),
                    derivative.dy().clone(),
                )),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided((point, tangent)))
        };
        let (offset_start, start_tangent) =
            match endpoint(source_range.start(), branch[0].1, start_scale)? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (offset_end, end_tangent) = match endpoint(source_range.end(), branch[1].1, end_scale)?
        {
            Classification::Decided(endpoint) => endpoint,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let mut endpoint_points = Vec::with_capacity(boundaries.len());
        for (index, (parameter, singular)) in boundaries.iter().enumerate() {
            if index == 0 {
                endpoint_points.push(offset_start.clone());
            } else if index + 1 == boundaries.len() {
                endpoint_points.push(offset_end.clone());
            } else if *singular {
                return Err(CurveError::Topology(
                    "a source singularity remained inside one regular branch".into(),
                ));
            } else {
                match exact_parallel_point_evidence(parallel, parameter, policy)? {
                    Classification::Decided(point) => endpoint_points.push(point),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let fragments = boundaries
            .windows(2)
            .zip(endpoint_points.windows(2))
            .map(|(parameters, points)| {
                if parameters[0].1 || parameters[1].1 {
                    BezierSplitFragment2::SelectedFiber(
                        crate::bezier_split::BezierSelectedFiberFragment2::new(
                            BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                            CurveParameterRange2::new_validated(
                                CurveParameter2::from(parameters[0].0.clone()),
                                CurveParameter2::from(parameters[1].0.clone()),
                            ),
                            points[0].clone(),
                            points[1].clone(),
                        ),
                    )
                } else {
                    BezierSplitFragment2::AnalyticParallel(
                        crate::BezierParallelFragment2::from_certified_range(
                            parallel.clone(),
                            BezierParameterRange2::new_validated(
                                parameters[0].0.clone(),
                                parameters[1].0.clone(),
                            ),
                            false,
                        ),
                    )
                }
            })
            .collect();
        let source_end = match crate::rational_bezier_general::exact_contact_point_evidence(
            &source_rational,
            source_range.end(),
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        spans.push(ExactOffsetSpan2 {
            fragments,
            source_end,
            offset_start,
            offset_end,
            start_tangent: Some(start_tangent),
            end_tangent: Some(end_tangent),
        });
    }
    // A stationary source parameter is not necessarily a geometric corner.
    // When both one-sided constructions prove the same offset endpoint and
    // codirected tangent, the common hodograph factor has preserved the normal
    // sheet. Keep the exact split fragments, but collapse their bookkeeping
    // into one span so stroke/offset composition does not Boolean two pieces of
    // one smooth band. Odd-multiplicity reversals have distinct nonzero-offset
    // endpoints (and opposite tangents at zero distance), so remain separate.
    let mut merged = Vec::<ExactOffsetSpan2>::with_capacity(spans.len());
    for span in spans {
        let joins_previous = merged.last().is_some_and(|previous| {
            policy.strict_predicate_pass(|| {
                previous.offset_end.same_point(&span.offset_start, policy)
                    == Classification::Decided(true)
                    && matches!(
                        previous.end_tangent.as_ref().zip(span.start_tangent.as_ref()),
                        Some((first, second))
                            if curve_tangent_cross_sign(first, second, policy)
                                == Classification::Decided(RealSign::Zero)
                                && curve_tangents_are_opposite(first, second, policy)
                                    == Classification::Decided(false)
                    )
            })
        });
        if joins_previous {
            let previous = merged
                .last_mut()
                .expect("a proven source-sheet join has a preceding span");
            previous.fragments.extend(span.fragments);
            previous.source_end = span.source_end;
            previous.offset_end = span.offset_end;
            previous.end_tangent = span.end_tangent;
        } else {
            merged.push(span);
        }
    }
    Ok(Classification::Decided(merged))
}

pub(super) fn exact_offset_spans_from_materialized_curve(
    curve: &BezierSubcurve2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    if let Classification::Decided(segment) = materialized_native_subcurve_segment(curve, policy)? {
        let native_offset = match &segment {
            Segment2::Line(line) => {
                Classification::Decided(Segment2::Line(line.offset_left(distance.clone())?))
            }
            Segment2::Arc(arc) => {
                match exact_offset_span_from_native_arc(curve, arc, distance, policy)? {
                    Classification::Decided(span) => {
                        return Ok(Classification::Decided(vec![span]));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        };
        if let Classification::Decided(offset) = native_offset {
            return exact_offset_span_from_native_segment(curve, &offset, policy)
                .map(|span| span.map(|span| vec![span]));
        }
    }

    let source = match curve {
        BezierSubcurve2::Quadratic(curve) => BezierParallelSource2::Quadratic(curve.clone()),
        BezierSubcurve2::Cubic(curve) => BezierParallelSource2::Cubic(curve.clone()),
        BezierSubcurve2::RationalQuadratic(curve) => {
            BezierParallelSource2::Rational(curve.clone().into())
        }
        BezierSubcurve2::Rational(curve) => BezierParallelSource2::Rational(curve.clone()),
    };
    let parallel = BezierParallel2::from_source(source, distance.clone());
    let analysis = match parallel.singularity_analysis(&CurveParameterRange2::unit(), policy)? {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !analysis.source_is_regular() {
        return exact_offset_spans_from_source_singular_parallel(
            curve, &parallel, &analysis, policy,
        );
    }

    let mut boundaries = Vec::with_capacity(analysis.parallel_cusps().len() + 2);
    boundaries.push(BezierParameter2::Exact(Real::zero()));
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    for cusp in analysis.parallel_cusps() {
        let after_zero = match cusp.cmp_by_refinement(&zero, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_one = match cusp.cmp_by_refinement(&one, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if after_zero == std::cmp::Ordering::Greater && before_one == std::cmp::Ordering::Less {
            let order = cusp.cmp_by_refinement(
                boundaries
                    .last()
                    .expect("parallel split inventory begins at zero"),
                policy,
            )?;
            match order {
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    boundaries.push(cusp.clone());
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {}
                Classification::Decided(std::cmp::Ordering::Less) => {
                    return Err(CurveError::Topology(
                        "parallel cusp isolators are not ordered".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    boundaries.push(one);

    let fragments = if boundaries.len() == 2 {
        match parallel.exact_pythagorean_hodograph_offset(policy)? {
            Classification::Decided(Some(offset)) => vec![materialized_offset_fragment(
                BezierSubcurve2::Rational(offset.curve().clone()),
            )],
            Classification::Decided(None) | Classification::Uncertain(_) => {
                exact_parallel_fragments(&parallel, &boundaries, false)
            }
        }
    } else {
        exact_parallel_fragments(&parallel, &boundaries, false)
    };
    let offset_start = match parallel.point_at(&Real::zero(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let offset_end = match parallel.point_at(&Real::one(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let start_tangent = match parallel.derivative_at(&Real::zero(), policy)? {
        Classification::Decided(derivative) => (derivative.dx().clone(), derivative.dy().clone()),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_tangent = match parallel.derivative_at(&Real::one(), policy)? {
        Classification::Decided(derivative) => (derivative.dx().clone(), derivative.dy().clone()),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(vec![ExactOffsetSpan2 {
        fragments,
        source_end: curve.end().clone().into(),
        offset_start: offset_start.into(),
        offset_end: offset_end.into(),
        start_tangent: Some(CurveTangent2::RepresentedDirection(start_tangent)),
        end_tangent: Some(CurveTangent2::RepresentedDirection(end_tangent)),
    }]))
}

pub(super) fn exact_offset_span_from_native_arc(
    source: &BezierSubcurve2,
    arc: &CircularArc2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let radius_scale = arc.left_offset_radius_scale(distance)?;
    match real_sign(&radius_scale, policy) {
        Some(RealSign::Zero) => {
            let center = CurvePoint2::from(arc.center().clone());
            Ok(Classification::Decided(ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end: source.end().clone().into(),
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }))
        }
        Some(RealSign::Positive | RealSign::Negative) => {
            let scale_point = |point: &Point2| {
                let (delta_x, delta_y) = point.delta_from(arc.center());
                arc.center()
                    .translated(&delta_x * &radius_scale, &delta_y * &radius_scale)
            };
            let offset = CircularArc2::try_from_center_with_bulge(
                scale_point(arc.start()),
                scale_point(arc.end()),
                arc.center().clone(),
                arc.is_clockwise(),
                arc.bulge().cloned(),
            )?;
            exact_offset_span_from_native_segment(source, &Segment2::Arc(offset), policy)
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

pub(super) fn exact_offset_span_from_algebraic_chord(
    chord: &crate::BezierAlgebraicChord2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    if let (Some(start), Some(end)) = (chord.start().coordinates(), chord.end().coordinates()) {
        // A normalized boundary can retain an ordinary represented line as a
        // chord. Keep its native offset, measurement, and output capabilities;
        // no selected coordinate or root is materialized by this branch.
        let line = LineSeg2::try_new(start.clone(), end.clone())?;
        let offset = Segment2::Line(line.offset_left(distance.clone())?);
        let source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line));
        return exact_offset_span_from_native_segment(&source, &offset, policy);
    }
    // Selected endpoints keep the common normal-displacement authority;
    // expanding them into unrelated coordinate expressions loses that proof.
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-span",
        "retained-oblique-algebraic-chord",
    );
    let offset_chord = chord.parallel_left_retained(distance.clone(), policy)?;
    let offset_start = offset_chord.start().clone();
    let offset_end = offset_chord.end().clone();
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![retained_chord_fragment(offset_chord)],
        source_end: chord.end().clone(),
        offset_start,
        offset_end,
        start_tangent: Some(CurveTangent2::AlgebraicChord(chord.clone())),
        end_tangent: Some(CurveTangent2::AlgebraicChord(chord.clone())),
    }))
}

pub(super) fn exact_algebraic_cusp_semicircle_endpoint(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match fragment.endpoint_point_evidence(at_start, policy)? {
        Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
        Classification::Decided(None) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(super) fn exact_offset_algebraic_cusp_semicircle_endpoint(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    offset: &crate::BezierAlgebraicCuspSemicircleFragment2,
    source_endpoint: &CurvePoint2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match source.translated_cardinal_offset_endpoint(offset, at_start, source_endpoint, policy)? {
        Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
        Classification::Decided(None) => {
            match source.concentric_offset_endpoint_point_evidence(offset, at_start, policy)? {
                Classification::Decided(Some(point)) => Ok(Classification::Decided(point)),
                Classification::Decided(None) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-selected-circle-endpoint",
                        "general-evaluation",
                    );
                    exact_algebraic_cusp_semicircle_endpoint(offset, at_start, policy)
                }
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-selected-circle-endpoint",
                        "concentric-uncertain",
                    );
                    Ok(Classification::Uncertain(reason))
                }
            }
        }
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-endpoint",
                "cardinal-translation-uncertain",
            );
            Ok(Classification::Uncertain(reason))
        }
    }
}

pub(super) fn selected_circle_endpoint_tangent(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    offset: &crate::BezierAlgebraicCuspSemicircleFragment2,
    at_start: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CurveTangent2>>> {
    match offset.endpoint_chord_tangent_relation(at_start, policy)? {
        Classification::Decided(Some((chord, circle_cross_chord, circle_dot_chord))) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-tangent",
                "selected-circle-chord-contact",
            );
            return Ok(Classification::Decided(Some(CurveTangent2::ChordContact {
                fragment: offset.clone(),
                at_start,
                chord,
                circle_cross_chord,
                circle_dot_chord,
            })));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let tangent = offset
        .represented_endpoint_tangent(at_start, policy)?
        .map(|tangent| {
            Some(tangent.map_or_else(
                || CurveTangent2::SelectedCircularEndpoint {
                    source_fragment: source.clone(),
                    fragment: offset.clone(),
                    at_start,
                },
                CurveTangent2::RepresentedDirection,
            ))
        });
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-tangent",
        match &tangent {
            Classification::Decided(Some(CurveTangent2::SelectedCircularEndpoint { .. })) => {
                "retained-selected-circle-endpoint"
            }
            Classification::Decided(Some(_)) => "represented-selected-circle-endpoint",
            Classification::Decided(None) => {
                unreachable!("mapped tangents always retain a carrier")
            }
            Classification::Uncertain(_) => "uncertain-selected-circle-endpoint",
        },
    );
    Ok(tangent)
}

pub(super) fn exact_offset_span_from_algebraic_cusp_semicircle(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let source_start = match exact_algebraic_cusp_semicircle_endpoint(fragment, true, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "source-start",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_end = match exact_algebraic_cusp_semicircle_endpoint(fragment, false, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "source-end",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_fragment = match fragment.offset_left(distance, policy)? {
        Classification::Decided(Some(fragment)) => fragment,
        Classification::Decided(None) => {
            // Every parameter maps to the selected center at the exact radius
            // collapse.  Retain that point at both span boundaries and emit
            // no degenerate curve; adjacent parallels can then meet there and
            // the authoritative regularizer sees the lower-complexity loop.
            let center = match fragment.semicircle().center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(ExactOffsetSpan2 {
                fragments: Vec::new(),
                source_end,
                offset_start: center.clone(),
                offset_end: center,
                start_tangent: None,
                end_tangent: None,
            }));
        }
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-carrier",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_start = match exact_offset_algebraic_cusp_semicircle_endpoint(
        fragment,
        &offset_fragment,
        &source_start,
        true,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-start",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let offset_end = match exact_offset_algebraic_cusp_semicircle_endpoint(
        fragment,
        &offset_fragment,
        &source_end,
        false,
        policy,
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-blocker",
                "offset-end",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let start_tangent =
        match selected_circle_endpoint_tangent(fragment, &offset_fragment, true, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-selected-circle-blocker",
                    "start-tangent",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
    let end_tangent =
        match selected_circle_endpoint_tangent(fragment, &offset_fragment, false, policy)? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-selected-circle-blocker",
                    "end-tangent",
                );
                return Ok(Classification::Uncertain(reason));
            }
        };
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments: vec![BezierSplitFragment2::AlgebraicCuspSemicircle(
            offset_fragment,
        )],
        source_end,
        offset_start,
        offset_end,
        start_tangent,
        end_tangent,
    }))
}

pub(super) fn analytic_parallel_traversal_start(
    fragment: &crate::BezierParallelFragment2,
) -> &BezierParameter2 {
    if fragment.is_reversed() {
        fragment.range().end()
    } else {
        fragment.range().start()
    }
}

pub(super) fn analytic_parallel_traversal_end(
    fragment: &crate::BezierParallelFragment2,
) -> &BezierParameter2 {
    if fragment.is_reversed() {
        fragment.range().start()
    } else {
        fragment.range().end()
    }
}

#[derive(Clone, Copy)]
pub(super) enum RetainedParallelOffsetFragmentRef2<'a> {
    Analytic(&'a crate::BezierParallelFragment2),
    Selected(&'a crate::bezier_split::BezierSelectedFiberFragment2),
}

impl<'a> RetainedParallelOffsetFragmentRef2<'a> {
    pub(super) fn from_fragment(fragment: &'a BezierSplitFragment2) -> Option<Self> {
        match fragment {
            BezierSplitFragment2::AnalyticParallel(fragment) => Some(Self::Analytic(fragment)),
            BezierSplitFragment2::SelectedFiber(fragment) => Some(Self::Selected(fragment)),
            BezierSplitFragment2::Materialized { .. }
            | BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        }
    }

    pub(super) fn parallel(self) -> BezierParallel2 {
        match self {
            Self::Analytic(fragment) => fragment.parallel().clone(),
            Self::Selected(fragment) => match fragment.source() {
                BezierSelectedFiberSource2::Rational(curve) => BezierParallel2::from_source(
                    BezierParallelSource2::Rational(curve.clone()),
                    Real::zero(),
                ),
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => parallel.clone(),
            },
        }
    }

    pub(super) fn range(self) -> CurveParameterRange2 {
        match self {
            Self::Analytic(fragment) => {
                CurveParameterRange2::from_bezier_range(fragment.range().clone())
            }
            Self::Selected(fragment) => fragment.range().clone(),
        }
    }

    pub(super) fn is_reversed(self) -> bool {
        match self {
            Self::Analytic(fragment) => fragment.is_reversed(),
            Self::Selected(fragment) => fragment.is_reversed(),
        }
    }

    pub(super) fn same_carrier(self, other: Self) -> bool {
        match (self, other) {
            (Self::Analytic(first), Self::Analytic(second)) => {
                first.parallel() == second.parallel()
            }
            (Self::Selected(first), Self::Selected(second)) => first.source() == second.source(),
            (Self::Analytic(first), Self::Selected(second))
            | (Self::Selected(second), Self::Analytic(first)) => match second.source() {
                BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                    first.parallel() == parallel
                }
                BezierSelectedFiberSource2::Rational(curve) => {
                    first.parallel().distance() == &Real::zero()
                        && matches!(
                            first.parallel().source(),
                            BezierParallelSource2::Rational(source) if source == curve
                        )
                }
            },
        }
    }
}

pub(super) fn retained_parallel_represented_parameter(
    parameter: &CurveParameter2,
) -> Option<&Real> {
    parameter.scalar().or_else(|| {
        parameter
            .as_selected_fiber()
            .and_then(|parameter| parameter.represented_value())
    })
}

pub(super) fn retained_parallel_traversal_start(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.end().clone()
    } else {
        range.start().clone()
    }
}

pub(super) fn retained_parallel_traversal_end(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
) -> CurveParameter2 {
    let range = fragment.range();
    if fragment.is_reversed() {
        range.start().clone()
    } else {
        range.end().clone()
    }
}

pub(super) fn exact_retained_parallel_fragment(
    parallel: BezierParallel2,
    range: &CurveParameterRange2,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Option<crate::BezierParallelFragment2>> {
    let promote = |parameter: &CurveParameter2| -> CurveResult<Option<BezierParameter2>> {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return Ok(Some(parameter.clone()));
        }
        let Some(parameter) = parameter.as_selected_fiber() else {
            return Ok(None);
        };
        Ok(match parameter.promoted_bezier_parameter(policy)? {
            Classification::Decided(parameter) => Some(parameter),
            Classification::Uncertain(_) => None,
        })
    };
    let Some(start) = promote(range.start())? else {
        return Ok(None);
    };
    let Some(end) = promote(range.end())? else {
        return Ok(None);
    };
    Ok(Some(crate::BezierParallelFragment2::from_certified_range(
        parallel,
        BezierParameterRange2::new_validated(start, end),
        reversed,
    )))
}

pub(super) fn exact_parallel_region_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return exact_parallel_point_evidence(parallel, parameter, policy);
    }
    let Some(point) =
        crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
            parallel.clone(),
            parameter,
            Real::zero(),
            policy,
        )
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(Classification::Decided(CurvePoint2::from(point)))
}

pub(super) fn exact_parallel_region_endpoint_tangent(
    parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &CurveParameter2,
    scale: RealSign,
    reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveTangent2>> {
    debug_assert_ne!(scale, RealSign::Zero);
    let source_direction = if (scale == RealSign::Positive) != reversed {
        RealSign::Positive
    } else {
        RealSign::Negative
    };
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return exact_parallel_endpoint_tangent(
            parallel,
            source_parallel,
            source_range,
            parameter,
            scale,
            reversed,
        );
    }
    if let Some(selected_source_parameter) = parameter.as_selected_fiber()
        && let Classification::Decided(parameter) =
            selected_source_parameter.promoted_bezier_parameter(policy)?
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-tangent",
            "selected-fiber-retained-parallel",
        );
        return Ok(Classification::Decided(CurveTangent2::RetainedParallel {
            parallel: parallel.clone(),
            source_parallel: source_parallel.clone(),
            source_range: source_range.clone(),
            parameter,
            selected_source_parameter: Some(selected_source_parameter.clone()),
            source_direction,
        }));
    }
    // The tangent direction is shared by every signed parallel of one source,
    // but a round join is centered on the unoffset boundary point. Retaining
    // the composed parallel here would move that center by the offset a
    // second time when the chord-normal fallback constructs its circle.
    Ok(
        crate::BezierAlgebraicChord2::from_certified_retained_parallel_oriented_unit_tangent(
            source_parallel.clone(),
            parameter,
            source_direction,
            policy,
        )?
        .map(CurveTangent2::AlgebraicChord),
    )
}

pub(super) fn promoted_endpoint_image_corner_fragment(
    fragment: &BezierSplitFragment2,
    operation: CurveOperation2,
) -> ExactCurveResult<crate::BezierParallelFragment2> {
    let BezierSplitFragment2::RetainedBezier {
        reversed,
        start,
        end,
        source_curve: source,
        ..
    } = fragment
    else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    // Endpoint-image fragments already carry a validated source interval and
    // first-order endpoint evidence. Re-enter corner editing as that source's
    // exact zero-distance parallel, so every later trim or extension uses the
    // same retained carrier instead of growing another endpoint-field engine.
    let parallel = retained_subcurve_parallel(source, Real::zero())
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    Ok(crate::BezierParallelFragment2::from_certified_range(
        parallel,
        BezierParameterRange2::new_validated(start.clone(), end.clone()),
        *reversed,
    ))
}

/// Promotes an algebraic-endpoint fragment whose complete image is a
/// certified line segment to the shared infinite affine-line carrier.
///
/// The endpoint images remain the finite chord boundaries while the retained
/// source fit supplies the complete support. This is the exact extension
/// authority for line-image endpoint fragments; nonlinear sources continue
/// through their analytic zero-distance parallel instead.
pub(super) fn promoted_endpoint_image_corner_chord(
    fragment: &BezierSplitFragment2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<crate::BezierAlgebraicChord2>> {
    let BezierSplitFragment2::RetainedBezier {
        source_curve: source,
        ..
    } = fragment
    else {
        return Ok(None);
    };
    let line = match subcurve_fit_exact_line_image(source, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit.line().clone(),
        Classification::Decided(BezierLineImageFitRelation::NotLine)
        | Classification::Uncertain(_) => return Ok(None),
    };
    let endpoint = |start_endpoint| -> ExactCurveResult<_> {
        match curve_fragment_endpoint_point(fragment, start_endpoint, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(Some(point)) => Ok(point),
            Classification::Decided(None) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    };
    let support = retained_algebraic_line_support(&line, operation, policy)?;
    match support
        .chord_between_certified_support_points(endpoint(true)?, endpoint(false)?, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(chord)) => Ok(Some(chord)),
        Classification::Decided(None) => Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "an algebraic-endpoint line-image corner collapsed to one point".into(),
            ),
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

/// One admitted retained boundary fragment and the exact carrier evidence
/// shared by region chamfer and fillet solving.
///
/// Admission deliberately precedes algebraic promotion so design-value
/// validation keeps its public ordering (most notably a zero-radius fillet).
/// `prepare` then promotes at most once and owns that evidence through solving
/// and retained publication.
pub(crate) struct CornerCarrierPreparation2<'a> {
    pub(super) top_level: Option<std::borrow::Cow<'a, Curve2>>,
    pub(super) fragment: Option<&'a BezierSplitFragment2>,
    pub(super) evidence: CornerCarrierEvidence2,
    pub(super) source_endpoint_is_end: bool,
    pub(super) source_chart: Option<(&'a Real, &'a Real)>,
}

pub(super) enum CornerCarrierEvidence2 {
    Source,
    Chord(crate::BezierAlgebraicChord2),
    Parallel(crate::BezierParallelFragment2),
    Circular(std::sync::Arc<crate::curve::RetainedRationalCornerArc2>),
}

impl<'a> CornerCarrierPreparation2<'a> {
    pub(super) fn admit(fragment: &'a BezierSplitFragment2) -> Self {
        let top_level = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                Some(std::borrow::Cow::Owned(Curve2::from(curve.clone())))
            }
            BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
            | BezierSplitFragment2::SelectedFiber(_) => None,
        };
        Self {
            top_level,
            fragment: Some(fragment),
            evidence: CornerCarrierEvidence2::Source,
            source_endpoint_is_end: false,
            source_chart: None,
        }
    }

    pub(crate) fn from_curve(curve: &'a Curve2, previous: bool) -> Self {
        if let Some(fragment) = curve.retained_fragment() {
            return Self::admit(fragment);
        }
        Self {
            top_level: Some(std::borrow::Cow::Borrowed(curve)),
            fragment: None,
            evidence: CornerCarrierEvidence2::Source,
            source_endpoint_is_end: previous,
            source_chart: None,
        }
    }

    pub(super) fn family(&self) -> CurveFamily2 {
        self.top_level
            .as_ref()
            .map_or(CurveFamily2::RationalBezier, |curve| curve.family())
    }

    pub(crate) fn prepare(
        &mut self,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        let source = match &self.top_level {
            Some(std::borrow::Cow::Borrowed(curve)) => Some(*curve),
            _ => None,
        };
        if let Some(source) = source
            && let Some(spans) = source.restricted_source_spans(policy, operation)?
        {
            let span = if self.source_endpoint_is_end {
                spans.last()
            } else {
                spans.first()
            }
            .expect("a source restriction contains a span");
            *self = Self::admit(&span.fragment);
            self.source_chart = Some((&span.source_scale, &span.source_offset));
        }
        if let Some(curve) = self.top_level.as_ref() {
            if let Some(crate::curve::ExactCornerCarrier2::RetainedRationalArc(arc)) =
                exact_corner_carrier(curve, self.source_endpoint_is_end, operation, policy)?
            {
                self.evidence = CornerCarrierEvidence2::Circular(arc);
            }
            return Ok(());
        }
        let fragment = self
            .fragment
            .expect("a nonnative corner retains its fragment");
        if let Some(arc) =
            crate::curve::RetainedRationalCornerArc2::from_fragment(fragment, operation, policy)?
        {
            self.evidence = CornerCarrierEvidence2::Circular(arc);
            return Ok(());
        }
        match fragment {
            BezierSplitFragment2::RetainedBezier { .. } => {
                self.evidence = if let Some(chord) =
                    promoted_endpoint_image_corner_chord(fragment, operation, policy)?
                {
                    CornerCarrierEvidence2::Chord(chord)
                } else {
                    CornerCarrierEvidence2::Parallel(promoted_endpoint_image_corner_fragment(
                        fragment, operation,
                    )?)
                };
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                if let Some(parallel) = exact_retained_parallel_fragment(
                    RetainedParallelOffsetFragmentRef2::Selected(fragment).parallel(),
                    fragment.range(),
                    fragment.is_reversed(),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause))?
                {
                    self.evidence = CornerCarrierEvidence2::Parallel(parallel);
                }
            }
            BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {}
            BezierSplitFragment2::Materialized { .. } => {
                return Err(ExactCurveError::blocked(
                    operation,
                    self.family(),
                    UncertaintyReason::Unsupported,
                ));
            }
        }
        Ok(())
    }

    pub(crate) fn exact_carrier(
        &self,
        previous: bool,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::curve::ExactCornerCarrier2<'_>> {
        match &self.evidence {
            CornerCarrierEvidence2::Circular(arc) => {
                return Ok(crate::curve::ExactCornerCarrier2::RetainedRationalArc(
                    std::sync::Arc::clone(arc),
                ));
            }
            CornerCarrierEvidence2::Chord(chord) => {
                return Ok(crate::curve::ExactCornerCarrier2::AlgebraicChord(chord));
            }
            CornerCarrierEvidence2::Parallel(parallel) => {
                return Ok(crate::curve::ExactCornerCarrier2::AnalyticParallel(
                    parallel,
                ));
            }
            CornerCarrierEvidence2::Source => {}
        }
        if let Some(curve) = self.top_level.as_ref() {
            return exact_corner_carrier(curve, previous, operation, policy)?.ok_or_else(|| {
                ExactCurveError::blocked(operation, self.family(), UncertaintyReason::Unsupported)
            });
        }
        let fragment = self
            .fragment
            .expect("a nonnative corner retains its fragment");
        match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => {
                Ok(crate::curve::ExactCornerCarrier2::AlgebraicChord(chord))
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => Ok(
                crate::curve::ExactCornerCarrier2::AnalyticParallel(fragment),
            ),
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                Ok(crate::curve::ExactCornerCarrier2::AlgebraicCusp(fragment))
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                Ok(crate::curve::ExactCornerCarrier2::SelectedFiber(fragment))
            }
            BezierSplitFragment2::RetainedBezier { .. }
            | BezierSplitFragment2::Materialized { .. } => Err(ExactCurveError::blocked(
                operation,
                self.family(),
                UncertaintyReason::Unsupported,
            )),
        }
    }

    pub(crate) fn promoted_parallel(&self) -> Option<&crate::BezierParallelFragment2> {
        match &self.evidence {
            CornerCarrierEvidence2::Parallel(parallel) => Some(parallel),
            _ => None,
        }
    }

    pub(crate) fn source_chart(&self) -> Option<(&Real, &Real)> {
        self.source_chart
    }
}

pub(super) fn retained_parallel_range_scale_sign(
    parallel: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let parameter = match range
        .start()
        .strict_scalar_between_ordered(range.end(), policy)?
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    parallel.parallel_derivative_scale_sign(&parameter.into(), policy)
}

pub(super) fn exact_offset_spans_from_retained_parallel_fragment(
    fragment: RetainedParallelOffsetFragmentRef2<'_>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<ExactOffsetSpan2>>> {
    let parallel = fragment.parallel();
    let range = fragment.range();
    let analysis = match parallel.singularity_analysis(&range, policy)? {
        Classification::Decided(analysis) => analysis,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut ranges = match analysis.regular_subranges(policy)? {
        Classification::Decided(ranges) => ranges,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if fragment.is_reversed() {
        ranges.reverse();
    }
    let mut spans = Vec::with_capacity(ranges.len());
    for (index, range) in ranges.iter().enumerate() {
        let source_end = match fragment {
            RetainedParallelOffsetFragmentRef2::Selected(fragment) if index + 1 == ranges.len() => {
                Some(fragment.end_point())
            }
            _ => None,
        };
        match exact_offset_span_from_regular_parallel_range(
            &parallel,
            range,
            fragment.is_reversed(),
            source_end,
            distance,
            policy,
        )? {
            Classification::Decided(span) => spans.push(span),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(spans))
}

pub(super) fn exact_offset_span_from_regular_parallel_range(
    parallel: &BezierParallel2,
    range: &CurveParameterRange2,
    reversed: bool,
    source_end: Option<&CurvePoint2>,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let source_scale = match retained_parallel_range_scale_sign(parallel, range, policy)? {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let traversal_agrees_with_source = (source_scale == RealSign::Positive) != reversed;
    let composed_distance = if traversal_agrees_with_source {
        parallel.distance() + distance
    } else {
        parallel.distance() - distance
    };
    let composed = parallel.with_distance(composed_distance);
    let composed_distance_sign = match real_sign(composed.distance(), policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };

    // A retained endpoint may be the finite one-sided limit of a stationary
    // source. Its canonical source root supplies the primitive tangent frame;
    // every other cut stays in its original parameter authority.
    let mut source_endpoints = [None, None];
    let analysis = match composed.singularity_analysis(range, policy) {
        Ok(Classification::Decided(analysis)) => Some(analysis),
        // Zero displacement already defines the source without a unit normal.
        // Optional endpoint-frame recovery must not narrow that existing domain.
        _ if composed_distance_sign == RealSign::Zero => None,
        Ok(Classification::Uncertain(reason)) => return Ok(Classification::Uncertain(reason)),
        Err(error) => return Err(error),
    };
    let ranges = if let Some(analysis) = analysis {
        for singularity in analysis.source_singularities() {
            let parameter = CurveParameter2::from(singularity.clone());
            let after_start = match parameter.cmp_by_refinement(range.start(), policy) {
                Ok(Classification::Decided(order)) => order,
                _ if composed_distance_sign == RealSign::Zero => continue,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
            let before_end = match parameter.cmp_by_refinement(range.end(), policy) {
                Ok(Classification::Decided(order)) => order,
                _ if composed_distance_sign == RealSign::Zero => continue,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
            if after_start.is_eq() {
                source_endpoints[0] = Some(singularity.clone());
            } else if before_end.is_eq() {
                source_endpoints[1] = Some(singularity.clone());
            } else if after_start.is_gt()
                && before_end.is_lt()
                && composed_distance_sign != RealSign::Zero
            {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
        match analysis.regular_subranges(policy)? {
            Classification::Decided(ranges) => ranges,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    } else {
        vec![range.clone()]
    };
    let boundaries = ranges
        .iter()
        .map(|range| range.start().clone())
        .chain(std::iter::once(range.end().clone()))
        .collect::<Vec<_>>();
    let (start_index, end_index) = if reversed { (1, 0) } else { (0, 1) };
    let endpoint = |index: usize,
                    scale: RealSign|
     -> CurveResult<Classification<(CurvePoint2, CurveTangent2)>> {
        if let Some(parameter) = &source_endpoints[index] {
            let direction = if reversed {
                match scale {
                    RealSign::Positive => RealSign::Negative,
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => unreachable!(),
                }
            } else {
                scale
            };
            // The point belongs to the new offset, but a join's tangent support
            // remains anchored at the original region corner.
            let limit = composed.regular_source_point_and_tangent_support(
                parallel,
                &parameter.clone().into(),
                range,
                direction,
                policy,
            );
            if composed_distance_sign != RealSign::Zero
                || matches!(&limit, Ok(Classification::Decided(_)))
            {
                return limit.map(|result| {
                    result.map(|(point, tangent)| (point, CurveTangent2::AlgebraicChord(tangent)))
                });
            }
        }
        let parameter = if index == 0 {
            range.start()
        } else {
            range.end()
        };
        let point = match exact_parallel_region_point_evidence(&composed, parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(exact_parallel_region_endpoint_tangent(
            &composed, parallel, range, parameter, scale, reversed, policy,
        )?
        .map(|tangent| (point, tangent)))
    };
    // Join tangents belong to the unoffset corner. On this regular source
    // range the boundary keeps one derivative-scale sign; the composed
    // parallel's sign can differ beside a boundary cusp, where the offset
    // crosses the curvature radius, and must not orient the corner.
    let (offset_start, start_tangent) = match endpoint(start_index, source_scale)? {
        Classification::Decided(endpoint) => endpoint,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (offset_end, end_tangent) = match endpoint(end_index, source_scale)? {
        Classification::Decided(endpoint) => endpoint,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let source_end = if let Some(point) = source_end {
        point.clone()
    } else if source_endpoints[end_index].is_some()
        && let CurveTangent2::AlgebraicChord(tangent) = &end_tangent
    {
        tangent.start().clone()
    } else {
        let parameter = if end_index == 0 {
            range.start()
        } else {
            range.end()
        };
        match exact_parallel_region_point_evidence(parallel, parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    };

    // Share each cut point between its incident fragments. Only cuts that need
    // selected parameters or one-sided source limits retain endpoint payloads.
    let mut points = Vec::new();
    let mut fragments = Vec::with_capacity(ranges.len());
    for (index, range) in ranges.iter().enumerate() {
        let has_source_limit = (index == 0 && source_endpoints[0].is_some())
            || (index + 1 == ranges.len() && source_endpoints[1].is_some());
        if !has_source_limit
            && let Some(fragment) =
                exact_retained_parallel_fragment(composed.clone(), range, reversed, policy)?
        {
            fragments.push(BezierSplitFragment2::AnalyticParallel(fragment));
        } else {
            if points.is_empty() {
                points.resize(boundaries.len(), None);
                points[0] = Some(if reversed {
                    offset_end.clone()
                } else {
                    offset_start.clone()
                });
                points[boundaries.len() - 1] = Some(if reversed {
                    offset_start.clone()
                } else {
                    offset_end.clone()
                });
            }
            for boundary in index..=index + 1 {
                if points[boundary].is_none() {
                    match exact_parallel_region_point_evidence(
                        &composed,
                        &boundaries[boundary],
                        policy,
                    )? {
                        Classification::Decided(point) => points[boundary] = Some(point),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(composed.clone()),
                range.clone(),
                points[index]
                    .as_ref()
                    .expect("the selected start point was retained")
                    .clone(),
                points[index + 1]
                    .as_ref()
                    .expect("the selected end point was retained")
                    .clone(),
            );
            fragments.push(BezierSplitFragment2::SelectedFiber(if reversed {
                selected.reversed()
            } else {
                selected
            }));
        }
    }
    if reversed {
        fragments.reverse();
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-span",
        "retained-region-parameter",
    );
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end,
        offset_start,
        offset_end,
        start_tangent: Some(start_tangent),
        end_tangent: Some(end_tangent),
    }))
}

/// Coalesces one traversal-contiguous retained-parallel run whose only
/// non-represented boundaries are internal arrangement partitions.
///
/// Analytic-parallel and selected-fiber fragments share this authority. A
/// Boolean or self-contact split does not create a geometric corner, so a run
/// with one carrier and traversal is recovered between represented outer
/// endpoints only when its entire interior is certified regular. Cusps and
/// source singularities remain span boundaries, even without a sign change.
pub(super) fn coalesced_retained_parallel_offset_run(
    fragments: &[BezierSplitFragment2],
    first_index: usize,
    maximum_run_length: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(crate::BezierParallelFragment2, usize)>>> {
    if fragments.is_empty() || maximum_run_length == 0 {
        return Ok(Classification::Decided(None));
    }
    let Some(first) = fragments
        .get(first_index % fragments.len())
        .and_then(RetainedParallelOffsetFragmentRef2::from_fragment)
    else {
        return Ok(Classification::Decided(None));
    };
    let parallel = first.parallel();
    if retained_parallel_represented_parameter(&retained_parallel_traversal_start(first)).is_none()
    {
        return Ok(Classification::Decided(None));
    }
    let mut last = first;
    for step in 1..maximum_run_length.min(fragments.len()) {
        let next_index = (first_index + step) % fragments.len();
        let Some(next) = RetainedParallelOffsetFragmentRef2::from_fragment(&fragments[next_index])
        else {
            break;
        };
        if !first.same_carrier(next) || first.is_reversed() != next.is_reversed() {
            break;
        }
        match retained_parallel_traversal_end(last)
            .cmp_by_refinement(&retained_parallel_traversal_start(next), policy)?
        {
            Classification::Decided(std::cmp::Ordering::Equal) => {}
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                break;
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        last = next;
        let traversal_end = retained_parallel_traversal_end(last);
        if retained_parallel_represented_parameter(&traversal_end).is_some() {
            let first_range = first.range();
            let last_range = last.range();
            let (start, end) = if first.is_reversed() {
                (last_range.start(), first_range.end())
            } else {
                (first_range.start(), last_range.end())
            };
            let range = CurveParameterRange2::new_validated(start.clone(), end.clone());
            let analysis = match parallel.singularity_analysis(&range, policy)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
            };
            match analysis.regular_subranges(policy)? {
                Classification::Decided(ranges) if ranges.len() == 1 => {}
                Classification::Decided(_) | Classification::Uncertain(_) => {
                    return Ok(Classification::Decided(None));
                }
            }
            let start = retained_parallel_represented_parameter(start)
                .expect("the coalesced retained-parallel start is represented")
                .clone();
            let end = retained_parallel_represented_parameter(end)
                .expect("the coalesced retained-parallel end is represented")
                .clone();
            return Ok(Classification::Decided(Some((
                crate::BezierParallelFragment2::from_certified_range(
                    parallel,
                    BezierParameterRange2::new_validated(
                        BezierParameter2::Exact(start),
                        BezierParameter2::Exact(end),
                    ),
                    first.is_reversed(),
                ),
                step + 1,
            ))));
        }
    }
    Ok(Classification::Decided(None))
}

/// Coalesces one traversal-contiguous selected-circle run before offsetting.
///
/// Boolean arrangements may split a regular circular carrier at a mapped
/// contact that is not a geometric corner. Keeping that partition through the
/// unary arrangement repeats correlated parameter proofs and constructs two
/// identical concentric carriers. The fragment authority accepts only the
/// same carrier, traversal, and an exactly shared/equal cut; every other case
/// falls back to the ordinary per-fragment path.
pub(super) fn coalesced_algebraic_circle_offset_run(
    fragments: &[BezierSplitFragment2],
    first_index: usize,
    maximum_run_length: usize,
    policy: &CurveContext,
) -> CurveResult<Option<(crate::BezierAlgebraicCuspSemicircleFragment2, usize)>> {
    if fragments.is_empty() || maximum_run_length == 0 {
        return Ok(None);
    }
    let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(first)) =
        fragments.get(first_index % fragments.len())
    else {
        return Ok(None);
    };
    let mut coalesced = first.clone();
    let mut consumed = 1;
    while consumed < maximum_run_length.min(fragments.len()) {
        let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(next)) =
            fragments.get((first_index + consumed) % fragments.len())
        else {
            break;
        };
        let Some(merged) = coalesced.coalesced_with_next(next, policy)? else {
            break;
        };
        coalesced = merged;
        consumed += 1;
    }
    Ok((consumed > 1).then_some((coalesced, consumed)))
}

pub(super) fn exact_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if let Some(parameter) = parameter.scalar() {
        return Ok(parallel.point_at(parameter, policy)?.map(Into::into));
    }
    Ok(Classification::Decided(CurvePoint2::from(
        crate::BezierAnalyticParallelPoint2::new(parallel.clone(), parameter.clone(), policy),
    )))
}

pub(super) fn exact_parallel_endpoint_tangent(
    parallel: &BezierParallel2,
    source_parallel: &BezierParallel2,
    source_range: &CurveParameterRange2,
    parameter: &BezierParameter2,
    scale: RealSign,
    reversed: bool,
) -> CurveResult<Classification<CurveTangent2>> {
    debug_assert_ne!(scale, RealSign::Zero);
    let source_direction = if (scale == RealSign::Positive) != reversed {
        RealSign::Positive
    } else {
        RealSign::Negative
    };
    Ok(Classification::Decided(CurveTangent2::RetainedParallel {
        parallel: parallel.clone(),
        source_parallel: source_parallel.clone(),
        source_range: source_range.clone(),
        parameter: parameter.clone(),
        selected_source_parameter: None,
        source_direction,
    }))
}

pub(super) fn exact_parallel_fragments(
    parallel: &BezierParallel2,
    boundaries: &[BezierParameter2],
    reversed: bool,
) -> Vec<BezierSplitFragment2> {
    let mut fragments = boundaries
        .windows(2)
        .map(|pair| {
            BezierSplitFragment2::AnalyticParallel(
                crate::BezierParallelFragment2::from_certified_range(
                    parallel.clone(),
                    BezierParameterRange2::new_validated(pair[0].clone(), pair[1].clone()),
                    reversed,
                ),
            )
        })
        .collect::<Vec<_>>();
    if reversed {
        fragments.reverse();
    }
    fragments
}

pub(super) fn materialized_offset_fragment(curve: BezierSubcurve2) -> BezierSplitFragment2 {
    BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve,
    }
}

pub(super) fn exact_offset_span_from_native_segment(
    source: &BezierSubcurve2,
    offset: &Segment2,
    policy: &CurveContext,
) -> CurveResult<Classification<ExactOffsetSpan2>> {
    let (fragments, start_tangent, end_tangent) = match offset {
        Segment2::Line(line) => {
            let tangent = line.delta();
            (
                vec![materialized_offset_fragment(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(line.clone()),
                ))],
                tangent.clone(),
                tangent,
            )
        }
        Segment2::Arc(arc) => {
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
            let fragments = decomposition
                .spans()
                .iter()
                .map(|span| {
                    materialized_offset_fragment(BezierSubcurve2::RationalQuadratic(
                        span.curve().clone(),
                    ))
                })
                .collect();
            (
                fragments,
                native_segment_endpoint_tangent(offset, true),
                native_segment_endpoint_tangent(offset, false),
            )
        }
    };
    Ok(Classification::Decided(ExactOffsetSpan2 {
        fragments,
        source_end: source.end().clone().into(),
        offset_start: offset.start().clone().into(),
        offset_end: offset.end().clone().into(),
        start_tangent: Some(CurveTangent2::RepresentedDirection(start_tangent)),
        end_tangent: Some(CurveTangent2::RepresentedDirection(end_tangent)),
    }))
}

pub(super) fn exact_offset_span_from_source_run(
    source_fragments: &[BezierSplitFragment2],
    fragment_index: usize,
    remaining: usize,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<(Vec<ExactOffsetSpan2>, usize)>> {
    let fragment = &source_fragments[fragment_index];
    let mut consumed = 1;
    let offset = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            exact_offset_spans_from_materialized_curve(curve, distance, policy)
        }
        BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_) => {
            match coalesced_retained_parallel_offset_run(
                source_fragments,
                fragment_index,
                remaining,
                policy,
            )? {
                Classification::Decided(Some((coalesced, run_length))) => {
                    consumed = run_length;
                    exact_offset_spans_from_retained_parallel_fragment(
                        RetainedParallelOffsetFragmentRef2::Analytic(&coalesced),
                        distance,
                        policy,
                    )
                }
                Classification::Decided(None) => {
                    exact_offset_spans_from_retained_parallel_fragment(
                        RetainedParallelOffsetFragmentRef2::from_fragment(fragment)
                            .expect("the retained-parallel match arm owns its view"),
                        distance,
                        policy,
                    )
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            exact_offset_span_from_algebraic_chord(chord, distance, policy)
                .map(|span| span.map(|span| vec![span]))
        }
        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
            match coalesced_algebraic_circle_offset_run(
                source_fragments,
                fragment_index,
                remaining,
                policy,
            )? {
                Some((coalesced, run_length)) => {
                    consumed = run_length;
                    exact_offset_span_from_algebraic_cusp_semicircle(&coalesced, distance, policy)
                        .map(|span| span.map(|span| vec![span]))
                }
                None => {
                    exact_offset_span_from_algebraic_cusp_semicircle(fragment, distance, policy)
                        .map(|span| span.map(|span| vec![span]))
                }
            }
        }
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve,
            ..
        } => exact_offset_spans_from_algebraic_endpoint_images(
            *reversed,
            start,
            end,
            source_curve,
            distance,
            policy,
        ),
    }?;
    Ok(offset.map(|span| (span, consumed)))
}

pub(super) fn exact_offset_span_runs_from_boundary_loop(
    boundary_loop: &CurveRegionBoundaryLoop2,
    distance: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<(ExactOffsetSpan2, usize)>>> {
    let source_fragments = boundary_loop.fragments();
    let processing_start = match source_fragments
        .first()
        .and_then(RetainedParallelOffsetFragmentRef2::from_fragment)
    {
        Some(first)
            if retained_parallel_represented_parameter(&retained_parallel_traversal_start(
                first,
            ))
            .is_none() =>
        {
            source_fragments
                .iter()
                .position(|fragment| {
                    RetainedParallelOffsetFragmentRef2::from_fragment(fragment).is_some_and(
                        |candidate| {
                            retained_parallel_represented_parameter(
                                &retained_parallel_traversal_start(candidate),
                            )
                            .is_some()
                        },
                    )
                })
                .unwrap_or(0)
        }
        _ if matches!(
            source_fragments.first(),
            Some(BezierSplitFragment2::AlgebraicCuspSemicircle(first))
                if !first.traversal_start_parameter_is_exact()
        ) =>
        {
            source_fragments
                .iter()
                .position(|fragment| {
                    matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(candidate)
                            if candidate.traversal_start_parameter_is_exact()
                    )
                })
                .unwrap_or(0)
        }
        _ => 0,
    };
    let mut runs = Vec::with_capacity(boundary_loop.len());
    let mut processed = 0;
    while processed < source_fragments.len() {
        let fragment_index = (processing_start + processed) % source_fragments.len();
        let (spans, consumed) = match exact_offset_span_from_source_run(
            source_fragments,
            fragment_index,
            source_fragments.len() - processed,
            distance,
            policy,
        )? {
            Classification::Decided(span) => span,
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                {
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-blocker",
                        "span",
                    );
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-exact-offset-span-blocker",
                        match &source_fragments[fragment_index] {
                            BezierSplitFragment2::Materialized { .. } => "materialized",
                            BezierSplitFragment2::AnalyticParallel(_) => "analytic-parallel",
                            BezierSplitFragment2::SelectedFiber(_) => "selected-fiber",
                            BezierSplitFragment2::AlgebraicChord(_) => "algebraic-chord",
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_) => {
                                "algebraic-cusp-semicircle"
                            }
                            BezierSplitFragment2::RetainedBezier { .. } => {
                                "algebraic-endpoint-images"
                            }
                        },
                    );
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        for (branch_index, span) in spans.into_iter().enumerate() {
            runs.push((span, if branch_index == 0 { consumed } else { 0 }));
        }
        processed += consumed;
    }
    Ok(Classification::Decided(runs))
}

pub(super) fn native_segment_endpoint_tangent(segment: &Segment2, start: bool) -> (Real, Real) {
    match segment {
        Segment2::Line(line) => line.delta(),
        Segment2::Arc(arc) => {
            let point = if start { arc.start() } else { arc.end() };
            let (rx, ry) = point.delta_from(arc.center());
            if arc.is_clockwise() {
                (ry, -rx)
            } else {
                (-ry, rx)
            }
        }
    }
}

pub(super) fn append_exact_offset_join(
    fragments: &mut Vec<BezierSplitFragment2>,
    previous: &ExactOffsetSpan2,
    next: &ExactOffsetSpan2,
    distance: &Real,
    style: &OffsetCornerStyle2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let tangents = previous
        .end_tangent
        .as_ref()
        .zip(next.start_tangent.as_ref());
    let turn_sign = match tangents {
        Some((previous_tangent, next_tangent)) => {
            match curve_tangent_cross_sign(previous_tangent, next_tangent, policy) {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        None => None,
    };
    // A smooth carrier switch owns stronger pair evidence than two separately
    // materialized endpoint images. Consume that tangent/overlap proof first:
    // under APPROXIMATE_512, asking the generic point predicate first could
    // unnecessarily spend the terminal equality policy on an exactly smooth
    // join. Opposite or unresolved parallel tangents retain the point test.
    let mut tangents_opposite = None;
    if turn_sign == Some(RealSign::Zero)
        && let Some((previous_tangent, next_tangent)) = tangents
    {
        match curve_tangents_are_opposite(previous_tangent, next_tangent, policy) {
            Classification::Decided(false) => {
                // Boundary-loop construction already certified a shared
                // source vertex. Equal signed offsets along equal oriented
                // normals therefore share the exact offset vertex even when
                // the images inhabit independent selected fields.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "smooth-source-overlap",
                );
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(true) => {
                tangents_opposite = Some(true);
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "opposite-source-overlap",
                );
            }
            Classification::Uncertain(_) => {}
        }
    }
    if turn_sign.is_none_or(|sign| sign == RealSign::Zero) {
        match previous.offset_end.same_point(&next.offset_start, policy) {
            Classification::Decided(true) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "shared-endpoint",
                );
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-exact-offset-join",
                    "endpoint-equality-uncertain",
                );
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let Some((previous_tangent, next_tangent)) = tangents else {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-join",
            "missing-tangent",
        );
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let turn_sign = turn_sign.expect("retained tangent pair has one exact cross sign");
    let distance_sign = match real_sign(distance, policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let inward = exact_sign_product(turn_sign, distance_sign) == RealSign::Positive;
    #[cfg(feature = "dispatch-trace")]
    if inward {
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-exact-offset-inner-join-tangents",
            match (previous_tangent, next_tangent) {
                (
                    CurveTangent2::RepresentedDirection(_),
                    CurveTangent2::RepresentedDirection(_),
                ) => "vector-vector",
                (CurveTangent2::RepresentedDirection(_), CurveTangent2::ChordContact { .. }) => {
                    "vector-chord-contact"
                }
                (CurveTangent2::ChordContact { .. }, CurveTangent2::RepresentedDirection(_)) => {
                    "chord-contact-vector"
                }
                (
                    CurveTangent2::SelectedCircularEndpoint { .. },
                    CurveTangent2::SelectedCircularEndpoint { .. },
                ) => "selected-circle-selected-circle",
                (
                    CurveTangent2::SelectedCircularEndpoint { .. },
                    CurveTangent2::RepresentedDirection(_),
                ) => "selected-circle-vector",
                (
                    CurveTangent2::RepresentedDirection(_),
                    CurveTangent2::SelectedCircularEndpoint { .. },
                ) => "vector-selected-circle",
                (CurveTangent2::AlgebraicChord(_), CurveTangent2::AlgebraicChord(_)) => {
                    "algebraic-chord-algebraic-chord"
                }
                (CurveTangent2::AlgebraicChord(_), CurveTangent2::RepresentedDirection(_)) => {
                    "algebraic-chord-vector"
                }
                (CurveTangent2::RepresentedDirection(_), CurveTangent2::AlgebraicChord(_)) => {
                    "vector-algebraic-chord"
                }
                _ => "other-retained-pair",
            },
        );
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "curve-region-exact-offset-join",
        match (style, inward) {
            (OffsetCornerStyle2::Round, false) => "round-outer",
            (OffsetCornerStyle2::Bevel, false) => "bevel-outer",
            (OffsetCornerStyle2::Miter { .. }, false) => "miter-outer",
            (OffsetCornerStyle2::Round, true) => "round-inner-miter",
            (OffsetCornerStyle2::Bevel, true) => "bevel-inner-miter",
            (OffsetCornerStyle2::Miter { .. }, true) => "miter-inner",
        },
    );
    match style {
        OffsetCornerStyle2::Round if !inward => {
            let opposite = match tangents_opposite {
                Some(opposite) => opposite,
                None => match curve_tangents_are_opposite(previous_tangent, next_tangent, policy) {
                    Classification::Decided(opposite) => opposite,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            append_exact_round_join(
                fragments,
                previous,
                next,
                distance,
                if opposite {
                    crate::arc_bezier::ArcSweepKind::Semicircle
                } else {
                    crate::arc_bezier::ArcSweepKind::Minor
                },
                policy,
            )
        }
        OffsetCornerStyle2::Bevel if !inward => append_exact_algebraic_line_join(
            fragments,
            &previous.offset_end,
            &next.offset_start,
            None,
            exact_offset_bevel_parameter_axis(
                previous_tangent,
                next_tangent,
                turn_sign,
                distance_sign,
                policy,
            ),
            turn_sign != RealSign::Zero,
            // For a nonzero turn, the difference of the two unit normals
            // cannot be parallel to either endpoint tangent. Thus this bevel
            // is strictly transverse to every selected-circle endpoint it
            // joins, independent of the represented offset distance.
            if turn_sign == RealSign::Zero {
                [false; 2]
            } else {
                [
                    exact_offset_tangent_is_selected_circle(previous_tangent),
                    exact_offset_tangent_is_selected_circle(next_tangent),
                ]
            },
            policy,
        ),
        OffsetCornerStyle2::Miter { limit } if !inward => append_exact_miter_join(
            fragments,
            previous,
            next,
            distance,
            Some(limit),
            turn_sign,
            distance_sign,
            policy,
        ),
        OffsetCornerStyle2::Round
        | OffsetCornerStyle2::Bevel
        | OffsetCornerStyle2::Miter { .. } => append_exact_miter_join(
            fragments,
            previous,
            next,
            distance,
            None,
            turn_sign,
            distance_sign,
            policy,
        ),
    }
}
