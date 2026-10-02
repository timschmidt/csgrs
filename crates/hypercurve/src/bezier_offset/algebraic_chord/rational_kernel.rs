//! Chord/rational-Bezier systems and intersections.

use super::*;

impl BezierAlgebraicChord2 {
    pub(in crate::bezier_offset) fn recursive_projective_rational_system(
        &self,
        source: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveProjectiveChordRationalSystem2>>> {
        // Boolean clipping can replace one finite endpoint with a correlated
        // contact while preserving the exact authored supporting line in the
        // chord's source chain. Incidence depends only on that line, so import
        // its oldest (and therefore smallest-field) endpoints first. The
        // current endpoints remain authoritative for finite clipping through
        // `parameter_at_certified_point`; they need not share an eager
        // primitive field merely to reject the usual interior candidate.
        let (support, support_reversed) = self.smallest_incidence_support();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            eprintln!(
                "recursive chord/rational support finite=[{},{}] support=[{},{}] has-source={}",
                kind(self.start()),
                kind(self.end()),
                kind(support.start()),
                kind(support.end()),
                self.data.source.is_some(),
            );
        }
        let retained_tangent_line = if Arc::ptr_eq(&self.data, &support.data) {
            None
        } else {
            match (support.start(), support.end()) {
                (
                    CurvePoint2(CurvePointData2::AnalyticParallel(start)),
                    CurvePoint2(CurvePointData2::AnalyticParallel(end)),
                ) => match start.recursive_tangent_line_to(end, policy) {
                    Ok(Classification::Decided(Some(line))) => Some(line),
                    Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => {
                        None
                    }
                },
                _ => None,
            }
        };
        let retained_frame = if Arc::ptr_eq(&self.data, &support.data)
            || retained_tangent_line.is_some()
        {
            None
        } else {
            match support.recursive_projective_endpoints_with_direction(policy) {
                Ok(Classification::Decided(Some(frame))) => Some(frame),
                Ok(Classification::Decided(None) | Classification::Uncertain(_)) | Err(_) => None,
            }
        };
        let (
            support_line,
            support_anchor,
            direction_endpoints,
            start,
            end,
            _uses_retained_support_incidence,
        ) = if let Some(line) = retained_tangent_line {
            let line = if support_reversed {
                let negative = Real::from(-1_i8);
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: line.x.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                    y: line.y.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                    denominator: line.denominator.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a retained analytic support line exceeded its field budget".into(),
                        )
                    })?,
                }
            } else {
                line
            };
            (Some(line), None, None, None, None, true)
        } else if let Some(frame) = retained_frame {
            let support_anchor = frame.displaced[usize::from(support_reversed)].clone();
            let direction_endpoints = if support_reversed {
                [
                    frame.direction_endpoints[1].clone(),
                    frame.direction_endpoints[0].clone(),
                ]
            } else {
                frame.direction_endpoints
            };
            (
                None,
                Some(support_anchor),
                Some(direction_endpoints),
                None,
                None,
                true,
            )
        } else {
            let [start, end] = match self.recursive_projective_endpoints(policy)? {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord/rational system stage=endpoints-unavailable");
                    }
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "recursive chord/rational system stage=endpoints reason={reason:?}"
                        );
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                None,
                Some(start.clone()),
                Some([start.clone(), end.clone()]),
                Some(start),
                Some(end),
                false,
            )
        };
        if _uses_retained_support_incidence {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "retained-support-incidence",
            );
        }
        let field = support_line
            .as_ref()
            .map(|line| line.denominator.field())
            .or_else(|| {
                support_anchor
                    .as_ref()
                    .map(|anchor| anchor.denominator.field())
            })
            .expect("a recursive chord support retains a line or anchor");
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "recursive chord/rational system stage=endpoints-complete field-depth={} finite-depth={:?} retained-support-incidence={_uses_retained_support_incidence}",
                field.base_and_extension_path().1.len(),
                start.as_ref().map(|point| point
                    .denominator
                    .field()
                    .base_and_extension_path()
                    .1
                    .len()),
            );
        }
        let source_power = source.homogeneous_power_basis()?;
        let source_weight_sign = match source.denominator_sign(&crate::CurveParameterRange2::unit())
        {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(_) => None,
        };
        let [tangent_power_x, tangent_power_y] =
            rational_parametric_tangent_numerator(source_power);
        let constant_coordinate = self.constant_axis_coordinate(
            match self.data.parameter_axis.axis {
                Axis2::X => Axis2::Y,
                Axis2::Y => Axis2::X,
            },
            policy,
        )?;
        let exact_support_line = self.strict_provenance_support_line(policy);
        let chord_axis_direction_sign = if self.data.parameter_axis.coordinate_increases {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let chord_cross_perpendicular_factor_sign = match self.data.parameter_axis.axis {
            Axis2::X => chord_axis_direction_sign,
            Axis2::Y => product_sign(chord_axis_direction_sign, RealSign::Negative),
        };
        let exact_line_cross_perpendicular_factor_sign = exact_support_line
            .as_ref()
            .and_then(|line| {
                let (line_x, line_y) = line.delta();
                real_sign(
                    match self.data.parameter_axis.axis {
                        Axis2::X => &line_x,
                        Axis2::Y => &line_y,
                    },
                    &policy.strict_counterpart(),
                )
            })
            .filter(|sign| *sign != RealSign::Zero)
            .map(|sign| match self.data.parameter_axis.axis {
                Axis2::X => sign,
                Axis2::Y => product_sign(sign, RealSign::Negative),
            });
        let uses_direct_axis_incidence =
            exact_support_line.is_none() && constant_coordinate.is_some();
        let affine_preimage_incidence_factor_sign = if exact_support_line.is_some() {
            exact_line_cross_perpendicular_factor_sign
        } else if uses_direct_axis_incidence {
            Some(RealSign::Positive)
        } else {
            Some(chord_cross_perpendicular_factor_sign)
        };
        let tangent_from_incidence_derivative_sign = if uses_direct_axis_incidence {
            Some(chord_cross_perpendicular_factor_sign)
        } else {
            Some(RealSign::Positive)
        };
        let Some((source_x, source_y, source_weight, incidence, tangent_cross)) = (|| {
            let real =
                |coefficients: &[Real]| recursive_quadratic_real_polynomial(&field, coefficients);
            let canonical_real = |value: Real| {
                value
                    .exact_rational_normal_form()
                    .map(Real::new)
                    .unwrap_or(value)
            };
            let exact_line_incidence = |line: &LineSeg2,
                                        x: &[Real],
                                        y: &[Real],
                                        weight: &[Real]| {
                let (line_x, line_y) = line.delta();
                let constant =
                    Real::diff_of_products(&line_y, line.start().x(), &line_x, line.start().y());
                let zero = Real::zero();
                (0..x.len().max(y.len()).max(weight.len()))
                    .map(|index| {
                        canonical_real(Real::signed_product_sum(
                            [true, false, true],
                            [
                                [&line_x, y.get(index).unwrap_or(&zero)],
                                [&line_y, x.get(index).unwrap_or(&zero)],
                                [&constant, weight.get(index).unwrap_or(&zero)],
                            ],
                        ))
                    })
                    .collect::<Vec<_>>()
            };
            let exact_line_tangent_cross =
                |line: &LineSeg2, tangent_x: &[Real], tangent_y: &[Real]| {
                    let (line_x, line_y) = line.delta();
                    let zero = Real::zero();
                    (0..tangent_x.len().max(tangent_y.len()))
                        .map(|index| {
                            canonical_real(Real::diff_of_products(
                                &line_x,
                                tangent_y.get(index).unwrap_or(&zero),
                                &line_y,
                                tangent_x.get(index).unwrap_or(&zero),
                            ))
                        })
                        .collect::<Vec<_>>()
                };
            let subtract = |first: &[RecursiveQuadraticValue],
                            second: &[RecursiveQuadraticValue]| {
                recursive_quadratic_polynomial_combine(first, second, true)
            };
            let add = |first: &[RecursiveQuadraticValue], second: &[RecursiveQuadraticValue]| {
                recursive_quadratic_polynomial_combine(first, second, false)
            };
            let scale = |polynomial: &[RecursiveQuadraticValue],
                         value: &RecursiveQuadraticValue| {
                recursive_quadratic_polynomial_scale(polynomial, value)
            };
            let source_x = real(&source_power.x_numerator)?;
            let source_y = real(&source_power.y_numerator)?;
            let source_weight = real(&source_power.weight)?;
            let tangent_x = real(&tangent_power_x)?;
            let tangent_y = real(&tangent_power_y)?;
            let direction = direction_endpoints
                .as_ref()
                .and_then(|endpoints| endpoints[1].difference_numerators(&endpoints[0]));
            let incidence = if let Some(line) = exact_support_line.as_ref() {
                real(&exact_line_incidence(
                    line,
                    &source_power.x_numerator,
                    &source_power.y_numerator,
                    &source_power.weight,
                ))?
            } else if let Some(line) = support_line.as_ref() {
                add(
                    &add(&scale(&source_x, &line.x)?, &scale(&source_y, &line.y)?)?,
                    &scale(&source_weight, &line.denominator)?,
                )?
            } else {
                let (direction_x, direction_y, _) = direction.as_ref()?;
                let support_start = support_anchor.as_ref()?;
                let point_delta_x = subtract(
                    &scale(&source_x, &support_start.denominator)?,
                    &scale(&source_weight, &support_start.x)?,
                )?;
                let point_delta_y = subtract(
                    &scale(&source_y, &support_start.denominator)?,
                    &scale(&source_weight, &support_start.y)?,
                )?;
                match (self.data.parameter_axis.axis, constant_coordinate.as_ref()) {
                    (Axis2::X, Some(constant_y)) => subtract(
                        &source_y,
                        &recursive_quadratic_polynomial_scale_real(&source_weight, constant_y)?,
                    )?,
                    (Axis2::Y, Some(constant_x)) => subtract(
                        &source_x,
                        &recursive_quadratic_polynomial_scale_real(&source_weight, constant_x)?,
                    )?,
                    _ => subtract(
                        &scale(&point_delta_y, direction_x)?,
                        &scale(&point_delta_x, direction_y)?,
                    )?,
                }
            };
            let tangent_cross = if let Some(line) = exact_support_line.as_ref() {
                real(&exact_line_tangent_cross(
                    line,
                    &tangent_power_x,
                    &tangent_power_y,
                ))?
            } else if let Some(line) = support_line.as_ref() {
                add(&scale(&tangent_x, &line.x)?, &scale(&tangent_y, &line.y)?)?
            } else {
                let (direction_x, direction_y, _) = direction.as_ref()?;
                subtract(
                    &scale(&tangent_y, direction_x)?,
                    &scale(&tangent_x, direction_y)?,
                )?
            };
            Some((source_x, source_y, source_weight, incidence, tangent_cross))
        })() else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("recursive chord/rational system stage=polynomial-assembly");
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(Some(
            BezierRecursiveProjectiveChordRationalSystem2 {
                field,
                start,
                end,
                source_x,
                source_y,
                source_weight,
                source_weight_sign,
                incidence,
                tangent_cross,
                affine_preimage_incidence_factor_sign,
                tangent_from_incidence_derivative_sign,
            },
        )))
    }

    pub(in crate::bezier_offset) fn recursive_projective_rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordRationalIntersections2>>> {
        macro_rules! recursive_rational_uncertain {
            ($stage:literal, $reason:expr) => {{
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational blocker stage={} reason={:?} selects-approximate={} permits-approximate={}",
                        $stage,
                        $reason,
                        policy.selects_approximate_512(),
                        policy.permits_approximate_512(),
                    );
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-rational-blocker",
                    $stage,
                );
                return Ok(Classification::Uncertain($reason));
            }};
        }
        let system = match self.recursive_projective_rational_system(source, policy)? {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational blocker stage=system-unavailable selects-approximate={} permits-approximate={}",
                        policy.selects_approximate_512(),
                        policy.permits_approximate_512(),
                    );
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-rational-blocker",
                    "system-unavailable",
                );
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                recursive_rational_uncertain!("system", reason);
            }
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=system-complete incidence-degree={} field-depth={}",
                system.incidence.len().saturating_sub(1),
                system.field.base_and_extension_path().1.len(),
            );
        }
        let unit_domain = range == &CurveParameterRange2::unit();
        let strict_unit_crossing = unit_domain
            .then(|| {
                recursive_quadratic_polynomial_strict_unit_crossing(
                    &system.field,
                    &system.incidence,
                )
            })
            .flatten();
        let mut certified_endpoint_roots = [false; 2];
        let (strict_unit_crossing, geometric_no_roots) = if !unit_domain
            || strict_unit_crossing.is_some()
        {
            (strict_unit_crossing, false)
        } else {
            let geometric_crossing = policy.bounded_exact_predicate_pass(|| -> CurveResult<
                (Option<BezierRecursiveQuadraticUnitCrossing2>, bool),
            > {
                let (predicate_chord, predicate_reversed) = self.smallest_incidence_support();
                let predicate = match BezierAlgebraicChordSupportPredicate2::try_new(
                    predicate_chord,
                    policy,
                )? {
                    Classification::Decided(predicate) => predicate,
                    Classification::Uncertain(_reason) => {
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                            eprintln!(
                                "algebraic chord/rational geometric crossing stage=predicate reason={_reason:?}"
                            );
                        }
                        return Ok((None, false));
                    }
                };
                let point_side = |point: Point2| -> CurveResult<Option<RealSign>> {
                    let point = CurvePoint2::from(point);
                    Ok(match predicate.oriented_side(&point, policy)? {
                        Classification::Decided(crate::classify::LineSide::Left) => {
                            Some(RealSign::Positive)
                        }
                        Classification::Decided(crate::classify::LineSide::Right) => {
                            Some(RealSign::Negative)
                        }
                        Classification::Decided(crate::classify::LineSide::On) => {
                            Some(RealSign::Zero)
                        }
                        Classification::Uncertain(_) => None,
                    })
                };
                let start_side = point_side(source.start().clone())?;
                let end_side = point_side(source.end().clone())?;
                // A finite endpoint on the supporting line is an incidence
                // root regardless of the other endpoint's sign or the signs
                // of the homogeneous controls. Mixed-weight major conics
                // retain this proof even when a convex-hull test is invalid.
                certified_endpoint_roots = [
                    start_side == Some(RealSign::Zero),
                    end_side == Some(RealSign::Zero),
                ];
                let (Some(weight_sign), Some(side_to_incidence_sign)) = (
                    system.source_weight_sign,
                    system.tangent_from_incidence_derivative_sign,
                ) else {
                    return Ok((None, false));
                };
                let incidence_factor = product_sign(
                    product_sign(weight_sign, side_to_incidence_sign),
                    if predicate_reversed {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    },
                );
                let point_sign = |point: Point2| -> CurveResult<Option<RealSign>> {
                    Ok(point_side(point)?.map(|side| product_sign(incidence_factor, side)))
                };
                let start_sign = start_side.map(|side| product_sign(incidence_factor, side));
                let end_sign = end_side.map(|side| product_sign(incidence_factor, side));
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "algebraic chord/rational geometric crossing stage=endpoints start={start_sign:?} end={end_sign:?} weight={weight_sign:?} side-factor={side_to_incidence_sign:?}"
                    );
                }
                let (Some(start_sign), Some(end_sign)) = (start_sign, end_sign) else {
                    return Ok((None, false));
                };
                // Opposite endpoint signs prove one unit root only when the
                // incidence has degree at most two. Higher-degree curves can
                // cross three or more times and need complete root isolation.
                if system.incidence.len() <= 3
                    && strict_signs_are_opposite(Some(start_sign), Some(end_sign))
                {
                    let mut lower = Real::zero();
                    let mut upper = Real::one();
                    let mut lower_sign = start_sign;
                    for _ in 0..12 {
                        let midpoint = ((&lower + &upper) / Real::from(2_i8))?;
                        let Ok(point) = source.point_at(&midpoint, policy) else {
                            break;
                        };
                        let Some(midpoint_sign) = point_sign(point)? else {
                            break;
                        };
                        match midpoint_sign {
                            RealSign::Zero => {
                                lower = midpoint.clone();
                                upper = midpoint;
                                break;
                            }
                            sign if sign == lower_sign => {
                                lower = midpoint;
                                lower_sign = sign;
                            }
                            _ => upper = midpoint,
                        }
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-crossing",
                        "geometric-endpoint-sides",
                    );
                    return Ok((
                        Some(BezierRecursiveQuadraticUnitCrossing2 {
                            start_sign,
                            end_sign,
                            leading_sign: None,
                            lower,
                            upper,
                        }),
                        false,
                    ));
                }
                let uniform_control_sign = match (start_sign, end_sign) {
                    (RealSign::Positive, RealSign::Positive) => RealSign::Positive,
                    (RealSign::Negative, RealSign::Negative) => RealSign::Negative,
                    _ => return Ok((None, false)),
                };
                let mut control_hull_is_disjoint = true;
                let Some(controls) = source.affine_control_points() else { return Ok((None, false)); };
                let interior_control_count = controls.len().saturating_sub(2);
                for (_index, control) in controls.iter()
                    .enumerate()
                    .skip(1)
                    .take(interior_control_count)
                {
                    let sign = point_sign(control.clone())?;
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "algebraic chord/rational geometric crossing stage=control index={_index} sign={sign:?}"
                        );
                    }
                    match sign {
                        Some(sign) if sign == uniform_control_sign || sign == RealSign::Zero => {}
                        _ => {
                            control_hull_is_disjoint = false;
                            break;
                        }
                    }
                }
                // Every interior Bernstein basis function is nonnegative on the
                // closed unit interval, while the endpoint basis functions are
                // strictly positive in its interior. Strict same-sign endpoint
                // coefficients and same-sign-or-zero interior coefficients
                // therefore certify that the incidence has no unit root.
                if control_hull_is_disjoint {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-crossing",
                        "geometric-control-hull-disjoint",
                    );
                    return Ok((None, true));
                }
                Ok((None, false))
            })?;
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=geometric-crossing result={} ",
                    geometric_crossing.0.is_some(),
                );
            }
            geometric_crossing
        };
        if geometric_no_roots {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "geometric-control-hull-disjoint",
            );
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
            )));
        }
        // Opposite certified endpoint signs are already an exact witness that
        // this incidence is not the zero polynomial. Reuse that witness rather
        // than signing every coefficient in a potentially deep retained field.
        let identically_zero = if strict_unit_crossing.is_some() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-collinearity",
                "strict-unit-crossing-nonzero",
            );
            Classification::Decided(false)
        } else {
            recursive_quadratic_polynomial_is_identically_zero(&system.incidence, policy)?
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=collinearity-complete result={identically_zero:?}"
            );
        }
        if matches!(identically_zero, Classification::Decided(true)) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "collinear-before-projection",
            );
            return Ok(self
                .collinear_rational_intersections(source, range, excluded_source_parameter, policy)?
                .map(Some));
        }
        let (candidates, strict_unit_crossing) = match system.parameters(
            range,
            strict_unit_crossing,
            certified_endpoint_roots,
            excluded_source_parameter,
            policy,
        )? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                recursive_rational_uncertain!("parameter-isolation", reason);
            }
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
            eprintln!(
                "algebraic chord/rational stage=parameters-complete candidates={} strict-unit-crossing={}",
                candidates.len(),
                strict_unit_crossing.is_some(),
            );
        }
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("algebraic chord/rational stage=candidate-begin");
            }
            if let Some(excluded) = excluded_source_parameter {
                match candidate.cmp_by_refinement(excluded, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => continue,
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        recursive_rational_uncertain!("excluded-parameter-order", reason);
                    }
                }
            }
            let parameter_order = |endpoint: &BezierRecursiveQuadraticProjectivePoint2| ->
             CurveResult<Option<std::cmp::Ordering>> {
                if let Some(crossing) = strict_unit_crossing.as_ref()
                    && let Some(order) = system.affine_parameter_axis_order(
                        endpoint,
                        self.data.parameter_axis.axis,
                        crossing,
                    )
                {
                    return Ok(Some(order));
                }
                if unit_domain && let Some(order) = system.parameter_hull_axis_order(
                    source,
                    &candidate,
                    endpoint,
                    self.data.parameter_axis.axis,
                )? {
                    return Ok(Some(order));
                }
                Ok(None)
            };
            let start_parameter_order = match system.start.as_ref() {
                Some(start) => parameter_order(start)?,
                None => None,
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=start-parameter-order order={start_parameter_order:?}"
                );
            }
            let end_parameter_order = match system.end.as_ref() {
                Some(end) => parameter_order(end)?,
                None => None,
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=end-parameter-order order={end_parameter_order:?}"
                );
            }
            let chord_location = match (start_parameter_order, end_parameter_order) {
                (Some(start_order), Some(end_order)) => {
                    let (start_order, end_order) = if self.data.parameter_axis.coordinate_increases
                    {
                        (start_order, end_order)
                    } else {
                        (start_order.reverse(), end_order.reverse())
                    };
                    match (start_order, end_order) {
                        (std::cmp::Ordering::Equal, _) => {
                            BezierRecursiveChordContactLocation2::Start
                        }
                        (_, std::cmp::Ordering::Equal) => BezierRecursiveChordContactLocation2::End,
                        (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => {
                            BezierRecursiveChordContactLocation2::Interior
                        }
                        _ => continue,
                    }
                }
                _ => {
                    // The native parameter hull is the hot finite-domain
                    // authority.  Only an overlapping endpoint falls back to
                    // the chord's canonical retained-point classifier; this
                    // avoids adjoining the selected root merely to create a
                    // transient Cartesian target point.
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("algebraic chord/rational stage=finite-point-begin");
                    }
                    let point = match rational_point_evidence_at_region_parameter(
                        source, &candidate, policy,
                    )? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            recursive_rational_uncertain!("finite-point-evidence", reason);
                        }
                    };
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("algebraic chord/rational stage=finite-parameter-begin");
                    }
                    // Strict outward axis separation is the cheapest complete
                    // finite-domain certificate. Overlapping boxes next
                    // consume construction-owned endpoint identity in a
                    // bounded pass; only both declines reach the complete
                    // retained-point comparison.
                    let bounds = policy.bounded_exact_predicate_pass(|| {
                        self.parameter_at_certified_support_point_by_local_evidence(
                            point.clone(),
                            policy,
                        )
                    })?;
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!(
                            "algebraic chord/rational stage=finite-parameter-bounds result={bounds:?}"
                        );
                    }
                    let parameter = match bounds {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => {
                            let bounded = policy.bounded_exact_predicate_pass(|| {
                                self.parameter_at_certified_point(point.clone(), policy)
                            })?;
                            match bounded {
                                Classification::Decided(parameter) => parameter,
                                Classification::Uncertain(_) => {
                                    match self.parameter_at_certified_point(point, policy)? {
                                        Classification::Decided(parameter) => parameter,
                                        Classification::Uncertain(reason) => {
                                            recursive_rational_uncertain!(
                                                "finite-chord-parameter",
                                                reason
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    };
                    let Some(parameter) = parameter else {
                        continue;
                    };
                    match parameter.data {
                        BezierAlgebraicChordParameterStorage2::Endpoint {
                            at_end: false, ..
                        } => BezierRecursiveChordContactLocation2::Start,
                        BezierAlgebraicChordParameterStorage2::Endpoint {
                            at_end: true, ..
                        } => BezierRecursiveChordContactLocation2::End,
                        BezierAlgebraicChordParameterStorage2::Interior(_) => {
                            BezierRecursiveChordContactLocation2::Interior
                        }
                    }
                }
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=location-complete location={chord_location:?}"
                );
            }
            let tangent_cross_sign =
                if let (Some(crossing), Some(weight_sign), Some(incidence_factor_sign)) = (
                    strict_unit_crossing.as_ref(),
                    system.source_weight_sign,
                    system.tangent_from_incidence_derivative_sign,
                ) {
                    // At a simple line-incidence root, differentiating the
                    // homogeneous incidence gives the rational tangent cross
                    // multiplied only by the target weight and positive chord
                    // projective factors. A unique strict unit crossing has
                    // derivative sign equal to its right-end sign.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-rational-tangent",
                        "unit-crossing-derivative",
                    );
                    product_sign(
                        product_sign(crossing.end_sign, weight_sign),
                        incidence_factor_sign,
                    )
                } else {
                    match policy.strict_predicate_pass(|| {
                        recursive_projective_polynomial_sign_at_parameter(
                            &system.field,
                            &system.tangent_cross,
                            &candidate,
                            policy,
                        )
                    })? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            recursive_rational_uncertain!("tangent-cross", reason);
                        }
                    }
                };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!(
                    "algebraic chord/rational stage=tangent-complete sign={tangent_cross_sign:?}"
                );
            }
            let other_parameter = candidate.with_chord_rational_tangent_identity(
                self.clone(),
                source.clone(),
                tangent_cross_sign,
                chord_location,
            );
            let point_evidence = match rational_point_evidence_at_region_parameter(
                source,
                &other_parameter,
                policy,
            )? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    recursive_rational_uncertain!("point-evidence", reason);
                }
            };
            let chord_parameter = match chord_location {
                BezierRecursiveChordContactLocation2::Start => self.start_parameter(),
                BezierRecursiveChordContactLocation2::End => self.end_parameter(),
                BezierRecursiveChordContactLocation2::Interior => {
                    self.parameter_at_certified_interior_point(point_evidence.clone())
                }
            };
            contacts.push(BezierAlgebraicChordRationalContact2 {
                chord_parameter,
                other_parameter,
                point: point_evidence,
                tangent_cross_sign,
            });
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("algebraic chord/rational stage=candidate-complete");
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-rational-kernel",
            "recursive-projective",
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
        )))
    }

    /// Discharges a caller-owned endpoint contact on a pole-free conic span.
    /// The line incidence has a numerator of degree at most two. Its strict
    /// sign immediately inside the owned endpoint and at the opposite endpoint
    /// agrees only if there is no second root. A further simple root would
    /// reverse that sign; a further even root would require degree at least
    /// three. A double root at the owned endpoint itself is permitted.
    ///
    /// The caller must first certify that the whole finite range is pole-free.
    /// This sufficient proof uses local point bounds and the existing tangent
    /// certificate; any failed optional construction leaves full isolation to
    /// the caller.
    pub(in crate::bezier_offset) fn rational_endpoint_contact_is_complete(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        owned: &CurveParameter2,
        policy: &CurveContext,
    ) -> bool {
        policy
            .bounded_exact_predicate_pass(|| -> CurveResult<bool> {
                let power = source.homogeneous_power_basis()?;
                if [&power.x_numerator, &power.y_numerator, &power.weight]
                    .into_iter()
                    .any(|coefficients| {
                        coefficients
                            .iter()
                            .skip(3)
                            .any(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
                    })
                {
                    return Ok(false);
                }
                let other = if owned.cmp_by_refinement(range.start(), policy)?
                    == Classification::Decided(std::cmp::Ordering::Equal)
                {
                    range.end()
                } else if owned.cmp_by_refinement(range.end(), policy)?
                    == Classification::Decided(std::cmp::Ordering::Equal)
                {
                    range.start()
                } else {
                    return Ok(false);
                };
                // Retain the refined authorities themselves: a bounded order
                // query intentionally declines a stored isolator that still
                // touches the other endpoint. Reusing the tighter envelopes
                // also certifies the subsequent interior sample's order.
                let (Classification::Decided(owned), Classification::Decided(other)) = (
                    owned.refined_for_finite_envelope(4, policy)?,
                    other.refined_for_finite_envelope(4, policy)?,
                ) else {
                    return Ok(false);
                };
                let direction = match other.cmp_by_refinement(&owned, policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => RealSign::Negative,
                    Classification::Decided(std::cmp::Ordering::Greater) => RealSign::Positive,
                    _ => return Ok(false),
                };
                let Classification::Decided(point) =
                    rational_point_evidence_at_region_parameter(source, &other, policy)?
                else {
                    return Ok(false);
                };
                let side = match self
                    .strict_oriented_side_by_local_interval_refinement(&point, policy)?
                {
                    Classification::Decided(crate::classify::LineSide::Left) => RealSign::Positive,
                    Classification::Decided(crate::classify::LineSide::Right) => RealSign::Negative,
                    _ => return Ok(false),
                };
                let parallel = source.parallel_left(Real::zero())?;
                let refined_range = CurveParameterRange2::new_validated(other, owned.clone());
                let Classification::Decided(interior) =
                    refined_range.strict_interior_scalar(policy)?
                else {
                    return Ok(false);
                };
                let terminal = CurveParameterRange2::new_validated(
                    BezierParameter2::Exact(interior).into(),
                    owned,
                );
                let Classification::Decided(tangent) =
                    self.parallel_tangent_cross_sign_on_region_range(&parallel, &terminal, policy)?
                else {
                    return Ok(false);
                };
                Ok(tangent != RealSign::Zero && side == product_sign(tangent, direction))
            })
            .unwrap_or(false)
    }

    /// Discovers exact incidence evidence covering the finite source range.
    /// Unit-domain fast paths may retain wider certified components; consumers
    /// clip their contacts and correspondences to the active operand domains.
    /// An excluded parameter names an already-certified contact owned by the caller.
    pub(crate) fn rational_intersections(
        &self,
        source: &RationalBezier2,
        range: &CurveParameterRange2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordRationalIntersections2>> {
        self.validate_policy(policy)?;
        let unit = CurveParameterRange2::unit();
        let unit_covers_range = CurveParameterDomain2::new(&unit, None)
            .contains_finite_range(range, &policy.strict_counterpart())?;
        let finite_unit_source = unit_covers_range == Classification::Decided(true)
            && matches!(
                source.denominator_sign(&crate::CurveParameterRange2::unit()),
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            );
        if !finite_unit_source {
            match polynomial_is_nonzero_on_parameter_range(
                &source.homogeneous_power_basis()?.weight,
                range,
                &policy.strict_counterpart(),
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        if let Some(owned) = excluded_source_parameter
            && self.rational_endpoint_contact_is_complete(source, range, owned, policy)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "conic-owned-endpoint-complete",
            );
            return Ok(Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Contacts(Vec::new()),
            ));
        }
        if !finite_unit_source {
            return Ok(
                match self.recursive_projective_rational_intersections(
                    source,
                    range,
                    excluded_source_parameter,
                    policy,
                )? {
                    Classification::Decided(Some(intersections)) => {
                        Classification::Decided(intersections)
                    }
                    Classification::Decided(None) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        if excluded_source_parameter.is_none()
            && let Some(intersections) = self.exact_linear_rational_intersections(source, policy)?
        {
            return Ok(Classification::Decided(intersections));
        }
        // A represented parameter can still contain arbitrary exact values.
        // Reuse its certified incidence as a polynomial factor before circle
        // reconstruction asks a freshly solved point to prove endpoint equality.
        if excluded_source_parameter
            .and_then(CurveParameter2::scalar)
            .is_some()
            && let Classification::Decided(Some(intersections)) =
                policy.strict_predicate_pass(|| {
                    self.recursive_projective_rational_intersections(
                        source,
                        range,
                        excluded_source_parameter,
                        policy,
                    )
                })?
        {
            return Ok(Classification::Decided(intersections));
        }
        // Diagonal fiber deflation needs a univariate algebraic root. Other
        // retained locations still participate in general contact ownership.
        let selected =
            match excluded_source_parameter.and_then(CurveParameter2::as_bezier_parameter) {
                Some(BezierParameter2::Algebraic(parameter)) => Some(parameter.clone()),
                None if excluded_source_parameter.is_none() => {
                    match self.algebraic_endpoint_parameter(policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => None,
                    }
                }
                Some(BezierParameter2::Exact(_)) | None => None,
            };
        if let Some(parameter) = selected {
            match self.source_related_intersections(
                source,
                &parameter,
                excluded_source_parameter.is_none(),
                policy,
            )? {
                Classification::Decided(
                    BezierAlgebraicChordRationalIntersections2::NotSourceRelated
                    | BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(UncertaintyReason::Unsupported) => {}
                intersections => return Ok(intersections),
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record("hypercurve", "algebraic-chord-pair", "general-rational");
        if let Some(line) = self
            .exact_line()
            .or_else(|| self.strict_provenance_support_line(policy))
        {
            if let Some(intersections) = self.exact_line_retained_circle_intersections(
                &line,
                source,
                excluded_source_parameter,
                policy,
            )? {
                return Ok(intersections);
            }
            let line_relation = source.relation_to_line_with_contacts(&line, policy);
            let line_contacts = match line_relation {
                Classification::Decided(
                    BezierLineContactRelation::ControlHullDisjoint { .. }
                    | BezierLineContactRelation::NoContact,
                ) => Vec::new(),
                Classification::Decided(BezierLineContactRelation::OnSupportingLine) => {
                    return self.collinear_rational_intersections(
                        source,
                        &unit,
                        excluded_source_parameter,
                        policy,
                    );
                }
                Classification::Decided(BezierLineContactRelation::Contacts { contacts }) => {
                    contacts
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut contacts = Vec::with_capacity(line_contacts.len());
            for contact in line_contacts {
                let source_parameter = contact.parameter().clone();
                if let Some(excluded) = excluded_source_parameter {
                    match excluded.cmp_by_refinement(&source_parameter.clone().into(), policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let point = match rational_point_evidence_at_parameter(
                    source,
                    &source_parameter,
                    policy,
                )? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let chord_parameter =
                    match self.parameter_at_certified_point(point.clone(), policy)? {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let tangent_cross_sign = match (contact.kind(), contact.crossing_direction()) {
                    (
                        BezierLineContactKind::Crossing,
                        Some(BezierLineCrossingDirection::NegativeToPositive),
                    ) => RealSign::Positive,
                    (
                        BezierLineContactKind::Crossing,
                        Some(BezierLineCrossingDirection::PositiveToNegative),
                    ) => RealSign::Negative,
                    (BezierLineContactKind::Tangent, None) => RealSign::Zero,
                    (BezierLineContactKind::Crossing, None)
                    | (BezierLineContactKind::Tangent, Some(_)) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                };
                contacts.push(BezierAlgebraicChordRationalContact2 {
                    chord_parameter,
                    other_parameter: CurveParameter2::from(source_parameter),
                    point,
                    tangent_cross_sign,
                });
            }
            return Ok(Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
            ));
        }
        match self.recursive_projective_rational_intersections(
            source,
            &CurveParameterRange2::unit(),
            excluded_source_parameter,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                Ok(Classification::Decided(intersections))
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Reuses finite chord replay when the rational source has the identical
    /// normalized affine parameter. All endpoint contacts remain present;
    /// adjacency belongs to the region consumer.
    pub(crate) fn exact_linear_rational_intersections(
        &self,
        source: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierAlgebraicChordRationalIntersections2>> {
        let Some(line) = source.exact_linear_parameterization_line() else {
            return Ok(None);
        };
        let chord = match Self::try_new(
            line.start().clone().into(),
            line.end().clone().into(),
            policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(_) => return Ok(None),
        };
        if policy
            .strict_predicate_pass(|| self.is_strictly_one_sided_of_exact_line(&line, policy))?
            == Classification::Decided(true)
        {
            return Ok(Some(BezierAlgebraicChordRationalIntersections2::Contacts(
                Vec::new(),
            )));
        }
        let intersections = match self.chord_intersections(&chord, policy)? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(_) => return Ok(None),
        };
        Ok(Some(match intersections {
            BezierAlgebraicChordPairIntersections2::Contacts(contacts) => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let parameter = match contact
                        .second_parameter
                        .exact_line_curve_parameter(policy)?
                    {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    let parameter = if let Some(parameter) = parameter.as_recursive_projective() {
                        // This contact owns the same tangent and finite-domain
                        // evidence as general rational replay. Keep it with the
                        // selected scalar so later corner operations can reuse
                        // the proof. A pre-existing specialized identity needs
                        // its complete kernel; never overwrite that evidence.
                        if parameter.data.identity.is_some() {
                            return Ok(None);
                        }
                        let location = if contact.first_parameter.is_endpoint_of(self, true) {
                            BezierRecursiveChordContactLocation2::Start
                        } else if contact.first_parameter.is_endpoint_of(self, false) {
                            BezierRecursiveChordContactLocation2::End
                        } else {
                            BezierRecursiveChordContactLocation2::Interior
                        };
                        CurveParameter2::from_recursive_projective(
                            parameter.clone().with_chord_rational_tangent_identity(
                                self.clone(),
                                source.clone(),
                                contact.tangent_cross_sign,
                                location,
                            ),
                        )
                    } else {
                        parameter
                    };
                    retained.push(BezierAlgebraicChordRationalContact2 {
                        chord_parameter: contact.first_parameter,
                        other_parameter: parameter,
                        point: contact.point,
                        tangent_cross_sign: contact.tangent_cross_sign,
                    });
                }
                BezierAlgebraicChordRationalIntersections2::Contacts(retained)
            }
            BezierAlgebraicChordPairIntersections2::Overlaps(overlaps) => {
                let mut retained = Vec::with_capacity(overlaps.len());
                for overlap in overlaps {
                    let mut parameters = [None, None];
                    for (index, parameter) in overlap.second_range.iter().enumerate() {
                        match parameter.exact_line_curve_parameter(policy)? {
                            Classification::Decided(parameter) => {
                                parameters[index] = Some(parameter)
                            }
                            Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                    let [start, end] = parameters.map(|p| p.expect("two overlap boundaries"));
                    retained.push(BezierAlgebraicChordRationalOverlap2 {
                        chord: self.clone(),
                        source: source.clone(),
                        chord_range: overlap.first_range,
                        source_range: CurveParameterRange2::new_validated(start, end),
                        orientation: overlap.orientation,
                    });
                }
                BezierAlgebraicChordRationalIntersections2::Overlaps(retained)
            }
        }))
    }
}
