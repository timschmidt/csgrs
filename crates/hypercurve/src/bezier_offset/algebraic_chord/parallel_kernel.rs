//! Chord/parallel tangent signs, systems and intersections.

use super::*;

impl BezierAlgebraicChord2 {
    /// Certifies that the oriented support incidence is strictly monotone on
    /// one retained analytic-parallel range.
    ///
    /// A boundary chord adjacent to that range already owns one common
    /// endpoint.  If `cross(chord_direction, parallel_tangent)` has one
    /// nonzero sign everywhere, the signed support incidence has at most one
    /// zero, so the authored endpoint is the complete finite contact set.
    /// Endpoint and tangent boxes are only sufficient certificates: an
    /// unresolved enclosure falls through to the general incidence kernel.
    pub(crate) fn parallel_tangent_cross_sign_on_region_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        policy.strict_predicate_pass(|| {
            let strict = policy;
            // Splits, reversals, and exact normal/tangent displacements preserve
            // one authored direction up to sign.  Refine that least-field
            // authority instead of the displaced finite endpoints: the latter
            // add normalization radicals which cancel completely from this
            // tangent-cross predicate.
            let (direction_authority, direction_reversed) = self.tangent_authority();
            #[cfg(feature = "dispatch-trace")]
            if !Arc::ptr_eq(&self.data, &direction_authority.data) {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "retained-tangent-authority",
                );
            }
            let range_order = match range.start().cmp_by_refinement(range.end(), strict)? {
                Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
                Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::InvalidBezierRange);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let certified_tangent_endpoint =
                |parameter: &CurveParameter2| -> CurveResult<bool> {
                    for contact in self
                        .parallel_tangent_contacts()
                        .iter()
                        .chain(direction_authority.parallel_tangent_contacts())
                    {
                        if contact.parallel() != parallel {
                            continue;
                        }
                        let retained = CurveParameter2::from(BezierParameter2::Exact(
                            contact.parameter().clone(),
                        ));
                        if parameter.cmp_by_refinement(&retained, strict)?
                            == Classification::Decided(std::cmp::Ordering::Equal)
                        {
                            return Ok(true);
                        }
                    }
                    Ok(false)
                };
            let retained_identity_tangent_sign =
                |parameter: &CurveParameter2| -> CurveResult<Option<RealSign>> {
                    let Some(parameter) = parameter.as_recursive_projective() else {
                        return Ok(None);
                    };
                    let BezierParallelSource2::Rational(source) = parallel.source() else {
                        return Ok(None);
                    };
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        match parameter.data.identity.as_deref() {
                            None => eprintln!("retained tangent identity: none"),
                            Some(BezierRecursiveProjectiveParameterIdentity2::Line(_)) => {
                                eprintln!("retained tangent identity: line")
                            }
                            Some(
                                BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(
                                    identity,
                                ),
                            ) => {
                                let kind = |point: &CurvePoint2| {
                                    match point {
                                        CurvePoint2(CurvePointData2::Exact(_)) => {
                                            "exact"
                                        }
                                        CurvePoint2(CurvePointData2::Algebraic(_)) => {
                                            "algebraic"
                                        }
                                        CurvePoint2(CurvePointData2::AlgebraicChordPair(
                                            _,
                                        )) => "pair",
                                        CurvePoint2(CurvePointData2::AlgebraicCuspChord(
                                            _,
                                        )) => "cusp",
                                        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(
                                            _,
                                        )) => "derived",
                                        CurvePoint2(CurvePointData2::AlgebraicChordParallel(
                                            _,
                                        )) => "parallel",
                                        CurvePoint2(CurvePointData2::AnalyticParallel(
                                            _,
                                        )) => "analytic",
                                        CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                                            "similarity"
                                        }
                                    }
                                };
                                let support = direction_authority.retained_support();
                                let procedural = chord_parallel_support_source(
                                    direction_authority,
                                    strict,
                                )
                                .ok()
                                .flatten();
                                eprintln!(
                                    "retained tangent identity: chord source_match={} orientation={:?} current=({},{}) support=({},{}) procedural={:?} procedural_source=({},{}) source_orientation={:?} identity=({},{})",
                                    source == &identity.source,
                                    identity.chord.shared_tangent_orientation(direction_authority),
                                    kind(direction_authority.start()),
                                    kind(direction_authority.end()),
                                    kind(support.start()),
                                    kind(support.end()),
                                    procedural.as_ref().map(|value| value.direction),
                                    procedural
                                        .as_ref()
                                        .map_or("n/a", |value| kind(value.source.start())),
                                    procedural
                                        .as_ref()
                                        .map_or("n/a", |value| kind(value.source.end())),
                                    procedural.as_ref().and_then(|value| identity
                                        .chord
                                        .shared_tangent_orientation(&value.source)),
                                    kind(identity.chord.start()),
                                    kind(identity.chord.end()),
                                );
                                if let (
                                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                                ) = (direction_authority.start(), direction_authority.end())
                                {
                                    eprintln!(
                                        "retained tangent parallel endpoints carrier={} at_end=[{},{}] direction=[{:?},{:?}] source-points=[{},{}] sources-identity=[{},{}] source-orientation={:?} distance-equal={} translation-equal={} point-storage={:?} point-equality={:?}",
                                        start.shares_carrier(end),
                                        start.at_end,
                                        end.at_end,
                                        start.data.direction,
                                        end.data.direction,
                                        kind(start.source_endpoint()),
                                        kind(end.source_endpoint()),
                                        start.data.source.shares_retained_support(&identity.chord),
                                        end.data.source.shares_retained_support(&identity.chord),
                                        start.data.source.shared_tangent_orientation(&end.data.source),
                                        start.data.distance == end.data.distance,
                                        start.data.translation_x == end.data.translation_x
                                            && start.data.translation_y == end.data.translation_y,
                                        [identity.chord.start(), identity.chord.end()].map(|endpoint| {
                                            [start.source_endpoint(), end.source_endpoint()]
                                                .map(|point| point.shares_storage(endpoint))
                                        }),
                                        [identity.chord.start(), identity.chord.end()].map(|endpoint| {
                                            [start.source_endpoint(), end.source_endpoint()]
                                                .map(|point| point == endpoint)
                                        }),
                                    );
                                }
                            }
                        }
                    }
                    let Some(sign) = parameter.chord_rational_tangent_cross_sign(
                        direction_authority,
                        source,
                        RealSign::Positive,
                        strict,
                    ) else {
                        return Ok(None);
                    };
                    Ok(match sign? {
                        Classification::Decided(sign) => Some(sign),
                        Classification::Uncertain(_) => None,
                    })
                };
            let certified_source_sign = |parameter: &CurveParameter2,
                                         tangent: bool|
             -> CurveResult<Option<RealSign>> {
                let identity = retained_identity_tangent_sign(parameter)?;
                if tangent && identity.is_some_and(|sign| sign != RealSign::Zero) {
                    return Err(CurveError::Topology(
                        "retained tangent and transverse endpoint certificates conflicted".into(),
                    ));
                }
                Ok(identity.or(tangent.then_some(RealSign::Zero)))
            };
            let singularities = match parallel.singularity_analysis(range, strict)? {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for parameter in singularities
                .source_singularities()
                .iter()
                .chain(singularities.parallel_cusps())
            {
                let parameter = CurveParameter2::from(parameter.clone());
                let start_order = match parameter.cmp_by_refinement(range.start(), strict)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let end_order = match parameter.cmp_by_refinement(range.end(), strict)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let lies_strictly_inside = match range_order {
                    std::cmp::Ordering::Less => {
                        start_order == std::cmp::Ordering::Greater
                            && end_order == std::cmp::Ordering::Less
                    }
                    std::cmp::Ordering::Greater => {
                        start_order == std::cmp::Ordering::Less
                            && end_order == std::cmp::Ordering::Greater
                    }
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
                if lies_strictly_inside {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-parallel-monotonicity",
                        "interior-singularity",
                    );
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            }
            let interior = match range.strict_interior_scalar(strict)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let derivative_scale = match parallel
                .parallel_derivative_scale_sign_at_exact(&interior, strict)?
            {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let regularized = match parallel
                .source_oriented_regularized_tangent_field_at_interior(&interior, strict)?
            {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let authored_tangent_at_range_start = direction_authority
                .is_authored_source_tangent_at_region_parameter(
                    parallel,
                    range.start(),
                    regularized.as_deref(),
                    strict,
                );
            let authored_tangent_at_range_end = direction_authority
                .is_authored_source_tangent_at_region_parameter(
                    parallel,
                    range.end(),
                    regularized.as_deref(),
                    strict,
                );
            #[cfg(feature = "dispatch-trace")]
            if authored_tangent_at_range_start || authored_tangent_at_range_end {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "authored-source-tangent-endpoint",
                );
            }
            let certified_tangent_at_range_start = certified_tangent_endpoint(range.start())?
                || authored_tangent_at_range_start;
            let certified_tangent_at_range_end = certified_tangent_endpoint(range.end())?
                || authored_tangent_at_range_end;
            // A join chord whose endpoints are independently displaced from
            // authored source points has a direction involving two distinct
            // unit-normal fields.  Endpoint-box refinement is only an
            // optional monotonicity certificate here; merging those complete
            // point fields can dwarf the authoritative chord/parallel
            // incidence kernel that follows this declined fast path.
            let endpoint_specific_displacement_direction = matches!(
                (direction_authority.start(), direction_authority.end()),
                (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
                ) if start.data.source_point.is_some() || end.data.source_point.is_some()
            );
            if direction_authority.certified_unit_tangent().is_none()
                && endpoint_specific_displacement_direction
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "defer-composite-join-direction",
                );
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let source_sign_at_range_start =
                certified_source_sign(range.start(), certified_tangent_at_range_start)?;
            let source_sign_at_range_end =
                certified_source_sign(range.end(), certified_tangent_at_range_end)?;
            let (source_sign_at_lower, source_sign_at_upper) = match range_order {
                std::cmp::Ordering::Less => {
                    (source_sign_at_range_start, source_sign_at_range_end)
                }
                std::cmp::Ordering::Greater => {
                    (source_sign_at_range_end, source_sign_at_range_start)
                }
                std::cmp::Ordering::Equal => unreachable!("validated above"),
            };
            let differential;
            let (tangent_x_coefficients, tangent_y_coefficients) = match regularized.as_deref() {
                Some(field) => (&field.x[..], &field.y[..]),
                None => {
                    differential = parallel.differential()?;
                    (&differential.tangent_x[..], &differential.tangent_y[..])
                }
            };
            let certified_direction_values = direction_authority.certified_unit_tangent();
            let certified_direction = certified_direction_values.as_ref().map(|(x, y)| {
                (
                    RealInterval::from_values(
                        [x.clone()],
                    )
                    .expect("one certified tangent coordinate defines an exact interval"),
                    RealInterval::from_values(
                        [y.clone()],
                    )
                    .expect("one certified tangent coordinate defines an exact interval"),
                )
            });
            for (steps, precision) in [
                (0, -32),
                (2, -64),
                (4, -96),
                (8, -128),
                (16, -192),
                (32, -256),
                (64, -384),
                (128, -512),
                (256, -768),
                (512, -1024),
            ] {
                let endpoint_direction = if certified_direction.is_none() {
                    let (
                        Classification::Decided(start_bounds),
                        Classification::Decided(end_bounds),
                    ) = (
                        algebraic_chord_endpoint_bounds_refined(
                            direction_authority.start(),
                            steps,
                            policy,
                        ),
                        algebraic_chord_endpoint_bounds_refined(
                            direction_authority.end(),
                            steps,
                            policy,
                        ),
                    )
                    else {
                        continue;
                    };
                    Some((
                        real_interval_from_axis(&end_bounds, Axis2::X)
                            .subtract(&real_interval_from_axis(
                                &start_bounds,
                                Axis2::X,
                            )),
                        real_interval_from_axis(&end_bounds, Axis2::Y)
                            .subtract(&real_interval_from_axis(
                                &start_bounds,
                                Axis2::Y,
                            )),
                    ))
                } else {
                    None
                };
                let refined_start =
                    match range.start().refined_for_finite_envelope(steps, strict)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => continue,
                    };
                let refined_end = match range.end().refined_for_finite_envelope(steps, strict)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => continue,
                };
                let (Some(start_bounds), Some(end_bounds)) = (
                    refined_start.finite_envelope_bounds(),
                    refined_end.finite_envelope_bounds(),
                ) else {
                    continue;
                };
                let (parameter_lower, parameter_upper) = match range_order {
                    std::cmp::Ordering::Less => (start_bounds.0, end_bounds.1),
                    std::cmp::Ordering::Greater => (end_bounds.0, start_bounds.1),
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
                let Some([tangent_x_lower, tangent_x_upper]) =
                    coefficients_value_interval_on_real_interval(
                        tangent_x_coefficients,
                        parameter_lower,
                        parameter_upper,
                        precision,
                    )?
                else {
                    continue;
                };
                let Some([tangent_y_lower, tangent_y_upper]) =
                    coefficients_value_interval_on_real_interval(
                        tangent_y_coefficients,
                        parameter_lower,
                        parameter_upper,
                        precision,
                    )?
                else {
                    continue;
                };
                let (direction_x, direction_y) = certified_direction
                    .as_ref()
                    .or(endpoint_direction.as_ref())
                    .expect("a retained chord supplies either a certified or endpoint direction");
                let tangent_x = RealInterval {
                    lower: Real::new(tangent_x_lower),
                    upper: Real::new(tangent_x_upper),
                };
                let tangent_y = RealInterval {
                    lower: Real::new(tangent_y_lower),
                    upper: Real::new(tangent_y_upper),
                };
                let Some(cross) = direction_x.multiply(&tangent_y).and_then(|first| {
                    direction_y
                        .multiply(&tangent_x)
                        .map(|second| first.subtract(&second))
                }) else {
                    continue;
                };
                let zero = Real::zero();
                let source_sign = if compare_reals(&cross.lower, &zero, strict)
                    == Some(std::cmp::Ordering::Greater)
                {
                    Some(RealSign::Positive)
                } else if compare_reals(&cross.upper, &zero, strict)
                    == Some(std::cmp::Ordering::Less)
                {
                    Some(RealSign::Negative)
                } else {
                    None
                };
                if let Some(source_sign) = source_sign {
                    let sign = product_sign(source_sign, derivative_scale);
                    return Ok(Classification::Decided(if direction_reversed {
                        product_sign(sign, RealSign::Negative)
                    } else {
                        sign
                    }));
                }

                // Direct range evaluation loses the endpoint correlation and
                // can contain zero forever when the tangent cross vanishes at
                // a closed endpoint.  Restrict the (at most quadratic in the
                // hot path) cross polynomial to this exact parameter box and
                // inspect its Bernstein controls instead.  Nonnegative
                // controls with one certified positive control (or the
                // negative counterpart) prove a strict sign throughout the
                // open retained range without isolating any recursive-field
                // roots.
                let degree = tangent_x_coefficients
                    .len()
                    .max(tangent_y_coefficients.len())
                    .saturating_sub(1);
                if degree <= 2 {
                    let exact_interval = |value: Real| RealInterval {
                        lower: value.clone(),
                        upper: value,
                    };
                    let cross_coefficient = |power: usize| {
                        let tangent_x = tangent_x_coefficients
                            .get(power)
                            .cloned()
                            .unwrap_or_else(Real::zero);
                        let tangent_y = tangent_y_coefficients
                            .get(power)
                            .cloned()
                            .unwrap_or_else(Real::zero);
                        direction_x
                            .multiply(&exact_interval(tangent_y))
                            .and_then(|first| {
                                direction_y
                                    .multiply(&exact_interval(tangent_x))
                                    .map(|second| first.subtract(&second))
                            })
                    };
                    let coefficients = [
                        cross_coefficient(0),
                        cross_coefficient(1),
                        cross_coefficient(2),
                    ];
                    if let [Some(c0), Some(c1), Some(c2)] = coefficients {
                        let (start_lower, start_upper, end_lower, end_upper) = match range_order {
                            std::cmp::Ordering::Less => {
                                (start_bounds.0, start_bounds.1, end_bounds.0, end_bounds.1)
                            }
                            std::cmp::Ordering::Greater => {
                                (end_bounds.0, end_bounds.1, start_bounds.0, start_bounds.1)
                            }
                            std::cmp::Ordering::Equal => unreachable!("validated above"),
                        };
                        let start = RealInterval {
                            lower: start_lower.clone(),
                            upper: start_upper.clone(),
                        };
                        let end = RealInterval {
                            lower: end_lower.clone(),
                            upper: end_upper.clone(),
                        };
                        let delta = end.subtract(&start);
                        let evaluate = |parameter: &RealInterval| {
                            c2.multiply(parameter)
                                .map(|value| value.add(&c1))?
                                .multiply(parameter)
                                .map(|value| value.add(&c0))
                        };
                        let controls = (|| {
                            let q0 = evaluate(&start)?;
                            let derivative = c2
                                .multiply(&start)?
                                .multiply(&exact_interval(Real::from(2_i8)))?
                                .add(&c1);
                            let q1 = delta.multiply(&derivative)?;
                            let first = q0.clone();
                            let middle = q0.add(&q1.multiply(
                                &exact_interval(
                                    (Real::one() / Real::from(2_i8)).expect("two is nonzero"),
                                ),
                            )?);
                            // Endpoint evaluation avoids the dependency loss
                            // from expanding (end - start) a second time.
                            let last = evaluate(&end)?;
                            Some([first, middle, last])
                        })();
                        if let Some(controls) = controls {
                            let zero = Real::zero();
                            let mut control_signs = controls.each_ref().map(|control| {
                                (
                                    compare_reals(&control.lower, &zero, strict),
                                    compare_reals(&control.upper, &zero, strict),
                                )
                            });
                            let sign_orders = |sign| {
                                let order = match sign {
                                    RealSign::Negative => std::cmp::Ordering::Less,
                                    RealSign::Zero => std::cmp::Ordering::Equal,
                                    RealSign::Positive => std::cmp::Ordering::Greater,
                                };
                                (Some(order), Some(order))
                            };
                            if let Some(sign) = source_sign_at_lower {
                                control_signs[0] = sign_orders(sign);
                            }
                            if let Some(sign) = source_sign_at_upper {
                                control_signs[2] = sign_orders(sign);
                            }
                            let strict_endpoint_sign = |(lower, upper)| {
                                if lower == Some(std::cmp::Ordering::Greater) {
                                    Some(RealSign::Positive)
                                } else if upper == Some(std::cmp::Ordering::Less) {
                                    Some(RealSign::Negative)
                                } else {
                                    None
                                }
                            };
                            if strict_signs_are_opposite(
                                strict_endpoint_sign(control_signs[0]),
                                strict_endpoint_sign(control_signs[2]),
                            ) {
                                // These controls enclose the actual endpoint
                                // values. Opposite signs prove an interior zero,
                                // so complete point-field projection cannot
                                // make this optional monotonicity proof succeed.
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-parallel-monotonicity",
                                    "opposed-endpoint-signs",
                                );
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                            #[cfg(test)]
                            if steps == 512
                                && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
                            {
                                eprintln!(
                                    "retained range Bernstein controls={control_signs:?} endpoints=[{source_sign_at_lower:?},{source_sign_at_upper:?}]",
                                );
                            }
                            let nonnegative = control_signs.iter().all(|(lower, _)| {
                                matches!(
                                    lower,
                                    Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
                                )
                            });
                            let nonpositive = control_signs.iter().all(|(_, upper)| {
                                matches!(
                                    upper,
                                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                                )
                            });
                            let positive = control_signs.iter().any(|(lower, _)| {
                                *lower == Some(std::cmp::Ordering::Greater)
                            });
                            let negative = control_signs.iter().any(|(_, upper)| {
                                *upper == Some(std::cmp::Ordering::Less)
                            });
                            let source_sign = if nonnegative && positive {
                                Some(RealSign::Positive)
                            } else if nonpositive && negative {
                                Some(RealSign::Negative)
                            } else {
                                None
                            };
                            if let Some(source_sign) = source_sign {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-parallel-monotonicity",
                                    "retained-range-bernstein-interval",
                                );
                                let sign = product_sign(source_sign, derivative_scale);
                                return Ok(Classification::Decided(if direction_reversed {
                                    product_sign(sign, RealSign::Negative)
                                } else {
                                    sign
                                }));
                            }
                        }
                    }
                }
            }

            // A closed range can legitimately include a stationary common
            // endpoint.  Its interval hull contains zero forever even when the
            // tangent cross is strict everywhere in the open span.  Bernstein
            // positivity is the exact finite certificate for that case: zero
            // endpoint controls are harmless because every basis function is
            // positive on `(0, 1)`. These unit-chart certificates apply only
            // when that chart covers the entire retained range.
            let unit = CurveParameterRange2::unit();
            let unit_covers_range = CurveParameterDomain2::new(&unit, None)
                .contains_finite_range(range, strict)?
                == Classification::Decided(true);
            let represented_source_sign =
                certified_direction_values
                    .as_ref()
                    .filter(|_| unit_covers_range)
                    .and_then(|(direction_x, direction_y)| {
                        let degree = tangent_x_coefficients
                            .len()
                            .max(tangent_y_coefficients.len())
                            .checked_sub(1)?;
                        let coefficients = (0..=degree)
                            .map(|power| {
                                direction_x
                                    * tangent_y_coefficients
                                        .get(power)
                                        .cloned()
                                        .unwrap_or_else(Real::zero)
                                    - direction_y
                                        * tangent_x_coefficients
                                            .get(power)
                                            .cloned()
                                            .unwrap_or_else(Real::zero)
                            })
                            .collect::<Vec<_>>();
                        let controls =
                            power_to_bernstein_coefficients(&coefficients, degree).ok()?;
                        let mut retained = None;
                        for control in controls {
                            match (retained, real_sign(&control, strict)?) {
                                (None, sign @ (RealSign::Positive | RealSign::Negative)) => {
                                    retained = Some(sign)
                                }
                                (
                                    Some(expected),
                                    actual @ (RealSign::Positive | RealSign::Negative),
                                ) if expected != actual => return None,
                                (_, RealSign::Zero) | (Some(_), _) => {}
                            }
                        }
                        retained
                    });
            let mut recursive_cross = None;
            let mut source_sign = represented_source_sign;
            if source_sign.is_none()
                && let Classification::Decided(Some(frame)) =
                    direction_authority.recursive_projective_endpoints_with_direction(strict)?
            {
                let [start, end] = frame.direction_endpoints;
                let Some((direction_x, direction_y, _)) = end.difference_numerators(&start) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let field = direction_x.field();
                let Some(tangent_x) =
                    recursive_quadratic_real_polynomial(&field, tangent_x_coefficients)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(tangent_y) =
                    recursive_quadratic_real_polynomial(&field, tangent_y_coefficients)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(cross) = recursive_quadratic_polynomial_scale(&tangent_y, &direction_x)
                    .and_then(|first| {
                        recursive_quadratic_polynomial_scale(&tangent_x, &direction_y).and_then(
                            |second| recursive_quadratic_polynomial_combine(&first, &second, true),
                        )
                    })
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                if unit_covers_range {
                    source_sign = recursive_quadratic_open_unit_bernstein_sign(&cross);
                }
                recursive_cross = Some((field, cross));
            }

            // Mixed Bernstein controls do not imply a root on this retained
            // subrange.  Tangent-cross degree is normally at most quadratic, so
            // isolate its recursive-field roots directly (without a dense norm)
            // and accept endpoint roots while rejecting any strict interior root.
            if source_sign.is_none()
                && let Some((field, cross)) = recursive_cross.as_ref()
                && cross.len() <= 3
            {
                let (roots, mut root_classification_complete) =
                    match recursive_projective_polynomial_parameters(
                        field,
                        cross.clone(), SelectedThirdAxisDomain2::Finite(range),
                        strict,
                    )? {
                        Classification::Decided(roots) => (roots, true),
                        Classification::Uncertain(_) => (Vec::new(), false),
                    };
                let mut has_interior_root = false;
                for root in roots {
                    let first = root.cmp_by_refinement(range.start(), strict)?;
                    let second = root.cmp_by_refinement(range.end(), strict)?;
                    let (Classification::Decided(first), Classification::Decided(second)) =
                        (first, second)
                    else {
                        root_classification_complete = false;
                        break;
                    };
                    has_interior_root |= match range_order {
                        std::cmp::Ordering::Less => {
                            first == std::cmp::Ordering::Greater
                                && second == std::cmp::Ordering::Less
                        }
                        std::cmp::Ordering::Greater => {
                            first == std::cmp::Ordering::Less
                                && second == std::cmp::Ordering::Greater
                        }
                        std::cmp::Ordering::Equal => unreachable!("validated above"),
                    };
                }
                if root_classification_complete && !has_interior_root {
                    let mut sample = field.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology("a recursive tangent cross lost its zero".into())
                    })?;
                    for coefficient in cross.iter().rev() {
                        sample = sample
                            .scale(&interior)
                            .and_then(|value| value.add(coefficient))
                            .ok_or_else(|| {
                                CurveError::Topology(
                                    "a recursive tangent-cross sample exceeded its field budget"
                                        .into(),
                                )
                            })?;
                    }
                    source_sign = match sample.sign(strict)? {
                        Classification::Decided(
                            sign @ (RealSign::Positive | RealSign::Negative),
                        ) => Some(sign),
                        Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                            None
                        }
                    };
                }
            }
            if let Some(source_sign) = source_sign {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotonicity",
                    "retained-range-sign",
                );
                let sign = product_sign(source_sign, derivative_scale);
                return Ok(Classification::Decided(if direction_reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }));
            }
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        })
    }

    /// Publishes the unique contact proved by a strictly monotone support
    /// incidence without forming the recursive coefficient tower's global
    /// norm.  Opposite endpoint sides provide existence, the supplied
    /// nonzero tangent-cross sign provides uniqueness, and exact scalar
    /// bisection retains an authored-sheet bracket for every later predicate.
    pub(crate) fn retained_monotone_parallel_contact_on_region_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        endpoint_sides: [crate::classify::LineSide; 2],
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordRetainedParallelContact2>>> {
        self.validate_policy(policy)?;
        if tangent_cross_sign == RealSign::Zero {
            return Err(CurveError::Topology(
                "a monotone chord/parallel contact requires a nonzero tangent cross".into(),
            ));
        }
        let prepared = policy.strict_predicate_pass(|| {
            let strict = policy;
            let range_order = match range.start().cmp_by_refinement(range.end(), strict)? {
                Classification::Decided(
                    order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                ) => order,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::InvalidBezierRange);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let interior = match range.strict_interior_scalar(strict)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let frame = match parallel
                .source_oriented_regularized_tangent_field_at_interior(&interior, strict)?
            {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Classification::Uncertain(reason) = parallel.certify_source_frame_in_domain(
                SelectedThirdAxisDomain2::Finite(range),
                frame.as_deref(),
                strict,
            )? {
                return Ok(Classification::Uncertain(reason));
            }
            // Splitting and Boolean walking can replace one finite endpoint
            // with a correlated chord-pair contact while retaining the exact
            // authored supporting line.  Incidence depends only on that line;
            // importing the clipped endpoint would unnecessarily join every
            // field owned by both defining supports.  Keep `self` as the
            // finite-domain authority below, but construct the root equation
            // from its smallest retained affine support.
            let (support, support_reversed) = self.smallest_incidence_support();
            #[cfg(feature = "dispatch-trace")]
            if !Arc::ptr_eq(&self.data, &support.data) {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-parallel-monotone",
                    "retained-support-incidence",
                );
            }
            let system = match support.recursive_projective_parallel_system_with_frame(
                parallel,
                frame.as_deref(),
                false,
                strict,
            )? {
                Classification::Decided(Some(system)) => Arc::new(system),
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let weight_sign = match strict.strict_predicate_pass(|| {
                system.polynomial_sign_at_real(&system.source_weight, &interior, strict)
            })? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let side_on_support = |side| {
                if !support_reversed {
                    return side;
                }
                match side {
                    crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                    crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                    crate::classify::LineSide::On => crate::classify::LineSide::On,
                }
            };
            let incidence_sign = |side| match side {
                crate::classify::LineSide::Left => Some(weight_sign),
                crate::classify::LineSide::Right => {
                    Some(product_sign(weight_sign, RealSign::Negative))
                }
                crate::classify::LineSide::On => None,
            };
            let [Some(first_sign), Some(second_sign)] =
                endpoint_sides.map(side_on_support).map(incidence_sign)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            };
            if !strict_signs_are_opposite(Some(first_sign), Some(second_sign)) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            let (mut lower_parameter, mut upper_parameter, lower_sign, upper_sign) =
                match range_order {
                    std::cmp::Ordering::Less => (
                        range.start().clone(),
                        range.end().clone(),
                        first_sign,
                        second_sign,
                    ),
                    std::cmp::Ordering::Greater => (
                        range.end().clone(),
                        range.start().clone(),
                        second_sign,
                        first_sign,
                    ),
                    std::cmp::Ordering::Equal => unreachable!("validated above"),
                };
            let mut lower_real = lower_parameter.scalar().cloned();
            let mut upper_real = upper_parameter.scalar().cloned();
            // A retained monotone bracket may not be authored from an
            // approximate midpoint sign. When the defining chord is
            // replayable under STRICT, finish that pure polynomial sign with
            // exact authority instead of treating the caller's bounded pass
            // as the completeness boundary. Retained point and range
            // topology below must keep the caller's policy identity: those
            // inputs can legitimately have been authored by APPROXIMATE_512.
            let exact_completion_policy = policy.strict_counterpart();
            let midpoint_sign_policy = if policy.has_bounded_exact_predicate_budget()
                && chord_parallel_support_source(self, policy)?.is_none()
                && self.validate_policy(&exact_completion_policy).is_ok()
            {
                &exact_completion_policy
            } else {
                strict
            };
            // Each step keeps an exact bracket, but only a scalar endpoint can
            // be replaced. When the retained algebraic endpoint itself is the
            // incidence, every midpoint keeps the scalar side's sign and the
            // bracket closes on that endpoint without ever replacing it. A
            // strict opposite-side certificate excludes that case, so a stall
            // means the side evidence and this incidence disagree: report it
            // instead of refining without end.
            const MAX_SCALAR_BRACKET_STEPS: usize = 128;
            let mut scalar_bracket_steps = 0;
            let parameter = loop {
                if let (Some(lower), Some(upper)) = (&lower_real, &upper_real) {
                    let parameter = match BezierRecursiveProjectiveParameter2::new_monotone(
                        Arc::clone(&system),
                        self.clone(),
                        parallel.clone(),
                        lower.clone(),
                        upper.clone(),
                        lower_sign,
                        upper_sign,
                        policy,
                    )? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    break CurveParameter2::from_recursive_projective(parameter);
                }
                scalar_bracket_steps += 1;
                if scalar_bracket_steps > MAX_SCALAR_BRACKET_STEPS {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                let midpoint = match CurveParameterRange2::new_validated(
                    lower_parameter.clone(),
                    upper_parameter.clone(),
                )
                .strict_interior_scalar(strict)?
                {
                    Classification::Decided(midpoint) => midpoint,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let midpoint_sign =
                    match system.incidence_sign_at_real(&midpoint, midpoint_sign_policy)? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let midpoint_parameter =
                    CurveParameter2::from(BezierParameter2::Exact(midpoint.clone()));
                match midpoint_sign {
                    RealSign::Zero => break midpoint_parameter,
                    sign if sign == lower_sign => {
                        lower_parameter = midpoint_parameter;
                        lower_real = Some(midpoint);
                    }
                    sign if sign == upper_sign => {
                        upper_parameter = midpoint_parameter;
                        upper_real = Some(midpoint);
                    }
                    _ => {
                        return Err(CurveError::Topology(
                            "a monotone chord/parallel sample left its endpoint sign partition"
                                .into(),
                        ));
                    }
                }
            };
            Ok(Classification::Decided((frame, weight_sign, parameter)))
        })?;
        let (frame, weight_sign, mut parameter) = match prepared {
            Classification::Decided(prepared) => prepared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point_at = |parameter: &CurveParameter2| {
            BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                parallel.clone(),
                parameter,
                frame.clone(),
                Real::zero(),
                policy,
            )
            .map(CurvePoint2::from)
            .ok_or_else(|| {
                CurveError::Topology("a retained monotone contact lost its source parameter".into())
            })
        };
        let mut point = point_at(&parameter)?;
        // Consult construction-owned endpoint order before asking this
        // analytic root for Cartesian bounds. A correlated pair endpoint can
        // separate itself from the root by one retained miter-anchor order;
        // refining the root first would materialize its recursive field tower.
        // A generic endpoint-order terminal may only guess that an unresolved
        // coordinate is equal.  This kernel retains the incidence equation
        // needed for exact finite clipping below, so exhaust endpoint identity
        // with terminal approximation suppressed before consuming that
        // correlated authority.
        let exact = policy
            .strict_predicate_pass(|| self.parameter_at_certified_point(point.clone(), policy))?;
        let mut chord_parameter = match exact {
            Classification::Decided(Some(parameter)) => Some(parameter),
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(_) => None,
        };

        if chord_parameter.is_none() {
            // Strictly separated outward boxes are the next-smallest
            // finite-domain authority. Endpoint overlap falls through to the
            // selected-coordinate elimination below.
            match self
                .parameter_at_certified_support_point_by_local_evidence(point.clone(), policy)?
            {
                Classification::Decided(Some(parameter)) => {
                    chord_parameter = Some(parameter);
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(_) => {}
            }
        }

        if chord_parameter.is_none() {
            // Endpoint boxes overlap this root, so construct the exact
            // selected-axis expressions on demand.  At the known incidence
            // root `A + B sqrt(V) = 0`, the sign of a coordinate difference
            // `D + E sqrt(V)` is the sign of `(D B - E A) / B`; this removes
            // the shared radical without forming the global field norm.
            let clipping_system = match self.recursive_projective_parallel_system_with_frame(
                parallel,
                frame.as_deref(),
                true,
                policy,
            )? {
                Classification::Decided(Some(system)) => system,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let coordinate_differences = clipping_system
                .coordinate_differences
                .as_ref()
                .expect("the clipping system retains endpoint coordinates");
            let eliminated_coordinate_numerators = (|| {
                let eliminate = |difference: &BezierRecursiveQuadraticParallelExpression2| {
                    let rational_radical = recursive_quadratic_polynomial_multiply(
                        &difference.rational,
                        &clipping_system.incidence.radical,
                    )?;
                    let radical_rational = recursive_quadratic_polynomial_multiply(
                        &difference.radical,
                        &clipping_system.incidence.rational,
                    )?;
                    recursive_quadratic_polynomial_combine(
                        &rational_radical,
                        &radical_rational,
                        true,
                    )
                };
                Some([
                    eliminate(&coordinate_differences[0])?,
                    eliminate(&coordinate_differences[1])?,
                ])
            })();
            let mut finite_location = None;
            for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64] {
                if refinement_steps != 0 {
                    parameter =
                        match parameter.refined_for_finite_envelope(refinement_steps, policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(_) => break,
                        };
                }
                let Some((lower, upper)) = parameter.finite_envelope_bounds() else {
                    break;
                };
                let target = RealInterval {
                    lower: lower.clone(),
                    upper: upper.clone(),
                };
                let polynomial_sign = |polynomial: &[BezierRecursiveQuadraticValue2]| {
                    [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512]
                        .into_iter()
                        .find_map(|source_steps| {
                            let coefficient_bits =
                                source_steps.saturating_add(64).min(i32::MAX as usize) as i32;
                            recursive_quadratic_polynomial_interval(
                                polynomial,
                                &target,
                                source_steps,
                                -coefficient_bits,
                            )
                            .as_ref()
                            .and_then(dense_strict_interval_sign)
                        })
                };
                let mut coordinate_signs = [None; 2];
                if let (Some(numerators), Some(radical_sign)) = (
                    eliminated_coordinate_numerators.as_ref(),
                    polynomial_sign(&clipping_system.incidence.radical)
                        .filter(|sign| *sign != RealSign::Zero),
                ) {
                    for (sign, numerator) in coordinate_signs.iter_mut().zip(numerators) {
                        *sign = polynomial_sign(numerator).map(|numerator_sign| {
                            product_sign(product_sign(numerator_sign, radical_sign), weight_sign)
                        });
                    }
                }
                for (sign, difference) in coordinate_signs.iter_mut().zip(coordinate_differences) {
                    if sign.is_some() {
                        continue;
                    }
                    for source_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                        let coefficient_bits =
                            source_steps.saturating_add(64).min(i32::MAX as usize) as i32;
                        *sign = recursive_quadratic_parallel_expression_interval(
                            difference,
                            false,
                            &target,
                            source_steps,
                            -coefficient_bits,
                        )
                        .as_ref()
                        .and_then(dense_strict_interval_sign)
                        .map(|sign| product_sign(sign, weight_sign));
                        if sign.is_some() {
                            break;
                        }
                    }
                }
                if !self.data.parameter_axis.coordinate_increases {
                    coordinate_signs = coordinate_signs
                        .map(|sign| sign.map(|sign| product_sign(sign, RealSign::Negative)));
                }
                finite_location = match coordinate_signs {
                    [Some(RealSign::Negative), _] | [_, Some(RealSign::Positive)] => Some(false),
                    [Some(RealSign::Positive), Some(RealSign::Negative)] => Some(true),
                    _ => None,
                };
                if finite_location.is_some() {
                    break;
                }
            }
            point = point_at(&parameter)?;
            chord_parameter = match finite_location {
                Some(true) => Some(self.parameter_at_certified_interior_point(point.clone())),
                Some(false) => return Ok(Classification::Decided(None)),
                None => match self.parameter_at_certified_point(point.clone(), policy)? {
                    Classification::Decided(Some(parameter)) => Some(parameter),
                    Classification::Decided(None) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
        }
        let chord_parameter = chord_parameter.expect("the finite root was classified above");
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-kernel",
            "retained-monotone-root",
        );
        Ok(Classification::Decided(Some(
            BezierAlgebraicChordRetainedParallelContact2 {
                chord_parameter,
                parallel_parameter: parameter,
                point,
                tangent_cross_sign,
            },
        )))
    }

    /// Builds the same procedural affine support from the oldest collinear
    /// source chord before importing its endpoints into a recursive field.
    ///
    /// Boolean clipping can replace one source endpoint with a correlated
    /// contact while retaining the authored line in `source`.  Applying the
    /// common unit-normal displacement to any two points on that line yields
    /// the same displaced support, so support-only incidence should use the
    /// smallest exact endpoint fields.  Finite clipping deliberately keeps
    /// using the descendant endpoints through the general frame below.
    pub(in crate::bezier_offset) fn compact_recursive_projective_parallel_support_frame(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelRecursiveFrame2>>> {
        let Some(structural) = chord_parallel_support_source(self, policy)? else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: no structural source");
            }
            return Ok(Classification::Decided(None));
        };
        if structural.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: non-normal displacement");
            }
            return Ok(Classification::Decided(None));
        }
        let (base, source_reversed) = structural.source.smallest_retained_support();
        let Some(self_reversed) = self.shared_tangent_orientation(&structural.source) else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!("compact procedural support: unknown source orientation");
            }
            return Ok(Classification::Decided(None));
        };
        let distance = if source_reversed {
            -structural.distance
        } else {
            structural.distance
        };
        let (point, _) = BezierAlgebraicChordParallelPoint2::new_pair(
            base.clone(),
            distance,
            structural.translation_x,
            structural.translation_y,
            policy,
        );
        let mut frame = match point.recursive_projective_frame(policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                    eprintln!("compact procedural support: frame unsupported");
                }
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                    eprintln!("compact procedural support: frame uncertain {reason:?}");
                }
                return Ok(Classification::Uncertain(reason));
            }
        };
        if self_reversed ^ source_reversed {
            frame.displaced.swap(0, 1);
            frame.direction_endpoints.swap(0, 1);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-projective-chord-parallel-frame",
            "canonical-procedural-support",
        );
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "compact procedural support: retained source-reversed={source_reversed} self-reversed={self_reversed}"
            );
        }
        Ok(Classification::Decided(Some(frame)))
    }

    #[track_caller]
    pub(in crate::bezier_offset) fn recursive_projective_parallel_system_with_frame(
        &self,
        parallel: &BezierParallel2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        build_coordinate_differences: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveProjectiveChordParallelSystem2>>> {
        let compact_frame = if !build_coordinate_differences {
            match self.compact_recursive_projective_parallel_support_frame(policy)? {
                Classification::Decided(frame) => frame,
                // This is a representation fast path.  A field-capacity or
                // predicate miss must not reduce the complete generic path.
                Classification::Uncertain(_) => None,
            }
        } else {
            None
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && compact_frame.is_none()
        {
            let caller = std::panic::Location::caller();
            eprintln!(
                "recursive parallel system generic caller={}:{} coordinates={build_coordinate_differences}",
                caller.file(),
                caller.line(),
            );
        }
        let frame = match compact_frame {
            Some(frame) => frame,
            None => match self.recursive_projective_endpoints_with_direction(policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let [start, end] = frame.displaced;
        let [direction_start, direction_end] = frame.direction_endpoints;
        let field = start.denominator.field();
        let source = parallel.source_power_basis()?;
        let differential = parallel.differential()?;
        let (tangent_x_coefficients, tangent_y_coefficients) = frame_tangent
            .map(|frame| (&frame.x[..], &frame.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));
        // The formal source equation is independent of a consumed parameter
        // domain. Whole-domain consumers certify their actual range or ray;
        // point predicates replay weight and speed at their retained parameter.
        let unit = [Real::one()];
        let weight_coefficients = source.weight.unwrap_or(&unit);
        let Some((incidence, source_weight, tangent_cross, tangent_dot, coordinate_differences)) =
            (|| {
                let real = |coefficients: &[Real]| {
                    recursive_quadratic_real_polynomial(&field, coefficients)
                };
                let add = |first: &[BezierRecursiveQuadraticValue2],
                           second: &[BezierRecursiveQuadraticValue2]| {
                    recursive_quadratic_polynomial_combine(first, second, false)
                };
                let subtract =
                    |first: &[BezierRecursiveQuadraticValue2],
                     second: &[BezierRecursiveQuadraticValue2]| {
                        recursive_quadratic_polynomial_combine(first, second, true)
                    };
                let multiply =
                    |first: &[BezierRecursiveQuadraticValue2],
                     second: &[BezierRecursiveQuadraticValue2]| {
                        recursive_quadratic_polynomial_multiply(first, second)
                    };
                let scale = |polynomial: &[BezierRecursiveQuadraticValue2],
                             value: &BezierRecursiveQuadraticValue2| {
                    recursive_quadratic_polynomial_scale(polynomial, value)
                };
                let source_x = real(source.x_numerator)?;
                let source_y = real(source.y_numerator)?;
                let source_weight = real(weight_coefficients)?;
                let tangent_x = real(tangent_x_coefficients)?;
                let tangent_y = real(tangent_y_coefficients)?;
                let (direction_x, direction_y, _) =
                    direction_end.difference_numerators(&direction_start)?;
                let point_delta_x = subtract(
                    &scale(&source_x, &start.denominator)?,
                    &scale(&source_weight, &start.x)?,
                )?;
                let point_delta_y = subtract(
                    &scale(&source_y, &start.denominator)?,
                    &scale(&source_weight, &start.y)?,
                )?;
                let source_incidence = subtract(
                    &scale(&point_delta_y, &direction_x)?,
                    &scale(&point_delta_x, &direction_y)?,
                )?;
                let tangent_dot = add(
                    &scale(&tangent_x, &direction_x)?,
                    &scale(&tangent_y, &direction_y)?,
                )?;
                let normal_incidence = recursive_quadratic_polynomial_scale_real(
                    &scale(&multiply(&tangent_dot, &source_weight)?, &start.denominator)?,
                    parallel.distance(),
                )?;
                let zero_distance = parallel.distance().zero_status() == ZeroKnowledge::Zero;
                let speed_squared: Arc<[_]> = if zero_distance {
                    real(&[Real::one()])?
                } else {
                    add(
                        &multiply(&tangent_x, &tangent_x)?,
                        &multiply(&tangent_y, &tangent_y)?,
                    )?
                }
                .into();
                let tangent_cross = subtract(
                    &scale(&tangent_y, &direction_x)?,
                    &scale(&tangent_x, &direction_y)?,
                )?;
                let (source_coordinate, start_coordinate, end_coordinate, normal_coordinate) =
                    match self.data.parameter_axis.axis {
                        Axis2::X => (
                            &source_x,
                            &start.x,
                            &end.x,
                            recursive_quadratic_polynomial_scale_real(
                                &tangent_y,
                                &Real::from(-1_i8),
                            )?,
                        ),
                        Axis2::Y => (&source_y, &start.y, &end.y, tangent_x.clone()),
                    };
                let coordinate_difference =
                    |endpoint_coordinate: &BezierRecursiveQuadraticValue2,
                     endpoint_denominator: &BezierRecursiveQuadraticValue2| {
                        let radical = subtract(
                            &scale(source_coordinate, endpoint_denominator)?,
                            &scale(&source_weight, endpoint_coordinate)?,
                        )?;
                        let rational = recursive_quadratic_polynomial_scale_real(
                            &scale(
                                &multiply(&normal_coordinate, &source_weight)?,
                                endpoint_denominator,
                            )?,
                            parallel.distance(),
                        )?;
                        Some(if zero_distance {
                            BezierRecursiveQuadraticParallelExpression2::new(
                                radical,
                                real(&[Real::zero()])?,
                                speed_squared.clone(),
                            )
                        } else {
                            BezierRecursiveQuadraticParallelExpression2::new(
                                rational,
                                radical,
                                speed_squared.clone(),
                            )
                        })
                    };
                let coordinate_differences = build_coordinate_differences
                    .then(|| {
                        Some([
                            coordinate_difference(start_coordinate, &start.denominator)?,
                            coordinate_difference(end_coordinate, &end.denominator)?,
                        ])
                    })
                    .flatten();
                Some((
                    if zero_distance {
                        BezierRecursiveQuadraticParallelExpression2::new(
                            source_incidence,
                            real(&[Real::zero()])?,
                            speed_squared.clone(),
                        )
                    } else {
                        BezierRecursiveQuadraticParallelExpression2::new(
                            normal_incidence,
                            source_incidence,
                            speed_squared.clone(),
                        )
                    },
                    source_weight,
                    tangent_cross,
                    tangent_dot,
                    coordinate_differences,
                ))
            })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let system = BezierRecursiveProjectiveChordParallelSystem2 {
            base: field.base_and_extension_path().0,
            field,
            projection: OnceLock::new(),
            incidence,
            source_weight,
            tangent_cross,
            tangent_dot,
            coordinate_differences,
        };
        Ok(Classification::Decided(Some(system)))
    }

    /// Replays one target domain against the authored positive target-speed
    /// sheet of the recursive projective chord equation. The squared norm is
    /// candidate enumeration only: every retained candidate must satisfy the
    /// unsquared incidence strictly before it may become topology evidence.
    pub(in crate::bezier_offset) fn recursive_projective_parallel_intersections_in_domain(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        if let Classification::Uncertain(reason) = parallel.certify_source_frame_in_domain(
            domain,
            frame_tangent.map(Arc::as_ref),
            policy,
        )? {
            return Ok(Classification::Uncertain(reason));
        }
        let endpoint_roots = [self.start(), self.end()].map(|point| {
            let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point else {
                return None;
            };
            (point.data.parallel == *parallel
                && point.data.frame_tangent.as_ref() == frame_tangent
                && policy.accepts_retained_policy(point.data.policy)
                && [
                    &point.data.tangent_distance,
                    &point.data.translation_x,
                    &point.data.translation_y,
                ]
                .into_iter()
                .all(|value| value.zero_status() == ZeroKnowledge::Zero))
            .then(|| point.data.parameter.curve_parameter())
        });
        if let Some(candidates) = policy
            .strict_predicate_pass(|| system.local_parameters(domain, endpoint_roots, policy))?
            && let Classification::Decided(contacts) = policy.strict_predicate_pass(|| {
                self.recursive_projective_parallel_contacts(
                    parallel,
                    system,
                    candidates,
                    BezierRecursiveParallelCandidateEvidence2::SelectedNorm,
                    frame_tangent,
                    derivative_scale_sign,
                    clip_to_finite_chord,
                    policy,
                )
            })?
        {
            return Ok(Classification::Decided(contacts));
        }
        self.recursive_projective_parallel_intersections_from_projection(
            parallel,
            system,
            frame_tangent,
            derivative_scale_sign,
            domain,
            component_sample,
            clip_to_finite_chord,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn recursive_projective_parallel_intersections_from_projection(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        let (candidates, original_projection_is_discrete) = match system
            .parameters(domain, policy)?
        {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                (candidates, true)
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                let sample = match component_sample.cloned().map(Classification::Decided) {
                    Some(sample) => sample,
                    None => domain.strict_sample(policy)?,
                };
                let sample = match sample {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match system.norm_component_sheet_at_real(&sample, policy)? {
                    Classification::Decided(Some(true)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "recursive-chord-parallel-degenerate",
                            "coincident-support",
                        );
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent { sample },
                        ));
                    }
                    Classification::Decided(Some(false)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "recursive-chord-parallel-degenerate",
                            "opposite-speed-sheet",
                        );
                    }
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }

                // A target-wide squared norm can belong wholly to the
                // opposite positive-speed sheet. Authored-sheet contacts are
                // then exactly the common zeros of the two unsquared terms.
                // Enumerate either nonzero term and replay the complete
                // expression below; no parallel-specific component solver is
                // introduced.
                let mut residual = None;
                for coefficients in [&system.incidence.radical, &system.incidence.rational] {
                    let Some(projection) = system.projected_polynomial(coefficients) else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    match system.projected_parameters(&projection, domain, policy)? {
                        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                            candidates,
                        )) => {
                            residual = Some(candidates);
                            break;
                        }
                        Classification::Decided(
                            BezierAlgebraicFiberProjection2::IdenticallyZero,
                        ) => {}
                        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                            return Ok(Classification::Decided(
                                BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let Some(candidates) = residual else {
                    return Ok(Classification::Decided(
                        BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                    ));
                };
                (candidates, false)
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.recursive_projective_parallel_contacts(
            parallel,
            system,
            candidates.into_iter().map(CurveParameter2::from).collect(),
            if original_projection_is_discrete {
                BezierRecursiveParallelCandidateEvidence2::Projected(
                    system
                        .projection
                        .get()
                        .expect("the global chord/parallel enumerator retains its projection"),
                )
            } else {
                BezierRecursiveParallelCandidateEvidence2::Replay
            },
            frame_tangent,
            derivative_scale_sign,
            clip_to_finite_chord,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn recursive_projective_parallel_contacts(
        &self,
        parallel: &BezierParallel2,
        system: &BezierRecursiveProjectiveChordParallelSystem2,
        candidates: Vec<CurveParameter2>,
        evidence: BezierRecursiveParallelCandidateEvidence2<'_>,
        frame_tangent: Option<&Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        let projection = match evidence {
            BezierRecursiveParallelCandidateEvidence2::Projected(projection) => Some(projection),
            _ => None,
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for mut candidate in candidates {
            // A local incidence root can rediscover a scalar already owned
            // by a chord endpoint. Reuse that authority before branch and
            // tangent replay rebuild a high-degree remainder over the joined
            // endpoint field. Only exact scalar equality permits reuse;
            // overlapping isolators or point equality alone do not.
            if candidate.as_recursive_projective().is_some() {
                for point in [self.start(), self.end()] {
                    let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point else {
                        continue;
                    };
                    if point.data.parallel != *parallel
                        || !policy.accepts_retained_policy(point.data.policy)
                    {
                        continue;
                    }
                    let retained = point.data.parameter.curve_parameter();
                    if policy
                        .bounded_exact_predicate_pass(|| candidate.same_value(&retained, policy))?
                        == Classification::Decided(true)
                    {
                        candidate = retained;
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-parallel-kernel",
                            "owned-endpoint-parameter",
                        );
                        break;
                    }
                }
            }
            let projected_incidence = projection.zip(candidate.as_bezier_parameter()).and_then(
                |(projection, parameter)| {
                    projected_selected_dense_candidate_box_incidence(
                        projection,
                        &system.base.sources,
                        parameter,
                        64,
                        64,
                    )
                },
            );
            let projected_certificate = match projected_incidence {
                Some(BezierDenseCandidateBoxIncidence2::Root(certificate)) => Some(certificate),
                Some(BezierDenseCandidateBoxIncidence2::Disjoint(_)) => continue,
                None => None,
            };
            let evaluation = if let Some(parameter) = candidate.as_bezier_parameter() {
                match system.candidate_evaluation(parameter, policy)? {
                    Classification::Decided(Some(evaluation)) => Some(evaluation),
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                None
            };
            let polynomial_sign = |polynomial: &[BezierRecursiveQuadraticValue2]| {
                if let Some(evaluation) = &evaluation {
                    system.polynomial_sign(polynomial, evaluation, policy)
                } else {
                    recursive_projective_polynomial_sign_at_parameter(
                        &system.field,
                        polynomial,
                        &candidate,
                        policy,
                    )
                }
            };
            let weight_sign = match policy
                .strict_predicate_pass(|| polynomial_sign(&system.source_weight))?
            {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let expression_sign = |expression: &BezierRecursiveQuadraticParallelExpression2| {
                if let Some(evaluation) = &evaluation {
                    system.expression_sign(expression, evaluation, policy)
                } else {
                    expression.sign_with_positive_speed(policy, polynomial_sign)
                }
            };
            // This is the terminal authored-sheet predicate, not persistent
            // object construction. The selected policy must therefore remain
            // able to consume APPROXIMATE_512 after every exact interval and
            // projected-zero certificate has declined.
            let replay = if matches!(
                evidence,
                BezierRecursiveParallelCandidateEvidence2::SelectedNorm
            ) {
                // Deflation changes the defining polynomial, not this norm
                // certificate. Reuse it without reconstructing a remainder or
                // refining intervals around an already-proven exact zero.
                if system
                    .incidence
                    .radical
                    .iter()
                    .all(BezierRecursiveQuadraticValue2::is_structurally_zero)
                {
                    Classification::Decided(RealSign::Zero)
                } else {
                    positive_root_sum_sign_from_components(
                        polynomial_sign(&system.incidence.rational)?,
                        polynomial_sign(&system.incidence.radical)?,
                        Classification::Decided(RealSign::Zero),
                    )
                }
            } else if let (Some(certificate), Some(evaluation)) =
                (&projected_certificate, &evaluation)
            {
                system.certified_expression_replay_sign(
                    &system.incidence,
                    evaluation,
                    certificate,
                    policy,
                )?
            } else {
                expression_sign(&system.incidence)?
            };
            match replay {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Negative | RealSign::Positive) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let contact_parameter = projected_certificate
                .as_ref()
                .and_then(|certificate| certificate.candidate.exact_point_witness())
                .cloned()
                .map(CurveParameter2::from)
                .unwrap_or_else(|| candidate.clone());
            let Some(point) =
                BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                    parallel.clone(),
                    &contact_parameter,
                    frame_tangent.cloned(),
                    Real::zero(),
                    policy,
                )
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let point = CurvePoint2::from(point);
            let chord_parameter = if clip_to_finite_chord {
                let mut coordinate_signs = [RealSign::Zero; 2];
                for (sign, difference) in coordinate_signs.iter_mut().zip(
                    system
                        .coordinate_differences
                        .as_ref()
                        .expect("the all-roots system retains finite-chord coordinates")
                        .iter(),
                ) {
                    *sign = match policy.strict_predicate_pass(|| expression_sign(difference))? {
                        Classification::Decided(sign) => product_sign(sign, weight_sign),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                }
                if !self.data.parameter_axis.coordinate_increases {
                    coordinate_signs =
                        coordinate_signs.map(|sign| product_sign(sign, RealSign::Negative));
                }
                match coordinate_signs {
                    [RealSign::Zero, RealSign::Zero] => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    [RealSign::Zero, _] => self.start_parameter(),
                    [_, RealSign::Zero] => self.end_parameter(),
                    [RealSign::Positive, RealSign::Negative] => {
                        self.parameter_at_certified_interior_point(point.clone())
                    }
                    [RealSign::Negative, _] | [_, RealSign::Positive] => continue,
                }
            } else {
                self.parameter_at_certified_support_point(point.clone(), policy)?
            };
            let source_cross =
                match policy.strict_predicate_pass(|| polynomial_sign(&system.tangent_cross))? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let source_dot =
                match policy.strict_predicate_pass(|| polynomial_sign(&system.tangent_dot))? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let derivative_scale = match if let Some(sign) = derivative_scale_sign {
                Classification::Decided(sign)
            } else {
                parallel.parallel_derivative_scale_sign(&contact_parameter, policy)?
            } {
                Classification::Decided(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicChordParallelContact2 {
                point: chord_parameter.point().clone(),
                chord_parameter,
                parallel_parameter: contact_parameter,
                tangent_cross_sign: product_sign(source_cross, derivative_scale),
                tangent_dot_sign: product_sign(source_dot, derivative_scale),
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-kernel",
            "recursive-projective",
        );
        Ok(Classification::Decided(
            BezierAlgebraicChordParallelIntersections2::Contacts(contacts),
        ))
    }

    /// Two distinct points on a strictly convex closed curve exhaust its
    /// intersections with their secant. Retain those endpoint authorities
    /// instead of rediscovering their roots over a joined coefficient field.
    /// Unit seam aliases remain separate parameter contacts; finite clipping
    /// consumes the caller's original range, including reversed ranges.
    pub(in crate::bezier_offset) fn closed_parallel_endpoint_contacts(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<BezierAlgebraicChordParallelContact2>>> {
        policy.strict_predicate_pass(|| {
            let [
                CurvePoint2(CurvePointData2::AnalyticParallel(start)),
                CurvePoint2(CurvePointData2::AnalyticParallel(end)),
            ] = [self.start(), self.end()]
            else {
                return Ok(None);
            };
            let unit = CurveParameterRange2::unit();
            let unit_domain = CurveParameterDomain2::new(&unit, None);
            let endpoints = [start, end];
            for point in endpoints {
                if point.data.parallel != *parallel
                    || point.data.frame_tangent.is_some()
                    || !policy.accepts_retained_policy(point.data.policy)
                    || [
                        &point.data.tangent_distance,
                        &point.data.translation_x,
                        &point.data.translation_y,
                    ]
                    .into_iter()
                    .any(|value| value.zero_status() != ZeroKnowledge::Zero)
                    || unit_domain.contains_finite_parameter(
                        &point.data.parameter.curve_parameter(),
                        policy,
                    )? != Classification::Decided(true)
                {
                    return Ok(None);
                }
            }
            if unit_domain.contains_finite_range(range, policy)? != Classification::Decided(true)
                || !parallel.certifies_simple_closed_parallel(policy)?
            {
                return Ok(None);
            }
            let differential = parallel.differential()?;
            let constant =
                |polynomial: &[Real]| polynomial.first().cloned().unwrap_or_else(Real::zero);
            let turn = Real::diff_of_products(
                &constant(&differential.tangent_x),
                &constant(&differential.tangent_derivative_y),
                &constant(&differential.tangent_y),
                &constant(&differential.tangent_derivative_x),
            );
            let Some(turn @ (RealSign::Positive | RealSign::Negative)) = real_sign(&turn, policy)
            else {
                return Ok(None);
            };
            let mut contacts: Vec<BezierAlgebraicChordParallelContact2> = Vec::new();
            for (index, point) in endpoints.into_iter().enumerate() {
                let parameter = point.data.parameter.curve_parameter();
                // Convexity has already proved the complete contact set.
                // Finish its tangent evidence in the retained endpoint fields;
                // a speculative field-join budget must not force fresh root
                // discovery merely to recover these same owned contacts.
                let Classification::Decided(dot) = self
                    .tangent_cross_dot_parallel_linear_combination_sign(
                        parallel,
                        &parameter,
                        &Real::zero(),
                        &Real::one(),
                        policy,
                    )?
                else {
                    return Ok(None);
                };
                let mut parameters = vec![parameter.clone()];
                for (seam, alias) in [(Real::zero(), Real::one()), (Real::one(), Real::zero())] {
                    match parameter.same_value(&seam.into(), policy)? {
                        Classification::Decided(true) => parameters.push(alias.into()),
                        Classification::Decided(false) => {}
                        Classification::Uncertain(_) => return Ok(None),
                    }
                }
                let chord_parameter = if index == 0 {
                    self.start_parameter()
                } else {
                    self.end_parameter()
                };
                for parameter in parameters {
                    match CurveParameterDomain2::new(range, None)
                        .contains_finite_parameter(&parameter, policy)?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(_) => return Ok(None),
                    }
                    let mut position = contacts.len();
                    for (index, contact) in contacts.iter().enumerate() {
                        match parameter.cmp_by_refinement(&contact.parallel_parameter, policy)? {
                            Classification::Decided(std::cmp::Ordering::Less) => {
                                position = index;
                                break;
                            }
                            Classification::Decided(std::cmp::Ordering::Greater) => {}
                            Classification::Decided(std::cmp::Ordering::Equal)
                            | Classification::Uncertain(_) => return Ok(None),
                        }
                    }
                    contacts.insert(
                        position,
                        BezierAlgebraicChordParallelContact2 {
                            point: chord_parameter.point().clone(),
                            chord_parameter: chord_parameter.clone(),
                            parallel_parameter: parameter,
                            tangent_cross_sign: if index == 0 {
                                product_sign(turn, RealSign::Negative)
                            } else {
                                turn
                            },
                            tangent_dot_sign: dot,
                        },
                    );
                }
            }
            Ok(Some(contacts))
        })
    }

    pub(in crate::bezier_offset) fn recursive_projective_parallel_intersections_with_frame(
        &self,
        parallel: &BezierParallel2,
        frame_tangent: Option<Arc<BezierAnalyticParallelTangentField2>>,
        derivative_scale_sign: Option<RealSign>,
        domain: SelectedThirdAxisDomain2<'_>,
        component_sample: Option<&Real>,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelIntersections2>>> {
        if clip_to_finite_chord
            && frame_tangent.is_none()
            && let SelectedThirdAxisDomain2::Finite(range) = domain
            && let Some(contacts) =
                self.closed_parallel_endpoint_contacts(parallel, range, policy)?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parallel-kernel",
                "closed-convex-endpoints",
            );
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordParallelIntersections2::Contacts(contacts),
            )));
        }
        if clip_to_finite_chord {
            // Enumerate the infinite-support contacts before importing the two
            // finite descendant endpoints. Procedural chords can then use their
            // oldest exact support frame; only actual contacts need a finite
            // axis-order test. If a contact cannot be clipped from local boxes,
            // the complete joined endpoint system below remains authoritative.
            let support_system = match self.recursive_projective_parallel_system_with_frame(
                parallel,
                frame_tangent.as_deref(),
                false,
                policy,
            )? {
                Classification::Decided(Some(system)) => Some(system),
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            };
            if let Some(support_system) = support_system
                && let Classification::Decided(intersections) = self
                    .recursive_projective_parallel_intersections_in_domain(
                        parallel,
                        &support_system,
                        frame_tangent.as_ref(),
                        derivative_scale_sign,
                        domain,
                        component_sample,
                        false,
                        policy,
                    )?
            {
                match intersections {
                    BezierAlgebraicChordParallelIntersections2::Contacts(contacts) => {
                        let mut clipped = Vec::with_capacity(contacts.len());
                        let mut complete = true;
                        for mut contact in contacts {
                            match policy.bounded_exact_predicate_pass(|| {
                                self.parameter_at_certified_support_point_by_local_evidence(
                                    contact.point.clone(),
                                    policy,
                                )
                            })? {
                                Classification::Decided(Some(parameter)) => {
                                    contact.point = parameter.point().clone();
                                    contact.chord_parameter = parameter;
                                    clipped.push(contact);
                                }
                                Classification::Decided(None) => {}
                                Classification::Uncertain(_) => {
                                    complete = false;
                                    break;
                                }
                            }
                        }
                        if complete {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-parallel-kernel",
                                "support-first-finite-clipping",
                            );
                            return Ok(Classification::Decided(Some(
                                BezierAlgebraicChordParallelIntersections2::Contacts(clipped),
                            )));
                        }
                    }
                    intersection => {
                        return Ok(Classification::Decided(Some(intersection)));
                    }
                }
            }
        }
        let system = match self.recursive_projective_parallel_system_with_frame(
            parallel,
            frame_tangent.as_deref(),
            true,
            policy,
        )? {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(self
            .recursive_projective_parallel_intersections_in_domain(
                parallel,
                &system,
                frame_tangent.as_ref(),
                derivative_scale_sign,
                domain,
                component_sample,
                clip_to_finite_chord,
                policy,
            )?
            .map(Some))
    }

    /// Replays every finite contact between this retained chord and a
    /// genuinely analytic parallel.
    ///
    /// Both authored endpoints enter their least shared recursive projective
    /// tower. Complete local root isolation keeps simple contacts in that
    /// coefficient field; unresolved cases demand the global norm projection.
    /// Both enumerators replay the authored positive-speed sheet before
    /// finite chord containment and tangent orientation are published. No
    /// approximate value selects a carrier representation.
    #[track_caller]
    pub(crate) fn parallel_intersections(
        &self,
        parallel: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let caller = std::panic::Location::caller();
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            eprintln!(
                "chord parallel intersections caller={}:{} endpoints=({},{}) support=({},{})",
                caller.file(),
                caller.line(),
                kind(self.start()),
                kind(self.end()),
                kind(self.retained_support().start()),
                kind(self.retained_support().end()),
            );
        }
        Ok(
            match self.recursive_projective_parallel_intersections_with_frame(
                parallel,
                None,
                None,
                SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
                None,
                true,
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
        )
    }

    /// Runs the authoritative chord/parallel incidence on one regular side
    /// of a source cusp.  The exact common hodograph factor is cancelled only
    /// for the selected unit-normal frame; source coordinates and the shared
    /// algebraic chord solver are otherwise unchanged.
    pub(crate) fn parallel_intersections_on_regular_range(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        let component_sample =
            match policy.strict_predicate_pass(|| range.strict_interior_scalar(policy))? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let frame = match policy.strict_predicate_pass(|| {
            parallel
                .source_oriented_regularized_tangent_field_at_interior(&component_sample, policy)
        })? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let derivative_scale_sign = if frame.is_some() {
            match parallel.parallel_derivative_scale_sign_at_exact(&component_sample, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    Some(sign)
                }
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            parallel.certified_derivative_scale_sign_on_range(range, policy)?
        };
        Ok(
            match self.recursive_projective_parallel_intersections_with_frame(
                parallel,
                frame,
                derivative_scale_sign,
                SelectedThirdAxisDomain2::Finite(range),
                Some(&component_sample),
                true,
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
        )
    }

    /// Replays this finite chord and its complete affine support against the
    /// authored analytic span plus one regular incident projective ray.
    ///
    /// This is the `TrimOrExtend` domain for a chord/parallel corner. The
    /// recursive incidence system is built once; both the authored target
    /// span and its exterior ray replay against the chord's affine support.
    /// Only final-axis isolation differs. The ray stops before its first source
    /// pole or tangent-speed zero.
    pub(crate) fn parallel_intersections_with_incident_ray(
        &self,
        parallel: &BezierParallel2,
        incident: &BezierParallelIncidentDomain2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordParallelIntersections2>> {
        self.validate_policy(policy)?;
        let system = match self
            .recursive_projective_parallel_system_with_frame(parallel, None, true, policy)?
        {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let finite = match self.recursive_projective_parallel_intersections_in_domain(
            parallel,
            &system,
            None,
            None,
            SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
            None,
            false,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let exterior = match self.recursive_projective_parallel_intersections_in_domain(
            parallel,
            &system,
            None,
            None,
            SelectedThirdAxisDomain2::IncidentRay {
                anchor: incident.anchor(),
                direction: incident.direction(),
                barrier: incident.barrier(),
            },
            None,
            false,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (finite, exterior) {
            (
                BezierAlgebraicChordParallelIntersections2::Contacts(mut finite),
                BezierAlgebraicChordParallelIntersections2::Contacts(exterior),
            ) => {
                finite.extend(exterior);
                BezierAlgebraicChordParallelIntersections2::Contacts(finite)
            }
            (
                component
                @ BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                    ..
                },
                _,
            )
            | (
                _,
                component
                @ BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                    ..
                },
            ) => component,
            _ => BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
        }))
    }

    /// Replays every finite contact between this retained chord and an
    /// arbitrary rational Bezier without adjoining its finite-boundary fields.
    ///
    /// The stable support's first endpoint field is eliminated into a
    /// bivariate polynomial by Hypersolve, its second endpoint field is
    /// eliminated into source-parameter candidates, and every candidate is
    /// replayed against the selected root triple before the current finite
    /// boundary admits it as topology evidence. One optional source parameter
    /// may be omitted when authored adjacency already owns it.
    pub(in crate::bezier_offset) fn exact_line_retained_circle_intersections(
        &self,
        line: &LineSeg2,
        source: &RationalBezier2,
        excluded_source_parameter: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<BezierAlgebraicChordRationalIntersections2>>> {
        if source.retained_circular_conic().is_none() {
            return Ok(None);
        }
        let arc = match crate::arc_bezier::rational_bezier_circular_arc(source, policy)? {
            Classification::Decided(Some(arc)) => arc,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        // A provenance line may be only a short directed witness for the
        // infinite support.  Classify that support first and let the retained
        // parameterization and this finite chord perform the two authoritative domain
        // filters; `intersect_arc` would incorrectly clip to the witness.
        let hits = match line.supporting_line_circle_relation(&arc, policy)? {
            LineCircleRelation::Disjoint => Vec::new(),
            LineCircleRelation::Tangent { point, .. } => vec![point],
            LineCircleRelation::Secant {
                first_point,
                second_point,
                ..
            } => vec![first_point, second_point],
            LineCircleRelation::Uncertain { reason } => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let (chord_dx, chord_dy) = line.delta();
        let mut contacts = Vec::with_capacity(hits.len());
        for hit in hits {
            // The inverse retained-circle parameter map is itself the exact
            // finite-span admission certificate.  Reclassifying the same hit
            // through reconstructed Cartesian sweep sides can needlessly
            // exhaust the predicate budget for algebraic line/circle roots.
            let source_parameters = match policy
                .strict_predicate_pass(|| source.retained_circle_point_parameters(&hit, policy))?
            {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(_) => {
                    // The retained-circle inverse is an accelerator.  A hit
                    // at the quadratic chart's omitted projective point can
                    // make that inverse undecidable.  The retained arc sweep
                    // can still reject that exact circle point before the
                    // ordinary Bernstein line-contact fallback is needed.
                    if arc.contains_sweep_point(&hit, policy) == Classification::Decided(false) {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-retained-circle",
                            "omitted-projective-point-outside-sweep",
                        );
                        continue;
                    }
                    return Ok(None);
                }
            };
            if source_parameters.is_empty() {
                continue;
            }
            let point = CurvePoint2::from(hit.clone());
            let chord_parameter = match self.parameter_at_certified_point(point.clone(), policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let (radial_x, radial_y) = hit.delta_from(arc.center());
            let (source_dx, source_dy) = if arc.is_clockwise() {
                (radial_y, -radial_x)
            } else {
                (-radial_y, radial_x)
            };
            let tangent_cross =
                Real::diff_of_products(&chord_dx, &source_dy, &chord_dy, &source_dx);
            let Some(tangent_cross_sign) = real_sign(&tangent_cross, policy) else {
                return Ok(Some(Classification::Uncertain(UncertaintyReason::RealSign)));
            };
            for source_parameter in source_parameters {
                let source_parameter = CurveParameter2::from(source_parameter);
                if let Some(excluded) = excluded_source_parameter {
                    match source_parameter.cmp_by_refinement(excluded, policy)? {
                        Classification::Decided(std::cmp::Ordering::Equal) => continue,
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    }
                }
                contacts.push(BezierAlgebraicChordRationalContact2 {
                    chord_parameter: chord_parameter.clone(),
                    other_parameter: source_parameter,
                    point: point.clone(),
                    tangent_cross_sign,
                });
            }
        }
        Ok(Some(Classification::Decided(
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts),
        )))
    }

    /// Reuses the selected circle/chord map's own recursive frame when this
    /// support was authored directly between that circle's center and the
    /// retained contact.  The structural center match is only a sufficient
    /// provenance certificate: unrelated but geometrically equal points fall
    /// through to the complete projective importer below.
    pub(in crate::bezier_offset) fn recursive_cusp_contact_frame_endpoints(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<[BezierRecursiveQuadraticProjectivePoint2; 2]>> {
        for (contact_endpoint, center_endpoint, contact_at_start) in [
            (self.start(), self.end(), true),
            (self.end(), self.start(), false),
        ] {
            let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = contact_endpoint else {
                continue;
            };
            let (map, contact) = point.map_contact();
            let center = match map.data.semicircle.center_point_evidence(policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(_) => continue,
            };
            let same_center = if center_endpoint == &center {
                true
            } else if let (
                CurvePoint2(CurvePointData2::Algebraic(first)),
                CurvePoint2(CurvePointData2::Algebraic(second)),
            ) = (center_endpoint, &center)
            {
                match policy
                    .strict_predicate_pass(|| first.same_retained_rational_point(second, policy))?
                {
                    Some(Classification::Decided(equal)) => equal,
                    Some(Classification::Uncertain(_)) | None => matches!(
                        policy
                            .strict_predicate_pass(|| center_endpoint.same_point(&center, policy)),
                        Classification::Decided(true)
                    ),
                }
            } else {
                false
            };
            if !same_center {
                continue;
            }
            let frame = match map.recursive_contact_frame(contact, policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-chord-endpoints",
                "cusp-contact-center-frame",
            );
            return Ok(Some(if contact_at_start {
                [frame.point, frame.center]
            } else {
                [frame.center, frame.point]
            }));
        }
        Ok(None)
    }

    /// Imports both finite endpoints together with the least-field direction
    /// witnesses for their affine support.  A procedural displacement shares
    /// one normalized translation between its endpoints, so subtracting the
    /// displaced points would only introduce and then cancel that radical.
    /// Retaining the source direction keeps support incidence small while the
    /// displaced points remain authoritative for finite-domain clipping.
    #[track_caller]
    pub(in crate::bezier_offset) fn recursive_projective_endpoints_with_direction(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParallelRecursiveFrame2>>> {
        if let (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
        ) = (self.start(), self.end())
            && start.shares_carrier(end)
            && start.at_end != end.at_end
            && start.data.source_point.is_none()
        {
            let frame = match start.recursive_projective_frame(policy)? {
                Classification::Decided(Some(frame)) => frame,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let indices = [usize::from(start.at_end), usize::from(end.at_end)];
            return Ok(Classification::Decided(Some(
                BezierAlgebraicChordParallelRecursiveFrame2 {
                    displaced: indices.map(|index| frame.displaced[index].clone()),
                    direction_endpoints: indices
                        .map(|index| frame.direction_endpoints[index].clone()),
                },
            )));
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (self.start(), self.end())
        {
            let caller = std::panic::Location::caller();
            eprintln!(
                "recursive parallel frame fallback caller={}:{} carrier={} normal-carrier={} ends=({},{}) origins=({},{}) sources={} directions=({:?},{:?}) distances={:?} translations={} policies={}",
                caller.file(),
                caller.line(),
                start.shares_carrier(end),
                start.shares_normal_offset_carrier(end),
                start.at_end,
                end.at_end,
                start.data.source_point.is_some(),
                end.data.source_point.is_some(),
                start.data.source == end.data.source,
                start.data.direction,
                end.data.direction,
                compare_reals(
                    &start.data.distance,
                    &end.data.distance,
                    &CurveContext::STRICT
                ),
                start.data.translation_x == end.data.translation_x
                    && start.data.translation_y == end.data.translation_y,
                start.data.policy == end.data.policy,
            );
            let kind = |point: &CurvePoint2| match point {
                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    "similarity"
                }
            };
            let start_direction = start.source_direction_endpoints(policy);
            let end_direction = end.source_direction_endpoints(policy);
            eprintln!(
                "recursive parallel frame fallback origins-kind=({},{}) origins-storage={} origins-equal={} direction-kinds=(({},{}),({},{})) source-axes=({:?},{:?})",
                kind(start.source_endpoint()),
                kind(end.source_endpoint()),
                start
                    .source_endpoint()
                    .shares_storage(end.source_endpoint()),
                start.source_endpoint() == end.source_endpoint(),
                kind(start_direction[0]),
                kind(start_direction[1]),
                kind(end_direction[0]),
                kind(end_direction[1]),
                start.data.source.data.parameter_axis,
                end.data.source.data.parameter_axis,
            );
            if start
                .source_endpoint()
                .shares_storage(end.source_endpoint())
                && matches!(
                    start.source_endpoint(),
                    CurvePoint2(CurvePointData2::Exact(_))
                )
            {
                let status =
                    |point: &CurvePoint2| match recursive_projective_point_source(point, policy) {
                        Ok(Classification::Decided(Some(_))) => "decided",
                        Ok(Classification::Decided(None)) => "none",
                        Ok(Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                            "unsupported"
                        }
                        Ok(Classification::Uncertain(_)) => "uncertain",
                        Err(_) => "error",
                    };
                eprintln!(
                    "recursive parallel frame direction-status=(({},{}),({},{}))",
                    status(start_direction[0]),
                    status(start_direction[1]),
                    status(end_direction[0]),
                    status(end_direction[1]),
                );
            }
        }
        Ok(self.recursive_projective_endpoints(policy)?.map(|points| {
            points.map(|displaced| BezierAlgebraicChordParallelRecursiveFrame2 {
                direction_endpoints: displaced.clone(),
                displaced,
            })
        }))
    }

    /// Imports both authored endpoints into one positively normalized
    /// recursive projective frame. Rational and analytic-parallel targets
    /// share this exact construction boundary; target-specific kernels add
    /// only their own parameter axis and procedural radicals afterward.
    #[track_caller]
    pub(in crate::bezier_offset) fn recursive_projective_endpoints(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[BezierRecursiveQuadraticProjectivePoint2; 2]>>> {
        let ([start, end], [start_sign, end_sign]) = if let Some(points) =
            self.recursive_cusp_contact_frame_endpoints(policy)?
        {
            (points, [RealSign::Positive; 2])
        } else {
            let evidence = [self.start(), self.end()];
            let mut denominator_signs = Vec::with_capacity(2);
            for point in evidence {
                match recursive_projective_evidence_denominator_sign(point, policy)? {
                    Classification::Decided(sign) => denominator_signs.push(sign),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                let caller = std::panic::Location::caller();
                let kind = |point: &CurvePoint2| match point {
                    CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                    CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
                    CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
                    CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
                    CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                        "similarity"
                    }
                };
                eprintln!(
                    "recursive chord endpoints caller={}:{} kinds=({},{}) selects-approximate={} permits-approximate={}",
                    caller.file(),
                    caller.line(),
                    kind(evidence[0]),
                    kind(evidence[1]),
                    policy.selects_approximate_512(),
                    policy.permits_approximate_512(),
                );
            }
            let recursive = if policy.selects_approximate_512() {
                // Exact coordinate-field selection must not consume the
                // terminal policy.  This also lets the bounded recursive
                // importer decline a divergent primitive compositum.
                policy.strict_predicate_pass(|| {
                    recursive_projective_evidence_points(&evidence, policy)
                })
            } else {
                recursive_projective_evidence_points(&evidence, policy)
            }?;
            let recursive = if policy.permits_approximate_512()
                && !matches!(&recursive, Classification::Decided(Some(_)))
            {
                // The endpoint tuple is still exact: import its selected
                // coordinate witnesses into a flat dense base.  Run this
                // bridge with approximation suppressed so no terminal
                // equality can select a persistent representation.
                policy.strict_predicate_pass(|| {
                    represented_projective_evidence_points(&evidence, policy)
                })?
            } else {
                recursive
            };
            let points = match recursive {
                Classification::Decided(Some(points)) => points,
                Classification::Decided(None) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord endpoints stage=evidence-unavailable");
                    }
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                        eprintln!("recursive chord endpoints stage=evidence reason={reason:?}");
                    }
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (
                points
                    .try_into()
                    .expect("a recursive chord support retains two authored endpoints"),
                denominator_signs
                    .try_into()
                    .expect("a recursive chord support retains two denominator signs"),
            )
        };
        let start = orient_recursive_projective_point_positive(start, start_sign)?;
        let end = orient_recursive_projective_point_positive(end, end_sign)?;
        if !start
            .denominator
            .field()
            .same_field(&end.denominator.field())
        {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                eprintln!("recursive chord endpoints stage=field-mismatch");
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Ok(Classification::Decided(Some([start, end])))
    }
}
