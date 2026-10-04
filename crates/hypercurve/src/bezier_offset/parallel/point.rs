//! Exact points on analytic parallels: evaluation, coordinates, bounds,
//! equality and order against other retained point evidence.

use super::*;

impl BezierAnalyticParallelPoint2 {
    /// The original source evaluation, without a replacement tangent frame,
    /// tangent displacement or translation. Only certified constructions can
    /// supply identities that will later be replayed under a strict policy.
    pub(in crate::bezier_offset) fn native_parallel_evaluation(
        &self,
    ) -> Option<(&BezierParallel2, CurveParameter2)> {
        if !CurveContext::STRICT.accepts_retained_policy(self.data.policy)
            || self.data.frame_tangent.is_some()
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return None;
        }
        Some((&self.data.parallel, self.data.parameter.curve_parameter()))
    }

    /// Classifies this point against an exact analytic tangent segment without
    /// materializing either tangent endpoint.
    ///
    /// If `Q` is the tangent parameter and `P` is this point, the oriented
    /// side is the sign of `cross(H(Q), P-Q)`, adjusted by the signed tangent
    /// displacement that orients the segment.  Independently refined exact
    /// enclosures preserve both retained parameter sheets and are normally
    /// far smaller than joining their recursive Cartesian point towers.
    pub(in crate::bezier_offset) fn oriented_side_to_analytic_tangent_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Ok(None);
        }
        let support = chord.retained_support();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (support.start(), support.end())
        else {
            return Ok(None);
        };
        if start.data.parallel != end.data.parallel
            || start.data.parameter != end.data.parameter
            || start.data.frame_tangent != end.data.frame_tangent
            || start.data.translation_x != end.data.translation_x
            || start.data.translation_y != end.data.translation_y
            || !policy.accepts_retained_policy(start.data.policy)
            || !policy.accepts_retained_policy(end.data.policy)
        {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "analytic tangent structural rejection endpoints=({},{},{},{},{}) point=(true,{},{},{},{}) policies=({},{}) tangent=({:?},{:?},{:?})",
                    start.data.parallel == end.data.parallel,
                    start.data.parameter == end.data.parameter,
                    start.data.frame_tangent == end.data.frame_tangent,
                    start.data.translation_x == end.data.translation_x,
                    start.data.translation_y == end.data.translation_y,
                    self.data.frame_tangent == start.data.frame_tangent,
                    self.data.tangent_distance == start.data.tangent_distance,
                    self.data.translation_x == start.data.translation_x,
                    self.data.translation_y == start.data.translation_y,
                    policy.accepts_retained_policy(start.data.policy),
                    policy.accepts_retained_policy(end.data.policy),
                    self.data.tangent_distance.zero_status(),
                    start.data.tangent_distance.zero_status(),
                    real_sign(
                        &(&self.data.tangent_distance - &start.data.tangent_distance),
                        &CurveContext::STRICT,
                    ),
                );
            }
            return Ok(None);
        }
        let tangent_displacement = &end.data.tangent_distance - &start.data.tangent_distance;
        let Some(tangent_displacement_sign @ (RealSign::Positive | RealSign::Negative)) =
            real_sign(&tangent_displacement, &CurveContext::STRICT)
        else {
            return Ok(None);
        };
        if self == start {
            return Ok(Some(crate::classify::LineSide::On));
        }
        let (point_tangent_x_coefficients, point_tangent_y_coefficients) =
            self.frame_tangent_power_basis()?;
        let (tangent_x_coefficients, tangent_y_coefficients) = start.frame_tangent_power_basis()?;
        let point_source = self.data.parallel.source_power_basis()?;
        let tangent_source = start.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let point_weight_coefficients = point_source.weight.unwrap_or(&unit_weight);
        let tangent_weight_coefficients = tangent_source.weight.unwrap_or(&unit_weight);
        let strict = &CurveContext::STRICT;
        let parameter_interval = |parameter: &BezierAnalyticParallelPointParameter2,
                                  refinement_steps|
         -> CurveResult<Option<RealInterval>> {
            Ok(Some(match parameter {
                BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                    let parameter = parameter
                        .clone()
                        .refined_isolating_interval(refinement_steps, policy);
                    real_interval_from_parameter(&parameter)
                }
                BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                    let parameter = match parameter.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    RealInterval {
                        lower: parameter.root().lower.clone(),
                        upper: parameter.root().upper.clone(),
                    }
                }
                BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                    let parameter = match parameter.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(None),
                    };
                    let (lower, upper) = parameter.isolating_bounds();
                    RealInterval {
                        lower: lower.clone(),
                        upper: upper.clone(),
                    }
                }
            }))
        };
        let oriented_side = |mut sign| {
            sign = product_sign(sign, tangent_displacement_sign);
            if chord.retained_support_orientation_is_reversed() {
                sign = product_sign(sign, RealSign::Negative);
            }
            crate::classify::LineSide::from_real_sign(sign)
        };
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let Some(parameter) = parameter_interval(&start.data.parameter, refinement_steps)?
            else {
                continue;
            };
            let Some(point_parameter) = parameter_interval(&self.data.parameter, refinement_steps)?
            else {
                continue;
            };
            let evaluate = |coefficients: &[Real], parameter| {
                RealInterval::evaluate_power_basis(coefficients, parameter)
            };
            let correlated_incidence = (|| {
                macro_rules! interval_or_none {
                    ($stage:literal, $value:expr) => {{
                        let value = $value;
                        #[cfg(test)]
                        if value.is_none()
                            && refinement_steps == 512
                            && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
                        {
                            eprintln!("same-parallel correlated failure={}", $stage);
                        }
                        value?
                    }};
                }
                let point_tangent_x = interval_or_none!(
                    "point-tangent-x",
                    evaluate(point_tangent_x_coefficients, &point_parameter)
                );
                let point_tangent_y = interval_or_none!(
                    "point-tangent-y",
                    evaluate(point_tangent_y_coefficients, &point_parameter)
                );
                let tangent_x =
                    interval_or_none!("tangent-x", evaluate(tangent_x_coefficients, &parameter));
                let tangent_y =
                    interval_or_none!("tangent-y", evaluate(tangent_y_coefficients, &parameter));
                let point_speed_squared =
                    interval_or_none!("point-speed-x-square", point_tangent_x.square()).add(
                        &interval_or_none!("point-speed-y-square", point_tangent_y.square()),
                    );
                let point_speed = interval_or_none!(
                    "point-speed-root",
                    point_speed_squared.nonnegative_square_root(None)
                );
                let tangent_speed_squared =
                    interval_or_none!("tangent-speed-x-square", tangent_x.square()).add(
                        &interval_or_none!("tangent-speed-y-square", tangent_y.square()),
                    );
                let tangent_speed = interval_or_none!(
                    "tangent-speed-root",
                    tangent_speed_squared.nonnegative_square_root(None)
                );
                let point_weight = interval_or_none!(
                    "point-weight",
                    evaluate(point_weight_coefficients, &point_parameter)
                );
                let tangent_weight = interval_or_none!(
                    "tangent-weight",
                    evaluate(tangent_weight_coefficients, &parameter)
                );
                let point_x_numerator = interval_or_none!(
                    "point-x-numerator",
                    evaluate(point_source.x_numerator, &point_parameter)
                );
                let point_x =
                    interval_or_none!("point-x-divide", point_x_numerator.divide(&point_weight));
                let point_y_numerator = interval_or_none!(
                    "point-y-numerator",
                    evaluate(point_source.y_numerator, &point_parameter)
                );
                let point_y =
                    interval_or_none!("point-y-divide", point_y_numerator.divide(&point_weight));
                let tangent_x_numerator = interval_or_none!(
                    "tangent-x-numerator",
                    evaluate(tangent_source.x_numerator, &parameter)
                );
                let tangent_x_coordinate = interval_or_none!(
                    "tangent-x-divide",
                    tangent_x_numerator.divide(&tangent_weight)
                );
                let tangent_y_numerator = interval_or_none!(
                    "tangent-y-numerator",
                    evaluate(tangent_source.y_numerator, &parameter)
                );
                let tangent_y_coordinate = interval_or_none!(
                    "tangent-y-divide",
                    tangent_y_numerator.divide(&tangent_weight)
                );
                let delta_x = point_x.subtract(&tangent_x_coordinate);
                let delta_y = point_y.subtract(&tangent_y_coordinate);
                let source_incidence =
                    interval_or_none!("source-incidence-x", tangent_x.multiply(&delta_y)).subtract(
                        &interval_or_none!("source-incidence-y", tangent_y.multiply(&delta_x)),
                    );
                let tangent_cross =
                    interval_or_none!("tangent-cross-x", tangent_x.multiply(&point_tangent_y))
                        .subtract(&interval_or_none!(
                            "tangent-cross-y",
                            tangent_y.multiply(&point_tangent_x)
                        ));
                let tangent_dot =
                    interval_or_none!("tangent-dot-x", tangent_x.multiply(&point_tangent_x)).add(
                        &interval_or_none!("tangent-dot-y", tangent_y.multiply(&point_tangent_y)),
                    );
                let speed_product =
                    interval_or_none!("speed-product", tangent_speed.multiply(&point_speed));
                let dot_plus_speed = tangent_dot.add(&speed_product);
                let normal_difference =
                    if compare_reals(&dot_plus_speed.lower, &Real::zero(), strict)
                        == Some(std::cmp::Ordering::Greater)
                    {
                        let squared_cross =
                            interval_or_none!("cross-square", tangent_cross.square());
                        let denominator = interval_or_none!(
                            "normal-denominator",
                            dot_plus_speed.multiply(&point_speed)
                        );
                        let quotient = interval_or_none!(
                            "normal-quotient",
                            squared_cross.divide(&denominator)
                        );
                        RealInterval {
                            lower: -quotient.upper,
                            upper: -quotient.lower,
                        }
                    } else {
                        interval_or_none!("normal-direct-divide", tangent_dot.divide(&point_speed))
                            .subtract(&tangent_speed)
                    };
                let exact = |value: &Real| RealInterval {
                    lower: value.clone(),
                    upper: value.clone(),
                };
                let normal_difference = interval_or_none!(
                    "normal-scale",
                    normal_difference.multiply(&exact(self.data.parallel.distance()))
                );
                let normal_distance_delta =
                    self.data.parallel.distance() - start.data.parallel.distance();
                let normal_delta = interval_or_none!(
                    "normal-delta-scale",
                    tangent_speed.multiply(&exact(&normal_distance_delta))
                );
                let tangent_over_speed =
                    interval_or_none!("tangent-divide", tangent_cross.divide(&point_speed));
                let tangent = interval_or_none!(
                    "tangent-scale",
                    tangent_over_speed.multiply(&exact(&self.data.tangent_distance))
                );
                let translation_x = &self.data.translation_x - &start.data.translation_x;
                let translation_y = &self.data.translation_y - &start.data.translation_y;
                let translation =
                    interval_or_none!("translation-x", tangent_x.multiply(&exact(&translation_y)))
                        .subtract(&interval_or_none!(
                            "translation-y",
                            tangent_y.multiply(&exact(&translation_x))
                        ));
                Some(
                    source_incidence
                        .add(&normal_difference)
                        .add(&normal_delta)
                        .add(&tangent)
                        .add(&translation),
                )
            })();
            #[cfg(test)]
            if refinement_steps == 512
                && std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            {
                eprintln!(
                    "same-parallel correlated incidence={:?}",
                    correlated_incidence.as_ref().map(|incidence| (
                        incidence.lower.to_f64_lossy(),
                        incidence.upper.to_f64_lossy(),
                        compare_reals(&incidence.lower, &Real::zero(), strict),
                        compare_reals(&incidence.upper, &Real::zero(), strict),
                    )),
                );
            }
            if let Some(incidence) = correlated_incidence {
                let sign = if compare_reals(&incidence.lower, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Greater)
                {
                    Some(RealSign::Positive)
                } else if compare_reals(&incidence.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Less)
                {
                    Some(RealSign::Negative)
                } else {
                    None
                };
                if let Some(sign) = sign {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "same-parallel-analytic-tangent-correlated-interval",
                    );
                    return Ok(Some(oriented_side(sign)));
                }
            }
            let (Classification::Decided(point), Classification::Decided(origin)) = (
                self.conservative_bounds_refined(refinement_steps, policy),
                start.conservative_bounds_refined(refinement_steps, policy),
            ) else {
                continue;
            };
            let Some(tangent_x) =
                RealInterval::evaluate_power_basis(tangent_x_coefficients, &parameter)
            else {
                continue;
            };
            let Some(tangent_y) =
                RealInterval::evaluate_power_basis(tangent_y_coefficients, &parameter)
            else {
                continue;
            };
            let delta_x = real_interval_from_axis(&point, Axis2::X)
                .subtract(&real_interval_from_axis(&origin, Axis2::X));
            let delta_y = real_interval_from_axis(&point, Axis2::Y)
                .subtract(&real_interval_from_axis(&origin, Axis2::Y));
            let Some(cross) = tangent_x.multiply(&delta_y).and_then(|first| {
                tangent_y
                    .multiply(&delta_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            let mut sign = if compare_reals(&cross.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(RealSign::Positive)
            } else if compare_reals(&cross.upper, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Less)
            {
                Some(RealSign::Negative)
            } else {
                None
            };
            if let Some(sign) = sign.take() {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "same-parallel-analytic-tangent-interval",
                );
                return Ok(Some(oriented_side(sign)));
            }
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            let bounds = |parameter: &BezierAnalyticParallelPointParameter2| {
                let (lower, upper) = match parameter {
                    BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                        let parameter = parameter.clone().refined_isolating_interval(512, policy);
                        let interval = real_interval_from_parameter(&parameter);
                        (interval.lower, interval.upper)
                    }
                    BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                        let Classification::Decided(parameter) =
                            parameter.refined(512, policy).ok()?
                        else {
                            return None;
                        };
                        (
                            parameter.root().lower.clone(),
                            parameter.root().upper.clone(),
                        )
                    }
                    BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                        let Classification::Decided(parameter) =
                            parameter.refined(512, policy).ok()?
                        else {
                            return None;
                        };
                        let (lower, upper) = parameter.isolating_bounds();
                        (lower.clone(), upper.clone())
                    }
                };
                Some((lower.to_f64_lossy(), upper.to_f64_lossy()))
            };
            eprintln!(
                "same-parallel tangent interval unresolved point={:?} tangent={:?} distance={:?}",
                bounds(&self.data.parameter),
                bounds(&start.data.parameter),
                self.data.parallel.distance().to_f64_lossy(),
            );
        }
        Ok(None)
    }

    /// Signs an authored chord/rational contact after both carriers receive
    /// equal-magnitude left-normal offsets.
    ///
    /// At the retained source contact `P`, let `u` be the chord unit tangent
    /// and `v` the analytic unit tangent. The displaced point and displaced
    /// chord have incidence
    ///
    /// `cross(u, (P + dp left(v)) - (P + dc left(u)))`
    /// `= dp dot(u, v) - dc`.
    ///
    /// A retained nonzero tangent cross proves `-1 < dot(u, v) < 1`. When
    /// `dc = ±dp`, the complete incidence therefore has sign `-dc`. This
    /// covers the opposite distance convention introduced by a reversed
    /// boundary traversal without constructing a coordinate, speed radical,
    /// or projected incidence polynomial.
    pub(in crate::bezier_offset) fn equal_normal_offset_contact_oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return Ok(None);
        };
        let Some(BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(identity)) =
            parameter.data.identity.as_deref()
        else {
            return Ok(None);
        };
        let BezierParallelSource2::Rational(source) = self.data.parallel.source() else {
            return Ok(None);
        };
        let Some(support) = chord_parallel_support_source(chord, policy)? else {
            return Ok(None);
        };
        let parallel_distance = self.data.parallel.distance();
        let same_distance =
            compare_reals(&support.distance, parallel_distance, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal);
        let opposite_distance = compare_reals(
            &support.distance,
            &(-parallel_distance.clone()),
            &CurveContext::STRICT,
        ) == Some(std::cmp::Ordering::Equal);
        if source != &identity.source
            || identity.tangent_cross_sign == RealSign::Zero
            || support.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
            || !identity.chord.shares_retained_support(&support.source)
            || (!same_distance && !opposite_distance)
            || compare_reals(
                &support.translation_x,
                &self.data.translation_x,
                &CurveContext::STRICT,
            ) != Some(std::cmp::Ordering::Equal)
            || compare_reals(
                &support.translation_y,
                &self.data.translation_y,
                &CurveContext::STRICT,
            ) != Some(std::cmp::Ordering::Equal)
        {
            return Ok(None);
        }
        let Some(reversed) = support.source.shared_tangent_orientation(chord) else {
            return Ok(None);
        };
        let Some(mut sign) = real_sign(&support.distance, &CurveContext::STRICT) else {
            return Ok(None);
        };
        sign = product_sign(sign, RealSign::Negative);
        if reversed {
            sign = product_sign(sign, RealSign::Negative);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-side-kernel",
            "equal-normal-offset-contact",
        );
        Ok(Some(crate::classify::LineSide::from_real_sign(sign)))
    }

    /// Returns whether this zero-displacement point is the retained root of
    /// the exact incidence between `chord` and its analytic parallel.  The
    /// monotone parameter owns that construction equation, so replaying it as
    /// endpoint boxes would only rediscover an authored zero.
    pub(in crate::bezier_offset) fn certifies_monotone_chord_incidence(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(false);
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return Ok(false);
        };
        let Some(authority) = parameter.monotone_authority() else {
            return Ok(false);
        };
        if authority.side_parallel != self.data.parallel {
            return Ok(false);
        }
        if authority.side_chord.shares_retained_support(chord) {
            return Ok(true);
        }
        Ok(false)
    }

    /// Classifies this retained parameter's point against another chord using
    /// its unsquared incidence equation and local parameter evidence. Native
    /// field replay precedes deep independent interval refinement.
    pub(in crate::bezier_offset) fn retained_parameter_oriented_side_to_chord(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        if let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
            && parameter
                .monotone_authority()
                .is_some_and(|authority| authority.side_parallel != self.data.parallel)
        {
            return Ok(None);
        }
        if let Some(direction) = chord.certified_axis_direction() {
            let point = CurvePoint2::from(self.clone());
            if let Some(Classification::Decided(side)) =
                chord.axis_oriented_side(&point, direction, policy)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "monotone-axis-coordinate",
                );
                return Ok(Some(side));
            }
        }
        if let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
            && let Some(authority) = parameter.monotone_authority()
        {
            // The retained point lies on `authority.side_chord`. If that
            // authored support and the query share an endpoint, its side is
            // purely the signed tangent cross times the point's exact support
            // displacement from that endpoint. This is the affine
            // line identity
            //
            //   cross(Q, P - E) = lambda * cross(Q, A),
            //
            // and avoids rebuilding the query's independently retained
            // endpoint fields merely to replay the same incidence.
            let query = chord.retained_support();
            let authored = authority.side_chord.retained_support();
            let shared_endpoint = [query.start(), query.end()]
                .into_iter()
                .enumerate()
                .find_map(|(query_at_end, query_point)| {
                    [authored.start(), authored.end()]
                        .into_iter()
                        .enumerate()
                        .find_map(|(authored_at_end, authored_point)| {
                            (query_point.shares_storage(authored_point)
                                || query_point == authored_point)
                                .then_some((query_at_end != 0, authored_at_end != 0))
                        })
                });
            if let Some((query_at_end, authored_at_end)) = shared_endpoint {
                let point = CurvePoint2::from(self.clone());
                let point_parameter =
                    authored.parameter_at_certified_support_point(point, policy)?;
                let endpoint_parameter = if authored_at_end {
                    authored.end_parameter()
                } else {
                    authored.start_parameter()
                };
                let displacement_order = policy.strict_predicate_pass(|| {
                    point_parameter.cmp_by_refinement(&endpoint_parameter, policy)
                })?;
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                    eprintln!(
                        "monotone shared endpoint displacement query-end={query_at_end} authored-end={authored_at_end} order={displacement_order:?}"
                    );
                }
                let displacement_sign = match displacement_order {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        return Ok(Some(crate::classify::LineSide::On));
                    }
                    Classification::Decided(std::cmp::Ordering::Less) => Some(RealSign::Negative),
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        Some(RealSign::Positive)
                    }
                    Classification::Uncertain(_) => None,
                };
                if let Some(displacement_sign) = displacement_sign {
                    let query_other = if query_at_end {
                        query.start()
                    } else {
                        query.end()
                    };
                    let procedural_cross = policy.strict_predicate_pass(|| {
                        authored.retained_procedural_point_side(query_other, policy)
                    })?;
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("monotone shared endpoint procedural side={procedural_cross:?}");
                    }
                    let cross = if let Some(side) = procedural_cross {
                        let side_sign = match side {
                            crate::classify::LineSide::Left => RealSign::Positive,
                            crate::classify::LineSide::On => RealSign::Zero,
                            crate::classify::LineSide::Right => RealSign::Negative,
                        };
                        // `side_sign = cross(A, Q_other - E)`. If E is the
                        // query start then `Q = Q_other - E`; if it is the
                        // query end then `Q = E - Q_other`.
                        Classification::Decided(if query_at_end {
                            side_sign
                        } else {
                            product_sign(side_sign, RealSign::Negative)
                        })
                    } else {
                        match query.tangent_cross_sign_with_shared_endpoint(authored, policy) {
                            Some(cross) => policy.strict_predicate_pass(|| cross)?,
                            None => policy.strict_predicate_pass(|| {
                                query.tangent_cross_sign(authored, policy)
                            })?,
                        }
                    };
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("monotone shared endpoint cross={cross:?}");
                    }
                    if let Classification::Decided(cross) = cross {
                        let mut side = crate::classify::LineSide::from_real_sign(product_sign(
                            cross,
                            displacement_sign,
                        ));
                        if chord.retained_support_orientation_is_reversed() {
                            side = match side {
                                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                                crate::classify::LineSide::On => crate::classify::LineSide::On,
                            };
                        }
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-side-kernel",
                            "monotone-shared-endpoint-affine-sign",
                        );
                        return Ok(Some(side));
                    }
                }
            }
        }
        let support = chord.retained_support();
        let system = match policy.strict_predicate_pass(|| {
            support.recursive_projective_parallel_system_with_frame(
                &self.data.parallel,
                self.data.frame_tangent.as_deref(),
                false,
                policy,
            )
        })? {
            Classification::Decided(Some(system)) => system,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let to_side = |mut sign, _lane: &'static str| {
            if chord.retained_support_orientation_is_reversed() {
                sign = product_sign(sign, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record("hypercurve", "algebraic-chord-side-kernel", _lane);
            crate::classify::LineSide::from_real_sign(sign)
        };
        if let BezierAnalyticParallelPointParameter2::Bezier(parameter) = &self.data.parameter {
            let evaluation = match policy
                .strict_predicate_pass(|| system.candidate_evaluation(parameter, policy))?
            {
                Classification::Decided(Some(evaluation)) => evaluation,
                Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
            };
            let sign = match policy.strict_predicate_pass(|| {
                system.expression_sign(&system.incidence, &evaluation, policy)
            })? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(_) => return Ok(None),
            };
            return Ok(Some(to_side(sign, "retained-bezier-incidence-sign")));
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return Ok(None);
        };
        let native_side = || -> CurveResult<Option<crate::classify::LineSide>> {
            Ok(
                match policy.strict_predicate_pass(|| {
                    system.incidence_sign_at_recursive_parameter(parameter, policy)
                })? {
                    Classification::Decided(sign) => {
                        Some(to_side(sign, "retained-recursive-incidence-sign"))
                    }
                    Classification::Uncertain(_) => None,
                },
            )
        };
        let mut terminal_refined = false;
        let mut refined = parameter.clone();
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if refinement_steps == 16 {
                if let Some(side) = policy.bounded_exact_predicate_pass(native_side)? {
                    return Ok(Some(side));
                }
                if policy.has_bounded_exact_predicate_budget() {
                    return Ok(None);
                }
            }
            refined =
                match policy.strict_predicate_pass(|| refined.refined(refinement_steps, policy))? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => continue,
                };
            terminal_refined |= refinement_steps == 512;
            let parameter_interval = RealInterval {
                lower: refined.data.lower.clone(),
                upper: refined.data.upper.clone(),
            };
            let coefficient_bits = refinement_steps.max(64).min(i32::MAX as usize) as i32;
            let Some(sign) = system.oriented_incidence_interval_sign(
                &parameter_interval,
                refinement_steps,
                -coefficient_bits,
            ) else {
                continue;
            };
            return Ok(Some(to_side(sign, "retained-monotone-incidence-interval")));
        }
        if let Some(side) = native_side()? {
            return Ok(Some(side));
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Some(crate::classify::LineSide::On));
        }
        Ok(None)
    }

    /// Returns the orientation of a displacement in one retained tangent
    /// frame. Different anchors or normal sheets cannot share this proof.
    pub(in crate::bezier_offset) fn shared_tangent_displacement_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<Classification<RealSign>> {
        if self.data.parallel != other.data.parallel
            || self.data.parameter != other.data.parameter
            || self.data.frame_tangent != other.data.frame_tangent
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
            || !policy.accepts_retained_policy(self.data.policy)
            || !policy.accepts_retained_policy(other.data.policy)
        {
            return None;
        }
        let displacement = &other.data.tangent_distance - &self.data.tangent_distance;
        match real_sign(&displacement, &CurveContext::STRICT) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => {
                Some(Classification::Decided(sign))
            }
            Some(RealSign::Zero) => None,
            None => Some(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    /// Signs a linear form of the exact displacement from `self` to `other`
    /// when both points were authored in one retained unit-tangent frame.
    ///
    /// Their source, normal offset, parameter, frame, and translation cancel
    /// structurally. Only the signed tangent-distance difference and one
    /// source-tangent polynomial remain, so a mixed represented/recursive
    /// chord relation never needs four Cartesian endpoint enclosures.
    pub(in crate::bezier_offset) fn shared_tangent_displacement_linear_form_sign(
        &self,
        other: &Self,
        coefficient_x: &Real,
        coefficient_y: &Real,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let displacement_sign = match self.shared_tangent_displacement_sign(other, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Some(Ok(Classification::Uncertain(reason)));
            }
        };
        let (tangent_x, tangent_y) = match self.frame_tangent_power_basis() {
            Ok(tangent) => tangent,
            Err(error) => return Some(Err(error)),
        };
        let polynomial = polynomial_add(
            &polynomial_scale(tangent_x, coefficient_x),
            &polynomial_scale(tangent_y, coefficient_y),
        );
        let sign = self.parameter_polynomial_sign(&polynomial, policy);
        Some(
            sign.map(|classification| {
                classification.map(|sign| product_sign(displacement_sign, sign))
            }),
        )
    }

    pub(in crate::bezier_offset) fn shared_rational_tangent_relation_sign_to_chord(
        &self,
        other: &Self,
        chord: &BezierAlgebraicChord2,
        cross: bool,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let shared_frame = self.data.parallel == other.data.parallel
            && self.data.parameter == other.data.parameter
            && self.data.frame_tangent == other.data.frame_tangent
            && self.data.translation_x == other.data.translation_x
            && self.data.translation_y == other.data.translation_y;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let parameter = match &self.data.parameter {
                BezierAnalyticParallelPointParameter2::Bezier(_) => "bezier",
                BezierAnalyticParallelPointParameter2::SelectedFiber(_) => "selected",
                BezierAnalyticParallelPointParameter2::RecursiveProjective(_) => "recursive",
            };
            let source = match self.data.parallel.source() {
                BezierParallelSource2::Quadratic(_) => "quadratic",
                BezierParallelSource2::Cubic(_) => "cubic",
                BezierParallelSource2::Rational(_) => "rational",
            };
            eprintln!(
                "retained rational tangent candidate shared={shared_frame} parameter={parameter} source={source}"
            );
        }
        if !shared_frame
            || self.data.parallel != other.data.parallel
            || self.data.parameter != other.data.parameter
            || self.data.frame_tangent != other.data.frame_tangent
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
        {
            return None;
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return None;
        };
        let BezierParallelSource2::Rational(source) = self.data.parallel.source() else {
            return None;
        };
        let displacement = &other.data.tangent_distance - &self.data.tangent_distance;
        let source_direction = match real_sign(&displacement, &CurveContext::STRICT) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) | None => return None,
        };
        let result = cross
            .then(|| {
                parameter.chord_rational_tangent_cross_sign(chord, source, source_direction, policy)
            })
            .flatten();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("retained rational tangent identity result={result:?}");
        }
        if result.is_some() {
            return result;
        }

        let support = chord.retained_support();
        let line = match support.recursive_projective_support_line(policy) {
            Ok(Some(line)) => line,
            Ok(None) => return None,
            Err(error) => return Some(Err(error)),
        };
        let (tangent_x, tangent_y) = match self.frame_tangent_power_basis() {
            Ok(tangent) => tangent,
            Err(error) => return Some(Err(error)),
        };
        let reversed = chord.retained_support_orientation_is_reversed();
        let orient = |sign| {
            let sign = product_sign(sign, source_direction);
            if reversed {
                product_sign(sign, RealSign::Negative)
            } else {
                sign
            }
        };

        // Exact and compact-witness line coefficients can be absorbed into
        // the ordinary hodograph polynomial immediately.  This covers exact
        // axes and other source-free supports without manufacturing a
        // foreign empty recursive base merely to join it to the contact.
        if let (Some(line_x), Some(line_y)) = (
            line.x.exact_real_value_with_retained_witnesses(),
            line.y.exact_real_value_with_retained_witnesses(),
        ) {
            let polynomial = if cross {
                polynomial_add(
                    &polynomial_scale(tangent_x, &line_x),
                    &polynomial_scale(tangent_y, &line_y),
                )
            } else {
                polynomial_subtract(
                    &polynomial_scale(tangent_x, &line_y),
                    &polynomial_scale(tangent_y, &line_x),
                )
            };
            let sign = match parameter.polynomial_sign(&polynomial, policy) {
                Ok(classification) => classification.map(orient),
                Err(error) => return Some(Err(error)),
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                eprintln!("retained exact-line tangent result={sign:?}");
            }
            return Some(Ok(sign));
        }

        // A projective contact already owns its selected scalar in the
        // recursive field.  Evaluate only the homogeneous hodograph there;
        // the tangent-line constant and its positive speed radical cannot
        // affect a direction cross product and would add a needless tower
        // generator before the field join.
        if parameter.projective_scalar().is_some() {
            let tangent_degree = tangent_x.len().max(tangent_y.len()).saturating_sub(1);
            let (Some(tangent_x), Some(tangent_y)) = (
                parameter.homogeneous_polynomial_value(tangent_x, tangent_degree),
                parameter.homogeneous_polynomial_value(tangent_y, tangent_degree),
            ) else {
                return None;
            };
            let tangent_field = tangent_x.field();
            let tangent = tangent_field.constant(Real::one()).map(|denominator| {
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: tangent_x,
                    y: tangent_y,
                    denominator,
                }
            })?;
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                let (tangent_base, tangent_extensions) =
                    tangent.denominator.field().base_and_extension_path();
                let (line_base, line_extensions) =
                    line.denominator.field().base_and_extension_path();
                eprintln!(
                    "retained projective tangent fields=({}+{},{}+{}) shared-base={} line-lifts={} tangent-lifts={}",
                    tangent_base.sources.len(),
                    tangent_extensions.len(),
                    line_base.sources.len(),
                    line_extensions.len(),
                    Arc::ptr_eq(&tangent_base, &line_base),
                    line.lifted_to(&tangent.denominator.field()).is_some(),
                    tangent.lifted_to(&line.denominator.field()).is_some(),
                );
            }
            let (_, line, tangent) = match line.joined_pair(&tangent, policy) {
                Ok(Classification::Decided(Some(joined))) => joined,
                Ok(Classification::Decided(None)) => return None,
                Ok(Classification::Uncertain(reason)) => {
                    return Some(Ok(Classification::Uncertain(reason)));
                }
                Err(error) => return Some(Err(error)),
            };
            let value = if cross {
                line.x
                    .multiply(&tangent.x)
                    .and_then(|x| line.y.multiply(&tangent.y).and_then(|y| x.add(&y)))
            } else {
                line.y
                    .multiply(&tangent.x)
                    .and_then(|x| line.x.multiply(&tangent.y).and_then(|y| x.subtract(&y)))
            };
            let value = value?;
            let sign = match value.sign(policy) {
                Ok(classification) => classification.map(orient),
                Err(error) => return Some(Err(error)),
            };
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                let (_, extensions) = value.field().base_and_extension_path();
                eprintln!(
                    "retained projective tangent result={sign:?} depth={}",
                    extensions.len(),
                );
            }
            return Some(Ok(sign));
        }

        // A local polynomial contact need not have been selected against the
        // chord currently querying its tangent.  Its hodograph still lives at
        // the same retained scalar, however, so `cross(chord, tangent)` is one
        // polynomial in that scalar with the chord-line coefficients in the
        // recursive coefficient field.  Replay that polynomial through the
        // selected root authority instead of promoting the contact to an
        // independent dense source axis.
        #[cfg(test)]
        let authority = parameter.polynomial_authority()?;
        let degree = tangent_x.len().max(tangent_y.len());
        let zero = Real::zero();
        let coefficients = (0..degree)
            .map(|index| {
                if cross {
                    line.x
                        .scale(tangent_x.get(index).unwrap_or(&zero))?
                        .add(&line.y.scale(tangent_y.get(index).unwrap_or(&zero))?)
                } else {
                    line.y
                        .scale(tangent_x.get(index).unwrap_or(&zero))?
                        .subtract(&line.x.scale(tangent_y.get(index).unwrap_or(&zero))?)
                }
            })
            .collect::<Option<Vec<_>>>()?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let (authority_base, authority_extensions) = authority.field.base_and_extension_path();
            let (line_base, line_extensions) = line.denominator.field().base_and_extension_path();
            eprintln!(
                "retained polynomial tangent fields=({}+{},{}+{}) shared-base={} query-lifts={} authority-lifts={}",
                authority_base.sources.len(),
                authority_extensions.len(),
                line_base.sources.len(),
                line_extensions.len(),
                Arc::ptr_eq(&authority_base, &line_base),
                coefficients
                    .iter()
                    .all(|coefficient| authority.field.lift(coefficient).is_some()),
                authority.coefficients.iter().all(|coefficient| line
                    .denominator
                    .field()
                    .lift(coefficient)
                    .is_some()),
            );
        }
        let mut sign = match parameter.recursive_polynomial_sign_joined(&coefficients, policy) {
            Ok(classification) => classification,
            Err(error) => return Some(Err(error)),
        };
        sign = sign.map(orient);
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("retained polynomial tangent result={sign:?}");
        }
        Some(Ok(sign))
    }

    /// Replaces this zero-distance finite-chord contact by the opposite
    /// authored endpoint when `shared_endpoint` is the other authored
    /// endpoint. The replacement preserves the contact-to-shared ray up to a
    /// strictly positive scale certified by the stored finite location.
    pub(in crate::bezier_offset) fn recursive_chord_collinear_support_endpoint<'a>(
        &'a self,
        shared_endpoint: &CurvePoint2,
        policy: &CurveContext,
    ) -> Option<&'a CurvePoint2> {
        if self.data.parallel.distance().zero_status() != ZeroKnowledge::Zero
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
            || !policy.accepts_retained_policy(self.data.policy)
        {
            return None;
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return None;
        };
        parameter.validate_policy(policy).ok()?;
        let BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(identity) =
            parameter.data.identity.as_deref()?
        else {
            return None;
        };
        let BezierParallelSource2::Rational(source) = self.data.parallel.source() else {
            return None;
        };
        if source != &identity.source {
            return None;
        }
        let shared_location = if shared_endpoint.shares_storage(identity.chord.start())
            || shared_endpoint == identity.chord.start()
        {
            BezierRecursiveChordContactLocation2::Start
        } else if shared_endpoint.shares_storage(identity.chord.end())
            || shared_endpoint == identity.chord.end()
        {
            BezierRecursiveChordContactLocation2::End
        } else {
            return None;
        };
        if shared_location == identity.chord_location {
            return None;
        }
        Some(match shared_location {
            BezierRecursiveChordContactLocation2::Start => identity.chord.end(),
            BezierRecursiveChordContactLocation2::End => identity.chord.start(),
            BezierRecursiveChordContactLocation2::Interior => unreachable!(),
        })
    }

    /// Classifies the shared endpoint against the diagonal from this
    /// Boolean-published rational contact to `other_endpoint`.
    ///
    /// The recursive identity certifies that the contact lies on its authored
    /// finite chord. When the shared endpoint is one authored chord endpoint,
    /// replacing the contact by the opposite authored endpoint multiplies the
    /// oriented area by a strictly positive factor. If that opposite endpoint
    /// and `other_endpoint` are selected-radial images, their fields then
    /// cancel through the native radial predicate instead of being rebuilt in
    /// Cartesian form.
    pub(in crate::bezier_offset) fn recursive_chord_contact_to_endpoint_oriented_side(
        &self,
        other_endpoint: &CurvePoint2,
        shared_endpoint: &CurvePoint2,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<crate::classify::LineSide>>> {
        let authored_endpoint =
            self.recursive_chord_collinear_support_endpoint(shared_endpoint, policy)?;
        Some((|| {
            let reverse_side = |side| match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            };
            let side = match (authored_endpoint, other_endpoint, shared_endpoint) {
                (
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(authored)),
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(other)),
                    shared,
                ) => {
                    authored.common_untranslated_radial_line_oriented_side(other, shared, policy)?
                }
                (
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(authored)),
                    other,
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(shared)),
                ) => authored
                    .common_untranslated_radial_line_oriented_side(shared, other, policy)?
                    .map(reverse_side),
                _ => None,
            };
            let Some(side) = side else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "recursive-contact-to-radial-endpoint",
            );
            Ok(Classification::Decided(side))
        })())
    }

    pub(crate) fn new(
        parallel: BezierParallel2,
        parameter: BezierParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self::new_with_tangent_distance_parameter(
            parallel,
            BezierAnalyticParallelPointParameter2::Bezier(parameter),
            Real::zero(),
            policy,
        )
    }

    pub(crate) fn new_selected_fiber(
        parallel: BezierParallel2,
        parameter: BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self::new_with_tangent_distance_parameter(
            parallel,
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter),
            Real::zero(),
            policy,
        )
    }

    /// Authors a point, optionally displaced along the unit tangent, without
    /// promoting the retained source parameter into a global algebraic root.
    /// This is the shared construction boundary for Boolean-published
    /// selected and recursive fragments that later re-enter offset/corner
    /// operations.
    pub(crate) fn new_with_region_parameter_and_tangent_distance(
        parallel: BezierParallel2,
        parameter: &CurveParameter2,
        tangent_distance: Real,
        policy: &CurveContext,
    ) -> Option<Self> {
        Self::new_with_region_parameter_and_frame_tangent(
            parallel,
            parameter,
            None,
            tangent_distance,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn new_with_region_parameter_and_frame_tangent(
        parallel: BezierParallel2,
        parameter: &CurveParameter2,
        frame_tangent: Option<Arc<BezierAnalyticParallelTangentField2>>,
        tangent_distance: Real,
        policy: &CurveContext,
    ) -> Option<Self> {
        let parameter = if let Some(parameter) = parameter.as_bezier_parameter() {
            BezierAnalyticParallelPointParameter2::Bezier(parameter.clone())
        } else if let Some(parameter) = parameter.as_selected_fiber() {
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter.clone())
        } else {
            BezierAnalyticParallelPointParameter2::RecursiveProjective(
                parameter.as_recursive_projective()?.clone(),
            )
        };
        Some(Self {
            data: Arc::new(BezierAnalyticParallelPointData2 {
                parallel,
                parameter,
                frame_tangent,
                tangent_distance,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
                policy: policy.retained_object_policy(),
                bounds_cache: Mutex::new(None),
                recursive_projective_point: OnceLock::new(),
            }),
        })
    }

    /// Maps a zero-distance rational-source point onto one coordinate of a
    /// collinear rational target without first resolving its point image.
    /// Constant source coordinates therefore remain O(target degree), even
    /// when the contact parameter's eliminant has high degree.
    pub(in crate::bezier_offset) fn zero_distance_rational_source_parameters_for_axis(
        &self,
        target: &RationalBezier2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<Vec<BezierParameter2>>>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.parallel.distance().zero_status() != ZeroKnowledge::Zero
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_y.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        let BezierParallelSource2::Rational(source) = self.data.parallel.source() else {
            return Ok(None);
        };
        let BezierAnalyticParallelPointParameter2::Bezier(parameter) = &self.data.parameter else {
            return Ok(None);
        };
        let point = source.homogeneous_power_basis()?;
        let target = target.homogeneous_power_basis()?;
        let point_axis = match axis {
            Axis2::X => &point.x_numerator,
            Axis2::Y => &point.y_numerator,
        };
        let target_axis = match axis {
            Axis2::X => &target.x_numerator,
            Axis2::Y => &target.y_numerator,
        };
        let equation = bivariate_subtract(
            &bivariate_outer_product(&point.weight, target_axis),
            &bivariate_outer_product(point_axis, &target.weight),
        );
        if equation
            .coefficients
            .iter()
            .skip(1)
            .flatten()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            let coefficients = equation
                .coefficients
                .first()
                .expect("a bivariate point-coordinate equation retains one row");
            let polynomial = match polynomial_from_coefficients(coefficients.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => {
                    return Ok(Some(Classification::Uncertain(UncertaintyReason::Boundary)));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            return Ok(Some(polynomial.isolate_unit_interval_roots(policy)?));
        }
        let projection = selected_parameter_fiber_parameters(
            &equation,
            parameter,
            MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
            &CurveParameterRange2::unit(),
            policy,
        )?;
        Ok(Some(match projection {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                Classification::Decided(parameters)
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => Classification::Uncertain(UncertaintyReason::Boundary),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }))
    }

    /// Retains one exact point in the source curve's selected orthonormal
    /// frame. `parallel.distance()` is the signed unit-normal displacement and
    /// `tangent_distance` is the signed unit-tangent displacement.
    pub(crate) fn new_with_tangent_distance(
        parallel: BezierParallel2,
        parameter: BezierParameter2,
        tangent_distance: Real,
        policy: &CurveContext,
    ) -> Self {
        Self::new_with_tangent_distance_parameter(
            parallel,
            BezierAnalyticParallelPointParameter2::Bezier(parameter),
            tangent_distance,
            policy,
        )
    }

    pub(in crate::bezier_offset) fn new_with_tangent_distance_parameter(
        parallel: BezierParallel2,
        parameter: BezierAnalyticParallelPointParameter2,
        tangent_distance: Real,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAnalyticParallelPointData2 {
                parallel,
                parameter,
                frame_tangent: None,
                tangent_distance,
                translation_x: Real::zero(),
                translation_y: Real::zero(),
                policy: policy.retained_object_policy(),
                bounds_cache: Mutex::new(None),
                recursive_projective_point: OnceLock::new(),
            }),
        }
    }

    pub(in crate::bezier_offset) fn frame_tangent_power_basis(
        &self,
    ) -> CurveResult<(&[Real], &[Real])> {
        if let Some(tangent) = &self.data.frame_tangent {
            return Ok((&tangent.x, &tangent.y));
        }
        let differential = self.data.parallel.differential()?;
        Ok((&differential.tangent_x, &differential.tangent_y))
    }

    /// A constant regular frame is a represented translation, even when its
    /// speed is irrational. Reuse those coefficients in the parameter's field
    /// instead of adjoining another copy of the same positive speed root.
    pub(in crate::bezier_offset) fn constant_frame_translation(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<[Real; 2]>> {
        let mut translation = [
            self.data.translation_x.clone(),
            self.data.translation_y.clone(),
        ];
        if self.data.parallel.distance().zero_status() == ZeroKnowledge::Zero
            && self.data.tangent_distance.zero_status() == ZeroKnowledge::Zero
        {
            return Ok(Some(translation));
        }
        let (x, y) = self.frame_tangent_power_basis()?;
        if [x, y].into_iter().any(|component| {
            component
                .iter()
                .skip(1)
                .any(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        }) {
            return Ok(None);
        }
        let x = x.first().cloned().unwrap_or_else(Real::zero);
        let y = y.first().cloned().unwrap_or_else(Real::zero);
        let speed_squared = &x * &x + &y * &y;
        if real_sign(&speed_squared, &policy.strict_counterpart()) != Some(RealSign::Positive) {
            return Ok(None);
        }
        let speed = speed_squared.sqrt()?;
        let normal = self.data.parallel.distance();
        let tangent = &self.data.tangent_distance;
        translation[0] =
            &translation[0] + (Real::diff_of_products(tangent, &x, normal, &y) / &speed)?;
        translation[1] = &translation[1] + ((tangent * &y + normal * &x) / speed)?;
        Ok(Some(translation))
    }

    /// Publishes the authored tangent support shared with `end` directly in
    /// the selected parameter field.
    ///
    /// Reconstructing both displaced endpoints and subtracting them repeats
    /// the same selected root and positive speed radical.  For source point
    /// `Q=(X/W,Y/W)`, tangent `H=(Hx,Hy)`, and normal distance `d`, the
    /// positively oriented tangent line has homogeneous coefficients
    ///
    /// `(-Hy*W, Hx*W, Hy*X - Hx*Y - d*W*sqrt(Hx^2+Hy^2))`.
    ///
    /// Tangential displacement cancels identically.  Retaining this compact
    /// line avoids adjoining two independently rebuilt copies of the same
    /// speed root when a miter point is classified against a third support.
    pub(in crate::bezier_offset) fn recursive_tangent_line_to(
        &self,
        end: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || !policy.accepts_retained_policy(end.data.policy)
            || self.data.parallel != end.data.parallel
            || self.data.parameter != end.data.parameter
            || self.data.frame_tangent != end.data.frame_tangent
            || self.data.translation_x != end.data.translation_x
            || self.data.translation_y != end.data.translation_y
        {
            return Ok(Classification::Decided(None));
        }
        let displacement = &end.data.tangent_distance - &self.data.tangent_distance;
        let displacement_sign = match real_sign(&displacement, &CurveContext::STRICT) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "an authored analytic tangent support retained zero displacement".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let source_weight_sign = match self.data.parallel.source() {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                RealSign::Positive
            }
            BezierParallelSource2::Rational(source) => {
                match self.parameter_polynomial_sign(
                    &source.homogeneous_power_basis()?.weight,
                    &policy.strict_counterpart(),
                )? {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "an analytic tangent support retained a zero source denominator".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        let orientation = Real::from(match product_sign(displacement_sign, source_weight_sign) {
            RealSign::Positive => 1_i8,
            RealSign::Negative => -1_i8,
            RealSign::Zero => unreachable!("both analytic tangent orientation factors are nonzero"),
        });
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let translated_x = polynomial_add(
            source.x_numerator,
            &polynomial_scale(weight, &self.data.translation_x),
        );
        let translated_y = polynomial_add(
            source.y_numerator,
            &polynomial_scale(weight, &self.data.translation_y),
        );
        let (tangent_x, tangent_y) = self.frame_tangent_power_basis()?;
        let speed_squared = polynomial_add(
            &polynomial_multiply(tangent_x, tangent_x),
            &polynomial_multiply(tangent_y, tangent_y),
        );
        let line_a = polynomial_scale(&polynomial_multiply(tangent_y, weight), &Real::from(-1_i8));
        let line_b = polynomial_multiply(tangent_x, weight);
        let line_c = polynomial_subtract(
            &polynomial_multiply(tangent_y, &translated_x),
            &polynomial_multiply(tangent_x, &translated_y),
        );

        if let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
            && parameter.projective_scalar().is_some()
        {
            parameter.validate_policy(policy)?;
            let source_degree = [translated_x.len(), translated_y.len(), weight.len()]
                .into_iter()
                .max()
                .unwrap_or(1)
                .saturating_sub(1);
            let tangent_degree = tangent_x.len().max(tangent_y.len()).saturating_sub(1);
            let Some(line_degree) = source_degree.checked_add(tangent_degree) else {
                return Ok(Classification::Decided(None));
            };
            let Some(speed_degree) = tangent_degree.checked_mul(2) else {
                return Ok(Classification::Decided(None));
            };
            let (Some(line_a), Some(line_b), Some(line_c), Some(weight), Some(speed_squared)) = (
                parameter.homogeneous_polynomial_value(&line_a, line_degree),
                parameter.homogeneous_polynomial_value(&line_b, line_degree),
                parameter.homogeneous_polynomial_value(&line_c, line_degree),
                parameter.homogeneous_polynomial_value(weight, source_degree),
                parameter.homogeneous_polynomial_value(&speed_squared, speed_degree),
            ) else {
                return Ok(Classification::Decided(None));
            };
            let mut field = speed_squared.field();
            let speed = if let Some(speed) = field.retained_positive_square_root(&speed_squared) {
                speed
            } else {
                match policy.strict_predicate_pass(|| {
                    parameter.polynomial_sign(
                        &polynomial_add(
                            &polynomial_multiply(tangent_x, tangent_x),
                            &polynomial_multiply(tangent_y, tangent_y),
                        ),
                        policy,
                    )
                })? {
                    Classification::Decided(RealSign::Positive) => {}
                    Classification::Decided(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Decided(RealSign::Negative) => {
                        return Err(CurveError::Topology(
                            "an analytic tangent support retained negative squared speed".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let Some(extension) = field.extension(speed_squared) else {
                    return Ok(Classification::Decided(None));
                };
                let Some(speed) = extension.element(
                    field.constant(Real::zero()).ok_or_else(|| {
                        CurveError::Topology("an analytic tangent line lost its zero".into())
                    })?,
                    field.constant(Real::one()).ok_or_else(|| {
                        CurveError::Topology("an analytic tangent line lost its unit".into())
                    })?,
                ) else {
                    return Ok(Classification::Decided(None));
                };
                field = extension;
                speed
            };
            let (Some(line_a), Some(line_b), Some(line_c), Some(weight)) = (
                field.lift(&line_a),
                field.lift(&line_b),
                field.lift(&line_c),
                field.lift(&weight),
            ) else {
                return Ok(Classification::Decided(None));
            };
            let Some(line_c) = weight
                .multiply(&speed)
                .and_then(|normal| normal.scale(self.data.parallel.distance()))
                .and_then(|normal| line_c.subtract(&normal))
            else {
                return Ok(Classification::Decided(None));
            };
            return Ok(Classification::Decided(Some(
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: line_a.scale(&orientation).ok_or_else(|| {
                        CurveError::Topology(
                            "an analytic tangent line exceeded its field budget".into(),
                        )
                    })?,
                    y: line_b.scale(&orientation).ok_or_else(|| {
                        CurveError::Topology(
                            "an analytic tangent line exceeded its field budget".into(),
                        )
                    })?,
                    denominator: line_c.scale(&orientation).ok_or_else(|| {
                        CurveError::Topology(
                            "an analytic tangent line exceeded its field budget".into(),
                        )
                    })?,
                },
            )));
        }

        let parameter = match &self.data.parameter {
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => parameter.clone(),
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                match policy.strict_predicate_pass(|| {
                    parameter.promoted_bezier_parameter_complete(policy)
                })? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                match policy.strict_predicate_pass(|| {
                    parameter.promoted_bezier_parameter_complete(policy)
                })? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        match policy.strict_predicate_pass(|| {
            signed_coefficients_at_parameter(&speed_squared, &parameter, policy)
        })? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an analytic tangent support retained negative squared speed".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let source_root = bezier_parameter_root_representation(&parameter);
        let tensor =
            |coefficients: &[Real]| DenseTensorPolynomial::from_axis_polynomial(1, 0, coefficients);
        let (Some(speed_squared), Some(one)) = (
            tensor(&speed_squared),
            tensor(std::slice::from_ref(&Real::one())),
        ) else {
            return Ok(Classification::Decided(None));
        };
        let Some(field) = RecursiveQuadraticField::base(vec![source_root], speed_squared, one)
        else {
            return Ok(Classification::Decided(None));
        };
        let RecursiveQuadraticField::Base(base) = &field else {
            unreachable!("an analytic tangent line begins at its dense base")
        };
        let rational =
            |coefficients: &[Real]| recursive_quadratic_rational_value(base, tensor(coefficients)?);
        let line_with_speed = |rational_coefficients: &[Real], speed_coefficients: &[Real]| {
            let zero = DenseTensorPolynomial::zero(vec![1])?;
            RecursiveQuadraticValue::from_base(
                base.clone(),
                TwoSquareRootExpression {
                    rational: tensor(rational_coefficients)?,
                    first: tensor(speed_coefficients)?,
                    second: zero.clone(),
                    product: zero,
                },
            )
        };
        let normal = polynomial_scale(weight, &(-self.data.parallel.distance().clone()));
        let (Some(line_a), Some(line_b), Some(line_c)) = (
            rational(&line_a),
            rational(&line_b),
            line_with_speed(&line_c, &normal),
        ) else {
            return Ok(Classification::Decided(None));
        };
        Ok(Classification::Decided(Some(
            BezierRecursiveQuadraticProjectivePoint2 {
                x: line_a.scale(&orientation).ok_or_else(|| {
                    CurveError::Topology(
                        "an analytic tangent line exceeded its field budget".into(),
                    )
                })?,
                y: line_b.scale(&orientation).ok_or_else(|| {
                    CurveError::Topology(
                        "an analytic tangent line exceeded its field budget".into(),
                    )
                })?,
                denominator: line_c.scale(&orientation).ok_or_else(|| {
                    CurveError::Topology(
                        "an analytic tangent line exceeded its field budget".into(),
                    )
                })?,
            },
        )))
    }

    /// Evaluates this point directly in an endpoint's existing recursive
    /// projective field. Source coordinates share one homogeneous degree; the
    /// tangent frame shares another. Adjoining the strictly positive source
    /// speed therefore preserves every scale factor and the authored radical
    /// sheet without first projecting the endpoint onto a global polynomial.
    pub(in crate::bezier_offset) fn recursive_projective_point_from_recursive_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        parameter.validate_policy(policy)?;
        let source_weight_sign = match self.data.parallel.source() {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                Some(RealSign::Positive)
            }
            BezierParallelSource2::Rational(source) => {
                match parameter.polynomial_sign(
                    &source.homogeneous_power_basis()?.weight,
                    &policy.strict_counterpart(),
                )? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => None,
                }
            }
        };
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let translation = self.constant_frame_translation(policy)?;
        let (translation_x, translation_y) = translation.as_ref().map_or(
            (&self.data.translation_x, &self.data.translation_y),
            |[x, y]| (x, y),
        );
        let translated_x =
            polynomial_add(source.x_numerator, &polynomial_scale(weight, translation_x));
        let translated_y =
            polynomial_add(source.y_numerator, &polynomial_scale(weight, translation_y));
        let source_degree = [translated_x.len(), translated_y.len(), weight.len()]
            .into_iter()
            .max()
            .unwrap_or(1)
            .saturating_sub(1);
        let Some(translated_x) =
            parameter.homogeneous_polynomial_value(&translated_x, source_degree)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(translated_y) =
            parameter.homogeneous_polynomial_value(&translated_y, source_degree)
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(weight_value) = parameter.homogeneous_polynomial_value(weight, source_degree)
        else {
            return Ok(Classification::Decided(None));
        };
        let point = if translation.is_some() {
            BezierRecursiveQuadraticProjectivePoint2 {
                x: translated_x,
                y: translated_y,
                denominator: weight_value,
            }
        } else {
            let (tangent_x, tangent_y) = self.frame_tangent_power_basis()?;
            let mut unresolved_component = None;
            let mut certified_nonzero_component = false;
            for component in [tangent_x, tangent_y] {
                if component
                    .iter()
                    .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
                {
                    continue;
                }
                match policy
                    .strict_predicate_pass(|| parameter.polynomial_sign(component, policy))?
                {
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                        certified_nonzero_component = true;
                        break;
                    }
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Uncertain(reason) => {
                        unresolved_component.get_or_insert(reason);
                    }
                }
            }
            if !certified_nonzero_component {
                return Ok(Classification::Uncertain(
                    unresolved_component.unwrap_or(UncertaintyReason::Boundary),
                ));
            }
            let tangent_degree = tangent_x.len().max(tangent_y.len()).saturating_sub(1);
            let Some(speed_degree) = tangent_degree.checked_mul(2) else {
                return Ok(Classification::Decided(None));
            };
            let Some(frame_degree) = source_degree.checked_add(tangent_degree) else {
                return Ok(Classification::Decided(None));
            };
            let frame_x = polynomial_subtract(
                &polynomial_scale(tangent_x, &self.data.tangent_distance),
                &polynomial_scale(tangent_y, self.data.parallel.distance()),
            );
            let frame_y = polynomial_add(
                &polynomial_scale(tangent_x, self.data.parallel.distance()),
                &polynomial_scale(tangent_y, &self.data.tangent_distance),
            );
            let speed_squared = polynomial_add(
                &polynomial_multiply(tangent_x, tangent_x),
                &polynomial_multiply(tangent_y, tangent_y),
            );
            let Some(speed_squared) =
                parameter.homogeneous_polynomial_value(&speed_squared, speed_degree)
            else {
                return Ok(Classification::Decided(None));
            };
            let Some(frame_x) = parameter
                .homogeneous_polynomial_value(&polynomial_multiply(weight, &frame_x), frame_degree)
            else {
                return Ok(Classification::Decided(None));
            };
            let Some(frame_y) = parameter
                .homogeneous_polynomial_value(&polynomial_multiply(weight, &frame_y), frame_degree)
            else {
                return Ok(Classification::Decided(None));
            };
            let field = speed_squared.field();
            let Some(extension) = field.extension(speed_squared) else {
                return Ok(Classification::Decided(None));
            };
            let Some(speed) = extension.element(
                field.constant(Real::zero()).ok_or_else(|| {
                    CurveError::Topology("a recursive analytic point lost its zero".into())
                })?,
                field.constant(Real::one()).ok_or_else(|| {
                    CurveError::Topology("a recursive analytic point lost its unit".into())
                })?,
            ) else {
                return Ok(Classification::Decided(None));
            };
            let Some(translated_x) = extension.lift(&translated_x) else {
                return Ok(Classification::Decided(None));
            };
            let Some(translated_y) = extension.lift(&translated_y) else {
                return Ok(Classification::Decided(None));
            };
            let Some(weight_value) = extension.lift(&weight_value) else {
                return Ok(Classification::Decided(None));
            };
            let Some(frame_x) = extension.lift(&frame_x) else {
                return Ok(Classification::Decided(None));
            };
            let Some(frame_y) = extension.lift(&frame_y) else {
                return Ok(Classification::Decided(None));
            };
            let Some(point) = (|| {
                Some(BezierRecursiveQuadraticProjectivePoint2 {
                    x: translated_x.multiply(&speed)?.add(&frame_x)?,
                    y: translated_y.multiply(&speed)?.add(&frame_y)?,
                    denominator: weight_value.multiply(&speed)?,
                })
            })() else {
                return Ok(Classification::Decided(None));
            };
            point
        };
        let point = match source_weight_sign {
            // The recursive parameter denominator is strictly positive, as
            // is every adjoined speed. The source denominator sign at the
            // retained parameter certifies the projective denominator and
            // avoids expanding its deep recursive norm merely to normalize a
            // point that was authored on that sheet.
            Some(RealSign::Positive) => point,
            Some(RealSign::Negative) => {
                let negative = Real::from(-1_i8);
                BezierRecursiveQuadraticProjectivePoint2 {
                    x: point.x.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive analytic point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                    y: point.y.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive analytic point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                    denominator: point.denominator.scale(&negative).ok_or_else(|| {
                        CurveError::Topology(
                            "a recursive analytic point normalization exceeded its field budget"
                                .into(),
                        )
                    })?,
                }
            }
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a recursive analytic point retained a zero source denominator".into(),
                ));
            }
            None => match positive_recursive_projective_point(point)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-projective-point",
            "analytic-parallel-retained-parameter",
        );
        Ok(Classification::Decided(Some(point)))
    }

    /// Imports this retained analytic point into one recursive projective
    /// field without materializing its Cartesian coordinates independently.
    ///
    /// A selected-fiber parameter is promoted only at this cold carrier
    /// boundary, and its already-isolated root chooses the global parameter
    /// under a strict predicate pass. The source point and its positive speed
    /// square root then remain correlated in one dense base axis. Therefore
    /// APPROXIMATE_512 may still terminate later equality predicates, but can
    /// never select the construction field or radical sheet.
    pub(in crate::bezier_offset) fn recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "an analytic-parallel point entered a recursive field under a different policy"
                    .into(),
            ));
        }
        if let Some(point) = self.data.recursive_projective_point.get() {
            return Ok(Classification::Decided(Some(point.clone())));
        }
        let result =
            policy.strict_predicate_pass(|| self.compute_recursive_projective_point(policy))?;
        if let Classification::Decided(Some(point)) = result {
            // Clones and concurrent queries must reuse the same selected
            // field, not merely reconstruct equivalent coefficient towers.
            // Only a strictly certified point is retained; uncertainty can
            // still be resolved by a later query with additional evidence.
            let _ = self.data.recursive_projective_point.set(point);
            return Ok(Classification::Decided(
                self.data.recursive_projective_point.get().cloned(),
            ));
        }
        Ok(result)
    }

    pub(in crate::bezier_offset) fn compute_recursive_projective_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticProjectivePoint2>>> {
        let parameter = match &self.data.parameter {
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => parameter.clone(),
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                parameter.validate_policy(policy)?;
                // Keep the local field's correlations even when another query
                // has also cached a higher-degree native projection.
                if let Some(parameter) = parameter.recursive_projective_parameter(policy)?
                    && let Ok(Classification::Decided(Some(point))) =
                        self.recursive_projective_point_from_recursive_parameter(&parameter, policy)
                {
                    return Ok(Classification::Decided(Some(point)));
                }
                if let Some(parameter) = parameter.retained_bezier_parameter() {
                    parameter
                } else {
                    if policy.has_bounded_exact_predicate_budget() {
                        // New global projection is cold work. Reusing an
                        // existing native or local-field proof above is not.
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                    match parameter.promoted_bezier_parameter_complete(policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                parameter.validate_policy(policy)?;
                if parameter.projective_scalar().is_some() {
                    return self
                        .recursive_projective_point_from_recursive_parameter(parameter, policy);
                }
                if let Some(parameter) = parameter.data.projection.parameter.get() {
                    parameter.clone()
                } else {
                    if policy.has_bounded_exact_predicate_budget() {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                    match parameter.promoted_bezier_parameter_complete(policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
        };
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let translation = self.constant_frame_translation(policy)?;
        let (translation_x, translation_y) = translation.as_ref().map_or(
            (&self.data.translation_x, &self.data.translation_y),
            |[x, y]| (x, y),
        );
        let translated_x =
            polynomial_add(source.x_numerator, &polynomial_scale(weight, translation_x));
        let translated_y =
            polynomial_add(source.y_numerator, &polynomial_scale(weight, translation_y));
        let parameter_source = bezier_parameter_root_representation(&parameter);
        let tensor =
            |coefficients: &[Real]| DenseTensorPolynomial::from_axis_polynomial(1, 0, coefficients);
        let Some(one) = tensor(std::slice::from_ref(&Real::one())) else {
            return Ok(Classification::Decided(None));
        };
        let point = if translation.is_some() {
            let Some(field) =
                RecursiveQuadraticField::base(vec![parameter_source], one.clone(), one)
            else {
                return Ok(Classification::Decided(None));
            };
            let RecursiveQuadraticField::Base(base) = &field else {
                unreachable!("an analytic point recursive field begins at its dense base")
            };
            let value = |coefficients: &[Real]| {
                recursive_quadratic_rational_value(base, tensor(coefficients)?)
            };
            let Some(point) = (|| {
                Some(BezierRecursiveQuadraticProjectivePoint2 {
                    x: value(&translated_x)?,
                    y: value(&translated_y)?,
                    denominator: value(weight)?,
                })
            })() else {
                return Ok(Classification::Decided(None));
            };
            point
        } else {
            let (frame_tangent_x, frame_tangent_y) = self.frame_tangent_power_basis()?;
            let speed_squared = polynomial_add(
                &polynomial_multiply(frame_tangent_x, frame_tangent_x),
                &polynomial_multiply(frame_tangent_y, frame_tangent_y),
            );
            match policy.strict_predicate_pass(|| {
                signed_coefficients_at_parameter(&speed_squared, &parameter, policy)
            })? {
                Classification::Decided(RealSign::Positive) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "an analytic point frame retained negative squared speed".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let Some(speed_squared) = tensor(&speed_squared) else {
                return Ok(Classification::Decided(None));
            };
            let Some(field) =
                RecursiveQuadraticField::base(vec![parameter_source], speed_squared, one)
            else {
                return Ok(Classification::Decided(None));
            };
            let RecursiveQuadraticField::Base(base) = &field else {
                unreachable!("an analytic point recursive field begins at its dense base")
            };
            let expression = |rational: &[Real], first: &[Real]| {
                let zero = DenseTensorPolynomial::zero(vec![1])?;
                RecursiveQuadraticValue::from_base(
                    base.clone(),
                    TwoSquareRootExpression {
                        rational: tensor(rational)?,
                        first: tensor(first)?,
                        second: zero.clone(),
                        product: zero,
                    },
                )
            };
            let frame_x = polynomial_subtract(
                &polynomial_scale(frame_tangent_x, &self.data.tangent_distance),
                &polynomial_scale(frame_tangent_y, self.data.parallel.distance()),
            );
            let frame_y = polynomial_add(
                &polynomial_scale(frame_tangent_x, self.data.parallel.distance()),
                &polynomial_scale(frame_tangent_y, &self.data.tangent_distance),
            );
            let weighted_frame_x = polynomial_multiply(weight, &frame_x);
            let weighted_frame_y = polynomial_multiply(weight, &frame_y);
            let zero = [Real::zero()];
            let Some(point) = (|| {
                Some(BezierRecursiveQuadraticProjectivePoint2 {
                    x: expression(&weighted_frame_x, &translated_x)?,
                    y: expression(&weighted_frame_y, &translated_y)?,
                    denominator: expression(&zero, weight)?,
                })
            })() else {
                return Ok(Classification::Decided(None));
            };
            point
        };
        let point = match positive_recursive_projective_point(point)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-projective-point",
            "analytic-parallel",
        );
        Ok(Classification::Decided(Some(point)))
    }

    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

    /// Compares coordinates of two points on one zero-distance source while
    /// preserving the selected fiber that relates their parameters.
    ///
    /// Finite-envelope corner reconstruction can retain one boundary as an
    /// ordinary algebraic parameter `alpha` and the other as a selected root
    /// `u` over that same `alpha`. Expanding either point into independent
    /// Cartesian roots discards exactly that correlation. The homogeneous
    /// difference
    ///
    /// `N(u) * W(alpha) - N(alpha) * W(u)`
    ///
    /// instead lives directly in the existing `(alpha, u)` field. Both its
    /// sign and the denominator sign are construction predicates, so they are
    /// always evaluated by a STRICT pass even when the retained carrier uses
    /// APPROXIMATE_512 terminal equality policy.
    pub(in crate::bezier_offset) fn same_zero_distance_source_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || !policy.accepts_retained_policy(other.data.policy)
        {
            return Err(CurveError::Topology(
                "analytic source points crossed predicate policies".into(),
            ));
        }
        if self.data.parallel != other.data.parallel
            || self.data.parallel.distance().zero_status() != ZeroKnowledge::Zero
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || other.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
        {
            return Ok(None);
        }
        let (selected, ordinary, reverse) = match (&self.data.parameter, &other.data.parameter) {
            (
                BezierAnalyticParallelPointParameter2::SelectedFiber(selected),
                BezierAnalyticParallelPointParameter2::Bezier(BezierParameter2::Algebraic(
                    ordinary,
                )),
            ) => (selected, ordinary, false),
            (
                BezierAnalyticParallelPointParameter2::Bezier(BezierParameter2::Algebraic(
                    ordinary,
                )),
                BezierAnalyticParallelPointParameter2::SelectedFiber(selected),
            ) => (selected, ordinary, true),
            _ => return Ok(None),
        };
        if ordinary != &selected.data.authority.data.retained_parameter {
            return Ok(None);
        }

        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let coordinate = match axis {
            Axis2::X => source.x_numerator,
            Axis2::Y => source.y_numerator,
        };
        let numerator = bivariate_subtract(
            &bivariate_outer_product(weight, coordinate),
            &bivariate_outer_product(coordinate, weight),
        );
        let denominator = bivariate_outer_product(weight, weight);
        let (numerator_sign, denominator_sign) = policy.strict_predicate_pass(|| {
            Ok::<_, CurveError>((
                selected.predicate_sign(&numerator, policy)?,
                selected.predicate_sign(&denominator, policy)?,
            ))
        })?;
        let order = match (numerator_sign, denominator_sign) {
            (
                Classification::Decided(numerator),
                Classification::Decided(denominator @ (RealSign::Positive | RealSign::Negative)),
            ) => {
                let sign = if denominator == RealSign::Positive {
                    numerator
                } else {
                    match numerator {
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                        RealSign::Positive => RealSign::Negative,
                    }
                };
                Classification::Decided(match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                })
            }
            (_, Classification::Decided(RealSign::Zero)) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Classification::Uncertain(reason)
            }
        };
        #[cfg(feature = "dispatch-trace")]
        if matches!(order, Classification::Decided(_)) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-axis-order",
                "same-zero-distance-selected-source",
            );
        }
        Ok(Some(if reverse {
            order.map(std::cmp::Ordering::reverse)
        } else {
            order
        }))
    }

    /// Re-enters the ordinary point authority when this retained analytic
    /// point has an exact Cartesian or one-parameter algebraic form.
    ///
    /// This is a predicate-only normalization: the retained analytic point
    /// remains authoritative. Exact parameters can evaluate any regular
    /// zero-tangent-displacement parallel directly; zero-frame algebraic
    /// parameters reuse the source's one-field point image. Both avoid a
    /// second two-parallel field without weakening equality under policy.
    pub(in crate::bezier_offset) fn predicate_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "analytic-parallel point entered a predicate under a different policy".into(),
            ));
        }
        // A zero-displacement image already has a one-field authority.
        // Warming an optional scalar view must not replace that relation in
        // later predicates that can prove more with the original root.
        if let BezierAnalyticParallelPointParameter2::Bezier(parameter) = &self.data.parameter
            && [
                self.data.parallel.distance(),
                &self.data.tangent_distance,
                &self.data.translation_x,
                &self.data.translation_y,
            ]
            .into_iter()
            .all(|value| real_sign(value, &CurveContext::STRICT) == Some(RealSign::Zero))
        {
            let source = self.data.parallel.source().to_rational_bezier()?;
            match crate::rational_bezier_general::exact_contact_point_evidence(
                &source, parameter, policy,
            )? {
                Classification::Decided(point) => return Ok(Classification::Decided(Some(point))),
                Classification::Uncertain(UncertaintyReason::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(_) => {}
            }
        }
        Ok(self
            .represented_point(policy)?
            .map(|point| point.map(CurvePoint2::from)))
    }

    /// Materializes this retained point only at a cold predicate boundary.
    ///
    /// The ordinary point authority deliberately keeps an algebraic source
    /// parameter and its positive speed radical correlated.  A line support
    /// whose other endpoint belongs to an unrelated retained carrier cannot
    /// use that compact one-field form, however.  In that case Hypersolve
    /// selects exact standalone coordinate roots under STRICT and the caller
    /// can build one complete mixed-carrier incidence tensor.  No represented
    /// coordinate is used to select a persistent geometry representation.
    pub(in crate::bezier_offset) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "analytic-parallel point entered a represented predicate under a different policy"
                    .into(),
            ));
        }
        if matches!(
            &self.data.parameter,
            BezierAnalyticParallelPointParameter2::RecursiveProjective(_)
        ) {
            return match self.recursive_projective_point(policy)? {
                Classification::Decided(Some(point)) => point.represented_coordinates(policy),
                Classification::Decided(None) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        let source = self.data.parallel.source_power_basis()?;
        let unit = [Real::one()];
        let weight = source.weight.unwrap_or(&unit);
        let zero_frame = self.data.parallel.distance().zero_status() == ZeroKnowledge::Zero
            && self.data.tangent_distance.zero_status() == ZeroKnowledge::Zero;
        let translated_coordinates =
            |x: AlgebraicRootRepresentation, y: AlgebraicRootRepresentation| {
                let x =
                    represented_affine_coordinate(&[(&x, &Real::one())], &self.data.translation_x);
                let y =
                    represented_affine_coordinate(&[(&y, &Real::one())], &self.data.translation_y);
                match (x, y) {
                    (Classification::Decided(x), Classification::Decided(y)) => {
                        Classification::Decided([x, y])
                    }
                    (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                    | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    _ => Classification::Uncertain(UncertaintyReason::Predicate),
                }
            };
        if let BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) =
            &self.data.parameter
        {
            // Construction may use the retained-field reduction only after an
            // exact predicate proves that this rational chart is finite at the
            // selected root. APPROXIMATE_512 remains terminal equality policy,
            // never construction evidence.
            let denominator_predicate = bivariate_outer_product(&[Real::one()], weight);
            let denominator_nonzero = match policy.strict_predicate_pass(|| {
                parameter.predicate_sign(&denominator_predicate, policy)
            })? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => true,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(_) => false,
            };
            if denominator_nonzero {
                let represented_pair = |x_numerator: &[Real],
                                        y_numerator: &[Real],
                                        denominator: &[Real]|
                 -> CurveResult<
                    Option<[AlgebraicRootRepresentation; 2]>,
                > {
                    let x = parameter.represented_retained_field_rational_value(
                        x_numerator,
                        denominator,
                        policy,
                    )?;
                    let y = parameter.represented_retained_field_rational_value(
                        y_numerator,
                        denominator,
                        policy,
                    )?;
                    Ok(match (x, y) {
                        (Classification::Decided(Some(x)), Classification::Decided(Some(y))) => {
                            Some([x, y])
                        }
                        _ => None,
                    })
                };
                let translated_x = polynomial_add(
                    source.x_numerator,
                    &polynomial_scale(weight, &self.data.translation_x),
                );
                let translated_y = polynomial_add(
                    source.y_numerator,
                    &polynomial_scale(weight, &self.data.translation_y),
                );
                if zero_frame {
                    if let Some(coordinates) =
                        represented_pair(&translated_x, &translated_y, weight)?
                    {
                        return Ok(Classification::Decided(coordinates));
                    }
                } else {
                    let (frame_tangent_x, frame_tangent_y) = self.frame_tangent_power_basis()?;
                    let speed_squared = polynomial_add(
                        &polynomial_multiply(frame_tangent_x, frame_tangent_x),
                        &polynomial_multiply(frame_tangent_y, frame_tangent_y),
                    );
                    let speed_positive = match policy.strict_predicate_pass(|| {
                        parameter.predicate_sign(
                            &bivariate_outer_product(&[Real::one()], &speed_squared),
                            policy,
                        )
                    })? {
                        Classification::Decided(RealSign::Positive) => true,
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                        }
                        Classification::Decided(RealSign::Negative) => {
                            return Err(CurveError::Topology(
                                "analytic point frame had negative squared speed".into(),
                            ));
                        }
                        Classification::Uncertain(_) => false,
                    };
                    if speed_positive {
                        let frame_x = polynomial_subtract(
                            &polynomial_scale(frame_tangent_x, &self.data.tangent_distance),
                            &polynomial_scale(frame_tangent_y, self.data.parallel.distance()),
                        );
                        let frame_y = polynomial_add(
                            &polynomial_scale(frame_tangent_x, self.data.parallel.distance()),
                            &polynomial_scale(frame_tangent_y, &self.data.tangent_distance),
                        );
                        let speed = policy.strict_predicate_pass(|| {
                            polynomial_square_root(&speed_squared, policy)
                        })?;
                        if let Classification::Decided(Some(speed)) = speed {
                            let speed_predicate = bivariate_outer_product(&[Real::one()], &speed);
                            let speed = match policy.strict_predicate_pass(|| {
                                parameter.predicate_sign(&speed_predicate, policy)
                            })? {
                                Classification::Decided(RealSign::Positive) => Some(speed),
                                Classification::Decided(RealSign::Negative) => {
                                    Some(polynomial_scale(&speed, &Real::from(-1_i8)))
                                }
                                Classification::Decided(RealSign::Zero) => {
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Boundary,
                                    ));
                                }
                                Classification::Uncertain(_) => None,
                            };
                            if let Some(speed) = speed {
                                let denominator = polynomial_multiply(weight, &speed);
                                let numerator_x = polynomial_add(
                                    &polynomial_multiply(&translated_x, &speed),
                                    &polynomial_multiply(weight, &frame_x),
                                );
                                let numerator_y = polynomial_add(
                                    &polynomial_multiply(&translated_y, &speed),
                                    &polynomial_multiply(weight, &frame_y),
                                );
                                if let Some(coordinates) =
                                    represented_pair(&numerator_x, &numerator_y, &denominator)?
                                {
                                    return Ok(Classification::Decided(coordinates));
                                }
                            }
                        }

                        let x_relation = polynomial_unit_frame_coordinate_relation(
                            &translated_x,
                            weight,
                            &frame_x,
                            &speed_squared,
                        );
                        let y_relation = polynomial_unit_frame_coordinate_relation(
                            &translated_y,
                            weight,
                            &frame_y,
                            &speed_squared,
                        );
                        if let Some((x_coefficients, provenance)) =
                            parameter.represented_polynomial_image_eliminant(&x_relation, policy)?
                            && let Some((y_coefficients, _)) = parameter
                                .represented_polynomial_image_eliminant(&y_relation, policy)?
                        {
                            let mut represented_x = None;
                            let mut represented_y = None;
                            for refinement_steps in [8, 16, 32, 64, 128, 256, 512] {
                                let Classification::Decided(bounds) =
                                    policy.strict_predicate_pass(|| {
                                        self.conservative_bounds_refined(refinement_steps, policy)
                                    })
                                else {
                                    continue;
                                };
                                if represented_x.is_none()
                                    && let Classification::Decided(value) =
                                        represented_univariate_coordinate(
                                            &x_coefficients,
                                            bounds.min().x(),
                                            bounds.max().x(),
                                            &provenance,
                                        )
                                {
                                    represented_x = Some(value);
                                }
                                if represented_y.is_none()
                                    && let Classification::Decided(value) =
                                        represented_univariate_coordinate(
                                            &y_coefficients,
                                            bounds.min().y(),
                                            bounds.max().y(),
                                            &provenance,
                                        )
                                {
                                    represented_y = Some(value);
                                }
                                if represented_x.is_some() && represented_y.is_some() {
                                    return Ok(Classification::Decided([
                                        represented_x
                                            .take()
                                            .expect("a represented x coordinate was retained"),
                                        represented_y
                                            .take()
                                            .expect("a represented y coordinate was retained"),
                                    ]));
                                }
                            }
                        }
                    }
                }
            }
        }
        let parameter = match &self.data.parameter {
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => parameter.clone(),
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                match parameter.promoted_bezier_parameter_complete(policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            BezierAnalyticParallelPointParameter2::RecursiveProjective(_) => {
                unreachable!("recursive analytic points retain their projective field")
            }
        };
        let parameter = bezier_parameter_root_representation(&parameter);
        let tensor =
            |coefficients: &[Real]| DenseTensorPolynomial::from_axis_polynomial(2, 0, coefficients);
        let (Some(x_numerator), Some(y_numerator), Some(weight_tensor)) = (
            tensor(source.x_numerator),
            tensor(source.y_numerator),
            tensor(weight),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let selected = std::slice::from_ref(&parameter);
        let source_x = represented_tensor_ratio(&x_numerator, &weight_tensor, selected);
        let source_y = represented_tensor_ratio(&y_numerator, &weight_tensor, selected);
        let reason = [&source_x, &source_y]
            .into_iter()
            .find_map(|value| match value {
                Classification::Decided(_) => None,
                Classification::Uncertain(reason) => Some(*reason),
            });
        let (Classification::Decided(source_x), Classification::Decided(source_y)) =
            (source_x, source_y)
        else {
            return Ok(Classification::Uncertain(
                reason.unwrap_or(UncertaintyReason::Unsupported),
            ));
        };
        if zero_frame {
            return Ok(translated_coordinates(source_x, source_y));
        }

        let (frame_tangent_x, frame_tangent_y) = self.frame_tangent_power_basis()?;
        let (Some(tangent_x), Some(tangent_y)) = (tensor(frame_tangent_x), tensor(frame_tangent_y))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let tangent_x = represented_dense_value_refined(&tangent_x, selected);
        let tangent_y = represented_dense_value_refined(&tangent_y, selected);
        let reason = [&tangent_x, &tangent_y]
            .into_iter()
            .find_map(|value| match value {
                Classification::Decided(_) => None,
                Classification::Uncertain(reason) => Some(*reason),
            });
        let (Classification::Decided(tangent_x), Classification::Decided(tangent_y)) =
            (tangent_x, tangent_y)
        else {
            return Ok(Classification::Uncertain(
                reason.unwrap_or(UncertaintyReason::Unsupported),
            ));
        };
        let speed_squared = match represented_vector_dot_cross(
            &[tangent_x.clone(), tangent_y.clone()],
            &[tangent_x.clone(), tangent_y.clone()],
        ) {
            Classification::Decided([speed_squared, _]) => speed_squared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed = square_root_algebraic_root_representation(&speed_squared, 1);
        let speed = match speed.status {
            AlgebraicRootSquareRootStatus::Transformed => speed
                .representation
                .expect("a represented analytic-parallel speed retains its positive root"),
            AlgebraicRootSquareRootStatus::UndecidedSign => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            AlgebraicRootSquareRootStatus::InvalidEvidence
            | AlgebraicRootSquareRootStatus::InvalidBranch
            | AlgebraicRootSquareRootStatus::NegativeRadicand
            | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
            | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let negate = |value: &AlgebraicRootRepresentation| {
            represented_affine_coordinate(&[(value, &Real::from(-1_i8))], &Real::zero())
        };
        let normal_x = match negate(&tangent_y) {
            Classification::Decided(value) => represented_ratio(&value, &speed),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        let normal_y = represented_ratio(&tangent_x, &speed);
        let unit_tangent_x = represented_ratio(&tangent_x, &speed);
        let unit_tangent_y = represented_ratio(&tangent_y, &speed);
        let reason = [&normal_x, &normal_y, &unit_tangent_x, &unit_tangent_y]
            .into_iter()
            .find_map(|value| match value {
                Classification::Decided(_) => None,
                Classification::Uncertain(reason) => Some(*reason),
            });
        let (
            Classification::Decided(normal_x),
            Classification::Decided(normal_y),
            Classification::Decided(unit_tangent_x),
            Classification::Decided(unit_tangent_y),
        ) = (normal_x, normal_y, unit_tangent_x, unit_tangent_y)
        else {
            return Ok(Classification::Uncertain(
                reason.unwrap_or(UncertaintyReason::Unsupported),
            ));
        };
        let x = represented_affine_coordinate(
            &[
                (&source_x, &Real::one()),
                (&normal_x, self.data.parallel.distance()),
                (&unit_tangent_x, &self.data.tangent_distance),
            ],
            &self.data.translation_x,
        );
        let y = represented_affine_coordinate(
            &[
                (&source_y, &Real::one()),
                (&normal_y, self.data.parallel.distance()),
                (&unit_tangent_y, &self.data.tangent_distance),
            ],
            &self.data.translation_y,
        );
        Ok(match (x, y) {
            (Classification::Decided(x), Classification::Decided(y)) => {
                Classification::Decided([x, y])
            }
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            _ => Classification::Uncertain(UncertaintyReason::Predicate),
        })
    }

    pub(crate) fn represented_point(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Point2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Ok(Classification::Decided(None));
        }
        if let Some(point) =
            self.data.recursive_projective_point.get().and_then(
                BezierRecursiveQuadraticProjectivePoint2::exact_point_with_retained_witnesses,
            )
        {
            return Ok(Classification::Decided(Some(point)));
        }
        let BezierAnalyticParallelPointParameter2::Bezier(BezierParameter2::Exact(parameter)) =
            &self.data.parameter
        else {
            return Ok(Classification::Decided(None));
        };
        let source = match self.data.parallel.source_point_at(parameter, policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if self.data.parallel.distance().zero_status() == ZeroKnowledge::Zero
            && self.data.tangent_distance.zero_status() == ZeroKnowledge::Zero
        {
            return Ok(Classification::Decided(Some(source.translated(
                self.data.translation_x.clone(),
                self.data.translation_y.clone(),
            ))));
        }
        let (frame_tangent_x, frame_tangent_y) = self.frame_tangent_power_basis()?;
        let tangent_x = Real::eval_poly(frame_tangent_x, parameter);
        let tangent_y = Real::eval_poly(frame_tangent_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "analytic point frame had negative squared speed".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.sqrt()?;
        let frame_x = (((Real::zero() - &tangent_y) * self.data.parallel.distance()
            + &tangent_x * &self.data.tangent_distance)
            / &speed)?;
        let frame_y = ((&tangent_x * self.data.parallel.distance()
            + &tangent_y * &self.data.tangent_distance)
            / speed)?;
        Ok(Classification::Decided(Some(source.translated(
            frame_x + &self.data.translation_x,
            frame_y + &self.data.translation_y,
        ))))
    }

    pub(in crate::bezier_offset) fn translated(
        &self,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "analytic-parallel point was translated under a different predicate policy".into(),
            ));
        }
        Ok(Self {
            data: Arc::new(BezierAnalyticParallelPointData2 {
                parallel: self.data.parallel.clone(),
                parameter: self.data.parameter.clone(),
                frame_tangent: self.data.frame_tangent.clone(),
                tangent_distance: self.data.tangent_distance.clone(),
                translation_x: &self.data.translation_x + delta_x,
                translation_y: &self.data.translation_y + delta_y,
                policy: policy.retained_object_policy(),
                bounds_cache: Mutex::new(None),
                recursive_projective_point: OnceLock::new(),
            }),
        })
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        if let Ok(cache) = self.data.bounds_cache.lock()
            && let Some((cached_policy, cached_steps, bounds)) = cache.as_ref()
            && cached_policy == policy
            && *cached_steps >= refinement_steps
        {
            return Classification::Decided(bounds.clone());
        }
        let result = match &self.data.parameter {
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                let parameter = parameter
                    .clone()
                    .refined_isolating_interval(refinement_steps, policy);
                retained_analytic_parallel_point_bounds_at_bezier_parameter(self, &parameter)
            }
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                let parameter = match parameter.refined(refinement_steps, policy) {
                    Ok(Classification::Decided(parameter)) => parameter,
                    Ok(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                };
                analytic_parallel_point_bounds_over_interval_with_tangent(
                    &self.data.parallel,
                    &RealInterval {
                        lower: parameter.root().lower.clone(),
                        upper: parameter.root().upper.clone(),
                    },
                    self.data
                        .frame_tangent
                        .as_ref()
                        .map(|tangent| (&tangent.x[..], &tangent.y[..])),
                    &self.data.tangent_distance,
                    &self.data.translation_x,
                    &self.data.translation_y,
                )
            }
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                let parameter = match parameter.refined(refinement_steps, policy) {
                    Ok(Classification::Decided(parameter)) => parameter,
                    Ok(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                };
                let (lower, upper) = parameter.isolating_bounds();
                analytic_parallel_point_bounds_over_interval_with_tangent(
                    &self.data.parallel,
                    &RealInterval {
                        lower: lower.clone(),
                        upper: upper.clone(),
                    },
                    self.data
                        .frame_tangent
                        .as_ref()
                        .map(|tangent| (&tangent.x[..], &tangent.y[..])),
                    &self.data.tangent_distance,
                    &self.data.translation_x,
                    &self.data.translation_y,
                )
            }
        };
        if let Classification::Decided(bounds) = &result
            && let Ok(mut cache) = self.data.bounds_cache.lock()
            && cache
                .as_ref()
                .is_none_or(|(cached_policy, cached_steps, _)| {
                    cached_policy != policy || *cached_steps <= refinement_steps
                })
        {
            *cache = Some((*policy, refinement_steps, bounds.clone()));
        }
        result
    }

    pub(in crate::bezier_offset) fn axis_coordinate_order_to_real(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> Classification<std::cmp::Ordering> {
        let bounded = policy.bounded_exact_predicate_pass(|| {
            retained_bounds_axis_order_to_real(
                |steps| self.conservative_bounds_refined(steps, policy),
                axis,
                value,
                policy,
            )
        });
        if matches!(bounded, Classification::Decided(_)) {
            return bounded;
        }
        if let Ok(Classification::Decided(sign)) =
            policy.strict_predicate_pass(|| self.axis_residual_sign(axis, value, policy))
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "analytic-parallel-axis-order",
                "retained-parameter-sign",
            );
            return Classification::Decided(match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            });
        }
        retained_bounds_axis_order_to_real(
            |refinement_steps| self.conservative_bounds_refined(refinement_steps, policy),
            axis,
            value,
            policy,
        )
    }

    /// Compares a rational source image at a locally retained root with a
    /// point already expressible in that root's coefficient field. The
    /// homogeneous difference is a polynomial in the selected parameter;
    /// its defining relation and source pole proof remain authoritative.
    pub(in crate::bezier_offset) fn retained_parameter_axis_order_to_point(
        &self,
        other: &CurvePoint2,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<std::cmp::Ordering>>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.parallel.distance().zero_status() != ZeroKnowledge::Zero
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(Classification::Decided(None));
        }
        let BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) =
            &self.data.parameter
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(authority) = parameter.polynomial_authority() else {
            return Ok(Classification::Decided(None));
        };
        // Only import the older point. Requiring a common field containing
        // the query's Cartesian image would first forget the very local
        // root/coefficient relation this predicate is meant to preserve.
        let other_source = match policy
            .bounded_exact_predicate_pass(|| recursive_projective_point_source(other, policy))?
        {
            Classification::Decided(Some(source)) => source,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let Some(other_coordinates) =
            recursive_projective_point_source_in_field(&authority.field, &other_source)
        else {
            return Ok(Classification::Decided(None));
        };
        let other_weight_sign = match recursive_projective_evidence_denominator_sign(other, policy)?
        {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let weight_sign = match self.parameter_polynomial_sign(weight, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (coordinate, translation, other_coordinate) = match axis {
            Axis2::X => (
                source.x_numerator,
                &self.data.translation_x,
                &other_coordinates.x,
            ),
            Axis2::Y => (
                source.y_numerator,
                &self.data.translation_y,
                &other_coordinates.y,
            ),
        };
        let coordinate = polynomial_add(coordinate, &polynomial_scale(weight, translation));
        let zero = Real::zero();
        let difference = (0..coordinate.len().max(weight.len()))
            .map(|power| {
                other_coordinates
                    .denominator
                    .scale(coordinate.get(power).unwrap_or(&zero))?
                    .subtract(&other_coordinate.scale(weight.get(power).unwrap_or(&zero))?)
            })
            .collect::<Option<Vec<_>>>();
        let Some(difference) = difference else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(parameter
            .recursive_polynomial_sign(&difference, policy)?
            .map(|sign| {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-point-axis-order",
                    "retained-coefficient-field",
                );
                Some(
                    match product_sign(sign, product_sign(weight_sign, other_weight_sign)) {
                        RealSign::Negative => std::cmp::Ordering::Less,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Greater,
                    },
                )
            }))
    }

    pub(in crate::bezier_offset) fn parameter_polynomial_sign(
        &self,
        polynomial: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        match &self.data.parameter {
            BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                signed_coefficients_at_parameter(polynomial, parameter, policy)
            }
            BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => parameter
                .predicate_sign(&bivariate_outer_product(&[Real::one()], polynomial), policy),
            BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                parameter.polynomial_sign(polynomial, policy)
            }
        }
    }

    /// Signs one coordinate minus `value` in the source parameter field.
    /// For A/W + B/sqrt(S), the numerator is A*sqrt(S) + B*W and the
    /// denominator has the sign of W. Neither coordinate needs publication.
    pub(in crate::bezier_offset) fn axis_residual_sign(
        &self,
        axis: Axis2,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "analytic-parallel axis predicate crossed retained policies".into(),
            ));
        }
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let weight_sign = match self.parameter_polynomial_sign(weight, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (coordinate, translation) = match axis {
            Axis2::X => (source.x_numerator, &self.data.translation_x),
            Axis2::Y => (source.y_numerator, &self.data.translation_y),
        };
        let rational = polynomial_add(
            coordinate,
            &polynomial_scale(weight, &(translation - value)),
        );
        if self.data.parallel.distance().zero_status() == ZeroKnowledge::Zero
            && self.data.tangent_distance.zero_status() == ZeroKnowledge::Zero
        {
            return Ok(self
                .parameter_polynomial_sign(&rational, policy)?
                .map(|sign| product_sign(sign, weight_sign)));
        }
        let (tangent_x, tangent_y) = self.frame_tangent_power_basis()?;
        let frame_coordinate = match axis {
            Axis2::X => polynomial_subtract(
                &polynomial_scale(tangent_x, &self.data.tangent_distance),
                &polynomial_scale(tangent_y, self.data.parallel.distance()),
            ),
            Axis2::Y => polynomial_add(
                &polynomial_scale(tangent_x, self.data.parallel.distance()),
                &polynomial_scale(tangent_y, &self.data.tangent_distance),
            ),
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(tangent_x, tangent_x),
            &polynomial_multiply(tangent_y, tangent_y),
        );
        Ok(self
            .parameter_radical_sum_sign(
                &rational,
                &polynomial_multiply(weight, &frame_coordinate),
                &speed_squared,
                policy,
            )?
            .map(|sign| product_sign(sign, weight_sign)))
    }

    /// Signs `|point - self|^2 - radius_squared` in the retained source
    /// parameter field without adjoining the source-speed square root.
    pub(in crate::bezier_offset) fn circle_residual_sign_to_exact(
        &self,
        point: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "analytic-parallel point entered a circle predicate under a different policy"
                    .into(),
            ));
        }
        let source = self.data.parallel.source_power_basis()?;
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let translated_x = point.x() - &self.data.translation_x;
        let translated_y = point.y() - &self.data.translation_y;
        let delta_x =
            polynomial_subtract(&polynomial_scale(weight, &translated_x), source.x_numerator);
        let delta_y =
            polynomial_subtract(&polynomial_scale(weight, &translated_y), source.y_numerator);
        let normal_distance = self.data.parallel.distance();
        let tangent_distance = &self.data.tangent_distance;
        let constant = normal_distance * normal_distance + tangent_distance * tangent_distance
            - radius_squared;
        let rational = polynomial_add(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_scale(&polynomial_multiply(weight, weight), &constant),
        );
        match self.parameter_polynomial_sign(weight, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if normal_distance.zero_status() == ZeroKnowledge::Zero
            && tangent_distance.zero_status() == ZeroKnowledge::Zero
        {
            // A finite source point remains valid at a stationary parameter;
            // a zero displacement does not require a unit tangent there.
            return self.parameter_polynomial_sign(&rational, policy);
        }
        let (frame_tangent_x, frame_tangent_y) = self.frame_tangent_power_basis()?;
        let frame_x = polynomial_subtract(
            &polynomial_scale(frame_tangent_x, tangent_distance),
            &polynomial_scale(frame_tangent_y, normal_distance),
        );
        let frame_y = polynomial_add(
            &polynomial_scale(frame_tangent_x, normal_distance),
            &polynomial_scale(frame_tangent_y, tangent_distance),
        );
        let radical = polynomial_scale(
            &polynomial_multiply(
                weight,
                &polynomial_add(
                    &polynomial_multiply(&delta_x, &frame_x),
                    &polynomial_multiply(&delta_y, &frame_y),
                ),
            ),
            &Real::from(-2_i8),
        );
        let speed_squared = polynomial_add(
            &polynomial_multiply(frame_tangent_x, frame_tangent_x),
            &polynomial_multiply(frame_tangent_y, frame_tangent_y),
        );
        self.parameter_radical_sum_sign(&rational, &radical, &speed_squared, policy)
    }

    /// Signs A*sqrt(S) + B in the retained parameter field, selecting S > 0
    /// before squaring. Axis and circle predicates share this replay; equal
    /// magnitudes with equal signs never become a false conjugate-sheet zero.
    pub(in crate::bezier_offset) fn parameter_radical_sum_sign(
        &self,
        rational: &[Real],
        radical: &[Real],
        speed_squared: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sign = |polynomial: &[Real]| self.parameter_polynomial_sign(polynomial, policy);
        let speed_sign = match sign(speed_squared)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match speed_sign {
            RealSign::Positive => {}
            RealSign::Zero => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            RealSign::Negative => {
                return Err(CurveError::Topology(
                    "analytic-parallel point had negative source speed squared".into(),
                ));
            }
        }

        let rational_sign = match sign(rational)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radical_sign = match sign(radical)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match (rational_sign, radical_sign) {
            (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
                return Ok(Classification::Decided(sign));
            }
            (first, second) if first == second => {
                return Ok(Classification::Decided(first));
            }
            _ => {}
        }
        let magnitude = polynomial_subtract(
            &polynomial_multiply(&polynomial_multiply(rational, rational), speed_squared),
            &polynomial_multiply(radical, radical),
        );
        Ok(match sign(&magnitude)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(rational_sign),
            Classification::Decided(RealSign::Negative) => Classification::Decided(radical_sign),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    /// Compares through the source circle and one coordinate. On a circle,
    /// fixing one coordinate and the sign of the other radial coordinate
    /// uniquely selects a point. This keeps irrational source coefficients
    /// and selected parameters in their existing field.
    pub(in crate::bezier_offset) fn rational_circle_source_point_equality(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        if !policy.accepts_retained_policy(self.data.policy)
            || self.data.parallel.distance().zero_status() != ZeroKnowledge::Zero
            || self.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
        {
            return Ok(None);
        }
        let BezierParallelSource2::Rational(source) = self.data.parallel.source() else {
            return Ok(None);
        };
        let Classification::Decided(Some(circle)) =
            crate::arc_bezier::rational_bezier_circular_arc(source, policy)?
        else {
            return Ok(None);
        };
        let point = point.translated(-&self.data.translation_x, -&self.data.translation_y);
        match crate::classify::is_zero(
            &(point.distance_squared(circle.center()) - circle.radius_squared_ref()),
            policy,
        ) {
            Some(true) => {}
            Some(false) => return Ok(Some(false)),
            None => return Ok(None),
        }
        let basis = source.homogeneous_power_basis()?;
        let sign = |polynomial: &[Real]| self.parameter_polynomial_sign(polynomial, policy);
        let Classification::Decided(weight_sign @ (RealSign::Positive | RealSign::Negative)) =
            sign(&basis.weight)?
        else {
            return Ok(None);
        };
        for (coordinate, value, other, other_value, center) in [
            (
                &basis.x_numerator,
                point.x(),
                &basis.y_numerator,
                point.y(),
                circle.center().y(),
            ),
            (
                &basis.y_numerator,
                point.y(),
                &basis.x_numerator,
                point.x(),
                circle.center().x(),
            ),
        ] {
            if sign(&polynomial_subtract(
                coordinate,
                &polynomial_scale(&basis.weight, value),
            ))? != Classification::Decided(RealSign::Zero)
            {
                continue;
            }
            let Some(expected) = real_sign(&(other_value - center), policy) else {
                continue;
            };
            if let Classification::Decided(actual) = sign(&polynomial_subtract(
                other,
                &polynomial_scale(&basis.weight, center),
            ))? {
                return Ok(Some(product_sign(actual, weight_sign) == expected));
            }
        }
        Ok(None)
    }

    /// At one shared source parameter, tangent/normal displacement is the
    /// linear map `a I + d J` applied to the unit tangent. It is injective
    /// unless both displacements vanish. Compare the two unit directions in
    /// that parameter's field, including raw and reduced hodographs, without
    /// reconstructing a global parameter or either Cartesian coordinate.
    pub(in crate::bezier_offset) fn shared_parameter_point_equality(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<bool>>> {
        if self.data.parallel != other.data.parallel
            || self.data.tangent_distance != other.data.tangent_distance
            || self.data.translation_x != other.data.translation_x
            || self.data.translation_y != other.data.translation_y
            || !policy.accepts_retained_policy(self.data.policy)
            || !policy.accepts_retained_policy(other.data.policy)
        {
            return Ok(Classification::Decided(None));
        }
        policy.strict_predicate_pass(|| {
            if self.data.parameter != other.data.parameter {
                match self
                    .data
                    .parameter
                    .curve_parameter()
                    .same_value(&other.data.parameter.curve_parameter(), policy)?
                {
                    Classification::Decided(true) => {}
                    // A source can visit one point at distinct parameters.
                    // Only equality is a point certificate here.
                    Classification::Decided(false) => return Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let distance = self.data.parallel.distance();
            let tangent = &self.data.tangent_distance;
            let displacement_squared = distance * distance + tangent * tangent;
            if displacement_squared.zero_status() == ZeroKnowledge::Zero {
                return Ok(Classification::Decided(Some(true)));
            }
            let (first_x, first_y) = self.frame_tangent_power_basis()?;
            let (second_x, second_y) = other.frame_tangent_power_basis()?;
            let cross = polynomial_subtract(
                &polynomial_multiply(first_x, second_y),
                &polynomial_multiply(first_y, second_x),
            );
            let same_direction = match self.parameter_polynomial_sign(&cross, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => false,
                Classification::Decided(RealSign::Zero) => {
                    let dot = polynomial_add(
                        &polynomial_multiply(first_x, second_x),
                        &polynomial_multiply(first_y, second_y),
                    );
                    match self.parameter_polynomial_sign(&dot, policy)? {
                        Classification::Decided(RealSign::Positive) => true,
                        Classification::Decided(RealSign::Negative) => false,
                        // A zero frame has no unit direction. Its one-sided
                        // source-cusp meaning belongs to its retained frame.
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if same_direction {
                return Ok(Classification::Decided(Some(true)));
            }
            Ok(match real_sign(&displacement_squared, policy) {
                Some(RealSign::Zero) => Classification::Decided(Some(true)),
                Some(RealSign::Positive) => Classification::Decided(Some(false)),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            })
        })
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if let CurvePoint2(CurvePointData2::AnalyticParallel(other)) = other {
            if self == other {
                return Classification::Decided(true);
            }
            if let Ok(Classification::Decided(Some(equal))) =
                self.shared_parameter_point_equality(other, policy)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "analytic-point-equality",
                    "shared-parameter-unit-directions",
                );
                return Classification::Decided(equal);
            }
        }
        // Transporting a source and then evaluating its retained parameter
        // has the same meaning as transporting the selected point. Reuse
        // that relation before reconstructing the parameter's global field.
        if let CurvePoint2(CurvePointData2::Similarity(image)) = other
            && let CurvePoint2(CurvePointData2::AnalyticParallel(source)) = &image.data.source
            && policy.accepts_retained_policy(image.data.policy)
            && policy.accepts_retained_policy(source.data.policy)
            && policy.accepts_retained_policy(self.data.policy)
            && self.data.parameter == source.data.parameter
            && self.data.frame_tangent.is_none()
            && source.data.frame_tangent.is_none()
            && self.data.tangent_distance
                == &source.data.tangent_distance * image.data.transform.scale()
            && let Ok(parallel) = source
                .data
                .parallel
                .transform_similarity(&image.data.transform)
            && self.data.parallel == parallel
        {
            let (x, y) = image.data.transform.transform_vector_coordinates(
                &source.data.translation_x,
                &source.data.translation_y,
            );
            if self.data.translation_x == x && self.data.translation_y == y {
                return Classification::Decided(true);
            }
        }
        // A represented parameter still owns a source map whose coefficients
        // may cancel before evaluation. All parameter forms reuse that map
        // and its positive-speed frame in the shared field; independently
        // materialized coordinates are only a later fallback.
        let retained = CurvePoint2::from(self.clone());
        if let Ok(Classification::Decided(Some(equal))) =
            recursive_projective_point_evidence_equality(&retained, other, policy)
        {
            return Classification::Decided(equal);
        }
        retained_point_evidence_equality_by_refinement(&retained, other, policy)
    }
}
