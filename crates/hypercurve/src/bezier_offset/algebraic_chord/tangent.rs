//! Chord tangent relations, cross/dot signs and oriented sides.

use super::*;

impl BezierAlgebraicChord2 {
    /// Returns the original direction authority and whether this traversal is
    /// reversed relative to it. Exact parallel construction preserves this
    /// relation structurally, even when its normalized vector is not a pair of
    /// represented `Real`s.
    pub(in crate::bezier_offset) fn tangent_authority(&self) -> (&Self, bool) {
        let mut current = self;
        let mut reversed = false;
        loop {
            reversed ^= current.retained_support_orientation_is_reversed();
            let support = current.retained_support();
            let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (support.start(), support.end())
            else {
                return (support, reversed);
            };
            if !start.shares_carrier(end) || start.at_end == end.at_end {
                return (support, reversed);
            }
            // Displacement preserves the source tangent in endpoint order.
            // The retained support may itself be reversed even when the
            // current finite chord was constructed in traversal order.
            reversed ^= start.at_end;
            current = &start.data.source;
        }
    }

    /// Returns the direction of this support relative to one source-unit
    /// tangent at `parameter`. The two analytic endpoints share every term
    /// except tangent distance, so their difference is exactly the signed
    /// tangent displacement; no endpoint coordinates or selected fields need
    /// to be reconstructed.
    pub(in crate::bezier_offset) fn authored_source_tangent_displacement_sign(
        &self,
        parameter: &CurveParameter2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
        source_matches: impl Fn(&BezierParallelSource2) -> bool,
    ) -> Option<RealSign> {
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (self.start(), self.end())
        else {
            return None;
        };
        let source_matches = source_matches(start.data.parallel.source());
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!(
                "authored tangent identity parallel={} source={} parameter={} region={} frame={} translation={} policy={}",
                start.data.parallel == end.data.parallel,
                source_matches,
                start.data.parameter == end.data.parameter,
                start.data.parameter.matches_region_parameter(parameter),
                start.data.frame_tangent == end.data.frame_tangent
                    && start.data.frame_tangent.as_deref() == frame_tangent,
                start.data.translation_x == end.data.translation_x
                    && start.data.translation_y == end.data.translation_y,
                policy.accepts_retained_policy(start.data.policy)
                    && policy.accepts_retained_policy(end.data.policy),
            );
        }
        if !(start.data.parallel == end.data.parallel
            && source_matches
            && start.data.parameter == end.data.parameter
            && start.data.parameter.matches_region_parameter(parameter)
            && start.data.frame_tangent == end.data.frame_tangent
            && start.data.frame_tangent.as_deref() == frame_tangent
            && start.data.translation_x == end.data.translation_x
            && start.data.translation_y == end.data.translation_y
            && policy.accepts_retained_policy(start.data.policy)
            && policy.accepts_retained_policy(end.data.policy))
        {
            return None;
        }
        match real_sign(
            &(&end.data.tangent_distance - &start.data.tangent_distance),
            &CurveContext::STRICT,
        ) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => Some(sign),
            Some(RealSign::Zero) | None => None,
        }
    }

    /// Returns whether this support was authored as one nonzero source-unit
    /// tangent at `parameter`.
    pub(in crate::bezier_offset) fn is_authored_source_tangent_at_region_parameter(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        frame_tangent: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> bool {
        self.authored_source_tangent_displacement_sign(parameter, frame_tangent, policy, |source| {
            source == parallel.source()
        })
        .is_some()
    }

    /// Returns whether `point` is the zero-tangent-distance origin of this
    /// authored analytic tangent line.  Unlike the direction-only predicate
    /// above, incidence also requires the same parallel distance, frame, and
    /// translation so the two constructions share the exact affine origin.
    pub(in crate::bezier_offset) fn contains_authored_source_tangent_origin(
        &self,
        point: &BezierAnalyticParallelPoint2,
        policy: &CurveContext,
    ) -> bool {
        if !policy.accepts_retained_policy(point.data.policy)
            || point.data.tangent_distance.zero_status() != ZeroKnowledge::Zero
        {
            return false;
        }
        let (authority, _) = self.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (authority.start(), authority.end())
        else {
            return false;
        };
        start.data.parallel == end.data.parallel
            && start.data.parallel == point.data.parallel
            && start.data.parameter == end.data.parameter
            && start.data.parameter == point.data.parameter
            && start.data.frame_tangent == end.data.frame_tangent
            && start.data.frame_tangent == point.data.frame_tangent
            && start.data.translation_x == end.data.translation_x
            && start.data.translation_x == point.data.translation_x
            && start.data.translation_y == end.data.translation_y
            && start.data.translation_y == point.data.translation_y
            && policy.accepts_retained_policy(start.data.policy)
            && policy.accepts_retained_policy(end.data.policy)
            && matches!(
                real_sign(
                    &(&end.data.tangent_distance - &start.data.tangent_distance),
                    &CurveContext::STRICT,
                ),
                Some(RealSign::Negative | RealSign::Positive),
            )
    }

    pub(in crate::bezier_offset) fn shared_tangent_orientation(
        &self,
        other: &Self,
    ) -> Option<bool> {
        let (first, first_reversed) = self.tangent_authority();
        let (second, second_reversed) = other.tangent_authority();
        if let Some(support_reversed) = first
            .retained_support_orientation_to(second)
            .or_else(|| first.shares_retained_support(second).then_some(false))
        {
            return Some(first_reversed ^ support_reversed ^ second_reversed);
        }
        // Offset subsegments can retain different finite support descendants
        // while still descending from the same procedural normal translation.
        // That construction owns the same exact tangent relation without any
        // endpoint-field comparison.
        self.retained_normal_offset_tangent_reversal_to(other)
    }

    /// Whether `other` traverses a collinear support opposite to this chord.
    ///
    /// Construction provenance decides first. Equal parameter axes compare
    /// their monotone directions directly. Distinct axes cannot be compared
    /// through their flags: each flag describes a different coordinate.
    /// Every injective axis of the common line is injective for both chords,
    /// so this chord's endpoint order along `other`'s axis is strict and
    /// fixes the relative traversal exactly.
    pub(in crate::bezier_offset) fn collinear_traversal_reversed(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Some(reversed) = self.shared_tangent_orientation(other) {
            return Ok(Classification::Decided(reversed));
        }
        let other_axis = other.data.parameter_axis;
        if self.data.parameter_axis.axis == other_axis.axis {
            return Ok(Classification::Decided(
                self.data.parameter_axis.coordinate_increases != other_axis.coordinate_increases,
            ));
        }
        Ok(
            match Self::point_axis_order(self.start(), self.end(), other_axis.axis, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::Topology(
                        "a collinear chord was constant along an injective support axis".into(),
                    ));
                }
                Classification::Decided(order) => Classification::Decided(
                    (order == std::cmp::Ordering::Less) != other_axis.coordinate_increases,
                ),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Recognizes two finite subchords of the same retained radial line even
    /// when clipping replaced both endpoints. Each authority has direction
    /// `(a_end-a_start)(P-C)`; comparing the two exact scalar signs is enough
    /// to recover their relative traversal without endpoint incidence.
    pub(in crate::bezier_offset) fn retained_radial_tangent_reversal_to(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        let (first, first_reversed) = self.tangent_authority();
        let (second, second_reversed) = other.tangent_authority();
        #[cfg(test)]
        let evidence_kind = |point: &CurvePoint2| match point {
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
        let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first_start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first_end)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second_start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second_end)),
        ) = (first.start(), first.end(), second.start(), second.end())
        else {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                eprintln!(
                    "radial tangent ancestry kinds=({},{})/({},{}) reversals={first_reversed}/{second_reversed}",
                    evidence_kind(first.start()),
                    evidence_kind(first.end()),
                    evidence_kind(second.start()),
                    evidence_kind(second.end()),
                );
            }
            return Ok(None);
        };
        let shared_source = first_start
            .data
            .source
            .shares_exact_evidence(&second_start.data.source);
        let shared_circle =
            first_start.data.source.semicircle() == second_start.data.source.semicircle();
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
            && (!shared_source || !shared_circle)
        {
            let source = |point: &BezierAlgebraicCuspChordDerivedPoint2| match &point.data.source {
                BezierAlgebraicCuspDerivedPointSource2::Chord(point) => {
                    ("chord", Arc::as_ptr(&point.data) as usize)
                }
                BezierAlgebraicCuspDerivedPointSource2::Mapped { parameter, .. } => {
                    ("mapped", Arc::as_ptr(parameter) as usize)
                }
            };
            eprintln!(
                "radial tangent ancestry shared-source={shared_source} shared-circle={shared_circle} first={:?}/{:?} second={:?}/{:?}",
                source(first_start),
                source(first_end),
                source(second_start),
                source(second_end),
            );
        }
        if !shared_source || !shared_circle {
            return Ok(None);
        }
        let first_difference =
            first_start.common_untranslated_radial_difference_sign(first_end, policy)?;
        let second_difference =
            second_start.common_untranslated_radial_difference_sign(second_end, policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
            && (first_difference.is_none() || second_difference.is_none())
        {
            eprintln!(
                "radial tangent ancestry differences={first_difference:?}/{second_difference:?}"
            );
        }
        let first_sign = match first_difference {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) | None => return Ok(None),
        };
        let second_sign = match second_difference {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) | None => return Ok(None),
        };
        Ok(Some(
            first_reversed ^ second_reversed ^ (first_sign != second_sign),
        ))
    }

    pub(in crate::bezier_offset) fn tangent_relation_sign_by_refinement(
        &self,
        other: &Self,
        cross: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let zero = Real::zero();
        let one = Real::one();
        self.tangent_cross_dot_linear_combination_sign(
            other,
            if cross { &one } else { &zero },
            if cross { &zero } else { &one },
            policy,
        )
    }

    /// Cancels the anchors and positive speed denominators of two retained
    /// tangent displacements before signing their polynomial directions.
    /// Selected parameters remain authoritative; equality can identify one
    /// shared scalar even when the two witnesses use different root carriers.
    pub(in crate::bezier_offset) fn analytic_tangent_pair_linear_combination_sign(
        &self,
        other: &Self,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(first)),
            CurvePoint2(CurvePointData2::AnalyticParallel(first_end)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second_end)),
        ) = (self.start(), self.end(), other.start(), other.end())
        else {
            return Ok(None);
        };
        policy.bounded_exact_predicate_pass(|| {
            let mut orientation = RealSign::Positive;
            for (start, end) in [(first, first_end), (second, second_end)] {
                let Some(Classification::Decided(sign)) =
                    start.shared_tangent_displacement_sign(end, policy)
                else {
                    return Ok(None);
                };
                orientation = product_sign(orientation, sign);
            }
            let (first_x, first_y) = first.frame_tangent_power_basis()?;
            let (second_x, second_y) = second.frame_tangent_power_basis()?;
            let first_parameter = first.data.parameter.curve_parameter();
            let second_parameter = second.data.parameter.curve_parameter();
            let strict = policy.strict_counterpart();
            let sign = if matches!(
                first_parameter.same_value(&second_parameter, &strict)?,
                Classification::Decided(true)
            ) {
                let cross = polynomial_subtract(
                    &polynomial_multiply(first_x, second_y),
                    &polynomial_multiply(first_y, second_x),
                );
                let dot = polynomial_add(
                    &polynomial_multiply(first_x, second_x),
                    &polynomial_multiply(first_y, second_y),
                );
                first.parameter_polynomial_sign(
                    &polynomial_add(
                        &polynomial_scale(&cross, cross_scale),
                        &polynomial_scale(&dot, dot_scale),
                    ),
                    &strict,
                )?
            } else if let (Some(first_parameter), Some(second_parameter)) = (
                first_parameter.as_bezier_parameter(),
                second_parameter.as_bezier_parameter(),
            ) {
                let cross = bivariate_subtract(
                    &bivariate_outer_product(first_x, second_y),
                    &bivariate_outer_product(first_y, second_x),
                );
                let dot = bivariate_add(
                    &bivariate_outer_product(first_x, second_x),
                    &bivariate_outer_product(first_y, second_y),
                );
                signed_bivariate_at_parameter_pair(
                    &bivariate_add(
                        &bivariate_scale(cross, cross_scale),
                        &bivariate_scale(dot, dot_scale),
                    ),
                    first_parameter,
                    second_parameter,
                    &strict,
                )?
            } else {
                return Ok(None);
            };
            Ok(match sign {
                Classification::Decided(sign) => Some(product_sign(orientation, sign)),
                Classification::Uncertain(_) => None,
            })
        })
    }

    pub(in crate::bezier_offset) fn certified_axis_tangent_relation_sign(
        &self,
        other: &Self,
        cross: bool,
    ) -> Option<RealSign> {
        let (first, second) = self
            .certified_axis_direction()
            .zip(other.certified_axis_direction())?;
        let ((first_x, first_y), (second_x, second_y)) =
            (first.cardinal_components(), second.cardinal_components());
        let value = if cross {
            first_x * second_y - first_y * second_x
        } else {
            first_x * second_x + first_y * second_y
        };
        Some(match value.cmp(&0) {
            std::cmp::Ordering::Less => RealSign::Negative,
            std::cmp::Ordering::Equal => RealSign::Zero,
            std::cmp::Ordering::Greater => RealSign::Positive,
        })
    }

    /// Returns either `chord x self` or `chord dot self` when this chord is
    /// the unit-tangent witness authored by a recursive rational contact.
    pub(in crate::bezier_offset) fn retained_rational_tangent_relation_sign_to(
        &self,
        chord: &Self,
        cross: bool,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let (authority, reversed) = self.tangent_authority();
        let (chord, chord_reversed) = chord.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (authority.start(), authority.end())
        else {
            return None;
        };
        start
            .shared_rational_tangent_relation_sign_to_chord(end, chord, cross, policy)
            .map(|result| {
                result.map(|classification| {
                    classification.map(|sign| {
                        if reversed ^ chord_reversed {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }
                    })
                })
            })
    }

    pub(in crate::bezier_offset) fn retained_rational_tangent_cross_sign_to(
        &self,
        chord: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        self.retained_rational_tangent_relation_sign_to(chord, true, policy)
    }

    pub(in crate::bezier_offset) fn retained_rational_tangent_dot_sign_to(
        &self,
        chord: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        self.retained_rational_tangent_relation_sign_to(chord, false, policy)
    }

    /// Returns `self x tangent` when `self` descends from two unrotated
    /// concentric radial images of one retained circle contact and `tangent`
    /// is the source-tangent chord authored at that same contact.
    pub(in crate::bezier_offset) fn retained_radial_tangent_cross_sign_to(
        &self,
        tangent: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let (radial, radial_reversed) = self.tangent_authority();
        let (tangent, tangent_reversed) = tangent.tangent_authority();
        let (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(start)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(end)),
        ) = (radial.start(), radial.end())
        else {
            return None;
        };
        let radial_sign = match start.common_untranslated_radial_difference_sign(end, policy) {
            Ok(Some(sign)) => sign,
            Ok(None) => return None,
            Err(error) => return Some(Err(error)),
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("retained radial tangent relation scale={radial_sign:?}");
        }
        let sign = start
            .data
            .source
            .radial_cross_authored_tangent_sign(tangent, policy)?;
        Some(sign.map(|classification| {
            classification.map(|sign| {
                let sign = product_sign(sign, radial_sign);
                if radial_reversed ^ tangent_reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }
            })
        }))
    }

    /// Returns only construction-owned or compact represented evidence for
    /// `self x other`. This deliberately excludes endpoint-box refinement and
    /// recursive norms so callers can exhaust provenance before selecting a
    /// cold general authority.
    pub(in crate::bezier_offset) fn retained_tangent_cross_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        if let Some(sign) = other.retained_rational_tangent_cross_sign_to(self, policy) {
            return Some(sign);
        }
        if let Some(sign) = self.retained_rational_tangent_cross_sign_to(other, policy) {
            return Some(sign.map(|classification| {
                classification.map(|sign| product_sign(sign, RealSign::Negative))
            }));
        }
        if let Some(sign) = self.retained_radial_tangent_cross_sign_to(other, policy) {
            return Some(sign);
        }
        if let Some(sign) = other.retained_radial_tangent_cross_sign_to(self, policy) {
            return Some(sign.map(|classification| {
                classification.map(|sign| product_sign(sign, RealSign::Negative))
            }));
        }
        if self.shared_tangent_orientation(other).is_some() {
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, true) {
            return Some(Ok(Classification::Decided(sign)));
        }
        if let (Some(first), Some(second)) = (
            self.data.certified_unit_tangent.as_ref(),
            other.data.certified_unit_tangent.as_ref(),
        ) && Arc::ptr_eq(first, second)
        {
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        let (Some(first), Some(second)) = (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) else {
            return None;
        };
        let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
        cross
            .refine_sign_until(-512)
            .map(|sign| Ok(Classification::Decided(sign)))
    }

    /// Signs `cross_scale * (self x other) + dot_scale * (self dot other)`.
    /// Both products share the same positive normalization factor, so the
    /// unnormalized endpoint differences are authoritative. Independent
    /// selected endpoint fields are refined in place and only an
    /// APPROXIMATE_512 query may terminate an unresolved equality.
    pub(crate) fn tangent_cross_dot_linear_combination_sign(
        &self,
        other: &Self,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        // Boolean clipping and offset trimming replace finite endpoints but
        // retain an exact direction authority. Evaluate every tangent
        // relation on those smallest authored supports before inspecting the
        // descendant endpoints: both cross and dot acquire the same sign
        // under one reversal, so the requested linear combination needs only
        // one final orientation correction. This also prevents a chord-pair
        // endpoint from being reconstructed merely to rediscover its
        // ancestor's tangent.
        let (first_authority, first_reversed) = self.tangent_authority();
        let (second_authority, second_reversed) = other.tangent_authority();
        if !Arc::ptr_eq(&first_authority.data, &self.data)
            || !Arc::ptr_eq(&second_authority.data, &other.data)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "retained-direction-authority",
            );
            return first_authority
                .tangent_cross_dot_linear_combination_sign(
                    second_authority,
                    cross_scale,
                    dot_scale,
                    policy,
                )
                .map(|classification| {
                    classification.map(|sign| {
                        if first_reversed ^ second_reversed {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }
                    })
                });
        }
        if let Some(sign) = self.analytic_tangent_pair_linear_combination_sign(
            other,
            cross_scale,
            dot_scale,
            policy,
        )? {
            return Ok(Classification::Decided(sign));
        }
        if dot_scale.zero_status() == ZeroKnowledge::Zero
            && let Some(cross_scale_sign @ (RealSign::Negative | RealSign::Positive)) =
                real_sign(cross_scale, &CurveContext::STRICT)
            && let Some(sign) = self.retained_tangent_cross_sign(other, policy)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "retained-construction-cross",
            );
            return sign.map(|classification| {
                classification.map(|sign| product_sign(sign, cross_scale_sign))
            });
        }
        if let Some(reversed) = self.shared_tangent_orientation(other) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                if reversed {
                    "shared-support-reversed"
                } else {
                    "shared-support-forward"
                },
            );
            let value = if reversed {
                -dot_scale.clone()
            } else {
                dot_scale.clone()
            };
            return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        if let (Some(first), Some(second)) = (
            self.data.certified_unit_tangent.as_ref(),
            other.data.certified_unit_tangent.as_ref(),
        ) && Arc::ptr_eq(first, second)
        {
            // Exact parallel translations clone this shared construction
            // authority.  Its vector is certified unit length, so the cross
            // is exactly zero and the dot exactly one; asking the scalar DAG
            // to rediscover x*y-y*x would lose the commutative identity.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-unit-tangent-authority",
            );
            return Ok(real_sign(dot_scale, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        // This is a structural fast path, not a prerequisite for the complete
        // tangent predicate below.  Do not invoke `axis_direction` here to
        // rediscover an unretained cardinal fact from composite endpoints:
        // that can build the same large recursive field the general
        // endpoint-difference refinement deliberately avoids.
        if let (Some(first), Some(second)) = (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) {
            let structural_orientation = if first.0 == second.0 && first.1 == second.1 {
                Some(false)
            } else if first.0 == -second.0.clone() && first.1 == -second.1.clone() {
                Some(true)
            } else {
                None
            };
            if let Some(reversed) = structural_orientation {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    if reversed {
                        "equal-represented-unit-tangents-reversed"
                    } else {
                        "equal-represented-unit-tangents-forward"
                    },
                );
                let value = if reversed {
                    -dot_scale.clone()
                } else {
                    dot_scale.clone()
                };
                return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    Classification::Decided,
                ));
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "represented-unit-tangents",
            );
            let cross = Real::diff_of_products(&first.0, &second.1, &first.1, &second.0);
            let dot = &first.0 * &second.0 + &first.1 * &second.1;
            let value = cross_scale * cross + dot_scale * dot;
            return Ok(real_sign(&value, &CurveContext::STRICT).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        match (
            self.certified_unit_tangent(),
            other.certified_unit_tangent(),
        ) {
            (Some(first), None) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "represented-retained-linear-form",
                );
                return other.tangent_cross_dot_vector_linear_combination_sign(
                    &first,
                    cross_scale,
                    dot_scale,
                    policy,
                );
            }
            (None, Some(second)) => {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "retained-represented-linear-form",
                );
                return self.tangent_cross_dot_vector_linear_combination_sign(
                    &second,
                    &(-cross_scale.clone()),
                    dot_scale,
                    policy,
                );
            }
            (Some(_), Some(_)) => {
                unreachable!("the represented pair was handled above")
            }
            (None, None) => {}
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-tangent-relation",
            "refinement",
        );
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let bounds = |chord: &Self| {
                let start = algebraic_chord_endpoint_local_bounds_refined(
                    chord.start(),
                    refinement_steps,
                    policy,
                );
                let end = algebraic_chord_endpoint_local_bounds_refined(
                    chord.end(),
                    refinement_steps,
                    policy,
                );
                match (start, end) {
                    (Classification::Decided(start), Classification::Decided(end)) => {
                        Some((start, end))
                    }
                    _ => None,
                }
            };
            let (Some((first_start, first_end)), Some((second_start, second_end))) =
                (bounds(self), bounds(other))
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta = |start: &Aabb2, end: &Aabb2, axis| {
                real_interval_from_axis(end, axis).subtract(&real_interval_from_axis(start, axis))
            };
            let first_x = delta(&first_start, &first_end, Axis2::X);
            let first_y = delta(&first_start, &first_end, Axis2::Y);
            let second_x = delta(&second_start, &second_end, Axis2::X);
            let second_y = delta(&second_start, &second_end, Axis2::Y);
            let strict = &CurveContext::STRICT;
            let cross = first_x.multiply(&second_y).and_then(|first| {
                first_y
                    .multiply(&second_x)
                    .map(|second| first.subtract(&second))
            });
            let dot = first_x
                .multiply(&second_x)
                .and_then(|first| first_y.multiply(&second_y).map(|second| first.add(&second)));
            let scale = |value: RealInterval, scale: &Real| {
                value.multiply(&RealInterval {
                    lower: scale.clone(),
                    upper: scale.clone(),
                })
            };
            let value = cross
                .and_then(|cross| scale(cross, cross_scale))
                .and_then(|cross| {
                    dot.and_then(|dot| scale(dot, dot_scale))
                        .map(|dot| cross.add(&dot))
                });
            let Some(value) = value else {
                continue;
            };
            if compare_reals(&value.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            if compare_reals(&value.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            if compare_reals(&value.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&value.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }

    pub(crate) fn tangent_cross_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, true) {
            return Ok(Classification::Decided(sign));
        }
        self.tangent_relation_sign_by_refinement(other, true, policy)
    }

    /// Cancels one structurally shared vertex before signing two adjacent
    /// chord tangents. Offset corner construction is the consumer: its source
    /// loop has already certified endpoint connectivity, while the general
    /// chord-pair kernel deliberately keeps its established four-endpoint
    /// dispatch for unrelated Boolean pairs.
    pub(crate) fn tangent_cross_sign_with_shared_endpoint(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        let first_points = [self.start(), self.end()];
        let second_points = [other.start(), other.end()];
        let (first_index, second_index) =
            first_points
                .iter()
                .enumerate()
                .find_map(|(first_index, first)| {
                    second_points
                        .iter()
                        .enumerate()
                        .find_map(|(second_index, second)| {
                            (first.shares_storage(second) || *first == *second)
                                .then_some((first_index, second_index))
                        })
                })?;
        // Split, reversed, coalesced, and parallel-translated descendants can
        // retain one exact tangent authority even when their independently
        // selected endpoint fields make the oriented-area replay expensive.
        // A shared tangent authority is already a complete zero-cross
        // certificate; the orientation only affects the dot product.
        if self.shared_tangent_orientation(other).is_some() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-endpoint-shared-tangent-authority",
            );
            return Some(Ok(Classification::Decided(RealSign::Zero)));
        }
        Some((|| {
            let shared_point = first_points[first_index];
            let first_other = first_points[1 - first_index];
            let second_other = second_points[1 - second_index];
            // Certified enclosures of the three vertices decide a clearly
            // turning corner before any structural side replay, which can
            // promote recursive endpoint coordinates at great cost.
            if let Some(mut sign) =
                enclosure_orientation_sign(shared_point, first_other, second_other, policy)
            {
                if first_index != second_index {
                    sign = product_sign(sign, RealSign::Negative);
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-tangent-relation",
                    "shared-endpoint-enclosure",
                );
                return Ok(Classification::Decided(sign));
            }
            let reverse_side = |side| match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            };
            let recursive_contact_side = match (first_other, second_other) {
                (CurvePoint2(CurvePointData2::AnalyticParallel(contact)), endpoint) => contact
                    .recursive_chord_contact_to_endpoint_oriented_side(
                        endpoint,
                        shared_point,
                        policy,
                    ),
                (endpoint, CurvePoint2(CurvePointData2::AnalyticParallel(contact))) => contact
                    .recursive_chord_contact_to_endpoint_oriented_side(
                        endpoint,
                        shared_point,
                        policy,
                    )
                    .map(|result| result.map(|classification| classification.map(reverse_side))),
                _ => None,
            };
            if let Some(side) = recursive_contact_side
                && let Classification::Decided(side) = side?
            {
                let mut sign = match side {
                    crate::classify::LineSide::Left => RealSign::Positive,
                    crate::classify::LineSide::On => RealSign::Zero,
                    crate::classify::LineSide::Right => RealSign::Negative,
                };
                if first_index != second_index {
                    sign = product_sign(sign, RealSign::Negative);
                }
                return Ok(Classification::Decided(sign));
            }
            if let (
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
            ) = (first_other, second_other)
                && let Some(side) = first.common_untranslated_radial_line_oriented_side(
                    second,
                    shared_point,
                    policy,
                )?
            {
                let mut sign = match side {
                    crate::classify::LineSide::Left => RealSign::Positive,
                    crate::classify::LineSide::On => RealSign::Zero,
                    crate::classify::LineSide::Right => RealSign::Negative,
                };
                if first_index != second_index {
                    sign = product_sign(sign, RealSign::Negative);
                }
                return Ok(Classification::Decided(sign));
            }
            match Self::try_new(first_other.clone(), second_other.clone(), policy) {
                Err(CurveError::ZeroLengthLine) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Ok(Classification::Decided(diagonal)) => {
                    if let Classification::Decided(side) =
                        diagonal.oriented_support_side(shared_point, policy)?
                    {
                        let mut sign = match side {
                            crate::classify::LineSide::Left => RealSign::Positive,
                            crate::classify::LineSide::On => RealSign::Zero,
                            crate::classify::LineSide::Right => RealSign::Negative,
                        };
                        if first_index != second_index {
                            sign = product_sign(sign, RealSign::Negative);
                        }
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-tangent-relation",
                            "shared-endpoint-diagonal",
                        );
                        return Ok(Classification::Decided(sign));
                    }
                }
                Ok(Classification::Uncertain(_)) | Err(_) => {}
            }
            let other_point = second_points[1 - second_index];
            let side = match self.represented_oriented_side(other_point, policy)? {
                Classification::Decided(side) => side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut sign = match side {
                crate::classify::LineSide::Left => RealSign::Positive,
                crate::classify::LineSide::On => RealSign::Zero,
                crate::classify::LineSide::Right => RealSign::Negative,
            };
            if second_index == 1 {
                sign = product_sign(sign, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-relation",
                "shared-endpoint-oriented-area",
            );
            Ok(Classification::Decided(sign))
        })())
    }

    pub(crate) fn tangent_dot_sign(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(sign) = self.certified_axis_tangent_relation_sign(other, false) {
            return Ok(Classification::Decided(sign));
        }
        self.tangent_relation_sign_by_refinement(other, false, policy)
    }

    /// Signs one exact linear form of this chord's traversal tangent.
    ///
    /// Ordinary algebraic endpoint images use their retained polynomial
    /// predicates first, including exact zero. Unresolved forms reuse the
    /// oriented support's recursive field before interval refinement reaches
    /// the APPROXIMATE_512 terminal equality policy.
    pub(in crate::bezier_offset) fn tangent_linear_form_sign(
        &self,
        coefficient_x: &Real,
        coefficient_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        if let Some(tangent) = self.certified_unit_tangent() {
            let value = &tangent.0 * coefficient_x + &tangent.1 * coefficient_y;
            return Ok(real_sign(&value, policy).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        // Splits retain the same oriented straight support. Its original
        // endpoint field owns the tangent; later contact endpoints only bound
        // the finite segment and need no refinement for this query.
        let (support, reversed) = self.smallest_incidence_support();
        support.validate_policy(policy)?;
        let mut endpoints = support.direction_endpoints(policy);
        if reversed {
            endpoints.swap(0, 1);
        }
        let [start_point, end_point] = endpoints;
        if let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (start_point, end_point)
            && let Some(sign) = start.shared_tangent_displacement_linear_form_sign(
                end,
                coefficient_x,
                coefficient_y,
                policy,
            )
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-tangent-linear-form",
                "retained-unit-displacement",
            );
            return sign;
        }
        let exact_endpoint_sign = policy.strict_predicate_pass(|| -> CurveResult<_> {
            Ok(match (start_point, end_point) {
                (
                    CurvePoint2(CurvePointData2::Exact(start)),
                    CurvePoint2(CurvePointData2::Exact(end)),
                ) => real_sign(
                    &(coefficient_x * (end.x() - start.x())
                        + coefficient_y * (end.y() - start.y())),
                    policy,
                )
                .map(Classification::Decided),
                (
                    CurvePoint2(CurvePointData2::Algebraic(start)),
                    CurvePoint2(CurvePointData2::Algebraic(end)),
                ) => {
                    let start = match start.predicate_evaluator(policy)? {
                        Classification::Decided(start) => start,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    let end = match end.predicate_evaluator(policy)? {
                        Classification::Decided(end) => end,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(signed_algebraic_point_linear_difference(
                        &end,
                        &start,
                        coefficient_x,
                        coefficient_y,
                        policy,
                    )?)
                }
                (
                    CurvePoint2(CurvePointData2::Exact(start)),
                    CurvePoint2(CurvePointData2::Algebraic(end)),
                ) => {
                    let end = match end.predicate_evaluator(policy)? {
                        Classification::Decided(end) => end,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(
                        end.homogeneous_linear_difference_sign(
                            start.x(),
                            start.y(),
                            &Real::one(),
                            coefficient_x,
                            coefficient_y,
                            RealSign::Positive,
                            policy,
                        )?
                        .map(|sign| product_sign(sign, RealSign::Negative)),
                    )
                }
                (
                    CurvePoint2(CurvePointData2::Algebraic(start)),
                    CurvePoint2(CurvePointData2::Exact(end)),
                ) => {
                    let start = match start.predicate_evaluator(policy)? {
                        Classification::Decided(start) => start,
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    };
                    Some(start.homogeneous_linear_difference_sign(
                        end.x(),
                        end.y(),
                        &Real::one(),
                        coefficient_x,
                        coefficient_y,
                        RealSign::Positive,
                        policy,
                    )?)
                }
                _ => None,
            })
        })?;
        if let Some(Classification::Decided(sign)) = exact_endpoint_sign {
            return Ok(Classification::Decided(sign));
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(start), Classification::Decided(end)) = (
                algebraic_chord_endpoint_bounds_refined(start_point, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(end_point, refinement_steps, policy),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let delta_x = real_interval_from_axis(&end, Axis2::X)
                .subtract(&real_interval_from_axis(&start, Axis2::X));
            let delta_y = real_interval_from_axis(&end, Axis2::Y)
                .subtract(&real_interval_from_axis(&start, Axis2::Y));
            let coefficient_x = RealInterval {
                lower: coefficient_x.clone(),
                upper: coefficient_x.clone(),
            };
            let coefficient_y = RealInterval {
                lower: coefficient_y.clone(),
                upper: coefficient_y.clone(),
            };
            let strict = &CurveContext::STRICT;
            let value = delta_x.multiply(&coefficient_x).and_then(|first| {
                delta_y
                    .multiply(&coefficient_y)
                    .map(|second| first.add(&second))
            });
            let Some(value) = value else {
                continue;
            };
            if compare_reals(&value.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            if compare_reals(&value.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            if compare_reals(&value.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&value.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
        // Preserve every native interval decision before adjoining fields.
        // Unresolved forms, including correlated zeros with arbitrary exact
        // coefficients, reuse the existing projective direction authority
        // before an approximate terminal. Procedural offsets contribute their
        // original direction, without a cancelled normal.
        if let Classification::Decided(Some(frame)) = policy.strict_predicate_pass(|| {
            support.recursive_projective_endpoints_with_direction(policy)
        })? {
            let [start, end] = frame.direction_endpoints;
            let value = (|| {
                let (x, y, _) = end.difference_numerators(&start)?;
                x.scale(coefficient_x)?.add(&y.scale(coefficient_y)?)
            })();
            if let Some(value) = value
                && let Classification::Decided(sign) =
                    policy.strict_predicate_pass(|| value.sign(policy))?
            {
                return Ok(Classification::Decided(if reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }

    pub(in crate::bezier_offset) fn tangent_relation_to_vector_sign(
        &self,
        vector: &(Real, Real),
        cross: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let (coefficient_x, coefficient_y) = if cross {
            (vector.1.clone(), -vector.0.clone())
        } else {
            (vector.0.clone(), vector.1.clone())
        };
        self.tangent_linear_form_sign(&coefficient_x, &coefficient_y, policy)
    }

    /// Signs `cross_scale * (anchor x tangent) +
    /// dot_scale * (anchor dot tangent)` without normalizing this chord.
    pub(crate) fn tangent_cross_dot_vector_linear_combination_sign(
        &self,
        anchor: &(Real, Real),
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let coefficient_x = dot_scale * &anchor.0 - cross_scale * &anchor.1;
        let coefficient_y = cross_scale * &anchor.0 + dot_scale * &anchor.1;
        self.tangent_linear_form_sign(&coefficient_x, &coefficient_y, policy)
    }

    /// Signs `cross_scale * (T_chord x T_parallel) + dot_scale *
    /// (T_chord dot T_parallel)` at one retained analytic parameter.
    ///
    /// The two chord endpoint fields and analytic source parameter stay on
    /// separate tensor axes. Projective denominator and parallel derivative
    /// scale signs are restored only after the exact trivariate linear form is
    /// signed, so the result is valid for ordinary and retained-offset chord
    /// carriers under either terminal policy.
    pub(crate) fn tangent_cross_dot_parallel_linear_combination_sign(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let source_sign = match self.tangent_cross_dot_parallel_source_linear_combination_sign(
            parallel,
            parameter,
            cross_scale,
            dot_scale,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let scale = match parallel.parallel_derivative_scale_sign(parameter, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(product_sign(source_sign, scale)))
    }

    /// Signs the same chord relation against the parallel's source tangent.
    /// Circle normal frames use this direction even where the parallel's
    /// derivative reverses. The traversal predicate above applies that scale
    /// only when the actual parallel tangent is requested.
    pub(in crate::bezier_offset) fn tangent_cross_dot_parallel_source_linear_combination_sign(
        &self,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        // A retained oriented unit tangent is already the complete chord-side
        // authority for this predicate. Keep the parallel parameter as the
        // only algebraic axis instead of rebuilding two endpoint fields and a
        // trivariate support system merely to recover the same direction.
        if let Some((tangent_x, tangent_y)) = self.certified_unit_tangent() {
            let source_sign = match parallel
                .vector_source_tangent_cross_dot_linear_combination_sign(
                    parameter,
                    &tangent_x,
                    &tangent_y,
                    cross_scale,
                    dot_scale,
                    policy,
                )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(source_sign));
        }
        // Retained endpoint fields also support native and exact target
        // parameters. A local root keeps its defining coefficient relations;
        // a native target joins this field only for the requested sign. The
        // oriented support line is (-dy, dx, c) up to positive scale.
        if let Some(line) = self.recursive_projective_support_line(policy)? {
            let differential = parallel.differential()?;
            let coefficients = (|| {
                let x_scale = line.x.scale(cross_scale)?.add(&line.y.scale(dot_scale)?)?;
                let y_scale = line
                    .y
                    .scale(cross_scale)?
                    .subtract(&line.x.scale(dot_scale)?)?;
                recursive_quadratic_polynomial_combine(
                    &differential
                        .tangent_x
                        .iter()
                        .map(|value| x_scale.scale(value))
                        .collect::<Option<Vec<_>>>()?,
                    &differential
                        .tangent_y
                        .iter()
                        .map(|value| y_scale.scale(value))
                        .collect::<Option<Vec<_>>>()?,
                    false,
                )
            })();
            if let Some(coefficients) = coefficients {
                let sign = if let Some(root) = parameter.as_recursive_projective() {
                    root.recursive_polynomial_sign_joined(&coefficients, policy)?
                } else {
                    recursive_projective_polynomial_sign_at_parameter(
                        &line.x.field(),
                        &coefficients,
                        parameter,
                        policy,
                    )?
                };
                if let Classification::Decided(sign) = sign {
                    return Ok(Classification::Decided(sign));
                }
            }
        }
        // The independent trivariate fallback needs an ordinary root axis;
        // promotion is demand-driven and does not replace the retained root.
        let parameter = match promote_curve_region_bezier_parameter(parameter, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let support = match self.independent_support_system(policy)? {
            Classification::Decided(support) => support,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let differential = parallel.differential()?;
        let Some(line_x) = trivariate_from_axis_bivariate_coefficients(
            std::slice::from_ref(&support.line_x),
            2,
            [0, 1],
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(line_y) = trivariate_from_axis_bivariate_coefficients(
            std::slice::from_ref(&support.line_y),
            2,
            [0, 1],
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(tangent_x) =
            TrivariatePolynomial::from_axis_polynomial_or_zero(&differential.tangent_x, 2)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(tangent_y) =
            TrivariatePolynomial::from_axis_polynomial_or_zero(&differential.tangent_y, 2)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(cross) = TrivariatePolynomial::sum_products(&[
            (&line_x, &tangent_y, false),
            (&line_y, &tangent_x, true),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(dot) = TrivariatePolynomial::sum_products(&[
            (&line_x, &tangent_x, false),
            (&line_y, &tangent_y, false),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(relation) = cross
            .scale(cross_scale)
            .and_then(|cross| dot.scale(dot_scale).and_then(|dot| cross.add(&dot)))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let relation = trivariate_reduce_parameter_pair_relations(
            &relation,
            &support.first_parameter,
            &support.second_parameter,
        )
        .unwrap_or(relation);
        let source_sign = match trivariate_parameter_triple_sign_by_refinement(
            &relation,
            &support.first_parameter,
            &support.second_parameter,
            &parameter,
            policy,
        )? {
            Classification::Decided(sign) => product_sign(sign, support.chord_denominator_sign),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(source_sign))
    }

    pub(crate) fn tangent_cross_vector_sign(
        &self,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.tangent_relation_to_vector_sign(vector, true, policy)
    }

    pub(crate) fn tangent_dot_vector_sign(
        &self,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.tangent_relation_to_vector_sign(vector, false, policy)
    }

    /// Publishes this oriented affine support as homogeneous line
    /// coefficients `(a, b, c)`. Analytic tangent supports use their direct
    /// differential identity, while other procedural supports retain one
    /// anchor and an undisplaced direction so normalized translations are
    /// never introduced merely to cancel them again.
    pub(in crate::bezier_offset) fn recursive_projective_support_line(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRecursiveQuadraticProjectivePoint2>> {
        if let (
            CurvePoint2(CurvePointData2::AnalyticParallel(start)),
            CurvePoint2(CurvePointData2::AnalyticParallel(end)),
        ) = (self.start(), self.end())
        {
            match start.recursive_tangent_line_to(end, policy)? {
                Classification::Decided(Some(line)) => return Ok(Some(line)),
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        let frame = match self.recursive_projective_endpoints_with_direction(policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let [anchor, _] = frame.displaced;
        let [direction_start, direction_end] = frame.direction_endpoints;
        let Some((direction_x, direction_y, _)) =
            direction_end.difference_numerators(&direction_start)
        else {
            return Ok(None);
        };
        Ok((|| {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: direction_y
                    .multiply(&anchor.denominator)?
                    .scale(&Real::from(-1_i8))?,
                y: direction_x.multiply(&anchor.denominator)?,
                denominator: direction_y
                    .multiply(&anchor.x)?
                    .subtract(&direction_x.multiply(&anchor.y)?)?,
            })
        })())
    }

    /// Returns the exact traversal cross sign from the two compact projective
    /// support lines when bounded local arithmetic can decide it. For line
    /// coefficients `(a,b,c)=(-dy,dx,c)`, `a1*b2-b1*a2` is
    /// `cross(tangent1,tangent2)` up to the positive projective scales.
    pub(in crate::bezier_offset) fn recursive_support_tangent_cross_sign(
        &self,
        other: &Self,
        permit_recursive_norm: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let first_support = self.retained_support();
        let second_support = other.retained_support();
        let (Some(first), Some(second)) = (
            first_support.recursive_projective_support_line(policy)?,
            second_support.recursive_projective_support_line(policy)?,
        ) else {
            return Ok(None);
        };
        let orient = |mut sign| {
            if self.retained_support_orientation_is_reversed()
                ^ other.retained_support_orientation_is_reversed()
            {
                sign = product_sign(sign, RealSign::Negative);
            }
            sign
        };
        let exact = |line: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                line.x.exact_real_value_with_retained_witnesses()?,
                line.y.exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(first), Some(second)) = (exact(&first), exact(&second)) {
            let cross = Real::diff_of_products(&first[0], &second[1], &first[1], &second[0]);
            if let Some(sign) = cross.refine_sign_until(-512) {
                return Ok(Some(orient(sign)));
            }
        }
        let schedule: &[usize] = if policy.has_bounded_exact_predicate_budget() {
            &[0, 8]
        } else {
            &[0, 8, 128, 512]
        };
        let interval_cross = |first: &BezierRecursiveQuadraticProjectivePoint2,
                              second: &BezierRecursiveQuadraticProjectivePoint2,
                              refinement_steps| {
            let first_x = first.x.interval(refinement_steps)?;
            let first_y = first.y.interval(refinement_steps)?;
            let second_x = second.x.interval(refinement_steps)?;
            let second_y = second.y.interval(refinement_steps)?;
            first_x.multiply(&second_y).and_then(|positive| {
                first_y
                    .multiply(&second_x)
                    .map(|negative| positive.subtract(&negative))
            })
        };
        let mut terminal_refined = false;
        for &refinement_steps in schedule {
            if let Some(cross) = interval_cross(&first, &second, refinement_steps) {
                terminal_refined |= refinement_steps == 512;
                if let Some(sign) = dense_strict_interval_sign(&cross) {
                    return Ok(Some(orient(sign)));
                }
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(None);
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Some(RealSign::Zero));
        }
        let direct_join = first.joined_pair(&second, policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            let (first_base, first_extensions) =
                first.denominator.field().base_and_extension_path();
            let (second_base, second_extensions) =
                second.denominator.field().base_and_extension_path();
            let joined = match &direct_join {
                Classification::Decided(Some(_)) => "joined",
                Classification::Decided(None) => "none",
                Classification::Uncertain(_) => "uncertain",
            };
            eprintln!(
                "support tangent cross direct-join={joined} fields=({}+{},{}+{})",
                first_base.sources.len(),
                first_extensions.len(),
                second_base.sources.len(),
                second_extensions.len(),
            );
        }
        let (first, second) = match direct_join {
            Classification::Decided(Some((_, first, second))) => (first, second),
            Classification::Decided(None) | Classification::Uncertain(_) => {
                let field = first.denominator.field();
                let merged = recursive_merge_projective_point_fields(
                    &field,
                    std::slice::from_ref(&first),
                    &second,
                    policy,
                )?;
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                    eprintln!(
                        "support tangent cross source-union={}",
                        match &merged {
                            Classification::Decided(Some(_)) => "joined",
                            Classification::Decided(None) => "none",
                            Classification::Uncertain(_) => "uncertain",
                        }
                    );
                }
                match merged {
                    Classification::Decided(Some((_, mut first, second))) => {
                        let first = first
                            .pop()
                            .expect("one support line enters a two-line field merge");
                        (first, second)
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                }
            }
        };
        let Some(cross) = first.x.multiply(&second.y).and_then(|positive| {
            first
                .y
                .multiply(&second.x)
                .and_then(|negative| positive.subtract(&negative))
        }) else {
            return Ok(None);
        };
        if cross.is_structurally_zero() {
            return Ok(Some(RealSign::Zero));
        }
        let refinement_schedule: &[usize] = if policy.selects_approximate_512() {
            &[0, 8, 128, 512]
        } else {
            // A transverse mixed-field determinant can be far smaller than
            // 2^-512 while still separating cheaply in its retained tower.
            // Exhaust a few exact local enclosures before constructing the
            // much larger global tensor image reserved for equality.
            &[0, 8, 128, 512, 1024, 1664, 2560, 4096]
        };
        for &refinement_steps in refinement_schedule {
            if let Some(interval) = cross.interval(refinement_steps)
                && let Some(sign) = dense_strict_interval_sign(&interval)
            {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                    eprintln!(
                        "support tangent cross merged interval={refinement_steps} sign={sign:?}"
                    );
                }
                return Ok(Some(orient(sign)));
            }
        }
        if let Some(value) = cross.exact_real_value_with_retained_witnesses()
            && let Some(sign) = value.refine_sign_until(-512)
        {
            return Ok(Some(orient(sign)));
        }
        if !permit_recursive_norm {
            return Ok(None);
        }
        let sign = cross.sign(policy)?;
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!("support tangent cross compact-sign={sign:?}");
        }
        match sign {
            Classification::Decided(sign) => Ok(Some(orient(sign))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    /// Signs one retained point against the compact support line before the
    /// generic three-point kernel imports two independently materialized
    /// support endpoints. This is especially important for analytic tangent
    /// supports, whose shared speed radical otherwise appears twice.
    pub(in crate::bezier_offset) fn recursive_support_line_oriented_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Option<crate::classify::LineSide>> {
        if matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_))) {
            return Ok(None);
        }
        let support = self.retained_support();
        let Some(line) = support.recursive_projective_support_line(policy)? else {
            return Ok(None);
        };
        let point = match recursive_projective_evidence_points(&[point], policy)? {
            Classification::Decided(Some(mut points)) => points
                .pop()
                .expect("one recursive point query retains one projective point"),
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let reverse = self.retained_support_orientation_is_reversed();
        let side = |mut sign| {
            if reverse {
                sign = product_sign(sign, RealSign::Negative);
            }
            crate::classify::LineSide::from_real_sign(sign)
        };
        let exact = |value: &BezierRecursiveQuadraticProjectivePoint2| {
            Some([
                value.x.exact_real_value_with_retained_witnesses()?,
                value.y.exact_real_value_with_retained_witnesses()?,
                value
                    .denominator
                    .exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(line), Some(point)) = (exact(&line), exact(&point)) {
            let incidence = Real::signed_product_sum(
                [true, true, true],
                [
                    [&line[0], &point[0]],
                    [&line[1], &point[1]],
                    [&line[2], &point[2]],
                ],
            );
            let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                -8
            } else {
                -512
            };
            if let Some(sign) = incidence.refine_sign_until(minimum_precision) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "compact-real-support-line-incidence",
                );
                return Ok(Some(side(sign)));
            }
        }
        let schedule: &[usize] = if policy.has_bounded_exact_predicate_budget() {
            &[0, 8]
        } else if policy.permits_approximate_512() {
            &[0, 8, 128, 512]
        } else {
            &[0, 8, 128, 256, 512]
        };
        for &refinement_steps in schedule {
            let interval = |value: &BezierRecursiveQuadraticProjectivePoint2| {
                Some([
                    value.x.interval(refinement_steps)?,
                    value.y.interval(refinement_steps)?,
                    value.denominator.interval(refinement_steps)?,
                ])
            };
            let (Some(line), Some(point)) = (interval(&line), interval(&point)) else {
                continue;
            };
            let incidence = line[0]
                .multiply(&point[0])
                .and_then(|value| line[1].multiply(&point[1]).map(|term| value.add(&term)))
                .and_then(|value| line[2].multiply(&point[2]).map(|term| value.add(&term)));
            if let Some(sign) = incidence.as_ref().and_then(dense_strict_interval_sign) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "recursive-support-line-incidence-interval",
                );
                return Ok(Some(side(sign)));
            }
        }
        if !policy.has_bounded_exact_predicate_budget()
            && let Classification::Decided(Some((_, line, point))) =
                line.joined_pair(&point, policy)?
            && let Some(incidence) = line
                .x
                .multiply(&point.x)
                .and_then(|value| line.y.multiply(&point.y).and_then(|term| value.add(&term)))
                .and_then(|value| {
                    line.denominator
                        .multiply(&point.denominator)
                        .and_then(|term| value.add(&term))
                })
        {
            let compact_real = incidence.exact_real_value_with_retained_witnesses();
            let compact_real_zero = compact_real
                .as_ref()
                .is_some_and(|value| value.zero_status() == ZeroKnowledge::Zero);
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "support-line incidence structural-zero={} compact-real-zero={}",
                    incidence.is_structurally_zero(),
                    compact_real_zero,
                );
            }
            if incidence.is_structurally_zero() || compact_real_zero {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "recursive-support-line-structural-incidence",
                );
                return Ok(Some(crate::classify::LineSide::On));
            }
            for refinement_steps in [0, 8, 128, 512] {
                if let Some(interval) = incidence.interval(refinement_steps)
                    && let Some(sign) = dense_strict_interval_sign(&interval)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "joined-recursive-support-line-incidence-interval",
                    );
                    return Ok(Some(side(sign)));
                }
            }
            if let Some(value) = compact_real
                && let Some(sign) = value.refine_sign_until(-512)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-side-kernel",
                    "bounded-real-joined-support-line-incidence",
                );
                return Ok(Some(side(sign)));
            }
        }
        Ok(None)
    }

    /// Reuses recursive projective point fields for an exact oriented-area
    /// predicate. This is the native authority for nested selected-radial
    /// contacts, whose compact quadratic tower should not be flattened into
    /// independent high-degree coordinate representations.
    pub(in crate::bezier_offset) fn recursive_projective_oriented_side(
        &self,
        point: &CurvePoint2,
        certified_nonzero: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        if let Some(side) = self.recursive_support_line_oriented_side(point, policy)? {
            return Ok(Classification::Decided(Some(side)));
        }
        recursive_projective_point_evidence_oriented_side(
            self.start(),
            self.end(),
            point,
            certified_nonzero,
            policy,
        )
    }

    /// Cold exact side predicate for retained points whose interval boxes keep
    /// sharing a boundary after full refinement.  All coordinate witnesses
    /// enter one tensor authority, so the oriented area is signed as one
    /// correlated algebraic value instead of comparing independently rounded
    /// endpoint boxes.
    pub(in crate::bezier_offset) fn represented_oriented_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        match self.recursive_projective_oriented_side(point, false, policy)? {
            Classification::Decided(Some(side)) => {
                return Ok(Classification::Decided(side));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        represented_point_evidence_oriented_side(self.start(), self.end(), point, policy)
    }

    /// Classifies a retained point against this oriented support without
    /// adjoining its endpoint fields. The cross product is evaluated over
    /// progressively refined exact boxes. Only APPROXIMATE_512 may turn an
    /// unresolved terminal overlap into equality.
    pub(in crate::bezier_offset) fn oriented_side_by_refinement_with_limit(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
        maximum_refinement_steps: usize,
        recursive_prepass: bool,
        local_bounds_only: bool,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.validate_policy(policy)?;
        if [self.start(), self.end()]
            .into_iter()
            .any(|endpoint| endpoint.shares_storage(point))
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        let procedural_point = matches!(
            point,
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        );
        let procedural_support = [self.start(), self.end()].into_iter().all(|endpoint| {
            matches!(
                endpoint,
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            )
        });
        if recursive_prepass
            && !policy.has_bounded_exact_predicate_budget()
            && procedural_point
            && procedural_support
        {
            match self.recursive_projective_oriented_side(point, false, policy)? {
                Classification::Decided(Some(side)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "recursive-projective-prepass",
                    );
                    return Ok(Classification::Decided(side));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if refinement_steps > maximum_refinement_steps {
                break;
            }
            let bounds = |point| {
                if local_bounds_only {
                    algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
                } else {
                    algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
                }
            };
            let (
                Classification::Decided(start),
                Classification::Decided(end),
                Classification::Decided(point),
            ) = (bounds(self.start()), bounds(self.end()), bounds(point))
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let strict = &CurveContext::STRICT;
            let start_x = real_interval_from_axis(&start, Axis2::X);
            let start_y = real_interval_from_axis(&start, Axis2::Y);
            let delta_x = real_interval_from_axis(&end, Axis2::X).subtract(&start_x);
            let delta_y = real_interval_from_axis(&end, Axis2::Y).subtract(&start_y);
            let point_x = real_interval_from_axis(&point, Axis2::X).subtract(&start_x);
            let point_y = real_interval_from_axis(&point, Axis2::Y).subtract(&start_y);
            let Some(cross) = delta_x.multiply(&point_y).and_then(|first| {
                delta_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            if compare_reals(&cross.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Left));
            }
            if compare_reals(&cross.upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Right));
            }
            if compare_reals(&cross.lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::On));
            }
        }
        // Independently rebuilt descendants can still be an exact support
        // endpoint without sharing its allocation. This compact equality
        // certificate is cheaper and more specific than joining all three
        // point fields, and remains valid under an approximate object's
        // retained policy identity.
        if terminal_refined && policy.selects_approximate_512() {
            for endpoint in [self.start(), self.end()] {
                let same = policy.strict_predicate_pass(|| {
                    endpoint.same_point(point, policy) == Classification::Decided(true)
                        || point.same_point(endpoint, policy) == Classification::Decided(true)
                });
                if same {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
            }
        }
        if recursive_prepass
            && !policy.has_bounded_exact_predicate_budget()
            && maximum_refinement_steps >= 512
        {
            match self.represented_oriented_side(point, policy)? {
                Classification::Decided(side) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-side-kernel",
                        "represented-cold-fallback",
                    );
                    return Ok(Classification::Decided(side));
                }
                Classification::Uncertain(_) => {}
            }
            // Equality is a cold residual case. Running two independent
            // endpoint comparisons before the correlated cross predicate
            // makes every ordinary interior query pay for unrelated selected
            // fields. Retain the exact fallback here for endpoint evidence
            // that cannot enter the represented oriented-area kernel.
            for endpoint in [self.start(), self.end()] {
                if policy.strict_predicate_pass(|| endpoint.same_point(point, policy))
                    == Classification::Decided(true)
                {
                    return Ok(Classification::Decided(crate::classify::LineSide::On));
                }
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }

    pub(in crate::bezier_offset) fn oriented_side_by_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        if policy.has_bounded_exact_predicate_budget() {
            return self.oriented_side_by_refinement_with_limit(point, policy, 8, true, true);
        }
        let bounded = policy.bounded_exact_predicate_pass(|| {
            self.oriented_side_by_refinement_with_limit(point, policy, 8, true, true)
        })?;
        if matches!(bounded, Classification::Decided(_)) {
            return Ok(bounded);
        }
        self.oriented_side_by_refinement_with_limit(point, policy, 512, true, false)
    }

    pub(crate) fn strict_oriented_side_by_fast_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.oriented_side_by_refinement_with_limit(point, policy, 8, true, false)
    }

    /// Applies the same exact interval-side test using construction-local
    /// endpoint boxes only. A composite point that lacks such a box declines
    /// immediately instead of materializing its recursive field tower for a
    /// speculative Boolean fast path.
    pub(crate) fn strict_oriented_side_by_local_interval_refinement(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.oriented_side_by_refinement_with_limit(point, policy, 512, false, true)
    }

    /// Classifies all four endpoint/support relations for two general retained
    /// chords in one refinement pass.
    ///
    /// Calling the scalar fallback four times repeats both endpoint equality
    /// and the same support enclosures. Unary regularization is dominated by
    /// these all-pairs tests, so share each exact enclosure while preserving
    /// identical STRICT and APPROXIMATE_512 terminal behavior.
    pub(in crate::bezier_offset) fn pair_sides_by_refinement(
        &self,
        other: &Self,
        retain_structural_incidence: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicChordPairSides2>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if !policy.has_bounded_exact_predicate_budget() {
            let bounded = policy.bounded_exact_predicate_pass(|| {
                self.pair_sides_by_refinement(other, retain_structural_incidence, policy)
            })?;
            if matches!(bounded, Classification::Decided(_)) {
                return Ok(bounded);
            }
        }
        let first_points = [self.start(), self.end()];
        let second_points = [other.start(), other.end()];
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some()
            && first_points
                .iter()
                .chain(second_points.iter())
                .any(|point| matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_))))
        {
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
            let describe = |chord: &BezierAlgebraicChord2| {
                let support = chord.retained_support();
                format!(
                    "({},{})/support=({},{})/same={}/rev={}/axis={:?}/unit={}",
                    kind(chord.start()),
                    kind(chord.end()),
                    kind(support.start()),
                    kind(support.end()),
                    Arc::ptr_eq(&chord.data, &support.data),
                    chord.retained_support_orientation_is_reversed(),
                    chord.certified_axis_direction(),
                    chord.certified_unit_tangent().is_some(),
                )
            };
            let relation = |first: &BezierAlgebraicChord2, second: &BezierAlgebraicChord2| {
                let parallel = chord_parallel_support_source(first, policy)
                    .ok()
                    .flatten()
                    .zip(chord_parallel_support_source(second, policy).ok().flatten())
                    .map(|(first, second)| {
                        format!(
                            "parallel(dir={},source={},orientation={:?},distance={:?},tx={:?},ty={:?})",
                            first.direction == second.direction,
                            first.source.shares_retained_support(&second.source),
                            first.source.shared_tangent_orientation(&second.source),
                            compare_reals(&first.distance, &second.distance, &CurveContext::STRICT),
                            compare_reals(
                                &first.translation_x,
                                &second.translation_x,
                                &CurveContext::STRICT,
                            ),
                            compare_reals(
                                &first.translation_y,
                                &second.translation_y,
                                &CurveContext::STRICT,
                            ),
                        )
                    })
                    .unwrap_or_else(|| "parallel(n/a)".into());
                format!(
                    "{parallel}/tangent={:?}/normal={:?}",
                    first.shared_tangent_orientation(second),
                    first.retained_normal_offset_distance_to(second),
                )
            };
            eprintln!(
                "pair sides retain={retain_structural_incidence} first={} second={}",
                describe(self),
                describe(other),
            );
            for (owner, points, query) in [
                ("first", first_points, other),
                ("second", second_points, self),
            ] {
                for (index, point) in points.into_iter().enumerate() {
                    let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                        continue;
                    };
                    eprintln!(
                        "pair endpoint owner={owner} index={index} a={} b={} query-a={} query-b={} rel-a={} rel-b={} location={:?}",
                        describe(&point.data.first),
                        describe(&point.data.second),
                        query.shares_retained_support(&point.data.first),
                        query.shares_retained_support(&point.data.second),
                        relation(query, &point.data.first),
                        relation(query, &point.data.second),
                        point.data.location,
                    );
                    for (endpoint_index, endpoint) in [
                        query.retained_support().start(),
                        query.retained_support().end(),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        let CurvePoint2(CurvePointData2::AlgebraicChordParallel(endpoint)) =
                            endpoint
                        else {
                            continue;
                        };
                        let source_point = endpoint.data.source_point.as_deref();
                        let anchors = match point.data.location {
                            BezierAlgebraicChordPairPointLocation2::AnchorOrders {
                                first_at_end,
                                second_at_end,
                                ..
                            } => [
                                if first_at_end {
                                    point.data.first.end()
                                } else {
                                    point.data.first.start()
                                },
                                if second_at_end {
                                    point.data.second.end()
                                } else {
                                    point.data.second.start()
                                },
                            ],
                            BezierAlgebraicChordPairPointLocation2::EndpointSides { .. } => {
                                [point.data.first.start(), point.data.second.start()]
                            }
                        };
                        let endpoint_evidence = CurvePoint2::from(endpoint.clone());
                        eprintln!(
                            "pair query endpoint={endpoint_index} parallel origin={} origin-pair={} dir={:?} distance={:?} tx-zero={:?} ty-zero={:?} source-a={} source-b={} source={} anchor-storage={:?} anchor-eq={:?} source-anchor-storage={:?} source-anchor-eq={:?}",
                            source_point.map(kind).unwrap_or("endpoint"),
                            matches!(
                                source_point,
                                Some(CurvePoint2(CurvePointData2::AlgebraicChordPair(
                                    origin,
                                ))) if point == origin
                            ),
                            endpoint.data.direction,
                            real_sign(&endpoint.data.distance, &CurveContext::STRICT),
                            endpoint.data.translation_x.zero_status(),
                            endpoint.data.translation_y.zero_status(),
                            endpoint
                                .data
                                .source
                                .shares_retained_support(&point.data.first),
                            endpoint
                                .data
                                .source
                                .shares_retained_support(&point.data.second),
                            describe(&endpoint.data.source),
                            anchors.map(|anchor| endpoint_evidence.shares_storage(anchor)),
                            anchors.map(|anchor| endpoint_evidence == *anchor),
                            anchors.map(|anchor| endpoint.source_endpoint().shares_storage(anchor)),
                            anchors.map(|anchor| endpoint.source_endpoint() == anchor),
                        );
                    }
                }
            }
        }
        let mut equal = [[false; 2]; 2];
        for (first_index, first) in first_points.iter().enumerate() {
            for (second_index, second) in second_points.iter().enumerate() {
                equal[first_index][second_index] = first.shares_storage(second);
            }
        }

        let mut first_sides = [None; 2];
        let mut second_sides = [None; 2];
        for index in 0..2 {
            if equal[index][0] || equal[index][1] {
                first_sides[index] = Some(crate::classify::LineSide::On);
            }
            if equal[0][index] || equal[1][index] {
                second_sides[index] = Some(crate::classify::LineSide::On);
            }
        }
        // A retained miter endpoint is the authored intersection of two
        // support lines. Preserve that incidence in the batched endpoint
        // kernel instead of asking interval boxes to rediscover an exact
        // zero. The remaining endpoint sides then decide whether the unique
        // support intersection lies inside both finite chords.
        if retain_structural_incidence {
            for (index, point) in first_points.iter().enumerate() {
                if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point
                    && (other.shares_retained_support(&point.data.first)
                        || other.shares_retained_support(&point.data.second))
                {
                    first_sides[index] = Some(crate::classify::LineSide::On);
                }
            }
            for (index, point) in second_points.iter().enumerate() {
                if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point
                    && (self.shares_retained_support(&point.data.first)
                        || self.shares_retained_support(&point.data.second))
                {
                    second_sides[index] = Some(crate::classify::LineSide::On);
                }
            }
        }

        // The scalar support-predicate path owns exact correlated authorities
        // for analytic-tangent and procedural normal-offset endpoints. Seed
        // those same facts before the batched interval pass; choosing the
        // batch because one support lacks an algebraic ray must not bypass
        // construction evidence carried by the individual endpoints.
        for index in 0..2 {
            if first_sides[index].is_none()
                && self.certified_axis_direction().is_none()
                && other.certified_axis_direction().is_none()
                && let Some(side) =
                    other.retained_procedural_point_side(first_points[index], policy)?
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && self.certified_axis_direction().is_none()
                && other.certified_axis_direction().is_none()
                && let Some(side) =
                    self.retained_procedural_point_side(second_points[index], policy)?
            {
                second_sides[index] = Some(side);
            }
        }

        let decided_sides =
            |first_sides: &[Option<crate::classify::LineSide>; 2],
             second_sides: &[Option<crate::classify::LineSide>; 2]| {
                let on = crate::classify::LineSide::On;
                let strictly_one_sided = |sides: &[Option<crate::classify::LineSide>; 2]| {
                    matches!(
                        sides,
                        [
                            Some(crate::classify::LineSide::Left),
                            Some(crate::classify::LineSide::Left)
                        ] | [
                            Some(crate::classify::LineSide::Right),
                            Some(crate::classify::LineSide::Right)
                        ]
                    )
                };
                if strictly_one_sided(first_sides) || strictly_one_sided(second_sides) {
                    return Some(BezierAlgebraicChordPairSides2::Disjoint);
                }
                if first_sides.iter().all(|side| *side == Some(on))
                    || second_sides.iter().all(|side| *side == Some(on))
                {
                    // Either nonzero chord contributes two distinct points
                    // to the other affine support, which proves that the two
                    // supports are identical. Do not ask a fourth selected
                    // endpoint predicate to rediscover the same line equality.
                    return Some(BezierAlgebraicChordPairSides2::Complete([on; 2], [on; 2]));
                }
                (first_sides.iter().all(Option::is_some)
                    && second_sides.iter().all(Option::is_some))
                .then(|| {
                    BezierAlgebraicChordPairSides2::Complete(
                        first_sides.map(Option::unwrap),
                        second_sides.map(Option::unwrap),
                    )
                })
            };

        // A nonincident retained pair endpoint can still classify itself
        // against the opposite support from its certified offset-anchor
        // orders. Consume that compact affine certificate before the shared
        // interval loop asks for the pair's Cartesian intersection box.
        let merge_sides = |target: &mut [Option<crate::classify::LineSide>; 2],
                           source: [crate::classify::LineSide; 2]|
         -> CurveResult<()> {
            for (target, source) in target.iter_mut().zip(source) {
                if target.is_some_and(|target| target != source) {
                    return Err(CurveError::Topology(
                        "retained chord-pair endpoint incidences conflicted".into(),
                    ));
                }
                *target = Some(source);
            }
            Ok(())
        };
        let owner_incidence_sides = |owner: &BezierAlgebraicChord2,
                                     point: &BezierAlgebraicChordPairPoint2,
                                     defining_sides: [[crate::classify::LineSide; 2]; 2]|
         -> Option<[crate::classify::LineSide; 2]> {
            for (support, sides) in [
                (&point.data.first, defining_sides[0]),
                (&point.data.second, defining_sides[1]),
            ] {
                if !owner.shares_retained_support(support) {
                    continue;
                }
                let reversed = support.shared_tangent_orientation(owner)?;
                return Some(if reversed {
                    sides.map(|side| match side {
                        crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                        crate::classify::LineSide::On => crate::classify::LineSide::On,
                        crate::classify::LineSide::Right => crate::classify::LineSide::Left,
                    })
                } else {
                    sides
                });
            }
            None
        };
        if !policy.has_bounded_exact_predicate_budget() {
            for (index, point) in first_points.iter().enumerate() {
                let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                    continue;
                };
                if first_sides[index].is_some() {
                    continue;
                }
                if other.certified_axis_direction().is_some() {
                    continue;
                }
                if let Some((side, defining_sides)) =
                    point.endpoint_incidence_oriented_side_to_chord(other, policy)?
                {
                    if let Some(sides) = owner_incidence_sides(self, point, defining_sides) {
                        merge_sides(&mut second_sides, sides)?;
                    }
                    if let Some(side) = side {
                        if first_sides[index].is_some_and(|existing| existing != side) {
                            return Err(CurveError::Topology(
                                "retained chord-pair point-side certificates conflicted".into(),
                            ));
                        }
                        first_sides[index] = Some(side);
                    }
                }
            }
            for (index, point) in second_points.iter().enumerate() {
                let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                    continue;
                };
                if second_sides[index].is_some() {
                    continue;
                }
                if self.certified_axis_direction().is_some() {
                    continue;
                }
                if let Some((side, defining_sides)) =
                    point.endpoint_incidence_oriented_side_to_chord(self, policy)?
                {
                    if let Some(sides) = owner_incidence_sides(other, point, defining_sides) {
                        merge_sides(&mut first_sides, sides)?;
                    }
                    if let Some(side) = side {
                        if second_sides[index].is_some_and(|existing| existing != side) {
                            return Err(CurveError::Topology(
                                "retained chord-pair point-side certificates conflicted".into(),
                            ));
                        }
                        second_sides[index] = Some(side);
                    }
                }
            }
        }

        let complete_shared_endpoint_sides = |first_sides: &mut [Option<crate::classify::LineSide>;
                                                       2],
                                              second_sides: &mut [Option<crate::classify::LineSide>;
                                                       2],
                                              equal: &[[bool; 2]; 2]|
         -> CurveResult<bool> {
            let Some((first_index, second_index)) =
                equal.iter().enumerate().find_map(|(first_index, row)| {
                    row.iter()
                        .position(|is_equal| *is_equal)
                        .map(|second_index| (first_index, second_index))
                })
            else {
                return Ok(false);
            };
            if first_sides[1 - first_index].is_some() && second_sides[1 - second_index].is_some() {
                return Ok(false);
            }
            let Classification::Decided(tangent_cross) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                return Ok(false);
            };
            let oriented_side = |reverse: bool| {
                crate::classify::LineSide::from_real_sign(if reverse {
                    product_sign(tangent_cross, RealSign::Negative)
                } else {
                    tangent_cross
                })
            };
            first_sides[1 - first_index] = Some(oriented_side(first_index == 0));
            second_sides[1 - second_index] = Some(oriented_side(second_index == 1));
            Ok(true)
        };
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            return Ok(Classification::Decided(sides));
        }

        // A retained chord-pair endpoint is the exact intersection of the
        // current support and the support carried by its incident chord.  If
        // the two current tangents are transverse, that endpoint is therefore
        // the unique support intersection.  Order it against the opposite
        // finite endpoints in the already-retained chord parameter instead of
        // refining the correlated Cartesian point together with three
        // unrelated endpoint fields.
        for (second_index, point) in second_points.iter().enumerate() {
            let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                continue;
            };
            if !self.shares_retained_support(&point.data.first)
                && !self.shares_retained_support(&point.data.second)
            {
                continue;
            }
            let Classification::Decided(cross @ (RealSign::Positive | RealSign::Negative)) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                continue;
            };
            second_sides[second_index] = Some(crate::classify::LineSide::On);
            second_sides[1 - second_index] = Some(crate::classify::LineSide::from_real_sign(
                if second_index == 0 {
                    cross
                } else {
                    product_sign(cross, RealSign::Negative)
                },
            ));
            for (first_index, endpoint) in first_points.iter().enumerate() {
                let order = match point.cmp_on_chord_to_evidence(self, endpoint, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(_) => continue,
                };
                first_sides[first_index] = Some(match order {
                    std::cmp::Ordering::Equal => crate::classify::LineSide::On,
                    std::cmp::Ordering::Less => crate::classify::LineSide::from_real_sign(
                        product_sign(cross, RealSign::Negative),
                    ),
                    std::cmp::Ordering::Greater => crate::classify::LineSide::from_real_sign(cross),
                });
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "retained-intersection-endpoint-order",
                );
                return Ok(Classification::Decided(sides));
            }
        }
        for (first_index, point) in first_points.iter().enumerate() {
            let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point else {
                continue;
            };
            if !other.shares_retained_support(&point.data.first)
                && !other.shares_retained_support(&point.data.second)
            {
                continue;
            }
            let Classification::Decided(cross @ (RealSign::Positive | RealSign::Negative)) =
                policy.strict_predicate_pass(|| self.tangent_cross_sign(other, policy))?
            else {
                continue;
            };
            first_sides[first_index] = Some(crate::classify::LineSide::On);
            first_sides[1 - first_index] = Some(crate::classify::LineSide::from_real_sign(
                if first_index == 0 {
                    product_sign(cross, RealSign::Negative)
                } else {
                    cross
                },
            ));
            for (second_index, endpoint) in second_points.iter().enumerate() {
                let order = match point.cmp_on_chord_to_evidence(other, endpoint, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(_) => continue,
                };
                second_sides[second_index] = Some(match order {
                    std::cmp::Ordering::Equal => crate::classify::LineSide::On,
                    std::cmp::Ordering::Less => crate::classify::LineSide::from_real_sign(cross),
                    std::cmp::Ordering::Greater => crate::classify::LineSide::from_real_sign(
                        product_sign(cross, RealSign::Negative),
                    ),
                });
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "retained-intersection-endpoint-order",
                );
                return Ok(Classification::Decided(sides));
            }
        }

        // When the two finite chords retain the same endpoint allocation,
        // their remaining endpoint sides are exactly the sign of the two
        // traversal tangents (with the appropriate endpoint orientation).
        // Reuse that two-vector predicate before adjoining the shared point
        // and both opposite endpoints into a generic three-point field.
        if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
            && let Some(sides) = decided_sides(&first_sides, &second_sides)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "shared-endpoint-tangent-cross",
            );
            return Ok(Classification::Decided(sides));
        }

        // A pair endpoint is natively the intersection of two retained
        // supports. Sign that three-line determinant before the batched
        // Cartesian loop asks four independently selected endpoint towers to
        // coexist. The bounded preliminary pass deliberately skips this cold
        // exact authority.
        if !policy.has_bounded_exact_predicate_budget() {
            for index in 0..2 {
                if first_sides[index].is_none()
                    && let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) =
                        first_points[index]
                {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("pair precedence first={index} begin");
                    }
                    if let Classification::Decided(side) =
                        point.oriented_side_to_chord(other, policy)?
                    {
                        first_sides[index] = Some(side);
                    }
                }
                if second_sides[index].is_none()
                    && let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) =
                        second_points[index]
                {
                    #[cfg(test)]
                    if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                        eprintln!("pair precedence second={index} begin");
                    }
                    if let Classification::Decided(side) =
                        point.oriented_side_to_chord(self, policy)?
                    {
                        second_sides[index] = Some(side);
                    }
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "retained-pair-precedence",
            );
            return Ok(Classification::Decided(sides));
        }

        // Publish each remaining ordinary endpoint directly into the other
        // support's compact projective line before requesting four Cartesian
        // endpoint boxes. A retained pair point already consumed its native
        // three-line determinant above; every other point needs only one
        // line-point dot product, preserving its correlated quadratic tower.
        for index in 0..2 {
            if first_sides[index].is_none()
                && !matches!(
                    first_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(side) =
                    other.recursive_support_line_oriented_side(first_points[index], policy)?
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && !matches!(
                    second_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(side) =
                    self.recursive_support_line_oriented_side(second_points[index], policy)?
            {
                second_sides[index] = Some(side);
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "compact-support-line-endpoint-seeding",
            );
            return Ok(Classification::Decided(sides));
        }

        // Incidence with an affine support changes linearly along the other
        // chord. If a known endpoint side agrees with that exact derivative,
        // the unknown endpoint cannot cross zero; an `On` endpoint takes the
        // derivative side immediately. This completes many retained offset
        // pairs without comparing two nearly cancelling endpoint values.
        let mut recursive_support_cross =
            self.recursive_support_tangent_cross_sign(other, false, policy)?;
        if recursive_support_cross.is_none()
            && !policy.has_bounded_exact_predicate_budget()
            && let Some(sign) = self.retained_tangent_cross_sign(other, policy)
        {
            recursive_support_cross = match sign? {
                Classification::Decided(sign) => Some(sign),
                Classification::Uncertain(_) => None,
            };
        }
        if recursive_support_cross.is_none() && !policy.has_bounded_exact_predicate_budget() {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
                eprintln!("pair tangent cross entering recursive norm");
            }
            recursive_support_cross =
                self.recursive_support_tangent_cross_sign(other, true, policy)?;
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
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
            let first_support = self.retained_support();
            let second_support = other.retained_support();
            let (first_authority, first_authority_reversed) = self.tangent_authority();
            let (second_authority, second_authority_reversed) = other.tangent_authority();
            eprintln!(
                "pair affine propagation first=({},{})/support=({},{})/rev={}/authority=({},{})/rev={} second=({},{})/support=({},{})/rev={}/authority=({},{})/rev={} sides={first_sides:?}/{second_sides:?} cross={recursive_support_cross:?}",
                kind(self.start()),
                kind(self.end()),
                kind(first_support.start()),
                kind(first_support.end()),
                self.retained_support_orientation_is_reversed(),
                kind(first_authority.start()),
                kind(first_authority.end()),
                first_authority_reversed,
                kind(other.start()),
                kind(other.end()),
                kind(second_support.start()),
                kind(second_support.end()),
                other.retained_support_orientation_is_reversed(),
                kind(second_authority.start()),
                kind(second_authority.end()),
                second_authority_reversed,
            );
        }
        if let Some(cross @ (RealSign::Positive | RealSign::Negative)) = recursive_support_cross {
            let cross_side = crate::classify::LineSide::from_real_sign(cross);
            let opposite = |side| match side {
                crate::classify::LineSide::Left => crate::classify::LineSide::Right,
                crate::classify::LineSide::On => crate::classify::LineSide::On,
                crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            };
            let propagate = |sides: &mut [Option<crate::classify::LineSide>; 2], delta_side| {
                if sides[1].is_none()
                    && let Some(start) = sides[0]
                    && (start == crate::classify::LineSide::On || start == delta_side)
                {
                    sides[1] = Some(if start == crate::classify::LineSide::On {
                        delta_side
                    } else {
                        start
                    });
                }
                let backward_side = opposite(delta_side);
                if sides[0].is_none()
                    && let Some(end) = sides[1]
                    && (end == crate::classify::LineSide::On || end == backward_side)
                {
                    sides[0] = Some(if end == crate::classify::LineSide::On {
                        backward_side
                    } else {
                        end
                    });
                }
            };
            // `side(other, self(t))` has derivative `-cross(self, other)`;
            // `side(self, other(t))` has derivative `cross(self, other)`.
            propagate(&mut first_sides, opposite(cross_side));
            propagate(&mut second_sides, cross_side);
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "affine-side-monotonicity",
                );
                return Ok(Classification::Decided(sides));
            }
        }

        if policy.has_bounded_exact_predicate_budget()
            && first_points
                .iter()
                .zip(first_sides)
                .chain(second_points.iter().zip(second_sides))
                .any(|(point, side)| {
                    side.is_none()
                        && matches!(point, CurvePoint2(CurvePointData2::AlgebraicChordPair(_)))
                })
        {
            // A pair point's bounded native determinant and all ordinary
            // line-point incidences have already run above. Cartesian boxes
            // for that support intersection require the very field promotion
            // this speculative pass is meant to defer, so yield directly to
            // the complete STRICT or APPROXIMATE_512 terminal pass.
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }

        let strict = &CurveContext::STRICT;
        let zero = Real::zero();
        let check_endpoint_equalities = |first_sides: &mut [Option<crate::classify::LineSide>;
                                                  2],
                                         second_sides: &mut [Option<crate::classify::LineSide>;
                                                  2],
                                         equal: &mut [[bool; 2]; 2]|
         -> CurveResult<()> {
            for (first_index, first) in first_points.iter().enumerate() {
                for (second_index, second) in second_points.iter().enumerate() {
                    if first_sides[first_index].is_some() && second_sides[second_index].is_some() {
                        continue;
                    }
                    // Either strict side already proves that this particular
                    // endpoint pair is distinct. Do not let an APPROXIMATE_512
                    // equality terminal contradict stronger exact evidence.
                    if first_sides[first_index]
                        .is_some_and(|side| side != crate::classify::LineSide::On)
                        || second_sides[second_index]
                            .is_some_and(|side| side != crate::classify::LineSide::On)
                    {
                        continue;
                    }
                    let is_equal = match first.same_point(second, policy) {
                        Classification::Decided(is_equal) => is_equal,
                        Classification::Uncertain(_) => {
                            second.same_point(first, policy) == Classification::Decided(true)
                        }
                    };
                    if !is_equal {
                        continue;
                    }
                    // Only certified equality may become the shared-endpoint
                    // premise for the tangent theorem below. An
                    // APPROXIMATE_512 terminal may finish this side predicate
                    // as `On`, but must not become reusable construction
                    // evidence for the opposite endpoints.
                    if !policy.permits_approximate_512() {
                        equal[first_index][second_index] = true;
                    }
                    for side in [
                        &mut first_sides[first_index],
                        &mut second_sides[second_index],
                    ] {
                        if side.is_some_and(|side| side != crate::classify::LineSide::On) {
                            return Err(CurveError::Topology(
                                "exact chord side and endpoint-equality certificates conflict"
                                    .into(),
                            ));
                        }
                        *side = Some(crate::classify::LineSide::On);
                    }
                }
            }
            Ok(())
        };
        let interval_side = |start: &Aabb2, end: &Aabb2, point: &Aabb2| {
            let start_x = real_interval_from_axis(start, Axis2::X);
            let start_y = real_interval_from_axis(start, Axis2::Y);
            let delta_x = real_interval_from_axis(end, Axis2::X).subtract(&start_x);
            let delta_y = real_interval_from_axis(end, Axis2::Y).subtract(&start_y);
            let point_x = real_interval_from_axis(point, Axis2::X).subtract(&start_x);
            let point_y = real_interval_from_axis(point, Axis2::Y).subtract(&start_y);
            let cross = delta_x.multiply(&point_y).and_then(|first| {
                delta_y
                    .multiply(&point_x)
                    .map(|second| first.subtract(&second))
            })?;
            if compare_reals(&cross.lower, &zero, strict) == Some(std::cmp::Ordering::Greater) {
                return Some(crate::classify::LineSide::Left);
            }
            if compare_reals(&cross.upper, &zero, strict) == Some(std::cmp::Ordering::Less) {
                return Some(crate::classify::LineSide::Right);
            }
            (compare_reals(&cross.lower, &zero, strict) == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &zero, strict) == Some(std::cmp::Ordering::Equal))
            .then_some(crate::classify::LineSide::On)
        };

        let mut terminal_refined = false;
        let endpoint_bounds = |point, refinement_steps| {
            if policy.has_bounded_exact_predicate_budget() {
                // A correlated support intersection may need four divergent
                // endpoint fields to publish Cartesian coordinates.  The
                // batched APPROXIMATE_512 pass needs only construction-local
                // boxes here; unresolved pair endpoints are signed below by
                // the native flat three-support determinant.  STRICT retains
                // the complete nonlocal recursive fallback.
                algebraic_chord_endpoint_local_bounds_refined(point, refinement_steps, policy)
            } else {
                algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
            }
        };
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let (
                Classification::Decided(first_start),
                Classification::Decided(first_end),
                Classification::Decided(second_start),
                Classification::Decided(second_end),
            ) = (
                endpoint_bounds(self.start(), refinement_steps),
                endpoint_bounds(self.end(), refinement_steps),
                endpoint_bounds(other.start(), refinement_steps),
                endpoint_bounds(other.end(), refinement_steps),
            )
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            if let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
                first_start.union(&first_end),
                second_start.union(&second_end),
            ) && first_bounds.overlaps(&second_bounds, &CurveContext::STRICT)
                == Classification::Decided(false)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "refined-chord-box-disjointness",
                );
                return Ok(Classification::Decided(
                    BezierAlgebraicChordPairSides2::Disjoint,
                ));
            }
            let first_bounds = [&first_start, &first_end];
            let second_bounds = [&second_start, &second_end];
            for index in 0..2 {
                if first_sides[index].is_none() {
                    first_sides[index] =
                        interval_side(&second_start, &second_end, first_bounds[index]);
                }
                if second_sides[index].is_none() {
                    second_sides[index] =
                        interval_side(&first_start, &first_end, second_bounds[index]);
                }
            }
            if let Some(sides) = decided_sides(&first_sides, &second_sides) {
                return Ok(Classification::Decided(sides));
            }
        }
        if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
            && let Some(sides) = decided_sides(&first_sides, &second_sides)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "shared-endpoint-tangent-cross",
            );
            return Ok(Classification::Decided(sides));
        }
        // Pair intersections have already consumed their flat three-line
        // authority above. Remaining endpoints may now use the exact
        // one-coordinate predicate of a retained cardinal support without
        // constructing a general oriented-area compositum.
        for index in 0..2 {
            if first_sides[index].is_none()
                && !matches!(
                    first_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(direction) = other.certified_axis_direction()
                && let Some(Classification::Decided(side)) =
                    other.axis_oriented_side(first_points[index], direction, policy)
            {
                first_sides[index] = Some(side);
            }
            if second_sides[index].is_none()
                && !matches!(
                    second_points[index],
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                )
                && let Some(direction) = self.certified_axis_direction()
                && let Some(Classification::Decided(side)) =
                    self.axis_oriented_side(second_points[index], direction, policy)
            {
                second_sides[index] = Some(side);
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "cardinal-coordinate-terminal",
            );
            return Ok(Classification::Decided(sides));
        }
        // The batched box pass requires all four endpoints to expose boxes in
        // the same iteration. A single retained pair point can therefore hide
        // an otherwise complete 512-bit scalar refinement for a procedural
        // endpoint. Finish each remaining oriented area independently before
        // considering a multi-field represented compositum.
        if !policy.has_bounded_exact_predicate_budget() {
            for index in 0..2 {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some()
                    && (first_sides[index].is_none() || second_sides[index].is_none())
                {
                    eprintln!(
                        "pair scalar refinement index={index} before={first_sides:?}/{second_sides:?}"
                    );
                }
                if first_sides[index].is_none()
                    && let Classification::Decided(side) =
                        other.oriented_side_by_refinement(first_points[index], policy)?
                {
                    first_sides[index] = Some(side);
                }
                if second_sides[index].is_none()
                    && let Classification::Decided(side) =
                        self.oriented_side_by_refinement(second_points[index], policy)?
                {
                    second_sides[index] = Some(side);
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(test)]
            if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
                eprintln!(
                    "pair sides scalar terminal first={first_sides:?} second={second_sides:?}"
                );
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "scalar-refinement-terminal",
            );
            return Ok(Classification::Decided(sides));
        }
        if !policy.has_bounded_exact_predicate_budget() && !policy.selects_approximate_512() {
            check_endpoint_equalities(&mut first_sides, &mut second_sides, &mut equal)?;
            if complete_shared_endpoint_sides(&mut first_sides, &mut second_sides, &equal)?
                && let Some(sides) = decided_sides(&first_sides, &second_sides)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair-side-kernel",
                    "exact-endpoint-equality-terminal",
                );
                return Ok(Classification::Decided(sides));
            }
        }
        // Interval boxes cannot prove every nonzero side when a retained
        // endpoint is an affine image of a selected-circle contact.  At the
        // terminal predicate stage, sign each remaining oriented area in one
        // correlated represented tensor. STRICT keeps its complete exact
        // authority; the full APPROXIMATE_512 replay reaches the same scalar
        // only after the operation-wide strict pass has declined, and may
        // therefore consume its 512-bit equality terminal even when no
        // Cartesian endpoint box could be constructed.
        if !policy.has_bounded_exact_predicate_budget()
            && (!policy.selects_approximate_512() || policy.permits_approximate_512())
        {
            for index in 0..2 {
                if first_sides[index].is_none()
                    && let Classification::Decided(side) =
                        other.represented_oriented_side(first_points[index], policy)?
                {
                    first_sides[index] = Some(side);
                }
                if second_sides[index].is_none()
                    && let Classification::Decided(side) =
                        self.represented_oriented_side(second_points[index], policy)?
                {
                    second_sides[index] = Some(side);
                }
            }
        }
        if let Some(sides) = decided_sides(&first_sides, &second_sides) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair-side-kernel",
                "represented-cold-fallback",
            );
            return Ok(Classification::Decided(sides));
        }
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_CHORD_PAIR_SIDES").is_some() {
            eprintln!(
                "pair sides terminal first={first_sides:?} second={second_sides:?} equal={equal:?} refined={terminal_refined} selected={} permits={}",
                policy.selects_approximate_512(),
                policy.permits_approximate_512(),
            );
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(
                BezierAlgebraicChordPairSides2::Complete(
                    first_sides.map(|side| side.unwrap_or(crate::classify::LineSide::On)),
                    second_sides.map(|side| side.unwrap_or(crate::classify::LineSide::On)),
                ),
            ))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }
}

/// Signs `cross(first - shared, second - shared)` from certified vertex
/// enclosures, refining them a bounded number of times. Returns `None` when
/// the enclosed cross product still contains zero, including at a genuinely
/// collinear corner, so callers keep their exact structural routes.
fn enclosure_orientation_sign(
    shared: &CurvePoint2,
    first: &CurvePoint2,
    second: &CurvePoint2,
    policy: &CurveContext,
) -> Option<RealSign> {
    let strict = policy.strict_counterpart();
    for refinement_steps in [0_usize, 16] {
        let bounds = |point: &CurvePoint2| {
            match crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                point,
                refinement_steps,
                &strict,
            ) {
                Classification::Decided(bounds) => Some([
                    RealInterval {
                        lower: bounds.min_x().clone(),
                        upper: bounds.max_x().clone(),
                    },
                    RealInterval {
                        lower: bounds.min_y().clone(),
                        upper: bounds.max_y().clone(),
                    },
                ]),
                Classification::Uncertain(_) => None,
            }
        };
        let (Some([sx, sy]), Some([fx, fy]), Some([tx, ty])) =
            (bounds(shared), bounds(first), bounds(second))
        else {
            continue;
        };
        let cross = fx
            .subtract(&sx)
            .multiply(&ty.subtract(&sy))
            .zip(fy.subtract(&sy).multiply(&tx.subtract(&sx)))
            .map(|(left, right)| left.subtract(&right));
        if let Some(sign) = cross.and_then(|cross| cross.strict_nonzero_sign()) {
            return Some(sign);
        }
    }
    None
}
