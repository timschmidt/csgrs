//! Cusp-semicircle fragment construction, splitting and queries.

use super::*;

impl BezierAlgebraicCuspSemicircleFragment2 {
    pub(crate) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "algebraic cusp fragment was replayed under a different predicate policy".into(),
            ));
        }
        Ok(())
    }

    pub(crate) fn full(semicircle: BezierAlgebraicCuspSemicircle2, policy: &CurveContext) -> Self {
        let policy = policy
            .retained_object_policy_with_dependencies(semicircle.data.frame.evidence_policy());
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle,
                start: BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                end: BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                start_point_image: OnceLock::new(),
                end_point_image: OnceLock::new(),
                certified_tangent_endpoints: 0,
                reversed: false,
                policy,
            }),
        }
    }

    pub(crate) fn try_new(
        semicircle: BezierAlgebraicCuspSemicircle2,
        start: BezierAlgebraicCuspSemicircleParameter2,
        end: BezierAlgebraicCuspSemicircleParameter2,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let parallel_complement = start.parallel_complementary_to(&end, policy)?;
        let parameter_order = if matches!(parallel_complement, Some(Classification::Decided(true)))
        {
            // For end=1-start, start<end is exactly start<1/2. Retain the
            // complement result for the unit-range proof below instead of
            // reconstructing its two-radical certificate twice.
            start.order_to_real(&(Real::one() / Real::from(2_i8))?, policy)?
        } else {
            start.cmp_by_refinement(&end, policy)?
        };
        match parameter_order {
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let start_order = match start.order_to_real(&Real::zero(), policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = if matches!(parallel_complement, Some(Classification::Decided(true))) {
            // If end=1-start, comparing end with 1 is exactly the reverse of
            // comparing start with 0. Reuse the strict pair certificate rather
            // than replaying a differently gauged analytic carrier to the
            // terminal refinement bound.
            start_order.reverse()
        } else {
            match end.order_to_real(&Real::one(), policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        if end_order == std::cmp::Ordering::Greater {
            return Err(CurveError::InvalidBezierRange);
        }
        let policy = policy.retained_object_policy_with_dependencies(
            semicircle
                .data
                .frame
                .evidence_policy()
                .into_iter()
                .chain(start.evidence_policy())
                .chain(end.evidence_policy()),
        );
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle,
                start,
                end,
                start_point_image: OnceLock::new(),
                end_point_image: OnceLock::new(),
                certified_tangent_endpoints: 0,
                reversed,
                policy,
            }),
        }))
    }

    /// Builds a range already certified by the authoritative construction
    /// that produced its mapped endpoint.
    ///
    /// Retained fillets derive the endpoint from a common exact offset center
    /// and separately certify the directed tangent sweep to be within this
    /// half circle. Replaying the mapped parameter's radical order here would
    /// duplicate that proof and may lose it through expression rearrangement.
    pub(crate) fn from_certified_range(
        semicircle: BezierAlgebraicCuspSemicircle2,
        start: BezierAlgebraicCuspSemicircleParameter2,
        end: BezierAlgebraicCuspSemicircleParameter2,
        reversed: bool,
        policy: &CurveContext,
    ) -> Self {
        let policy = policy.retained_object_policy_with_dependencies(
            semicircle
                .data
                .frame
                .evidence_policy()
                .into_iter()
                .chain(start.evidence_policy())
                .chain(end.evidence_policy()),
        );
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle,
                start,
                end,
                start_point_image: OnceLock::new(),
                end_point_image: OnceLock::new(),
                certified_tangent_endpoints: 0,
                reversed,
                policy,
            }),
        }
    }

    /// Marks both traversal endpoints as tangential to their authored
    /// boundary neighbors.
    ///
    /// Fillet and round-join construction proves this topology before either
    /// endpoint is split into independent selected fields.  Retaining two
    /// bits lets later unary regularization consume that proof directly
    /// instead of reconstructing a circle/line resultant.
    pub(crate) fn with_certified_tangent_endpoints(mut self) -> Self {
        Arc::make_mut(&mut self.data).certified_tangent_endpoints = 0b11;
        self
    }

    pub(crate) fn without_certified_tangent_endpoints(mut self) -> Self {
        Arc::make_mut(&mut self.data).certified_tangent_endpoints = 0;
        self
    }

    pub(crate) fn certified_tangent_endpoint(&self, start_endpoint: bool) -> bool {
        let source_start = start_endpoint != self.data.reversed;
        self.data.certified_tangent_endpoints & if source_start { 1 } else { 2 } != 0
    }

    pub(crate) fn selected_chord_normal_contact_endpoint(&self, start_endpoint: bool) -> bool {
        matches!(
            self.endpoint_parameter(start_endpoint),
            BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter)
                if matches!(
                    parameter.as_ref(),
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact { .. }
                )
        )
    }

    /// Preserves only the authored outer-endpoint certificates retained by a
    /// selected-circle subrange.  Interior arrangement cuts deliberately gain
    /// no tangent claim.
    pub(crate) fn inherit_certified_tangent_endpoints(mut self, source: &Self) -> Self {
        let inherited = |parameter: &BezierAlgebraicCuspSemicircleParameter2| {
            u8::from(
                source.data.certified_tangent_endpoints & 1 != 0
                    && parameter.shares_exact_evidence(&source.data.start),
            ) | (u8::from(
                source.data.certified_tangent_endpoints & 2 != 0
                    && parameter.shares_exact_evidence(&source.data.end),
            ) << 1)
        };
        let bits = (inherited(&self.data.start) != 0) as u8
            | (((inherited(&self.data.end) != 0) as u8) << 1);
        Arc::make_mut(&mut self.data).certified_tangent_endpoints = bits;
        self
    }

    pub(crate) fn semicircle(&self) -> &BezierAlgebraicCuspSemicircle2 {
        &self.data.semicircle
    }

    pub(crate) fn start_parameter(&self) -> &BezierAlgebraicCuspSemicircleParameter2 {
        &self.data.start
    }

    pub(crate) fn end_parameter(&self) -> &BezierAlgebraicCuspSemicircleParameter2 {
        &self.data.end
    }

    pub(crate) fn is_reversed(&self) -> bool {
        self.data.reversed
    }

    /// Returns the signed-radius unit direction at one structurally cardinal
    /// endpoint of a directly framed retained circle.
    ///
    /// This deliberately inspects only authored `Real` identities. In
    /// particular, a mapped cut or an APPROXIMATE_512 terminal equality can
    /// never promote a general endpoint into the cardinal fast path.
    pub(in crate::bezier_offset) fn cardinal_endpoint_radial_components(
        &self,
        start_endpoint: bool,
    ) -> Option<(i8, i8)> {
        let (normal_x, normal_y) = self
            .data
            .semicircle
            .data
            .frame
            .rational()?
            .data
            .cardinal_normal?;
        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) =
            self.endpoint_parameter(start_endpoint)
        else {
            return None;
        };
        let parameter_slot = if parameter.zero_status() == hyperreal::ZeroKnowledge::Zero {
            0_u8
        } else if (Real::from(2_i8) * parameter - Real::one()).zero_status()
            == hyperreal::ZeroKnowledge::Zero
        {
            1
        } else if (parameter - Real::one()).zero_status() == hyperreal::ZeroKnowledge::Zero {
            2
        } else {
            return None;
        };
        let turn_sign = if self.data.semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        };
        Some(match parameter_slot {
            0 => (normal_x, normal_y),
            1 => (-normal_y * turn_sign, normal_x * turn_sign),
            2 => (-normal_x, -normal_y),
            _ => unreachable!("cardinal parameter slot is closed"),
        })
    }

    /// Returns a represented exact traversal tangent at an authored diameter
    /// or quarter-turn endpoint whenever the retained frame has a constant
    /// represented unit normal. Rational rotations of direct round joins use
    /// this path even though their tangent is no longer cardinal.
    pub(crate) fn represented_endpoint_tangent(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(Real, Real)>>> {
        self.validate_policy(policy)?;
        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) =
            self.endpoint_parameter(start_endpoint)
        else {
            return Ok(Classification::Decided(None));
        };
        let parameter_slot = if parameter.zero_status() == ZeroKnowledge::Zero {
            0_u8
        } else if (Real::from(2_i8) * parameter - Real::one()).zero_status() == ZeroKnowledge::Zero
        {
            1
        } else if (parameter - Real::one()).zero_status() == ZeroKnowledge::Zero {
            2
        } else {
            return Ok(Classification::Decided(None));
        };
        let Some((normal_x, normal_y)) =
            self.data.semicircle.data.frame.represented_unit_normal()?
        else {
            return Ok(Classification::Decided(None));
        };
        let turn = if self.data.semicircle.is_clockwise() {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let radial = match parameter_slot {
            0 => (normal_x, normal_y),
            1 => (-(&normal_y * &turn), &normal_x * &turn),
            2 => (-normal_x, -normal_y),
            _ => unreachable!("cardinal parameter slot is closed"),
        };
        let radial_sign = match real_sign(self.data.semicircle.radial_distance(), policy) {
            Some(RealSign::Positive) => Real::one(),
            Some(RealSign::Negative) => Real::from(-1_i8),
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected algebraic semicircle retained a zero radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let traversal = if self.data.reversed {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let scale = turn * radial_sign * traversal;
        Ok(Classification::Decided(Some((
            -(&radial.1 * &scale),
            &radial.0 * scale,
        ))))
    }

    /// Retains a second point on the endpoint traversal-tangent support.
    ///
    /// A mapped selected-circle endpoint may span several independent exact
    /// fields, so materializing its unit tangent would require an unnecessary
    /// compositum and square root.  Rotating the nonzero endpoint radius by a
    /// quarter turn gives the same supporting line with no normalization:
    /// `P + turn * traversal * perp(P - C)`.  The existing derived-point
    /// carrier keeps that affine rotation correlated with the mapped endpoint.
    pub(crate) fn endpoint_tangent_support_point(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        let traversal = if self.data.reversed {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let perpendicular_scale = self.data.semicircle.turn_sign() * traversal;
        match self.endpoint_parameter(start_endpoint) {
            BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) => {
                if parameter.semicircle_carrier().data.frame != self.data.semicircle.data.frame {
                    return Ok(Classification::Decided(None));
                }
                let point = match self.endpoint_point_evidence(start_endpoint, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(Classification::Decided(Some(CurvePoint2::from(
                    BezierAlgebraicCuspChordDerivedPoint2::rotated_from_mapped_source(
                        parameter.clone(),
                        point,
                        Real::one(),
                        perpendicular_scale,
                    ),
                ))))
            }
            BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) => {
                let (denominator, normal_scale, tangent_scale) = match self
                    .data
                    .semicircle
                    .represented_frame_scales(parameter, policy)?
                {
                    Classification::Decided(scales) => scales,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radial_normal_scale =
                    &normal_scale - &denominator * self.data.semicircle.center_parallel_distance();
                let support_normal_scale = &normal_scale - &perpendicular_scale * &tangent_scale;
                let support_tangent_scale =
                    &tangent_scale + &perpendicular_scale * radial_normal_scale;
                if let Some(frame) = self.data.semicircle.data.frame.rational() {
                    return Ok(Classification::Decided(Some(CurvePoint2::from(
                        frame.point_image_from_frame_scales(
                            &denominator,
                            &support_normal_scale,
                            &support_tangent_scale,
                            policy,
                        )?,
                    ))));
                }
                if let Some(frame) = self.data.semicircle.data.frame.selected_radial() {
                    if !policy.accepts_retained_policy(frame.policy) {
                        return Err(CurveError::Topology(
                            "a selected-radial tangent support crossed predicate policies".into(),
                        ));
                    }
                    let common_denominator = &denominator * &frame.normal_denominator;
                    match real_sign(&common_denominator, &CurveContext::STRICT) {
                        Some(RealSign::Positive | RealSign::Negative) => {}
                        Some(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a selected-radial tangent support had a zero frame denominator"
                                    .into(),
                            ));
                        }
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                        }
                    }
                    let radial_scale = Real::one() + (&support_normal_scale / &common_denominator)?;
                    let perpendicular_scale = (&support_tangent_scale / common_denominator)?;
                    return Ok(Classification::Decided(Some(CurvePoint2::from(
                        BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source_with_rotation(
                            frame.center_parameter.clone(),
                            frame.center_parameter.retained_point_evidence().cloned(),
                            radial_scale,
                            perpendicular_scale,
                        ),
                    ))));
                }
                if let Some(frame) = self.data.semicircle.data.frame.chord_normal() {
                    if !policy.accepts_retained_policy(frame.policy) {
                        return Err(CurveError::Topology(
                            "a chord-normal tangent support crossed predicate policies".into(),
                        ));
                    }
                    let normal_distance = (&support_normal_scale / &denominator)?;
                    let tangent_distance = -(&support_tangent_scale / denominator)?;
                    let point = frame.anchor.normal_displaced_point_evidence(
                        frame.center.clone(),
                        normal_distance,
                        policy,
                    )?;
                    return Ok(Classification::Decided(Some(
                        frame.anchor.tangent_displaced_point_evidence(
                            point,
                            tangent_distance,
                            policy,
                        )?,
                    )));
                }
                let frame = self
                    .data
                    .semicircle
                    .data
                    .frame
                    .parallel_normal()
                    .expect("every non-rational selected-circle frame is parallel-normal");
                if !policy.accepts_retained_policy(frame.policy) {
                    return Err(CurveError::Topology(
                        "a selected-circle tangent support crossed predicate policies".into(),
                    ));
                }
                let normal_distance = (&support_normal_scale / &denominator)?;
                let tangent_distance = -(&support_tangent_scale / denominator)?;
                if let Some(center_parameter) = frame.center_parameter.scalar() {
                    let parallel = frame.center_support.with_distance(normal_distance.clone());
                    let point = match parallel.point_at_with_policy(center_parameter, policy)? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let tangent = match parallel.source_tangent_at(center_parameter, policy)? {
                        Classification::Decided(tangent) => tangent,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let speed = Real::dot2_refs([&tangent.0, &tangent.1], [&tangent.0, &tangent.1])
                        .sqrt()?;
                    return Ok(Classification::Decided(Some(CurvePoint2::from(
                        point.translated(
                            (&tangent.0 * &tangent_distance / &speed)?,
                            (tangent.1 * tangent_distance / speed)?,
                        ),
                    ))));
                }
                Ok(Classification::Decided(Some(CurvePoint2::from(
                    BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                        frame.center_support.with_distance(normal_distance),
                        &frame.center_parameter,
                        tangent_distance,
                        policy,
                    )
                    .expect("a parallel-normal frame owns a scalar parameter"),
                ))))
            }
        }
    }

    /// Retains the endpoint traversal tangent as one certified finite chord.
    ///
    /// This is the common directional authority for selected-circle relation
    /// predicates.  Keeping the endpoint and its rotated support point in the
    /// native retained fields avoids materializing a normalized tangent or a
    /// compositum of independently selected coordinates.
    pub(crate) fn endpoint_tangent_chord(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChord2>>> {
        let point = match self.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support = match self.endpoint_tangent_support_point(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(point, support, policy)
            .map(|classification| classification.map(Some))
    }

    /// Proves that these finite selected-circle fragments share exactly one
    /// supporting-circle contact at a pair of fragment endpoints.
    ///
    /// The authored round companion identity is the constant-time lane.  A
    /// recursively offset descendant can instead retain the same endpoint and
    /// tangent through pair provenance after that companion wrapper is gone;
    /// exact point equality plus zero cross and nonzero dot is the equivalent
    /// certificate.  Unresolved cases continue through the authoritative
    /// circle-pair kernel under the active predicate policy.
    pub(crate) fn unique_shared_tangent_endpoint_contact(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<
            Option<(
                BezierAlgebraicCuspSemicircleParameter2,
                BezierAlgebraicCuspSemicircleParameter2,
            )>,
        >,
    > {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let radius_difference = self.data.semicircle.radial_distance()
            * self.data.semicircle.radial_distance()
            - other.data.semicircle.radial_distance() * other.data.semicircle.radial_distance();
        let supports_are_distinct = match real_sign(&radius_difference, policy) {
            Some(RealSign::Positive | RealSign::Negative) => true,
            Some(RealSign::Zero) => {
                let first_center = match self.data.semicircle.center_point_evidence(policy)? {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                };
                let second_center = match other.data.semicircle.center_point_evidence(policy)? {
                    Classification::Decided(center) => center,
                    Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                };
                match first_center.same_point(&second_center, policy) {
                    Classification::Decided(same) => !same,
                    Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                }
            }
            None => return Ok(Classification::Decided(None)),
        };
        if !supports_are_distinct {
            return Ok(Classification::Decided(None));
        }

        // Round construction stores the companion fragment and endpoint in
        // the mapped angular parameter. That authored tangent certificate is
        // stronger and cheaper than rebuilding two tangent witness chords.
        for owner_start in [true, false] {
            if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
                self.endpoint_parameter(owner_start)
                && let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                    companion,
                    companion_at_start,
                    ..
                } = data.as_ref()
                && companion == other
            {
                return Ok(Classification::Decided(Some((
                    self.endpoint_parameter(owner_start).clone(),
                    other.endpoint_parameter(*companion_at_start).clone(),
                ))));
            }
            if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
                other.endpoint_parameter(owner_start)
                && let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                    companion,
                    companion_at_start,
                    ..
                } = data.as_ref()
                && companion == self
            {
                return Ok(Classification::Decided(Some((
                    self.endpoint_parameter(*companion_at_start).clone(),
                    other.endpoint_parameter(owner_start).clone(),
                ))));
            }
        }

        // Concentric offsetting can preserve the pair-authored endpoint and
        // tangent while replacing the direct companion wrapper.  Ask the
        // retained pair map first; only its unique zero-cross endpoint needs
        // a point-equality predicate.
        for first_start in [true, false] {
            for second_start in [true, false] {
                let tangent = match self.endpoint_pair_tangent_cross_and_dot(
                    first_start,
                    other,
                    second_start,
                    policy,
                )? {
                    Classification::Decided(Some((RealSign::Zero, Some(dot))))
                        if dot != RealSign::Zero =>
                    {
                        true
                    }
                    Classification::Decided(Some(_)) | Classification::Decided(None) => false,
                    Classification::Uncertain(_) => false,
                };
                if !tangent {
                    continue;
                }
                let first_point = match self.endpoint_point_evidence(first_start, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
                let second_point = match other.endpoint_point_evidence(second_start, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
                if first_point.same_point(&second_point, policy) == Classification::Decided(true) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-pair-kernel",
                        "retained-pair-endpoint-tangent",
                    );
                    return Ok(Classification::Decided(Some((
                        self.endpoint_parameter(first_start).clone(),
                        other.endpoint_parameter(second_start).clone(),
                    ))));
                }
            }
        }

        Ok(Classification::Decided(None))
    }

    /// Returns the exact traversal-tangent relation against the retained
    /// chord that created a mapped endpoint. A concentric offset changes that
    /// tangent only by the sign of its signed-radius scale.
    ///
    /// The dot sign is retained when the endpoint was authored as the signed
    /// left-normal displacement of the chord. That construction proves a
    /// nonzero tangent direction without comparing independently selected
    /// endpoint fields.
    pub(crate) fn endpoint_chord_tangent_relation(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(BezierAlgebraicChord2, RealSign, Option<RealSign>)>>>
    {
        self.validate_policy(policy)?;
        self.data.semicircle.parameter_chord_tangent_relation(
            self.endpoint_parameter(start_endpoint),
            self.data.reversed,
            policy,
        )
    }

    /// Replays the exact tangent-cross sign at a circle-circle contact shared
    /// by two retained selected-circle fragments.
    ///
    /// The pair map owns `T_first x T_second`. Concentric offsets preserve
    /// each local parameter and multiply its tangent by the signed radial
    /// scale; fragment reversal contributes the remaining sign. This avoids
    /// materializing either multi-field tangent vector while keeping the
    /// original exact pair branch authoritative.
    pub(in crate::bezier_offset) fn tangent_orientation_factor_from(
        &self,
        source: &BezierAlgebraicCuspSemicircle2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        if self.data.semicircle.data.frame != source.data.frame
            || self.data.semicircle.is_clockwise() != source.is_clockwise()
        {
            return Ok(Classification::Decided(None));
        }
        let radial = match real_sign(
            &(self.data.semicircle.radial_distance() * source.radial_distance()),
            policy,
        ) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "selected-circle tangent retained a zero radial scale".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        Ok(Classification::Decided(Some(if self.data.reversed {
            product_sign(radial, RealSign::Negative)
        } else {
            radial
        })))
    }

    /// Reduces every endpoint whose tangent is authored by a retained circle
    /// pair to that pair's shared branch and one participating support.
    ///
    /// Direct pair cuts, concentric source circles, recursively selected
    /// fillet circles, and coincident-circle Boolean transports all enter this
    /// one representation. The returned factor is exact traversal orientation
    /// relative to the participating support; no endpoint coordinate or
    /// primitive-element field is reconstructed.
    pub(in crate::bezier_offset) fn endpoint_retained_pair_tangent_provenance(
        &self,
        start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRetainedPairTangentProvenance2<'_>>>> {
        self.validate_policy(policy)?;
        let endpoint = self.endpoint_parameter(start);
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = endpoint {
            if let Some((map, contact, first, transported_reversed)) = data.coincident_pair_source()
            {
                let orientation = match self
                    .tangent_orientation_factor_from(data.semicircle_carrier(), policy)?
                {
                    Classification::Decided(Some(orientation)) => orientation,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(Classification::Decided(Some(
                    BezierRetainedPairTangentProvenance2 {
                        map,
                        contact,
                        first,
                        orientation: if transported_reversed {
                            product_sign(orientation, RealSign::Negative)
                        } else {
                            orientation
                        },
                    },
                )));
            }
            if let Some((
                selected_circle,
                map,
                contact,
                anchor_first,
                radial_product_sign,
                selected_policy,
                transported_reversed,
            )) = data.coincident_selected_pair_contact()
            {
                if !policy.accepts_retained_policy(selected_policy)
                    || !policy.accepts_retained_policy(map.data.policy)
                {
                    return Err(CurveError::Topology(
                        "a retained pair tangent crossed predicate policies".into(),
                    ));
                }
                let Some(frame) = selected_circle.data.frame.selected_radial() else {
                    return Err(CurveError::Topology(
                        "a selected pair contact lost its pair-radial frame".into(),
                    ));
                };
                if !policy.accepts_retained_policy(frame.policy) {
                    return Err(CurveError::Topology(
                        "a selected pair tangent frame crossed predicate policies".into(),
                    ));
                }
                let Some((center_map, center_contact, center_first, center_reversed)) =
                    frame.center_parameter.coincident_pair_source()
                else {
                    return Err(CurveError::Topology(
                        "a selected pair tangent lost its center branch".into(),
                    ));
                };
                if !Arc::ptr_eq(&center_map.data, &map.data)
                    || center_contact != contact
                    || center_first != anchor_first
                    || center_reversed
                {
                    return Err(CurveError::Topology(
                        "a selected pair tangent disagreed with its center branch".into(),
                    ));
                }
                let companion = if anchor_first {
                    &map.data.second_semicircle
                } else {
                    &map.data.first_semicircle
                };
                let radial_frame_sign = match real_sign(
                    &(selected_circle.radial_distance() * &frame.normal_denominator),
                    policy,
                ) {
                    Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a selected pair tangent retained a zero radial frame".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                };
                // The stored radial-product sign is
                //   sign(r_f * d * (r_companion-r_support) * r_support),
                // with one extra sign when the pair charts have opposite
                // clockwise senses. Cancel the known frame factors to recover
                // the terminal radial relative to the companion support.
                let pair_chart_factor = if map.data.first_semicircle.is_clockwise()
                    != map.data.second_semicircle.is_clockwise()
                {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                let terminal_radial_orientation = product_sign(
                    radial_product_sign,
                    product_sign(radial_frame_sign, pair_chart_factor),
                );
                let turn = |circle: &BezierAlgebraicCuspSemicircle2| {
                    if circle.is_clockwise() {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    }
                };
                let base_orientation = product_sign(
                    terminal_radial_orientation,
                    product_sign(turn(selected_circle), turn(companion)),
                );
                let current_orientation = match self
                    .tangent_orientation_factor_from(data.semicircle_carrier(), policy)?
                {
                    Classification::Decided(Some(orientation)) => orientation,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let transport_orientation = if transported_reversed {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                return Ok(Classification::Decided(Some(
                    BezierRetainedPairTangentProvenance2 {
                        map,
                        contact,
                        first: !anchor_first,
                        orientation: product_sign(
                            base_orientation,
                            product_sign(current_orientation, transport_orientation),
                        ),
                    },
                )));
            }
        }

        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) = endpoint else {
            return Ok(Classification::Decided(None));
        };
        if parameter.zero_status() != ZeroKnowledge::Zero {
            return Ok(Classification::Decided(None));
        }
        let Some(frame) = self.data.semicircle.data.frame.selected_radial() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a pair-radial start tangent crossed predicate policies".into(),
            ));
        }
        let Some((map, contact, first, support_reversed)) =
            frame.center_parameter.coincident_pair_source()
        else {
            return Ok(Classification::Decided(None));
        };
        let support = frame.center_parameter.semicircle_carrier();
        let radial_orientation = match real_sign(
            &(self.data.semicircle.radial_distance() * &frame.normal_denominator),
            policy,
        ) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a pair-radial start tangent retained a zero radial frame".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let turn = |circle: &BezierAlgebraicCuspSemicircle2| {
            if circle.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            }
        };
        let mut orientation = product_sign(
            radial_orientation,
            product_sign(turn(&self.data.semicircle), turn(support)),
        );
        if self.data.reversed ^ support_reversed {
            orientation = product_sign(orientation, RealSign::Negative);
        }
        Ok(Classification::Decided(Some(
            BezierRetainedPairTangentProvenance2 {
                map,
                contact,
                first,
                orientation,
            },
        )))
    }

    /// Replays the diameter tangent of a selected-radial circle against the
    /// concentric source circle that authored its radial frame.
    ///
    /// The frame center parameter names a point `Q` on `support`, and the new
    /// parameter-zero radial is `(Q-O)/normal_denominator`.  Consequently its
    /// diameter tangent is a signed copy of the support tangent at that same
    /// retained parameter.  A concentric source fragment can reuse the exact
    /// parameter allocation after an offset, so this relation needs only
    /// pointer/evidence identity and scalar signs; it never reconstructs a
    /// selected endpoint or a primitive-element field.  The two circle
    /// centers are structurally distinct because `support` has nonzero radius
    /// while the selected-radial circle is centered at `Q`.
    pub(in crate::bezier_offset) fn endpoint_selected_radial_source_tangent_relation_one_way(
        &self,
        self_start: bool,
        source: &Self,
        source_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, RealSign)>>> {
        let BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) =
            self.endpoint_parameter(self_start)
        else {
            return Ok(Classification::Decided(None));
        };
        let diameter_orientation = if parameter.zero_status() == ZeroKnowledge::Zero {
            RealSign::Positive
        } else if (parameter - Real::one()).zero_status() == ZeroKnowledge::Zero {
            RealSign::Negative
        } else {
            return Ok(Classification::Decided(None));
        };
        let Some(frame) = self.data.semicircle.data.frame.selected_radial() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected-radial source tangent crossed predicate policies".into(),
            ));
        }
        let authored_parameter =
            BezierAlgebraicCuspSemicircleParameter2::Mapped(frame.center_parameter.clone());
        if !authored_parameter.shares_exact_evidence(source.endpoint_parameter(source_start)) {
            return Ok(Classification::Decided(None));
        }
        let support = frame.center_parameter.semicircle_carrier();
        if support.is_clockwise() != source.data.semicircle.is_clockwise()
            || !support
                .data
                .frame
                .shares_storage(&source.data.semicircle.data.frame)
        {
            return Ok(Classification::Decided(None));
        }
        let radial_orientation = match real_sign(
            &(self.data.semicircle.radial_distance() * &frame.normal_denominator),
            policy,
        ) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected-radial diameter tangent retained a zero radial frame".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let turn = |circle: &BezierAlgebraicCuspSemicircle2| {
            if circle.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            }
        };
        let mut selected_orientation = product_sign(
            diameter_orientation,
            product_sign(
                radial_orientation,
                product_sign(turn(&self.data.semicircle), turn(support)),
            ),
        );
        if self.data.reversed {
            selected_orientation = product_sign(selected_orientation, RealSign::Negative);
        }
        let source_orientation = match real_sign(
            &(source.data.semicircle.radial_distance() * support.radial_distance()),
            policy,
        ) {
            Some(sign @ (RealSign::Negative | RealSign::Positive)) => {
                if source.data.reversed {
                    product_sign(sign, RealSign::Negative)
                } else {
                    sign
                }
            }
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected-radial source circle retained a zero radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        Ok(Classification::Decided(Some((
            RealSign::Zero,
            product_sign(selected_orientation, source_orientation),
        ))))
    }

    /// Returns the exact tangent relation for either ordering of a retained
    /// selected-radial/source-circle diameter contact.  `Some` also certifies
    /// that the two supporting circles are distinct.
    pub(crate) fn endpoint_selected_radial_source_tangent_relation(
        &self,
        self_start: bool,
        other: &Self,
        other_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, RealSign)>>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        match self.endpoint_selected_radial_source_tangent_relation_one_way(
            self_start,
            other,
            other_start,
            policy,
        )? {
            Classification::Decided(Some(relation)) => {
                return Ok(Classification::Decided(Some(relation)));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        other.endpoint_selected_radial_source_tangent_relation_one_way(
            other_start,
            self,
            self_start,
            policy,
        )
    }

    /// Replays the complete exact tangent relation owned by a retained pair
    /// branch. Same-side recursive fillet contacts are structurally parallel;
    /// opposite sides reuse the map's exact cross and dot values.
    pub(crate) fn endpoint_pair_tangent_cross_and_dot(
        &self,
        self_start: bool,
        other: &Self,
        other_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, Option<RealSign>)>>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        match self.endpoint_selected_radial_source_tangent_relation(
            self_start,
            other,
            other_start,
            policy,
        )? {
            Classification::Decided(Some((cross, dot))) => {
                return Ok(Classification::Decided(Some((cross, Some(dot)))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let first = match self.endpoint_retained_pair_tangent_provenance(self_start, policy)? {
            Classification::Decided(Some(provenance)) => provenance,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match other.endpoint_retained_pair_tangent_provenance(other_start, policy)? {
            Classification::Decided(Some(provenance)) => provenance,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !Arc::ptr_eq(&first.map.data, &second.map.data) || first.contact != second.contact {
            return Ok(Classification::Decided(None));
        }
        let orientation = product_sign(first.orientation, second.orientation);
        let cross = if first.first == second.first {
            RealSign::Zero
        } else if first.first {
            first.contact.tangent_cross_sign
        } else {
            match first.contact.tangent_cross_sign {
                RealSign::Negative => RealSign::Positive,
                RealSign::Zero => RealSign::Zero,
                RealSign::Positive => RealSign::Negative,
            }
        };
        let cross = product_sign(cross, orientation);
        if cross != RealSign::Zero {
            return Ok(Classification::Decided(Some((cross, None))));
        }
        let dot = if first.first == second.first {
            RealSign::Positive
        } else {
            match first.map.tangent_dot_sign(first.contact, policy)? {
                Classification::Decided(dot) => dot,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        Ok(Classification::Decided(Some((
            cross,
            Some(product_sign(dot, orientation)),
        ))))
    }

    /// Replays an exactly parameterized endpoint of a parallel-normal circle
    /// against another retained parallel tangent. The rational semicircle
    /// chart expresses its tangent as `a*U + b*left_normal(U)`; rotating the
    /// requested cross/dot form back onto the two source tangents avoids both
    /// unit-speed radicals and any circle-pair reconstruction.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn exact_endpoint_tangent_cross_dot_retained_parallel(
        &self,
        start_endpoint: bool,
        parallel: &BezierParallel2,
        parameter: &BezierParameter2,
        source_direction: RealSign,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-circle-parallel-tangent-fast-path",
            match (
                self.endpoint_parameter(start_endpoint),
                &self.data.semicircle.data.frame,
            ) {
                (
                    BezierAlgebraicCuspSemicircleParameter2::Exact(_),
                    BezierSelectedCircleFrame2::ParallelNormal(_),
                ) => "exact-parallel-normal",
                (BezierAlgebraicCuspSemicircleParameter2::Exact(_), _) => "exact-other-frame",
                (
                    BezierAlgebraicCuspSemicircleParameter2::Mapped(_),
                    BezierSelectedCircleFrame2::ParallelNormal(_),
                ) => "mapped-parallel-normal",
                (BezierAlgebraicCuspSemicircleParameter2::Mapped(_), _) => "mapped-other-frame",
            },
        );
        #[cfg(feature = "dispatch-trace")]
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(mapped) =
            self.endpoint_parameter(start_endpoint)
        {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-parallel-tangent-mapped-kind",
                match mapped.as_ref() {
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { .. } => {
                        "rational"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                        ..
                    } => "selected-fiber-rational",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                        ..
                    } => "selected-fiber-parallel",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { .. } => {
                        "parallel"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                        ..
                    } => "selected-parallel-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedCircularTangentContact {
                        ..
                    } => "selected-circular-tangent-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedPairContact {
                        ..
                    } => "selected-pair-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                        ..
                    } => "selected-chord-normal-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                        ..
                    } => "selected-chord-parallel-normal-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Pair { .. } => "pair",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. } => "chord",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlap { .. } => {
                        "pair-overlap"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap {
                        ..
                    } => "pair-overlap-map",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                        ..
                    } => "similarity-transport",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { .. } => {
                        "chamfer"
                    }
                },
            );
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-parallel-tangent-frame-kind",
                match &self.data.semicircle.data.frame {
                    BezierSelectedCircleFrame2::Rational(_) => "rational",
                    BezierSelectedCircleFrame2::ParallelNormal(_) => "parallel-normal",
                    BezierSelectedCircleFrame2::ChordNormal(_) => "chord-normal",
                    BezierSelectedCircleFrame2::SelectedRadial(_) => "selected-radial",
                },
            );
        }
        let BezierAlgebraicCuspSemicircleParameter2::Exact(endpoint) =
            self.endpoint_parameter(start_endpoint)
        else {
            return Ok(None);
        };
        let Some(frame) = self.data.semicircle.data.frame.parallel_normal() else {
            return Ok(None);
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a selected-circle endpoint tangent crossed predicate policies".into(),
            ));
        }
        match in_closed_unit_interval(endpoint, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => {
                return Ok(Some(Classification::Uncertain(UncertaintyReason::Ordering)));
            }
        }
        let radial_orientation = match real_sign(self.data.semicircle.radial_distance(), policy) {
            Some(RealSign::Positive) => 1_i8,
            Some(RealSign::Negative) => -1_i8,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a selected circle retained a zero signed radius".into(),
                ));
            }
            None => {
                return Ok(Some(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
        } * if self.data.reversed { -1_i8 } else { 1_i8 };
        let turn = if self.data.semicircle.is_clockwise() {
            -1_i8
        } else {
            1_i8
        };
        let radial = Real::one() - Real::from(2_i8) * endpoint;
        let tangential = Real::from(2_i8) * endpoint * (Real::one() - endpoint);
        let tangent_source_scale = Real::from(-turn * radial_orientation) * radial;
        let tangent_normal_scale = Real::from(-radial_orientation) * tangential;
        let source_cross_scale =
            cross_scale * &tangent_source_scale + dot_scale * &tangent_normal_scale;
        let source_dot_scale =
            dot_scale * tangent_source_scale - cross_scale * tangent_normal_scale;
        let tangent_parameter =
            match promote_curve_region_bezier_parameter(&frame.center_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
        Ok(Some(
            frame
                .center_support
                .source_tangent_pair_cross_dot_linear_combination_sign(
                    &tangent_parameter,
                    parallel,
                    parameter,
                    &source_cross_scale,
                    &source_dot_scale,
                    policy,
                )?
                .map(|sign| product_sign(sign, source_direction)),
        ))
    }

    /// General endpoint/parallel tangent authority for independently authored
    /// selected fields. Each tangent is retained as two certified-distinct
    /// point evidences, then the shared algebraic-chord predicate signs the
    /// requested cross/dot combination without adjoining either field.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn endpoint_tangent_cross_dot_retained_parallel_by_chords(
        &self,
        start_endpoint: bool,
        parallel: &BezierParallel2,
        parameter: &CurveParameter2,
        source_direction: RealSign,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        let point = match self.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support = match self.endpoint_tangent_support_point(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let circle_tangent = match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
            point, support, policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "retained parallel endpoint supplied a zero traversal direction".into(),
            ));
        }
        let anchor_tangent = if let Some(parameter) = parameter.as_bezier_parameter() {
            let anchor_point = CurvePoint2::from(BezierAnalyticParallelPoint2::new(
                parallel.clone(),
                parameter.clone(),
                policy,
            ));
            let anchor_support =
                CurvePoint2::from(BezierAnalyticParallelPoint2::new_with_tangent_distance(
                    parallel.clone(),
                    parameter.clone(),
                    Real::one(),
                    policy,
                ));
            match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                anchor_point,
                anchor_support,
                policy,
            )? {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else if parameter.is_retained_scalar() {
            match BezierAlgebraicChord2::from_certified_retained_parallel_unit_tangent(
                parallel.clone(),
                parameter,
                policy,
            )? {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            return Ok(Classification::Decided(None));
        };
        Ok(circle_tangent
            .tangent_cross_dot_linear_combination_sign(
                &anchor_tangent,
                cross_scale,
                dot_scale,
                policy,
            )?
            .map(|sign| Some(product_sign(sign, source_direction))))
    }

    /// Replays the analytic tangent stored by this endpoint's own mapped
    /// circle/parallel contact.
    ///
    /// A selected-fiber contact already owns the complete two-normal map and
    /// its compact target parameter.  Round-join publication must consume
    /// that authority directly instead of asking the selected scalar to
    /// project globally merely so a caller can hand the same parameter back.
    pub(in crate::bezier_offset) fn endpoint_tangent_cross_dot_authored_parallel_contact(
        &self,
        start_endpoint: bool,
        source_direction: RealSign,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        self.validate_policy(policy)?;
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "retained parallel endpoint supplied a zero traversal direction".into(),
            ));
        }
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
            self.endpoint_parameter(start_endpoint)
        else {
            return Ok(Classification::Decided(None));
        };
        let source_circle = data.semicircle_carrier();
        let circle_factor = match self.tangent_orientation_factor_from(source_circle, policy)? {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let retained_cross_only = |tangent_cross_sign| {
            (real_sign(dot_scale, &CurveContext::STRICT) == Some(RealSign::Zero)).then(|| {
                real_sign(cross_scale, policy).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    |scale| Classification::Decided(product_sign(scale, tangent_cross_sign)),
                )
            })
        };
        let sign = match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { map, contact } => map
                .tangent_cross_dot_linear_combination_sign(
                    contact,
                    cross_scale,
                    dot_scale,
                    policy,
                )?,
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                tangent_cross_sign,
                ..
            } => match retained_cross_only(*tangent_cross_sign) {
                Some(sign) => sign,
                None => map.tangent_cross_dot_linear_combination_sign(
                    other_parameter,
                    cross_scale,
                    dot_scale,
                    policy,
                )?,
            },
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                tangent_cross_sign,
                ..
            } => match retained_cross_only(*tangent_cross_sign) {
                Some(sign) => sign,
                None => map.tangent_cross_dot_linear_combination_sign(
                    other_parameter,
                    cross_scale,
                    dot_scale,
                    policy,
                )?,
            },
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => map
                .data
                .semicircle
                .parallel_contact_tangent_cross_dot_source_sign(
                    &map.data.parallel,
                    contact,
                    cross_scale,
                    dot_scale,
                    policy,
                )?,
            _ => return Ok(Classification::Decided(None)),
        };
        Ok(sign.map(|sign| {
            Some(product_sign(
                sign,
                product_sign(circle_factor, source_direction),
            ))
        }))
    }

    /// Replays a selected circle/curve contact after the curve side has been
    /// promoted and offset as an analytic parallel.
    ///
    /// The mapped endpoint already owns the exact circle-tangent cross sign.
    /// Concentric circle offsets, candidate parallel composition, and fragment
    /// reversal can only multiply that tangent by certified nonzero signs, so
    /// no coordinate or primitive-element tangent is needed.
    pub(crate) fn endpoint_tangent_cross_retained_parallel(
        &self,
        start_endpoint: bool,
        parallel: &BezierParallel2,
        parameter: &BezierParameter2,
        source_direction: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        self.validate_policy(policy)?;
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "retained parallel endpoint supplied a zero traversal direction".into(),
            ));
        }
        if let Some(sign) = self.exact_endpoint_tangent_cross_dot_retained_parallel(
            start_endpoint,
            parallel,
            parameter,
            source_direction,
            &Real::one(),
            &Real::zero(),
            policy,
        )? {
            return Ok(sign.map(Some));
        }
        let fallback = || {
            self.endpoint_tangent_cross_dot_retained_parallel_by_chords(
                start_endpoint,
                parallel,
                &CurveParameter2::from(parameter.clone()),
                source_direction,
                &Real::one(),
                &Real::zero(),
                policy,
            )
        };
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
            self.endpoint_parameter(start_endpoint)
        else {
            return fallback();
        };
        let source_circle = data.semicircle_carrier();
        let circle_factor = match self.tangent_orientation_factor_from(source_circle, policy)? {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) => return fallback(),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match data.selected_parallel_contact_source_tangent_dot_sign(parallel, parameter, policy)? {
            Classification::Decided(Some(_)) => {
                return Ok(Classification::Decided(Some(RealSign::Zero)));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        if let BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } =
            data.as_ref()
        {
            if !policy.accepts_retained_policy(map.data.policy)
                || parallel.source() != map.data.parallel.source()
            {
                return fallback();
            }
            match contact.parallel_parameter.same_value(parameter, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => return fallback(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let Some(stored_cross) = contact.tangent_cross_sign else {
                return fallback();
            };
            if stored_cross == RealSign::Zero {
                return Ok(Classification::Decided(Some(RealSign::Zero)));
            }
            let mapped_scale = match map.data.parallel.parallel_derivative_scale_sign(
                &contact.parallel_parameter.clone().into(),
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
            return Ok(Classification::Decided(Some(product_sign(
                product_sign(stored_cross, circle_factor),
                product_sign(source_direction, mapped_scale),
            ))));
        }
        let (selected_parameter, stored_cross, candidate_factor) = match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                tangent_cross_sign,
                ..
            } => {
                if !policy.accepts_retained_policy(map.data.policy)
                    || !matches!(
                        parallel.source(),
                        BezierParallelSource2::Rational(source) if source == &map.data.curve
                    )
                {
                    return fallback();
                }
                (other_parameter, *tangent_cross_sign, source_direction)
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                tangent_cross_sign,
                ..
            } => {
                if !policy.accepts_retained_policy(map.data.policy)
                    || parallel.source() != map.data.parallel.source()
                {
                    return fallback();
                }
                let mapped_scale = match map
                    .data
                    .parallel
                    .parallel_derivative_scale_sign(&parameter.clone().into(), policy)?
                {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (
                    other_parameter,
                    *tangent_cross_sign,
                    product_sign(source_direction, mapped_scale),
                )
            }
            _ => return fallback(),
        };
        match selected_parameter.cmp_bezier_parameter(parameter, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => {}
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                return fallback();
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(Classification::Decided(Some(product_sign(
            product_sign(stored_cross, circle_factor),
            candidate_factor,
        ))))
    }

    /// Signs `a * (T_circle x T_parallel) + b *
    /// (T_circle dot T_parallel)` at a retained selected-fiber endpoint.
    ///
    /// This is strictly more informative than the cached contact cross sign:
    /// round-join parameter ordering needs a represented angular linear
    /// combination. The original two-normal contact map supplies both
    /// magnitudes under one positive scale, while concentric offset,
    /// traversal reversal, and promoted-parallel traversal contribute only a
    /// common nonzero orientation factor.
    pub(crate) fn endpoint_tangent_cross_dot_linear_combination_retained_parallel(
        &self,
        start_endpoint: bool,
        parallel: &BezierParallel2,
        parameter: &BezierParameter2,
        source_direction: RealSign,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        self.validate_policy(policy)?;
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "retained parallel endpoint supplied a zero traversal direction".into(),
            ));
        }
        if let Some(sign) = self.exact_endpoint_tangent_cross_dot_retained_parallel(
            start_endpoint,
            parallel,
            parameter,
            source_direction,
            cross_scale,
            dot_scale,
            policy,
        )? {
            return Ok(sign.map(Some));
        }
        let fallback = || {
            self.endpoint_tangent_cross_dot_retained_parallel_by_chords(
                start_endpoint,
                parallel,
                &CurveParameter2::from(parameter.clone()),
                source_direction,
                cross_scale,
                dot_scale,
                policy,
            )
        };
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
            self.endpoint_parameter(start_endpoint)
        else {
            return fallback();
        };
        let source_circle = data.semicircle_carrier();
        let circle_factor = match self.tangent_orientation_factor_from(source_circle, policy)? {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) => return fallback(),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match data.selected_parallel_contact_source_tangent_dot_sign(parallel, parameter, policy)? {
            Classification::Decided(Some(source_dot)) => {
                let coefficient = match real_sign(dot_scale, policy) {
                    Some(sign) => sign,
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                };
                return Ok(Classification::Decided(Some(product_sign(
                    coefficient,
                    product_sign(source_dot, product_sign(circle_factor, source_direction)),
                ))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let sign = match data.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberRational {
                map,
                other_parameter,
                ..
            } => {
                if !policy.accepts_retained_policy(map.data.policy)
                    || !matches!(
                        parallel.source(),
                        BezierParallelSource2::Rational(source) if source == &map.data.curve
                    )
                {
                    return fallback();
                }
                match other_parameter.cmp_bezier_parameter(parameter, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => {}
                    Classification::Decided(
                        std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                    ) => return fallback(),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                map.tangent_cross_dot_linear_combination_sign(
                    other_parameter,
                    cross_scale,
                    dot_scale,
                    policy,
                )?
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedFiberParallel {
                map,
                other_parameter,
                ..
            } => {
                if !policy.accepts_retained_policy(map.data.policy)
                    || parallel.source() != map.data.parallel.source()
                {
                    return fallback();
                }
                match other_parameter.cmp_bezier_parameter(parameter, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => {}
                    Classification::Decided(
                        std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                    ) => return fallback(),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                map.tangent_cross_dot_linear_combination_sign(
                    other_parameter,
                    cross_scale,
                    dot_scale,
                    policy,
                )?
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } => {
                if !policy.accepts_retained_policy(map.data.policy)
                    || parallel.source() != map.data.parallel.source()
                {
                    return fallback();
                }
                match contact.parallel_parameter.same_value(parameter, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => return fallback(),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                map.data
                    .semicircle
                    .parallel_contact_tangent_cross_dot_source_sign(
                        &map.data.parallel,
                        contact,
                        cross_scale,
                        dot_scale,
                        policy,
                    )?
            }
            _ => return fallback(),
        };
        let sign = match sign {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(product_sign(
            sign,
            product_sign(circle_factor, source_direction),
        ))))
    }

    /// Recovers the oriented tangent relation at a smooth source-carrier
    /// overlap after both carriers have been offset independently.
    ///
    /// The offset circle and composed parallel can place their common endpoint
    /// in unrelated selected fields.  Their source carriers still own a
    /// positive-length exact overlap, however.  The overlap orientation gives
    /// the source tangent dot sign; concentric-circle scaling, parallel
    /// derivative scaling, and traversal reversal contribute only certified
    /// nonzero factors.  This is therefore a proof of both zero cross product
    /// and nonzero dot product without comparing Cartesian endpoint images.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn endpoint_tangent_dot_retained_parallel_source_overlap(
        &self,
        source_fragment: &Self,
        start_endpoint: bool,
        source_parallel: &BezierParallel2,
        source_range: &CurveParameterRange2,
        parameter: &BezierParameter2,
        selected_source_parameter: Option<&BezierAlgebraicSelectedFiberParameter2>,
        source_direction: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        self.validate_policy(policy)?;
        source_fragment.validate_policy(policy)?;
        if source_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "retained parallel source overlap supplied a zero traversal direction".into(),
            ));
        }
        let circle_factor =
            match self.tangent_orientation_factor_from(&source_fragment.data.semicircle, policy)? {
                Classification::Decided(Some(factor)) => factor,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let source_parameter = source_fragment.endpoint_parameter(start_endpoint);
        let intersections = match source_fragment.data.semicircle.parallel_intersections(
            source_parallel,
            source_range,
            None,
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained_orientation = None;
        let mut retain_orientation = |orientation: CurveOverlapOrientation2| {
            let sign = match orientation {
                CurveOverlapOrientation2::Same => RealSign::Positive,
                CurveOverlapOrientation2::Reversed => RealSign::Negative,
            };
            match retained_orientation {
                Some(retained) if retained != sign => Err(CurveError::Topology(
                    "one smooth circle/parallel endpoint retained conflicting overlap orientations"
                        .into(),
                )),
                Some(_) => Ok(()),
                None => {
                    retained_orientation = Some(sign);
                    Ok(())
                }
            }
        };
        match intersections {
            BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber {
                overlaps, ..
            } => {
                for overlap in overlaps {
                    let candidate =
                        match overlap.other_parameter_for_cusp(source_parameter, policy)? {
                            Classification::Decided(candidate) => candidate,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    let order = if let Some(selected_source_parameter) = selected_source_parameter {
                        candidate.cmp_by_refinement(selected_source_parameter, policy)?
                    } else {
                        candidate.cmp_bezier_parameter(parameter, policy)?
                    };
                    match order {
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            retain_orientation(overlap.orientation())?;
                        }
                        Classification::Decided(
                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                        ) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { overlaps, .. } => {
                for overlap in overlaps {
                    let candidate =
                        match overlap.other_parameter_for_cusp(source_parameter, policy)? {
                            Classification::Decided(candidate) => candidate,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    match candidate.same_value(&CurveParameter2::from(parameter.clone()), policy)? {
                        Classification::Decided(true) => {
                            retain_orientation(overlap.orientation())?;
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(_)
            | BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
            | BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {}
        }
        Ok(Classification::Decided(retained_orientation.map(
            |orientation| product_sign(orientation, product_sign(circle_factor, source_direction)),
        )))
    }

    /// Replays a smooth adjacency between two structurally coincident source
    /// circles after independent concentric offsets.
    ///
    /// Complementary semicircles intentionally use opposite signed radii, so
    /// carrier equality is the shared frame plus equal squared radius.  Once
    /// the source endpoint identity is certified, their physical clockwise
    /// senses determine the nonzero source tangent dot sign.  Each offset
    /// contributes its exact radial/traversal orientation factor.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn endpoint_pair_tangent_cross_and_dot_source_circle(
        &self,
        source_fragment: &Self,
        start_endpoint: bool,
        other: &Self,
        other_source_fragment: &Self,
        other_start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RealSign, RealSign)>>> {
        self.validate_policy(policy)?;
        source_fragment.validate_policy(policy)?;
        other.validate_policy(policy)?;
        other_source_fragment.validate_policy(policy)?;
        if source_fragment.data.semicircle.data.frame
            != other_source_fragment.data.semicircle.data.frame
        {
            return Ok(Classification::Decided(None));
        }
        let radius_difference = source_fragment.data.semicircle.radial_distance()
            * source_fragment.data.semicircle.radial_distance()
            - other_source_fragment.data.semicircle.radial_distance()
                * other_source_fragment.data.semicircle.radial_distance();
        match real_sign(&radius_difference, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Negative | RealSign::Positive) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let first_point = match source_fragment.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second_point =
            match other_source_fragment.endpoint_point_evidence(other_start_endpoint, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        match first_point.same_point(&second_point, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let first_factor =
            match self.tangent_orientation_factor_from(&source_fragment.data.semicircle, policy)? {
                Classification::Decided(Some(factor)) => factor,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let second_factor = match other
            .tangent_orientation_factor_from(&other_source_fragment.data.semicircle, policy)?
        {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_dot = if source_fragment.data.semicircle.is_clockwise()
            == other_source_fragment.data.semicircle.is_clockwise()
        {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        Ok(Classification::Decided(Some((
            RealSign::Zero,
            product_sign(source_dot, product_sign(first_factor, second_factor)),
        ))))
    }

    /// Signs this selected-circle traversal tangent crossed with one
    /// represented vector at an arbitrary retained endpoint.
    ///
    /// For radial vector `R=P-C`, a clockwise circle tangent crossed with
    /// `V` has sign `R dot V`; counterclockwise traversal reverses that sign.
    /// Point and center may occupy independent selected fields, so the shared
    /// linear-order authority refines their existing bounds without creating
    /// a primitive element. Only APPROXIMATE_512 may terminate an unresolved
    /// equality at the 512-bit policy boundary.
    pub(crate) fn endpoint_tangent_cross_vector(
        &self,
        start_endpoint: bool,
        vector: &(Real, Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-circle-vector-tangent-endpoint",
            match self.endpoint_parameter(start_endpoint) {
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                    if parameter.zero_status() == ZeroKnowledge::Zero =>
                {
                    "exact-zero"
                }
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                    if (parameter - Real::one()).zero_status() == ZeroKnowledge::Zero =>
                {
                    "exact-one"
                }
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter)
                    if (Real::from(2_i8) * parameter - Real::one()).zero_status()
                        == ZeroKnowledge::Zero =>
                {
                    "exact-half"
                }
                BezierAlgebraicCuspSemicircleParameter2::Exact(_) => "exact-other",
                BezierAlgebraicCuspSemicircleParameter2::Mapped(data) => match data.as_ref() {
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { .. } => {
                        "mapped-rational"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { .. } => {
                        "mapped-parallel"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                        ..
                    } => "mapped-selected-parallel-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { .. } => {
                        "mapped-chord"
                    }
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                        ..
                    } => "mapped-selected-chord-normal-contact",
                    BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                        ..
                    } => "mapped-selected-chord-parallel-normal-contact",
                    _ => "mapped-other",
                },
            },
        );
        match self.represented_endpoint_tangent(start_endpoint, policy)? {
            Classification::Decided(Some(tangent)) => {
                let cross = Real::diff_of_products(&tangent.0, &vector.1, &tangent.1, &vector.0);
                return Ok(real_sign(&cross, policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Exact(endpoint) =
            self.endpoint_parameter(start_endpoint)
            && (endpoint.zero_status() == ZeroKnowledge::Zero
                || (endpoint - Real::one()).zero_status() == ZeroKnowledge::Zero)
        {
            if let Some(frame) = self.data.semicircle.data.frame.rational()
                && let Classification::Decided(mut radial_projection) =
                    frame.normal_dot_vector_sign(vector, policy)?
            {
                if radial_projection == RealSign::Zero {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                let radial_sign = match real_sign(self.data.semicircle.radial_distance(), policy) {
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a selected circle retained a zero signed radius".into(),
                        ));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                };
                radial_projection = product_sign(radial_projection, radial_sign);
                if (endpoint - Real::one()).zero_status() == ZeroKnowledge::Zero {
                    radial_projection = product_sign(radial_projection, RealSign::Negative);
                }
                if self.data.semicircle.is_clockwise() == self.data.reversed {
                    radial_projection = product_sign(radial_projection, RealSign::Negative);
                }
                return Ok(Classification::Decided(radial_projection));
            }
            let source = (
                self.data.semicircle.source_parallel(),
                self.data.semicircle.selected_frame_parameter(),
            );
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-vector-diameter-frame",
                if source.0.is_some() && source.1.is_some() {
                    "retained-parallel"
                } else {
                    "unrepresented"
                },
            );
            if let (Some(parallel), Some(parameter)) = source {
                let relation = parallel
                    .vector_tangent_cross_and_dot_signs(&parameter, &vector.0, &vector.1, policy)?;
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "selected-circle-vector-diameter-relation",
                    match &relation {
                        Classification::Decided((
                            RealSign::Zero,
                            RealSign::Positive | RealSign::Negative,
                        )) => "parallel",
                        Classification::Decided(_) => "transverse-or-degenerate",
                        Classification::Uncertain(_) => "uncertain",
                    },
                );
                if let Classification::Decided((RealSign::Zero, dot)) = relation
                    && dot != RealSign::Zero
                {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
            }
        }
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) =
            self.endpoint_parameter(start_endpoint)
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::Parallel { map, contact } =
                data.as_ref()
            && policy.accepts_retained_policy(map.data.policy)
            && contact.tangent_cross_sign == Some(RealSign::Zero)
        {
            match map.data.parallel.vector_tangent_cross_and_dot_signs(
                &contact.parallel_parameter.clone().into(),
                &vector.0,
                &vector.1,
                policy,
            )? {
                Classification::Decided((RealSign::Zero, dot)) if dot != RealSign::Zero => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-circle-vector-tangent-fast-path",
                        "mapped-parallel-line",
                    );
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let point = match self.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self.data.semicircle.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radial_projection = match algebraic_chord_points_linear_order(
            &point, &center, &vector.0, &vector.1, policy,
        )? {
            Classification::Decided(std::cmp::Ordering::Less) => RealSign::Negative,
            Classification::Decided(std::cmp::Ordering::Equal) => RealSign::Zero,
            Classification::Decided(std::cmp::Ordering::Greater) => RealSign::Positive,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let clockwise = self.data.semicircle.is_clockwise() != self.data.reversed;
        Ok(Classification::Decided(if clockwise {
            radial_projection
        } else {
            match radial_projection {
                RealSign::Negative => RealSign::Positive,
                RealSign::Zero => RealSign::Zero,
                RealSign::Positive => RealSign::Negative,
            }
        }))
    }

    /// Signs this selected-circle traversal tangent crossed with one retained
    /// algebraic-chord direction without adjoining their selected fields.
    ///
    /// For radial vector `R=P-C`, the cross product is, up to traversal
    /// orientation, `R dot (chord.end-chord.start)`.  The existing mapped
    /// circle/chord contact remains the constant-time authority whenever the
    /// candidate shares that tangent direction.  Otherwise progressively
    /// refined exact endpoint boxes decide the dot-product sign.  Only an
    /// APPROXIMATE_512 policy may terminate an unresolved equality at 512-bit
    /// refinement only when the caller permits it. Speculative fast proofs
    /// disable that terminal so they cannot consume certainty before a later
    /// exact certificate is tried.
    pub(crate) fn endpoint_tangent_cross_algebraic_chord(
        &self,
        start_endpoint: bool,
        chord: &BezierAlgebraicChord2,
        permit_terminal_approximation: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        chord.validate_policy(policy)?;

        let endpoint_relation = self.endpoint_chord_tangent_relation(start_endpoint, policy)?;
        if let Classification::Decided(Some((reference, reference_cross))) =
            endpoint_relation.map(|relation| relation.map(|(chord, cross, _)| (chord, cross)))
        {
            if let Some(reversed) = chord.retained_normal_offset_tangent_reversal_to(&reference) {
                return Ok(Classification::Decided(product_sign(
                    reference_cross,
                    if reversed {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    },
                )));
            }
            let reference_relation = policy.strict_predicate_pass(|| -> CurveResult<_> {
                Ok((
                    reference.tangent_cross_sign(chord, policy)?,
                    reference.tangent_dot_sign(chord, policy)?,
                ))
            })?;
            if let (
                Classification::Decided(RealSign::Zero),
                Classification::Decided(factor @ (RealSign::Negative | RealSign::Positive)),
            ) = reference_relation
            {
                return Ok(Classification::Decided(product_sign(
                    reference_cross,
                    factor,
                )));
            }
        }

        // Every retained selected-circle endpoint can publish a distinct
        // point on its traversal tangent.  When an adjacent chord has been
        // rebuilt through a different selected field, comparing the two
        // affine supports recovers exact tangency without asking interval
        // boxes to prove a correlated zero or invoking the full circle/line
        // intersection resultant.
        let tangent = self.endpoint_tangent_chord(start_endpoint, policy)?;
        if let Classification::Decided(Some(tangent)) = tangent {
            let direct =
                policy.strict_predicate_pass(|| tangent.tangent_cross_sign(chord, policy))?;
            if let Classification::Decided(cross) = direct {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "selected-circle-chord-tangent",
                    "endpoint-tangent-direction",
                );
                return Ok(Classification::Decided(cross));
            }
            match policy.strict_predicate_pass(|| tangent.chord_intersections(chord, policy))? {
                Classification::Decided(BezierAlgebraicChordPairIntersections2::Contacts(
                    contacts,
                )) => {
                    // Two noncollinear affine chords have one global tangent
                    // relation, independent of which finite contact owns the
                    // evidence.  The endpoint construction guarantees at
                    // least their authored endpoint contact when the finite
                    // adjacent chord is incident.
                    if let Some(contact) = contacts.first() {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "selected-circle-chord-tangent",
                            "endpoint-tangent-contact",
                        );
                        return Ok(Classification::Decided(contact.tangent_cross_sign()));
                    }
                }
                Classification::Decided(BezierAlgebraicChordPairIntersections2::Overlaps(_)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-circle-chord-tangent",
                        "endpoint-tangent-overlap",
                    );
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Uncertain(_) => {}
            }
        }

        let point = match self.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self.data.semicircle.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (
                Classification::Decided(point),
                Classification::Decided(center),
                Classification::Decided(chord_start),
                Classification::Decided(chord_end),
            ) = (
                algebraic_chord_endpoint_bounds_refined(&point, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(&center, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(chord.start(), refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(chord.end(), refinement_steps, policy),
            )
            else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let radial_x = real_interval_from_axis(&point, Axis2::X)
                .subtract(&real_interval_from_axis(&center, Axis2::X));
            let radial_y = real_interval_from_axis(&point, Axis2::Y)
                .subtract(&real_interval_from_axis(&center, Axis2::Y));
            let chord_x = real_interval_from_axis(&chord_end, Axis2::X)
                .subtract(&real_interval_from_axis(&chord_start, Axis2::X));
            let chord_y = real_interval_from_axis(&chord_end, Axis2::Y)
                .subtract(&real_interval_from_axis(&chord_start, Axis2::Y));
            let strict = &CurveContext::STRICT;
            let Some(projection) = radial_x
                .multiply(&chord_x)
                .and_then(|x| radial_y.multiply(&chord_y).map(|y| x.add(&y)))
            else {
                continue;
            };
            let projection_sign = if compare_reals(&projection.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(RealSign::Positive)
            } else if compare_reals(&projection.upper, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Less)
            {
                Some(RealSign::Negative)
            } else if compare_reals(&projection.lower, &Real::zero(), strict)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&projection.upper, &Real::zero(), strict)
                    == Some(std::cmp::Ordering::Equal)
            {
                Some(RealSign::Zero)
            } else {
                None
            };
            if let Some(projection_sign) = projection_sign {
                let clockwise = self.data.semicircle.is_clockwise() != self.data.reversed;
                return Ok(Classification::Decided(if clockwise {
                    projection_sign
                } else {
                    match projection_sign {
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                        RealSign::Positive => RealSign::Negative,
                    }
                }));
            }
        }
        // Interval refinement cannot prove an exact zero shared across
        // independently retained circle and chord fields. Replay the complete
        // selected-circle/chord kernel before APPROXIMATE_512 is allowed to
        // terminate that equality; its mapped contact owns both the finite
        // endpoint identity and the oriented tangent sign.
        let retained_intersections = self.data.semicircle.chord_intersections(chord, policy)?;
        if let Classification::Decided(contacts) = retained_intersections {
            let endpoint = self.endpoint_parameter(start_endpoint);
            for contact in contacts {
                let order = contact.cusp_parameter.cmp_by_refinement(endpoint, policy)?;
                match order {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "selected-circle-chord-tangent",
                            "complete-intersection-replay",
                        );
                        return Ok(Classification::Decided(if self.data.reversed {
                            product_sign(contact.tangent_cross_sign, RealSign::Negative)
                        } else {
                            contact.tangent_cross_sign
                        }));
                    }
                    Classification::Decided(_) | Classification::Uncertain(_) => {}
                }
            }
        }
        if permit_terminal_approximation && terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(RealSign::Zero))
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    }

    /// Signs this selected-circle traversal tangent dotted with one retained
    /// algebraic-chord direction.
    ///
    /// Tangent incidence is handled structurally above because an exact zero
    /// may span independent selected fields.  Once parallelism is known, the
    /// nonzero orientation is obtained either from the authored chord contact
    /// or from the common endpoint tangent chord; its dot interval separates
    /// without a circle/line resultant.
    pub(crate) fn endpoint_tangent_dot_algebraic_chord(
        &self,
        start_endpoint: bool,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        chord.validate_policy(policy)?;

        if let Classification::Decided(Some((reference, _, Some(reference_dot)))) =
            self.endpoint_chord_tangent_relation(start_endpoint, policy)?
        {
            if let Some(reversed) = chord.retained_normal_offset_tangent_reversal_to(&reference) {
                return Ok(Classification::Decided(product_sign(
                    reference_dot,
                    if reversed {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    },
                )));
            }
            if reference.tangent_cross_sign(chord, policy)?
                == Classification::Decided(RealSign::Zero)
                && let Classification::Decided(factor @ (RealSign::Negative | RealSign::Positive)) =
                    reference.tangent_dot_sign(chord, policy)?
            {
                return Ok(Classification::Decided(product_sign(reference_dot, factor)));
            }
        }

        let tangent = match self.endpoint_tangent_chord(start_endpoint, policy)? {
            Classification::Decided(Some(tangent)) => tangent,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        tangent.tangent_dot_sign(chord, policy)
    }

    /// Replays this concentric offset's endpoint tangent through the source
    /// circle before comparing it with an adjacent chord direction.
    ///
    /// The chord tangent belongs to the source boundary and need not pass
    /// through the offset circle's new endpoint.  Source incidence supplies
    /// the exact cross sign; the two retained signed-radius/reversal factors
    /// transport that sign without rebuilding either selected point field.
    pub(crate) fn endpoint_tangent_cross_algebraic_chord_from_source(
        &self,
        source: &Self,
        start_endpoint: bool,
        chord: &BezierAlgebraicChord2,
        permit_terminal_approximation: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        source.validate_policy(policy)?;
        let target_factor =
            match self.tangent_orientation_factor_from(&source.data.semicircle, policy)? {
                Classification::Decided(Some(factor)) => factor,
                Classification::Decided(None) | Classification::Uncertain(_) => {
                    return self.endpoint_tangent_cross_algebraic_chord(
                        start_endpoint,
                        chord,
                        permit_terminal_approximation,
                        policy,
                    );
                }
            };
        // A chord authored from both endpoints of this selected semicircle is
        // a certified secant. Its endpoint tangent-cross sign depends only on
        // traversal orientation and chord direction; neither recursive center
        // nor endpoint coordinate field participates in that predicate.
        if let Classification::Decided(Some(chord_reversed)) =
            source.authored_endpoint_chord_orientation(chord, policy)?
        {
            // `chord_reversed` is relative to fragment traversal. Reversing a
            // fragment reverses both that secant and its tangent, so their
            // cross sign is already the carrier sign below; only the target
            // fragment's retained orientation factor remains to transport.
            let mut source_cross = if source.data.semicircle.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            if !start_endpoint {
                source_cross = product_sign(source_cross, RealSign::Negative);
            }
            if chord_reversed {
                source_cross = product_sign(source_cross, RealSign::Negative);
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-chord-tangent",
                "authored-two-endpoint-secant",
            );
            return Ok(Classification::Decided(product_sign(
                source_cross,
                target_factor,
            )));
        }
        let source_factor =
            match source.tangent_orientation_factor_from(&source.data.semicircle, policy)? {
                Classification::Decided(Some(factor)) => factor,
                Classification::Decided(None) | Classification::Uncertain(_) => {
                    return self.endpoint_tangent_cross_algebraic_chord(
                        start_endpoint,
                        chord,
                        permit_terminal_approximation,
                        policy,
                    );
                }
            };
        // The offset kernel invokes this predicate only for neighboring
        // source fragments.  Their authored adjacency is stronger than two
        // independently reconstructed direction fields: once the source
        // circle and chord certify their unique tangent endpoint, a
        // concentric target circle has the same tangent line.  Consume that
        // exact topology before attempting a selected-field zero predicate.
        if policy.strict_predicate_pass(|| {
            source.certified_adjacent_chord_is_endpoint_only(chord, start_endpoint, policy)
        })? == Classification::Decided(true)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-chord-tangent",
                "concentric-authored-adjacency",
            );
            return Ok(Classification::Decided(RealSign::Zero));
        }
        match source.endpoint_tangent_cross_algebraic_chord(
            start_endpoint,
            chord,
            permit_terminal_approximation,
            policy,
        )? {
            Classification::Decided(sign) => Ok(Classification::Decided(product_sign(
                sign,
                product_sign(target_factor, source_factor),
            ))),
            Classification::Uncertain(_) => self.endpoint_tangent_cross_algebraic_chord(
                start_endpoint,
                chord,
                permit_terminal_approximation,
                policy,
            ),
        }
    }

    /// Transports an endpoint tangent-dot sign from a concentric source
    /// circle to this offset circle without moving the adjacent source chord.
    pub(crate) fn endpoint_tangent_dot_algebraic_chord_from_source(
        &self,
        source: &Self,
        start_endpoint: bool,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        source.validate_policy(policy)?;
        let target_factor = match self
            .tangent_orientation_factor_from(&source.data.semicircle, policy)?
        {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                return self.endpoint_tangent_dot_algebraic_chord(start_endpoint, chord, policy);
            }
        };
        let source_factor = match source
            .tangent_orientation_factor_from(&source.data.semicircle, policy)?
        {
            Classification::Decided(Some(factor)) => factor,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                return self.endpoint_tangent_dot_algebraic_chord(start_endpoint, chord, policy);
            }
        };
        match source.endpoint_tangent_dot_algebraic_chord(start_endpoint, chord, policy)? {
            Classification::Decided(sign) => Ok(Classification::Decided(product_sign(
                sign,
                product_sign(target_factor, source_factor),
            ))),
            Classification::Uncertain(_) => {
                self.endpoint_tangent_dot_algebraic_chord(start_endpoint, chord, policy)
            }
        }
    }

    /// Reuses one retained endpoint field for an exact concentric cardinal
    /// circle offset and caches that translated image on the result fragment.
    ///
    /// General frames, mapped cuts, and noncardinal exact parameters decline
    /// this path. Their endpoints remain owned by `endpoint_point_image`, so
    /// this optimization cannot alter STRICT/APPROXIMATE_512 decisions.
    pub(crate) fn translated_cardinal_offset_endpoint(
        &self,
        offset: &Self,
        start_endpoint: bool,
        source_endpoint: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        offset.validate_policy(policy)?;
        if self.data.reversed != offset.data.reversed
            || self.data.semicircle.is_clockwise() != offset.data.semicircle.is_clockwise()
            || self.data.semicircle.data.frame != offset.data.semicircle.data.frame
            || !self
                .endpoint_parameter(start_endpoint)
                .shares_exact_evidence(offset.endpoint_parameter(start_endpoint))
        {
            return Ok(Classification::Decided(None));
        }
        let Some((radial_x, radial_y)) = self.cardinal_endpoint_radial_components(start_endpoint)
        else {
            return Ok(Classification::Decided(None));
        };
        let radial_delta =
            offset.data.semicircle.radial_distance() - self.data.semicircle.radial_distance();
        let scaled_component = |component| match component {
            -1 => -radial_delta.clone(),
            0 => Real::zero(),
            1 => radial_delta.clone(),
            _ => unreachable!("cardinal radial component is closed"),
        };
        let translated = match BezierAlgebraicChord2::translated_endpoint(
            source_endpoint,
            &scaled_component(radial_x),
            &scaled_component(radial_y),
            policy,
        )? {
            Classification::Decided(translated) => translated,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let CurvePoint2(CurvePointData2::Algebraic(point)) = &translated {
            let source_start = start_endpoint != offset.data.reversed;
            let cache = if source_start {
                &offset.data.start_point_image
            } else {
                &offset.data.end_point_image
            };
            let _ = cache.set(Some(point.clone()));
        }
        Ok(Classification::Decided(Some(translated)))
    }

    /// Replays an endpoint on an exact concentric offset using the general
    /// point authority, including correlated selected-circle/chord cuts.
    pub(crate) fn concentric_offset_endpoint_point_evidence(
        &self,
        offset: &Self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        offset.validate_policy(policy)?;
        if self.data.reversed != offset.data.reversed
            || !self
                .endpoint_parameter(start_endpoint)
                .shares_exact_evidence(offset.endpoint_parameter(start_endpoint))
        {
            return Ok(Classification::Decided(None));
        }
        self.endpoint_parameter(start_endpoint)
            .concentric_offset_point_evidence(
                &self.data.semicircle,
                &offset.data.semicircle,
                policy,
            )
    }

    /// Returns whether this traversal endpoint and one retained circle/chord
    /// point are the two carriers created from the same mapped contact.
    pub(crate) fn shares_endpoint_point_evidence(
        &self,
        start_endpoint: bool,
        point: &BezierAlgebraicCuspChordPoint2,
    ) -> bool {
        let source_start = start_endpoint != self.data.reversed;
        let parameter = if source_start {
            &self.data.start
        } else {
            &self.data.end
        };
        matches!(
            parameter,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(data)
                if Arc::ptr_eq(data, &point.data)
        )
    }

    pub(crate) fn reversed(&self) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle: self.data.semicircle.clone(),
                start: self.data.start.clone(),
                end: self.data.end.clone(),
                start_point_image: cloned_once_lock(&self.data.start_point_image),
                end_point_image: cloned_once_lock(&self.data.end_point_image),
                certified_tangent_endpoints: self.data.certified_tangent_endpoints,
                reversed: !self.data.reversed,
                policy: self.data.policy,
            }),
        }
    }

    #[cfg(test)]
    pub(crate) fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        self.transform_similarity_cached(
            transform,
            &mut BezierAlgebraicCuspSemicircleSimilarityCache2::default(),
        )
    }

    pub(crate) fn transform_similarity_cached(
        &self,
        transform: &Similarity2,
        cache: &mut BezierAlgebraicCuspSemicircleSimilarityCache2,
    ) -> CurveResult<Self> {
        let semicircle = cache.semicircle(&self.data.semicircle, transform)?;
        let start = cache.parameter(&self.data.start, &self.data.semicircle, transform)?;
        let end = cache.parameter(&self.data.end, &self.data.semicircle, transform)?;
        Ok(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle,
                start,
                end,
                start_point_image: OnceLock::new(),
                end_point_image: OnceLock::new(),
                certified_tangent_endpoints: self.data.certified_tangent_endpoints,
                reversed: self.data.reversed,
                policy: self.data.policy,
            }),
        })
    }

    /// Composes a left offset while retaining this fragment's exact circle
    /// parameter interval.
    ///
    /// Reversed fragments traverse the supporting semicircle in the opposite
    /// direction, so their left parallel is the carrier's right parallel.
    /// Endpoint images are deliberately rebuilt from the new radius rather
    /// than copied from the source circle.
    pub(crate) fn offset_left(
        &self,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        self.validate_policy(policy)?;
        let carrier_distance = if self.data.reversed {
            -distance
        } else {
            distance.clone()
        };
        let semicircle = match self
            .data
            .semicircle
            .offset_left(&carrier_distance, policy)?
        {
            Classification::Decided(Some(semicircle)) => semicircle,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle,
                start: self.data.start.clone(),
                end: self.data.end.clone(),
                start_point_image: OnceLock::new(),
                end_point_image: OnceLock::new(),
                certified_tangent_endpoints: self.data.certified_tangent_endpoints,
                reversed: self.data.reversed,
                policy: self.data.policy,
            }),
        })))
    }

    pub(crate) fn representative_parameter(&self) -> CurveResult<Classification<Real>> {
        self.data
            .start
            .strict_scalar_between(&self.data.end, &self.data.policy)
    }

    pub(crate) fn representative_point(
        &self,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
        let parameter = match self.representative_parameter()? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.data.semicircle.point_at(&parameter, &self.data.policy)
    }

    /// Prepares one exact minor-arc evaluator for an algebraic query ray.
    ///
    /// Exact `Real` subrange cuts preserve a single cusp field. A mapped cut
    /// enters the same fast path when its correspondence proves a rational
    /// value, or when its geometric point has one selected carrier field and
    /// the companion endpoint is exactly rational. Genuinely distinct endpoint
    /// fields retain separate images for the cold multi-field chord predicate.
    pub(crate) fn algebraic_ray_evaluator(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleAlgebraicRay2>> {
        self.validate_policy(policy)?;
        let Some(_) = self.data.semicircle.data.frame.rational() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut start = match self
            .data
            .start
            .coincident_point_image(&self.data.semicircle, policy)?
        {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut end = match self
            .data
            .end
            .coincident_point_image(&self.data.semicircle, policy)?
        {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let endpoints_share_field = start.parameter() == end.parameter();
        if !endpoints_share_field {
            if let (Some(start_point), Some(end_point)) =
                (start.exact_point(policy), end.exact_point(policy))
            {
                start = algebraic_constant_point_image(
                    &start_point,
                    self.data.semicircle.cusp_parameter(),
                    policy,
                );
                end = algebraic_constant_point_image(
                    &end_point,
                    self.data.semicircle.cusp_parameter(),
                    policy,
                );
            } else if let (Some(point), Some(parameter)) =
                (start.exact_point(policy), end.retained_parameter())
            {
                start = algebraic_constant_point_image(&point, parameter, policy);
            } else if let (Some(point), Some(parameter)) =
                (end.exact_point(policy), start.retained_parameter())
            {
                end = algebraic_constant_point_image(&point, parameter, policy);
            }
        }
        let (start, end) = if self.data.reversed {
            (end, start)
        } else {
            (start, end)
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleAlgebraicRay2 {
                start,
                end,
                center: self.data.semicircle.center_point_image(policy)?,
                radius_squared: self.data.semicircle.radial_distance()
                    * self.data.semicircle.radial_distance(),
                clockwise: self.data.semicircle.is_clockwise() ^ self.data.reversed,
            },
        ))
    }

    pub(crate) fn conservative_bounds(&self) -> CurveResult<Classification<Aabb2>> {
        self.data.semicircle.conservative_bounds(&self.data.policy)
    }

    pub(crate) fn endpoint_analytic_source(
        &self,
        start_endpoint: bool,
    ) -> Option<(BezierParallel2, CurveParameter2)> {
        let source_start = start_endpoint != self.data.reversed;
        let parameter = if source_start {
            &self.data.start
        } else {
            &self.data.end
        };
        match parameter {
            BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) => {
                let parallel = if parameter == &Real::zero() {
                    self.data.semicircle.start_parallel()?
                } else if parameter == &Real::one() {
                    self.data.semicircle.end_parallel()?
                } else {
                    return None;
                };
                Some((parallel, self.data.semicircle.selected_frame_parameter()?))
            }
            BezierAlgebraicCuspSemicircleParameter2::Mapped(data) => {
                if let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact { parallel, parameter, policy, .. } = data.as_ref() {
                    return (self.data.policy.accepts_retained_policy(*policy)
                        && data.semicircle_carrier() == &self.data.semicircle)
                        .then(|| (parallel.clone(), parameter.clone()));
                }
                let BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                    parallel,
                    parameter,
                    policy,
                } = data.coincident_tangent_source()?
                else {
                    return None;
                };
                (self.data.policy.accepts_retained_policy(policy)
                    && data.semicircle_carrier() == &self.data.semicircle)
                    .then(|| (parallel.clone(), parameter.clone().into()))
            }
        }
    }

    pub(crate) fn endpoint_point_image(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierAlgebraicPointImage2>> {
        self.validate_policy(policy)?;
        let source_start = start_endpoint != self.data.reversed;
        self.source_endpoint_point_image(source_start, policy)
    }

    pub(in crate::bezier_offset) fn source_endpoint_point_image(
        &self,
        source_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<RationalBezierAlgebraicPointImage2>> {
        let cache = if source_start {
            &self.data.start_point_image
        } else {
            &self.data.end_point_image
        };
        if let Some(point) = cache.get() {
            return Ok(point.clone());
        }
        let parameter = if source_start {
            &self.data.start
        } else {
            &self.data.end
        };
        let point = match parameter.coincident_point_image(&self.data.semicircle, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => None,
        };
        let _ = cache.set(point.clone());
        Ok(point)
    }

    /// Returns exact endpoint evidence without forcing correlated selected
    /// fields into a single coordinate tower.
    pub(crate) fn endpoint_point_evidence(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        self.validate_policy(policy)?;
        let source_start = start_endpoint != self.data.reversed;
        let parameter = if source_start {
            &self.data.start
        } else {
            &self.data.end
        };
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter {
            let source = data.semicircle_carrier();
            if source != &self.data.semicircle {
                return parameter.concentric_offset_point_evidence(
                    source,
                    &self.data.semicircle,
                    policy,
                );
            }
        }
        let point_evidence = parameter.coincident_point_evidence(&self.data.semicircle, policy)?;
        let Classification::Decided(Some(CurvePoint2(CurvePointData2::Algebraic(point_image)))) =
            &point_evidence
        else {
            return Ok(point_evidence);
        };
        let peer = if source_start {
            &self.data.end
        } else {
            &self.data.start
        };
        let peer_retains_nonrational_parallel = matches!(
            peer,
            BezierAlgebraicCuspSemicircleParameter2::Mapped(data)
                if matches!(
                    data.coincident_tangent_source(),
                    Some(BezierAlgebraicCuspSemicircleMappedTangentSource2::Parallel {
                        parameter: BezierParameter2::Algebraic(_),
                        ..
                    })
                )
        );
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter else {
            return Ok(point_evidence);
        };
        if !peer_retains_nonrational_parallel || data.semicircle_carrier() != &self.data.semicircle
        {
            return Ok(point_evidence);
        }
        Ok(Classification::Decided(Some(CurvePoint2::from(
            BezierAlgebraicCuspChordDerivedPoint2::from_mapped_source(
                data.clone(),
                Some(CurvePoint2::from(point_image.clone())),
                Real::one(),
            ),
        ))))
    }

    /// Returns the unique point in one requested angular direction at an
    /// authored chord-distance from a traversal endpoint.
    ///
    /// On the rational half-circle chart, adding an angle with
    /// `q = tan(theta/2)` is a degree-one Mobius map. The chord setback `s`
    /// fixes `q^2 = s^2 / (4 r^2-s^2)`, so no resultant or root allocation is
    /// needed. `outward` selects the incident extension direction; the other
    /// direction is returned only when it is a strict authored-span trim.
    /// Represented endpoints stay represented; mapped endpoints keep their
    /// original point field and one exact center-relative rotation. A result
    /// on the other half-circle chart sets the final tuple member.
    pub(crate) fn endpoint_chord_setback_cut(
        &self,
        start_endpoint: bool,
        setback: &Real,
        outward: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<Option<(BezierAlgebraicCuspSemicircleParameter2, CurvePoint2, bool)>>,
    > {
        self.endpoint_chord_setback_cut_internal(start_endpoint, setback, outward, true, policy)
    }

    /// Returns the same exact setback point on the supporting circle without
    /// clipping it to this representation fragment. CurveRegion uses this only
    /// after proving that an adjacent smooth run may own the inward cut; the
    /// run rebinder still rejects points outside that finite authored domain.
    pub(crate) fn endpoint_chord_setback_support_cut(
        &self,
        start_endpoint: bool,
        setback: &Real,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<Option<(BezierAlgebraicCuspSemicircleParameter2, CurvePoint2, bool)>>,
    > {
        self.endpoint_chord_setback_cut_internal(start_endpoint, setback, false, false, policy)
    }

    pub(in crate::bezier_offset) fn endpoint_chord_setback_cut_internal(
        &self,
        start_endpoint: bool,
        setback: &Real,
        outward: bool,
        clip_inward_to_fragment: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<Option<(BezierAlgebraicCuspSemicircleParameter2, CurvePoint2, bool)>>,
    > {
        self.validate_policy(policy)?;
        let radius_squared =
            self.data.semicircle.radial_distance() * self.data.semicircle.radial_distance();
        let remaining = Real::from(4_i8) * radius_squared - setback * setback;
        match real_sign(&remaining, policy) {
            Some(RealSign::Positive) => {}
            // Both angular directions meet at the diameter antipode. It can
            // never be a strict trim from a boundary of this half-circle, but
            // it is the terminal exact extension point on the other chart.
            Some(RealSign::Zero) => {
                if !outward && clip_inward_to_fragment {
                    return Ok(Classification::Decided(None));
                }
                return self.endpoint_chord_setback_parameter(start_endpoint, None, true, policy);
            }
            Some(RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let certified_strict_interior = if !outward
            && clip_inward_to_fragment
            && real_sign(setback, policy) == Some(RealSign::Positive)
        {
            let corner = self.endpoint_point_evidence(start_endpoint, policy)?;
            let opposite = self.endpoint_point_evidence(!start_endpoint, policy)?;
            match (corner, opposite) {
                (
                    Classification::Decided(Some(corner)),
                    Classification::Decided(Some(opposite)),
                ) => {
                    match algebraic_point_distance_squared_at_most(
                        &corner,
                        &opposite,
                        &(setback * setback),
                        policy,
                    ) {
                        Classification::Decided(false) => {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "selected-circle-chamfer-trim-domain",
                                "endpoint-chord-distance",
                            );
                            true
                        }
                        Classification::Decided(true) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Uncertain(_) => false,
                    }
                }
                _ => false,
            }
        } else {
            false
        };
        let half_angle_magnitude = (setback / remaining.sqrt()?)?;
        let source_start = start_endpoint != self.data.reversed;
        let interior_half_angle = if source_start {
            half_angle_magnitude
        } else {
            -half_angle_magnitude
        };
        let physical_half_angle = if outward {
            -interior_half_angle
        } else {
            interior_half_angle
        };
        let source = self.endpoint_parameter(start_endpoint);
        let complementary = if !outward && clip_inward_to_fragment {
            // An admissible inward cut is strictly inside this finite
            // fragment, whose whole parameter range belongs to the current
            // half-circle chart. Let the retained parameter-order predicate
            // accept or reject that candidate directly; reconstructing its
            // chart from three independent affine signs can lose the shared
            // nested chamfer provenance before that stronger evidence runs.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-circle-chamfer-chart",
                "inward-fragment",
            );
            false
        } else {
            match cusp_chamfer_parameter_uses_complement(source, &physical_half_angle, policy)? {
                Classification::Decided(complementary) => complementary,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let candidate = match self.endpoint_chord_setback_parameter(
            start_endpoint,
            Some(&physical_half_angle),
            complementary,
            policy,
        )? {
            Classification::Decided(Some(candidate)) => candidate,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if certified_strict_interior {
            return Ok(Classification::Decided(Some(candidate)));
        }
        if outward || !clip_inward_to_fragment {
            return Ok(Classification::Decided(Some(candidate)));
        }
        let (parameter, point, complementary) = candidate;
        if complementary {
            return Ok(Classification::Decided(None));
        }

        let compare = |boundary: &BezierAlgebraicCuspSemicircleParameter2| {
            parameter.cmp_by_refinement(boundary, policy)
        };
        let start = match compare(&self.data.start)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match compare(&self.data.end)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if start != std::cmp::Ordering::Greater || end != std::cmp::Ordering::Less {
            return Ok(Classification::Decided(None));
        }
        Ok(Classification::Decided(Some((parameter, point, false))))
    }

    pub(in crate::bezier_offset) fn endpoint_chord_setback_parameter(
        &self,
        start_endpoint: bool,
        physical_half_angle: Option<&Real>,
        complementary: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<Option<(BezierAlgebraicCuspSemicircleParameter2, CurvePoint2, bool)>>,
    > {
        let source = self.endpoint_parameter(start_endpoint);
        let target_semicircle = if complementary {
            self.data.semicircle.complementary_half()
        } else {
            self.data.semicircle.clone()
        };
        let chart_half_angle = match physical_half_angle {
            Some(half_angle) if complementary => (-Real::one() / half_angle.clone())?,
            Some(half_angle) => half_angle.clone(),
            None => Real::zero(),
        };
        let represented = source.scalar_value(policy)?;
        let (parameter, point) = match represented {
            Classification::Decided(Some(source)) => {
                let parameter =
                    match cusp_chamfer_parameter_value(&source, &chart_half_angle, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let point = match target_semicircle.point_evidence_at(&parameter, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (
                    BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
                    point,
                )
            }
            Classification::Decided(None) => {
                let BezierAlgebraicCuspSemicircleParameter2::Mapped(source_data) = source else {
                    unreachable!("an inline cusp parameter has a represented scalar value")
                };
                if source_data.semicircle_carrier().data.frame != self.data.semicircle.data.frame {
                    return Err(CurveError::Topology(
                        "cusp chamfer endpoint did not retain its supporting center".into(),
                    ));
                }
                let source_point = match self.endpoint_point_evidence(start_endpoint, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let (radial_scale, perpendicular_scale) = match physical_half_angle {
                    Some(half_angle) => {
                        let half_angle_squared = half_angle * half_angle;
                        let rotation_denominator = Real::one() + &half_angle_squared;
                        (
                            ((Real::one() - &half_angle_squared) / &rotation_denominator)?,
                            (self.data.semicircle.turn_sign() * Real::from(2_i8) * half_angle
                                / &rotation_denominator)?,
                        )
                    }
                    None => (Real::from(-1_i8), Real::zero()),
                };
                let point = CurvePoint2::from(
                    BezierAlgebraicCuspChordDerivedPoint2::rotated_from_mapped_source(
                        source_data.clone(),
                        source_point,
                        radial_scale,
                        perpendicular_scale,
                    ),
                );
                let parameter = BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                    BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer {
                        semicircle: target_semicircle,
                        source: source.clone(),
                        half_angle: chart_half_angle,
                        point: point.clone(),
                        policy: policy.retained_object_policy_with_dependencies([self.data.policy]),
                    },
                ));
                (parameter, point)
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some((
            parameter,
            point,
            complementary,
        ))))
    }

    /// Verifies one authored fragment endpoint against the circle equation,
    /// then retains that exact endpoint image for all later topology replay.
    pub(crate) fn certify_and_cache_authored_endpoint(
        &self,
        start_endpoint: bool,
        expected: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        let source_start = start_endpoint != self.data.reversed;
        let actual = match self.endpoint_point_evidence(start_endpoint, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let equal = actual.same_point(expected, policy);
        if equal == Classification::Decided(true)
            && let Some(cached) = expected
                .as_algebraic()
                .or_else(|| actual.as_algebraic())
                .cloned()
        {
            let cache = if source_start {
                &self.data.start_point_image
            } else {
                &self.data.end_point_image
            };
            let _ = cache.set(Some(cached));
        }
        Ok(equal)
    }

    /// Recognizes only retained-provenance tangent adjacency. A miss must
    /// enter the authoritative circle/chord kernel instead of turning this
    /// optional Boolean shortcut into another intersection engine.
    pub(crate) fn authored_adjacent_chord_is_structurally_endpoint_only(
        &self,
        chord: &BezierAlgebraicChord2,
        shared_circle_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        self.validate_policy(policy)?;
        chord.validate_policy(policy)?;
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) =
            self.endpoint_parameter(shared_circle_start)
        else {
            return Ok(false);
        };
        match parameter.as_ref() {
            BezierAlgebraicCuspSemicircleMappedParameterData2::Rational { contact, .. }
                if matches!(
                    &contact.correlation,
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                        chord: source,
                        circle_cross_chord: RealSign::Zero,
                    } if source.shares_retained_support(chord)
                        || chord.shares_retained_support(source)
                ) =>
            {
                return Ok(true);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact }
                if contact.tangent_cross_sign == RealSign::Zero
                    && (map.data.chord == *chord
                        || map.data.chord.shares_retained_support(chord)
                        || chord.shares_retained_support(&map.data.chord)) =>
            {
                return Ok(true);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                parallel,
                parameter,
                ..
            } if parameter.scalar().is_some_and(|source_parameter| chord.parallel_tangent_contacts().iter().any(|contact| {
                contact.parallel().source() == parallel.source()
                    && compare_reals(
                        contact.parameter(),
                        source_parameter,
                        &CurveContext::STRICT,
                    ) == Some(std::cmp::Ordering::Equal)
            })) => return Ok(true),
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                chord: source,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                chord: source,
                ..
            } if source.shares_retained_support(chord)
                || chord
                    .retained_normal_offset_tangent_reversal_to(source)
                    .is_some() =>
            {
                return Ok(true);
            }
            BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                chord: source,
                point,
                ..
            }
            | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                chord: source,
                point,
                ..
            } if source.shares_retained_support(chord)
                && [chord.start(), chord.end()]
                    .into_iter()
                    .any(|endpoint| endpoint == point) =>
            {
                return Ok(true);
            }
            _ => {}
        }
        Ok(false)
    }

    /// Reuses an endpoint contact only after proving it is the sole contact
    /// between this finite circle fragment and chord. This geometry certificate
    /// is independent of path adjacency or region boundary ownership.
    pub(crate) fn certified_chord_endpoint_contact(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleRetainedChordContact2>>>
    {
        let mut uncertainty = None;
        for cusp_at_start in [true, false] {
            let point = match self.endpoint_point_evidence(cusp_at_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            for (chord_at_start, endpoint) in [(true, chord.start()), (false, chord.end())] {
                if !point.shares_storage(endpoint) {
                    // Endpoint replay is only an accelerator. Suppress the
                    // terminal here so an inconclusive retained-point
                    // comparison falls through to the complete circle/chord
                    // kernel instead of weakening an otherwise certified
                    // arrangement.
                    let first = policy.strict_predicate_pass(|| point.same_point(endpoint, policy));
                    let second = if first == Classification::Decided(true) {
                        Classification::Decided(true)
                    } else {
                        policy.strict_predicate_pass(|| endpoint.same_point(&point, policy))
                    };
                    match (first, second) {
                        (Classification::Decided(true), _) | (_, Classification::Decided(true)) => {
                        }
                        (Classification::Uncertain(reason), _)
                        | (_, Classification::Uncertain(reason)) => {
                            uncertainty.get_or_insert(reason);
                            continue;
                        }
                        (Classification::Decided(false), Classification::Decided(false)) => {
                            continue;
                        }
                    }
                }
                match policy.strict_predicate_pass(|| {
                    self.certified_adjacent_chord_is_endpoint_only(chord, cusp_at_start, policy)
                })? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        uncertainty.get_or_insert(reason);
                        continue;
                    }
                }
                let cross = match policy.strict_predicate_pass(|| {
                    self.endpoint_tangent_cross_algebraic_chord(cusp_at_start, chord, false, policy)
                })? {
                    Classification::Decided(cross) => cross,
                    Classification::Uncertain(reason) => {
                        uncertainty.get_or_insert(reason);
                        continue;
                    }
                };
                return Ok(Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                        cusp_parameter: self.endpoint_parameter(cusp_at_start).clone(),
                        chord_parameter: if chord_at_start {
                            chord.start_parameter()
                        } else {
                            chord.end_parameter()
                        },
                        point,
                        // Contacts use the supporting circle's parameter
                        // orientation, independent of this fragment's traversal.
                        tangent_cross_sign: product_sign(
                            cross,
                            if self.is_reversed() {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            },
                        ),
                    },
                )));
            }
        }
        Ok(uncertainty.map_or(Classification::Decided(None), Classification::Uncertain))
    }

    /// Certifies that a directly framed round join and an adjacent retained
    /// chord share only their authored endpoint.
    ///
    /// At parameters 0 and 1 the half-circle tangent is perpendicular to the
    /// stored cardinal normal; at 1/2 it is parallel to that normal. A nonzero
    /// circle and a line through one of those points in the certified tangent
    /// direction have exactly one support contact, so no general selected-field
    /// circle/chord resultant is needed for authored adjacency.
    pub(crate) fn certified_adjacent_chord_is_endpoint_only(
        &self,
        chord: &BezierAlgebraicChord2,
        shared_circle_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        chord.validate_policy(policy)?;

        // A selected chord-normal endpoint and the paired parallel chord that
        // owns it have a structural tangent-line certificate. The circle
        // radius is the source chord's unit left normal, while every point of
        // the adjacent chord has the source tangent direction; a nonzero
        // circle and that line therefore meet only at the authored endpoint.
        if let BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) =
            self.endpoint_parameter(shared_circle_start)
            && let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                parallel,
                parameter: source_parameter,
                ..
            } = parameter.as_ref()
        {
            let retained_contact = source_parameter.scalar().is_some_and(|source_parameter| {
                chord.parallel_tangent_contacts().iter().any(|contact| {
                    contact.parallel().source() == parallel.source()
                        && compare_reals(
                            contact.parameter(),
                            source_parameter,
                            &CurveContext::STRICT,
                        ) == Some(std::cmp::Ordering::Equal)
                })
            });
            if retained_contact {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-parallel-authored-tangent-leg",
                );
                return Ok(Classification::Decided(true));
            }
            let represented_tangent = chord.certified_unit_tangent();
            let represented_parallel = match represented_tangent.as_ref() {
                Some((x, y)) => policy.strict_predicate_pass(|| {
                    parallel.vector_tangent_cross_and_dot_signs(source_parameter, x, y, policy)
                })?,
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            };
            if matches!(
                represented_parallel,
                Classification::Decided((RealSign::Zero, RealSign::Negative | RealSign::Positive))
            ) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-parallel-represented-tangent",
                );
                return Ok(Classification::Decided(true));
            }
        }
        for circle_start in [true, false] {
            let BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) =
                self.endpoint_parameter(circle_start)
            else {
                continue;
            };
            if let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                chord: source,
                point,
                ..
            } = parameter.as_ref()
                && source.shares_retained_support(chord)
                && [chord.start(), chord.end()]
                    .into_iter()
                    .any(|endpoint| endpoint == point)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-chord-parallel-normal-tangent",
                );
                return Ok(Classification::Decided(true));
            }
            let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                chord: source,
                point,
                ..
            } = parameter.as_ref()
            else {
                continue;
            };
            if source.shares_retained_support(chord)
                && [chord.start(), chord.end()].into_iter().any(|endpoint| {
                    endpoint == point
                        || endpoint.same_point(point, policy) == Classification::Decided(true)
                })
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-chord-normal-retained-support-tangent",
                );
                return Ok(Classification::Decided(true));
            }
            let (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
            ) = (chord.start(), chord.end())
            else {
                continue;
            };
            if !start.shares_carrier(end)
                || start.data.direction != BezierAlgebraicChordUnitDisplacement2::LeftNormal
                || !(start.data.source == *source || start.data.source == source.reversed())
                || ![chord.start(), chord.end()]
                    .into_iter()
                    .any(|endpoint| endpoint == point)
            {
                continue;
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-chord-kernel",
                "selected-chord-normal-tangent",
            );
            return Ok(Classification::Decided(true));
        }
        // Prefer the endpoint-bearing certificate above when it is available.
        // The support-only relation remains a complete fallback for trimmed or
        // transformed chords that no longer retain the authored point wrapper.
        for circle_start in [true, false] {
            let BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) =
                self.endpoint_parameter(circle_start)
            else {
                continue;
            };
            let source = match parameter.as_ref() {
                BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                    chord,
                    ..
                }
                | BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordParallelNormalContact {
                    chord,
                    ..
                } => Some(chord),
                _ => None,
            };
            if let Some(source) = source
                && (source.shares_retained_support(chord)
                    || chord
                        .retained_normal_offset_tangent_reversal_to(source)
                        .is_some())
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-chord-normal-offset-tangent",
                );
                return Ok(Classification::Decided(true));
            }
        }

        // A represented line enters the selected parallel-normal circle
        // authority as an exact rational parallel. Trimming that line at an
        // algebraic fillet contact rebuilds its finite remainder as a chord,
        // but does not change the infinite tangent support. Reuse the exact
        // rational component to certify that support and the shared authored
        // endpoint without introducing a circle/chord resultant.
        if let Some(chord_line) = chord
            .exact_line()
            .or_else(|| chord.strict_provenance_support_line(policy))
        {
            for circle_start in [true, false] {
                let Some((parallel, _)) = self.endpoint_analytic_source(circle_start) else {
                    continue;
                };
                let component = match parallel.exact_rational_parallel_component(policy)? {
                    Classification::Decided(Some(component)) => component,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
                if !matches!(
                    component.relation_to_line_with_contacts(&chord_line, policy),
                    Classification::Decided(BezierLineContactRelation::OnSupportingLine)
                ) {
                    continue;
                }
                let circle_point = match self.endpoint_point_evidence(circle_start, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
                if [chord.start(), chord.end()].into_iter().any(|endpoint| {
                    endpoint == &circle_point
                        || endpoint.same_point(&circle_point, policy)
                            == Classification::Decided(true)
                }) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-chord-kernel",
                        "selected-parallel-normal-line-tangent",
                    );
                    return Ok(Classification::Decided(true));
                }
            }
        }

        // Retained miter legs keep the authored infinite tangent support as
        // their root chord. Replaying that construction is a complete proof:
        // a nonzero circle and its tangent line share exactly one point, and
        // the adjacent finite leg contains the same authored endpoint.
        let support = chord.retained_support();
        for circle_start in [true, false] {
            let circle_point = match self.endpoint_point_evidence(circle_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            let tangent_point = match self.endpoint_tangent_support_point(circle_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            let support_matches = (support.start() == &circle_point
                && support.end() == &tangent_point)
                || (support.end() == &circle_point && support.start() == &tangent_point);
            if support_matches
                && [chord.start(), chord.end()]
                    .into_iter()
                    .any(|point| point == &circle_point)
            {
                return Ok(Classification::Decided(true));
            }
        }

        // A represented unit-normal circle frame and a retained chord unit
        // tangent are already a complete tangent-line certificate. This is
        // the common-corner chord/chord fillet anchor: its selected center is
        // algebraic, but both local directions remain canonical Real vectors.
        if let Some(chord_tangent) = chord.certified_unit_tangent() {
            for circle_start in [true, false] {
                let circle_tangent =
                    match self.represented_endpoint_tangent(circle_start, policy)? {
                        Classification::Decided(Some(tangent)) => tangent,
                        Classification::Decided(None) | Classification::Uncertain(_) => continue,
                    };
                if Real::diff_of_products(
                    &circle_tangent.0,
                    &chord_tangent.1,
                    &circle_tangent.1,
                    &chord_tangent.0,
                )
                .zero_status()
                    != ZeroKnowledge::Zero
                {
                    continue;
                }
                let circle_point = match self.endpoint_point_evidence(circle_start, policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
                if [chord.start(), chord.end()].into_iter().any(|endpoint| {
                    endpoint == &circle_point
                        || endpoint.same_point(&circle_point, policy)
                            == Classification::Decided(true)
                }) {
                    return Ok(Classification::Decided(true));
                }
            }
        }

        let chord_axis = match chord.axis_direction(policy)? {
            Classification::Decided(Some(direction)) => Some(direction.axis()),
            Classification::Decided(None) => None,
            Classification::Uncertain(_) => None,
        };
        if let (Some(cardinal_normal), Some(chord_axis)) = (
            self.data
                .semicircle
                .data
                .frame
                .rational()
                .and_then(|frame| frame.data.cardinal_normal),
            chord_axis,
        ) {
            let half = (Real::one() / Real::from(2_i8))?;
            for start_endpoint in [true, false] {
                let parameter = self.endpoint_parameter(start_endpoint);
                let parameter = match parameter.scalar_value(policy)? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(_) => continue,
                };
                let tangent_axis = if compare_reals(&parameter, &half, policy)
                    == Some(std::cmp::Ordering::Equal)
                {
                    if cardinal_normal.0 == 0 {
                        Axis2::Y
                    } else {
                        Axis2::X
                    }
                } else if compare_reals(&parameter, &Real::zero(), policy)
                    == Some(std::cmp::Ordering::Equal)
                    || compare_reals(&parameter, &Real::one(), policy)
                        == Some(std::cmp::Ordering::Equal)
                {
                    if cardinal_normal.0 == 0 {
                        Axis2::X
                    } else {
                        Axis2::Y
                    }
                } else {
                    continue;
                };
                if tangent_axis != chord_axis {
                    continue;
                }
                let point = match self.endpoint_point_image(start_endpoint, policy)? {
                    Some(point) => CurvePoint2::from(point),
                    None => continue,
                };
                if [chord.start(), chord.end()]
                    .into_iter()
                    .any(|endpoint| point.shares_storage(endpoint))
                {
                    return Ok(Classification::Decided(true));
                }
                let mut shared = false;
                for endpoint in [chord.start(), chord.end()] {
                    match point.same_point(endpoint, policy) {
                        Classification::Decided(true) => {
                            shared = true;
                            break;
                        }
                        Classification::Decided(false) | Classification::Uncertain(_) => {}
                    }
                }
                if shared {
                    return Ok(Classification::Decided(true));
                }
            }
        }

        // A bevel can join an axis-offset endpoint to a radial circle-offset
        // endpoint at a noncardinal mapped cut. If its other endpoint is
        // strictly inside the disk, convexity puts every open bevel point
        // inside too. If it instead departs radially outward or tangentially,
        // squared distance is monotone on the segment. Either proof makes the
        // shared boundary point the only possible circle contact.
        for circle_start in [true, false] {
            let circle_point = match self.endpoint_point_evidence(circle_start, policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            let chord_points = [chord.start(), chord.end()];
            let departs_outward_or_tangent = |chord_index: usize| -> CurveResult<bool> {
                let opposite = |sign| match sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                };
                let cross = match self.endpoint_tangent_cross_algebraic_chord(
                    circle_start,
                    chord,
                    false,
                    policy,
                )? {
                    Classification::Decided(cross) => cross,
                    Classification::Uncertain(_) => return Ok(false),
                };
                // `cross` uses the chord's stored start-to-end direction. At
                // its end the finite segment leaves the shared circle point
                // in the opposite direction.
                let toward_other_cross = if chord_index == 0 {
                    cross
                } else {
                    opposite(cross)
                };
                // For clockwise circle traversal, T x D = R dot D; for
                // counterclockwise traversal the sign is reversed. A
                // nonnegative radial derivative at a boundary point makes
                // |R+tD|^2 monotone increasing for every t >= 0, so the
                // finite adjacent segment has no second circle contact.
                let clockwise = self.data.semicircle.is_clockwise() != self.data.reversed;
                let radial_dot = if clockwise {
                    toward_other_cross
                } else {
                    opposite(toward_other_cross)
                };
                Ok(matches!(radial_dot, RealSign::Zero | RealSign::Positive))
            };
            let other_endpoint_is_strictly_inside = |chord_index: usize| -> CurveResult<bool> {
                let incidence = self
                    .data
                    .semicircle
                    .retained_point_incidence_sign(chord_points[1 - chord_index], policy)?;
                Ok(matches!(
                    incidence,
                    Classification::Decided(RealSign::Negative)
                ))
            };
            if let Some(chord_index) = chord_points
                .iter()
                .position(|endpoint| circle_point == **endpoint)
            {
                if departs_outward_or_tangent(chord_index)?
                    || other_endpoint_is_strictly_inside(chord_index)?
                {
                    return Ok(Classification::Decided(true));
                }
                continue;
            }
            for (chord_index, chord_point) in chord_points.iter().enumerate() {
                if circle_point.same_point(chord_point, policy) != Classification::Decided(true) {
                    continue;
                }
                if departs_outward_or_tangent(chord_index)?
                    || other_endpoint_is_strictly_inside(chord_index)?
                {
                    return Ok(Classification::Decided(true));
                }
            }
        }
        Ok(Classification::Decided(false))
    }

    #[cfg(test)]
    pub(crate) fn endpoint_exact_point(
        &self,
        start_endpoint: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<Point2>> {
        if let Some(point) = self
            .endpoint_point_image(start_endpoint, policy)?
            .and_then(|point| point.exact_point(policy))
        {
            return Ok(Some(point));
        }
        let Some((parallel, parameter)) = self.endpoint_analytic_source(start_endpoint) else {
            return Ok(None);
        };
        let Some(BezierParameter2::Algebraic(parameter)) = parameter.as_bezier_parameter() else {
            return Ok(None);
        };
        let parameter = match parameter.represented_exact_point_with_policy(policy)? {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        Ok(match parallel.point_at_with_policy(&parameter, policy)? {
            Classification::Decided(point) => Some(point),
            Classification::Uncertain(_) => None,
        })
    }

    pub(crate) fn endpoint_parameter(
        &self,
        start_endpoint: bool,
    ) -> &BezierAlgebraicCuspSemicircleParameter2 {
        let source_start = start_endpoint != self.data.reversed;
        if source_start {
            &self.data.start
        } else {
            &self.data.end
        }
    }

    /// Recognizes a chord authored from this fragment's two traversal
    /// endpoints. `Some(false)` follows the fragment and `Some(true)` opposes
    /// it. The result is pure retained incidence; no coordinate field is
    /// flattened and no approximate equality becomes construction evidence.
    pub(crate) fn authored_endpoint_chord_orientation(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<bool>>> {
        self.validate_policy(policy)?;
        chord.validate_policy(policy)?;
        let start = match self.endpoint_point_evidence(true, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.endpoint_point_evidence(false, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let same = |first: &CurvePoint2, second: &CurvePoint2| {
            first.shares_storage(second) || first == second
        };
        Ok(Classification::Decided(
            if same(chord.start(), &start) && same(chord.end(), &end) {
                Some(false)
            } else if same(chord.start(), &end) && same(chord.end(), &start) {
                Some(true)
            } else {
                None
            },
        ))
    }

    /// Returns whether the traversal begins at an inline represented
    /// parameter rather than a retained correlation.
    pub(crate) fn traversal_start_parameter_is_exact(&self) -> bool {
        matches!(
            self.endpoint_parameter(true),
            BezierAlgebraicCuspSemicircleParameter2::Exact(_)
        )
    }

    /// Coalesces two traversal-contiguous fragments of one selected circle.
    ///
    /// Shared retained evidence is the constant-time path. An independently
    /// reconstructed but exactly equal cut may use the existing parameter
    /// comparator; any inconclusive comparison simply declines coalescing so
    /// the complete per-fragment offset path remains authoritative.
    pub(crate) fn coalesced_with_next(
        &self,
        next: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Self>> {
        self.validate_policy(policy)?;
        next.validate_policy(policy)?;
        if self.data.semicircle != next.data.semicircle || self.data.reversed != next.data.reversed
        {
            return Ok(None);
        }
        let current_end = self.endpoint_parameter(false);
        let next_start = next.endpoint_parameter(true);
        if !current_end.shares_exact_evidence(next_start)
            && !matches!(
                current_end.cmp_by_refinement(next_start, policy)?,
                Classification::Decided(std::cmp::Ordering::Equal)
            )
        {
            return Ok(None);
        }
        let (start, start_point_image, end, end_point_image) = if self.data.reversed {
            (
                next.data.start.clone(),
                cloned_once_lock(&next.data.start_point_image),
                self.data.end.clone(),
                cloned_once_lock(&self.data.end_point_image),
            )
        } else {
            (
                self.data.start.clone(),
                cloned_once_lock(&self.data.start_point_image),
                next.data.end.clone(),
                cloned_once_lock(&next.data.end_point_image),
            )
        };
        let certified_tangent_endpoints = if self.data.reversed {
            (next.data.certified_tangent_endpoints & 1)
                | (self.data.certified_tangent_endpoints & 2)
        } else {
            (self.data.certified_tangent_endpoints & 1)
                | (next.data.certified_tangent_endpoints & 2)
        };
        Ok(Some(Self {
            data: Arc::new(BezierAlgebraicCuspSemicircleFragmentData2 {
                semicircle: self.data.semicircle.clone(),
                start,
                end,
                start_point_image,
                end_point_image,
                certified_tangent_endpoints,
                reversed: self.data.reversed,
                policy: self.data.policy,
            }),
        }))
    }

    pub(crate) fn shares_endpoint_evidence(
        &self,
        start_endpoint: bool,
        other: &Self,
        other_start_endpoint: bool,
    ) -> bool {
        let parameter = self.endpoint_parameter(start_endpoint);
        let other_parameter = other.endpoint_parameter(other_start_endpoint);
        if self.data.semicircle == other.data.semicircle
            && parameter.shares_exact_evidence(other_parameter)
        {
            return true;
        }
        self.endpoint_pair_overlap_source(start_endpoint, other, other_start_endpoint)
            .is_some()
    }

    /// Returns which endpoint is the source of an exact coincident-circle
    /// parameter map. `true` names `self`; `false` names `other`. This lets a
    /// smooth run solve in the ancestral parameter frame instead of rebuilding
    /// the same contact in a descendant chart and later asking STRICT to prove
    /// equality between independently formed radical expressions.
    pub(crate) fn endpoint_pair_overlap_source(
        &self,
        start_endpoint: bool,
        other: &Self,
        other_start_endpoint: bool,
    ) -> Option<bool> {
        let parameter = self.endpoint_parameter(start_endpoint);
        let other_parameter = other.endpoint_parameter(other_start_endpoint);
        let mapped_endpoint =
            |source_semicircle,
             source_parameter,
             target_semicircle,
             target_parameter: &BezierAlgebraicCuspSemicircleParameter2| {
                target_parameter
                    .pair_overlap_evidence()
                    .is_some_and(|overlap| {
                        overlap.maps_parameter_evidence(
                            source_semicircle,
                            source_parameter,
                            target_semicircle,
                            target_parameter,
                        )
                    })
            };
        if mapped_endpoint(
            &self.data.semicircle,
            parameter,
            &other.data.semicircle,
            other_parameter,
        ) {
            Some(true)
        } else if mapped_endpoint(
            &other.data.semicircle,
            other_parameter,
            &self.data.semicircle,
            parameter,
        ) {
            Some(false)
        } else {
            None
        }
    }

    /// Classifies exact incidence of any retained affine point on this finite
    /// selected-circle fragment. Circle and diameter-side predicates consume
    /// the point's native evidence; no Cartesian compositum is constructed.
    pub(in crate::bezier_offset) fn incident_point_chord_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
        self.validate_policy(policy)?;
        let residual = match self
            .data
            .semicircle
            .retained_point_incidence_sign(point, policy)?
        {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if residual != RealSign::Zero {
            return Ok(Classification::Decided(None));
        }
        Ok(self.endpoint_chord_side(point, policy)?.map(Some))
    }

    pub(in crate::bezier_offset) fn endpoint_chord_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        self.validate_policy(policy)?;
        let start = match self.endpoint_point_evidence(true, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.endpoint_point_evidence(false, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        selected_circle_endpoint_chord_side(&start, &end, point, true, policy)
    }

    /// Classifies represented and retained points through the same circle
    /// incidence and finite endpoint-chord ownership predicates.
    pub(crate) fn contains_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        // A chord contact authored by this exact selected semicircle already
        // retains its circle parameter and incidence proof in one shared map.
        // Classify that parameter against this finite range directly instead
        // of rebuilding the same point's recursive endpoint-chord side.
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
            let (map, _) = point.map_contact();
            map.validate_policy(policy)?;
            if self.data.semicircle == map.data.semicircle {
                return self.contains_parameter(
                    &BezierAlgebraicCuspSemicircleParameter2::Mapped(point.data.clone()),
                    true,
                    true,
                    policy,
                );
            }
        }
        Ok(self.incident_point_chord_side(point, policy)?.map(|side| {
            let Some(side) = side else {
                return false;
            };
            let clockwise = self.data.semicircle.is_clockwise() ^ self.data.reversed;
            if clockwise {
                side != crate::classify::LineSide::Right
            } else {
                side != crate::classify::LineSide::Left
            }
        }))
    }

    /// Proves strict finite-fragment interior for a point whose incidence on
    /// this supporting circle has already been certified by its constructor.
    ///
    /// A circle and its endpoint chord meet only at the two endpoints. Once
    /// incidence on this retained circle is exact, the strict interior chord
    /// side excludes both endpoint contacts without comparing independently
    /// represented angular parameters. Callers must retain the circle-contact
    /// certificate; arbitrary point evidence must use `contains_point`.
    pub(crate) fn certified_incident_point_evidence_is_strict_interior(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let side = policy.strict_predicate_pass(|| self.endpoint_chord_side(point, policy))?;
        Ok(side.map(|side| {
            let clockwise = self.data.semicircle.is_clockwise() ^ self.data.reversed;
            side == if clockwise {
                crate::classify::LineSide::Left
            } else {
                crate::classify::LineSide::Right
            }
        }))
    }

    /// Locates a point whose incidence on this supporting circle is already
    /// certified. Retained scalar order is tried first, followed by angular
    /// order in the circle's compact recursive frame. The angular predicate
    /// is only a fallback because it is monotone on the selected half, not on
    /// an incident extension point on the complementary half. The endpoint
    /// chord is the complete representation-independent fallback. At an
    /// endpoint, one decided inequality against the opposite endpoint is
    /// enough to identify the equal endpoint exactly.
    pub(crate) fn certified_incident_point_evidence_location(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleIncidentLocation2>> {
        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Start};
        for (at_start, location) in [(true, Start), (false, End)] {
            if parameter.shares_exact_evidence(self.endpoint_parameter(at_start)) {
                return Ok(Classification::Decided(location));
            }
            let endpoint = match self.endpoint_point_evidence(at_start, policy)? {
                Classification::Decided(Some(endpoint)) => endpoint,
                Classification::Decided(None) | Classification::Uncertain(_) => continue,
            };
            if point.shares_storage(&endpoint) {
                return Ok(Classification::Decided(location));
            }
        }
        // Cheap spatial separation is useful for ordinary interior contacts.
        // At an endpoint, independent coordinate boxes necessarily overlap;
        // give the retained parameter/field identities below authority before
        // requesting the complete correlated chord predicate.
        let endpoint_side =
            policy.bounded_exact_predicate_pass(|| self.endpoint_chord_side(point, policy))?;
        if let Classification::Decided(side) = endpoint_side
            && side != crate::classify::LineSide::On
        {
            let clockwise = self.data.semicircle.is_clockwise() ^ self.data.reversed;
            let interior_side = if clockwise {
                crate::classify::LineSide::Left
            } else {
                crate::classify::LineSide::Right
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-fragment-location",
                "endpoint-chord-decision",
            );
            return Ok(Classification::Decided(if side == interior_side {
                BezierAlgebraicCuspSemicircleIncidentLocation2::Interior
            } else {
                BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior
            }));
        }
        let point_order = |source_start: bool| -> CurveResult<Classification<std::cmp::Ordering>> {
            let endpoint_parameter = if source_start {
                &self.data.start
            } else {
                &self.data.end
            };
            let scalar_order = policy.bounded_exact_predicate_pass(|| {
                parameter.cmp_by_refinement(endpoint_parameter, policy)
            })?;
            if matches!(scalar_order, Classification::Decided(_)) {
                return Ok(scalar_order);
            }
            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            let endpoint = match self.endpoint_point_evidence(
                if source_start {
                    !self.data.reversed
                } else {
                    self.data.reversed
                },
                policy,
            )? {
                Classification::Decided(Some(endpoint)) => endpoint,
                Classification::Decided(None) | Classification::Uncertain(_) => {
                    return parameter.cmp_by_refinement(endpoint_parameter, policy);
                }
            };
            let projective = recursive_projective_incident_point_order(
                point,
                &endpoint,
                &self.data.semicircle,
                || {
                    Ok(matches!(
                        point,
                        CurvePoint2(CurvePointData2::AlgebraicCuspChord(contact))
                            if contact.contact_support_separates_point(&endpoint, policy)?
                                == Classification::Decided(true)
                    ))
                },
                policy,
            )?;
            if let Some(Classification::Decided(order)) = projective {
                return Ok(Classification::Decided(order));
            }
            parameter.cmp_by_refinement(endpoint_parameter, policy)
        };
        // An independent scalar or angular image may need the approximate
        // terminal even when the shared endpoint chord has an exact proof.
        // Exhaust all certified authorities before consuming that terminal.
        let start = policy.strict_predicate_pass(|| point_order(true))?;
        let end = policy.strict_predicate_pass(|| point_order(false))?;
        if let (Classification::Decided(start), Classification::Decided(end)) = (start, end) {
            return Ok(Classification::Decided(
                self.incident_location_from_orders(start, end),
            ));
        }
        let side = match endpoint_side {
            Classification::Decided(side) => side,
            Classification::Uncertain(_) => {
                match policy.strict_predicate_pass(|| self.endpoint_chord_side(point, policy))? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        if policy.permits_approximate_512()
                            && let (Classification::Decided(start), Classification::Decided(end)) =
                                (point_order(true)?, point_order(false)?)
                        {
                            return Ok(Classification::Decided(
                                self.incident_location_from_orders(start, end),
                            ));
                        }
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        if side != crate::classify::LineSide::On {
            let clockwise = self.data.semicircle.is_clockwise() ^ self.data.reversed;
            let interior_side = if clockwise {
                crate::classify::LineSide::Left
            } else {
                crate::classify::LineSide::Right
            };
            return Ok(Classification::Decided(if side == interior_side {
                BezierAlgebraicCuspSemicircleIncidentLocation2::Interior
            } else {
                BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior
            }));
        }

        // The chord-side `On` result proves this is one of the two distinct
        // endpoints. One exact comparison therefore identifies the endpoint:
        // equality selects that endpoint, while inequality selects the other.
        // Do not eagerly refine both unrelated parameter fields.
        let start = policy.strict_predicate_pass(|| {
            parameter.cmp_by_refinement(self.endpoint_parameter(true), policy)
        })?;
        match start {
            Classification::Decided(std::cmp::Ordering::Equal) => {
                Ok(Classification::Decided(Start))
            }
            Classification::Decided(_) => Ok(Classification::Decided(End)),
            Classification::Uncertain(start_reason) => {
                let end = policy.strict_predicate_pass(|| {
                    parameter.cmp_by_refinement(self.endpoint_parameter(false), policy)
                })?;
                Ok(match end {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        Classification::Decided(End)
                    }
                    Classification::Decided(_) => Classification::Decided(Start),
                    Classification::Uncertain(_) => Classification::Uncertain(start_reason),
                })
            }
        }
    }

    pub(in crate::bezier_offset) fn incident_location_from_orders(
        &self,
        start: std::cmp::Ordering,
        end: std::cmp::Ordering,
    ) -> BezierAlgebraicCuspSemicircleIncidentLocation2 {
        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Exterior, Interior, Start};
        match (start, end) {
            (std::cmp::Ordering::Equal, _) => {
                if self.data.reversed {
                    End
                } else {
                    Start
                }
            }
            (_, std::cmp::Ordering::Equal) => {
                if self.data.reversed {
                    Start
                } else {
                    End
                }
            }
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => Interior,
            _ => Exterior,
        }
    }

    /// Transports a certified sibling endpoint into this finite half-circle
    /// chart. `None` proves that the point is outside the consumed fragment.
    pub(crate) fn parameter_of_shared_circle_endpoint(
        &self,
        source: &Self,
        at_start: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleParameter2>>> {
        use std::cmp::Ordering::{Equal, Greater, Less};
        self.validate_policy(policy)?;
        source.validate_policy(policy)?;
        let parameter = source.endpoint_parameter(at_start);
        let mapped = match self
            .semicircle()
            .shared_frame_chart_relation(source.semicircle(), policy)
        {
            Classification::Decided(Some(false)) => parameter.clone(),
            Classification::Decided(Some(true)) => match (
                parameter.order_to_real(&Real::zero(), policy)?,
                parameter.order_to_real(&Real::one(), policy)?,
            ) {
                (Classification::Decided(Equal), _) => {
                    BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
                (_, Classification::Decided(Equal)) => {
                    BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero())
                }
                (Classification::Decided(Greater), Classification::Decided(Less)) => {
                    return Ok(Classification::Decided(None));
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            },
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .contains_parameter(&mapped, true, true, policy)?
            .map(|contains| contains.then_some(mapped)))
    }

    pub(crate) fn parameter_location_by_order(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleIncidentLocation2>> {
        self.validate_policy(policy)?;
        let start = match parameter.cmp_by_refinement(&self.data.start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match parameter.cmp_by_refinement(&self.data.end, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            self.incident_location_from_orders(start, end),
        ))
    }

    pub(crate) fn contains_parameter(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        include_start: bool,
        include_end: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .parameter_location_by_order(parameter, policy)?
            .map(|location| match location {
                BezierAlgebraicCuspSemicircleIncidentLocation2::Start => {
                    if self.data.reversed {
                        include_end
                    } else {
                        include_start
                    }
                }
                BezierAlgebraicCuspSemicircleIncidentLocation2::Interior => true,
                BezierAlgebraicCuspSemicircleIncidentLocation2::End => {
                    if self.data.reversed {
                        include_start
                    } else {
                        include_end
                    }
                }
                BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior => false,
            }))
    }

    /// Uses translated pair radials to classify a strict trim-domain contact
    /// before the general endpoint-chord predicate constructs point fields.
    /// `None` leaves every other parameter family to that complete fallback.
    pub(crate) fn translated_pair_parameter_is_strict_interior(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        self.validate_policy(policy)?;
        let Some(start) = parameter.translated_pair_or_exact_order(&self.data.start, policy) else {
            return Ok(None);
        };
        let Some(end) = parameter.translated_pair_or_exact_order(&self.data.end, policy) else {
            return Ok(None);
        };
        Ok(Some(match (start, end) {
            (Classification::Decided(start), Classification::Decided(end)) => {
                Classification::Decided(start.is_gt() && end.is_lt())
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Classification::Uncertain(reason)
            }
        }))
    }

    /// Applies the representation-independent half-open vertex rule used by
    /// winding queries.  In the base circle parameterization an endpoint is
    /// owned exactly when this fragment approaches its positive ray side:
    /// negative tangent/ray cross at the range start, positive at the end.
    /// Reversing the fragment swaps both endpoint role and tangent sign, so
    /// these two tests remain unchanged.
    pub(in crate::bezier_offset) fn contains_parameter_for_ray_winding(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        tangent_cross_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.validate_policy(policy)?;
        let start = match parameter.cmp_by_refinement(&self.data.start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match parameter.cmp_by_refinement(&self.data.end, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (start, end) {
            (std::cmp::Ordering::Equal, _) => tangent_cross_sign == RealSign::Negative,
            (_, std::cmp::Ordering::Equal) => tangent_cross_sign == RealSign::Positive,
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Less) => true,
            _ => false,
        }))
    }

    /// Returns this subfragment's spatially half-open winding contribution to
    /// one represented forward ray. Boundary incidence is closed at both
    /// fragment ends; the branch on the positive ray side owns a shared
    /// endpoint independently of carrier representation or traversal cuts.
    pub(crate) fn forward_ray_winding_delta(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.forward_ray_winding_delta_with_origin_contact(
            origin,
            direction_x,
            direction_y,
            None,
            false,
            policy,
        )
    }

    /// Returns this subfragment's winding contribution while omitting its one
    /// certified transverse contact at the ray origin.
    pub(crate) fn forward_ray_winding_delta_skipping_origin(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
        crossing_direction: BezierLineCrossingDirection,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.forward_ray_winding_delta_with_origin_contact(
            origin,
            direction_x,
            direction_y,
            Some((parameter, crossing_direction)),
            true,
            policy,
        )
    }

    /// Omits this finite arc's transverse contact at a represented side-ray
    /// origin while preserving any second forward circle contact.
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        match self.contains_point(&CurvePoint2::from(origin.clone()), policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        self.forward_ray_winding_delta_with_origin_contact(
            origin,
            direction_x,
            direction_y,
            None,
            true,
            policy,
        )
        .map(|delta| delta.map(Some))
    }

    /// Classifies one ray directly from retained circle, endpoint, and center
    /// predicates. This representation-independent path is authoritative for
    /// recursive frames and a compact fast path for rational frames; the
    /// latter retain their complete circle/rational intersection fallback.
    pub(in crate::bezier_offset) fn retained_forward_ray_winding_delta(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        skipped_origin: Option<(
            &BezierAlgebraicCuspSemicircleParameter2,
            BezierLineCrossingDirection,
        )>,
        skip_incident_origin: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let direction_squared = direction_x * direction_x + direction_y * direction_y;
        match real_sign(&direction_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => return Err(CurveError::ZeroLengthLine),
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "ray direction had a negative squared norm".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let start = match self.endpoint_point_evidence(true, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.endpoint_point_evidence(false, policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center = match self.data.semicircle.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let residual = match self
            .data
            .semicircle
            .retained_point_incidence_sign(&CurvePoint2::from(origin.clone()), policy)?
        {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let line_side = match selected_circle_endpoint_chord_side(
            &start,
            &end,
            &CurvePoint2::from(origin.clone()),
            false,
            policy,
        )? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let clockwise = self.data.semicircle.is_clockwise() ^ self.data.reversed;
        let point_on_fragment = residual == RealSign::Zero
            && if clockwise {
                line_side != crate::classify::LineSide::Right
            } else {
                line_side != crate::classify::LineSide::Left
            };
        let mut skipped_origin_delta = 0_i32;
        if point_on_fragment {
            if !skip_incident_origin {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            if let Some((source_parameter, _)) = skipped_origin {
                match self.contains_parameter(source_parameter, true, true, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let source_point = match source_parameter
                    .coincident_point_evidence(&self.data.semicircle, policy)?
                {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match source_point.same_point(&CurvePoint2::from(origin.clone()), policy) {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(CurveError::Topology(
                            "selected-circle origin crossing referenced a different point".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let radial_order = match algebraic_chord_points_linear_order(
                &CurvePoint2::from(origin.clone()),
                &center,
                direction_x,
                direction_y,
                policy,
            )? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut traversal_cross_sign = match radial_order {
                std::cmp::Ordering::Less => RealSign::Negative,
                std::cmp::Ordering::Equal => RealSign::Zero,
                std::cmp::Ordering::Greater => RealSign::Positive,
            };
            if !clockwise {
                traversal_cross_sign = match traversal_cross_sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                };
            }
            let certified_direction = match traversal_cross_sign {
                RealSign::Positive => BezierLineCrossingDirection::PositiveToNegative,
                RealSign::Negative => BezierLineCrossingDirection::NegativeToPositive,
                RealSign::Zero => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            };
            if let Some((_, crossing_direction)) = skipped_origin
                && certified_direction != crossing_direction
            {
                return Err(CurveError::Topology(
                    "selected-circle origin crossing disagreed with its boundary tangent".into(),
                ));
            }
            // The minor-arc predicate below uses the circle's outside-side
            // limit at a boundary query. It consequently counts an inward
            // origin crossing, but already excludes an outward one. Remove
            // only the former so a second forward contact on this same
            // finite arc remains in the winding sum.
            if radial_order == std::cmp::Ordering::Less {
                skipped_origin_delta = match certified_direction {
                    BezierLineCrossingDirection::NegativeToPositive => 1,
                    BezierLineCrossingDirection::PositiveToNegative => -1,
                };
            }
        } else if skip_incident_origin {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }

        let exact_origin = CurvePoint2::from(origin.clone());
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let order =
            |first: &CurvePoint2, second: &CurvePoint2, x_factor: &Real, y_factor: &Real| {
                algebraic_chord_points_linear_order(first, second, x_factor, y_factor, policy)
            };
        let start_y = match order(&start, &exact_origin, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_y = match order(&end, &exact_origin, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let inside_circle = residual == RealSign::Negative;
        let is_ccw = !clockwise;
        let point_is_left = if is_ccw {
            line_side == crate::classify::LineSide::Left
        } else {
            line_side != crate::classify::LineSide::Right
        };
        let decision = crate::contour::minor_arc_winding_decision(
            start_y != std::cmp::Ordering::Greater,
            end_y == std::cmp::Ordering::Greater,
            point_is_left,
            inside_circle,
            is_ccw,
        );
        let (lower, upper, delta) = match decision {
            crate::contour::MinorArcWindingDecision::Delta(delta) => {
                return Ok(Classification::Decided(delta - skipped_origin_delta));
            }
            crate::contour::MinorArcWindingDecision::PointBetweenStartAndEnd(delta) => {
                (&start, &end, delta)
            }
            crate::contour::MinorArcWindingDecision::PointBetweenEndAndStart(delta) => {
                (&end, &start, delta)
            }
        };
        let lower_x = match order(lower, &exact_origin, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if lower_x != std::cmp::Ordering::Less {
            return Ok(Classification::Decided(-skipped_origin_delta));
        }
        let upper_x = match order(upper, &exact_origin, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            if upper_x == std::cmp::Ordering::Greater {
                delta - skipped_origin_delta
            } else {
                -skipped_origin_delta
            },
        ))
    }

    pub(in crate::bezier_offset) fn forward_ray_winding_delta_with_origin_contact(
        &self,
        origin: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        skipped_origin: Option<(
            &BezierAlgebraicCuspSemicircleParameter2,
            BezierLineCrossingDirection,
        )>,
        skip_incident_origin: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.validate_policy(policy)?;
        let retained = self.retained_forward_ray_winding_delta(
            origin,
            direction_x,
            direction_y,
            skipped_origin,
            skip_incident_origin,
            policy,
        )?;
        if let decided @ Classification::Decided(_) = retained {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-ray-winding",
                "retained-point-predicates",
            );
            return Ok(decided);
        }
        if self.data.semicircle.data.frame.rational().is_none() {
            return Ok(retained);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-ray-winding",
            "rational-intersection-fallback",
        );
        let (_, intersections, parameter_map) = match self
            .data
            .semicircle
            .forward_ray_rational_contacts(origin, direction_x, direction_y, true, policy)?
        {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let contacts = match intersections {
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps }
                if overlaps.is_empty() =>
            {
                contacts
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { .. } => {
                return Err(CurveError::Topology(
                    "a rational-frame cusp ray produced selected-fiber contacts".into(),
                ));
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { .. } => {
                return Err(CurveError::Topology(
                    "a nonzero ray overlapped a selected circle".into(),
                ));
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let ray_start = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
        let mut winding = 0_i32;
        let mut origin_was_skipped = false;
        for contact in contacts {
            let parameter = algebraic_cusp_semicircle_endpoint_parameter(contact.location)
                .unwrap_or_else(|| {
                    parameter_map
                        .as_ref()
                        .expect("an interior ray contact retains its shared parameter map")
                        .contact_parameter(&contact)
                });
            let at_origin = match contact.other_parameter.same_value(&ray_start, policy)? {
                Classification::Decided(at_origin) => at_origin,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if at_origin {
                match self.contains_parameter(&parameter, true, true, policy)? {
                    Classification::Decided(true) => {
                        if !skip_incident_origin || origin_was_skipped {
                            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                        }
                        if let Some((source_parameter, _)) = skipped_origin {
                            let is_source =
                                match parameter.cmp_by_refinement(source_parameter, policy)? {
                                    Classification::Decided(order) => {
                                        order == std::cmp::Ordering::Equal
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                            if !is_source {
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                        }
                        let traversal_cross_sign = if self.data.reversed {
                            match contact.tangent_cross_sign {
                                RealSign::Positive => RealSign::Negative,
                                RealSign::Negative => RealSign::Positive,
                                RealSign::Zero => RealSign::Zero,
                            }
                        } else {
                            contact.tangent_cross_sign
                        };
                        let certified_direction = match traversal_cross_sign {
                            RealSign::Positive => BezierLineCrossingDirection::PositiveToNegative,
                            RealSign::Negative => BezierLineCrossingDirection::NegativeToPositive,
                            RealSign::Zero => {
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                        };
                        if let Some((_, crossing_direction)) = skipped_origin
                            && certified_direction != crossing_direction
                        {
                            return Err(CurveError::Topology(
                                "algebraic cusp origin crossing disagreed with its boundary tangent"
                                    .into(),
                            ));
                        }
                        origin_was_skipped = true;
                        continue;
                    }
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            match self.contains_parameter_for_ray_winding(
                &parameter,
                contact.tangent_cross_sign,
                policy,
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let delta = match contact.tangent_cross_sign {
                RealSign::Negative => 1,
                RealSign::Positive => -1,
                RealSign::Zero => 0,
            };
            winding += if self.data.reversed { -delta } else { delta };
        }
        if skip_incident_origin && !origin_was_skipped {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Ok(Classification::Decided(winding))
    }
}
