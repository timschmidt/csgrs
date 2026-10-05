//! Parallel/parallel and self intersections.

use super::*;

impl BezierParallel2 {
    #[cfg(test)]
    /// Constructs complete polynomial parameter projections against another parallel.
    ///
    /// Let `Delta=Q-P`, homogeneous tangent numerators be `Hp,Hq`, squared
    /// speeds be `Sp,Sq`, tangent cross product be `C`, tangent dot product be
    /// `T`, source-weight product be `W`, and signed distances be `d,e`.
    /// Every contact satisfies
    ///
    /// `Sp(Delta·Hq)^2-d^2 C^2 W^2 = 0`,
    ///
    /// `Sq(Delta·Hp)^2-e^2 C^2 W^2 = 0`, and
    ///
    /// `(Delta²-(d²+e²)W²)^2 Sp Sq-4d²e²T²W⁴ = 0`.
    ///
    /// The two lower-degree projection equations are preferred. If their
    /// resultant has a shared component, the first and norm equations provide
    /// an independent fallback basis. Projection remains unsigned candidate
    /// evidence; [`Self::parallel_intersections`] replays all three radical
    /// equations and their selected normal branches.
    pub(crate) fn parallel_intersection_candidates(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveIntersectionCandidates2>> {
        if structural_parallel_overlap(self, other, policy)?.is_some() {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::DegenerateResultant,
            ));
        }
        match other.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(other)) => {
                return self.intersection_candidates(&other, policy);
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(first)) => {
                return Ok(other
                    .intersection_candidates(&first, policy)?
                    .map(CurveIntersectionCandidates2::swapped));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Some(system) = (match parallel_pair_equation_system(self, other, true, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::NoIntersection,
            ));
        };
        if bivariate_pair_may_have_component(&system.first_equation, &system.second_equation)
            && matches!(
                certified_parallel_source_overlap(self, other, policy)?,
                Classification::Decided(CertifiedParallelSourceOverlap2 {
                    kind: CertifiedParallelSourceOverlapKind2::Selected(_)
                        | CertifiedParallelSourceOverlapKind2::Excluded,
                    ..
                })
            )
        {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::DegenerateResultant,
            ));
        }
        project_parallel_intersection_system(
            &system.first_equation,
            &system.second_equation,
            [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
            policy,
        )
    }

    /// Certifies a regular closed parallel whose tangent makes exactly one turn.
    ///
    /// A strictly signed source curvature makes the tangent angle strictly
    /// monotone. Counting its positive-x ray crossings on [0,1) proves one
    /// complete turn, rather than assuming that a closed locally convex walk
    /// is simple. A cusp-free parallel multiplies that tangent by one nonzero
    /// continuous scalar, preserving the turn count and closure. Each linear
    /// functional then has one maximum and one minimum: the image is a simple
    /// strictly convex boundary, with only the (0,1) closing contact.
    ///
    /// This optional certificate uses the existing univariate authorities.
    /// Flat curvature, cusps, open seams, multiple turns or unresolved signs
    /// leave complete bivariate discovery responsible for the result.
    pub(in crate::bezier_offset) fn certifies_simple_closed_parallel(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        policy.bounded_exact_predicate_pass(|| {
            let source = self.source().to_rational_bezier()?;
            if source.start() != source.end()
                && real_sign(&source.start().distance_squared(source.end()), policy)
                    != Some(RealSign::Zero)
            {
                return Ok(false);
            }
            let unit = CurveParameterRange2::unit();
            let differential = self.differential()?;
            let curvature = polynomial_subtract(
                &polynomial_multiply(&differential.tangent_x, &differential.tangent_derivative_y),
                &polynomial_multiply(&differential.tangent_y, &differential.tangent_derivative_x),
            );
            if polynomial_is_nonzero_on_parameter_range(&curvature, &unit, policy)?
                != Classification::Decided(true)
            {
                return Ok(false);
            }
            let start = BezierParameter2::Exact(Real::zero());
            let end = BezierParameter2::Exact(Real::one());
            if self.source_tangent_pair_cross_and_dot_signs(&start, self, &end, policy)?
                != Classification::Decided((RealSign::Zero, RealSign::Positive))
            {
                return Ok(false);
            }
            let Classification::Decided(Some(ray)) =
                polynomial_from_coefficients(differential.tangent_y.clone(), policy)?
            else {
                return Ok(false);
            };
            let Classification::Decided(crossings) = ray.isolate_unit_interval_roots(policy)?
            else {
                return Ok(false);
            };
            let mut turns = 0;
            for crossing in crossings {
                match crossing.cmp_by_refinement(&end, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => continue,
                    Classification::Decided(std::cmp::Ordering::Less) => {}
                    Classification::Decided(std::cmp::Ordering::Greater)
                    | Classification::Uncertain(_) => return Ok(false),
                }
                match signed_coefficients_at_parameter(&differential.tangent_x, &crossing, policy)?
                {
                    Classification::Decided(RealSign::Positive) => turns += 1,
                    Classification::Decided(RealSign::Negative) => {}
                    Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                        return Ok(false);
                    }
                }
                if turns > 1 {
                    return Ok(false);
                }
            }
            if turns != 1 {
                return Ok(false);
            }
            Ok(matches!(
                self.singularity_analysis(&unit, policy)?,
                Classification::Decided(analysis)
                    if analysis.source_is_regular() && analysis.parallel_is_cusp_free()
            ))
        })
    }

    /// Returns every unordered off-diagonal self-contact of this analytic parallel.
    ///
    /// Zero-distance and exactly materializable Pythagorean-hodograph carriers
    /// delegate to the rational self-contact authority, including non-injective
    /// carriers. A regular closed single-turn certificate retains just the
    /// closing contact. Other carriers use one bivariate projection and replay graph.
    /// Every isolated pair must satisfy all three
    /// squared equations and the corresponding unsquared signs. At parallel
    /// tangents, norm replay selects `|d-sign(Hp·Hq)e|` and a final normal-side
    /// predicate selects its direction. No square root is numerically evaluated.
    /// The structural parameter diagonal is divided from both equations before
    /// projection, so ordinary identity is not mistaken for overlap evidence.
    /// Any further shared component remains explicit incomplete replay.
    pub(crate) fn unit_self_intersections(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        match self.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(curve)) => {
                let mut fallback_point_evidence = |parameter: &BezierParameter2| {
                    Ok(Some(CurvePoint2::from(
                        crate::BezierAnalyticParallelPoint2::new(
                            self.clone(),
                            parameter.clone(),
                            policy,
                        ),
                    )))
                };
                let result = match curve.self_intersection_contacts_with_point_evidence_classified(
                    policy,
                    &mut fallback_point_evidence,
                )? {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(Classification::Decided(
                    parallel_pair_set_from_rational_self_contacts(self, result, policy)?,
                ));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        if self.certifies_simple_closed_parallel(policy)? {
            return Ok(Classification::Decided(
                BezierParallelPairIntersectionSet2::complete(
                    Arc::from([BezierParallelPairIntersectionContact2 {
                        first_parameter: Real::zero().into(),
                        second_parameter: Real::one().into(),
                        certified_transverse: false,
                        tangent_cross_sign: Some(RealSign::Zero),
                        tangent_dot_sign: Some(RealSign::Positive),
                    }]),
                    Arc::from([]),
                ),
            ));
        }
        let Some(system) = (match parallel_pair_equation_system(self, self, true, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                BezierParallelPairIntersectionSet2::complete(Arc::from([]), Arc::from([])),
            ));
        };
        let source_diagonal_excluded =
            Classification::Decided(CertifiedParallelSourceOverlap2::without_contacts(
                CertifiedParallelSourceOverlapKind2::Excluded,
            ));
        let unit = CurveParameterRange2::unit();
        let increasing =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![-Real::one()]]);
        let Some(BezierParallelPairDomainProjection2::Enumerated {
            projection,
            retained_contacts,
            components,
        }) = project_parallel_pair_without_components_in_domain(
            &system,
            self,
            self,
            &source_diagonal_excluded,
            [CurveParameterDomain2::new(&unit, None); 2],
            ParameterComponentQuery2::RetainFinite,
            Some(&increasing),
            policy,
        )?
        else {
            return Ok(Classification::Decided(
                BezierParallelPairIntersectionSet2::incomplete(
                    Arc::from([]),
                    Arc::from([]),
                    CurveIntersectionCandidates2::DegenerateResultant,
                ),
            ));
        };
        if !components.is_empty() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let result = self.replay_parallel_pair_projection(
            self,
            &system,
            projection,
            BezierParallelPairParameterSelection2::Increasing,
            policy,
        )?;
        extend_parallel_pair_contacts(result, retained_contacts, policy)
    }

    /// Returns the selected-branch intersections with another analytic parallel.
    ///
    /// Rationally materializable carriers delegate directly. General carriers
    /// share the exact projection and selected-branch replay used by
    /// off-diagonal self-contact analysis; certified source correspondences
    /// are saturated before their residual isolated contacts are replayed.
    pub(crate) fn parallel_intersections(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        match self.rational_parallel_pair_intersections(other, None, policy)? {
            Classification::Decided(Some(intersections)) => {
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        self.parallel_intersections_without_regular_frame(other, policy)
    }

    pub(in crate::bezier_offset) fn parallel_intersections_without_regular_frame(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        let Some(system) = (match parallel_pair_equation_system(self, other, true, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                BezierParallelPairIntersectionSet2::complete(Arc::from([]), Arc::from([])),
            ));
        };
        let projection =
            match project_unit_parallel_pair_intersection_system(&system, self, other, policy)? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let BezierParallelPairDomainProjection2::Enumerated {
            projection,
            retained_contacts,
            components,
        } = projection
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !components.is_empty() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let result = self.replay_parallel_pair_projection(
            other,
            &system,
            projection,
            BezierParallelPairParameterSelection2::All,
            policy,
        )?;
        extend_parallel_pair_contacts(result, retained_contacts, policy)
    }

    /// Intersects two retained regular source branches, including branches
    /// whose authored source tangent vanishes at an endpoint.
    ///
    /// A source cusp contributes a common hodograph factor `g`.  On either
    /// open regular branch, `H = g U` has the same selected unit normal as the
    /// source-oriented field `sign(g) U`.  The ordinary full-source pair
    /// equations cannot admit the cusp because every squared equation then
    /// carries a spurious zero-speed component.  This branch form removes the
    /// exact common factor, keeps its one-sided sign, and runs the same
    /// resultant/saturation/replay authority with a nonvanishing frame.  The
    /// returned contacts keep the original source parameters and are restricted
    /// to the retained ranges. Native overlap evidence may enclose those ranges:
    /// support and region callers clip its retained correspondence before
    /// publishing an overlap. Selected endpoints need not be projected back to
    /// the native overlap's narrower parameter representation.
    pub(crate) fn parallel_intersections_on_regular_ranges(
        &self,
        other: &Self,
        first_range: &CurveParameterRange2,
        second_range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        Ok(self
            .parallel_intersections_on_regular_domains(
                other,
                [first_range, second_range].map(|range| CurveParameterDomain2::new(range, None)),
                ParameterComponentQuery2::RetainFinite,
                policy,
            )?
            .map(|result| {
                debug_assert!(result.components.is_empty());
                result.intersections
            }))
    }

    /// Keeps selected components and isolated contacts on the same regular
    /// source branches. A component query needs correlated domain clipping,
    /// including its endpoint frames, before a family can be published.
    pub(crate) fn parallel_intersections_on_regular_domains(
        &self,
        other: &Self,
        domains: [CurveParameterDomain2<'_>; 2],
        query: ParameterComponentQuery2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairDomainIntersectionSet2>> {
        let ranges = domains.map(|domain| domain.finite);
        let closed_finite = domains
            .iter()
            .all(|domain| domain.extension.is_none() && domain.inclusion == [true; 2]);
        let mut extension_frames = None;
        if domains.iter().any(|domain| domain.extension.is_some()) {
            let mut frames = [None, None];
            for (axis, parallel) in [self, other].into_iter().enumerate() {
                frames[axis] =
                    match parallel.source_tangent_field_in_regular_domain(domains[axis], policy)? {
                        Classification::Decided(frame) => frame,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
            }
            extension_frames = Some(frames);
        }
        // A rational image keeps its exact source chart even when a retained
        // range supplies a branch-oriented tangent frame for the other operand.
        let rational = if closed_finite && matches!(query, ParameterComponentQuery2::RetainFinite) {
            self.rational_parallel_pair_intersections(other, Some(ranges), policy)?
                .map(|result| result.map(BezierParallelPairDomainIntersectionSet2::enumerated))
        } else {
            self.rational_parallel_pair_intersections_in_regular_domains(
                other, domains, query, policy,
            )?
        };
        match rational {
            Classification::Decided(Some(result)) => {
                return Ok(self
                    .replay_regular_pair_endpoint_tangents(
                        other,
                        ranges,
                        result.intersections,
                        policy,
                    )?
                    .map(|intersections| {
                        BezierParallelPairDomainIntersectionSet2::with_components(
                            intersections,
                            result.components,
                        )
                    }));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let [first_range, second_range] = ranges;
        let first_frame = if let Some(frames) = &extension_frames {
            frames[0].clone()
        } else {
            match self.source_oriented_regularized_tangent_field(first_range, policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let second_frame = if let Some(frames) = &extension_frames {
            frames[1].clone()
        } else {
            match other.source_oriented_regularized_tangent_field(second_range, policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        // Unit-span bounds and root enumeration are valid only for the
        // queried unit spans. Retained and extended ranges own their own
        // projection, even when neither source needs a cancelled frame.
        let unit = CurveParameterRange2::unit();
        let unit_domain = first_range == &unit
            && second_range == &unit
            && domains.iter().all(|domain| domain.extension.is_none());
        if unit_domain
            && closed_finite
            && first_frame.is_none()
            && second_frame.is_none()
            && matches!(query, ParameterComponentQuery2::RetainFinite)
        {
            return Ok(self
                .parallel_intersections_without_regular_frame(other, policy)?
                .map(BezierParallelPairDomainIntersectionSet2::enumerated));
        }
        let Some(system) = (match parallel_pair_equation_system_with_tangent_fields(
            self,
            other,
            first_frame.as_deref(),
            second_frame.as_deref(),
            unit_domain,
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                BezierParallelPairDomainIntersectionSet2::from_components(Vec::new()),
            ));
        };
        // A retained subdomain of the authored unit spans can reuse the
        // structural source correspondence, while residual projection still
        // owns both requested domains. No cancelled source frame is involved;
        // opposite one-sided source normals keep the general domain selector.
        if !unit_domain
            && closed_finite
            && first_frame.is_none()
            && second_frame.is_none()
            && matches!(query, ParameterComponentQuery2::RetainFinite)
            && ranges.iter().all(|range| {
                matches!(
                    CurveParameterDomain2::new(&unit, None).contains_finite_range(range, policy),
                    Ok(Classification::Decided(true))
                )
            })
            && let Some(overlap) = structural_parallel_overlap(self, other, policy)?
        {
            let source_overlap =
                Classification::Decided(CertifiedParallelSourceOverlap2::without_contacts(
                    CertifiedParallelSourceOverlapKind2::Selected(overlap),
                ));
            if let Some(BezierParallelPairDomainProjection2::Enumerated {
                projection,
                retained_contacts,
                components,
            }) = project_parallel_pair_without_components_in_domain(
                &system,
                self,
                other,
                &source_overlap,
                domains,
                query,
                None,
                policy,
            )? {
                let result = self.replay_parallel_pair_projection_with_ranges(
                    other,
                    &system,
                    projection,
                    BezierParallelPairParameterSelection2::All,
                    Some(ranges),
                    policy,
                )?;
                let result = extend_parallel_pair_contacts(result, retained_contacts, policy)?;
                return Ok(result.map(|intersections| {
                    BezierParallelPairDomainIntersectionSet2::with_components(
                        intersections,
                        components,
                    )
                }));
            }
        }
        if !unit_domain
            || !closed_finite
            || !matches!(query, ParameterComponentQuery2::RetainFinite)
        {
            return self.parallel_intersections_from_system_in_domain(
                other,
                system,
                domains,
                query,
                Some(ranges),
                policy,
            );
        }
        let projection =
            match project_unit_parallel_pair_intersection_system(&system, self, other, policy)? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let BezierParallelPairDomainProjection2::Enumerated {
            projection,
            retained_contacts,
            components,
        } = projection
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let result = self.replay_parallel_pair_projection_with_ranges(
            other,
            &system,
            projection,
            BezierParallelPairParameterSelection2::All,
            Some(ranges),
            policy,
        )?;
        Ok(
            extend_parallel_pair_contacts(result, retained_contacts, policy)?.map(
                |intersections| {
                    BezierParallelPairDomainIntersectionSet2::with_components(
                        intersections,
                        components,
                    )
                },
            ),
        )
    }

    /// Rational materialization preserves point and parameter identity, but
    /// its raw derivative vanishes at an endpoint cusp. Retain the original
    /// source branches' limiting tangent relation there. Ordinary contacts
    /// reuse their existing signs without constructing tangent supports.
    pub(in crate::bezier_offset) fn replay_regular_pair_endpoint_tangents(
        &self,
        other: &Self,
        ranges: [&CurveParameterRange2; 2],
        mut intersections: BezierParallelPairIntersectionSet2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        if !intersections.is_complete() {
            return Ok(Classification::Decided(intersections));
        }
        let strict = policy.strict_counterpart();
        'contacts: for index in 0..intersections.contacts.len() {
            let contact = &intersections.contacts[index];
            // Native incidence kernels may leave both signs absent when
            // a derivative vanishes, instead of explicitly reporting zero.
            // Any nonzero sign already proves both tangent directions exist.
            if [contact.tangent_cross_sign, contact.tangent_dot_sign]
                .into_iter()
                .any(|sign| matches!(sign, Some(RealSign::Positive | RealSign::Negative)))
            {
                continue;
            }
            let parameters = [contact.first_parameter(), contact.second_parameter()];
            let mut endpoint = false;
            for (parameter, range) in parameters.into_iter().zip(ranges) {
                for bound in [range.start(), range.end()] {
                    match parameter.same_value(bound, &strict)? {
                        Classification::Decided(true) => {
                            endpoint = true;
                            break;
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            if !endpoint {
                continue;
            }
            let mut tangents = [None, None];
            for (axis, parallel) in [self, other].into_iter().enumerate() {
                let direction = match parallel.parallel_derivative_scale_sign_on_regular_range(
                    parameters[axis],
                    ranges[axis],
                    &strict,
                )? {
                    Classification::Decided(RealSign::Zero) => {
                        // An interior cusp or collapsed parallel has no unique
                        // tangent selected by these range endpoints.
                        continue 'contacts;
                    }
                    Classification::Decided(direction) => direction,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                tangents[axis] = Some(
                    match parallel.regular_source_point_and_tangent_support(
                        parallel,
                        parameters[axis],
                        ranges[axis],
                        direction,
                        &strict,
                    )? {
                        Classification::Decided((_, tangent)) => tangent,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                );
            }
            let [first, second] =
                tangents.map(|tangent| tangent.expect("both original source frames were retained"));
            let cross = match first.tangent_cross_sign(&second, &strict)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let dot = match first.tangent_dot_sign(&second, &strict)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let contact = &mut Arc::make_mut(&mut intersections.contacts)[index];
            contact.tangent_cross_sign = Some(cross);
            contact.tangent_dot_sign = Some(dot);
            contact.certified_transverse = cross != RealSign::Zero;
        }
        Ok(Classification::Decided(intersections))
    }

    /// Returns selected-branch intersections on both retained finite ranges
    /// plus an independently optional regular extension for each source.
    ///
    /// Finite cuts and TrimOrExtend share this projective corner domain. The ordinary
    /// parallel-pair equations and exact replay remain authoritative; only the
    /// two univariate resultant projections use their requested domains. Each ray
    /// stops before its first source pole or source-speed zero. Exact source
    /// and radical components reuse the finite topology through the requested
    /// compact charts, with correlated clipping against exact regularity barriers.
    pub(crate) fn parallel_intersections_in_domain(
        &self,
        other: &Self,
        domains: [CurveParameterDomain2<'_>; 2],
        query: ParameterComponentQuery2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairDomainIntersectionSet2>> {
        let query = query.without_identity_constraints(policy);
        // The complete unit spans can reuse their rational and cached finite
        // kernels. A restricted or exterior interval needs domain projection
        // and component clipping before a correspondence can be admitted.
        if query.normal_constraints().is_none()
            && domains.iter().all(|domain| {
                domain.extension.is_none()
                    && domain.inclusion == [true; 2]
                    && domain.finite == &CurveParameterRange2::unit()
            })
        {
            let intersections = match self.parallel_intersections(other, policy)? {
                Classification::Decided(intersections) => Some(intersections),
                // A stationary endpoint may have a unique one-sided frame.
                // Component queries must reach the same regular-branch solver
                // as finite intersections before declaring it undecidable.
                Classification::Uncertain(UncertaintyReason::Boundary) => None,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if let Some(intersections) = intersections {
                if matches!(query, ParameterComponentQuery2::RetainFinite)
                    || !intersections.is_complete()
                {
                    return Ok(Classification::Decided(
                        BezierParallelPairDomainIntersectionSet2::enumerated(intersections),
                    ));
                }
                let components = match retain_finite_parallel_components(
                    self,
                    other,
                    &intersections,
                    domains,
                    policy,
                )? {
                    Classification::Decided(components) => components,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if matches!(query, ParameterComponentQuery2::FirstComponent(_))
                    && !components.is_empty()
                {
                    return Ok(Classification::Decided(
                        BezierParallelPairDomainIntersectionSet2::from_components(components),
                    ));
                }
                return Ok(Classification::Decided(
                    BezierParallelPairDomainIntersectionSet2::with_components(
                        intersections,
                        components,
                    ),
                ));
            }
        }
        if domains.iter().all(|domain| domain.extension.is_none()) {
            let retain_finite = matches!(query, ParameterComponentQuery2::RetainFinite);
            // Pointwise source normals are undefined at a source cusp. A zero
            // only at an endpoint still has one regular branch frame. An open
            // interior zero does not, and must not be published as a contact.
            let strict = policy.strict_counterpart();
            let mut rational_images = [None, None];
            let mut endpoint_cusp = [false, false];
            let mut regular_sources = true;
            for (index, (parallel, domain)) in [self, other].into_iter().zip(domains).enumerate() {
                match real_sign(parallel.distance(), &strict) {
                    Some(RealSign::Zero) => {
                        rational_images[index] = Some(parallel.clone());
                        continue;
                    }
                    Some(_) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
                let speed = parallel_speed_squared_polynomial(parallel.differential()?);
                match polynomial_is_nonzero_on_parameter_range(&speed, domain.finite, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        match polynomial_roots_touch_only_range_endpoints(
                            &speed,
                            domain.finite,
                            policy,
                        )? {
                            Classification::Decided(true) => endpoint_cusp[index] = true,
                            Classification::Decided(false) => {
                                if retain_finite {
                                    // Native overlap evidence is attached to
                                    // regular retained carriers; it cannot
                                    // supply one frame across this cusp.
                                    return Ok(Classification::Uncertain(
                                        UncertaintyReason::Boundary,
                                    ));
                                }
                                // A general domain can contain several normal
                                // sheets. Its unsquared selector excludes the
                                // undefined speed-zero point on every sheet.
                                regular_sources = false;
                                continue;
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                if endpoint_cusp[index] || !retain_finite {
                    continue;
                }
                // A PH image must own the requested normal sheet. Native
                // materialization cannot be reused across an exterior speed zero.
                if let Ok(Classification::Decided(Some(component))) = policy
                    .bounded_exact_predicate_pass(|| {
                        parallel.exact_rational_parallel_component_on_regular_range(
                            domain.finite,
                            policy,
                        )
                    })
                {
                    rational_images[index] = Some(component.curve().parallel_left(Real::zero())?);
                }
            }
            if regular_sources && (endpoint_cusp[0] || endpoint_cusp[1]) {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "parallel-pair-domain",
                    "one-sided-source-cusp",
                );
                return self
                    .parallel_intersections_on_regular_domains(other, domains, query, policy);
            }
            if retain_finite && regular_sources {
                match rational_images.each_ref() {
                    [Some(first), Some(second)] => {
                        if domains.iter().any(|domain| domain.inclusion != [true; 2]) {
                            return first.zero_distance_pair_intersections_in_domain(
                                second, domains, false, false, query, None, policy,
                            );
                        }
                        return Ok(rational_pair_intersections_on_ranges(
                            &first.source().to_rational_bezier()?,
                            &second.source().to_rational_bezier()?,
                            domains.map(|domain| domain.finite),
                            false,
                            policy,
                        )?
                        .map(BezierParallelPairDomainIntersectionSet2::enumerated));
                    }
                    [None, Some(second)] => {
                        return self.zero_distance_pair_intersections_in_domain(
                            second, domains, false, false, query, None, policy,
                        );
                    }
                    [Some(first), None] => {
                        return other.zero_distance_pair_intersections_in_domain(
                            first,
                            [domains[1], domains[0]],
                            true,
                            false,
                            query,
                            None,
                            policy,
                        );
                    }
                    [None, None] => {}
                }
            }
        }
        // A zero displacement is its rational source on every finite chart,
        // including stationary source parameters. It needs no normal field.
        // Preserve the operand roles while reusing the smaller rational pair
        // equations and the same selected domain/component authority.
        for swapped in [false, true] {
            let (parallel, zero, domains) = if swapped {
                (other, self, [domains[1], domains[0]])
            } else {
                (self, other, domains)
            };
            if real_sign(zero.distance(), &policy.strict_counterpart()) == Some(RealSign::Zero) {
                return parallel.zero_distance_pair_intersections_in_domain(
                    zero, domains, swapped, false, query, None, policy,
                );
            }
        }
        if !matches!(query, ParameterComponentQuery2::RetainFinite) {
            // Component queries use the same domain-certified PH images as
            // self-intersections. Keep the original parameter charts and
            // normal constraints while avoiding unnecessary radical equations.
            // Each image must certify one speed sheet over its entire domain,
            // including any requested incident extension.
            let strict = policy.strict_counterpart();
            if let Classification::Decided(Some([first])) =
                self.rational_parallel_components_in_domains([domains[0]], &strict)?
                && let Classification::Decided(Some([second])) =
                    other.rational_parallel_components_in_domains([domains[1]], &strict)?
            {
                let first = first.parallel_left(Real::zero())?;
                let second = second.parallel_left(Real::zero())?;
                return first.zero_distance_pair_intersections_in_domain(
                    &second, domains, false, false, query, None, policy,
                );
            }
        }
        let Some(system) = (match parallel_pair_equation_system(self, other, false, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                BezierParallelPairDomainIntersectionSet2::enumerated(
                    BezierParallelPairIntersectionSet2::complete(Arc::from([]), Arc::from([])),
                ),
            ));
        };
        self.parallel_intersections_from_system_in_domain(
            other, system, domains, query, None, policy,
        )
    }

    /// Shares finite/incident projection and component replay with retained
    /// regular frames. Equations keep their original source parameters;
    /// every residual projection and selected family uses the same domains.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn parallel_intersections_from_system_in_domain(
        &self,
        other: &Self,
        system: BezierParallelPairEquationSystem2,
        domains: [CurveParameterDomain2<'_>; 2],
        query: ParameterComponentQuery2<'_>,
        regular_ranges: Option<[&CurveParameterRange2; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairDomainIntersectionSet2>> {
        let candidates = match project_parallel_intersection_system(
            &system.first_equation,
            &system.second_equation,
            domains,
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut projection = BezierParallelPairProjection2 {
            candidates,
            basis: BezierParallelPairProjectionBasis2::ProjectionEquations,
            overlap: None,
            component_overlap_evidence: Arc::from([]),
            component_pairs: Arc::from([]),
            selected_component_pair_count: 0,
            residual_equations: None,
            radical_component_projection: None,
        };
        let mut source_isolated_projection = None;
        let mut retained_contacts = Vec::new();
        let mut domain_components = Vec::new();
        let axis_component = extract_bivariate_polynomial_system_axis_factors(
            &system.first_equation,
            &system.second_equation,
        )
        .status
            == BivariatePolynomialAxisFactorStatus::Reduced;
        if axis_component
            || matches!(
                projection.candidates,
                CurveIntersectionCandidates2::DegenerateResultant
            )
        {
            let config = CurveIntersectionResultantConfig {
                min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            };
            let components = match parallel_source_parameter_components(self, other, config)? {
                Classification::Decided(components) => components,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let source_constraint = match parameter_component_union_support(&components) {
                Some(support) => match parameter_domain_constraint(
                    support,
                    &system.norm_equation,
                    domains,
                    policy,
                    config,
                )? {
                    Classification::Decided(constraint) => Some(constraint),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                None => None,
            };
            if let Some(source_constraint) = source_constraint {
                let mut selected_pairs = Vec::new();
                let mut component_overlaps = Vec::new();
                if let Some(support) = source_constraint.component_support {
                    let selection = match select_parameter_component_in_domain(
                        &support,
                        &ParameterComponentSelector2::ParallelPair {
                            normal_constraints: None,
                            system: &system,
                            parameter_filter: None,
                        },
                        domains,
                        query,
                        policy,
                        config,
                    )? {
                        Classification::Decided(selection) => selection,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    if selection.has_components()
                        && matches!(query, ParameterComponentQuery2::FirstComponent(_))
                    {
                        return Ok(Classification::Decided(
                            BezierParallelPairDomainIntersectionSet2::from_components(
                                selection.components,
                            ),
                        ));
                    }
                    domain_components.extend(selection.components);
                    selected_pairs = selection.selected_pairs;
                    component_overlaps = selection.component_overlaps;
                    retained_contacts.extend(selection.retained_contacts);
                }
                source_isolated_projection = retain_parameter_component_pairs(
                    source_constraint.isolated_projection,
                    selected_pairs,
                    component_overlaps,
                );
            }
            let excluded =
                Classification::Decided(CertifiedParallelSourceOverlap2::without_contacts(
                    CertifiedParallelSourceOverlapKind2::Excluded,
                ));
            if let Some(residual_projection) = project_parallel_pair_without_components_in_domain(
                &system, self, other, &excluded, domains, query, None, policy,
            )? {
                match residual_projection {
                    BezierParallelPairDomainProjection2::Enumerated {
                        projection: residual_projection,
                        retained_contacts: residual_contacts,
                        components: residual_components,
                    } => {
                        projection = residual_projection;
                        retained_contacts.extend(residual_contacts);
                        domain_components.extend(residual_components);
                    }
                    BezierParallelPairDomainProjection2::Components(components) => {
                        return Ok(Classification::Decided(
                            BezierParallelPairDomainIntersectionSet2::from_components(components),
                        ));
                    }
                }
            }
            if let Some(source_projection) = source_isolated_projection.take() {
                prepend_parallel_pair_projection(&mut projection, source_projection);
            }
        }
        let result = self.replay_parallel_pair_projection_with_ranges(
            other,
            &system,
            projection,
            BezierParallelPairParameterSelection2::All,
            regular_ranges,
            policy,
        )?;
        let result = extend_parallel_pair_contacts(result, retained_contacts, policy)?;
        Ok(result.map(|result| {
            BezierParallelPairDomainIntersectionSet2::with_components(result, domain_components)
        }))
    }

    /// The second operand has certified zero displacement. Its exact source
    /// supplies the rational equations and a cheap retained point witness;
    /// finite cuts and incident rays still share the common domain selector.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn zero_distance_pair_intersections_in_domain(
        &self,
        zero: &Self,
        domains: [CurveParameterDomain2<'_>; 2],
        swapped: bool,
        off_diagonal: bool,
        query: ParameterComponentQuery2<'_>,
        regular_parallel_range: Option<&CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairDomainIntersectionSet2>> {
        // This kernel puts the rational operand second. The supplied
        // constraints still refer to the caller's first and second axes.
        let reordered_constraints = match query.normal_constraints() {
            Some(constraints) if swapped => Some([
                constraints[1].parallel.derivative_scale_constraint(
                    CurveResultantParameter::First,
                    constraints[1].expected,
                    constraints[1].side,
                ),
                constraints[0].parallel.derivative_scale_constraint(
                    CurveResultantParameter::Second,
                    constraints[0].expected,
                    constraints[0].side,
                ),
            ]),
            _ => None,
        };
        let query = match (reordered_constraints.as_ref(), query) {
            (Some(constraints), ParameterComponentQuery2::FirstComponent(_)) => {
                ParameterComponentQuery2::FirstComponent(Some(constraints))
            }
            (Some(constraints), ParameterComponentQuery2::AllComponents(_)) => {
                ParameterComponentQuery2::AllComponents(Some(constraints))
            }
            _ => query,
        };
        let other = zero.source().to_rational_bezier()?;
        let source = self.source_power_basis()?;
        let other_power = other.homogeneous_power_basis()?;
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        // A retained regular cell may own a one-sided source normal at an
        // endpoint. Use that same primitive field in both equations and replay.
        // Zero displacement needs coordinate equality, without a normal field.
        let tangent_field = if distance_sign != RealSign::Zero
            && let Some(range) = regular_parallel_range
        {
            match self.source_oriented_regularized_tangent_field(range, policy)? {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            None
        };
        let regularized_differential =
            tangent_field
                .as_ref()
                .map(|field| BezierParallelDifferential2 {
                    tangent_derivative_x: polynomial_derivative(&field.x),
                    tangent_derivative_y: polynomial_derivative(&field.y),
                    tangent_x: field.x.clone(),
                    tangent_y: field.y.clone(),
                });
        let (mut equations, mut branch) = if distance_sign == RealSign::Zero {
            // Coordinate equality retains stationary contacts without a
            // zero-speed component or an unused differential construction.
            let unit_weight = [Real::one()];
            (
                parallel_source_equality_equations(self, zero)?,
                rational_pair_defined_selector(
                    source.weight.unwrap_or(&unit_weight),
                    &other_power.weight,
                ),
            )
        } else {
            let differential = match regularized_differential.as_ref() {
                Some(differential) => differential,
                None => self.differential()?,
            };
            let (orthogonality, distance) = parallel_rational_intersection_equations(
                &source,
                differential,
                self.distance(),
                other_power,
            );
            (
                [orthogonality, distance],
                parallel_rational_component_branch(
                    &source,
                    differential,
                    self.distance(),
                    other_power,
                    distance_sign,
                ),
            )
        };
        if off_diagonal {
            debug_assert_eq!(distance_sign, RealSign::Zero);
            let diagonal =
                BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![-Real::one()]]);
            // Two rational images may select opposite normal sheets of one
            // source. Exclude equal source parameters in either case, but
            // remove the diagonal factor only when exact division proves it.
            if let Some(residual) = divide_bivariate_system_component(&equations, &diagonal) {
                equations = residual;
            }
            // Removing the identity factor does not exclude diagonal points
            // where another component meets it, such as a stationary fold.
            // Keep that exclusion in the component cell predicate itself.
            branch = bivariate_multiply(&branch, &bivariate_multiply(&diagonal, &diagonal));
        }
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        let [first_equation, second_equation] = equations;
        let constraint = match parameter_domain_constraint(
            first_equation,
            &second_equation,
            domains,
            policy,
            config,
        )? {
            Classification::Decided(constraint) => constraint,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut selected_pairs = Vec::new();
        let mut retained_contacts = Vec::new();
        let mut domain_components = Vec::new();
        let mut component_overlaps = Vec::new();
        if let Some(support) = constraint.component_support {
            if matches!(query, ParameterComponentQuery2::RetainFinite)
                && !off_diagonal
                && domains
                    .iter()
                    .all(|domain| domain.extension.is_none() && domain.inclusion == [true; 2])
            {
                match self.replay_constant_parameter_components(
                    &other,
                    domains.map(|domain| domain.finite),
                    policy,
                )? {
                    Classification::Decided(Some(intersections)) => {
                        return Ok(Classification::Decided(
                            BezierParallelPairDomainIntersectionSet2::enumerated(
                                parallel_pair_set_from_parallel_rational(intersections, swapped),
                            ),
                        ));
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let selection = match select_parameter_component_in_domain(
                &support,
                &ParameterComponentSelector2::Positive(&branch, None),
                domains,
                query,
                policy,
                config,
            )? {
                Classification::Decided(selection) => selection,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if selection.has_components()
                && matches!(query, ParameterComponentQuery2::FirstComponent(_))
            {
                return Ok(Classification::Decided(
                    BezierParallelPairDomainIntersectionSet2::from_components(if swapped {
                        selection
                            .components
                            .into_iter()
                            .map(CurveParameterComponent2::swapped)
                            .collect()
                    } else {
                        selection.components
                    }),
                ));
            }
            domain_components.extend(selection.components);
            selected_pairs = selection.selected_pairs;
            retained_contacts = selection.retained_contacts;
            component_overlaps = selection.component_overlaps;
        }
        let mut candidate_system = match constraint.isolated_projection {
            Some(projection) => BezierParallelIntersectionCandidateSystem2::projected(
                projection.candidates,
                projection.residual_equations.map(|equations| *equations),
            ),
            None => BezierParallelIntersectionCandidateSystem2::projected(
                CurveIntersectionCandidates2::NoIntersection,
                None,
            ),
        };
        candidate_system.selected_component_pair_count = selected_pairs.len();
        candidate_system.component_pairs = selected_pairs.into();
        candidate_system.overlaps = component_overlaps
            .iter()
            .map(|overlap| overlap.overlap().clone())
            .collect();
        candidate_system.component_overlaps = component_overlaps.into();
        let intersections = match self.replay_parallel_rational_candidate_system(
            &other,
            candidate_system,
            off_diagonal,
            tangent_field.as_deref(),
            regular_parallel_range,
            |parameter| {
                Ok(Classification::Decided(CurvePoint2::from(
                    BezierAnalyticParallelPoint2::new(zero.clone(), parameter.clone(), policy),
                )))
            },
            policy,
        )? {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let intersections = parallel_pair_set_from_parallel_rational(intersections, swapped);
        if swapped {
            domain_components = domain_components
                .into_iter()
                .map(CurveParameterComponent2::swapped)
                .collect();
        }
        if retained_contacts.is_empty() {
            return Ok(Classification::Decided(
                BezierParallelPairDomainIntersectionSet2::with_components(
                    intersections,
                    domain_components,
                ),
            ));
        }
        if swapped {
            for contact in &mut retained_contacts {
                std::mem::swap(&mut contact.first_parameter, &mut contact.second_parameter);
                contact.tangent_cross_sign = contact
                    .tangent_cross_sign
                    .map(|sign| product_sign(sign, RealSign::Negative));
            }
        }
        Ok(merge_parallel_pair_intersection_sets(
            intersections,
            BezierParallelPairIntersectionSet2::complete(retained_contacts.into(), Arc::from([])),
            policy,
        )?
        .map(|result| {
            BezierParallelPairDomainIntersectionSet2::with_components(result, domain_components)
        }))
    }

    /// Returns ordered off-diagonal self-contacts on two retained finite ranges
    /// with independently optional, oriented extensions.
    ///
    /// Zero and PH offsets share the rational pair domain authority. Each PH
    /// axis first certifies its own polynomial speed sign, so a unit-chart
    /// rational image cannot select an exterior normal. General parallels
    /// divide the structural diagonal from both radical equations. All routes
    /// retain the ordered operand roles. Finite region queries retain every
    /// component map; corner queries retain components and isolated contacts.
    /// `regular_sources` selects one-sided endpoint frames only for domains
    /// already partitioned into regular source cells, with same-sheet rays.
    /// General domains keep their pointwise normal selector across singularities.
    pub(crate) fn self_intersections_in_domain(
        &self,
        domains: [CurveParameterDomain2<'_>; 2],
        query: ParameterComponentQuery2<'_>,
        regular_sources: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairDomainIntersectionSet2>> {
        let query = query.without_identity_constraints(policy);
        let strict = policy.strict_counterpart();
        let distance_sign = match real_sign(self.distance(), &strict) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if distance_sign == RealSign::Zero {
            return self.zero_distance_pair_intersections_in_domain(
                self, domains, false, true, query, None, policy,
            );
        }
        if let Classification::Decided(Some([first, second])) =
            self.rational_parallel_components_in_domains(domains, &strict)?
        {
            let first = first.parallel_left(Real::zero())?;
            let second = second.parallel_left(Real::zero())?;
            return first.zero_distance_pair_intersections_in_domain(
                &second, domains, false, true, query, None, policy,
            );
        }
        let mut frames = [None, None];
        if regular_sources {
            let speed = parallel_speed_squared_polynomial(self.differential()?);
            for (axis, domain) in domains.into_iter().enumerate() {
                match polynomial_is_nonzero_on_parameter_range(&speed, domain.finite, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        match polynomial_roots_touch_only_range_endpoints(
                            &speed,
                            domain.finite,
                            policy,
                        )? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => {
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                frames[axis] = match self.source_tangent_field_in_regular_domain(domain, policy)? {
                    Classification::Decided(frame) => frame,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
        }
        let Some(system) = (match parallel_pair_equation_system_with_tangent_fields(
            self,
            self,
            frames[0].as_deref(),
            frames[1].as_deref(),
            false,
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }) else {
            return Ok(Classification::Decided(
                BezierParallelPairDomainIntersectionSet2::enumerated(
                    BezierParallelPairIntersectionSet2::complete(Arc::from([]), Arc::from([])),
                ),
            ));
        };
        let source_diagonal_excluded =
            Classification::Decided(CertifiedParallelSourceOverlap2::without_contacts(
                CertifiedParallelSourceOverlapKind2::Excluded,
            ));
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        let source_components =
            match parallel_nonstructural_source_parameter_components(self, self, config)? {
                Classification::Decided(components) => components,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let source_constraint = match parameter_component_union_support(&source_components) {
            Some(support) => match parameter_domain_constraint(
                support,
                &system.norm_equation,
                domains,
                policy,
                config,
            )? {
                Classification::Decided(constraint) => Some(constraint),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            None => None,
        };
        let diagonal =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![-Real::one()]]);
        let off_diagonal = bivariate_multiply(&diagonal, &diagonal);
        let mut source_isolated_projection = None;
        let mut retained_contacts = Vec::new();
        let mut domain_components = Vec::new();
        if let Some(source_constraint) = source_constraint {
            let mut selected_pairs = Vec::new();
            let mut component_overlaps = Vec::new();
            if let Some(support) = source_constraint.component_support {
                let selection = match select_parameter_component_in_domain(
                    &support,
                    &ParameterComponentSelector2::ParallelPair {
                        normal_constraints: None,
                        system: &system,
                        parameter_filter: Some(&off_diagonal),
                    },
                    domains,
                    query,
                    policy,
                    config,
                )? {
                    Classification::Decided(selection) => selection,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if selection.has_components()
                    && matches!(query, ParameterComponentQuery2::FirstComponent(_))
                {
                    return Ok(Classification::Decided(
                        BezierParallelPairDomainIntersectionSet2::from_components(
                            selection.components,
                        ),
                    ));
                }
                domain_components.extend(selection.components);
                selected_pairs = selection.selected_pairs;
                component_overlaps = selection.component_overlaps;
                retained_contacts.extend(selection.retained_contacts);
            }
            source_isolated_projection = retain_parameter_component_pairs(
                source_constraint.isolated_projection,
                selected_pairs,
                component_overlaps,
            );
        }
        let Some(projection) = project_parallel_pair_without_components_in_domain(
            &system,
            self,
            self,
            &source_diagonal_excluded,
            domains,
            query,
            Some(&off_diagonal),
            policy,
        )?
        else {
            return Ok(Classification::Decided(
                BezierParallelPairDomainIntersectionSet2::with_components(
                    BezierParallelPairIntersectionSet2::incomplete(
                        retained_contacts.into(),
                        Arc::from([]),
                        CurveIntersectionCandidates2::DegenerateResultant,
                    ),
                    domain_components,
                ),
            ));
        };
        let mut projection = match projection {
            BezierParallelPairDomainProjection2::Enumerated {
                projection,
                retained_contacts: residual_contacts,
                components: residual_components,
            } => {
                retained_contacts.extend(residual_contacts);
                domain_components.extend(residual_components);
                projection
            }
            BezierParallelPairDomainProjection2::Components(components) => {
                return Ok(Classification::Decided(
                    BezierParallelPairDomainIntersectionSet2::from_components(components),
                ));
            }
        };
        if let Some(source_projection) = source_isolated_projection {
            prepend_parallel_pair_projection(&mut projection, source_projection);
        }
        let result = self.replay_parallel_pair_projection_with_ranges(
            self,
            &system,
            projection,
            BezierParallelPairParameterSelection2::OffDiagonal,
            regular_sources.then(|| domains.map(|domain| domain.finite)),
            policy,
        )?;
        let result = extend_parallel_pair_contacts(result, retained_contacts, policy)?;
        let mut result = match result {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        // Saturation removes the identity component, but residual branches
        // can still meet it. Those visits are not off-diagonal self contacts.
        let mut contacts = Vec::with_capacity(result.contacts.len());
        for contact in result.contacts.iter() {
            match contact
                .first_parameter()
                .cmp_by_refinement(contact.second_parameter(), &strict)?
            {
                Classification::Decided(std::cmp::Ordering::Equal) => {}
                Classification::Decided(_) => contacts.push(contact.clone()),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        result.contacts = contacts.into();
        Ok(Classification::Decided(
            BezierParallelPairDomainIntersectionSet2::with_components(result, domain_components),
        ))
    }

    /// Tries the exact-rational parallel-pair routes.
    ///
    /// Native unit queries keep their specialized prefix. Retained branches
    /// select a rational image on the requested normal sheet and share the
    /// finite-domain authority, including source endpoints and components.
    /// General projection follows when neither operand has a rational image.
    pub(in crate::bezier_offset) fn rational_parallel_pair_intersections(
        &self,
        other: &Self,
        ranges: Option<[&CurveParameterRange2; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelPairIntersectionSet2>>> {
        let unit = CurveParameterRange2::unit();
        let strict = policy.strict_counterpart();
        let complete_unit =
            ranges.is_none_or(|ranges| ranges.into_iter().all(|range| range == &unit));
        let covered_by_unit = complete_unit
            || ranges.is_some_and(|ranges| {
                let unit_domain = CurveParameterDomain2::new(&unit, None);
                ranges.into_iter().all(|range| {
                    matches!(
                        unit_domain.contains_finite_range(range, &strict),
                        Ok(Classification::Decided(true))
                    )
                })
            });
        if covered_by_unit {
            let native = self.rational_unit_parallel_pair_intersections(other, ranges, policy);
            if complete_unit && !matches!(&native, Ok(Classification::Decided(None))) {
                return native;
            }
            if let Ok(Classification::Decided(Some(mut intersections))) = native
                && intersections.is_complete()
                && intersections.parameter_components().is_empty()
            {
                // A complete enclosing projection can retain its native root
                // identities. Exact membership alone clips isolated contacts.
                // Retain overlap correspondence evidence for the support/region
                // caller's correlated clipping; rebuilding it from coupled
                // polynomials would discard certified maps and selected fibers.
                let ranges = ranges.expect("a restricted query has both ranges");
                let mut contacts = Vec::new();
                let mut complete = true;
                'contacts: for contact in intersections.contacts() {
                    for (parameter, range) in
                        [contact.first_parameter(), contact.second_parameter()]
                            .into_iter()
                            .zip(ranges)
                    {
                        match CurveParameterDomain2::new(range, None)
                            .contains_finite_parameter(parameter, &strict)
                        {
                            Ok(Classification::Decided(true)) => {}
                            Ok(Classification::Decided(false)) => continue 'contacts,
                            _ => {
                                complete = false;
                                break 'contacts;
                            }
                        }
                    }
                    contacts.push(contact.clone());
                }
                if complete {
                    intersections.contacts = contacts.into();
                    return Ok(Classification::Decided(Some(intersections)));
                }
            }
        }
        let Some(ranges) = ranges else {
            return Ok(Classification::Decided(None));
        };
        Ok(self
            .rational_parallel_pair_intersections_in_regular_domains(
                other,
                ranges.map(|range| CurveParameterDomain2::new(range, None)),
                ParameterComponentQuery2::RetainFinite,
                policy,
            )?
            .map(|result| {
                result.map(|result| {
                    debug_assert!(result.components.is_empty());
                    result.intersections
                })
            }))
    }

    pub(in crate::bezier_offset) fn rational_parallel_pair_intersections_in_regular_domains(
        &self,
        other: &Self,
        domains: [CurveParameterDomain2<'_>; 2],
        query: ParameterComponentQuery2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelPairDomainIntersectionSet2>>> {
        let strict = policy.strict_counterpart();
        // PH images must select the requested normal sheet, including exterior
        // intervals. Their unchanged source parameters enter the existing
        // finite-domain authority; no new coordinate or parameter image is needed.
        for swapped in [false, true] {
            let (parallel, rational, domains) = if swapped {
                (other, self, [domains[1], domains[0]])
            } else {
                (self, other, domains)
            };
            let ranges = domains.map(|domain| domain.finite);
            let Ok(Classification::Decided(Some(component))) =
                strict.bounded_exact_predicate_pass(|| {
                    rational.exact_rational_parallel_component_on_regular_range(ranges[1], &strict)
                })
            else {
                continue;
            };
            let zero = component.curve().parallel_left(Real::zero())?;
            // When both images exist, coordinate equality is sufficient.
            // Keeping an unnecessary radical here would raise the equation
            // degree and repeat component extraction for the same exact curves.
            let first_image = match strict.bounded_exact_predicate_pass(|| {
                parallel.exact_rational_parallel_component_on_regular_range(ranges[0], &strict)
            }) {
                Ok(Classification::Decided(Some(component))) => {
                    Some(component.curve().parallel_left(Real::zero())?)
                }
                _ => None,
            };
            let regular_parallel_range = first_image.is_none().then_some(ranges[0]);
            let parallel = first_image.as_ref().unwrap_or(parallel);
            return Ok(parallel
                .zero_distance_pair_intersections_in_domain(
                    &zero,
                    domains,
                    swapped,
                    false,
                    query,
                    regular_parallel_range,
                    policy,
                )?
                .map(|result| {
                    #[cfg(feature = "dispatch-trace")]
                    if result.intersections.is_complete() {
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "parallel-pair-regular-ranges",
                            "rational-domains",
                        );
                    }
                    Some(result)
                }));
        }
        Ok(Classification::Decided(None))
    }

    /// Reuses native unit projection and its specialized circle/line routes.
    /// Retained ranges select source frames. The enclosing unit evidence needs
    /// contact membership and correlated overlap clipping before publication
    /// for a smaller parameter domain.
    pub(in crate::bezier_offset) fn rational_unit_parallel_pair_intersections(
        &self,
        other: &Self,
        ranges: Option<[&CurveParameterRange2; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelPairIntersectionSet2>>> {
        match other.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(other)) => {
                // Certified finite line images can decide disjointness with
                // one support predicate. Restrict this prefix to line fits:
                // a second general intersection solve would duplicate algebra
                // and refinement for positive or nonlinear pairs.
                if let Classification::Decided(Some(first)) =
                    self.exact_rational_parallel_component(policy)?
                    && let Classification::Decided(BezierLineImageFitRelation::Fit(first)) =
                        first.fit_exact_line_image_with_policy(policy)?
                    && let Classification::Decided(BezierLineImageFitRelation::Fit(second)) =
                        other.fit_exact_line_image_with_policy(policy)?
                    && matches!(
                        first
                            .line()
                            .intersect_line_with_policy(second.line(), policy)?,
                        crate::LineLineIntersection::None
                    )
                {
                    return Ok(Classification::Decided(Some(
                        BezierParallelPairIntersectionSet2::complete(Arc::from([]), Arc::from([])),
                    )));
                }
                let intersections = match ranges {
                    Some([first_range, _]) => {
                        self.intersections_on_regular_range(&other, first_range, policy)
                    }
                    None => self.intersections(&other, policy),
                }?;
                return Ok(intersections
                    .map(|result| Some(parallel_pair_set_from_parallel_rational(result, false))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match self.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(first)) => {
                let intersections = match ranges {
                    Some([_, second_range]) => {
                        other.intersections_on_regular_range(&first, second_range, policy)
                    }
                    None => other.intersections(&first, policy),
                }?;
                return Ok(intersections
                    .map(|result| Some(parallel_pair_set_from_parallel_rational(result, true))));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(Classification::Decided(None))
    }

    pub(in crate::bezier_offset) fn replay_parallel_pair_projection(
        &self,
        other: &Self,
        system: &BezierParallelPairEquationSystem2,
        projection: BezierParallelPairProjection2,
        selection: BezierParallelPairParameterSelection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        self.replay_parallel_pair_projection_with_ranges(
            other, system, projection, selection, None, policy,
        )
    }

    pub(in crate::bezier_offset) fn replay_parallel_pair_projection_with_ranges(
        &self,
        other: &Self,
        system: &BezierParallelPairEquationSystem2,
        mut projection: BezierParallelPairProjection2,
        selection: BezierParallelPairParameterSelection2,
        regular_ranges: Option<[&CurveParameterRange2; 2]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
        if let Some(radical_component_projection) = projection.radical_component_projection.take() {
            let radical_component = match self.replay_parallel_pair_projection_with_ranges(
                other,
                system,
                *radical_component_projection,
                selection,
                regular_ranges,
                policy,
            )? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let residual = match self.replay_parallel_pair_projection_with_ranges(
                other,
                system,
                projection,
                selection,
                regular_ranges,
                policy,
            )? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return merge_parallel_pair_intersection_sets(radical_component, residual, policy);
        }
        let BezierParallelPairProjection2 {
            candidates,
            basis: projection_basis,
            overlap,
            component_overlap_evidence,
            component_pairs,
            selected_component_pair_count,
            residual_equations,
            radical_component_projection: _,
        } = projection;
        if let Some(overlap) = overlap.as_ref()
            && component_overlap_evidence.is_empty()
            && component_pairs.is_empty()
            && residual_equations.is_none()
        {
            return Ok(Classification::Decided(
                BezierParallelPairIntersectionSet2::complete(
                    Arc::from([]),
                    Arc::from([overlap.clone()]),
                ),
            ));
        }
        // The component certificates own their domains. Publish each identical
        // interval record once, while retaining every map and selected branch.
        // A duplicated interval would make consumers visit all matching maps
        // repeatedly; no exact comparison or geometric equivalence is assumed.
        let mut component_overlaps = Vec::new();
        for component in component_overlap_evidence.iter() {
            let overlap = component.overlap();
            if !component_overlaps.contains(overlap) {
                component_overlaps.push(overlap.clone());
            }
        }
        let overlap_correspondence = if let Some(overlap) = overlap.as_ref() {
            let first_source = self.source().to_rational_bezier()?;
            let second_source = other.source().to_rational_bezier()?;
            Some(RationalBezierOverlapParameterCorrespondence2::for_overlap(
                &first_source,
                &second_source,
                overlap,
                policy,
            ))
        } else {
            None
        };

        let empty_parameters: &[BezierParameter2] = &[];
        let (first_parameters, second_parameters, projection_incomplete) = match &candidates {
            CurveIntersectionCandidates2::NoIntersection => {
                (empty_parameters, empty_parameters, false)
            }
            CurveIntersectionCandidates2::DegenerateResultant => {
                (empty_parameters, empty_parameters, true)
            }
            CurveIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => (
                first_parameters.as_slice(),
                second_parameters.as_slice(),
                false,
            ),
        };

        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        let mut first_lifts: [Option<CurveIntersectionParameterLiftReport>; 2] = [None, None];
        let mut second_lifts: [Option<CurveIntersectionParameterLiftReport>; 2] = [None, None];
        let mut contacts = Vec::new();
        let mut incomplete = projection_incomplete;
        let derivative_scale_sign =
            |parallel: &BezierParallel2, parameter: &BezierParameter2, index: usize| {
                if let Some(ranges) = regular_ranges {
                    parallel.parallel_derivative_scale_sign_on_regular_range(
                        &parameter.clone().into(),
                        ranges[index],
                        policy,
                    )
                } else {
                    parallel.parallel_derivative_scale_sign(&parameter.clone().into(), policy)
                }
            };
        let (
            projection_first,
            projection_second,
            replay_first,
            replay_second,
            projection_proves_both_radicals,
        ) = if let Some(residual) = residual_equations.as_deref() {
            (
                &residual[0],
                &residual[1],
                &residual[0],
                &system.norm_equation,
                true,
            )
        } else {
            match projection_basis {
                BezierParallelPairProjectionBasis2::ProjectionEquations => (
                    &system.first_equation,
                    &system.second_equation,
                    &system.first_equation,
                    &system.norm_equation,
                    true,
                ),
                BezierParallelPairProjectionBasis2::FirstAndNorm => (
                    &system.first_equation,
                    &system.norm_equation,
                    &system.second_equation,
                    &system.norm_equation,
                    false,
                ),
            }
        };
        for first_parameter in first_parameters {
            for second_parameter in second_parameters {
                if let (Some(overlap), Some(correspondence)) =
                    (overlap.as_ref(), overlap_correspondence.as_ref())
                {
                    match parallel_parameter_pair_is_on_overlap_correspondence(
                        overlap,
                        correspondence,
                        first_parameter,
                        second_parameter,
                        policy,
                    )? {
                        Classification::Decided(true) => continue,
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                match parallel_parameter_pair_is_overlap_boundary(
                    &component_overlaps,
                    first_parameter,
                    second_parameter,
                    policy,
                )? {
                    Classification::Decided(true) => continue,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                }
                match selection.admits(first_parameter, second_parameter, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let third_filter = if projection_proves_both_radicals {
                    &system.norm_equation
                } else {
                    &system.second_equation
                };
                // Test every equation at each box before refining either root
                // again. A zero equation cannot consume all refinement steps
                // before another equation gets a chance to reject the pair.
                if refine_parameter_pair_for_certificate(
                    first_parameter,
                    second_parameter,
                    policy,
                    |first, second| {
                        for equation in [projection_first, projection_second, third_filter] {
                            if bivariate_parameter_box_strict_sign(equation, first, second, policy)?
                                .is_some()
                            {
                                return Ok(Some(()));
                            }
                        }
                        Ok(None)
                    },
                )?
                .is_some()
                {
                    continue;
                }
                let first_replay = if projected_bivariate_parameter_pair_has_box_root(
                    projection_first,
                    projection_second,
                    first_parameter,
                    second_parameter,
                    policy,
                )? {
                    BivariateParameterPairReplay::Direct
                } else {
                    match replay_bivariate_parameter_pair(
                        projection_first,
                        projection_second,
                        first_parameter,
                        second_parameter,
                        policy,
                        config,
                        &mut first_lifts,
                    )? {
                        Classification::Decided(BivariateParameterPairReplay::Rejected) => continue,
                        Classification::Decided(replay) => replay,
                        Classification::Uncertain(_) => {
                            incomplete = true;
                            continue;
                        }
                    }
                };
                let tangent_cross = match signed_bivariate_for_replay_or_parameter_box(
                    &system.tangent_cross,
                    first_parameter,
                    second_parameter,
                    first_replay,
                    &first_lifts,
                    policy,
                )? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => None,
                };
                // For independent regular tangents the two selected radical
                // equations determine the separation vector uniquely. Their
                // norm eliminant is then an algebraic consequence, so avoid a
                // second algebraic-fiber replay. Tangent-parallel candidates
                // and the FirstAndNorm projection retain that replay.
                let radicals_determine_separation = projection_proves_both_radicals
                    && matches!(tangent_cross, Some(RealSign::Positive | RealSign::Negative));
                let second_replay = if radicals_determine_separation {
                    first_replay
                } else {
                    match replay_bivariate_parameter_pair(
                        replay_first,
                        replay_second,
                        first_parameter,
                        second_parameter,
                        policy,
                        config,
                        &mut second_lifts,
                    )? {
                        Classification::Decided(BivariateParameterPairReplay::Rejected) => continue,
                        Classification::Decided(replay) => replay,
                        Classification::Uncertain(_) => {
                            incomplete = true;
                            continue;
                        }
                    }
                };
                let second_replay_lifts = if radicals_determine_separation {
                    &first_lifts
                } else {
                    &second_lifts
                };
                match parallel_pair_selected_branch(
                    system,
                    first_parameter,
                    second_parameter,
                    first_replay,
                    &first_lifts,
                    second_replay,
                    second_replay_lifts,
                    radicals_determine_separation,
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                }
                let tangent_relation = match (
                    derivative_scale_sign(self, first_parameter, 0)?,
                    derivative_scale_sign(other, second_parameter, 1)?,
                ) {
                    (
                        Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)),
                        Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)),
                    ) => {
                        let scale = product_sign(first, second);
                        let cross = match signed_bivariate_for_either_replay(
                            &system.tangent_cross,
                            first_parameter,
                            second_parameter,
                            first_replay,
                            &first_lifts,
                            second_replay,
                            second_replay_lifts,
                            policy,
                        )? {
                            Classification::Decided(sign) => Some(product_sign(sign, scale)),
                            Classification::Uncertain(_) => None,
                        };
                        let dot = match signed_bivariate_for_either_replay(
                            &system.tangent_dot,
                            first_parameter,
                            second_parameter,
                            first_replay,
                            &first_lifts,
                            second_replay,
                            second_replay_lifts,
                            policy,
                        )? {
                            Classification::Decided(sign) => Some(product_sign(sign, scale)),
                            Classification::Uncertain(_) => None,
                        };
                        (cross, dot)
                    }
                    _ => (None, None),
                };
                let (tangent_cross_sign, tangent_dot_sign) = tangent_relation;
                let contact = BezierParallelPairIntersectionContact2 {
                    first_parameter: first_parameter.clone().into(),
                    second_parameter: second_parameter.clone().into(),
                    certified_transverse: matches!(
                        tangent_cross_sign,
                        Some(RealSign::Positive | RealSign::Negative)
                    ) || self.certified_transverse_parallel_contact(
                        other,
                        first_parameter,
                        second_parameter,
                        policy,
                    ),
                    tangent_cross_sign,
                    tangent_dot_sign,
                };
                match parallel_pair_contact_parameters_are_retained(
                    &contacts,
                    &first_parameter.clone().into(),
                    &second_parameter.clone().into(),
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => contacts.push(contact),
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        contacts.push(contact);
                    }
                }
            }
        }
        let (selected_component_pairs, _) = component_pairs.split_at(selected_component_pair_count);
        for pair in selected_component_pairs {
            let first_parameter = &pair.parallel_parameter;
            let second_parameter = &pair.other_parameter;
            match parallel_parameter_pair_is_overlap_boundary(
                &component_overlaps,
                first_parameter,
                second_parameter,
                policy,
            )? {
                Classification::Decided(true) => continue,
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => {
                    incomplete = true;
                    continue;
                }
            }
            if let (Some(overlap), Some(correspondence)) =
                (overlap.as_ref(), overlap_correspondence.as_ref())
            {
                match parallel_parameter_pair_is_on_overlap_correspondence(
                    overlap,
                    correspondence,
                    first_parameter,
                    second_parameter,
                    policy,
                )? {
                    Classification::Decided(true) => continue,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            match selection.admits(first_parameter, second_parameter, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let mut equations_hold = true;
            for equation in [
                &system.first_equation,
                &system.second_equation,
                &system.norm_equation,
            ] {
                match signed_bivariate_at_parameter_pair(
                    equation,
                    first_parameter,
                    second_parameter,
                    policy,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                        equations_hold = false;
                        break;
                    }
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        equations_hold = false;
                        break;
                    }
                }
            }
            if !equations_hold {
                continue;
            }
            match parallel_pair_selected_branch(
                system,
                first_parameter,
                second_parameter,
                BivariateParameterPairReplay::Direct,
                &[None, None],
                BivariateParameterPairReplay::Direct,
                &[None, None],
                false,
                policy,
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(_) => {
                    incomplete = true;
                    continue;
                }
            }
            let tangent_relation = match (
                derivative_scale_sign(self, first_parameter, 0)?,
                derivative_scale_sign(other, second_parameter, 1)?,
                signed_bivariate_at_parameter_pair(
                    &system.tangent_cross,
                    first_parameter,
                    second_parameter,
                    policy,
                )?,
                signed_bivariate_at_parameter_pair(
                    &system.tangent_dot,
                    first_parameter,
                    second_parameter,
                    policy,
                )?,
            ) {
                (
                    Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)),
                    Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)),
                    Classification::Decided(cross),
                    Classification::Decided(dot),
                ) => {
                    let scale = product_sign(first, second);
                    (
                        Some(product_sign(cross, scale)),
                        Some(product_sign(dot, scale)),
                    )
                }
                _ => (None, None),
            };
            let (tangent_cross_sign, tangent_dot_sign) = tangent_relation;
            let contact = BezierParallelPairIntersectionContact2 {
                first_parameter: first_parameter.clone().into(),
                second_parameter: second_parameter.clone().into(),
                certified_transverse: matches!(
                    tangent_cross_sign,
                    Some(RealSign::Positive | RealSign::Negative)
                ) || self.certified_transverse_parallel_contact(
                    other,
                    first_parameter,
                    second_parameter,
                    policy,
                ),
                tangent_cross_sign,
                tangent_dot_sign,
            };
            match parallel_pair_contact_parameters_are_retained(
                &contacts,
                &first_parameter.clone().into(),
                &second_parameter.clone().into(),
                policy,
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => contacts.push(contact),
                Classification::Uncertain(_) => {
                    incomplete = true;
                    contacts.push(contact);
                }
            }
        }
        let contacts = contacts.into();
        let mut overlaps = component_overlaps;
        if let Some(overlap) = overlap
            && !overlaps.contains(&overlap)
        {
            overlaps.push(overlap);
        }
        let overlaps = overlaps.into();
        Ok(Classification::Decided(if incomplete {
            BezierParallelPairIntersectionSet2::incomplete(contacts, overlaps, candidates)
        } else {
            BezierParallelPairIntersectionSet2::complete_with_supplement(
                contacts,
                overlaps,
                Arc::from([]),
                component_overlap_evidence,
            )
        }))
    }
}
