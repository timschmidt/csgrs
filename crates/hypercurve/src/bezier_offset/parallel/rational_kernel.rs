//! Parallel/rational-Bezier intersections and rational components.

use super::*;

impl BezierParallel2 {
    #[cfg(test)]
    /// Constructs complete parameter projections for intersections with a rational Bezier.
    ///
    /// For target `Q(u)=A(u)/B(u)`, source `P(t)=(X(t)/W(t),Y(t)/W(t))`,
    /// homogeneous tangent numerator `H(t)`, and
    /// `Delta=(A_x W-XB,A_y W-YB)`, every unsigned parallel contact satisfies
    /// `Delta dot H=0` and `Delta dot Delta-d^2 W^2 B^2=0`. Hypersolve
    /// eliminates each parameter from that one bivariate system. The returned
    /// projections are candidate evidence: exact contact replay must still
    /// pair roots and reject the opposite normal branch introduced by the
    /// squared distance equation.
    pub(crate) fn intersection_candidates(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveIntersectionCandidates2>> {
        if let Some(Some(offset)) = self.data.certified_ph_offset.get() {
            return offset
                .curve()
                .intersection_candidates_classified(other, policy);
        }
        Ok(self
            .intersection_candidate_system(other, policy)?
            .map(|system| {
                if system.overlaps.is_empty() {
                    system.candidates
                } else {
                    CurveIntersectionCandidates2::DegenerateResultant
                }
            }))
    }

    #[cfg(test)]
    pub(in crate::bezier_offset) fn intersection_candidate_system(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
        self.intersection_candidate_system_with_tangent_field(other, None, policy)
    }

    pub(in crate::bezier_offset) fn intersection_candidate_system_with_tangent_field(
        &self,
        other: &RationalBezier2,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if distance_sign == RealSign::Zero {
            let source = self.source().to_rational_bezier()?;
            if let Classification::Decided(candidates) =
                source.intersection_candidates_classified(other, policy)?
                && !matches!(
                    candidates,
                    CurveIntersectionCandidates2::DegenerateResultant
                )
            {
                return Ok(Classification::Decided(
                    BezierParallelIntersectionCandidateSystem2::projected(candidates, None),
                ));
            }
        }
        if tangent_field.is_none()
            && let Some(Some(offset)) = self.data.certified_ph_offset.get()
        {
            match offset
                .curve()
                .intersection_candidates_classified(other, policy)?
            {
                Classification::Decided(candidates) => {
                    if !matches!(
                        candidates,
                        CurveIntersectionCandidates2::DegenerateResultant
                    ) {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionCandidateSystem2::projected(candidates, None),
                        ));
                    }
                    if let Classification::Decided(contacts) = offset
                        .curve()
                        .intersection_contacts_classified(other, policy)?
                        && let Some(overlap) = contacts.overlap().cloned()
                    {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionCandidateSystem2::overlaps(Arc::from([
                                overlap,
                            ])),
                        ));
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let source = self.source_power_basis()?;
        if let Classification::Uncertain(reason) = Self::certify_finite_source(&source, policy)? {
            return Ok(Classification::Uncertain(reason));
        }
        let other_power = other.homogeneous_power_basis()?;
        if let Classification::Uncertain(reason) =
            Self::certify_finite_weight(Some(&other_power.weight), policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        let regularized_differential = tangent_field.map(|field| BezierParallelDifferential2 {
            tangent_derivative_x: polynomial_derivative(&field.x),
            tangent_derivative_y: polynomial_derivative(&field.y),
            tangent_x: field.x.clone(),
            tangent_y: field.y.clone(),
        });
        let differential = match regularized_differential.as_ref() {
            Some(differential) => differential,
            None => self.differential()?,
        };
        if distance_sign != RealSign::Zero
            && let Classification::Uncertain(reason) =
                Self::certify_regular_differential(differential, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }

        let other_bounds = other.certified_bounds_classified();
        if let (Classification::Decided(parallel_bounds), Classification::Decided(other_bounds)) =
            (self.conservative_bounds()?, other_bounds)
            && matches!(
                parallel_bounds.overlaps(&other_bounds, policy),
                Classification::Decided(false)
            )
        {
            return Ok(Classification::Decided(
                BezierParallelIntersectionCandidateSystem2::projected(
                    CurveIntersectionCandidates2::NoIntersection,
                    None,
                ),
            ));
        }

        let (orthogonality, distance_relation) = parallel_rational_intersection_equations(
            &source,
            differential,
            self.distance(),
            other_power,
        );
        let equations = [orthogonality, distance_relation];
        for parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            if equations.iter().all(|equation| {
                matches!(
                    bivariate_polynomial_is_independent_of_parameter(equation, parameter, policy),
                    Classification::Decided(true)
                )
            }) {
                return Ok(Classification::Decided(
                    BezierParallelIntersectionCandidateSystem2::projected(
                        CurveIntersectionCandidates2::DegenerateResultant,
                        Some(equations),
                    ),
                ));
            }
        }
        if other.degree() >= 4 && bivariate_system_may_have_component(&equations) {
            let reduced = hypersolve::saturate_rootless_bivariate_axis_factors(
                &equations,
                [[&Real::zero(), &Real::one()]; 2],
            );
            let component_equations = reduced.as_ref().unwrap_or(&equations);
            if bivariate_system_may_have_component(component_equations) {
                if tangent_field.is_none()
                    && let Classification::Decided(Some(exact_parallel)) =
                        self.exact_rational_parallel_component(policy)?
                    && let Classification::Decided(contacts) =
                        exact_parallel.intersection_contacts_classified(other, policy)?
                    && let Some(overlap) = contacts.overlap().cloned()
                {
                    return Ok(Classification::Decided(
                        BezierParallelIntersectionCandidateSystem2::overlaps(Arc::from([overlap])),
                    ));
                }
                let branch = parallel_rational_component_branch(
                    &source,
                    differential,
                    self.distance(),
                    other_power,
                    distance_sign,
                );
                let config = CurveIntersectionResultantConfig {
                    min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                    max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                };
                if let Classification::Decided(Some(component)) =
                    parameter_component_system(component_equations, &branch, policy, config)?
                {
                    return parallel_candidate_system_from_parameter_components(component, policy);
                }
            }
        }
        let candidates = match project_parallel_intersection_system(
            &equations[0],
            &equations[1],
            [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if matches!(
            candidates,
            CurveIntersectionCandidates2::DegenerateResultant
        ) {
            match if tangent_field.is_none() {
                self.exact_rational_parallel_component(policy)?
            } else {
                Classification::Decided(None)
            } {
                Classification::Decided(Some(exact_parallel)) => {
                    match exact_parallel.intersection_candidates_classified(other, policy)? {
                        Classification::Decided(candidates) => {
                            if !matches!(
                                candidates,
                                CurveIntersectionCandidates2::DegenerateResultant
                            ) {
                                return Ok(Classification::Decided(
                                    BezierParallelIntersectionCandidateSystem2::projected(
                                        candidates, None,
                                    ),
                                ));
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let candidate_system =
            match parallel_intersection_candidate_system(equations, candidates, policy)? {
                Classification::Decided(candidate_system) => candidate_system,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if matches!(
            candidate_system.candidates,
            CurveIntersectionCandidates2::DegenerateResultant
        ) && let Some(component_equations) = candidate_system.replay_equations.as_ref()
        {
            let branch = parallel_rational_component_branch(
                &source,
                differential,
                self.distance(),
                other_power,
                distance_sign,
            );
            let config = CurveIntersectionResultantConfig {
                min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            };
            if let Classification::Decided(Some(component)) =
                parameter_component_system(component_equations, &branch, policy, config)?
            {
                return parallel_candidate_system_from_parameter_components(component, policy);
            }
        }
        Ok(Classification::Decided(candidate_system))
    }

    /// Intersects an analytic parallel with a recognized rational
    /// circle through the shared univariate circle-incidence authority.
    pub(in crate::bezier_offset) fn rational_quadratic_circle_intersections_fast_path_with_tangent_field(
        &self,
        other: &RationalBezier2,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        retained_parallel_range: Option<&CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelIntersectionSet2>>> {
        let no_fast_path = || Classification::Decided(None);
        if policy.permits_approximate_512() {
            let strict_policy = policy.strict_counterpart();
            match self.rational_quadratic_circle_intersections_fast_path_with_tangent_field(
                other,
                tangent_field,
                retained_parallel_range,
                &strict_policy,
            )? {
                Classification::Decided(Some(intersections)) => {
                    return Ok(Classification::Decided(Some(intersections)));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
        }
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(no_fast_path());
        }
        let conic = match other.materialized_quadratic_representative(policy)? {
            Classification::Decided(Some(conic)) => conic,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                return Ok(no_fast_path());
            }
        };
        let support = match crate::arc_bezier::rational_quadratic_circular_arc(&conic, policy)? {
            Classification::Decided(Some(support)) => support,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                return Ok(no_fast_path());
            }
        };
        let certified_tangent_parameters = other
            .retained_circular_conic()
            .and_then(|circle| circle.tangent_contacts.as_deref())
            .into_iter()
            .flatten()
            .filter_map(|contact| match contact {
                crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(
                    contact,
                ) if contact.parallel == *self => Some((
                    contact.parameter.clone(),
                    contact.eliminant_root_multiplicity,
                )),
                crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(_)
                | crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line { .. } => {
                    None
                }
            })
            .collect::<Vec<_>>();
        let parameters = match self.circle_incidence_with_tangent_field(
            support.center(),
            support.radius_squared_ref(),
            retained_parallel_range.unwrap_or(&CurveParameterRange2::unit()),
            &certified_tangent_parameters,
            tangent_field,
            policy,
        )? {
            Classification::Decided(parameters) => parameters,
            // A coincident circle or an unsupported coefficient tower must
            // continue through the component-aware generic authority.
            Classification::Uncertain(_) => {
                return Ok(no_fast_path());
            }
        };
        if parameters.is_empty() {
            // A certified empty supporting-circle intersection needs no
            // inverse parameter chart or generic algebraic replay.
            return Ok(Classification::Decided(Some(
                BezierParallelIntersectionSet2::complete(Arc::from([]), Arc::from([])),
            )));
        }
        let Ok(Classification::Decided(decomposition)) =
            support.rational_bezier_decomposition_raw(policy)
        else {
            return Ok(no_fast_path());
        };
        let [canonical_span] = decomposition.spans() else {
            return Ok(no_fast_path());
        };
        let canonical_conic = canonical_span.curve().clone();
        let Some(parameter_map_coefficients) = self.circle_rational_quadratic_parameter_maps(
            support.center(),
            support.radius_squared_ref(),
            &canonical_conic,
            tangent_field,
            parameters.iter().map(|(parameter, _)| parameter),
            policy,
        )?
        else {
            return Ok(no_fast_path());
        };
        let parameter_map_coefficients =
            parameter_map_coefficients.map(|(numerator, denominator)| {
                reduce_exact_rational_parameter_map(numerator, denominator)
            });

        let authored_conic: RationalBezier2 = conic.into();
        let rational_conic: RationalBezier2 = canonical_conic.into();
        let source = self.source_power_basis()?;
        let conic_power = rational_conic.homogeneous_power_basis()?;
        let [conic_tangent_x, conic_tangent_y] = rational_parametric_tangent_numerator(conic_power)
            .map(polynomial_trim_structural_zeros);
        let regularized_differential = tangent_field.map(|field| BezierParallelDifferential2 {
            tangent_derivative_x: polynomial_derivative(&field.x),
            tangent_derivative_y: polynomial_derivative(&field.y),
            tangent_x: field.x.clone(),
            tangent_y: field.y.clone(),
        });
        let differential = match regularized_differential.as_ref() {
            Some(differential) => differential,
            None => self.differential()?,
        };
        let tangent_cross = bivariate_subtract(
            &bivariate_outer_product(&differential.tangent_x, &conic_tangent_y),
            &bivariate_outer_product(&differential.tangent_y, &conic_tangent_x),
        );
        let tangent_dot = bivariate_add(
            &bivariate_outer_product(&differential.tangent_x, &conic_tangent_x),
            &bivariate_outer_product(&differential.tangent_y, &conic_tangent_y),
        );
        let (orthogonality, distance_relation) = parallel_rational_intersection_equations(
            &source,
            differential,
            self.distance(),
            conic_power,
        );
        let selected_branch =
            parallel_rational_selected_branch(&source, differential, self.distance(), conic_power);
        let mut parameter_maps =
            parameter_map_coefficients
                .each_ref()
                .map(|(numerator, denominator)| {
                    RationalParameterImageMap2::new(numerator.clone(), denominator.clone(), policy)
                });
        let mut parameter_lifts: [Option<CurveIntersectionParameterLiftReport>; 2] = [None, None];
        let lift_config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        let mut contacts = Vec::with_capacity(parameters.len());
        for (parallel_parameter, radial_crossing_sign) in parameters {
            if let Some(range) = retained_parallel_range {
                match CurveParameterDomain2::new(range, None)
                    .contains_finite_parameter(&parallel_parameter.clone().into(), policy)?
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(_) => {
                        return Ok(no_fast_path());
                    }
                }
            }
            let mut mapped = None;
            let mut unresolved = false;
            for (map_index, parameter_map) in parameter_maps.iter_mut().enumerate() {
                let (numerator, denominator) = &parameter_map_coefficients[map_index];
                let selected_image = selected_rational_parameter_image(
                    numerator,
                    denominator,
                    &parallel_parameter,
                    policy,
                )?;
                let image_is_correlated = selected_image.is_some();
                let image = match selected_image {
                    Some(image) => image,
                    None => parameter_map.image(&parallel_parameter)?,
                };
                match image {
                    Classification::Decided(Some(parameter)) => {
                        match replay_parallel_rational_contact_pair(
                            &orthogonality,
                            &distance_relation,
                            &selected_branch,
                            &parallel_parameter,
                            &parameter,
                            lift_config,
                            &mut parameter_lifts,
                            policy,
                        )? {
                            Classification::Decided(Some(replay)) => {
                                mapped = Some((parameter, Some(replay)));
                                break;
                            }
                            Classification::Decided(None) => {}
                            Classification::Uncertain(_) if image_is_correlated => {
                                // `selected_rational_parameter_image` retains
                                // the exact fiber relation `u D(t)-N(t)=0` at
                                // this isolated circle-incidence root.  The
                                // chart was derived after substituting the
                                // already branch-filtered circle equation, so
                                // this correlation proves the same affine
                                // contact even when independent-root replay
                                // cannot rebuild a common coefficient field.
                                mapped = Some((parameter, None));
                                break;
                            }
                            Classification::Uncertain(_) => unresolved = true,
                        }
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(_) => unresolved = true,
                }
            }
            if mapped.is_none() {
                for canonical_parameter in [
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ] {
                    match replay_parallel_rational_contact_pair(
                        &orthogonality,
                        &distance_relation,
                        &selected_branch,
                        &parallel_parameter,
                        &canonical_parameter,
                        lift_config,
                        &mut parameter_lifts,
                        policy,
                    )? {
                        Classification::Decided(Some(replay)) => {
                            mapped = Some((canonical_parameter, Some(replay)));
                            break;
                        }
                        Classification::Decided(None) => {}
                        Classification::Uncertain(_) => unresolved = true,
                    }
                }
            }
            let Some((canonical_parameter, replay)) = mapped else {
                if unresolved {
                    // Only the generic component-aware authority may decide
                    // an unresolved inverse chart without losing a finite
                    // contact. Fully rejected charts prove that this support
                    // contact lies outside the retained conic span.
                    return Ok(no_fast_path());
                }
                continue;
            };
            let Classification::Decided(point) =
                exact_contact_point_evidence(&rational_conic, &canonical_parameter, policy)?
            else {
                return Ok(no_fast_path());
            };
            let other_parameter =
                match RationalBezierOverlapParameterCorrespondence2::map_parameter_between_curves(
                    &rational_conic,
                    &authored_conic,
                    &canonical_parameter,
                    policy,
                )? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        return Ok(no_fast_path());
                    }
                };
            let derivative_scale_sign = match retained_parallel_range {
                Some(range) => match self.parallel_derivative_scale_sign_on_regular_range(
                    &parallel_parameter.clone().into(),
                    range,
                    policy,
                )? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => None,
                },
                None => None,
            };
            let tangent_cross_sign = match replay {
                Some(replay) => self
                    .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                        signed_bivariate_for_replay_or_parameter_box(
                            &tangent_cross,
                            &parallel_parameter,
                            &canonical_parameter,
                            replay,
                            &parameter_lifts,
                            policy,
                        )?,
                        &parallel_parameter,
                        derivative_scale_sign,
                        policy,
                    )?,
                None => None,
            };
            let circle_tangent_cross_sign = radial_crossing_sign.map(|sign| {
                if support.is_clockwise() {
                    match sign {
                        RealSign::Positive => RealSign::Negative,
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                    }
                } else {
                    sign
                }
            });
            let tangent_cross_sign = match (tangent_cross_sign, circle_tangent_cross_sign) {
                (Some(first), Some(second)) if first != second => {
                    return Err(CurveError::Topology(
                        "parallel/circle tangent orientation certificates disagree".into(),
                    ));
                }
                (Some(sign), _) | (None, Some(sign)) => Some(sign),
                (None, None) => None,
            };
            let tangent_dot_sign = match replay {
                Some(replay) => self
                    .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                        signed_bivariate_for_replay_or_parameter_box(
                            &tangent_dot,
                            &parallel_parameter,
                            &canonical_parameter,
                            replay,
                            &parameter_lifts,
                            policy,
                        )?,
                        &parallel_parameter,
                        derivative_scale_sign,
                        policy,
                    )?,
                None => None,
            };
            contacts.push(BezierParallelIntersectionContact2 {
                parallel_parameter,
                other_parameter,
                point,
                certified_transverse: matches!(
                    tangent_cross_sign,
                    Some(RealSign::Positive | RealSign::Negative)
                ),
                tangent_cross_sign,
                tangent_dot_sign,
            });
        }
        Ok(Classification::Decided(Some(
            BezierParallelIntersectionSet2::complete(contacts.into(), Arc::from([])),
        )))
    }

    /// Replays every resultant candidate into exact selected-branch contacts.
    ///
    /// Directly represented pairs and pairs with one isolated algebraic
    /// parameter are substituted into both original equations exactly. Pairs
    /// of algebraic parameters use univariate/identity/reversal fast paths,
    /// followed by Hypersolve's exact nullity-one Sylvester lift or one
    /// specialization-first common-fiber GCD and local-field Sturm count for
    /// genuinely coupled systems. Even-multiplicity roots and specialized
    /// degree drops are retained exactly. A degenerate resultant first
    /// delegates to the rational shared-component authority whenever zero
    /// distance or a certified Pythagorean hodograph supplies an exact
    /// rational parallel. Otherwise a primitive first subresultant may expose
    /// every extractable parameter component; rational maps retain their
    /// partitioned fast path, while implicit components are accepted only after
    /// exact closed-domain, critical-point, cell-orientation, singular-incidence,
    /// and selected-branch certification. Axis-wide and boundary-coincident
    /// factors are replayed as point-image parameter components rather than
    /// false curve overlaps; exactly extractable repeated implicit factors are
    /// reduced to one square-free geometric support. Both authored residual
    /// equations remain in the same recursive engine. Unsupported coefficient
    /// towers remain explicit
    /// [`BezierParallelIntersectionSet2::incomplete_candidates`] evidence; no
    /// projected root is promoted without exact replay.
    pub(crate) fn intersections(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        self.intersections_with_tangent_field(other, None, None, policy)
    }

    /// Intersects one retained regular source branch with a rational Bezier.
    ///
    /// When the authored hodograph has a common factor, the branch-oriented
    /// quotient supplies the same selected unit normal on the open retained
    /// range and its exact one-sided limit at a source cusp.  Projection,
    /// component extraction, selected-branch replay, and contact evidence all
    /// remain in the ordinary parallel/rational authority; only its tangent
    /// frame and contact derivative-orientation replay use the retained branch.
    /// The caller clips the returned full-parameter evidence to `range`.
    pub(crate) fn intersections_on_regular_range(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return self.intersections(other, policy);
        }
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let frame =
            match self.source_oriented_regularized_tangent_field_at_interior(&interior, &strict)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let Some(frame) = frame else {
            return self.intersections(other, policy);
        };
        self.intersections_with_tangent_field(other, Some(&frame), Some(range), policy)
    }

    pub(in crate::bezier_offset) fn intersections_with_tangent_field(
        &self,
        other: &RationalBezier2,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        retained_parallel_range: Option<&CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            let source = self.source().to_rational_bezier()?;
            if let Classification::Decided(intersections) =
                source.intersection_contacts_classified(other, policy)?
            {
                if matches!(
                    intersections,
                    RationalBezierIntersectionContacts2::Overlap(_)
                        | RationalBezierIntersectionContacts2::ContactsAndOverlap { .. }
                ) {
                    match self.replay_constant_parameter_components(
                        other,
                        [&CurveParameterRange2::unit(), &CurveParameterRange2::unit()],
                        policy,
                    )? {
                        Classification::Decided(Some(result)) => {
                            return Ok(Classification::Decided(result));
                        }
                        Classification::Decided(None) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                if !matches!(
                    intersections,
                    RationalBezierIntersectionContacts2::Incomplete { .. }
                        | RationalBezierIntersectionContacts2::DegenerateResultant
                ) {
                    return Ok(Classification::Decided(
                        parallel_set_from_rational_contacts(intersections),
                    ));
                }
            }
        }
        match self.rational_quadratic_circle_intersections_fast_path_with_tangent_field(
            other,
            tangent_field,
            retained_parallel_range,
            policy,
        )? {
            Classification::Decided(Some(intersections)) => {
                return Ok(Classification::Decided(intersections));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let candidate_system = match self.intersection_candidate_system_with_tangent_field(
            other,
            tangent_field,
            policy,
        )? {
            Classification::Decided(candidate_system) => candidate_system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.replay_parallel_rational_candidate_system(
            other,
            candidate_system,
            false,
            tangent_field,
            retained_parallel_range,
            |parameter| {
                crate::rational_bezier_general::exact_contact_point_evidence(
                    other, parameter, policy,
                )
            },
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::bezier_offset) fn replay_parallel_rational_candidate_system(
        &self,
        other: &RationalBezier2,
        candidate_system: BezierParallelIntersectionCandidateSystem2,
        off_diagonal: bool,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        retained_parallel_range: Option<&CurveParameterRange2>,
        mut point_evidence: impl FnMut(&BezierParameter2) -> CurveResult<Classification<CurvePoint2>>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        let BezierParallelIntersectionCandidateSystem2 {
            candidates,
            replay_equations,
            overlaps,
            component_overlaps,
            component_pairs,
            selected_component_pair_count,
        } = candidate_system;
        let (component_pairs, excluded_component_pairs) =
            component_pairs.split_at(selected_component_pair_count);
        let residual_degenerate = matches!(
            candidates,
            CurveIntersectionCandidates2::DegenerateResultant
        );
        let empty_parameters: &[BezierParameter2] = &[];
        let (parallel_parameters, other_parameters) = match &candidates {
            CurveIntersectionCandidates2::NoIntersection => {
                if component_pairs.is_empty() {
                    return Ok(Classification::Decided(
                        BezierParallelIntersectionSet2::complete_with_supplement(
                            Arc::from([]),
                            overlaps,
                            Arc::from([]),
                            component_overlaps,
                        ),
                    ));
                }
                (empty_parameters, empty_parameters)
            }
            CurveIntersectionCandidates2::DegenerateResultant => {
                if component_pairs.is_empty() {
                    // Replaying the original system would lose strict-zero exclusions.
                    if !overlaps.is_empty() || !excluded_component_pairs.is_empty() {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionSet2::incomplete(
                                Arc::from([]),
                                overlaps,
                                CurveIntersectionCandidates2::DegenerateResultant,
                            ),
                        ));
                    }
                    if tangent_field.is_some() {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionSet2::incomplete(
                                Arc::from([]),
                                overlaps,
                                CurveIntersectionCandidates2::DegenerateResultant,
                            ),
                        ));
                    }
                    return self.replay_degenerate_component(other, policy);
                }
                (empty_parameters, empty_parameters)
            }
            CurveIntersectionCandidates2::Candidates {
                first_parameters: parallel_parameters,
                second_parameters: other_parameters,
            } => (parallel_parameters.as_slice(), other_parameters.as_slice()),
        };

        let source = self.source_power_basis()?;
        let regularized_differential = tangent_field.map(|field| BezierParallelDifferential2 {
            tangent_derivative_x: polynomial_derivative(&field.x),
            tangent_derivative_y: polynomial_derivative(&field.y),
            tangent_x: field.x.clone(),
            tangent_y: field.y.clone(),
        });
        let differential = match regularized_differential.as_ref() {
            Some(differential) => differential,
            None => self.differential()?,
        };
        let other_power = other.homogeneous_power_basis()?;
        let [other_tangent_x, other_tangent_y] = rational_parametric_tangent_numerator(other_power);
        let tangent_cross = bivariate_subtract(
            &bivariate_outer_product(&differential.tangent_x, &other_tangent_y),
            &bivariate_outer_product(&differential.tangent_y, &other_tangent_x),
        );
        let tangent_dot = bivariate_add(
            &bivariate_outer_product(&differential.tangent_x, &other_tangent_x),
            &bivariate_outer_product(&differential.tangent_y, &other_tangent_y),
        );
        let [orthogonality, distance_relation] = replay_equations.unwrap_or_else(|| {
            let (orthogonality, distance_relation) = parallel_rational_intersection_equations(
                &source,
                differential,
                self.distance(),
                other_power,
            );
            [orthogonality, distance_relation]
        });
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let branch = parallel_rational_component_branch(
            &source,
            differential,
            self.distance(),
            other_power,
            distance_sign,
        );

        let mut contacts = Vec::new();
        let mut incomplete = residual_degenerate;
        let mut parameter_lifts: [Option<CurveIntersectionParameterLiftReport>; 2] = [None, None];
        let lift_config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for parallel_parameter in parallel_parameters {
            if let Some(range) = retained_parallel_range {
                match CurveParameterDomain2::new(range, None)
                    .contains_finite_parameter(&parallel_parameter.clone().into(), policy)?
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                }
            }
            for other_parameter in other_parameters {
                match parallel_parameter_pair_is_excluded(
                    excluded_component_pairs,
                    parallel_parameter,
                    other_parameter,
                    policy,
                )? {
                    Classification::Decided(true) => continue,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                }
                if matches!(
                    parallel_parameter_pair_is_overlap_boundary(
                        &overlaps,
                        parallel_parameter,
                        other_parameter,
                        policy,
                    )?,
                    Classification::Decided(true)
                ) {
                    continue;
                }
                let replay = match replay_bivariate_parameter_pair(
                    &orthogonality,
                    &distance_relation,
                    parallel_parameter,
                    other_parameter,
                    policy,
                    lift_config,
                    &mut parameter_lifts,
                )? {
                    Classification::Decided(replay) => replay,
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                };
                if replay == BivariateParameterPairReplay::Rejected {
                    continue;
                }
                {
                    let sign = signed_bivariate_for_replay_or_parameter_box(
                        &branch,
                        parallel_parameter,
                        other_parameter,
                        replay,
                        &parameter_lifts,
                        policy,
                    )?;
                    match sign {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Negative | RealSign::Zero) => continue,
                        Classification::Uncertain(_) => {
                            incomplete = true;
                            continue;
                        }
                    }
                }
                if off_diagonal {
                    match parallel_parameter.same_value(other_parameter, policy)? {
                        Classification::Decided(false) => {}
                        Classification::Decided(true) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let Classification::Decided(point) = point_evidence(other_parameter)? else {
                    incomplete = true;
                    continue;
                };
                let derivative_scale_sign = match retained_parallel_range {
                    Some(range) => match self.parallel_derivative_scale_sign_on_regular_range(
                        &parallel_parameter.clone().into(),
                        range,
                        policy,
                    )? {
                        Classification::Decided(sign) => Some(sign),
                        Classification::Uncertain(_) => None,
                    },
                    None => None,
                };
                let tangent_cross_sign = self
                    .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                        signed_bivariate_for_replay_or_parameter_box(
                            &tangent_cross,
                            parallel_parameter,
                            other_parameter,
                            replay,
                            &parameter_lifts,
                            policy,
                        )?,
                        parallel_parameter,
                        derivative_scale_sign,
                        policy,
                    )?;
                let tangent_dot_sign = self
                    .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                        signed_bivariate_for_replay_or_parameter_box(
                            &tangent_dot,
                            parallel_parameter,
                            other_parameter,
                            replay,
                            &parameter_lifts,
                            policy,
                        )?,
                        parallel_parameter,
                        derivative_scale_sign,
                        policy,
                    )?;
                let tangent_cross_sign = tangent_cross_sign.or_else(|| {
                    tangent_field.is_none().then(|| {
                        self.certified_transverse_contact_sign(
                            other,
                            parallel_parameter,
                            other_parameter,
                            policy,
                        )
                    })?
                });
                contacts.push(BezierParallelIntersectionContact2 {
                    parallel_parameter: parallel_parameter.clone(),
                    other_parameter: other_parameter.clone(),
                    point,
                    certified_transverse: matches!(
                        tangent_cross_sign,
                        Some(RealSign::Positive | RealSign::Negative)
                    ),
                    tangent_cross_sign,
                    tangent_dot_sign,
                });
            }
        }
        for pair in component_pairs.iter() {
            if let Some(range) = retained_parallel_range {
                match CurveParameterDomain2::new(range, None)
                    .contains_finite_parameter(&pair.parallel_parameter.clone().into(), policy)?
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                }
            }
            match parallel_contact_pair_is_retained(&contacts, pair, policy)? {
                Classification::Decided(true) => continue,
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => {
                    incomplete = true;
                    continue;
                }
            }
            let Classification::Decided(point) = point_evidence(&pair.other_parameter)? else {
                incomplete = true;
                continue;
            };
            let derivative_scale_sign = match retained_parallel_range {
                Some(range) => match self.parallel_derivative_scale_sign_on_regular_range(
                    &pair.parallel_parameter.clone().into(),
                    range,
                    policy,
                )? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => None,
                },
                None => None,
            };
            let tangent_cross_sign = self
                .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                    signed_bivariate_at_parameter_pair(
                        &tangent_cross,
                        &pair.parallel_parameter,
                        &pair.other_parameter,
                        policy,
                    )?,
                    &pair.parallel_parameter,
                    derivative_scale_sign,
                    policy,
                )?;
            let tangent_dot_sign = self
                .apply_parallel_derivative_scale_to_tangent_sign_with_override(
                    signed_bivariate_at_parameter_pair(
                        &tangent_dot,
                        &pair.parallel_parameter,
                        &pair.other_parameter,
                        policy,
                    )?,
                    &pair.parallel_parameter,
                    derivative_scale_sign,
                    policy,
                )?;
            let tangent_cross_sign = tangent_cross_sign.or_else(|| {
                tangent_field.is_none().then(|| {
                    self.certified_transverse_contact_sign(
                        other,
                        &pair.parallel_parameter,
                        &pair.other_parameter,
                        policy,
                    )
                })?
            });
            contacts.push(BezierParallelIntersectionContact2 {
                parallel_parameter: pair.parallel_parameter.clone(),
                other_parameter: pair.other_parameter.clone(),
                point,
                certified_transverse: matches!(
                    tangent_cross_sign,
                    Some(RealSign::Positive | RealSign::Negative)
                ),
                tangent_cross_sign,
                tangent_dot_sign,
            });
        }
        let contacts: Arc<[BezierParallelIntersectionContact2]> = contacts.into();
        if incomplete {
            return Ok(Classification::Decided(
                BezierParallelIntersectionSet2::incomplete(contacts, overlaps, candidates),
            ));
        }
        Ok(Classification::Decided(
            BezierParallelIntersectionSet2::complete_with_supplement(
                contacts,
                overlaps,
                Arc::from([]),
                component_overlaps,
            ),
        ))
    }

    pub(in crate::bezier_offset) fn replay_degenerate_component(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        match self.replay_constant_parameter_components(
            other,
            [&CurveParameterRange2::unit(), &CurveParameterRange2::unit()],
            policy,
        )? {
            Classification::Decided(Some(result)) => {
                return Ok(Classification::Decided(result));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let exact_parallel = match self.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(exact_parallel)) => Some(exact_parallel),
            Classification::Decided(None) => None,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let Some(exact_parallel) = exact_parallel else {
            return Ok(Classification::Decided(
                BezierParallelIntersectionSet2::incomplete(
                    Arc::from([]),
                    Arc::from([]),
                    CurveIntersectionCandidates2::DegenerateResultant,
                ),
            ));
        };
        let replay = match exact_parallel.intersection_contacts_classified(other, policy)? {
            Classification::Decided(replay) => replay,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            parallel_set_from_rational_contacts(replay),
        ))
    }

    /// Replays axis-wide common factors as parameter components with point image.
    ///
    /// Under the certified finite parallel equations, fixing one parameter while
    /// leaving the other arbitrary forces the arbitrary operand to be constant:
    /// the orthogonality relation confines it to one normal line and the signed
    /// distance relation confines it to one selected point on that line. This
    /// geometric replay is both cheaper and more informative than treating the
    /// corresponding axis factor as a positive-length image overlap.
    pub(in crate::bezier_offset) fn replay_constant_parameter_components(
        &self,
        other: &RationalBezier2,
        ranges: [&CurveParameterRange2; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelIntersectionSet2>>> {
        let interior = match ranges[1].strict_interior_scalar(policy)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        let other_point = match other.point_at_affine_classified(&interior, policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        match other.point_incidence_on_range(&other_point, ranges[1], policy)? {
            Classification::Decided(crate::RationalBezierPointIncidence2::EntireCurve) => {
                let incidence = match self.point_incidence(&other_point, ranges[0], policy)? {
                    Classification::Decided(incidence) => incidence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let components = match incidence {
                    BezierParallelIncidence2::EntireCurve => Arc::from([
                        BezierParallelIntersectionParameterComponent2::entire_parameter_square(
                            other_point,
                        ),
                    ]),
                    BezierParallelIncidence2::Parameters(parameters) => parameters
                        .into_iter()
                        .map(|parameter| {
                            BezierParallelIntersectionParameterComponent2::fixed_parallel_parameter(
                                parameter,
                                other_point.clone(),
                            )
                        })
                        .collect(),
                };
                return Ok(Classification::Decided(Some(
                    BezierParallelIntersectionSet2::complete_parameter_components(components),
                )));
            }
            Classification::Decided(crate::RationalBezierPointIncidence2::Parameters(_)) => {}
            // This is only a constant-curve probe. Preserve the established
            // exact-rational/component replay when constancy is not decidable.
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        }

        let interior = match ranges[0].strict_interior_scalar(policy)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        let parallel_point = match self.point_at_with_policy(&interior, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        match self.point_incidence(&parallel_point, ranges[0], policy)? {
            Classification::Decided(BezierParallelIncidence2::Parameters(_)) => {
                Ok(Classification::Decided(None))
            }
            Classification::Decided(BezierParallelIncidence2::EntireCurve) => {
                let incidence =
                    match other.point_incidence_on_range(&parallel_point, ranges[1], policy)? {
                        Classification::Decided(incidence) => incidence,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let components = match incidence {
                    crate::RationalBezierPointIncidence2::EntireCurve => Arc::from([
                        BezierParallelIntersectionParameterComponent2::entire_parameter_square(
                            parallel_point,
                        ),
                    ]),
                    crate::RationalBezierPointIncidence2::Parameters(parameters) => parameters
                        .into_iter()
                        .map(|parameter| {
                            BezierParallelIntersectionParameterComponent2::fixed_other_parameter(
                                parameter,
                                parallel_point.clone(),
                            )
                        })
                        .collect(),
                };
                Ok(Classification::Decided(Some(
                    BezierParallelIntersectionSet2::complete_parameter_components(components),
                )))
            }
            // As above, undecidable constancy falls through. Incidence after
            // `EntireCurve` is proved remains authoritative and may be uncertain.
            Classification::Uncertain(_) => Ok(Classification::Decided(None)),
        }
    }

    /// Selects rational parallel images on exact finite/ray domains. Callers
    /// share one PH proof and construct its opposite sheet only once.
    /// A zero displacement remains its source without requiring a normal.
    pub(in crate::bezier_offset) fn rational_parallel_components_in_domains<const N: usize>(
        &self,
        domains: [CurveParameterDomain2<'_>; N],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[RationalBezier2; N]>>> {
        policy.strict_predicate_pass(|| {
            match real_sign(self.distance(), policy) {
                Some(RealSign::Zero) => {
                    let source = self.source().to_rational_bezier()?;
                    return Ok(Classification::Decided(Some(std::array::from_fn(|_| {
                        source.clone()
                    }))));
                }
                Some(RealSign::Positive | RealSign::Negative) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let offset = match self.exact_pythagorean_hodograph_offset_with_policy(policy)? {
                Classification::Decided(Some(offset)) => offset,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let mut opposite = None;
            let mut curves = std::array::from_fn(|_| None);
            for (domain, curve) in domains.into_iter().zip(&mut curves) {
                let Some(sign) = offset.speed_sign_on_domain(domain, policy)? else {
                    return Ok(Classification::Decided(None));
                };
                *curve = Some(match sign {
                    RealSign::Positive => offset.curve().clone(),
                    RealSign::Negative => {
                        if opposite.is_none() {
                            opposite = match self
                                .with_distance(-self.distance())
                                .exact_pythagorean_hodograph_offset_with_policy(policy)?
                            {
                                Classification::Decided(Some(offset)) => Some(offset.curve),
                                Classification::Decided(None) => {
                                    return Ok(Classification::Decided(None));
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                        }
                        opposite
                            .as_ref()
                            .expect("the opposite sheet was constructed")
                            .clone()
                    }
                    RealSign::Zero => unreachable!("a certified speed sheet has a strict sign"),
                });
            }
            Ok(Classification::Decided(Some(curves.map(|curve| {
                curve.expect("every domain selected its rational sheet")
            }))))
        })
    }

    /// Returns the rational image certified on the authored unit interval.
    pub(crate) fn exact_rational_parallel_component(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        match real_sign(self.distance(), policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(Some(
                self.source().to_rational_bezier()?,
            ))),
            Some(RealSign::Positive | RealSign::Negative) => {
                if let Some(cached) = self.data.certified_ph_offset.get() {
                    return Ok(Classification::Decided(
                        cached.as_deref().map(|offset| offset.curve().clone()),
                    ));
                }
                Ok(Classification::Decided(
                    match self.exact_pythagorean_hodograph_offset_with_policy(policy)? {
                        Classification::Decided(Some(offset)) => Some(offset.curve().clone()),
                        Classification::Decided(None) | Classification::Uncertain(_) => None,
                    },
                ))
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    /// Materializes the exact parallel image selected by one regular source range.
    ///
    /// A globally singular PH source has no single polynomial speed sheet: its
    /// authored unit normal can reverse across a stationary parameter. Exact
    /// hodograph-GCD cancellation nevertheless leaves one primitive oriented
    /// tangent field on each regular side. When that quotient field is itself
    /// Pythagorean, the ordinary rational component authority can publish the
    /// selected branch without fitting or approximately selecting a curve.
    pub(crate) fn exact_rational_parallel_component_on_regular_range(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelRationalComponent2>>> {
        policy.strict_predicate_pass(|| {
            let global = self
                .rational_parallel_components_in_domains(
                    [CurveParameterDomain2::new(range, None)],
                    policy,
                )?
                .map(|curves| curves.map(|[curve]| curve));
            if let Classification::Decided(Some(curve)) = &global {
                return Ok(Classification::Decided(Some(
                    BezierParallelRationalComponent2 {
                        curve: curve.clone(),
                        support_line: None,
                        regular_range: range.clone(),
                    },
                )));
            }
            let global = match global {
                Classification::Decided(None) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
                Classification::Decided(Some(_)) => unreachable!("the exact component returned"),
            };
            let interior = match range.strict_interior_scalar(policy)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (tangent_x, tangent_y) = match self
                .source_oriented_regularized_tangent_field_at_interior(&interior, policy)?
            {
                Classification::Decided(Some(frame)) => (frame.x.clone(), frame.y.clone()),
                Classification::Decided(None) => {
                    // A regular hodograph needs no GCD cancellation. Its
                    // native PH admission may have failed only at an unused
                    // source pole; prove its speed on this consumed range.
                    let differential = self.differential()?;
                    (
                        differential.tangent_x.clone(),
                        differential.tangent_y.clone(),
                    )
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_x = polynomial_trim_structural_zeros(tangent_x);
            let tangent_y = polynomial_trim_structural_zeros(tangent_y);
            let speed = match certify_ph_speed_on_range(
                &tangent_x, &tangent_y, &interior, range, policy,
            )? {
                Classification::Decided(Some(speed)) => speed,
                Classification::Decided(None) => return Ok(global),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let curve = match self.compute_pythagorean_hodograph_offset_from_tangent_field(
                &tangent_x, &tangent_y, &speed, range, false, policy,
            )? {
                Classification::Decided(Some(offset)) => offset,
                Classification::Decided(None) => return Ok(global),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "analytic-parallel-rational-component",
                "regularized-pythagorean-hodograph",
            );
            let support_line = match (tangent_x.as_slice(), tangent_y.as_slice()) {
                ([tangent_x], [tangent_y]) => {
                    let anchor = match curve.point_at_affine_classified(&interior, policy) {
                        Classification::Decided(anchor) => anchor,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    Some(LineSeg2::try_new(
                        anchor.clone(),
                        anchor.translated(tangent_x.clone(), tangent_y.clone()),
                    )?)
                }
                _ => None,
            };
            Ok(Classification::Decided(Some(
                BezierParallelRationalComponent2 {
                    curve,
                    support_line,
                    regular_range: range.clone(),
                },
            )))
        })
    }
}
