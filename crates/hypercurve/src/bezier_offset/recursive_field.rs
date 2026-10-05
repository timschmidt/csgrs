//! Recursive quadratic-field towers: projective and monotone parameters, circle-target and chord-parallel systems for nested square-root offset carriers.

use super::*;

impl BezierRecursiveCircleFrame2 {
    /// Imports a center and radial anchor together, retaining their shared
    /// source roots and positive generators before publishing the frame.
    pub(super) fn from_point_evidence(
        center: &CurvePoint2,
        support_center: &CurvePoint2,
        normal_denominator: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let points = match recursive_projective_evidence_points(&[center, support_center], policy)?
        {
            Classification::Decided(Some(points)) => points,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let [center, support_center]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
            .try_into()
            .expect("a circle frame retains its center and radial anchor");
        let center = match positive_recursive_projective_point(center)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let support_center = match positive_recursive_projective_point(support_center)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(Self {
            field: center.denominator.field(),
            center,
            support_center,
            normal_denominator,
        })))
    }

    pub(super) fn lifted_to(&self, field: &RecursiveQuadraticField) -> Option<Self> {
        Some(Self {
            field: field.clone(),
            center: self.center.lifted_to(field)?,
            support_center: self.support_center.lifted_to(field)?,
            normal_denominator: self.normal_denominator.clone(),
        })
    }
}

impl BezierRecursiveCircleTargetSystem2 {
    pub(super) fn expression_polynomial(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
    ) -> Option<Vec<RecursiveQuadraticValue>> {
        if self.unit_target_speed {
            recursive_quadratic_polynomial_combine(&expression.rational, &expression.radical, false)
        } else {
            expression.squared_magnitude_difference().map(<[_]>::to_vec)
        }
    }

    pub(super) fn projected_polynomial(
        &self,
        coefficients: &[RecursiveQuadraticValue],
    ) -> Option<DenseTensorPolynomial> {
        let (base, projection) = recursive_quadratic_polynomial_projection(coefficients.to_vec())?;
        (Arc::ptr_eq(&base, &self.base)
            || (base.sources == self.base.sources
                && base.first_speed_squared == self.base.first_speed_squared
                && base.second_speed_squared == self.base.second_speed_squared))
            .then_some(projection)
    }

    pub(super) fn expression_projection(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
    ) -> Option<DenseTensorPolynomial> {
        let coefficients = self.expression_polynomial(expression)?;
        self.projected_polynomial(&coefficients)
    }

    pub(super) fn diameter_parameter_expression(
        &self,
        denominator: &Real,
        radial_coefficient: &Real,
    ) -> Option<BezierRecursiveQuadraticParallelExpression2> {
        let combine = |diameter: &[RecursiveQuadraticValue], radius: &[RecursiveQuadraticValue]| {
            let diameter = recursive_quadratic_polynomial_scale_real(diameter, denominator)?;
            let radius = recursive_quadratic_polynomial_scale_real(radius, radial_coefficient)?;
            recursive_quadratic_polynomial_combine(&diameter, &radius, true)
        };
        Some(BezierRecursiveQuadraticParallelExpression2::new(
            combine(
                &self.diameter.rational,
                &self.radius_squared_denominator.rational,
            )?,
            combine(
                &self.diameter.radical,
                &self.radius_squared_denominator.radical,
            )?,
            self.circle.speed_squared.clone(),
        ))
    }

    pub(super) fn diameter_parameter_projection(
        &self,
        denominator: &Real,
        radial_coefficient: &Real,
    ) -> Option<DenseTensorPolynomial> {
        self.expression_projection(
            &self.diameter_parameter_expression(denominator, radial_coefficient)?,
        )
    }

    pub(super) fn expression_root_by_interval(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        target_parameter: &BezierParameter2,
    ) -> Option<bool> {
        recursive_quadratic_parallel_expression_root_by_interval(
            expression,
            self.unit_target_speed,
            target_parameter,
        )
    }

    pub(super) fn expression_interval_sign(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        target_parameter: &BezierParameter2,
    ) -> Option<RealSign> {
        recursive_quadratic_parameter_interval_sign(
            target_parameter,
            |target, source_steps, coefficient_precision| {
                recursive_quadratic_parallel_expression_interval(
                    expression,
                    self.unit_target_speed,
                    target,
                    source_steps,
                    coefficient_precision,
                )
            },
        )
    }

    pub(super) fn polynomial_interval_sign(
        &self,
        polynomial: &[RecursiveQuadraticValue],
        target_parameter: &BezierParameter2,
    ) -> Option<RealSign> {
        recursive_quadratic_parameter_interval_sign(
            target_parameter,
            |target, source_steps, coefficient_precision| {
                recursive_quadratic_polynomial_interval(
                    polynomial,
                    target,
                    source_steps,
                    coefficient_precision,
                )
            },
        )
    }

    /// Returns `Some(None)` for a certified point on the excluded half,
    /// `Some(Some(location))` for a certified retained location, and `None`
    /// when exact recursive-field replay remains necessary.
    pub(super) fn contact_location_by_interval(
        &self,
        target_parameter: &BezierParameter2,
    ) -> Option<Option<BezierAlgebraicCuspSemicircleContactLocation2>> {
        match self.expression_interval_sign(&self.selected_half_plane, target_parameter)? {
            RealSign::Negative => Some(None),
            RealSign::Positive => Some(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            RealSign::Zero => {
                match self.expression_interval_sign(&self.diameter, target_parameter)? {
                    RealSign::Positive => {
                        Some(Some(BezierAlgebraicCuspSemicircleContactLocation2::Start))
                    }
                    RealSign::Negative => {
                        Some(Some(BezierAlgebraicCuspSemicircleContactLocation2::End))
                    }
                    RealSign::Zero => None,
                }
            }
        }
    }

    pub(super) fn candidate_evaluation(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticParallelEvaluation2>>> {
        recursive_quadratic_parallel_candidate_evaluation(
            &self.field,
            &self.base,
            &self.weight,
            &self.circle.speed_squared,
            self.unit_target_speed,
            target_parameter,
            policy,
        )
    }

    pub(super) fn expression_sign_with_evaluation(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let value = evaluation.expression_value(expression).ok_or_else(|| {
            CurveError::Topology(
                "a recursive circle/parallel expression exceeded its field budget".into(),
            )
        })?;
        value.sign(policy)
    }

    pub(super) fn expression_sign_at_parameter(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let evaluation = match self.candidate_evaluation(target_parameter, policy)? {
            Classification::Decided(Some(evaluation)) => evaluation,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.expression_sign_with_evaluation(expression, &evaluation, policy)
    }

    pub(super) fn polynomial_sign_with_evaluation(
        &self,
        polynomial: &[RecursiveQuadraticValue],
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let value = evaluation.polynomial_value(polynomial).ok_or_else(|| {
            CurveError::Topology(
                "a recursive circle/parallel polynomial exceeded its field budget".into(),
            )
        })?;
        value.sign(policy)
    }

    pub(super) fn polynomial_sign_at_region_parameter(
        &self,
        polynomial: &[RecursiveQuadraticValue],
        target_parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if target_parameter.as_recursive_projective().is_some() {
            match policy.strict_predicate_pass(|| {
                recursive_projective_polynomial_sign_at_parameter(
                    &self.field,
                    &self.weight,
                    target_parameter,
                    policy,
                )
            })? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            return recursive_projective_polynomial_sign_at_parameter(
                &self.field,
                polynomial,
                target_parameter,
                policy,
            );
        }
        let Some(parameter) = target_parameter.as_bezier_parameter() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let evaluation = match self.candidate_evaluation(parameter, policy)? {
            Classification::Decided(Some(evaluation)) => evaluation,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.polynomial_sign_with_evaluation(polynomial, &evaluation, policy)
    }

    pub(super) fn expression_sign_at_region_parameter(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        target_parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if self.unit_target_speed {
            let Some(polynomial) = self.expression_polynomial(expression) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return self.polynomial_sign_at_region_parameter(&polynomial, target_parameter, policy);
        }
        if let Some(parameter) = target_parameter.as_bezier_parameter() {
            return self.expression_sign_at_parameter(expression, parameter, policy);
        }
        expression.sign_with_positive_speed(policy, |polynomial| {
            self.polynomial_sign_at_region_parameter(polynomial, target_parameter, policy)
        })
    }

    pub(super) fn contact_location_at_region_parameter(
        &self,
        target_parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let selected = match self.expression_sign_at_region_parameter(
            &self.selected_half_plane,
            target_parameter,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(match selected {
            RealSign::Negative => Classification::Decided(None),
            RealSign::Positive => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            RealSign::Zero => match self.expression_sign_at_region_parameter(
                &self.diameter,
                target_parameter,
                policy,
            )? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleContactLocation2::Start,
                )),
                Classification::Decided(RealSign::Negative) => Classification::Decided(Some(
                    BezierAlgebraicCuspSemicircleContactLocation2::End,
                )),
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a recursive circle/rational contact had zero diameter side".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        })
    }

    pub(super) fn expression_parameters(
        &self,
        polynomial: &DenseTensorPolynomial,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        policy.strict_predicate_pass(|| {
            selected_dense_last_axis_parameters(polynomial, &self.base.sources, domain, policy)
        })
    }

    pub(super) fn retained_expression_parameters(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        let Some(projection) = self.expression_projection(expression) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match self.expression_parameters(&projection, domain, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(projection) => {
                return Ok(Classification::Decided(projection));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            match policy.strict_predicate_pass(|| {
                self.expression_sign_at_parameter(expression, &candidate, policy)
            })? {
                Classification::Decided(RealSign::Zero) => retained.push(candidate),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(
            BezierAlgebraicFiberProjection2::Parameters(retained),
        ))
    }

    pub(super) fn parameters_with_incident_domain(
        &self,
        polynomial: &DenseTensorPolynomial,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        policy.strict_predicate_pass(|| {
            selected_dense_last_axis_parameters_with_incident_domain(
                polynomial,
                &self.base.sources,
                domain,
                policy,
            )
        })
    }

    pub(super) fn incidence_parameters_with_incident_domain(
        &self,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        if let Some(fast_path) = self.direct_pair_fast_path.as_ref() {
            return fast_path.parameters_with_incident_domain(domain, policy);
        }
        let Some(projection) = self.projection.as_ref() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        policy.strict_predicate_pass(|| {
            let univariate = if let Some(univariate) = self.incidence_univariate.get() {
                univariate
            } else {
                let univariate = match selected_dense_last_axis_univariate(
                    projection,
                    &self.base.sources,
                    policy,
                )? {
                    Classification::Decided(univariate) => univariate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let _ = self.incidence_univariate.set(univariate);
                self.incidence_univariate
                    .get()
                    .expect("a selected-fiber univariate was just retained")
            };
            selected_dense_last_axis_univariate_parameters_with_incident_domain(
                univariate, domain, policy,
            )
        })
    }

    pub(super) fn contact_location_with_evaluation(
        &self,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let selected = match self.expression_sign_with_evaluation(
            &self.selected_half_plane,
            evaluation,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(match selected {
            RealSign::Negative => Classification::Decided(None),
            RealSign::Positive => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            RealSign::Zero => {
                match self.expression_sign_with_evaluation(&self.diameter, evaluation, policy)? {
                    Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    )),
                    Classification::Decided(RealSign::Negative) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::End,
                    )),
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a recursive circle/parallel contact had zero diameter side".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        })
    }

    pub(super) fn tangent_cross_dot_source_expression(
        &self,
        cross_scale: &Real,
        dot_scale: &Real,
    ) -> Option<BezierRecursiveQuadraticParallelExpression2> {
        // The stored dot expression includes one positive target-speed
        // factor. Apply that same factor to cross before combining them.
        Some(BezierRecursiveQuadraticParallelExpression2::new(
            recursive_quadratic_polynomial_scale_real(
                &self.tangent_dot_source.rational,
                dot_scale,
            )?,
            recursive_quadratic_polynomial_combine(
                &recursive_quadratic_polynomial_scale_real(
                    &self.tangent_cross_source,
                    cross_scale,
                )?,
                &recursive_quadratic_polynomial_scale_real(
                    &self.tangent_dot_source.radical,
                    dot_scale,
                )?,
                false,
            )?,
            self.circle.speed_squared.clone(),
        ))
    }

    pub(super) fn tangent_cross_dot_source_sign(
        &self,
        target_parameter: &CurveParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(parameter) = target_parameter.as_bezier_parameter()
            && let Some(fast_path) = self.direct_pair_fast_path.as_ref()
            && let Classification::Decided(sign) = fast_path.tangent_cross_dot_source_sign(
                parameter,
                cross_scale,
                dot_scale,
                policy,
            )?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-parallel-map-predicate",
                "direct-pair-tangent",
            );
            return Ok(Classification::Decided(sign));
        }
        let Some(expression) = self.tangent_cross_dot_source_expression(cross_scale, dot_scale)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if let Some(parameter) = target_parameter.as_bezier_parameter()
            && let Some(sign) = self.expression_interval_sign(&expression, parameter)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-parallel-map-predicate",
                "recursive-tangent-interval",
            );
            return Ok(Classification::Decided(sign));
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-circle-parallel-map-predicate",
            "recursive-tangent",
        );
        self.expression_sign_at_region_parameter(&expression, target_parameter, policy)
    }

    pub(super) fn diameter_parameter_sign(
        &self,
        target_parameter: &CurveParameter2,
        denominator: &Real,
        radial_coefficient: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(parameter) = target_parameter.as_bezier_parameter()
            && let Some(fast_path) = self.direct_pair_fast_path.as_ref()
            && let Classification::Decided(sign) = fast_path.diameter_parameter_sign(
                parameter,
                denominator,
                radial_coefficient,
                policy,
            )?
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-parallel-map-predicate",
                "direct-pair-diameter",
            );
            return Ok(Classification::Decided(sign));
        }
        let Some(expression) = self.diameter_parameter_expression(denominator, radial_coefficient)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if let Some(parameter) = target_parameter.as_bezier_parameter()
            && let Some(sign) = self.expression_interval_sign(&expression, parameter)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-circle-parallel-map-predicate",
                "recursive-diameter-interval",
            );
            return Ok(Classification::Decided(sign));
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-circle-parallel-map-predicate",
            "recursive-diameter",
        );
        self.expression_sign_at_region_parameter(&expression, target_parameter, policy)
    }
}

impl BezierRecursiveFixedDistanceSystem2 {
    pub(super) fn projected_polynomial(
        &self,
        coefficients: &[RecursiveQuadraticValue],
    ) -> Option<DenseTensorPolynomial> {
        let (base, projection) = recursive_quadratic_polynomial_projection(coefficients.to_vec())?;
        recursive_quadratic_bases_equivalent(&base, &self.base).then_some(projection)
    }

    pub(super) fn parameters(
        &self,
        projection: &DenseTensorPolynomial,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        policy.strict_predicate_pass(|| {
            selected_dense_last_axis_parameters(projection, &self.base.sources, domain, policy)
        })
    }

    pub(super) fn candidate_evaluation(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticParallelEvaluation2>>> {
        recursive_quadratic_parallel_candidate_evaluation(
            &self.field,
            &self.base,
            &self.source_weight,
            &self.incidence.speed_squared,
            self.unit_target_speed,
            target_parameter,
            policy,
        )
    }

    pub(super) fn expression_sign(
        &self,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        evaluation
            .expression_value(&self.incidence)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive fixed-distance replay exceeded its retained field budget".into(),
                )
            })?
            .sign(policy)
    }

    pub(super) fn expression_root_by_interval(
        &self,
        target_parameter: &BezierParameter2,
    ) -> Option<bool> {
        recursive_quadratic_parallel_expression_root_by_interval(
            &self.incidence,
            self.unit_target_speed,
            target_parameter,
        )
    }
}

impl BezierRecursiveLineParameterIdentity2 {
    pub(super) fn same_parameterization(&self, other: &Self) -> bool {
        if !Arc::ptr_eq(&self.source_frame, &other.source_frame)
            || self.source_clockwise != other.source_clockwise
            || compare_reals(
                &self.source_radial_distance,
                &other.source_radial_distance,
                &CurveContext::STRICT,
            ) != Some(std::cmp::Ordering::Equal)
        {
            return false;
        }
        let same_point = |first: &Point2, second: &Point2| {
            compare_reals(first.x(), second.x(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(first.y(), second.y(), &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal)
        };
        let same_line = |first: &LineSeg2, second: &LineSeg2| {
            same_point(first.start(), second.start()) && same_point(first.end(), second.end())
        };
        let transformed_line = |source: &LineSeg2, transform: &Similarity2, target: &LineSeg2| {
            same_point(
                &source.start().transform_similarity(transform),
                target.start(),
            ) && same_point(&source.end().transform_similarity(transform), target.end())
        };
        match (&self.transform, &other.transform) {
            (None, None) => same_line(&self.line, &other.line),
            (Some(first), Some(second)) if first == second => same_line(&self.line, &other.line),
            (None, Some(transform)) => transformed_line(&self.line, transform, &other.line),
            (Some(transform), None) => transformed_line(&other.line, transform, &self.line),
            (Some(_), Some(_)) => false,
        }
    }
}

impl BezierRecursiveMonotoneParameter2 {
    /// Decides whether a real polynomial vanishes at this mapped monotone
    /// root without bracketing it further.
    ///
    /// The source root is the unique zero of the authored incidence
    /// `f = a + b*sqrt(S)` in its bracket, and a root of `R = a^2 - b^2*S`
    /// over the retained field. A strict interval sign of the conjugate sheet
    /// `a - b*sqrt(S)` on the bracket proves `R` has no other root there, so
    /// the gcd of `R` and the query composed with the projective map decides
    /// vanishing exactly. `None` leaves the comparison to its fallbacks.
    pub(super) fn polynomial_vanishes(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        let Some(degree) = coefficients
            .iter()
            .rposition(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        else {
            return Ok(Some(true));
        };
        let coefficients = &coefficients[..=degree];
        if let Some(source) = self.exact_source_value() {
            let mapped = self.map_value(source)?;
            return Ok(real_sign(
                &Real::eval_poly(coefficients, &mapped),
                &policy.strict_counterpart(),
            )
            .map(|sign| sign == RealSign::Zero));
        }
        let Some(defining) = self.system.incidence_polynomial() else {
            return Ok(None);
        };
        let interval = RealInterval {
            lower: self.source_lower.clone(),
            upper: self.source_upper.clone(),
        };
        if !self
            .system
            .incidence
            .radical
            .iter()
            .all(RecursiveQuadraticValue::is_structurally_zero)
        {
            let conjugate_is_separated = [(0_usize, -128_i32), (8, -256), (32, -512)]
                .into_iter()
                .any(|(steps, precision)| {
                    matches!(
                        self.system
                            .conjugate_incidence_interval_sign(&interval, steps, precision),
                        Some(RealSign::Positive | RealSign::Negative)
                    )
                });
            if !conjugate_is_separated {
                return Ok(None);
            }
        }
        // Q(t) = d(t)^n P(n(t)/d(t)) for the projective map n/d.
        let numerator = [self.map_numerator[0].clone(), self.map_numerator[1].clone()];
        let denominator = [
            self.map_denominator[0].clone(),
            self.map_denominator[1].clone(),
        ];
        let mut composed = vec![Real::zero()];
        for (power, coefficient) in coefficients.iter().enumerate() {
            let term = polynomial_scale(
                &polynomial_multiply(
                    &polynomial_power(&numerator, power),
                    &polynomial_power(&denominator, degree - power),
                ),
                coefficient,
            );
            composed = polynomial_add(&composed, &term);
        }
        let Some(query) = recursive_quadratic_real_polynomial(&self.system.field, &composed) else {
            return Ok(None);
        };
        let mut context = RecursiveQuadraticOrderedFieldContext {
            field: self.system.field.clone(),
            policy: *policy,
        };
        match hypersolve::ordered_field_vanishes_at_selected_root(
            defining,
            &query,
            &hypersolve::IsolatedRootInterval {
                lower: interval.lower,
                upper: interval.upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            &mut context,
        ) {
            Ok(vanishes) => Ok(vanishes),
            Err(RecursiveQuadraticOrderedFieldError::Uncertain) => Ok(None),
            Err(RecursiveQuadraticOrderedFieldError::Context(error)) => Err(error),
        }
    }

    pub(super) fn exact_source_value(&self) -> Option<&Real> {
        (self.source_lower_sign == RealSign::Zero && self.source_upper_sign == RealSign::Zero)
            .then_some(&self.source_lower)
    }

    pub(super) fn map_value(&self, source: &Real) -> CurveResult<Real> {
        let numerator = &self.map_numerator[0] + &self.map_numerator[1] * source;
        let denominator = &self.map_denominator[0] + &self.map_denominator[1] * source;
        Ok((numerator / denominator)?)
    }

    pub(super) fn map_derivative_sign(&self) -> Option<RealSign> {
        real_sign(
            &Real::diff_of_products(
                &self.map_numerator[1],
                &self.map_denominator[0],
                &self.map_numerator[0],
                &self.map_denominator[1],
            ),
            &CurveContext::STRICT,
        )
    }

    pub(super) fn mapped_bounds(&self) -> CurveResult<Option<(Real, Real)>> {
        let first = self.map_value(&self.source_lower)?;
        if self.exact_source_value().is_some() {
            return Ok(Some((first.clone(), first)));
        }
        let second = self.map_value(&self.source_upper)?;
        Ok(match self.map_derivative_sign() {
            Some(RealSign::Positive) => Some((first, second)),
            Some(RealSign::Negative) => Some((second, first)),
            Some(RealSign::Zero) | None => None,
        })
    }

    /// Lazily prepares one coefficient enclosure shared by every clone of
    /// this monotone root. Refining the selected coefficient-field roots is
    /// substantially more expensive than bisecting the target parameter, so
    /// it must happen once rather than once per coefficient and midpoint.
    pub(super) fn prepared_interval_system(
        &self,
    ) -> Option<Arc<BezierRecursiveProjectiveChordParallelIntervalSystem2>> {
        const SOURCE_REFINEMENT_STEPS: usize = 1664;
        const COEFFICIENT_PRECISION: i32 = -1920;

        {
            let mut cache = self
                .refinement_cache
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if let Some(system) = cache.interval_system.as_ref() {
                return Some(Arc::clone(system));
            }
            if cache.interval_system_attempted {
                return None;
            }
            cache.interval_system_attempted = true;
        }

        if std::env::var_os("HYPERCURVE_DEBUG_MONOTONE_INTERVAL").is_some() {
            eprintln!(
                "preparing monotone interval system at target step {}",
                self.source_refinement_steps
            );
        }
        let prepared = self
            .system
            .prepare_incidence_interval_system(SOURCE_REFINEMENT_STEPS, COEFFICIENT_PRECISION);
        if std::env::var_os("HYPERCURVE_DEBUG_MONOTONE_INTERVAL").is_some() {
            eprintln!("prepared monotone interval system={}", prepared.is_some());
        }
        let prepared = prepared?;
        let prepared = Arc::new(prepared);
        let mut cache = self
            .refinement_cache
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let retained = cache
            .interval_system
            .get_or_insert_with(|| Arc::clone(&prepared));
        Some(Arc::clone(retained))
    }

    pub(super) fn refine_once(&mut self, policy: &CurveContext) -> CurveResult<Classification<()>> {
        if self.exact_source_value().is_some() {
            return Ok(Classification::Decided(()));
        }
        let midpoint = ((&self.source_lower + &self.source_upper) / Real::from(2_i8))?;
        let interval_system = self.prepared_interval_system();
        let target = interval_system
            .as_ref()
            .and_then(|system| system.parameter_interval(&midpoint))
            .unwrap_or_else(|| RealInterval {
                lower: midpoint.clone(),
                upper: midpoint.clone(),
            });
        let interval_sign = interval_system
            .as_ref()
            .and_then(|system| system.incidence_sign(&target));
        if std::env::var_os("HYPERCURVE_DEBUG_MONOTONE_INTERVAL").is_some()
            && interval_system.is_some()
            && interval_sign.is_none()
        {
            eprintln!(
                "prepared monotone interval declined target step {}",
                self.source_refinement_steps
            );
        }
        let sign = match interval_sign.map_or_else(
            || {
                policy
                    .strict_predicate_pass(|| self.system.incidence_sign_at_real(&midpoint, policy))
            },
            |sign| {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-monotone-refinement",
                    "prepared-incidence-interval",
                );
                Ok(Classification::Decided(sign))
            },
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match sign {
            RealSign::Zero => {
                self.source_lower = midpoint.clone();
                self.source_upper = midpoint;
                self.source_lower_sign = RealSign::Zero;
                self.source_upper_sign = RealSign::Zero;
            }
            sign if sign == self.source_lower_sign => {
                self.source_lower = midpoint;
            }
            sign if sign == self.source_upper_sign => {
                self.source_upper = midpoint;
            }
            _ => {
                return Err(CurveError::Topology(
                    "a monotone recursive root left its opposite-sign bracket".into(),
                ));
            }
        }
        self.source_refinement_steps = self
            .source_refinement_steps
            .checked_add(1)
            .ok_or_else(|| CurveError::Topology("monotone refinement depth overflow".into()))?;
        Ok(Classification::Decided(()))
    }

    pub(super) fn refined_by(
        &self,
        steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let target_steps = self
            .source_refinement_steps
            .checked_add(steps)
            .ok_or_else(|| CurveError::Topology("monotone refinement depth overflow".into()))?;
        let cached = self
            .refinement_cache
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone();
        let mut refined = self.clone();
        if cached.steps > refined.source_refinement_steps {
            refined.source_lower = cached.source_lower;
            refined.source_upper = cached.source_upper;
            refined.source_lower_sign = cached.source_lower_sign;
            refined.source_upper_sign = cached.source_upper_sign;
            refined.source_refinement_steps = cached.steps;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-monotone-refinement",
                "shared-bracket",
            );
        }
        while refined.source_refinement_steps < target_steps {
            if refined.exact_source_value().is_some() {
                refined.source_refinement_steps = target_steps;
                break;
            }
            match refined.refine_once(policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let mut cache = self
            .refinement_cache
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if refined.source_refinement_steps > cache.steps {
            cache.source_lower = refined.source_lower.clone();
            cache.source_upper = refined.source_upper.clone();
            cache.source_lower_sign = refined.source_lower_sign;
            cache.source_upper_sign = refined.source_upper_sign;
            cache.steps = refined.source_refinement_steps;
        }
        Ok(Classification::Decided(refined))
    }

    pub(super) fn source_order_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(exact) = self.exact_source_value() {
            return Ok(compare_reals(exact, value, &policy.strict_counterpart())
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
        }
        if compare_reals(&self.source_upper, value, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Less));
        }
        if compare_reals(value, &self.source_lower, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Greater));
        }
        let sign = match policy
            .strict_predicate_pass(|| self.system.incidence_sign_at_real(value, policy))?
        {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if sign == RealSign::Zero {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        let root_minus_value = if self.source_lower_sign == RealSign::Negative {
            product_sign(sign, RealSign::Negative)
        } else {
            sign
        };
        Ok(Classification::Decided(match root_minus_value {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }

    pub(super) fn order_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let constant = &self.map_numerator[0] - value * &self.map_denominator[0];
        let linear = &self.map_numerator[1] - value * &self.map_denominator[1];
        let linear_sign = match real_sign(&linear, &policy.strict_counterpart()) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if linear_sign == RealSign::Zero {
            return Ok(match real_sign(&constant, &policy.strict_counterpart()) {
                Some(RealSign::Negative) => Classification::Decided(std::cmp::Ordering::Less),
                Some(RealSign::Zero) => Classification::Decided(std::cmp::Ordering::Equal),
                Some(RealSign::Positive) => Classification::Decided(std::cmp::Ordering::Greater),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            });
        }
        let preimage = ((-constant) / &linear)?;
        Ok(self
            .source_order_to_real(&preimage, policy)?
            .map(|ordering| match linear_sign {
                RealSign::Positive => ordering,
                RealSign::Negative => ordering.reverse(),
                RealSign::Zero => unreachable!(),
            }))
    }

    pub(super) fn composed_map(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let compose = |outer: &[Real; 2]| {
            [
                &outer[0] * &self.map_denominator[0] + &outer[1] * &self.map_numerator[0],
                &outer[0] * &self.map_denominator[1] + &outer[1] * &self.map_numerator[1],
            ]
        };
        let mut mapped = Self {
            system: Arc::clone(&self.system),
            side_chord: self.side_chord.clone(),
            side_parallel: self.side_parallel.clone(),
            source_lower: self.source_lower.clone(),
            source_upper: self.source_upper.clone(),
            source_lower_sign: self.source_lower_sign,
            source_upper_sign: self.source_upper_sign,
            source_refinement_steps: self.source_refinement_steps,
            refinement_cache: Arc::clone(&self.refinement_cache),
            map_numerator: compose(numerator),
            map_denominator: compose(denominator),
        };
        let denominator_sign = |source: &Real| {
            real_sign(
                &(&mapped.map_denominator[0] + &mapped.map_denominator[1] * source),
                &policy.strict_counterpart(),
            )
        };
        let first = denominator_sign(&mapped.source_lower);
        let second = denominator_sign(&mapped.source_upper);
        let sign = match (first, second) {
            (Some(first @ (RealSign::Positive | RealSign::Negative)), Some(second))
                if first == second =>
            {
                first
            }
            (Some(RealSign::Zero), _) | (_, Some(RealSign::Zero)) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if sign == RealSign::Negative {
            for coefficient in mapped
                .map_numerator
                .iter_mut()
                .chain(mapped.map_denominator.iter_mut())
            {
                *coefficient = -coefficient.clone();
            }
        }
        if mapped
            .map_derivative_sign()
            .is_none_or(|sign| sign == RealSign::Zero)
        {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        Ok(Classification::Decided(mapped))
    }

    pub(super) fn source_polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(_) = coefficients
            .iter()
            .rposition(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        let mut refined = self.clone();
        let mut completed_steps = 0_usize;
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
            for _ in completed_steps..steps {
                match refined.refine_once(policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            completed_steps = steps;
            if let Some(exact) = refined.exact_source_value() {
                return Ok(real_sign(&Real::eval_poly(coefficients, exact), policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
            if let Some([lower, upper]) = coefficients_value_interval_on_real_interval(
                coefficients,
                &refined.source_lower,
                &refined.source_upper,
                precision,
            )? {
                let interval = RealInterval {
                    lower: Real::new(lower),
                    upper: Real::new(upper),
                };
                if let Some(sign) = dense_strict_interval_sign(&interval) {
                    return Ok(Classification::Decided(sign));
                }
            }
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(RealSign::Zero));
        }
        let parameter = match self.promoted_source_parameter(policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        signed_coefficients_at_parameter(coefficients, &parameter, policy)
    }

    pub(super) fn mapped_polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(_) = coefficients
            .iter()
            .rposition(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        let mut refined = self.clone();
        let mut completed_steps = 0_usize;
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
            for _ in completed_steps..steps {
                match refined.refine_once(policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            completed_steps = steps;
            let Some((lower, upper)) = refined.mapped_bounds()? else {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            };
            if compare_reals(&lower, &upper, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
            {
                return Ok(real_sign(&Real::eval_poly(coefficients, &lower), policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign)));
            }
            if let Some([lower, upper]) = coefficients_value_interval_on_real_interval(
                coefficients,
                &lower,
                &upper,
                precision,
            )? {
                let interval = RealInterval {
                    lower: Real::new(lower),
                    upper: Real::new(upper),
                };
                if let Some(sign) = dense_strict_interval_sign(&interval) {
                    return Ok(Classification::Decided(sign));
                }
            }
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(RealSign::Zero));
        }
        let parameter = match self.promoted_mapped_parameter(policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        signed_coefficients_at_parameter(coefficients, &parameter, policy)
    }

    pub(super) fn promoted_source_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        if let Some(exact) = self.exact_source_value() {
            return Ok(Classification::Decided(BezierParameter2::Exact(
                exact.clone(),
            )));
        }
        self.system.select_incidence_parameter_in_strict_interval(
            &self.source_lower,
            &self.source_upper,
            policy,
        )
    }

    pub(super) fn promoted_mapped_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        let source = match self.promoted_source_parameter(policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match source {
            BezierParameter2::Exact(source) => Ok(Classification::Decided(
                BezierParameter2::Exact(self.map_value(&source)?),
            )),
            BezierParameter2::Algebraic(_) => policy.strict_predicate_pass(|| {
                let representation = bezier_parameter_root_representation(&source);
                let report = transform_algebraic_root_mobius(
                    &representation,
                    self.map_numerator[1].clone(),
                    self.map_numerator[0].clone(),
                    self.map_denominator[1].clone(),
                    self.map_denominator[0].clone(),
                    policy.predicate_policy(),
                );
                match report.status {
                    AlgebraicRootMobiusTransformStatus::Transformed => {
                        BezierParameter2::from_algebraic_root_representation_unbounded(
                            &report.representation.expect(
                                "a transformed monotone parameter retains its representation",
                            ),
                            policy,
                        )
                    }
                    AlgebraicRootMobiusTransformStatus::Undecided => {
                        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                    }
                    _ => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
                }
            }),
        }
    }
}

impl BezierRecursivePolynomialParameterAuthority2 {
    pub(super) fn new(
        field: RecursiveQuadraticField,
        mut coefficients: Vec<RecursiveQuadraticValue>,
    ) -> Self {
        // Isolation and every later refinement retain the same primitive
        // rational gauge. The scale is positive, preserving endpoint signs
        // as well as roots; arbitrary exact coefficients remain unchanged
        // when rational-content normalization is unavailable.
        RecursiveQuadraticValue::normalize_positive_scale(&mut coefficients);
        Self {
            field,
            coefficients,
        }
    }

    pub(super) fn value_at_real(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        parameter: &Real,
    ) -> Option<RecursiveQuadraticValue> {
        let mut value = self.field.constant(Real::zero())?;
        for coefficient in coefficients.iter().rev() {
            value = value.scale(parameter)?.add(coefficient)?;
        }
        Some(value)
    }

    pub(super) fn value_at_field_element(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        parameter: &RecursiveQuadraticValue,
    ) -> Option<RecursiveQuadraticValue> {
        coefficients
            .iter()
            .rev()
            .try_fold(self.field.constant(Real::zero())?, |value, coefficient| {
                value.multiply(parameter)?.add(coefficient)
            })
    }

    pub(super) fn sign_at_real(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.value_at_real(coefficients, parameter)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive local polynomial crossed its coefficient field".into(),
                )
            })?
            .sign(policy)
    }

    pub(super) fn defining_sign_at_real(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-polynomial-sign",
            "defining-at-real",
        );
        self.sign_at_real(&self.coefficients, parameter, policy)
    }

    pub(super) fn refined_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        target_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRecursiveProjectiveParameter2>> {
        // The retained isolator already certifies these bounds. Replaying its
        // endpoint signs cannot improve a satisfied request and can expand a
        // deep coefficient field merely to republish the same root.
        if target_steps <= parameter.data.refinement_steps {
            return Ok(Classification::Decided(parameter.clone()));
        }
        let strict = policy.strict_counterpart();
        let mut lower = parameter.data.lower.clone();
        let mut upper = parameter.data.upper.clone();
        let lower_sign = match parameter.polynomial_endpoint_sign(0, &strict)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let upper_sign = match parameter.polynomial_endpoint_sign(1, &strict)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if lower_sign == RealSign::Zero {
            upper = lower.clone();
        } else if upper_sign == RealSign::Zero {
            lower = upper.clone();
        } else if !strict_signs_are_opposite(Some(lower_sign), Some(upper_sign)) {
            return Err(CurveError::Topology(
                "a recursive local root lost its isolating sign bracket".into(),
            ));
        }
        for _ in parameter.data.refinement_steps..target_steps {
            if lower == upper {
                break;
            }
            let midpoint = ((&lower + &upper) / Real::from(2_u8))?;
            let midpoint_sign = match self.defining_sign_at_real(&midpoint, &strict)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if midpoint_sign == RealSign::Zero {
                lower = midpoint.clone();
                upper = midpoint;
                break;
            }
            if midpoint_sign == lower_sign {
                lower = midpoint;
            } else {
                upper = midpoint;
            }
        }
        let signs = if lower == upper {
            [RealSign::Zero; 2]
        } else {
            [lower_sign, upper_sign]
        };
        Ok(Classification::Decided(
            BezierRecursiveProjectiveParameter2 {
                data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                    projection: parameter.data.projection.clone(),
                    authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                        authority: parameter
                            .polynomial_authority()
                            .expect("a local root retains its defining polynomial")
                            .clone(),
                        endpoint_signs: signs.map(OnceLock::from),
                    },
                    lower,
                    upper,
                    refinement_steps: target_steps,
                    identity: parameter.data.identity.clone(),
                    line_branch: parameter.data.line_branch,
                    policy: parameter.data.policy,
                }),
            },
        ))
    }

    /// Decides whether a polynomial vanishes at this selected root through
    /// the gcd of the defining relation and the query in the retained field.
    /// `None` leaves the complete sign query to the caller.
    pub(super) fn polynomial_vanishes_at_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        coefficients: &[RecursiveQuadraticValue],
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        if coefficients
            .iter()
            .any(|coefficient| !self.field.same_field(&coefficient.field()))
        {
            return Ok(None);
        }
        let mut context = RecursiveQuadraticOrderedFieldContext {
            field: self.field.clone(),
            policy: *policy,
        };
        match hypersolve::ordered_field_vanishes_at_selected_root(
            &self.coefficients,
            coefficients,
            &hypersolve::IsolatedRootInterval {
                lower: parameter.data.lower.clone(),
                upper: parameter.data.upper.clone(),
                exact_root: None,
                distinct_root_count: 1,
            },
            &mut context,
        ) {
            Ok(vanishes) => Ok(vanishes),
            Err(RecursiveQuadraticOrderedFieldError::Uncertain) => Ok(None),
            Err(RecursiveQuadraticOrderedFieldError::Context(error)) => Err(error),
        }
    }

    pub(super) fn polynomial_sign_at_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        coefficients: &[RecursiveQuadraticValue],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if coefficients.is_empty() {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        if coefficients
            .iter()
            .any(|coefficient| !self.field.same_field(&coefficient.field()))
        {
            // This is a local retained-field accelerator, not a primitive-
            // element authority. A foreign coefficient field simply asks the
            // caller to join fields or use its global exact fallback.
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        // Root equations are projective. Put a possible defining relation in
        // the same positive rational gauge before testing its retained
        // identity or reducing it. Otherwise normalization of the authority
        // alone can hide a known zero behind redundant refinement. The
        // positive scale preserves every nonzero predicate sign as well.
        let normalized = (coefficients.len() >= self.coefficients.len()).then(|| {
            let mut normalized = coefficients.to_vec();
            RecursiveQuadraticValue::normalize_positive_scale(&mut normalized);
            normalized
        });
        let coefficients = normalized.as_deref().unwrap_or(coefficients);
        // Construction already certifies this defining relation. Replaying
        // the same equation requires no leading-coefficient decision: its
        // exact degree may remain unknown while a crossing owns one root.
        if coefficients.len() == self.coefficients.len()
            && coefficients
                .iter()
                .zip(&self.coefficients)
                .all(|(query, defining)| query.is_stored_equivalent_to(defining))
        {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        let mut refined = parameter.clone();
        // At this selected root the defining polynomial is zero. Reuse that
        // relation before interval evaluation loses its coefficient/root
        // correlation. The shared division-free remainder preserves signs,
        // including with a negative or nonrational leading coefficient.
        let reduced = if coefficients.len() >= self.coefficients.len() {
            // Pseudo-division bounds degree, but a nonmonic coefficient
            // field can grow much faster than a few exact bisections. First
            // try bounded refinement of the retained singleton. Keep every
            // tighter certified bracket for subsequent relation replay; an
            // unresolved optional refinement leaves that replay intact.
            for steps in [0_usize, 2, 4, 8] {
                refined = match policy.bounded_exact_predicate_pass(|| {
                    self.refined_parameter(&refined, steps, policy)
                })? {
                    Classification::Decided(refined) => refined,
                    Classification::Uncertain(_) => break,
                };
                if let Some(sign) = recursive_quadratic_polynomial_interval(
                    coefficients,
                    &RealInterval {
                        lower: refined.data.lower.clone(),
                        upper: refined.data.upper.clone(),
                    },
                    64,
                    -64,
                )
                .as_ref()
                .and_then(dense_strict_interval_sign)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-polynomial-sign",
                        "interval-before-remainder",
                    );
                    return Ok(Classification::Decided(sign));
                }
            }
            let mut context = RecursiveQuadraticOrderedFieldContext {
                field: self.field.clone(),
                policy: *policy,
            };
            match policy.bounded_exact_predicate_pass(|| {
                hypersolve::ordered_field_polynomial_sign_remainder(
                    coefficients,
                    &self.coefficients,
                    &mut context,
                )
            }) {
                Ok(Some(remainder)) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-polynomial-sign",
                        "retained-field-remainder",
                    );
                    Some(remainder)
                }
                Ok(None) => {
                    return Err(CurveError::Topology(
                        "a retained polynomial root lost its defining relation".into(),
                    ));
                }
                Err(RecursiveQuadraticOrderedFieldError::Uncertain) => None,
                Err(RecursiveQuadraticOrderedFieldError::Context(error)) => return Err(error),
            }
        } else {
            None
        };
        let coefficients = reduced.as_deref().unwrap_or(coefficients);
        let Some(degree) = coefficients
            .iter()
            .rposition(|coefficient| !coefficient.is_structurally_zero())
        else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        let coefficients = &coefficients[..=degree];
        if degree == 0 {
            return coefficients[0].sign(policy);
        }
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            refined = match self.refined_parameter(
                &refined,
                refinement_steps,
                &policy.strict_counterpart(),
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let target = RealInterval {
                lower: refined.data.lower.clone(),
                upper: refined.data.upper.clone(),
            };
            let coefficient_steps = refinement_steps.saturating_add(64);
            let coefficient_bits = coefficient_steps.max(64).min(i32::MAX as usize) as i32;
            if let Some(sign) = recursive_quadratic_polynomial_interval(
                coefficients,
                &target,
                coefficient_steps,
                -coefficient_bits,
            )
            .as_ref()
            .and_then(dense_strict_interval_sign)
            {
                return Ok(Classification::Decided(sign));
            }
            if refinement_steps == 0 {
                // A contact may already be a selected generator of this
                // coefficient field. Reuse its certified root identity before
                // another bisection, singleton sign query or scalar projection.
                let base = self.field.base_and_extension_path().0;
                for source in &base.sources {
                    let sign = policy.bounded_exact_predicate_pass(
                        || -> CurveResult<Option<RealSign>> {
                            let Some((root, std::cmp::Ordering::Equal)) =
                                parameter.coefficient_root_order(source, policy)?
                            else {
                                return Ok(None);
                            };
                            let Some(value) = self.value_at_field_element(coefficients, &root)
                            else {
                                return Ok(None);
                            };
                            Ok(match value.sign(&policy.strict_counterpart())? {
                                Classification::Decided(sign) => Some(sign),
                                Classification::Uncertain(_) => None,
                            })
                        },
                    )?;
                    if let Some(sign) = sign {
                        return Ok(Classification::Decided(sign));
                    }
                }
            }
            if refinement_steps == 8 {
                // A selected root can belong to a proper factor of the
                // defining polynomial. Intervals cannot prove that equality;
                // ask the shared native-field authority before deeper
                // bisection or global scalar promotion.
                let mut context = RecursiveQuadraticOrderedFieldContext {
                    field: self.field.clone(),
                    policy: *policy,
                };
                match policy.bounded_exact_predicate_pass(|| {
                    hypersolve::ordered_field_sign_at_selected_root(
                        &self.coefficients,
                        coefficients,
                        &hypersolve::IsolatedRootInterval {
                            // Root selection already owns this interval.
                            // Tighter query bounds only make endpoint signs
                            // harder; they add no uniqueness evidence here.
                            lower: parameter.data.lower.clone(),
                            upper: parameter.data.upper.clone(),
                            exact_root: None,
                            distinct_root_count: 1,
                        },
                        &mut context,
                    )
                }) {
                    Ok(Some(sign)) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "recursive-polynomial-sign",
                            "retained-field-singleton-query",
                        );
                        return Ok(Classification::Decided(match sign {
                            std::cmp::Ordering::Less => RealSign::Negative,
                            std::cmp::Ordering::Equal => RealSign::Zero,
                            std::cmp::Ordering::Greater => RealSign::Positive,
                        }));
                    }
                    Ok(None) | Err(RecursiveQuadraticOrderedFieldError::Uncertain) => {}
                    Err(RecursiveQuadraticOrderedFieldError::Context(error)) => return Err(error),
                }
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(RealSign::Zero));
        }
        // The original isolator already selects this root. Query-driven
        // tighter bounds add close-order predicates to global replay without
        // strengthening that identity, so retain its original selection domain.
        let promoted = match self.promoted_parameter(parameter, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let base = self.field.base_and_extension_path().0;
        recursive_quadratic_target_embedding(
            &self.field,
            &base,
            &bezier_parameter_root_representation(&promoted),
        )
        .and_then(|embedding| embedding.polynomial_value(coefficients))
        .ok_or_else(|| {
            CurveError::Topology(
                "a promoted recursive local predicate exceeded its field budget".into(),
            )
        })?
        .sign(policy)
    }

    pub(super) fn promoted_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        let Some((base, projection)) =
            recursive_quadratic_polynomial_projection(self.coefficients.clone())
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if !recursive_quadratic_bases_equivalent(&base, &self.field.base_and_extension_path().0) {
            return Err(CurveError::Topology(
                "a recursive local promotion changed its retained base".into(),
            ));
        }
        let univariate = match selected_dense_last_axis_univariate(
            &projection,
            &base.sources,
            &policy.strict_counterpart(),
        )? {
            Classification::Decided(univariate) => univariate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // The eliminant contains every root of this local relation. When it
        // has exactly one root inside the selected bracket, that root is this
        // parameter. A local monotonicity or Bernstein certificate on the
        // bracket avoids a global Sturm sequence of a dense eliminant whose
        // other roots are irrelevant here.
        if let BezierSelectedDenseLastAxisUnivariate2::Polynomial { polynomial, .. } = &univariate {
            let mut refined = parameter.clone();
            for steps in [0_usize, 4, 8, 16] {
                refined = match self.refined_parameter(&refined, steps, policy)? {
                    Classification::Decided(refined) => refined,
                    Classification::Uncertain(_) => break,
                };
                let (lower, upper) = (&refined.data.lower, &refined.data.upper);
                let coefficients = polynomial.coefficients();
                let endpoint_sign = |value: &Real| {
                    real_sign(&Real::eval_poly(coefficients, value), &CurveContext::STRICT)
                };
                if !matches!(
                    (endpoint_sign(lower), endpoint_sign(upper)),
                    (Some(RealSign::Negative), Some(RealSign::Positive))
                        | (Some(RealSign::Positive), Some(RealSign::Negative))
                ) {
                    continue;
                }
                if hypersolve::polynomial_has_one_distinct_root_in_open_interval(
                    coefficients,
                    lower,
                    upper,
                    hypersolve::PredicatePolicy::STRICT,
                ) != Some(true)
                {
                    continue;
                }
                let interval = match BezierParameterInterval::try_new(
                    lower.clone(),
                    upper.clone(),
                    &CurveContext::STRICT,
                )? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(_) => continue,
                };
                if let Some(local) = BezierAlgebraicParameter2::from_certified_simple_power_basis(
                    coefficients.to_vec(),
                    interval,
                ) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "recursive-parameter-promotion",
                        "local-singleton",
                    );
                    return Ok(Classification::Decided(BezierParameter2::Algebraic(local)));
                }
            }
        }
        let candidates = match isolate_selected_dense_last_axis_univariate(
            &univariate,
            SelectedThirdAxisDomain2::AffineLine,
            &policy.strict_counterpart(),
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let strict = policy.strict_counterpart();
        let lower = BezierParameter2::Exact(parameter.data.lower.clone());
        let upper = BezierParameter2::Exact(parameter.data.upper.clone());
        let mut retained = Vec::new();
        for candidate in candidates {
            let after_lower = candidate.cmp_by_refinement(&lower, &strict)?;
            let before_upper = candidate.cmp_by_refinement(&upper, &strict)?;
            match (after_lower, before_upper) {
                (Classification::Decided(std::cmp::Ordering::Less), _)
                | (_, Classification::Decided(std::cmp::Ordering::Greater)) => {}
                (Classification::Decided(_), Classification::Decided(_)) => {
                    retained.push(candidate)
                }
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if retained.len() == 1 {
            // The local certificate proves a root exists in this interval.
            // Projection contains every such root; its unique candidate here
            // must therefore be that same root, regardless of other sheets.
            return Ok(Classification::Decided(retained.pop().unwrap()));
        }
        let mut selected = None;
        for candidate in retained {
            let evaluation = recursive_quadratic_target_embedding(
                &self.field,
                &base,
                &bezier_parameter_root_representation(&candidate),
            )
            .and_then(|embedding| embedding.polynomial_value(&self.coefficients));
            let Some(evaluation) = evaluation else {
                continue;
            };
            match evaluation.sign(&strict)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            if selected.is_some() {
                return Err(CurveError::Topology(
                    "one recursive local root selected multiple global parameters".into(),
                ));
            }
            selected = Some(candidate);
        }
        Ok(selected.map_or_else(
            || Classification::Uncertain(UncertaintyReason::Boundary),
            Classification::Decided,
        ))
    }

    pub(super) fn projective_image_coefficients(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
    ) -> Option<Vec<RecursiveQuadraticValue>> {
        // Inverse of v=(n0+n1*t)/(d0+d1*t):
        // t=(n0-d0*v)/(-n1+d1*v).
        let inverse_numerator = recursive_quadratic_real_polynomial(
            &self.field,
            &[numerator[0].clone(), -denominator[0].clone()],
        )?;
        let inverse_denominator = recursive_quadratic_real_polynomial(
            &self.field,
            &[-numerator[1].clone(), denominator[1].clone()],
        )?;
        let mut transformed = vec![self.coefficients.last()?.clone()];
        let mut denominator_power =
            recursive_quadratic_real_polynomial(&self.field, std::slice::from_ref(&Real::one()))?;
        for coefficient in self.coefficients[..self.coefficients.len() - 1]
            .iter()
            .rev()
        {
            denominator_power =
                recursive_quadratic_polynomial_multiply(&denominator_power, &inverse_denominator)?;
            transformed = recursive_quadratic_polynomial_combine(
                &recursive_quadratic_polynomial_multiply(&transformed, &inverse_numerator)?,
                &recursive_quadratic_polynomial_scale(&denominator_power, coefficient)?,
                false,
            )?;
        }
        Some(transformed)
    }
}

impl BezierRecursiveProjectiveChordParallelIntervalSystem2 {
    /// Replaces one structurally deep exact midpoint by certified dyadic
    /// bounds. The interval still contains the authored value exactly, while
    /// all later Horner arithmetic stays rational and can prove signs below a
    /// predicate evaluator's scalar-refinement horizon.
    pub(super) fn parameter_interval(&self, parameter: &Real) -> Option<RealInterval> {
        if let Some(value) = parameter.exact_rational_normal_form() {
            let value = Real::new(value);
            return Some(RealInterval {
                lower: value.clone(),
                upper: value,
            });
        }
        let [lower, upper] = parameter.certified_rational_interval(self.precision)?;
        Some(RealInterval {
            lower: Real::new(lower),
            upper: Real::new(upper),
        })
    }

    pub(super) fn polynomial_interval(
        coefficients: &[RealInterval],
        parameter: &RealInterval,
    ) -> Option<RealInterval> {
        let mut value = RealInterval {
            lower: Real::zero(),
            upper: Real::zero(),
        };
        for coefficient in coefficients.iter().rev() {
            value = value.multiply(parameter)?.add(coefficient);
        }
        Some(value)
    }

    pub(super) fn incidence_sign(&self, parameter: &RealInterval) -> Option<RealSign> {
        let rational = Self::polynomial_interval(&self.incidence_rational, parameter)?;
        let radical = Self::polynomial_interval(&self.incidence_radical, parameter)?;
        let speed_squared = Self::polynomial_interval(&self.speed_squared, parameter)?;
        let speed = speed_squared.nonnegative_square_root(Some(self.precision))?;
        let radical_speed = radical.multiply(&speed)?;
        let incidence = rational.add(&radical_speed);
        let sign = dense_strict_interval_sign(&incidence);
        if sign.is_none() && std::env::var_os("HYPERCURVE_DEBUG_MONOTONE_INTERVAL").is_some() {
            let approximate = |value: &RealInterval| {
                (
                    value.lower.to_f64_lossy(),
                    value.upper.to_f64_lossy(),
                    (&value.upper - &value.lower).to_f64_lossy(),
                )
            };
            eprintln!(
                "prepared incidence overlap parameter={:?} rational={:?} radical={:?} speed-squared={:?} speed={:?} radical-speed={:?} incidence={:?}",
                approximate(parameter),
                approximate(&rational),
                approximate(&radical),
                approximate(&speed_squared),
                approximate(&speed),
                approximate(&radical_speed),
                approximate(&incidence),
            );
        }
        sign
    }
}

impl BezierRecursiveProjectiveChordParallelSystem2 {
    pub(super) fn projected_polynomial(
        &self,
        coefficients: &[RecursiveQuadraticValue],
    ) -> Option<DenseTensorPolynomial> {
        let (base, projection) = recursive_quadratic_polynomial_projection(coefficients.to_vec())?;
        recursive_quadratic_bases_equivalent(&base, &self.base).then_some(projection)
    }

    pub(super) fn projected_parameters(
        &self,
        projection: &DenseTensorPolynomial,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        policy.strict_predicate_pass(|| {
            selected_dense_last_axis_parameters(projection, &self.base.sources, domain, policy)
        })
    }

    pub(super) fn parameters(
        &self,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        let Some(projection) = self.incidence_projection() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.projected_parameters(projection, domain, policy)
    }

    pub(super) fn incidence_polynomial(&self) -> Option<&[RecursiveQuadraticValue]> {
        // A source-only incidence has no target-speed radical. Squaring it
        // needlessly doubles multiplicities before each coefficient-field
        // norm, and can obscure selected zero coefficients during projection.
        if self
            .incidence
            .radical
            .iter()
            .all(RecursiveQuadraticValue::is_structurally_zero)
        {
            return Some(&self.incidence.rational);
        }
        self.incidence.squared_magnitude_difference()
    }

    /// Cache only successful global elimination. Finite local isolation and
    /// replay do not construct it, while later carrier switches can request it.
    pub(super) fn incidence_projection(&self) -> Option<&DenseTensorPolynomial> {
        if self.projection.get().is_none() {
            let projection = self.projected_polynomial(self.incidence_polynomial()?)?;
            let _ = self.projection.set(projection);
        }
        self.projection.get()
    }

    /// A complete local isolation report covers an outward finite envelope;
    /// the retained endpoints, not that envelope, decide domain membership.
    /// The caller certifies endpoint incidence on the authored parallel.
    /// Remove any such roots already in the coefficient field before isolation,
    /// then merge their original parameters back into the complete result.
    /// Repeated roots and unresolved coefficient signs use global elimination.
    pub(super) fn local_parameters(
        &self,
        domain: SelectedThirdAxisDomain2<'_>,
        endpoint_roots: [Option<CurveParameter2>; 2],
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<CurveParameter2>>> {
        let SelectedThirdAxisDomain2::Finite(range) = domain else {
            return Ok(None);
        };
        let domain = CurveParameterDomain2::new(range, None);
        let Classification::Decided((_, bounds)) = domain.finite_envelope(policy)? else {
            return Ok(None);
        };
        if compare_reals(bounds[0], bounds[1], &CurveContext::STRICT)
            != Some(std::cmp::Ordering::Less)
        {
            return Ok(None);
        }
        let Some(coefficients) = self.incidence_polynomial() else {
            return Ok(None);
        };
        let mut coefficients = Cow::Borrowed(coefficients);
        let mut owned_roots: Vec<CurveParameter2> = Vec::new();
        let mut context = RecursiveQuadraticOrderedFieldContext {
            field: self.field.clone(),
            policy: policy.strict_counterpart(),
        };
        'endpoint: for parameter in endpoint_roots.into_iter().flatten() {
            let Some(root) =
                recursive_field_retained_parameter_value(&self.field, &parameter, policy)?
            else {
                continue;
            };
            for retained in &owned_roots {
                if !matches!(
                    policy.bounded_exact_predicate_pass(|| {
                        parameter.cmp_by_refinement(retained, policy)
                    })?,
                    Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater)
                ) {
                    // One certified factor is enough when distinctness is
                    // unresolved; the remaining polynomial still owns it.
                    continue 'endpoint;
                }
            }
            coefficients = match hypersolve::ordered_field_polynomial_linear_quotient(
                &coefficients,
                &root,
                &mut context,
            ) {
                Ok(quotient) => Cow::Owned(quotient),
                Err(RecursiveQuadraticOrderedFieldError::Context(error)) => return Err(error),
                Err(RecursiveQuadraticOrderedFieldError::Uncertain) => return Ok(None),
            };
            owned_roots.push(parameter);
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parallel-kernel",
                "retained-contact-factor",
            );
        }
        let Some(mut candidates) = recursive_quadratic_polynomial_local_parameters(
            &self.field,
            &coefficients,
            bounds,
            policy,
        )?
        else {
            return Ok(None);
        };
        for root in owned_roots {
            let mut index = 0;
            while let Some(candidate) = candidates.get(index) {
                match root.cmp_by_refinement(candidate, policy)? {
                    Classification::Decided(std::cmp::Ordering::Less) => break,
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        // A residual multiple root keeps its original authority.
                        candidates.remove(index);
                        break;
                    }
                    Classification::Decided(std::cmp::Ordering::Greater) => index += 1,
                    Classification::Uncertain(_) => return Ok(None),
                }
            }
            candidates.insert(index, root);
        }
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            match domain.contains_finite_parameter(&candidate, policy)? {
                Classification::Decided(true) => retained.push(candidate),
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => return Ok(None),
            }
        }
        Ok(Some(retained))
    }

    /// Some(true) proves the authored sheet is a component; Some(false)
    /// proves the opposite sheet is. None means only a foreign coefficient
    /// conjugate made the projection vanish. A common zero of A and B at the
    /// sample is inconclusive: use their first nonzero Taylor coefficients.
    pub(super) fn norm_component_sheet_at_real(
        &self,
        sample: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<bool>>> {
        let Some(norm) = self.incidence.squared_magnitude_difference() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match recursive_quadratic_polynomial_is_identically_zero(norm, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        for (polynomial, positive) in [
            (&self.source_weight[..], false),
            (&self.incidence.speed_squared[..], true),
        ] {
            match self.polynomial_sign_at_real(polynomial, sample, policy)? {
                Classification::Decided(RealSign::Positive) => {}
                Classification::Decided(RealSign::Negative) if !positive => {}
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a recursive norm component had negative squared speed".into(),
                    ));
                }
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let derivative = |coefficients: &[RecursiveQuadraticValue]| {
            coefficients
                .iter()
                .enumerate()
                .skip(1)
                .map(|(power, coefficient)| {
                    coefficient.scale(&Real::from(i64::try_from(power).ok()?))
                })
                .collect::<Option<Vec<_>>>()
        };
        let mut rational = self.incidence.rational.clone();
        let mut radical = self.incidence.radical.clone();
        for _ in 0..rational.len().max(radical.len()) {
            let first = self.polynomial_sign_at_real(&rational, sample, policy)?;
            let second = self.polynomial_sign_at_real(&radical, sample, policy)?;
            match (first, second) {
                (
                    Classification::Decided(RealSign::Zero),
                    Classification::Decided(RealSign::Zero),
                ) => {}
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                (Classification::Decided(RealSign::Zero), _)
                | (_, Classification::Decided(RealSign::Zero)) => {
                    return Err(CurveError::Topology(
                        "a zero norm with positive speed had unequal vanishing orders".into(),
                    ));
                }
                (Classification::Decided(first), Classification::Decided(second)) => {
                    // Norm identity and S>0 give equal vanishing orders and
                    // squared magnitudes. Opposite leading signs cancel on
                    // the positive speed sheet throughout this regular cell.
                    return Ok(Classification::Decided(Some(first != second)));
                }
            }
            let (Some(first), Some(second)) = (derivative(&rational), derivative(&radical)) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            rational = first;
            radical = second;
        }
        // All Taylor coefficients vanished, so both terms are identically zero.
        Ok(Classification::Decided(Some(true)))
    }

    pub(super) fn candidate_evaluation(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierRecursiveQuadraticParallelEvaluation2>>> {
        recursive_quadratic_parallel_candidate_evaluation(
            &self.field,
            &self.base,
            &self.source_weight,
            &self.incidence.speed_squared,
            self.incidence.speed_squared.len() == 1
                && self.incidence.speed_squared[0]
                    .exact_real_value_with_retained_witnesses()
                    .is_some_and(|speed| speed == Real::one()),
            target_parameter,
            policy,
        )
    }

    pub(super) fn expression_sign(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        evaluation
            .expression_value(expression)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel expression exceeded its field budget".into(),
                )
            })?
            .sign(policy)
    }

    pub(super) fn certified_expression_replay_sign(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        certificate: &BezierDenseSelectedCandidateBox2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sources = evaluation.source_box(certificate).ok_or_else(|| {
            CurveError::Topology(
                "a certified recursive chord/parallel replay lost its source box".into(),
            )
        })?;
        let value = evaluation.expression_value(expression).ok_or_else(|| {
            CurveError::Topology(
                "a certified recursive chord/parallel replay exceeded its field budget".into(),
            )
        })?;
        let mut component_sign = |component: &RecursiveQuadraticValue| {
            self.certified_component_sign(component, evaluation, certificate, policy)
        };
        value.sign_at_projected_zero(&sources, policy, &mut component_sign)
    }

    pub(super) fn certified_component_projection(
        &self,
        value: &RecursiveQuadraticValue,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
    ) -> Option<DenseTensorPolynomial> {
        let (base, projection) = recursive_quadratic_polynomial_projection(vec![value.clone()])?;
        if !recursive_quadratic_bases_equivalent(&base, &evaluation.embedding.target_base) {
            return None;
        }
        let output_axis = projection.dimensions().len().checked_sub(1)?;
        let projection = projection.remove_certified_independent_axis(
            output_axis,
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        )?;
        (evaluation.embedding.target_axis + 1 == projection.dimensions().len()
            && evaluation
                .embedding
                .source_axes
                .iter()
                .copied()
                .eq(0..evaluation.embedding.target_axis))
        .then_some(projection)
    }

    pub(super) fn certified_component_sign(
        &self,
        value: &RecursiveQuadraticValue,
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        certificate: &BezierDenseSelectedCandidateBox2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sources = evaluation.source_box(certificate).ok_or_else(|| {
            CurveError::Topology(
                "a recursive chord/parallel component lost its selected source box".into(),
            )
        })?;
        if let Some(sign) = value.sign_over_source_box(&sources) {
            return Ok(Classification::Decided(sign));
        }
        if let Some(sign) =
            value.sign_over_progressively_refined_source_box(&sources, 0..=64, true)?
        {
            return Ok(Classification::Decided(sign));
        }
        let Some(projection) = self.certified_component_projection(value, evaluation) else {
            return value.sign(policy);
        };
        let candidate = match BezierParameter2::from_algebraic_root_representation_unbounded(
            &certificate.candidate,
            &CurveContext::STRICT,
        )? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(_) => return value.sign(policy),
        };
        match projected_selected_dense_candidate_box_incidence(
            &projection,
            &certificate.sources,
            &candidate,
            16,
            0,
        ) {
            // Unlike the system enumerator, an arbitrary component did not
            // define the candidate's isolating polynomial. A transverse zero
            // somewhere in the candidate box therefore does not prove that
            // the component vanishes at the selected candidate itself.
            Some(BezierDenseCandidateBoxIncidence2::Root(_)) => value.sign(policy),
            Some(BezierDenseCandidateBoxIncidence2::Disjoint(certificate)) => {
                let sources = evaluation.source_box(&certificate).ok_or_else(|| {
                    CurveError::Topology(
                        "a nonzero recursive chord/parallel component lost its source box".into(),
                    )
                })?;
                Ok(value.sign_with_nonzero_certificate_over_source_box(&sources)?)
            }
            None => value.sign(policy),
        }
    }

    pub(super) fn polynomial_sign(
        &self,
        polynomial: &[RecursiveQuadraticValue],
        evaluation: &BezierRecursiveQuadraticParallelEvaluation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        evaluation
            .polynomial_value(polynomial)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel polynomial exceeded its field budget".into(),
                )
            })?
            .sign(policy)
    }

    /// Signs the unsquared chord/parallel incidence at an already-retained
    /// recursive target parameter. The positive target speed is eliminated
    /// by one squared-magnitude comparison in the parameter's native ordered
    /// field, avoiding the global target-axis projection used for enumeration.
    pub(super) fn incidence_sign_at_recursive_parameter(
        &self,
        parameter: &BezierRecursiveProjectiveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sign = |polynomial: &[RecursiveQuadraticValue]| {
            parameter.recursive_polynomial_sign_joined(polynomial, policy)
        };
        let weight_sign = match policy.strict_predicate_pass(|| sign(&self.source_weight))? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(self
            .incidence
            .sign_with_positive_speed(policy, sign)?
            .map(|incidence_sign| product_sign(incidence_sign, weight_sign)))
    }

    pub(super) fn polynomial_value_at_real(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        target_parameter: &Real,
    ) -> Option<RecursiveQuadraticValue> {
        let mut value = self.field.constant(Real::zero())?;
        for coefficient in coefficients.iter().rev() {
            value = value.scale(target_parameter)?.add(coefficient)?;
        }
        Some(value)
    }

    /// Encloses one retained-field polynomial over a represented parameter
    /// interval without adjoining that parameter to the coefficient field.
    /// This is deliberately a sufficient predicate: dependency inflation may
    /// decline, but every returned interval remains exact.
    pub(super) fn polynomial_interval_on_real_interval(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        parameter: &RealInterval,
        refinement_steps: usize,
        coefficient_precision: i32,
    ) -> Option<RealInterval> {
        let mut value = RealInterval {
            lower: Real::zero(),
            upper: Real::zero(),
        };
        for coefficient in coefficients.iter().rev() {
            let coefficient = coefficient.interval_with_coefficient_precision(
                refinement_steps,
                Some(coefficient_precision),
            )?;
            value = value.multiply(parameter)?.add(&coefficient);
        }
        Some(value)
    }

    /// Refines the independent coefficient-field sources once and retains an
    /// exact outward enclosure for every target-polynomial coefficient.
    /// Subsequent monotone bisections contain only small interval Horner
    /// evaluations; no algebraic source isolation is repeated.
    pub(super) fn prepare_incidence_interval_system(
        &self,
        source_refinement_steps: usize,
        coefficient_precision: i32,
    ) -> Option<BezierRecursiveProjectiveChordParallelIntervalSystem2> {
        let sources = self
            .base
            .sources
            .iter()
            .enumerate()
            .map(|(index, source)| {
                let refined = refined_represented_root(source, source_refinement_steps);
                if std::env::var_os("HYPERCURVE_DEBUG_MONOTONE_INTERVAL").is_some() {
                    eprintln!(
                        "prepared source {index} changed={} original-width={:?} refined-width={:?}",
                        &refined != source,
                        (&source.interval.upper - &source.interval.lower).to_f64_lossy(),
                        (&refined.interval.upper - &refined.interval.lower).to_f64_lossy(),
                    );
                }
                refined
            })
            .collect::<Vec<_>>();
        let intervals = |coefficients: &[RecursiveQuadraticValue]| {
            coefficients
                .iter()
                .map(|coefficient| {
                    coefficient.interval_over_source_box_with_witnesses(
                        &sources,
                        Some(coefficient_precision),
                        true,
                    )
                })
                .collect::<Option<Vec<_>>>()
        };
        Some(BezierRecursiveProjectiveChordParallelIntervalSystem2 {
            incidence_rational: intervals(&self.incidence.rational)?,
            incidence_radical: intervals(&self.incidence.radical)?,
            speed_squared: intervals(&self.incidence.speed_squared)?,
            precision: coefficient_precision,
        })
    }

    /// Signs this chord's physical oriented incidence over a retained target
    /// parameter interval.  The target speed stays a positive interval root;
    /// coefficient fields remain independent of the monotone root authority.
    pub(super) fn incidence_interval_sign(
        &self,
        parameter: &RealInterval,
        refinement_steps: usize,
        coefficient_precision: i32,
    ) -> Option<RealSign> {
        let rational = self.polynomial_interval_on_real_interval(
            &self.incidence.rational,
            parameter,
            refinement_steps,
            coefficient_precision,
        )?;
        let radical = self.polynomial_interval_on_real_interval(
            &self.incidence.radical,
            parameter,
            refinement_steps,
            coefficient_precision,
        )?;
        let speed = self
            .polynomial_interval_on_real_interval(
                &self.incidence.speed_squared,
                parameter,
                refinement_steps,
                coefficient_precision,
            )?
            .nonnegative_square_root(Some(coefficient_precision))?;
        let incidence = radical
            .multiply(&speed)
            .map(|radical| rational.add(&radical))?;
        dense_strict_interval_sign(&incidence)
    }

    /// Signs the conjugate sheet `a - b*sqrt(S)` of the incidence over a
    /// target parameter interval, with the same enclosures as the authored
    /// sheet. A strict sign proves the squared incidence has no conjugate
    /// root in the interval.
    pub(super) fn conjugate_incidence_interval_sign(
        &self,
        parameter: &RealInterval,
        refinement_steps: usize,
        coefficient_precision: i32,
    ) -> Option<RealSign> {
        let rational = self.polynomial_interval_on_real_interval(
            &self.incidence.rational,
            parameter,
            refinement_steps,
            coefficient_precision,
        )?;
        let radical = self.polynomial_interval_on_real_interval(
            &self.incidence.radical,
            parameter,
            refinement_steps,
            coefficient_precision,
        )?;
        let speed = self
            .polynomial_interval_on_real_interval(
                &self.incidence.speed_squared,
                parameter,
                refinement_steps,
                coefficient_precision,
            )?
            .nonnegative_square_root(Some(coefficient_precision))?;
        let conjugate = radical
            .multiply(&speed)
            .map(|radical| rational.subtract(&radical))?;
        dense_strict_interval_sign(&conjugate)
    }

    pub(super) fn oriented_incidence_interval_sign(
        &self,
        parameter: &RealInterval,
        refinement_steps: usize,
        coefficient_precision: i32,
    ) -> Option<RealSign> {
        let incidence_sign =
            self.incidence_interval_sign(parameter, refinement_steps, coefficient_precision)?;
        let weight = self.polynomial_interval_on_real_interval(
            &self.source_weight,
            parameter,
            refinement_steps,
            coefficient_precision,
        )?;
        let weight_sign = dense_strict_interval_sign(&weight)?;
        if weight_sign == RealSign::Zero {
            return None;
        }
        Some(product_sign(incidence_sign, weight_sign))
    }

    pub(super) fn polynomial_sign_at_real(
        &self,
        polynomial: &[RecursiveQuadraticValue],
        target_parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.polynomial_value_at_real(polynomial, target_parameter)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel exact polynomial exceeded its field budget".into(),
                )
            })?
            .sign(&policy.strict_counterpart())
    }

    /// Signs the authored unsquared line/parallel equation at one represented
    /// target parameter.  The target-speed root is appended to the existing
    /// recursive coefficient tower directly; no target axis, global norm, or
    /// endpoint-box reconstruction is needed for monotone-root refinement.
    pub(super) fn incidence_sign_at_real(
        &self,
        target_parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let rational = self
            .polynomial_value_at_real(&self.incidence.rational, target_parameter)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel rational incidence exceeded its field budget"
                        .into(),
                )
            })?;
        let radical = self
            .polynomial_value_at_real(&self.incidence.radical, target_parameter)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel radical incidence exceeded its field budget".into(),
                )
            })?;
        if radical.is_structurally_zero() {
            return rational.sign(policy);
        }
        let speed_squared = self
            .polynomial_value_at_real(&self.incidence.speed_squared, target_parameter)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive chord/parallel exact speed exceeded its field budget".into(),
                )
            })?;
        match speed_squared.sign(policy)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return rational.sign(policy);
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive chord/parallel exact speed squared was negative".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        RecursiveQuadraticValue::affine_positive_root_sign(
            &rational,
            &radical,
            &speed_squared,
            None,
            None,
            policy,
        )
    }

    pub(super) fn select_incidence_parameter_in_strict_interval(
        &self,
        lower: &Real,
        upper: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        policy.strict_predicate_pass(|| {
            let Some(projection) = self.incidence_projection() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let candidates = match self.projected_parameters(
                projection,
                SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
                policy,
            )? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero
                    | BezierAlgebraicFiberProjection2::Degenerate,
                ) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let strict = policy;
            let lower = BezierParameter2::Exact(lower.clone());
            let upper = BezierParameter2::Exact(upper.clone());
            let mut selected = None;
            let mut uncertainty = None;
            let mut bracketed = Vec::new();
            let mut strictly_inside = 0_usize;
            for candidate in candidates {
                let after_lower = candidate.cmp_by_refinement(&lower, strict)?;
                let before_upper = candidate.cmp_by_refinement(&upper, strict)?;
                match (after_lower, before_upper) {
                    (
                        Classification::Decided(
                            after @ (std::cmp::Ordering::Equal | std::cmp::Ordering::Greater),
                        ),
                        Classification::Decided(
                            before @ (std::cmp::Ordering::Equal | std::cmp::Ordering::Less),
                        ),
                    ) => {
                        if after == std::cmp::Ordering::Greater
                            && before == std::cmp::Ordering::Less
                        {
                            strictly_inside += 1;
                        }
                        bracketed.push(candidate);
                    }
                    (Classification::Decided(_), Classification::Decided(_)) => {}
                    (Classification::Uncertain(reason), _)
                    | (_, Classification::Uncertain(reason)) => {
                        uncertainty = Some(reason);
                    }
                }
            }
            // The bracket's strict opposite endpoint signs place one root of
            // the monotone incidence strictly inside it, and the projection
            // contains every incidence root. A sole projected candidate
            // strictly inside is therefore that root, without the exact zero
            // test of the radical incidence at an algebraic candidate.
            if uncertainty.is_none() && strictly_inside == 1 && bracketed.len() == 1 {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-monotone-promotion",
                    "sole-projected-candidate",
                );
                return Ok(Classification::Decided(
                    bracketed.pop().expect("one bracketed candidate"),
                ));
            }
            for candidate in bracketed {
                let evaluation = match self.candidate_evaluation(&candidate, strict)? {
                    Classification::Decided(Some(evaluation)) => evaluation,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        uncertainty = Some(reason);
                        continue;
                    }
                };
                match self.expression_sign(&self.incidence, &evaluation, strict)? {
                    Classification::Decided(RealSign::Zero) => {
                        if selected.is_some() {
                            return Err(CurveError::Topology(
                                "one monotone recursive bracket selected multiple projected roots"
                                    .into(),
                            ));
                        }
                        selected = Some(candidate);
                    }
                    Classification::Decided(RealSign::Negative | RealSign::Positive) => {}
                    Classification::Uncertain(reason) => uncertainty = Some(reason),
                }
            }
            Ok(match selected {
                Some(parameter) => Classification::Decided(parameter),
                None => {
                    Classification::Uncertain(uncertainty.unwrap_or(UncertaintyReason::Boundary))
                }
            })
        })
    }
}

impl BezierRecursiveProjectiveChordRationalSystem2 {
    pub(super) fn parameters(
        &self,
        range: &CurveParameterRange2,
        crossing: Option<BezierRecursiveQuadraticUnitCrossing2>,
        endpoint_roots: [bool; 2],
        excluded_contact: Option<&CurveParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            Vec<CurveParameter2>,
            Option<BezierRecursiveQuadraticUnitCrossing2>,
        )>,
    > {
        let mut coefficients = self.incidence.clone();
        let mut field = RecursiveQuadraticOrderedFieldContext {
            field: self.field.clone(),
            policy: policy.strict_counterpart(),
        };
        let known_roots = endpoint_roots
            .into_iter()
            .enumerate()
            .filter(|&(_, certified)| certified)
            .map(|(index, _)| Real::from(index as i8))
            .collect::<Vec<_>>();
        let excluded_root = if let Some(contact) = excluded_contact {
            policy.bounded_exact_predicate_pass(|| -> CurveResult<Option<_>> {
                let Some(value) =
                    recursive_field_retained_parameter_value(&self.field, contact, policy)?
                else {
                    return Ok(None);
                };
                for root in &known_roots {
                    let endpoint = CurveParameter2::from(root.clone());
                    if !matches!(
                        contact.cmp_by_refinement(&endpoint, policy)?,
                        Classification::Decided(
                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater
                        )
                    ) {
                        return Ok(None);
                    }
                }
                Ok(Some(value))
            })?
        } else {
            None
        };
        let mut known_roots = known_roots
            .into_iter()
            .map(|root| self.field.constant(root))
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| {
                CurveError::Topology("a certified endpoint lost its coefficient field".into())
            })?;
        if let Some(root) = excluded_root {
            known_roots.push(root);
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "retained-contact-factor",
            );
        }
        let crossing = if known_roots.is_empty() {
            crossing
        } else {
            None
        };
        for root in &known_roots {
            // Endpoint sides and the already-owned adjacent contact prove
            // these distinct factors. Keep that incidence instead of asking
            // reconstructed roots to rediscover their scalar equalities.
            coefficients = match hypersolve::ordered_field_polynomial_linear_quotient(
                &coefficients,
                root,
                &mut field,
            ) {
                Ok(coefficients) => coefficients,
                Err(RecursiveQuadraticOrderedFieldError::Context(error)) => return Err(error),
                Err(RecursiveQuadraticOrderedFieldError::Uncertain) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
            };
        }
        match recursive_projective_polynomial_parameters_with_crossing(
            &self.field,
            coefficients,
            crossing.clone(),
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(mut parameters) => {
                for (index, certified) in endpoint_roots.into_iter().enumerate() {
                    if !certified {
                        continue;
                    }
                    let endpoint =
                        CurveParameter2::from(BezierParameter2::Exact(Real::from(index as i8)));
                    let mut repeated = false;
                    for parameter in &parameters {
                        match parameter.same_value(&endpoint, &field.policy)? {
                            Classification::Decided(true) => repeated = true,
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    if !repeated {
                        if index == 0 {
                            parameters.insert(0, endpoint);
                        } else {
                            parameters.push(endpoint);
                        }
                    }
                }
                Ok(Classification::Decided((parameters, crossing)))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Compares a retained unit-domain contact to one chord endpoint without
    /// adjoining the contact's quadratic generator to the endpoint field.
    /// A common-sign rational subcurve is contained by its exact homogeneous
    /// Bernstein hull. Disjoint hull/endpoint intervals are authoritative;
    /// overlapping intervals deliberately decline to the complete recursive
    /// projective predicate.
    pub(super) fn parameter_hull_axis_order(
        &self,
        source: &RationalBezier2,
        parameter: &CurveParameter2,
        endpoint: &BezierRecursiveQuadraticProjectivePoint2,
        axis: Axis2,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        let Some(parameter) = parameter.as_recursive_projective() else {
            return Ok(None);
        };
        let (lower, upper) = parameter.isolating_bounds();
        let strict = &CurveContext::STRICT;
        let source = match source.subcurve_between_exact_with_policy(lower, upper, strict)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        let source_bounds = match source.certified_bounds_classified() {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(_) => return Ok(None),
        };
        let source_interval = real_interval_from_axis(&source_bounds, axis);
        let endpoint_numerator = match axis {
            Axis2::X => &endpoint.x,
            Axis2::Y => &endpoint.y,
        };
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128] {
            let Some(endpoint_interval) =
                endpoint_numerator
                    .interval(refinement_steps)
                    .and_then(|numerator| {
                        endpoint
                            .denominator
                            .interval(refinement_steps)
                            .and_then(|denominator| numerator.divide(&denominator))
                    })
            else {
                continue;
            };
            if compare_reals(&source_interval.upper, &endpoint_interval.lower, strict)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-axis-order",
                    "source-parameter-hull",
                );
                return Ok(Some(std::cmp::Ordering::Less));
            }
            if compare_reals(&endpoint_interval.upper, &source_interval.lower, strict)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-axis-order",
                    "source-parameter-hull",
                );
                return Ok(Some(std::cmp::Ordering::Greater));
            }
        }
        Ok(None)
    }

    /// Compares a direct quadratic-incidence contact to an endpoint when the
    /// target coordinate is affine in its native parameter.  Rather than
    /// adjoining the contact radical, this orders the unique incidence root
    /// against the affine coordinate preimage in the original endpoint field.
    /// Every sign used here is either structural or interval-separated; a
    /// declined certificate falls through to the general projective path.
    pub(super) fn affine_parameter_axis_order(
        &self,
        endpoint: &BezierRecursiveQuadraticProjectivePoint2,
        axis: Axis2,
        crossing: &BezierRecursiveQuadraticUnitCrossing2,
    ) -> Option<std::cmp::Ordering> {
        let weight_sign = self.source_weight_sign?;
        let endpoint_coordinate = match axis {
            Axis2::X => &endpoint.x,
            Axis2::Y => &endpoint.y,
        };
        let target_coordinate = match axis {
            Axis2::X => &self.source_x,
            Axis2::Y => &self.source_y,
        };
        let mut difference = recursive_quadratic_polynomial_combine(
            &recursive_quadratic_polynomial_scale(target_coordinate, &endpoint.denominator)?,
            &recursive_quadratic_polynomial_scale(&self.source_weight, endpoint_coordinate)?,
            true,
        )?;
        while difference
            .last()
            .is_some_and(RecursiveQuadraticValue::is_structurally_zero)
        {
            difference.pop();
        }
        let quick_sign = |value: &RecursiveQuadraticValue| {
            if value.is_structurally_zero() {
                Some(RealSign::Zero)
            } else {
                value.bounded_interval_sign(0..=512)
            }
        };
        let coordinate_sign = match difference.as_slice() {
            [] => RealSign::Zero,
            [constant] => quick_sign(constant)?,
            [constant, linear] => {
                let linear_sign = quick_sign(linear)?;
                if linear_sign == RealSign::Zero {
                    quick_sign(constant)?
                } else {
                    let start_sign = quick_sign(constant)?;
                    let end_value = constant.add(linear)?;
                    let end_sign = quick_sign(&end_value)?;
                    match (start_sign, end_sign) {
                        (RealSign::Zero, _) => linear_sign,
                        (_, RealSign::Zero) => product_sign(linear_sign, RealSign::Negative),
                        (start, end) if start == end => start,
                        (RealSign::Positive, RealSign::Negative)
                        | (RealSign::Negative, RealSign::Positive) => {
                            let (numerator, denominator) = if linear_sign == RealSign::Positive {
                                (constant.scale(&Real::from(-1_i8))?, linear.clone())
                            } else {
                                (constant.clone(), linear.scale(&Real::from(-1_i8))?)
                            };
                            let preimage = RecursiveQuadraticProjectiveScalar {
                                numerator,
                                denominator,
                            };
                            // At the affine preimage the selected-axis
                            // difference from `endpoint` is exactly zero. The
                            // authored line incidence consequently reduces to
                            // the perpendicular difference times the already-
                            // certified chord-axis direction, preserving
                            // correlation without expanding the full recursive
                            // cross product.
                            let (perpendicular_coordinate, endpoint_perpendicular) = match axis {
                                Axis2::X => (&self.source_y, &endpoint.y),
                                Axis2::Y => (&self.source_x, &endpoint.x),
                            };
                            let perpendicular_difference = recursive_quadratic_polynomial_combine(
                                &recursive_quadratic_polynomial_scale(
                                    perpendicular_coordinate,
                                    &endpoint.denominator,
                                )?,
                                &recursive_quadratic_polynomial_scale(
                                    &self.source_weight,
                                    endpoint_perpendicular,
                                )?,
                                true,
                            )?;
                            let perpendicular_sign =
                                quick_sign(&recursive_projective_polynomial_value(
                                    &perpendicular_difference,
                                    &preimage,
                                )?)?;
                            let incidence_sign = product_sign(
                                self.affine_preimage_incidence_factor_sign?,
                                perpendicular_sign,
                            );
                            let root_minus_preimage_sign = if incidence_sign == RealSign::Zero {
                                RealSign::Zero
                            } else if incidence_sign == crossing.start_sign {
                                RealSign::Positive
                            } else if incidence_sign == crossing.end_sign {
                                RealSign::Negative
                            } else {
                                return None;
                            };
                            product_sign(linear_sign, root_minus_preimage_sign)
                        }
                        _ => return None,
                    }
                }
            }
            _ => return None,
        };
        let affine_sign = product_sign(coordinate_sign, weight_sign);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-projective-axis-order",
            "affine-preimage",
        );
        Some(match affine_sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        })
    }
}

impl BezierRecursiveProjectiveParameter2 {
    pub(super) fn projective_scalar(&self) -> Option<&RecursiveQuadraticProjectiveScalar> {
        match &self.data.authority {
            BezierRecursiveProjectiveParameterAuthority2::Projective(scalar) => Some(scalar),
            BezierRecursiveProjectiveParameterAuthority2::Monotone(_)
            | BezierRecursiveProjectiveParameterAuthority2::Polynomial { .. } => None,
        }
    }

    pub(super) fn monotone_authority(&self) -> Option<&BezierRecursiveMonotoneParameter2> {
        match &self.data.authority {
            BezierRecursiveProjectiveParameterAuthority2::Projective(_)
            | BezierRecursiveProjectiveParameterAuthority2::Polynomial { .. } => None,
            BezierRecursiveProjectiveParameterAuthority2::Monotone(authority) => Some(authority),
        }
    }

    pub(super) fn polynomial_authority(
        &self,
    ) -> Option<&Arc<BezierRecursivePolynomialParameterAuthority2>> {
        match &self.data.authority {
            BezierRecursiveProjectiveParameterAuthority2::Polynomial { authority, .. } => {
                Some(authority)
            }
            BezierRecursiveProjectiveParameterAuthority2::Projective(_)
            | BezierRecursiveProjectiveParameterAuthority2::Monotone(_) => None,
        }
    }

    pub(super) fn polynomial_endpoint_sign(
        &self,
        endpoint: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let BezierRecursiveProjectiveParameterAuthority2::Polynomial {
            authority,
            endpoint_signs,
        } = &self.data.authority
        else {
            unreachable!("a local polynomial endpoint retains its defining authority")
        };
        if let Some(sign) = endpoint_signs[endpoint].get() {
            return Ok(Classification::Decided(*sign));
        }
        let value = [&self.data.lower, &self.data.upper][endpoint];
        let result =
            policy.strict_predicate_pass(|| authority.defining_sign_at_real(value, policy))?;
        if let Classification::Decided(sign) = result {
            let _ = endpoint_signs[endpoint].set(sign);
        }
        Ok(result)
    }

    pub(super) fn shares_polynomial_root(&self, other: &Self) -> bool {
        let (Some(first), Some(second)) =
            (self.polynomial_authority(), other.polynomial_authority())
        else {
            return false;
        };
        if !Arc::ptr_eq(first, second) {
            return false;
        }
        let contains = |outer: &Self, inner: &Self| {
            matches!(
                compare_reals(&outer.data.lower, &inner.data.lower, &CurveContext::STRICT,),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
            ) && matches!(
                compare_reals(&inner.data.upper, &outer.data.upper, &CurveContext::STRICT,),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
            )
        };
        contains(self, other) || contains(other, self)
    }

    pub(super) fn new_monotone(
        system: Arc<BezierRecursiveProjectiveChordParallelSystem2>,
        side_chord: BezierAlgebraicChord2,
        side_parallel: BezierParallel2,
        source_lower: Real,
        source_upper: Real,
        source_lower_sign: RealSign,
        source_upper_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if !strict_signs_are_opposite(Some(source_lower_sign), Some(source_upper_sign)) {
            return Err(CurveError::Topology(
                "a retained monotone root requires opposite strict endpoint signs".into(),
            ));
        }
        if compare_reals(&source_lower, &source_upper, &CurveContext::STRICT)
            != Some(std::cmp::Ordering::Less)
        {
            return Err(CurveError::InvalidBezierRange);
        }
        let refinement_cache = Arc::new(Mutex::new(BezierRecursiveMonotoneRefinementCache2 {
            source_lower: source_lower.clone(),
            source_upper: source_upper.clone(),
            source_lower_sign,
            source_upper_sign,
            steps: 0,
            interval_system: None,
            interval_system_attempted: false,
        }));
        let authority = BezierRecursiveMonotoneParameter2 {
            system,
            side_chord,
            side_parallel,
            source_lower: source_lower.clone(),
            source_upper: source_upper.clone(),
            source_lower_sign,
            source_upper_sign,
            source_refinement_steps: 0,
            refinement_cache,
            map_numerator: [Real::zero(), Real::one()],
            map_denominator: [Real::one(), Real::zero()],
        };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: Arc::default(),
                authority: BezierRecursiveProjectiveParameterAuthority2::Monotone(authority),
                lower: source_lower,
                upper: source_upper,
                refinement_steps: 0,
                identity: None,
                line_branch: 0,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    pub(super) fn from_mapped_monotone(
        authority: BezierRecursiveMonotoneParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let Some((lower, upper)) = authority.mapped_bounds()? else {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        };
        let refinement_steps = authority.source_refinement_steps;
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: Arc::default(),
                authority: BezierRecursiveProjectiveParameterAuthority2::Monotone(authority),
                lower,
                upper,
                refinement_steps,
                identity: None,
                line_branch: 0,
                policy: policy.retained_object_policy(),
            }),
        }))
    }

    pub(super) fn new(
        scalar: RecursiveQuadraticProjectiveScalar,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::new_with_certified_bounds(scalar, None, policy)
    }

    pub(super) fn new_with_certified_bounds(
        scalar: RecursiveQuadraticProjectiveScalar,
        certified_bounds: Option<(Real, Real)>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match scalar.denominator.sign(&policy.strict_counterpart())? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive projective parameter lost its positive denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        if let Some((lower, upper)) = certified_bounds {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-parameter",
                "certified-unit-bounds",
            );
            return Ok(Classification::Decided(Self {
                data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                    projection: Arc::default(),
                    authority: BezierRecursiveProjectiveParameterAuthority2::Projective(scalar),
                    lower,
                    upper,
                    refinement_steps: 0,
                    identity: None,
                    line_branch: 0,
                    policy: policy.retained_object_policy(),
                }),
            }));
        }
        let mut refinement_steps = 0_usize;
        loop {
            if let Some(interval) = scalar.interval(refinement_steps) {
                return Ok(Classification::Decided(Self {
                    data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                        projection: Arc::default(),
                        authority: BezierRecursiveProjectiveParameterAuthority2::Projective(scalar),
                        lower: interval.lower,
                        upper: interval.upper,
                        refinement_steps,
                        identity: None,
                        line_branch: 0,
                        policy: policy.retained_object_policy(),
                    }),
                }));
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology(
                        "recursive projective parameter refinement overflow".into(),
                    )
                })?;
        }
    }

    pub(super) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "a recursive projective parameter crossed predicate policies".into(),
            ));
        }
        Ok(())
    }

    pub(super) fn with_line_identity(
        self,
        line_identity: Arc<BezierRecursiveLineParameterIdentity2>,
        line_branch: i8,
    ) -> Self {
        Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: self.data.projection.clone(),
                authority: self.data.authority.clone(),
                lower: self.data.lower.clone(),
                upper: self.data.upper.clone(),
                refinement_steps: self.data.refinement_steps,
                identity: Some(Arc::new(BezierRecursiveProjectiveParameterIdentity2::Line(
                    (*line_identity).clone(),
                ))),
                line_branch,
                policy: self.data.policy,
            }),
        }
    }

    pub(crate) fn with_chord_rational_tangent_identity(
        self,
        chord: BezierAlgebraicChord2,
        source: RationalBezier2,
        tangent_cross_sign: RealSign,
        chord_location: BezierRecursiveChordContactLocation2,
    ) -> Self {
        Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: self.data.projection.clone(),
                authority: self.data.authority.clone(),
                lower: self.data.lower.clone(),
                upper: self.data.upper.clone(),
                refinement_steps: self.data.refinement_steps,
                identity: Some(Arc::new(
                    BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(
                        BezierRecursiveChordRationalTangentIdentity2 {
                            chord,
                            source,
                            tangent_cross_sign,
                            chord_location,
                        },
                    ),
                )),
                line_branch: 0,
                policy: self.data.policy,
            }),
        }
    }

    /// Replays the tangent orientation certified by the chord/rational
    /// intersection that authored this parameter. Split/reversed descendants
    /// of the same chord support and reversed target traversal are adjusted
    /// structurally; unrelated carriers deliberately decline the certificate.
    pub(super) fn chord_rational_tangent_cross_sign(
        &self,
        chord: &BezierAlgebraicChord2,
        source: &RationalBezier2,
        source_direction: RealSign,
        policy: &CurveContext,
    ) -> Option<CurveResult<Classification<RealSign>>> {
        if let Err(error) = self.validate_policy(policy) {
            return Some(Err(error));
        }
        let BezierRecursiveProjectiveParameterIdentity2::ChordRationalTangent(identity) =
            self.data.identity.as_deref()?
        else {
            return None;
        };
        #[cfg(test)]
        if std::env::var_os("HYPERCURVE_DEBUG_PAIR_SCALAR").is_some() {
            eprintln!(
                "recursive tangent identity source={} direct-orientation={:?} radial-orientation={:?}",
                source == &identity.source,
                identity.chord.shared_tangent_orientation(chord),
                identity
                    .chord
                    .retained_radial_tangent_reversal_to(chord, policy),
            );
        }
        if source != &identity.source {
            return None;
        }
        let chord_reversed = match identity.chord.shared_tangent_orientation(chord) {
            Some(reversed) => reversed,
            None => match identity
                .chord
                .retained_radial_tangent_reversal_to(chord, policy)
            {
                Ok(Some(reversed)) => reversed,
                Ok(None) => return None,
                Err(error) => return Some(Err(error)),
            },
        };
        let mut sign = identity.tangent_cross_sign;
        if chord_reversed {
            sign = product_sign(sign, RealSign::Negative);
        }
        match source_direction {
            RealSign::Negative => sign = product_sign(sign, RealSign::Negative),
            RealSign::Positive => {}
            RealSign::Zero => {
                return Some(Err(CurveError::Topology(
                    "a retained rational tangent had zero traversal direction".into(),
                )));
            }
        }
        Some(Ok(Classification::Decided(sign)))
    }

    pub(crate) fn transported_line_identity(
        &self,
        line: LineSeg2,
        transform: &Similarity2,
    ) -> Self {
        let Some(BezierRecursiveProjectiveParameterIdentity2::Line(identity)) =
            self.data.identity.as_deref()
        else {
            return self.clone();
        };
        let transform = identity
            .transform
            .as_ref()
            .map_or_else(|| transform.clone(), |source| source.then(transform));
        Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: self.data.projection.clone(),
                authority: self.data.authority.clone(),
                lower: self.data.lower.clone(),
                upper: self.data.upper.clone(),
                refinement_steps: self.data.refinement_steps,
                identity: Some(Arc::new(BezierRecursiveProjectiveParameterIdentity2::Line(
                    BezierRecursiveLineParameterIdentity2 {
                        source_frame: identity.source_frame.clone(),
                        source_radial_distance: identity.source_radial_distance.clone(),
                        source_clockwise: identity.source_clockwise,
                        line,
                        transform: Some(transform),
                    },
                ))),
                line_branch: self.data.line_branch,
                policy: self.data.policy,
            }),
        }
    }

    pub(crate) fn isolating_bounds(&self) -> (&Real, &Real) {
        (&self.data.lower, &self.data.upper)
    }

    /// Evaluates `d^degree * polynomial(n / d)` in this scalar's retained
    /// recursive field. Callers may choose a degree above the polynomial's
    /// active degree to align several rational expressions to one homogeneous
    /// scale without constructing a global scalar image.
    pub(super) fn homogeneous_polynomial_value(
        &self,
        coefficients: &[Real],
        degree: usize,
    ) -> Option<RecursiveQuadraticValue> {
        if coefficients
            .iter()
            .skip(degree.saturating_add(1))
            .any(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        {
            return None;
        }
        let scalar = self.projective_scalar()?;
        let field = scalar.numerator.field();
        let mut value =
            field.constant(coefficients.get(degree).cloned().unwrap_or_else(Real::zero))?;
        let mut denominator_power = field.constant(Real::one())?;
        for index in (0..degree).rev() {
            denominator_power = denominator_power.multiply(&scalar.denominator)?;
            let coefficient =
                field.constant(coefficients.get(index).cloned().unwrap_or_else(Real::zero))?;
            value = value
                .multiply(&scalar.numerator)?
                .add(&coefficient.multiply(&denominator_power)?)?;
        }
        Some(value)
    }

    pub(super) fn translated(
        &self,
        delta: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.affine_image_unbounded(&Real::one(), delta, policy)
    }

    /// Signs one power-basis polynomial directly at this retained projective
    /// scalar.  If `t = n / d` with the stored `d > 0`, the homogeneous Horner
    /// replay below signs `d^degree * polynomial(t)` in the existing recursive
    /// quadratic field.  No global scalar image or resultant is constructed.
    pub(crate) fn polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        let Some(degree) = coefficients
            .iter()
            .rposition(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
        else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        if let Some(authority) = self.polynomial_authority() {
            let Some(coefficients) =
                recursive_quadratic_real_polynomial(&authority.field, coefficients)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return authority.polynomial_sign_at_parameter(self, &coefficients, policy);
        }
        if let Some(authority) = self.monotone_authority() {
            return authority.mapped_polynomial_sign(coefficients, policy);
        }
        let Some(value) = self.homogeneous_polynomial_value(coefficients, degree) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        value.sign(policy)
    }

    /// Decides whether a real polynomial vanishes at this parameter without
    /// signing it, when a polynomial authority's field can find the common
    /// divisor of the query and the defining relation.
    pub(crate) fn polynomial_vanishes(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Option<bool>> {
        self.validate_policy(policy)?;
        if let Some(authority) = self.monotone_authority() {
            return authority.polynomial_vanishes(coefficients, policy);
        }
        let Some(authority) = self.polynomial_authority() else {
            return Ok(None);
        };
        let Some(coefficients) =
            recursive_quadratic_real_polynomial(&authority.field, coefficients)
        else {
            return Ok(None);
        };
        authority.polynomial_vanishes_at_parameter(self, &coefficients, policy)
    }

    /// Signs a polynomial over retained recursive coefficients. Polynomial
    /// roots reuse their defining coefficient field; projective parameters
    /// also lift ancestor coefficients through homogeneous Horner evaluation.
    /// No coefficient projection or primitive element is formed.
    pub(super) fn recursive_polynomial_sign(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        let Some(degree) = coefficients
            .iter()
            .rposition(|coefficient| !coefficient.is_structurally_zero())
        else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        match &self.data.authority {
            BezierRecursiveProjectiveParameterAuthority2::Polynomial { authority, .. } => {
                authority.polynomial_sign_at_parameter(self, &coefficients[..=degree], policy)
            }
            BezierRecursiveProjectiveParameterAuthority2::Projective(scalar) => {
                // Formula roots can adjoin a radical to their coefficient
                // field. The shared homogeneous evaluator lifts ancestor
                // coefficients into that extension as it evaluates them.
                let Some(value) =
                    recursive_projective_polynomial_value(&coefficients[..=degree], scalar)
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                value.sign(policy)
            }
            BezierRecursiveProjectiveParameterAuthority2::Monotone(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Signs a polynomial at this retained root after embedding foreign
    /// coefficients into the least shared recursive tower.
    ///
    /// A locally isolated polynomial root must remain correlated with its
    /// defining coefficient field.  Promoting that root to an independent
    /// dense axis discards precisely the relation needed by small tangent and
    /// incidence determinants.  When the query coefficients live in an
    /// ancestor, descendant, or divergent branch of the same recursive base,
    /// replay the defining polynomial in the joined field and reuse the same
    /// isolating bracket instead.
    pub(super) fn recursive_polynomial_sign_joined(
        &self,
        coefficients: &[RecursiveQuadraticValue],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        let Some(authority) = self.polynomial_authority() else {
            return self.recursive_polynomial_sign(coefficients, policy);
        };
        let Some(first) = coefficients.first() else {
            return Ok(Classification::Decided(RealSign::Zero));
        };
        let coefficient_field = first.field();
        if coefficients
            .iter()
            .any(|coefficient| !coefficient_field.same_field(&coefficient.field()))
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }

        let sign_in_field = |field: RecursiveQuadraticField,
                             defining: Vec<RecursiveQuadraticValue>,
                             query: Vec<RecursiveQuadraticValue>|
         -> CurveResult<Classification<RealSign>> {
            let authority = Arc::new(BezierRecursivePolynomialParameterAuthority2::new(
                field, defining,
            ));
            let parameter = Self {
                data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                    projection: self.data.projection.clone(),
                    authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                        authority: authority.clone(),
                        // Exact field embedding and positive normalization
                        // preserve signs at these unchanged endpoints.
                        endpoint_signs: match &self.data.authority {
                            BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                                endpoint_signs,
                                ..
                            } => endpoint_signs.clone(),
                            _ => unreachable!("field replay retains a local polynomial root"),
                        },
                    },
                    lower: self.data.lower.clone(),
                    upper: self.data.upper.clone(),
                    refinement_steps: self.data.refinement_steps,
                    identity: self.data.identity.clone(),
                    line_branch: self.data.line_branch,
                    policy: self.data.policy,
                }),
            };
            authority.polynomial_sign_at_parameter(&parameter, &query, policy)
        };

        if let Some(query) = coefficients
            .iter()
            .map(|coefficient| authority.field.lift(coefficient))
            .collect::<Option<Vec<_>>>()
        {
            return authority.polynomial_sign_at_parameter(self, &query, policy);
        }
        if let Some(defining) = authority
            .coefficients
            .iter()
            .map(|coefficient| coefficient_field.lift(coefficient))
            .collect::<Option<Vec<_>>>()
        {
            return sign_in_field(coefficient_field, defining, coefficients.to_vec());
        }
        if let Some((target_base, embeddings)) =
            coefficient_field.extension_embeddings_to_equivalent_tower(&authority.field)
            && let Some(query) = coefficients
                .iter()
                .map(|coefficient| {
                    coefficient
                        .rebased_to_equivalent_base(target_base.clone(), &embeddings)
                        .and_then(|coefficient| authority.field.lift(&coefficient))
                })
                .collect::<Option<Vec<_>>>()
        {
            return authority.polynomial_sign_at_parameter(self, &query, policy);
        }
        let (joined, query) = match authority.field.joined_with(&coefficient_field, policy)? {
            Classification::Decided(Some((joined, embeddings))) => {
                let Some(query) = coefficients
                    .iter()
                    .map(|coefficient| joined.embed_value(coefficient, &embeddings))
                    .collect::<Option<Vec<_>>>()
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                (joined, query)
            }
            Classification::Decided(None) => {
                // Different base allocations or polynomial-valued speed
                // generators do not make the selected coefficient roots
                // independent. Replay them over the existing source tuple.
                let (source_base, _) = coefficient_field.base_and_extension_path();
                let (target_base, _) = authority.field.base_and_extension_path();
                let Some(axes) = source_base
                    .sources
                    .iter()
                    .map(|source| {
                        target_base
                            .sources
                            .iter()
                            .position(|target| target == source)
                    })
                    .collect::<Option<Vec<_>>>()
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let (joined, embedding) = match recursive_embed_foreign_field(
                    &coefficient_field,
                    target_base,
                    axes,
                    authority.field.clone(),
                    policy,
                )? {
                    Classification::Decided(Some(joined)) => joined,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let Some(query) = coefficients
                    .iter()
                    .map(|coefficient| embedding.value(coefficient, &joined))
                    .collect::<Option<Vec<_>>>()
                else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                (joined, query)
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if joined.same_field(&authority.field) {
            return authority.polynomial_sign_at_parameter(self, &query, policy);
        }
        let Some(defining) = authority
            .coefficients
            .iter()
            .map(|coefficient| joined.lift(coefficient))
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        sign_in_field(joined, defining, query)
    }

    pub(crate) fn refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        let refinement_steps = refinement_steps.max(self.data.refinement_steps);
        if let Some(authority) = self.monotone_authority() {
            let additional = refinement_steps.saturating_sub(self.data.refinement_steps);
            let authority = match authority.refined_by(additional, policy)? {
                Classification::Decided(authority) => authority,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let Some((lower, upper)) = authority.mapped_bounds()? else {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            };
            let refinement_steps = authority.source_refinement_steps;
            return Ok(Classification::Decided(Self {
                data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                    projection: self.data.projection.clone(),
                    authority: BezierRecursiveProjectiveParameterAuthority2::Monotone(authority),
                    lower,
                    upper,
                    refinement_steps,
                    identity: self.data.identity.clone(),
                    line_branch: self.data.line_branch,
                    policy: self.data.policy,
                }),
            }));
        }
        if let Some(authority) = self.polynomial_authority() {
            return authority.refined_parameter(self, refinement_steps, policy);
        }
        let scalar = self
            .projective_scalar()
            .expect("a non-monotone recursive parameter owns a projective scalar");
        let Some(interval) = scalar.interval(refinement_steps) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        // The scalar's recursively projected interval may be coarser than
        // the construction-domain isolator retained on this parameter. A
        // refinement is an intersection, never a replacement: discarding a
        // certified unit/chord bracket can make deeper requests grow without
        // bound and defeats every exact enclosure consumer.
        let lower = if compare_reals(&interval.lower, &self.data.lower, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            interval.lower
        } else {
            self.data.lower.clone()
        };
        let upper = if compare_reals(&interval.upper, &self.data.upper, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            interval.upper
        } else {
            self.data.upper.clone()
        };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                projection: self.data.projection.clone(),
                authority: self.data.authority.clone(),
                lower,
                upper,
                refinement_steps,
                identity: self.data.identity.clone(),
                line_branch: self.data.line_branch,
                policy: self.data.policy,
            }),
        }))
    }

    pub(crate) fn order_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        let upper_order = compare_reals(&self.data.upper, value, &CurveContext::STRICT);
        if upper_order == Some(std::cmp::Ordering::Less) {
            return Ok(Classification::Decided(std::cmp::Ordering::Less));
        }
        let lower_order = compare_reals(value, &self.data.lower, &CurveContext::STRICT);
        if lower_order == Some(std::cmp::Ordering::Less) {
            return Ok(Classification::Decided(std::cmp::Ordering::Greater));
        }
        match &self.data.authority {
            BezierRecursiveProjectiveParameterAuthority2::Projective(scalar) => {
                scalar.order_to_real(value, policy)
            }
            BezierRecursiveProjectiveParameterAuthority2::Monotone(authority) => {
                authority.order_to_real(value, policy)
            }
            BezierRecursiveProjectiveParameterAuthority2::Polynomial { authority, .. } => {
                let sign = match if lower_order == Some(std::cmp::Ordering::Equal) {
                    self.polynomial_endpoint_sign(0, policy)
                } else if upper_order == Some(std::cmp::Ordering::Equal) {
                    self.polynomial_endpoint_sign(1, policy)
                } else {
                    authority.defining_sign_at_real(value, &policy.strict_counterpart())
                }? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if sign == RealSign::Zero {
                    return Ok(Classification::Decided(std::cmp::Ordering::Equal));
                }
                let lower_sign = match self.polynomial_endpoint_sign(0, policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(Classification::Decided(if sign == lower_sign {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Less
                }))
            }
        }
    }

    /// Orders two retained scalars using only their native outward isolators.
    ///
    /// This is the bounded non-projecting authority for optional topology
    /// shortcuts: disjoint envelopes prove an exact order, shared construction
    /// storage proves equality, and any residual overlap remains explicit for
    /// the authoritative geometric equality predicate. In particular this
    /// helper never consumes APPROXIMATE_512's terminal equality decision.
    pub(crate) fn cmp_by_native_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if self == other {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.shares_polynomial_root(other) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.data.line_branch == other.data.line_branch
            && let (
                Some(BezierRecursiveProjectiveParameterIdentity2::Line(first)),
                Some(BezierRecursiveProjectiveParameterIdentity2::Line(second)),
            ) = (
                self.data.identity.as_deref(),
                other.data.identity.as_deref(),
            )
            && first.same_parameterization(second)
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if let (Some(first), Some(second)) = (self.monotone_authority(), other.monotone_authority())
            && Arc::ptr_eq(&first.system, &second.system)
            && first.map_numerator == second.map_numerator
            && first.map_denominator == second.map_denominator
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32] {
            let first = match self.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match other.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if compare_reals(&first.data.upper, &second.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(&second.data.upper, &first.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
        }
        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }

    pub(crate) fn certifies_monotone_chord_parallel_contact(
        &self,
        chord: &BezierAlgebraicChord2,
        parallel: &BezierParallel2,
    ) -> bool {
        let Some(authority) = self.monotone_authority() else {
            return false;
        };
        let chord_matches = authority.side_chord.shares_retained_support(chord);
        let parallel_matches = authority.side_parallel == *parallel;
        chord_matches && parallel_matches
    }

    pub(crate) fn cmp_selected_fiber_parameter(
        &self,
        other: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        use std::cmp::Ordering;
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let strict = policy.strict_counterpart();
        let separated = |lower: &Real, upper: &Real, other_lower: &Real, other_upper: &Real| {
            if compare_reals(upper, other_lower, &strict) == Some(Ordering::Less) {
                Some(Ordering::Less)
            } else if compare_reals(other_upper, lower, &strict) == Some(Ordering::Less) {
                Some(Ordering::Greater)
            } else {
                None
            }
        };
        if let Some(order) = separated(
            &self.data.lower,
            &self.data.upper,
            &other.root().lower,
            &other.root().upper,
        ) {
            return Ok(Classification::Decided(order));
        }
        if self.data.projection.parameter.get().is_none()
            && let Some(chart) = &self.data.projection.chart
            && chart.source.data.projection.parameter.get().is_some()
        {
            // Replaying an already-projected source through its retained
            // chart needs no new elimination and preserves coefficient-field
            // identity for the direct comparison below.
            let _ = self.promoted_bezier_parameter_complete(policy)?;
        }
        if let Some(native) = self.data.projection.parameter.get() {
            if let Some(other_native) = other.data.representations.bezier.get()
                && let Classification::Decided(order) = policy
                    .strict_predicate_pass(|| native.cmp_by_refinement(other_native, policy))?
            {
                return Ok(Classification::Decided(order));
            }
            if let BezierParameter2::Algebraic(native) = native
                && native
                    .projective_map_from(&other.data.authority.data.retained_parameter)
                    .is_some()
                && let Classification::Decided(order) = policy.strict_predicate_pass(|| {
                    other.cmp_bezier_parameter(&BezierParameter2::Algebraic(native.clone()), policy)
                })?
            {
                // The prior projection already identifies an element of this
                // fiber's coefficient field. Reuse that linear predicate
                // before refining two enclosures of the same boundary root.
                return Ok(Classification::Decided(order.reverse()));
            }
        }
        for steps in [0, 2, 4, 8, 16, 32] {
            let Classification::Decided(first) = self.refined(steps, policy)? else {
                break;
            };
            let Classification::Decided(second) = other.refined(steps, policy)? else {
                break;
            };
            if let Some(order) = separated(
                &first.data.lower,
                &first.data.upper,
                &second.root().lower,
                &second.root().upper,
            ) {
                return Ok(Classification::Decided(order));
            }
            if first.data.lower == first.data.upper && second.root().lower == second.root().upper {
                if let Some(order) = compare_reals(&first.data.lower, &second.root().lower, &strict)
                {
                    return Ok(Classification::Decided(order));
                }
                break;
            }
        }
        policy.strict_predicate_pass(|| {
            let projected = self.promoted_bezier_parameter_complete(policy)?;
            if let Classification::Decided(native) = &projected
                && let Classification::Decided(order) =
                    other.cmp_bezier_parameter(native, policy)?
            {
                return Ok(Classification::Decided(order.reverse()));
            }
            match other.promoted_bezier_parameter_complete(policy)? {
                Classification::Decided(other_native) => match projected {
                    Classification::Decided(native) => {
                        native.cmp_by_refinement(&other_native, policy)
                    }
                    Classification::Uncertain(_) => {
                        self.cmp_bezier_parameter(&other_native, policy)
                    }
                },
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        })
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        if self == other {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.shares_polynomial_root(other) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if self.data.line_branch == other.data.line_branch
            && let (
                Some(BezierRecursiveProjectiveParameterIdentity2::Line(first)),
                Some(BezierRecursiveProjectiveParameterIdentity2::Line(second)),
            ) = (
                self.data.identity.as_deref(),
                other.data.identity.as_deref(),
            )
            && first.same_parameterization(second)
        {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if compare_reals(&self.data.upper, &other.data.lower, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-parameter-order",
                "stored-interval-separated",
            );
            return Ok(Classification::Decided(std::cmp::Ordering::Less));
        }
        if compare_reals(&other.data.upper, &self.data.lower, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-parameter-order",
                "stored-interval-separated",
            );
            return Ok(Classification::Decided(std::cmp::Ordering::Greater));
        }
        if let (Some(first), Some(second)) = (self.monotone_authority(), other.monotone_authority())
            && Arc::ptr_eq(&first.system, &second.system)
        {
            if first.map_numerator == second.map_numerator
                && first.map_denominator == second.map_denominator
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            let multiply_linear = |left: &[Real; 2], right: &[Real; 2]| {
                vec![
                    &left[0] * &right[0],
                    &left[0] * &right[1] + &left[1] * &right[0],
                    &left[1] * &right[1],
                ]
            };
            let first_product = multiply_linear(&first.map_numerator, &second.map_denominator);
            let second_product = multiply_linear(&second.map_numerator, &first.map_denominator);
            let difference = polynomial_subtract(&first_product, &second_product);
            return Ok(first
                .source_polynomial_sign(&difference, policy)?
                .map(|sign| match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                }));
        }
        // Independent recursive fields usually name distinct parameters.  Use
        // their native outward isolators before constructing a joined field:
        // disjoint boxes are a complete exact order certificate, while an
        // overlap merely declines this fast path and preserves the symbolic
        // equality fallback below.
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32] {
            let first = self.refined(refinement_steps, policy)?;
            let second = other.refined(refinement_steps, policy)?;
            let (Classification::Decided(first), Classification::Decided(second)) = (first, second)
            else {
                break;
            };
            if compare_reals(&first.data.upper, &second.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-parameter-order",
                    "refined-interval-separated",
                );
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(&second.data.upper, &first.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-parameter-order",
                    "refined-interval-separated",
                );
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
        }
        if let (Some(first), Some(second)) = (self.projective_scalar(), other.projective_scalar()) {
            if let Some(difference) =
                first
                    .numerator
                    .multiply(&second.denominator)
                    .and_then(|first_value| {
                        second
                            .numerator
                            .multiply(&first.denominator)
                            .and_then(|second_value| first_value.subtract(&second_value))
                    })
            {
                let sign = match difference.sign(policy)? {
                    decided @ Classification::Decided(_) => decided,
                    Classification::Uncertain(_) => {
                        difference.sign_with_projected_zero_fallback(policy)?
                    }
                };
                return Ok(sign.map(|sign| match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                }));
            }
            match projective_scalar_joined_projective_difference(first, second, policy)? {
                Classification::Decided(Some(difference)) => {
                    let sign = match difference.sign(policy)? {
                        decided @ Classification::Decided(_) => decided,
                        Classification::Uncertain(_) => {
                            difference.sign_with_projected_zero_fallback(policy)?
                        }
                    };
                    return Ok(sign.map(|sign| match sign {
                        RealSign::Negative => std::cmp::Ordering::Less,
                        RealSign::Zero => std::cmp::Ordering::Equal,
                        RealSign::Positive => std::cmp::Ordering::Greater,
                    }));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            if let Classification::Decided(Some(difference)) =
                projective_scalar_merged_projective_difference(first, second, policy)?
            {
                let sign = match difference.sign(policy)? {
                    decided @ Classification::Decided(_) => decided,
                    Classification::Uncertain(_) => {
                        difference.sign_with_projected_zero_fallback(policy)?
                    }
                };
                return Ok(sign.map(|sign| match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                }));
            }
        }
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let first = match self.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match other.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if compare_reals(&first.data.upper, &second.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(&second.data.upper, &first.data.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
        }
        let first = match policy
            .strict_predicate_pass(|| self.promoted_bezier_parameter_complete(policy))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match policy
            .strict_predicate_pass(|| other.promoted_bezier_parameter_complete(policy))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        policy.strict_predicate_pass(|| first.cmp_by_refinement(&second, policy))
    }

    /// Orders this singleton against an existing coefficient-field root.
    /// Exact endpoint comparisons can separate them even when their stored
    /// bounds overlap. Within the interval, the defining polynomial has one
    /// simple root, so its sign at the generator decides order and equality.
    /// A declined certificate makes no claim about either value.
    pub(super) fn coefficient_root_order(
        &self,
        source: &AlgebraicRootRepresentation,
        policy: &CurveContext,
    ) -> CurveResult<Option<(RecursiveQuadraticValue, std::cmp::Ordering)>> {
        self.validate_policy(policy)?;
        let Some(authority) = self.polynomial_authority() else {
            return Ok(None);
        };
        if !source.is_valid() {
            return Ok(None);
        }
        let Some(root) = authority.field.retained_root_value(source) else {
            return Ok(None);
        };
        let strict = policy.strict_counterpart();
        let lower_order = represented_order_to_real(source, &self.data.lower, &strict);
        let upper_order = represented_order_to_real(source, &self.data.upper, &strict);
        let order = if lower_order == Classification::Decided(std::cmp::Ordering::Less) {
            std::cmp::Ordering::Greater
        } else if upper_order == Classification::Decided(std::cmp::Ordering::Greater) {
            std::cmp::Ordering::Less
        } else if matches!(
            lower_order,
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
        ) && matches!(
            upper_order,
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            let Some(value) = authority.value_at_field_element(&authority.coefficients, &root)
            else {
                return Ok(None);
            };
            let Classification::Decided(sign) = value.sign(&strict)? else {
                return Ok(None);
            };
            if sign == RealSign::Zero {
                std::cmp::Ordering::Equal
            } else {
                let Classification::Decided(lower_sign) =
                    self.polynomial_endpoint_sign(0, &strict)?
                else {
                    return Ok(None);
                };
                if sign == lower_sign {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Less
                }
            }
        } else {
            return Ok(None);
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "recursive-parameter-comparison",
            if order == std::cmp::Ordering::Equal {
                "retained-generator-identity"
            } else {
                "retained-generator-order"
            },
        );
        Ok(Some((root, order)))
    }

    pub(crate) fn cmp_bezier_parameter(
        &self,
        other: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        if let Some(value) = other.scalar() {
            return self.order_to_real(value, policy);
        }
        if let Some(scalar) = self.projective_scalar() {
            let field = scalar.denominator.field();
            if let Some(root) =
                recursive_field_retained_parameter_value(&field, &other.clone().into(), policy)?
                && let Some(difference) = scalar
                    .denominator
                    .multiply(&root)
                    .and_then(|product| scalar.numerator.subtract(&product))
                && let Classification::Decided(sign) =
                    policy.strict_predicate_pass(|| difference.sign(policy))?
            {
                // The positive projective denominator preserves order. This
                // native root is already a generator of the formula's field;
                // compare there before refining equal enclosures or projecting
                // a new scalar that would discard their shared identity.
                return Ok(Classification::Decided(match sign {
                    RealSign::Negative => std::cmp::Ordering::Less,
                    RealSign::Zero => std::cmp::Ordering::Equal,
                    RealSign::Positive => std::cmp::Ordering::Greater,
                }));
            }
        }
        if let Some((_, order)) =
            self.coefficient_root_order(&bezier_parameter_root_representation(other), policy)?
        {
            return Ok(Classification::Decided(order));
        }
        let BezierParameter2::Algebraic(selection) = other else {
            unreachable!("represented native parameters returned above")
        };
        let strict = policy.strict_counterpart();
        let mut native_refinement = BezierParameterRefinement2::new(other, &strict);
        let mut selected = self.clone();
        let mut is_other_root = false;
        let mut refinement_steps = 0_usize;
        loop {
            selected = match selected.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let refined_native = native_refinement.refine_to(refinement_steps);
            let BezierParameter2::Algebraic(other) = refined_native else {
                return selected.order_to_real(
                    refined_native
                        .scalar()
                        .expect("a refined root may become represented"),
                    policy,
                );
            };
            if compare_reals(
                &selected.data.upper,
                other.interval().start(),
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(
                other.interval().end(),
                &selected.data.lower,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
            if refinement_steps == 0 {
                // Reconstructed formula roots need not own a polynomial-root
                // authority, even when they equal this native endpoint.
                // Replay its defining relation in the retained field before
                // trying to separate equal values through deeper refinement.
                // A common divisor of the two defining relations decides
                // equality without the Sturm-Tarski sign chain of their
                // product, whose field coefficients grow at each remainder.
                is_other_root = match policy.bounded_exact_predicate_pass(|| {
                    selected.polynomial_vanishes(selection.polynomial().coefficients(), policy)
                })? {
                    Some(vanishes) => vanishes,
                    None => matches!(
                        policy.bounded_exact_predicate_pass(|| {
                            selected.polynomial_sign(selection.polynomial().coefficients(), policy)
                        })?,
                        Classification::Decided(RealSign::Zero),
                    ),
                };
            }
            if is_other_root
                && compare_reals(
                    selection.interval().start(),
                    &selected.data.lower,
                    &CurveContext::STRICT,
                ) == Some(std::cmp::Ordering::Less)
                && compare_reals(
                    &selected.data.upper,
                    selection.interval().end(),
                    &CurveContext::STRICT,
                ) == Some(std::cmp::Ordering::Less)
            {
                // A strict interior bracket and the native singleton proof
                // select this root, rather than another zero of the relation.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-parameter-comparison",
                    "native-root-replay",
                );
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            if policy.has_bounded_exact_predicate_budget() && refinement_steps >= 8 {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            if refinement_steps == 15 || refinement_steps >= 512 {
                // Equal formulas can require radical-sheet replay even when
                // their native endpoint is already a coefficient-field root.
                // Give that exact authority a turn after the cheap interval
                // pass, before spending hundreds of bisections on equality.
                // If it declines, the progressively retained brackets still
                // supply the full former refinement route.
                let exact = policy.strict_predicate_pass(|| {
                    match selected.promoted_bezier_parameter_complete(policy)? {
                        Classification::Decided(parameter) => {
                            parameter.cmp_by_refinement(refined_native, policy)
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    }
                })?;
                if let Classification::Decided(_) = exact {
                    return Ok(exact);
                }
                if refinement_steps >= 512 {
                    if policy.permits_approximate_512() {
                        policy.observe_approximate_512();
                        return Ok(Classification::Decided(std::cmp::Ordering::Equal));
                    }
                    return Ok(exact);
                }
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("recursive projective/Bezier refinement overflow".into())
                })?;
        }
    }

    pub(crate) fn promoted_bezier_parameter_complete(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        self.validate_policy(policy)?;
        if let Some(parameter) = self.data.projection.parameter.get() {
            return Ok(Classification::Decided(parameter.clone()));
        }
        let projected = policy.strict_predicate_pass(|| {
            if let Some(chart) = &self.data.projection.chart {
                let replayed = match chart.source.promoted_bezier_parameter_complete(policy)? {
                    Classification::Decided(source) => source.projective_image_unbounded(
                        &chart.numerator,
                        &chart.denominator,
                        policy,
                    )?,
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                };
                if let Classification::Decided(_) = replayed {
                    return Ok(replayed);
                }
                // Reusing a source chart is an additional replay route. Its
                // native conversion may be unresolved even when the mapped
                // polynomial has a simpler exact projection.
            }
            if let Some(authority) = self.polynomial_authority() {
                return authority.promoted_parameter(self, policy);
            }
            if let Some(authority) = self.monotone_authority() {
                return authority.promoted_mapped_parameter(policy);
            }
            let scalar = self
                .projective_scalar()
                .expect("a non-monotone recursive parameter owns a projective scalar");
            if let Some(value) = scalar.exact_real_value() {
                return Ok(Classification::Decided(BezierParameter2::Exact(value)));
            }
            match scalar.represented_value(policy)? {
                Classification::Decided(represented) => {
                    BezierParameter2::from_algebraic_root_representation_unbounded(
                        &represented,
                        policy,
                    )
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        })?;
        if let Classification::Decided(parameter) = projected {
            let _ = self.data.projection.parameter.set(parameter);
            return Ok(Classification::Decided(
                self.data
                    .projection
                    .parameter
                    .get()
                    .expect("strict projection was retained")
                    .clone(),
            ));
        }
        Ok(projected)
    }

    pub(crate) fn unit_complement(&self) -> Self {
        let Classification::Decided(parameter) = self
            .affine_image_unbounded(&Real::from(-1_i8), &Real::one(), &self.data.policy)
            .expect("the recursive unit complement is nondegenerate")
        else {
            unreachable!("the recursive unit complement has a strict scale")
        };
        parameter
    }

    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        match real_sign(scale, &CurveContext::STRICT) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if let Some(authority) = self.monotone_authority() {
            let authority = match authority.composed_map(
                &[offset.clone(), scale.clone()],
                &[Real::one(), Real::zero()],
                policy,
            )? {
                Classification::Decided(authority) => authority,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Self::from_mapped_monotone(authority, policy);
        }
        if self.polynomial_authority().is_some() {
            return self.projective_image_unbounded(
                &[offset.clone(), scale.clone()],
                &[Real::one(), Real::zero()],
                policy,
            );
        }
        let scalar = self
            .projective_scalar()
            .expect("a non-monotone recursive parameter owns a projective scalar");
        let numerator = scalar
            .numerator
            .scale(scale)
            .and_then(|numerator| {
                scalar
                    .denominator
                    .scale(offset)
                    .and_then(|constant| numerator.add(&constant))
            })
            .ok_or_else(|| {
                CurveError::Topology("a recursive affine parameter crossed retained fields".into())
            })?;
        Self::new(
            RecursiveQuadraticProjectiveScalar {
                numerator,
                denominator: scalar.denominator.clone(),
            },
            policy,
        )
    }

    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        let derivative = &numerator[1] * &denominator[0] - &numerator[0] * &denominator[1];
        match real_sign(&derivative, &CurveContext::STRICT) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        if let Some(authority) = self.monotone_authority() {
            let authority = match authority.composed_map(numerator, denominator, policy)? {
                Classification::Decided(authority) => authority,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Self::from_mapped_monotone(authority, policy);
        }
        if self.polynomial_authority().is_some() {
            let denominator_sign =
                policy.strict_predicate_pass(|| self.polynomial_sign(denominator, policy))?;
            match denominator_sign {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let mut refinement_steps = self.data.refinement_steps;
            let (refined, first, second) = loop {
                let refined = match policy
                    .strict_predicate_pass(|| self.refined(refinement_steps, policy))?
                {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let map = |source: &Real| -> CurveResult<Option<(RealSign, Real)>> {
                    let mapped_denominator = &denominator[0] + &denominator[1] * source;
                    let Some(sign @ (RealSign::Positive | RealSign::Negative)) =
                        real_sign(&mapped_denominator, &CurveContext::STRICT)
                    else {
                        return Ok(None);
                    };
                    Ok(Some((
                        sign,
                        ((&numerator[0] + &numerator[1] * source) / mapped_denominator)?,
                    )))
                };
                if let (Some((first_sign, first)), Some((second_sign, second))) =
                    (map(&refined.data.lower)?, map(&refined.data.upper)?)
                    && first_sign == second_sign
                {
                    break (refined, first, second);
                }
                refinement_steps = refinement_steps
                    .checked_mul(2)
                    .and_then(|steps| steps.checked_add(1))
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "recursive local projective refinement overflow".into(),
                        )
                    })?;
            };
            let (lower, upper) =
                if real_sign(&derivative, &CurveContext::STRICT) == Some(RealSign::Positive) {
                    (first, second)
                } else {
                    (second, first)
                };
            let (source, numerator, denominator) = match &self.data.projection.chart {
                Some(chart) => {
                    let compose = |row: &[Real; 2]| {
                        std::array::from_fn(|i| {
                            &row[0] * &chart.denominator[i] + &row[1] * &chart.numerator[i]
                        })
                    };
                    (
                        chart.source.clone(),
                        compose(numerator),
                        compose(denominator),
                    )
                }
                None => (self.clone(), numerator.clone(), denominator.clone()),
            };
            let (numerator, denominator) =
                crate::bezier_parameter::normalized_projective_chart(numerator, denominator);
            if numerator[0].zero_status() == ZeroKnowledge::Zero
                && denominator[1].zero_status() == ZeroKnowledge::Zero
                && numerator[1] == denominator[0]
            {
                // The composed bijection is the identity. Keep the original
                // selected root and its shared certificates, without growing
                // either a construction chain or polynomial common scale.
                if lower == source.data.lower && upper == source.data.upper {
                    return Ok(Classification::Decided(source));
                }
                // Refinement performed between inverse maps still belongs
                // to this root. Keep its tighter certified bracket, while
                // reusing the original relation and projection authority.
                return Ok(Classification::Decided(Self {
                    data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                        projection: source.data.projection.clone(),
                        authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                            authority: source
                                .polynomial_authority()
                                .expect("an identity chart retains its polynomial source")
                                .clone(),
                            endpoint_signs: if lower == upper {
                                [RealSign::Zero; 2].map(OnceLock::from)
                            } else {
                                std::array::from_fn(|_| OnceLock::new())
                            },
                        },
                        lower,
                        upper,
                        refinement_steps: refined
                            .data
                            .refinement_steps
                            .max(source.data.refinement_steps),
                        identity: source.data.identity.clone(),
                        line_branch: source.data.line_branch,
                        policy: source.data.policy,
                    }),
                }));
            }
            let source_authority = source
                .polynomial_authority()
                .expect("a polynomial chart retains its original polynomial root");
            let Some(coefficients) =
                source_authority.projective_image_coefficients(&numerator, &denominator)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let field = source_authority.field.clone();
            let projection = Arc::new(BezierRecursiveParameterProjection2 {
                parameter: OnceLock::new(),
                chart: Some(Arc::new(BezierRecursiveParameterChart2 {
                    source,
                    numerator,
                    denominator,
                })),
            });
            // The transformed polynomial can have a different orientation.
            // Only an exact point bracket preserves its zero endpoint signs.
            let endpoint_signs = if lower == upper {
                [RealSign::Zero; 2].map(OnceLock::from)
            } else {
                std::array::from_fn(|_| OnceLock::new())
            };
            return Ok(Classification::Decided(Self {
                data: Arc::new(BezierRecursiveProjectiveParameterData2 {
                    projection,
                    authority: BezierRecursiveProjectiveParameterAuthority2::Polynomial {
                        authority: Arc::new(BezierRecursivePolynomialParameterAuthority2::new(
                            field,
                            coefficients,
                        )),
                        endpoint_signs,
                    },
                    lower,
                    upper,
                    refinement_steps: refined.data.refinement_steps,
                    identity: None,
                    line_branch: 0,
                    policy: policy.retained_object_policy(),
                }),
            }));
        }
        let scalar = self
            .projective_scalar()
            .expect("a non-monotone recursive parameter owns a projective scalar");
        let combine = |coefficients: &[Real; 2]| {
            scalar
                .denominator
                .scale(&coefficients[0])?
                .add(&scalar.numerator.scale(&coefficients[1])?)
        };
        let Some(mapped_numerator) = combine(numerator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(mut mapped_denominator) = combine(denominator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut mapped_numerator = mapped_numerator;
        match mapped_denominator.sign(&policy.strict_counterpart())? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Negative) => {
                mapped_numerator = mapped_numerator
                    .scale(&Real::from(-1_i8))
                    .expect("recursive projective negation preserves its field");
                mapped_denominator = mapped_denominator
                    .scale(&Real::from(-1_i8))
                    .expect("recursive projective negation preserves its field");
            }
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Self::new(
            RecursiveQuadraticProjectiveScalar {
                numerator: mapped_numerator,
                denominator: mapped_denominator,
            },
            policy,
        )
    }

    pub(crate) fn finite_projection_interval(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Real, Real, Real)>> {
        let refined = match self.refined(refinement_steps, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let lower = refined.data.lower.clone();
        let upper = refined.data.upper.clone();
        let representative = ((&lower + &upper) / Real::from(2_u8))?;
        Ok(Classification::Decided((lower, representative, upper)))
    }
}

impl BezierRecursiveQuadraticLineParameterMapSystem2 {
    pub(super) fn contact(
        &self,
        branch: i8,
    ) -> CurveResult<&BezierRecursiveQuadraticLineContactSystem2> {
        self.contacts
            .iter()
            .find(|contact| contact.branch == branch)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive quadratic line contact lost its retained branch".into(),
                )
            })
    }

    pub(super) fn contact_order_to_real(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let Some(order) = algebraic_cusp_semicircle_endpoint_contact_order(
            contact.cusp_location,
            parameter,
            policy,
        ) {
            return Ok(order);
        }
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let retained = self.contact(contact.branch)?;
        let one_minus = Real::one() - parameter;
        let parameter_denominator = &one_minus * &one_minus + parameter * parameter;
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let predicate = retained
            .diameter
            .scale(&parameter_denominator)
            .and_then(|diameter| {
                retained
                    .radius_squared_denominator
                    .scale(&radial_coefficient)
                    .and_then(|radius| diameter.subtract(&radius))
            })
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive angular predicate exceeded its retained field budget".into(),
                )
            })?;
        Ok(predicate.sign(policy)?.map(|sign| match sign {
            RealSign::Positive => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Negative => std::cmp::Ordering::Greater,
        }))
    }

    /// Signs an exact point against the support-center/contact radial in the
    /// contact map's existing quadratic tower. The contact and center were
    /// normalized together when the map was authored, so this must not route
    /// through the general three-carrier field merge.
    pub(super) fn radial_oriented_side_to_exact_point(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let retained = self.contact(contact.branch)?;
        let field = retained.point.denominator.field();
        let Some(center) = self.center.lifted_to(&field) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let exact_x = RealInterval {
            lower: point.x().clone(),
            upper: point.x().clone(),
        };
        let exact_y = RealInterval {
            lower: point.y().clone(),
            upper: point.y().clone(),
        };
        let zero = Real::zero();
        let mut terminal_refined = false;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(contact_bounds), Classification::Decided(center_bounds)) = (
                retained.point.bounds_refined(refinement_steps),
                center.bounds_refined(refinement_steps),
            ) else {
                continue;
            };
            terminal_refined |= refinement_steps == 512;
            let center_x = real_interval_from_axis(&center_bounds, Axis2::X);
            let center_y = real_interval_from_axis(&center_bounds, Axis2::Y);
            let radial_x = real_interval_from_axis(&contact_bounds, Axis2::X).subtract(&center_x);
            let radial_y = real_interval_from_axis(&contact_bounds, Axis2::Y).subtract(&center_y);
            let target_x = exact_x.subtract(&center_x);
            let target_y = exact_y.subtract(&center_y);
            let Some(cross) = radial_x.multiply(&target_y).and_then(|first| {
                radial_y
                    .multiply(&target_x)
                    .map(|second| first.subtract(&second))
            }) else {
                continue;
            };
            if compare_reals(&cross.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Left));
            }
            if compare_reals(&cross.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::Right));
            }
            if compare_reals(&cross.lower, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(&cross.upper, &zero, &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal)
            {
                return Ok(Classification::Decided(crate::classify::LineSide::On));
            }
        }
        if terminal_refined && policy.permits_approximate_512() {
            policy.observe_approximate_512();
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        let Some(point) = (|| {
            Some(BezierRecursiveQuadraticProjectivePoint2 {
                x: field.constant(point.x().clone())?,
                y: field.constant(point.y().clone())?,
                denominator: field.constant(Real::one())?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        BezierRecursiveQuadraticProjectivePoint2::oriented_side(
            &center,
            &retained.point,
            &point,
            false,
            policy,
        )
    }

    pub(super) fn tangent_cross_dot_linear_combination_sign(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        cross_scale: &Real,
        dot_scale: &Real,
        turn: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let retained = self.contact(contact.branch)?;
        let expression = retained
            .tangent_cross
            .scale(cross_scale)
            .and_then(|cross| {
                retained
                    .angular_tangent
                    .scale(&(dot_scale * turn))
                    .and_then(|dot| cross.add(&dot))
            })
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive quadratic tangent predicate exceeded its field budget".into(),
                )
            })?;
        let sign = expression.sign(policy)?;
        Ok(sign)
    }

    pub(super) fn contact_bounds_refined(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
        refinement_steps: usize,
    ) -> Classification<Aabb2> {
        let Ok(contact) = self.contact(contact.branch) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        contact.point.bounds_refined(refinement_steps)
    }
}

impl BezierRecursiveQuadraticParallelEvaluation2 {
    pub(super) fn polynomial_value(
        &self,
        coefficients: &[RecursiveQuadraticValue],
    ) -> Option<RecursiveQuadraticValue> {
        self.speed_field
            .lift(&self.embedding.polynomial_value(coefficients)?)
    }

    pub(super) fn expression_value(
        &self,
        expression: &BezierRecursiveQuadraticParallelExpression2,
    ) -> Option<RecursiveQuadraticValue> {
        let rational = self.polynomial_value(&expression.rational)?;
        let radical = self.polynomial_value(&expression.radical)?;
        rational.add(&radical.multiply(&self.speed)?)
    }

    pub(super) fn source_box(
        &self,
        certificate: &BezierDenseSelectedCandidateBox2,
    ) -> Option<Vec<AlgebraicRootRepresentation>> {
        (certificate.sources.len() == self.embedding.source_axes.len()).then_some(())?;
        let mut sources = self.embedding.target_base.sources.clone();
        for (source, axis) in certificate.sources.iter().zip(&self.embedding.source_axes) {
            *sources.get_mut(*axis)? = source.clone();
        }
        *sources.get_mut(self.embedding.target_axis)? = certificate.candidate.clone();
        Some(sources)
    }
}

impl BezierRecursiveQuadraticParallelExpression2 {
    pub(super) fn new(
        rational: Vec<RecursiveQuadraticValue>,
        radical: Vec<RecursiveQuadraticValue>,
        speed_squared: Arc<[RecursiveQuadraticValue]>,
    ) -> Self {
        Self {
            rational,
            radical,
            speed_squared,
            squared_magnitude: Arc::new(OnceLock::new()),
        }
    }

    pub(super) fn squared_magnitude_difference(&self) -> Option<&[RecursiveQuadraticValue]> {
        if self.squared_magnitude.get().is_none() {
            let rational_squared =
                recursive_quadratic_polynomial_multiply(&self.rational, &self.rational)?;
            let radical_squared =
                recursive_quadratic_polynomial_multiply(&self.radical, &self.radical)?;
            let radical_speed =
                recursive_quadratic_polynomial_multiply(&radical_squared, &self.speed_squared)?;
            let magnitude =
                recursive_quadratic_polynomial_combine(&rational_squared, &radical_speed, true)?;
            let _ = self.squared_magnitude.set(magnitude);
        }
        self.squared_magnitude.get().map(Vec::as_slice)
    }

    /// Signs A+B*sqrt(S) through the retained parameter's polynomial
    /// authority. Strict positive speed and component signs select the
    /// authored sheet; a squared magnitude alone cannot certify cancellation.
    pub(super) fn sign_with_positive_speed(
        &self,
        policy: &CurveContext,
        mut polynomial_sign: impl FnMut(
            &[RecursiveQuadraticValue],
        ) -> CurveResult<Classification<RealSign>>,
    ) -> CurveResult<Classification<RealSign>> {
        match policy.strict_predicate_pass(|| polynomial_sign(&self.speed_squared))? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive parallel target had negative speed squared".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let rational_sign = polynomial_sign(&self.rational)?;
        let radical_sign = polynomial_sign(&self.radical)?;
        if let (Classification::Decided(rational), Classification::Decided(radical)) =
            (&rational_sign, &radical_sign)
            && let Some(sign) = same_positive_root_sheet_signs(*rational, *radical)
        {
            return Ok(Classification::Decided(sign));
        }
        let Some(magnitude) = self.squared_magnitude_difference() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let magnitude_sign = polynomial_sign(magnitude)?;
        Ok(positive_root_sum_sign_from_components(
            rational_sign,
            radical_sign,
            magnitude_sign,
        ))
    }
}

impl BezierRecursiveQuadraticProjectivePoint2 {
    /// Reuses the selected generators' scalar witnesses without projecting
    /// independent coordinate roots. The point already owns the nonzero
    /// denominator certificate; its original field remains authoritative.
    pub(super) fn exact_point_with_retained_witnesses(&self) -> Option<Point2> {
        let inverse = self
            .denominator
            .exact_real_value_with_retained_witnesses()?
            .inverse_ref_assuming_nonzero()
            .ok()?;
        Some(Point2::new(
            self.x.exact_real_value_with_retained_witnesses()? * &inverse,
            self.y.exact_real_value_with_retained_witnesses()? * inverse,
        ))
    }

    /// Publishes standalone exact coordinate roots only for consumers that
    /// cannot operate on this correlated projective point directly.
    pub(super) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
        let coordinate = |numerator: &RecursiveQuadraticValue| {
            RecursiveQuadraticProjectiveScalar {
                numerator: numerator.clone(),
                denominator: self.denominator.clone(),
            }
            .represented_value(policy)
        };
        let x = match coordinate(&self.x)? {
            Classification::Decided(x) => x,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let y = match coordinate(&self.y)? {
            Classification::Decided(y) => y,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided([x, y]))
    }

    pub(super) fn embedded_to_equivalent_field(
        &self,
        field: &RecursiveQuadraticField,
    ) -> Option<Self> {
        let source_field = self.denominator.field();
        let (target_base, embeddings) =
            source_field.extension_embeddings_to_equivalent_tower(field)?;
        Self {
            x: self
                .x
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?,
            y: self
                .y
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?,
            denominator: self
                .denominator
                .rebased_to_equivalent_base(target_base, &embeddings)?,
        }
        .lifted_to(field)
    }

    pub(super) fn rebased_to_equivalent_base(
        &self,
        target_base: Arc<RecursiveQuadraticBaseField>,
    ) -> Option<Self> {
        let source_field = self.denominator.field();
        let (target_field, embeddings) =
            source_field.rebased_to_equivalent_base(target_base.clone())?;
        Self {
            x: self
                .x
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?,
            y: self
                .y
                .rebased_to_equivalent_base(target_base.clone(), &embeddings)?,
            denominator: self
                .denominator
                .rebased_to_equivalent_base(target_base, &embeddings)?,
        }
        .lifted_to(&target_field)
    }

    pub(super) fn lifted_to(&self, field: &RecursiveQuadraticField) -> Option<Self> {
        Some(Self {
            x: field.lift(&self.x)?,
            y: field.lift(&self.y)?,
            denominator: field.lift(&self.denominator)?,
        })
    }

    pub(super) fn embedded_to(
        &self,
        field: &RecursiveQuadraticField,
        embeddings: &[RecursiveQuadraticExtensionEmbedding],
    ) -> Option<Self> {
        Some(Self {
            x: field.embed_value(&self.x, embeddings)?,
            y: field.embed_value(&self.y, embeddings)?,
            denominator: field.embed_value(&self.denominator, embeddings)?,
        })
    }

    /// Embeds two projective points in their least retained common quadratic
    /// tower. Ancestor fields only gain zero radical coefficients; divergent
    /// descendants append their already-certified positive generators.
    pub(super) fn joined_pair(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(RecursiveQuadraticField, Self, Self)>>> {
        let first_field = self.denominator.field();
        if let Some(other) = other.lifted_to(&first_field) {
            return Ok(Classification::Decided(Some((
                first_field,
                self.clone(),
                other,
            ))));
        }
        let second_field = other.denominator.field();
        if let Some(first) = self.lifted_to(&second_field) {
            return Ok(Classification::Decided(Some((
                second_field,
                first,
                other.clone(),
            ))));
        }
        let (joined, embeddings) = match first_field.joined_with(&second_field, policy)? {
            Classification::Decided(Some(joined)) => joined,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (Some(first), Some(second)) = (
            self.lifted_to(&joined),
            other.embedded_to(&joined, &embeddings),
        ) else {
            return Ok(Classification::Decided(None));
        };
        Ok(Classification::Decided(Some((
            joined.clone(),
            first,
            second,
        ))))
    }

    /// Signs `(end-start) x (point-start)` without materializing affine
    /// coordinates. Projective denominators in this authority are positive,
    /// so their product cannot reverse the oriented-area sign.
    pub(super) fn oriented_side(
        start: &Self,
        end: &Self,
        point: &Self,
        certified_nonzero: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let (field, mut start, mut end) = match start.joined_pair(end, policy)? {
            Classification::Decided(Some(joined)) => joined,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point = if let Some(point) = point.lifted_to(&field) {
            point
        } else {
            let point_field = point.denominator.field();
            if let (Some(lifted_start), Some(lifted_end)) =
                (start.lifted_to(&point_field), end.lifted_to(&point_field))
            {
                start = lifted_start;
                end = lifted_end;
                point.clone()
            } else {
                let (joined, embeddings) = match field.joined_with(&point_field, policy)? {
                    Classification::Decided(Some(joined)) => joined,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let (Some(lifted_start), Some(lifted_end), Some(embedded_point)) = (
                    start.lifted_to(&joined),
                    end.lifted_to(&joined),
                    point.embedded_to(&joined, &embeddings),
                ) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                start = lifted_start;
                end = lifted_end;
                embedded_point
            }
        };
        // When the selected axes already have compact scalar witnesses, retain
        // the homogeneous determinant's factored construction instead of
        // first expanding four projective differences into the recursive
        // tensor basis.  The fused scalar products preserve shared operands
        // and exact cancellations recognized by Hyperreal's structural layer.
        let exact = |point: &Self| {
            Some([
                point.x.exact_real_value_with_retained_witnesses()?,
                point.y.exact_real_value_with_retained_witnesses()?,
                point
                    .denominator
                    .exact_real_value_with_retained_witnesses()?,
            ])
        };
        if let (Some(start), Some(end), Some(point)) = (exact(&start), exact(&end), exact(&point)) {
            let direction_x = Real::diff_of_products(&end[0], &start[2], &start[0], &end[2]);
            let direction_y = Real::diff_of_products(&end[1], &start[2], &start[1], &end[2]);
            let point_x = Real::diff_of_products(&point[0], &start[2], &start[0], &point[2]);
            let point_y = Real::diff_of_products(&point[1], &start[2], &start[1], &point[2]);
            let cross = Real::diff_of_products(&direction_x, &point_y, &direction_y, &point_x);
            let minimum_precision = if policy.has_bounded_exact_predicate_budget() {
                -128
            } else {
                -512
            };
            let sign = cross
                .immediate_sign()
                .or_else(|| cross.certified_sign_until(minimum_precision).sign())
                .or_else(|| {
                    (!policy.has_bounded_exact_predicate_budget())
                        .then(|| real_sign(&cross, policy))
                        .flatten()
                });
            if let Some(sign) = sign
                && (!certified_nonzero || sign != RealSign::Zero)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-side",
                    "factored-compact-real-determinant",
                );
                return Ok(Classification::Decided(
                    crate::classify::LineSide::from_real_sign(sign),
                ));
            }
        }
        let Some((direction_x, direction_y, _)) = end.difference_numerators(&start) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((point_x, point_y, _)) = point.difference_numerators(&start) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(cross) = direction_x.multiply(&point_y).and_then(|first| {
            direction_y
                .multiply(&point_x)
                .and_then(|second| first.subtract(&second))
        }) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let sign = if certified_nonzero {
            // A separate exact topological certificate already excludes the
            // zero sheet. Interval refinement is therefore complete and
            // avoids forming a potentially enormous tensor norm merely to
            // rediscover non-equality.
            cross.sign_with_nonzero_certificate()?
        } else {
            cross.sign(policy)?
        };
        Ok(sign.map(crate::classify::LineSide::from_real_sign))
    }

    pub(super) fn transformed_affine(
        &self,
        m00: &Real,
        m01: &Real,
        m10: &Real,
        m11: &Real,
        tx: &Real,
        ty: &Real,
    ) -> Option<Self> {
        let x = self
            .x
            .scale(m00)?
            .add(&self.y.scale(m01)?)?
            .add(&self.denominator.scale(tx)?)?;
        let y = self
            .x
            .scale(m10)?
            .add(&self.y.scale(m11)?)?
            .add(&self.denominator.scale(ty)?)?;
        Some(Self {
            x,
            y,
            denominator: self.denominator.clone(),
        })
    }

    /// Applies `C + a(P-C) + b J(P-C) + T` in the retained recursive field.
    /// The result remains projective and shares the field tower; no Cartesian
    /// primitive element or independent coordinate root is introduced.
    pub(super) fn rotated_radial_image(
        &self,
        center: &Self,
        radial_scale: &Real,
        perpendicular_scale: &Real,
        translation_x: &Real,
        translation_y: &Real,
    ) -> Option<Self> {
        let field = self.denominator.field();
        let center = center.lifted_to(&field)?;
        let denominator = self.denominator.multiply(&center.denominator)?;
        let point_x = self.x.multiply(&center.denominator)?;
        let point_y = self.y.multiply(&center.denominator)?;
        let center_x = center.x.multiply(&self.denominator)?;
        let center_y = center.y.multiply(&self.denominator)?;
        let one_minus_radial = Real::one() - radial_scale;
        let x = point_x
            .scale(radial_scale)?
            .add(&point_y.scale(&(-perpendicular_scale.clone()))?)?
            .add(&center_x.scale(&one_minus_radial)?)?
            .add(&center_y.scale(perpendicular_scale)?)?
            .add(&denominator.scale(translation_x)?)?;
        let y = point_x
            .scale(perpendicular_scale)?
            .add(&point_y.scale(radial_scale)?)?
            .add(&center_x.scale(&(-perpendicular_scale.clone()))?)?
            .add(&center_y.scale(&one_minus_radial)?)?
            .add(&denominator.scale(translation_y)?)?;
        Some(Self { x, y, denominator })
    }

    pub(super) fn same_field_axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let (first, second) = match axis {
            Axis2::X => (&self.x, &other.x),
            Axis2::Y => (&self.y, &other.y),
        };
        // Most topology-event comparisons are strict and well separated.
        // Prove those from certified projective intervals before expanding a
        // deep recursive quadratic cross-product merely to recover its sign.
        // Equality and overlapping enclosures retain the complete exact path
        // below.
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
                break;
            }
            let coordinate_interval =
                |numerator: &RecursiveQuadraticValue, denominator: &RecursiveQuadraticValue| {
                    numerator
                        .interval(refinement_steps)?
                        .divide(&denominator.interval(refinement_steps)?)
                };
            let (Some(first), Some(second)) = (
                coordinate_interval(first, &self.denominator),
                coordinate_interval(second, &other.denominator),
            ) else {
                continue;
            };
            if compare_reals(&first.upper, &second.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-axis-order",
                    "interval-separated",
                );
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(&second.upper, &first.lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "recursive-projective-axis-order",
                    "interval-separated",
                );
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
        }
        if policy.has_bounded_exact_predicate_budget() {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let numerator = if Arc::ptr_eq(&self.denominator.data, &other.denominator.data) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-axis-order",
                "shared-denominator",
            );
            first.subtract(second)
        } else {
            first.multiply(&other.denominator).and_then(|first| {
                second
                    .multiply(&self.denominator)
                    .and_then(|second| first.subtract(&second))
            })
        };
        let Some(numerator) = numerator else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(numerator.sign(policy)?.map(|sign| match sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }

    /// Compares two retained projective points. The ancestor case only adds
    /// zero radical coefficients. Divergent descendants are embedded in a
    /// transient common tower containing each positive quadratic generator.
    pub(super) fn axis_order(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
        let first_field = self.denominator.field();
        if let Some(other) = other.lifted_to(&first_field) {
            return self.same_field_axis_order(&other, axis, policy).map(Some);
        }
        let second_field = other.denominator.field();
        if let Some(first) = self.lifted_to(&second_field) {
            return first.same_field_axis_order(other, axis, policy).map(Some);
        }
        let (joined, embeddings) = match first_field.joined_with(&second_field, policy)? {
            Classification::Decided(Some(joined)) => joined,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let (Some(first), Some(second)) = (
            self.lifted_to(&joined),
            other.embedded_to(&joined, &embeddings),
        ) else {
            return Ok(Some(Classification::Uncertain(
                UncertaintyReason::Unsupported,
            )));
        };
        let order = first.same_field_axis_order(&second, axis, policy)?;
        Ok(Some(order))
    }

    pub(super) fn difference_numerators(
        &self,
        other: &Self,
    ) -> Option<(
        RecursiveQuadraticValue,
        RecursiveQuadraticValue,
        RecursiveQuadraticValue,
    )> {
        if Arc::ptr_eq(&self.denominator.data, &other.denominator.data) {
            return Some((
                self.x.subtract(&other.x)?,
                self.y.subtract(&other.y)?,
                self.denominator.clone(),
            ));
        }
        let x = self
            .x
            .multiply(&other.denominator)?
            .subtract(&other.x.multiply(&self.denominator)?)?;
        let y = self
            .y
            .multiply(&other.denominator)?
            .subtract(&other.y.multiply(&self.denominator)?)?;
        let denominator = self.denominator.multiply(&other.denominator)?;
        Some((x, y, denominator))
    }

    pub(super) fn linear_numerator(
        &self,
        x_factor: &Real,
        y_factor: &Real,
        value: &Real,
    ) -> Option<RecursiveQuadraticValue> {
        self.x
            .scale(x_factor)?
            .add(&self.y.scale(y_factor)?)?
            .subtract(&self.denominator.scale(value)?)
    }

    pub(super) fn bounds_refined(&self, refinement_steps: usize) -> Classification<Aabb2> {
        // This point already certifies its denominator strictly positive.
        // Replay the correlated scalar coordinates before interval expansion.
        let scalar_bounds = || {
            let point = self.exact_point_with_retained_witnesses()?;
            let precision = -(refinement_steps.max(64).min(i32::MAX as usize) as i32);
            let [x_lower, x_upper] = point
                .x()
                .certified_rational_interval(precision)?
                .map(Real::new);
            let [y_lower, y_upper] = point
                .y()
                .certified_rational_interval(precision)?
                .map(Real::new);
            Some(Aabb2::new_unchecked(
                Point2::new(x_lower, y_lower),
                Point2::new(x_upper, y_upper),
            ))
        };
        if let Some(bounds) = scalar_bounds() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "recursive-projective-bounds",
                "retained-scalar",
            );
            return Classification::Decided(bounds);
        }
        let (Some(x), Some(y), Some(denominator)) = (
            self.x.interval(refinement_steps),
            self.y.interval(refinement_steps),
            self.denominator.interval(refinement_steps),
        ) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let (Some(x), Some(y)) = (x.divide(&denominator), y.divide(&denominator)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }
}

impl BezierRecursiveQuadraticTargetEmbedding2 {
    pub(super) fn value(&self, value: &RecursiveQuadraticValue) -> Option<RecursiveQuadraticValue> {
        recursive_rebase_value_preserving_base(
            value,
            &self.source_base,
            &self.target_base,
            &self.source_axes,
            &self.extensions,
        )
    }

    pub(super) fn polynomial_value(
        &self,
        coefficients: &[RecursiveQuadraticValue],
    ) -> Option<RecursiveQuadraticValue> {
        let mut value = self.field.constant(Real::zero())?;
        for coefficient in coefficients.iter().rev() {
            value = value
                .multiply(&self.parameter)?
                .add(&self.value(coefficient)?)?;
        }
        Some(value)
    }
}

/// Reuses a parameter already represented in this field. This optional
/// import never adjoins a generator, rebases coefficients or projects a
/// selected root into a new scalar representation.
pub(super) fn recursive_field_retained_parameter_value(
    field: &RecursiveQuadraticField,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RecursiveQuadraticValue>> {
    let selected = parameter.as_selected_fiber();
    if let Some(selected) = selected {
        selected.validate_policy(policy)?;
    }
    let retained = selected.and_then(|selected| selected.retained_bezier_parameter());
    if let Some(native) = parameter.as_bezier_parameter().or(retained.as_ref()) {
        if matches!(native, BezierParameter2::Algebraic(_)) {
            return Ok(field.retained_root_value(&bezier_parameter_root_representation(native)));
        }
        return Ok(native
            .scalar()
            .and_then(|value| field.constant(value.clone())));
    }
    let projective = parameter
        .as_recursive_projective()
        .or_else(|| selected.and_then(|selected| selected.data.representations.projective.get()));
    if let Some(projective) = projective {
        projective.validate_policy(policy)?;
        return Ok(projective.projective_scalar().and_then(|scalar| {
            scalar
                .denominator
                .is_coefficientwise_stored_one()
                .then(|| field.lift(&scalar.numerator))
                .flatten()
        }));
    }
    Ok(parameter
        .scalar()
        .and_then(|value| field.constant(value.clone())))
}

pub(super) fn foreign_embedding_projective_point(
    embedding: &RecursiveQuadraticForeignBaseEmbedding,
    point: &BezierRecursiveQuadraticProjectivePoint2,
    field: &RecursiveQuadraticField,
) -> Option<BezierRecursiveQuadraticProjectivePoint2> {
    Some(BezierRecursiveQuadraticProjectivePoint2 {
        x: embedding.value(&point.x, field)?,
        y: embedding.value(&point.y, field)?,
        denominator: embedding.value(&point.denominator, field)?,
    })
}

/// Forms `self - other` in the least retained common quadratic tower.
/// This is the exact comparison path for independently rebuilt contacts:
/// interval refinement can separate unequal values, but equal transformed
/// roots require their authored positive generators to be joined rather
/// than globally projected.
pub(super) fn projective_scalar_joined_projective_difference(
    scalar: &RecursiveQuadraticProjectiveScalar,
    other: &RecursiveQuadraticProjectiveScalar,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RecursiveQuadraticValue>>> {
    let first_field = scalar.denominator.field();
    let second_field = other.denominator.field();
    let Some(first_zero) = first_field.constant(Real::zero()) else {
        return Ok(Classification::Decided(None));
    };
    let first = BezierRecursiveQuadraticProjectivePoint2 {
        x: scalar.numerator.clone(),
        y: first_zero,
        denominator: scalar.denominator.clone(),
    };
    let Some(second_zero) = second_field.constant(Real::zero()) else {
        return Ok(Classification::Decided(None));
    };
    let second = BezierRecursiveQuadraticProjectivePoint2 {
        x: other.numerator.clone(),
        y: second_zero,
        denominator: other.denominator.clone(),
    };
    let joined = match first.joined_pair(&second, policy)? {
        Classification::Decided(Some(joined)) => joined,
        Classification::Decided(None) => {
            let (first_base, _) = first_field.base_and_extension_path();
            let Some(second) = second.rebased_to_equivalent_base(first_base) else {
                return Ok(Classification::Decided(None));
            };
            match first.joined_pair(&second, policy)? {
                Classification::Decided(Some(joined)) => joined,
                Classification::Decided(None) => {
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
    let (_, first, second) = joined;
    let Some(second_product) = second.x.multiply(&first.denominator) else {
        return Ok(Classification::Decided(None));
    };
    let Some(difference) = first
        .x
        .multiply(&second.denominator)
        .and_then(|first| first.subtract(&second_product))
    else {
        return Ok(Classification::Decided(None));
    };
    Ok(Classification::Decided(Some(difference)))
}

/// General common-field comparison when the two towers were rebuilt over
/// different dense base allocations (for example after an exact
/// similarity). The merge imports each already-selected positive base
/// root and extension; it does not choose a new algebraic sheet.
pub(super) fn projective_scalar_merged_projective_difference(
    scalar: &RecursiveQuadraticProjectiveScalar,
    other: &RecursiveQuadraticProjectiveScalar,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RecursiveQuadraticValue>>> {
    let first_field = scalar.denominator.field();
    let second_field = other.denominator.field();
    let first = BezierRecursiveQuadraticProjectivePoint2 {
        x: scalar.numerator.clone(),
        y: first_field
            .constant(Real::zero())
            .ok_or_else(|| CurveError::Topology("a recursive scalar lost its field zero".into()))?,
        denominator: scalar.denominator.clone(),
    };
    let second = BezierRecursiveQuadraticProjectivePoint2 {
        x: other.numerator.clone(),
        y: second_field
            .constant(Real::zero())
            .ok_or_else(|| CurveError::Topology("a recursive scalar lost its field zero".into()))?,
        denominator: other.denominator.clone(),
    };
    let (_, mut first, second) =
        match recursive_merge_projective_point_fields(&first_field, &[first], &second, policy)? {
            Classification::Decided(Some(merged)) => merged,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let first = first
        .pop()
        .expect("one recursive scalar merge preserves its first operand");
    let difference = first.x.multiply(&second.denominator).and_then(|value| {
        second
            .x
            .multiply(&first.denominator)
            .and_then(|other| value.subtract(&other))
    });
    Ok(Classification::Decided(difference))
}
