//! Selected fibers of algebraic parameters: authorities and fiber-parameter evaluation, ordering and transport.

use super::*;

impl BezierAlgebraicSelectedFiberAuthority2 {
    pub(super) fn new(
        incidence: BivariatePolynomial,
        retained_parameter: BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicSelectedFiberAuthorityData2 {
                incidence,
                retained_parameter,
                policy: policy.retained_object_policy(),
                retained_refinement_64: OnceLock::new(),
                incidence_has_parameter_diagonal: OnceLock::new(),
                root_refiner: Mutex::new(None),
            }),
        }
    }

    pub(super) fn retained_parameter_refined(&self, refinement_steps: usize) -> BezierParameter2 {
        if refinement_steps <= 64 {
            return self
                .data
                .retained_refinement_64
                .get_or_init(|| {
                    Arc::new(
                        BezierParameter2::Algebraic(self.data.retained_parameter.clone())
                            .refined_isolating_interval(64, &CurveContext::STRICT),
                    )
                })
                .as_ref()
                .clone();
        }
        BezierParameter2::Algebraic(self.data.retained_parameter.clone())
            .refined_isolating_interval(refinement_steps, &CurveContext::STRICT)
    }

    pub(super) fn parameter(
        &self,
        root: IsolatedRootInterval,
    ) -> BezierAlgebraicSelectedFiberParameter2 {
        BezierAlgebraicSelectedFiberParameter2 {
            data: Arc::new(BezierAlgebraicSelectedFiberParameterData2 {
                authority: self.clone(),
                root,
                representations: Arc::new(BezierSelectedFiberRepresentations2::default()),
            }),
        }
    }

    pub(super) fn exact_parameter(
        retained_parameter: BezierAlgebraicParameter2,
        value: Real,
        policy: &CurveContext,
    ) -> BezierAlgebraicSelectedFiberParameter2 {
        Self::new(
            bivariate_outer_product(&[Real::one()], &[(-value.clone()), Real::one()]),
            retained_parameter,
            policy,
        )
        .parameter(IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value),
            distinct_root_count: 1,
        })
    }

    /// Retains an ordinary exact scalar as one root in the selected base
    /// field without constructing a compositum or degree-multiplied norm.
    ///
    /// The univariate root relation is independent of `alpha`, but keeping it
    /// under the selected authority lets later point and tangent predicates
    /// evaluate the scalar together with the already-selected circle frame.
    pub(super) fn from_bezier_parameter(
        retained_parameter: BezierAlgebraicParameter2,
        parameter: BezierParameter2,
        policy: &CurveContext,
    ) -> BezierAlgebraicSelectedFiberParameter2 {
        match parameter {
            BezierParameter2::Exact(value) => {
                Self::exact_parameter(retained_parameter, value, policy)
            }
            BezierParameter2::Algebraic(parameter) => {
                let root = IsolatedRootInterval {
                    lower: parameter.interval().start().clone(),
                    upper: parameter.interval().end().clone(),
                    exact_root: None,
                    distinct_root_count: 1,
                };
                let selected = Self::new(
                    bivariate_outer_product(&[Real::one()], parameter.polynomial().coefficients()),
                    retained_parameter,
                    policy,
                )
                .parameter(root);
                selected.retain_certified_parameter(BezierParameter2::Algebraic(parameter));
                selected
            }
        }
    }
}

impl BezierAlgebraicSelectedFiberParameter2 {
    pub(super) fn validate_policy(&self, policy: &CurveContext) -> CurveResult<()> {
        if !policy.accepts_retained_policy(self.data.authority.data.policy) {
            return Err(CurveError::Topology(
                "a selected-fiber scalar crossed predicate policies".into(),
            ));
        }
        Ok(())
    }

    pub(super) fn root(&self) -> &IsolatedRootInterval {
        &self.data.root
    }

    /// Retains an alternate scalar representation after a strict equality
    /// proof. The defining fiber and singleton remain the primary authority.
    pub(super) fn retain_certified_parameter(&self, parameter: BezierParameter2) {
        let _ = self.data.representations.bezier.set(parameter);
    }

    /// Returns the represented scalar when exact fiber isolation recovered one.
    pub(crate) fn represented_value(&self) -> Option<&Real> {
        self.data.root.exact_root.as_ref()
    }

    /// Reuses an already proved native representation without projecting the
    /// fiber. Bounded consumers may import this evidence even when constructing
    /// a new global parameter would exceed their budget.
    pub(super) fn retained_bezier_parameter(&self) -> Option<BezierParameter2> {
        self.data.representations.bezier.get().cloned().or_else(|| {
            self.represented_value()
                .cloned()
                .map(BezierParameter2::Exact)
        })
    }

    /// Imports a linear or quadratic fiber root into its retained coefficient
    /// field. The original certified singleton selects the radical branch;
    /// neither the base root nor this scalar needs an independent global image.
    /// Successful imports are shared across clones and interval refinements.
    pub(super) fn recursive_projective_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<BezierRecursiveProjectiveParameter2>> {
        self.validate_policy(policy)?;
        if let Some(parameter) = self.data.representations.projective.get() {
            return Ok(Some(parameter.clone()));
        }
        if self.represented_value().is_some() || self.data.representations.bezier.get().is_some() {
            return Ok(None);
        }
        let incidence = &self.data.authority.data.incidence;
        let degree = incidence
            .coefficients
            .iter()
            .filter_map(|row| {
                row.iter()
                    .rposition(|coefficient| coefficient.zero_status() != ZeroKnowledge::Zero)
            })
            .max()
            .unwrap_or(0);
        if !(1..=2).contains(&degree) {
            return Ok(None);
        }
        // This optional import never turns a failed field construction or
        // bounded predicate into a restriction on the complete projection.
        let attempt = policy.bounded_exact_predicate_pass(|| -> CurveResult<Option<_>> {
            let source = certified_parameter_representation(
                &self.data.authority.data.retained_parameter,
                policy,
            );
            let Some(one) = DenseTensorPolynomial::from_axis_polynomial(1, 0, &[Real::one()])
            else {
                return Ok(None);
            };
            let Some(field) = RecursiveQuadraticField::base(vec![source], one.clone(), one) else {
                return Ok(None);
            };
            let RecursiveQuadraticField::Base(base) = &field else {
                unreachable!("a selected fiber begins at its retained base");
            };
            let coefficients = (0..=degree)
                .map(|power| {
                    let coefficients = incidence
                        .coefficients
                        .iter()
                        .map(|row| row.get(power).cloned().unwrap_or_else(Real::zero))
                        .collect::<Vec<_>>();
                    recursive_quadratic_rational_value(
                        base,
                        DenseTensorPolynomial::from_axis_polynomial(1, 0, &coefficients)?,
                    )
                })
                .collect::<Option<Vec<_>>>();
            let Some(coefficients) = coefficients else {
                return Ok(None);
            };
            let Some(roots) = recursive_quadratic_polynomial_projective_roots(
                &field,
                &coefficients,
                None,
                policy,
            )?
            else {
                return Ok(None);
            };
            let mut selected = None;
            for root in roots {
                let lower = root.order_to_real(&self.data.root.lower, policy)?;
                let upper = root.order_to_real(&self.data.root.upper, policy)?;
                let (Classification::Decided(lower), Classification::Decided(upper)) =
                    (lower, upper)
                else {
                    return Ok(None);
                };
                if lower != std::cmp::Ordering::Less && upper != std::cmp::Ordering::Greater {
                    if selected.is_some() {
                        return Ok(None);
                    }
                    selected = Some(root);
                }
            }
            let Some(selected) = selected else {
                return Ok(None);
            };
            Ok(
                match BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                    selected,
                    Some((self.data.root.lower.clone(), self.data.root.upper.clone())),
                    policy,
                )? {
                    Classification::Decided(parameter) => Some(parameter),
                    Classification::Uncertain(_) => None,
                },
            )
        });
        let Ok(Some(parameter)) = attempt else {
            return Ok(None);
        };
        let _ = self.data.representations.projective.set(parameter);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-fiber-parameter",
            "retained-projective-root",
        );
        Ok(self.data.representations.projective.get().cloned())
    }

    /// Materializes a rational function of this selected scalar when its
    /// exact fiber residue belongs to the retained base field.
    ///
    /// If `u` is selected by `F(alpha, u) = 0`, Hypersolve first reduces the
    /// numerator and denominator in `Q(alpha)[u] / (F)`. A fiber-independent
    /// residue is then represented directly as a rational image of `alpha`,
    /// avoiding both the global projection of `u` and subsequent degree
    /// multiplication by the source expression.
    pub(super) fn represented_retained_field_rational_value(
        &self,
        numerator: &[Real],
        denominator: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<AlgebraicRootRepresentation>>> {
        self.validate_policy(policy)?;
        let retained_root = policy.strict_predicate_pass(|| {
            parameter_representation(&self.data.authority.data.retained_parameter, policy)
        });
        let report = policy.strict_predicate_pass(|| {
            reduce_bivariate_rational_function_at_algebraic_parameter(
                &self.data.authority.data.incidence,
                &bivariate_outer_product(&[Real::one()], numerator),
                &bivariate_outer_product(&[Real::one()], denominator),
                CurveResultantParameter::First,
                &retained_root,
                policy.predicate_policy(),
            )
        });
        if report.certainty == PredicateCertainty::Approximate
            || report.status != AlgebraicFiberRationalReductionStatus::ReducedToRetainedField
        {
            return Ok(Classification::Decided(None));
        }
        let (Some(numerator), Some(denominator)) = (
            DenseTensorPolynomial::from_axis_polynomial(2, 0, &report.numerator_coefficients),
            DenseTensorPolynomial::from_axis_polynomial(2, 0, &report.denominator_coefficients),
        ) else {
            return Ok(Classification::Decided(None));
        };
        Ok(
            match represented_tensor_ratio(
                &numerator,
                &denominator,
                std::slice::from_ref(&retained_root),
            ) {
                Classification::Decided(value) => Classification::Decided(Some(value)),
                Classification::Uncertain(_) => Classification::Decided(None),
            },
        )
    }

    /// Projects one final polynomial image of the selected scalar without
    /// first constructing the degree-multiplied global scalar itself.
    ///
    /// `relation(u, z) = 0` may contain a squared radical branch; the caller
    /// must isolate `z` from bounds that retain the authored branch. This
    /// method supplies only exact construction evidence and deliberately
    /// treats every bounded or approximate outcome as a declined fast path.
    pub(super) fn represented_polynomial_image_eliminant(
        &self,
        relation: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Option<(Vec<Real>, AlgebraicRootRepresentation)>> {
        self.validate_policy(policy)?;
        let retained_root = policy.strict_predicate_pass(|| {
            parameter_representation(&self.data.authority.data.retained_parameter, policy)
        });
        let report = policy.strict_predicate_pass(|| {
            project_algebraic_fiber_polynomial_image(
                &self.data.authority.data.incidence,
                CurveResultantParameter::First,
                relation,
                CurveResultantParameter::Second,
                &retained_root,
                AlgebraicFiberPolynomialImageProjectionConfig {
                    max_fiber_degree: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_FIBER_DEGREE,
                    max_retained_degree: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_RETAINED_DEGREE,
                    max_image_degree_bound: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_DEGREE,
                },
                policy.predicate_policy(),
            )
        });
        if report.certainty == PredicateCertainty::Approximate {
            return Ok(None);
        }
        if let Some(factor) = &report.identically_zero_fiber_factor {
            match self.predicate_sign(factor, &policy.strict_counterpart())? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                    return Ok(None);
                }
            }
        }
        match report.status {
            AlgebraicFiberPolynomialImageProjectionStatus::Constructed
                if report.coefficients.len() > 1 =>
            {
                Ok(Some((report.coefficients, retained_root)))
            }
            AlgebraicFiberPolynomialImageProjectionStatus::InvalidEvidence => {
                Err(CurveError::InvalidBezierAlgebraicParameter)
            }
            AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            | AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroFiber
            | AlgebraicFiberPolynomialImageProjectionStatus::ConstantNonzeroFiber
            | AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation
            | AlgebraicFiberPolynomialImageProjectionStatus::UnsupportedCoefficient
            | AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded
            | AlgebraicFiberPolynomialImageProjectionStatus::Undecided => Ok(None),
        }
    }

    /// Retains one polynomial image as another exact fiber over the same
    /// selected base root.
    ///
    /// Hypersolve stops after the local norm in `Q(alpha)[u] / F(alpha,u)`;
    /// it does not construct the degree-multiplied global image scalar.  The
    /// returned relation is enumeration evidence only: callers must replay
    /// their authored equation against this particular selected `u` root.
    pub(super) fn retained_polynomial_image_relation(
        &self,
        relation: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierSelectedPolynomialImage2>>> {
        self.validate_policy(policy)?;
        let retained_root = policy.strict_predicate_pass(|| {
            parameter_representation(&self.data.authority.data.retained_parameter, policy)
        });
        let fiber_degree = self
            .data
            .authority
            .data
            .incidence
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default()
            .saturating_sub(1);
        let image_degree = relation
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default()
            .saturating_sub(1);
        let local_image_degree = fiber_degree.checked_mul(image_degree);
        let global_image_degree = local_image_degree.and_then(|degree| {
            degree.checked_mul(
                self.data
                    .authority
                    .data
                    .retained_parameter
                    .polynomial()
                    .degree(),
            )
        });
        let construct_global_schedule = global_image_degree
            .is_some_and(|degree| degree <= MAX_SELECTED_FIBER_GLOBAL_IMAGE_SCHEDULE_DEGREE);
        let project = |config| {
            policy.strict_predicate_pass(|| {
                if construct_global_schedule {
                    project_algebraic_fiber_polynomial_image(
                        &self.data.authority.data.incidence,
                        CurveResultantParameter::First,
                        relation,
                        CurveResultantParameter::Second,
                        &retained_root,
                        config,
                        hypersolve::PredicatePolicy::STRICT,
                    )
                } else {
                    project_algebraic_fiber_polynomial_image_relation(
                        &self.data.authority.data.incidence,
                        CurveResultantParameter::First,
                        relation,
                        CurveResultantParameter::Second,
                        &retained_root,
                        config,
                        hypersolve::PredicatePolicy::STRICT,
                    )
                }
            })
        };
        let mut report = project(AlgebraicFiberPolynomialImageProjectionConfig {
            max_fiber_degree: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_FIBER_DEGREE,
            max_retained_degree: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_RETAINED_DEGREE,
            max_image_degree_bound: MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_DEGREE,
        });
        if report.status == AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-fiber-polynomial-image-relation",
                "unbounded-cold-continuation",
            );
            report = project(AlgebraicFiberPolynomialImageProjectionConfig {
                max_fiber_degree: usize::MAX,
                max_retained_degree: usize::MAX,
                max_image_degree_bound: usize::MAX,
            });
        }
        if report.certainty == PredicateCertainty::Approximate {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        match report.status {
            AlgebraicFiberPolynomialImageProjectionStatus::Constructed => {
                // A degree-zero global norm is a certified nonzero constant:
                // it has no roots to schedule.  Let the retained relation
                // take the ordinary local-isolation path instead of trying to
                // construct a root polynomial from that constant.
                let schedule = (report.coefficients.len() > 1).then_some(report.coefficients);
                Ok(Classification::Decided(Some(
                    BezierSelectedPolynomialImage2 {
                        relation: report.retained_relation,
                        global_schedule: schedule,
                        identically_zero_source_factor: report.identically_zero_fiber_factor,
                        identically_zero_image_relation: false,
                    },
                )))
            }
            AlgebraicFiberPolynomialImageProjectionStatus::InvalidEvidence => {
                Err(CurveError::InvalidBezierAlgebraicParameter)
            }
            AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation => Ok(
                Classification::Decided(Some(BezierSelectedPolynomialImage2 {
                    relation: None,
                    global_schedule: None,
                    identically_zero_source_factor: report.identically_zero_fiber_factor,
                    identically_zero_image_relation: true,
                })),
            ),
            AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroFiber
            | AlgebraicFiberPolynomialImageProjectionStatus::ConstantNonzeroFiber => {
                Ok(Classification::Decided(None))
            }
            AlgebraicFiberPolynomialImageProjectionStatus::UnsupportedCoefficient
            | AlgebraicFiberPolynomialImageProjectionStatus::DegreeLimitExceeded => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            AlgebraicFiberPolynomialImageProjectionStatus::Undecided => {
                Ok(report.retained_relation.map_or_else(
                    || Classification::Uncertain(UncertaintyReason::Predicate),
                    |relation| {
                        Classification::Decided(Some(BezierSelectedPolynomialImage2 {
                            relation: Some(relation),
                            global_schedule: None,
                            identically_zero_source_factor: report.identically_zero_fiber_factor,
                            identically_zero_image_relation: false,
                        }))
                    },
                ))
            }
        }
    }

    /// Promotes this compact local scalar only when an exact construction
    /// requires an ordinary Bezier parameter.
    ///
    /// Boolean splitting and traversal deliberately keep the selected-fiber
    /// representation: it avoids the degree-multiplied projection polynomial.
    /// A true carrier switch can, however, need the scalar as the center or
    /// endpoint of another exact carrier. In that case the existing complete
    /// resultant/quotient projection enumerates the ordinary algebraic roots
    /// and this local authority selects the one identical root. No isolating
    /// midpoint or APPROXIMATE_512 value becomes construction evidence.
    pub(crate) fn promoted_bezier_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        self.project_bezier_parameter(
            MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
            policy,
        )
    }

    /// Completes exact scalar promotion after the bounded hot schedule declines.
    /// The cold continuation removes degree limits while retaining the original
    /// selected fiber and its finite isolator. No compact chart is constructed.
    pub(crate) fn promoted_bezier_parameter_complete(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        match self.promoted_bezier_parameter(policy)? {
            decided @ Classification::Decided(_) => return Ok(decided),
            Classification::Uncertain(_) => {}
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-scalar-promotion",
            "complete-projection",
        );
        self.project_bezier_parameter(
            usize::MAX,
            self.data
                .authority
                .data
                .retained_parameter
                .polynomial()
                .degree()
                .max(MAX_SELECTED_FIBER_QUOTIENT_DEGREE),
            policy,
        )
    }

    pub(super) fn project_bezier_parameter(
        &self,
        max_resultant_degree: usize,
        max_quotient_degree: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        self.validate_policy(policy)?;
        if let Some(parameter) = self.retained_bezier_parameter() {
            return Ok(Classification::Decided(parameter));
        }
        let range = CurveParameterRange2::new_validated(
            CurveParameter2::from(BezierParameter2::Exact(self.data.root.lower.clone())),
            CurveParameter2::from(BezierParameter2::Exact(self.data.root.upper.clone())),
        );
        let retained = &self.data.authority.data.retained_parameter;
        let represented = match retained.represented_exact_point_with_policy(policy)? {
            Classification::Decided(value) => value,
            Classification::Uncertain(_) => None,
        };
        let projection = policy.strict_predicate_pass(|| match represented {
            Some(value) => selected_parameter_fiber_parameters(
                &self.data.authority.data.incidence,
                &BezierParameter2::Exact(value),
                max_resultant_degree,
                max_quotient_degree,
                &range,
                policy,
            ),
            None => algebraic_selected_fiber_parameters_with_resultant_limit(
                &self.data.authority.data.incidence,
                retained,
                max_resultant_degree,
                max_quotient_degree,
                &range,
                policy,
            ),
        })?;
        let candidates = match projection {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.select_promoted_bezier_parameter(candidates, policy)
    }

    pub(super) fn select_promoted_bezier_parameter(
        &self,
        candidates: Vec<BezierParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        let mut selected = None;
        let mut uncertainty = None;
        for candidate in candidates {
            match self.cmp_bezier_parameter(&candidate, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    if selected.is_some() {
                        return Err(CurveError::Topology(
                            "one selected-fiber scalar matched multiple projected roots".into(),
                        ));
                    }
                    selected = Some(candidate);
                }
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                }
                Classification::Uncertain(reason) => uncertainty = Some(reason),
            }
        }
        Ok(match selected {
            Some(parameter) => Classification::Decided(parameter),
            None => Classification::Uncertain(uncertainty.unwrap_or(UncertaintyReason::Boundary)),
        })
    }

    pub(crate) fn unit_complement(&self) -> Self {
        let policy = self.data.authority.data.policy;
        let Classification::Decided(complement) = self
            .affine_image_unbounded(&Real::from(-1_i8), &Real::one(), &policy)
            .expect("the represented unit-complement chart is nondegenerate")
        else {
            unreachable!("the represented unit-complement chart has a strict sign")
        };
        complement
    }

    /// Applies `scale * parameter + offset` without constructing the selected
    /// scalar's global resultant.
    ///
    /// If `u` is selected by `F(alpha, u) = 0` and `v = scale*u + offset`,
    /// the returned authority selects `v` through
    /// `F(alpha, (v-offset)/scale) = 0`.  The represented affine coefficients
    /// and transformed isolator are exact construction evidence; an
    /// `APPROXIMATE_512` terminal equality decision is never used to choose
    /// this chart.
    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        let scale_sign = match real_sign(scale, &CurveContext::STRICT) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if scale == &Real::one() && offset.zero_status() == ZeroKnowledge::Zero {
            return Ok(Classification::Decided(self.clone()));
        }
        let inverse_scale = (Real::one() / scale)?;
        let inverse_offset = ((-offset.clone()) / scale)?;
        let authority = BezierAlgebraicSelectedFiberAuthority2::new(
            self.data.authority.data.incidence.substitute_affine(
                &Real::one(),
                &Real::zero(),
                &inverse_scale,
                &inverse_offset,
            ),
            self.data.authority.data.retained_parameter.clone(),
            &self.data.authority.data.policy,
        );
        let first = scale * &self.data.root.lower + offset;
        let second = scale * &self.data.root.upper + offset;
        let (lower, upper) = match scale_sign {
            RealSign::Positive => (first, second),
            RealSign::Negative => (second, first),
            RealSign::Zero => unreachable!(),
        };
        Ok(Classification::Decided(
            authority.parameter(IsolatedRootInterval {
                lower,
                upper,
                exact_root: self
                    .data
                    .root
                    .exact_root
                    .as_ref()
                    .map(|root| scale * root + offset),
                distinct_root_count: self.data.root.distinct_root_count,
            }),
        ))
    }

    /// Applies `(n0 + n1*u) / (d0 + d1*u)` without constructing the selected
    /// scalar's global resultant.
    ///
    /// The forward denominator and constant derivative sign are certified
    /// under a strict predicate pass.  The local incidence is composed with
    /// the exact inverse chart and its denominator is cleared symbolically;
    /// refined isolating endpoints provide an outward interval only, never a
    /// representative construction value.
    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        policy.strict_predicate_pass(|| {
            let derivative = &numerator[1] * &denominator[0] - &numerator[0] * &denominator[1];
            let derivative_sign = match real_sign(&derivative, &CurveContext::STRICT) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                }
            };
            let denominator_predicate =
                bivariate_outer_product(&[Real::one()], denominator.as_slice());
            match self.predicate_sign(&denominator_predicate, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }

            let mut refinement_steps = 0_usize;
            let (refined, first, second) = loop {
                let refined = match self.refined(refinement_steps, policy)? {
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
                if let (Some((first_sign, first)), Some((second_sign, second))) = (
                    map(&refined.data.root.lower)?,
                    map(&refined.data.root.upper)?,
                ) && first_sign == second_sign
                {
                    break (refined, first, second);
                }
                refinement_steps = refinement_steps
                    .checked_mul(2)
                    .and_then(|steps| steps.checked_add(1))
                    .ok_or_else(|| {
                        CurveError::Topology("selected-fiber projective refinement overflow".into())
                    })?;
            };
            let (lower, upper) = match derivative_sign {
                RealSign::Positive => (first, second),
                RealSign::Negative => (second, first),
                RealSign::Zero => unreachable!(),
            };
            let exact_root = refined
                .data
                .root
                .exact_root
                .as_ref()
                .map(|source| {
                    let mapped_denominator = &denominator[0] + &denominator[1] * source;
                    (&numerator[0] + &numerator[1] * source) / mapped_denominator
                })
                .transpose()?;
            let inverse_numerator = [numerator[0].clone(), -denominator[0].clone()];
            let inverse_denominator = [-numerator[1].clone(), denominator[1].clone()];
            let authority = BezierAlgebraicSelectedFiberAuthority2::new(
                bivariate_projective_second_parameter(
                    &self.data.authority.data.incidence,
                    &inverse_numerator,
                    &inverse_denominator,
                ),
                self.data.authority.data.retained_parameter.clone(),
                &self.data.authority.data.policy,
            );
            Ok(Classification::Decided(authority.parameter(
                IsolatedRootInterval {
                    lower,
                    upper,
                    exact_root,
                    distinct_root_count: refined.data.root.distinct_root_count,
                },
            )))
        })
    }

    /// Translates this exact local scalar without constructing its global
    /// resultant. If `u` is selected by `F(alpha, u) = 0`, the returned scalar
    /// `v = u + offset` is selected by `F(alpha, v - offset) = 0`.
    pub(super) fn translated(&self, offset: &Real) -> Self {
        let policy = self.data.authority.data.policy;
        let Classification::Decided(translated) = self
            .affine_image_unbounded(&Real::one(), offset, &policy)
            .expect("a represented translation is a nondegenerate affine chart")
        else {
            unreachable!("a represented translation has a strict positive scale")
        };
        translated
    }

    pub(crate) fn refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.validate_policy(policy)?;
        Ok(algebraic_selected_fiber_root_interval_refined(
            &self.data.authority,
            &self.data.root,
            refinement_steps,
            policy,
        )?
        .map(|root| Self {
            data: Arc::new(BezierAlgebraicSelectedFiberParameterData2 {
                authority: self.data.authority.clone(),
                root,
                representations: self.data.representations.clone(),
            }),
        }))
    }

    pub(crate) fn isolating_bounds(&self) -> (&Real, &Real) {
        (&self.data.root.lower, &self.data.root.upper)
    }

    pub(crate) fn predicate_sign(
        &self,
        predicate: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        algebraic_selected_fiber_root_predicate_sign(
            &self.data.authority,
            predicate,
            &self.data.root,
            policy,
        )
    }

    pub(super) fn radical_sum_sign(
        &self,
        expression: &BezierAlgebraicCuspTwoTermExpression2,
        speed_squared: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        algebraic_selected_fiber_root_radical_sum_sign(
            &self.data.authority,
            expression,
            speed_squared,
            &self.data.root,
            policy,
        )
    }

    pub(super) fn square_root_sum_sign(
        &self,
        expression: &BezierAlgebraicCuspTwoTermExpression2,
        radicand: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        algebraic_selected_fiber_root_square_root_sum_sign(
            &self.data.authority,
            expression,
            radicand,
            &self.data.root,
            policy,
        )
    }

    pub(super) fn two_normal_sum_sign(
        &self,
        expression: &BezierParallelTwoNormalExpression2,
        center_speed_squared: &BivariatePolynomial,
        candidate_speed_squared: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.validate_policy(policy)?;
        algebraic_selected_fiber_root_two_normal_sum_sign(
            &self.data.authority,
            expression,
            center_speed_squared,
            candidate_speed_squared,
            &self.data.root,
            policy,
        )
    }

    pub(crate) fn order_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        let strict = CurveContext::STRICT;
        if compare_reals(&self.data.root.upper, value, &strict) == Some(std::cmp::Ordering::Less) {
            return Ok(Classification::Decided(std::cmp::Ordering::Less));
        }
        if compare_reals(value, &self.data.root.lower, &strict) == Some(std::cmp::Ordering::Less) {
            return Ok(Classification::Decided(std::cmp::Ordering::Greater));
        }
        if self.data.root.exact_root.as_ref() == Some(value) {
            return Ok(Classification::Decided(std::cmp::Ordering::Equal));
        }
        if let Some(order) = self.order_positive_rational_tower_branch(value, policy)? {
            return Ok(Classification::Decided(order));
        }
        let predicate = bivariate_outer_product(&[Real::one()], &[(-value.clone()), Real::one()]);
        Ok(self
            .predicate_sign(&predicate, policy)?
            .map(|sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            }))
    }

    /// `s + r√q` has rational coefficients in its annihilator. A zero there,
    /// together with `u - s > 0`, selects the principal real branch.
    pub(super) fn order_positive_rational_tower_branch(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<std::cmp::Ordering>> {
        let Some(shift) = value.quadratic_tower_positive_rational_branch() else {
            return Ok(None);
        };
        let Some(polynomial) = value.quadratic_tower_annihilating_polynomial() else {
            return Ok(None);
        };
        let coefficients = polynomial.into_iter().map(Real::from).collect::<Vec<_>>();
        let annihilator = bivariate_outer_product(&[Real::one()], &coefficients);
        if !matches!(
            self.predicate_sign(&annihilator, policy)?,
            Classification::Decided(RealSign::Zero)
        ) {
            return Ok(None);
        }
        let above_center =
            bivariate_outer_product(&[Real::one()], &[Real::from(-shift), Real::one()]);
        Ok(match self.predicate_sign(&above_center, policy)? {
            Classification::Decided(RealSign::Positive) => Some(std::cmp::Ordering::Equal),
            Classification::Decided(_) | Classification::Uncertain(_) => None,
        })
    }

    /// Returns a finite-boundary isolating interval and representative.
    ///
    /// The returned value is only for explicit IO/display projection.  It is
    /// the midpoint of a certified isolating interval after bounded exact
    /// refinement and must never be fed back into topology or construction.
    pub(crate) fn finite_projection_interval(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Real, Real, Real)>> {
        let parameter = match self.refined(refinement_steps, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let lower = parameter.root().lower.clone();
        let upper = parameter.root().upper.clone();
        let representative = parameter
            .root()
            .exact_root
            .clone()
            .map_or_else(|| (&lower + &upper) / Real::from(2_u8), Ok)?;
        Ok(Classification::Decided((lower, representative, upper)))
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
        let shares_retained_parameter = self.data.authority.data.retained_parameter
            == other.data.authority.data.retained_parameter;
        let is_other_root = if !shares_retained_parameter {
            false
        } else if self.data.authority == other.data.authority {
            true
        } else {
            match self.predicate_sign(&other.data.authority.data.incidence, policy)? {
                Classification::Decided(RealSign::Zero) => true,
                Classification::Decided(RealSign::Negative | RealSign::Positive) => false,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let mut refinement_steps = 0_usize;
        loop {
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
            // A shared polynomial root is not yet a shared selected root.
            // Prove containment in the other original singleton isolator;
            // mere overlap can cover two different roots. Keeping that
            // original isolator also lets equal roots with independently
            // refined brackets converge to this certificate.
            let inside_other = is_other_root
                && matches!(
                    compare_reals(
                        &other.root().lower,
                        &first.root().lower,
                        &CurveContext::STRICT
                    ),
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                )
                && matches!(
                    compare_reals(
                        &first.root().upper,
                        &other.root().upper,
                        &CurveContext::STRICT
                    ),
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                );
            if first == second || inside_other {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            if compare_reals(
                &first.root().upper,
                &second.root().lower,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Less));
            }
            if compare_reals(
                &second.root().upper,
                &first.root().lower,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Greater));
            }
            if !shares_retained_parameter && refinement_steps >= 512 {
                break;
            }
            if shares_retained_parameter
                && refinement_steps == 512
                && policy.permits_approximate_512()
            {
                policy.observe_approximate_512();
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("selected-fiber scalar refinement overflow".into())
                })?;
        }

        // Distinct retained fields usually separate from their certified
        // local isolators without constructing either degree-multiplied norm.
        // Equality is the only case for which interval refinement cannot
        // decide. Exhaust the existing exact projection only at that terminal
        // boundary; it remains construction-independent and selects the same
        // already-isolated roots under STRICT.
        let (first, second) = policy.strict_predicate_pass(|| -> CurveResult<_> {
            Ok((
                self.promoted_bezier_parameter(policy)?,
                other.promoted_bezier_parameter(policy)?,
            ))
        })?;
        let mut reason = UncertaintyReason::Unsupported;
        if let (Classification::Decided(first), Classification::Decided(second)) = (&first, &second)
        {
            match policy
                .strict_predicate_pass(|| first.cmp_by_refinement_with_policy(second, policy))?
            {
                Classification::Decided(order) => {
                    return Ok(Classification::Decided(order));
                }
                Classification::Uncertain(uncertainty) => reason = uncertainty,
            }
        } else {
            for projection in [&first, &second] {
                if let Classification::Uncertain(uncertainty) = projection {
                    reason = *uncertainty;
                }
            }
        }
        if policy.permits_approximate_512() {
            policy.observe_approximate_512();
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        } else {
            Ok(Classification::Uncertain(reason))
        }
    }

    pub(crate) fn cmp_bezier_parameter(
        &self,
        other: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        self.validate_policy(policy)?;
        if let Some(parameter) = self.data.representations.bezier.get() {
            return parameter.cmp_by_refinement_with_policy(other, policy);
        }
        let outcome = crate::policy::resolve_certified_value(policy, |attempt| {
            self.cmp_bezier_parameter_uncached(other, attempt)
        });
        let order = outcome.value?;
        // An exact fiber root already is the native representation; only
        // proofs for otherwise unrepresented roots are worth retaining.
        if order == Classification::Decided(std::cmp::Ordering::Equal)
            && self.represented_value().is_none()
            && outcome.certainty == crate::CurveCertainty::Certified
            && policy
                .strict_counterpart()
                .accepts_retained_policy(self.data.authority.data.policy)
        {
            self.retain_certified_parameter(other.clone());
        }
        Ok(order)
    }

    pub(super) fn cmp_bezier_parameter_uncached(
        &self,
        other: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let BezierParameter2::Algebraic(other_algebraic) = other else {
            return self.order_to_real(
                other
                    .scalar()
                    .expect("an exact Bezier parameter exposes its Real"),
                policy,
            );
        };
        // Separated certified intervals already decide order. In particular,
        // a selected spline boundary need not evaluate an unrelated contact
        // polynomial in its high-degree field just to reject a distant root.
        let strict = policy.strict_counterpart();
        let separated_order = |lower: &Real, upper: &Real, other: &BezierAlgebraicParameter2| {
            if compare_reals(upper, other.interval().start(), &strict)
                == Some(std::cmp::Ordering::Less)
            {
                Some(std::cmp::Ordering::Less)
            } else if compare_reals(other.interval().end(), lower, &strict)
                == Some(std::cmp::Ordering::Less)
            {
                Some(std::cmp::Ordering::Greater)
            } else {
                None
            }
        };
        if let Some(order) =
            separated_order(&self.root().lower, &self.root().upper, other_algebraic)
        {
            return Ok(Classification::Decided(order));
        }
        if let Some((numerator, denominator)) =
            other_algebraic.projective_map_from(&self.data.authority.data.retained_parameter)
        {
            // The chart retains beta = N(alpha)/D(alpha). Compare u with beta
            // through u*D(alpha)-N(alpha), including D's sign, rather than
            // evaluating beta's global polynomial in the selected fiber.
            let difference = BivariatePolynomial::new(
                (0..2)
                    .map(|i| vec![-numerator[i].clone(), denominator[i].clone()])
                    .collect(),
            );
            match self.predicate_sign(&difference, policy)? {
                // Invertible chart construction already certified D(alpha)
                // nonzero. Equality does not need to rediscover its sign.
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(std::cmp::Ordering::Equal));
                }
                Classification::Decided(sign) => match signed_coefficients_at_parameter(
                    &denominator,
                    &BezierParameter2::Algebraic(
                        self.data.authority.data.retained_parameter.clone(),
                    ),
                    &strict,
                )? {
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    Classification::Decided(denominator_sign) => {
                        return Ok(Classification::Decided(if sign == denominator_sign {
                            std::cmp::Ordering::Greater
                        } else {
                            std::cmp::Ordering::Less
                        }));
                    }
                    Classification::Uncertain(_) => {}
                },
                Classification::Uncertain(_) => {}
            }
            // A retained chart is an additional replay route. Arbitrary
            // exact coefficients may defeat this local predicate while the
            // original polynomial still provides a usable zero certificate.
        }
        let other_predicate =
            bivariate_outer_product(&[Real::one()], other_algebraic.polynomial().coefficients());
        let is_other_root = match self.predicate_sign(&other_predicate, policy)? {
            Classification::Decided(RealSign::Zero) => true,
            Classification::Decided(RealSign::Negative | RealSign::Positive) => false,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut refinement_steps = 0_usize;
        loop {
            let selected = match self.refined(refinement_steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let other = BezierParameter2::Algebraic(other_algebraic.clone())
                .refined_isolating_interval(refinement_steps, &CurveContext::STRICT);
            let BezierParameter2::Algebraic(other) = other else {
                return selected.order_to_real(
                    other
                        .scalar()
                        .expect("a refined root may become represented"),
                    policy,
                );
            };
            if let Some(order) =
                separated_order(&selected.root().lower, &selected.root().upper, &other)
            {
                return Ok(Classification::Decided(order));
            }
            let strict = &CurveContext::STRICT;
            if is_other_root
                && matches!(
                    compare_reals(
                        other_algebraic.interval().start(),
                        &selected.root().lower,
                        strict
                    ),
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                )
                && matches!(
                    compare_reals(
                        &selected.root().upper,
                        other_algebraic.interval().end(),
                        strict
                    ),
                    Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                )
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("selected-fiber/Bezier refinement overflow".into())
                })?;
        }
    }

    pub(crate) fn strict_scalar_between_ordered(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        self.validate_policy(policy)?;
        other.validate_policy(policy)?;
        let shares_retained_parameter = self.data.authority.data.retained_parameter
            == other.data.authority.data.retained_parameter;
        let mut refinement_steps = 0_usize;
        loop {
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
            if compare_reals(
                &first.root().upper,
                &second.root().lower,
                &CurveContext::STRICT,
            ) == Some(std::cmp::Ordering::Less)
            {
                return Ok(Classification::Decided(
                    ((&first.root().upper + &second.root().lower) / Real::from(2_i8))?,
                ));
            }
            if !shares_retained_parameter && refinement_steps >= 512 {
                break;
            }
            refinement_steps = refinement_steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("selected-fiber separation refinement overflow".into())
                })?;
        }

        let first = match policy.strict_predicate_pass(|| self.promoted_bezier_parameter(policy))? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second =
            match policy.strict_predicate_pass(|| other.promoted_bezier_parameter(policy))? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        policy.strict_predicate_pass(|| first.strict_scalar_between_ordered(&second, policy))
    }
}
