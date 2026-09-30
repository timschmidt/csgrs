//! Exact analytic parallel carrier: construction, evaluation, singularity analysis, incidence and pair intersections.

use super::*;

impl BezierParallel2 {
    /// Constructs an exact analytic parallel from its structural source and
    /// signed left-normal distance.
    ///
    /// [`Self::source`] and [`Self::distance`] provide the inverse structural
    /// view for exact serialization and diagnostics.
    pub fn from_source(source: BezierParallelSource2, distance: Real) -> Self {
        Self::from_shared_source(
            Arc::new(BezierParallelSourceData2 {
                source,
                polynomial_power_basis: OnceLock::new(),
                differential: OnceLock::new(),
                primitive_tangent: OnceLock::new(),
                unit_ph_speed: OnceLock::new(),
            }),
            distance,
        )
    }

    pub(super) fn from_shared_source(
        source: Arc<BezierParallelSourceData2>,
        distance: Real,
    ) -> Self {
        Self {
            data: Arc::new(BezierParallelData2 {
                source,
                distance,
                certified_ph_offset: OnceLock::new(),
            }),
        }
    }

    /// Returns the same exact source kernel at another signed normal distance.
    ///
    /// Source power-basis, differential, primitive tangent and unit PH speed proofs are
    /// distance-independent and remain clone-shared. PH materialization keeps its
    /// own cache in the new one-word carrier.
    pub(crate) fn with_distance(&self, distance: Real) -> Self {
        Self::from_shared_source(Arc::clone(&self.data.source), distance)
    }

    /// Returns the exact source representation retained by this parallel.
    pub fn source(&self) -> &BezierParallelSource2 {
        &self.data.source.source
    }

    /// Returns whether two parallel carriers use the same exact parameterized
    /// source and signed distance, allowing the canonical unit-weight rational
    /// embedding of a polynomial source.
    ///
    /// This is intentionally narrower than geometric curve equality: it is a
    /// provenance comparison for replaying a selected parameter on two
    /// structurally equivalent procedural carriers.
    pub(crate) fn shares_parameterized_curve_evidence(&self, other: &Self) -> bool {
        if self.data.distance != other.data.distance {
            return false;
        }
        if self.source() == other.source() {
            return true;
        }
        let as_rational = |source: &BezierParallelSource2| match source {
            BezierParallelSource2::Quadratic(source) => RationalBezier2::try_from_subcurve(
                &crate::BezierSubcurve2::Quadratic(source.clone()),
            )
            .ok(),
            BezierParallelSource2::Cubic(source) => {
                RationalBezier2::try_from_subcurve(&crate::BezierSubcurve2::Cubic(source.clone()))
                    .ok()
            }
            BezierParallelSource2::Rational(source) => Some(source.clone()),
        };
        match (as_rational(self.source()), as_rational(other.source())) {
            (Some(first), Some(second)) => first == second,
            (None, _) | (_, None) => false,
        }
    }

    /// Returns the degree of the retained source span.
    pub fn source_degree(&self) -> usize {
        match self.source() {
            BezierParallelSource2::Quadratic(_) => 2,
            BezierParallelSource2::Cubic(_) => 3,
            BezierParallelSource2::Rational(source) => source.degree(),
        }
    }

    /// Certifies an injective coordinate on the requested finite range.
    ///
    /// A regular parallel derivative is the source derivative multiplied by
    /// one continuous scalar. Excluding source singularities and interior
    /// parallel cusps proves that scalar has one sign in the open range.
    /// The caller may ask about a larger range than a previously certified
    /// fragment, so regularity is proved here rather than inferred from the
    /// carrier's history. The native source coordinate certificate covers
    /// only the unit chart. Unresolved proofs leave complete incidence live.
    pub(crate) fn range_has_certified_injective_axis(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> bool {
        [Axis2::X, Axis2::Y]
            .into_iter()
            .any(|axis| self.range_has_certified_injective_axis_on(axis, range, policy))
    }

    pub(crate) fn range_has_certified_injective_axis_on(
        &self,
        axis: Axis2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> bool {
        policy.bounded_exact_predicate_pass(|| {
            if !matches!(
                CurveParameterDomain2::new(&CurveParameterRange2::unit(), None)
                    .contains_finite_range(range, &policy.strict_counterpart()),
                Ok(Classification::Decided(true))
            ) {
                return false;
            }
            let strict = policy.strict_counterpart();
            if !self
                .source()
                .to_rational_bezier()
                .is_ok_and(|source| source.has_certified_injective_axis_on(axis, &strict))
            {
                return false;
            }
            if real_sign(self.distance(), &strict) == Some(RealSign::Zero) {
                return true;
            }
            let Ok(Classification::Decided(analysis)) = self.singularity_analysis(range, &strict)
            else {
                return false;
            };
            analysis.source_is_regular()
                && analysis.parallel_cusps().iter().all(|cusp| {
                    // An endpoint cusp does not change monotonicity on the
                    // open interval. Interior cusps require the general path.
                    let parameter = CurveParameter2::from(cusp.clone());
                    [range.start(), range.end()].into_iter().any(|endpoint| {
                        matches!(
                            parameter.cmp_by_refinement(endpoint, &strict),
                            Ok(Classification::Decided(std::cmp::Ordering::Equal))
                        )
                    })
                })
        })
    }

    /// Returns the orientation of this parallel's derivative relative to its
    /// retained source on one certified regular fragment.
    ///
    /// The two derivatives are collinear on a regular parallel.  The fragment
    /// invariant excludes every interior parallel cusp, so their nonzero dot
    /// product has one sign throughout the open range.  One exact rational
    /// interior parameter therefore certifies whether a subsequent signed
    /// left offset adds to or subtracts from the retained source distance.
    pub(crate) fn regular_fragment_derivative_scale_sign(
        &self,
        range: &BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let parameter = match range.strict_interior_scalar(policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.parallel_derivative_scale_sign_at_exact(&parameter, policy)
    }

    /// Evaluates a nonzero tangent direction of the retained source.
    ///
    /// Rational sources use their homogeneous derivative numerator.  Its
    /// orientation agrees with the affine derivative because the omitted
    /// denominator is a positive square.
    pub(crate) fn source_tangent_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Real, Real)>> {
        if let Some(source) = self.rational_source() {
            let weight = Real::eval_poly(&source.homogeneous_power_basis()?.weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let differential = self.differential()?;
        let tangent = (
            Real::eval_poly(&differential.tangent_x, parameter),
            Real::eval_poly(&differential.tangent_y, parameter),
        );
        let speed_squared = &tangent.0 * &tangent.0 + &tangent.1 * &tangent.1;
        Ok(match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => Classification::Decided(tangent),
            Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "Bezier source tangent squared norm was certified negative".to_owned(),
                ));
            }
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        })
    }

    pub(super) fn source_oriented_regularized_tangent_field(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Arc<BezierAnalyticParallelTangentField2>>>> {
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.source_oriented_regularized_tangent_field_at_interior(&interior, &strict)
    }

    /// A finite cell may share its normal field with an incident extension
    /// only when that extension stays on the same regular source sheet.
    pub(super) fn source_tangent_field_in_regular_domain(
        &self,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Arc<BezierAnalyticParallelTangentField2>>>> {
        let field = match self.source_oriented_regularized_tangent_field(domain.finite, policy)? {
            Classification::Decided(field) => field,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (Some(frame), Some(ray)) = (field.as_deref(), domain.extension) else {
            return Ok(Classification::Decided(field));
        };
        match ray.is_empty(policy)? {
            Classification::Decided(true) => return Ok(Classification::Decided(field)),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let differential = self.differential()?;
        let dot = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &frame.x),
            &polynomial_multiply(&differential.tangent_y, &frame.y),
        );
        let strict = policy.strict_counterpart();
        match real_sign(&Real::eval_poly(&dot, ray.anchor), &strict) {
            Some(RealSign::Positive) => {}
            Some(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let polynomial = match polynomial_from_coefficients(dot, &strict)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                unreachable!("the dot product was positive at the anchor")
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match (SelectedThirdAxisDomain2::IncidentRay {
            anchor: ray.anchor,
            direction: ray.direction,
            barrier: ray.barrier,
        })
        .isolate(&polynomial, &strict)?
        {
            Classification::Decided(roots) if roots.is_empty() => {
                Ok(Classification::Decided(field))
            }
            Classification::Decided(_) => {
                Ok(Classification::Uncertain(UncertaintyReason::Boundary))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(super) fn source_oriented_regularized_tangent_field_at_interior(
        &self,
        interior: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Arc<BezierAnalyticParallelTangentField2>>>> {
        let differential = self.differential()?;
        if self.data.source.primitive_tangent.get().is_none() {
            let Some(common_factor) = greatest_common_divisor_univariate_polynomials_exact(
                &differential.tangent_x,
                &differential.tangent_y,
            ) else {
                // Unavailable monic normalization does not prove absence of
                // a common factor. Keep the independent exact rank-one
                // fallback available and allow later queries to retry.
                return Ok(
                    match self.source_constant_tangent_field_at_interior(
                        differential,
                        interior,
                        policy,
                    )? {
                        Classification::Decided(Some(field)) => {
                            Classification::Decided(Some(field))
                        }
                        Classification::Decided(None) => {
                            Classification::Uncertain(UncertaintyReason::Unsupported)
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    },
                );
            };
            let primitive = if common_factor.len() <= 1 {
                None
            } else {
                let (Some(x), Some(y)) = (
                    divide_univariate_polynomial_exact(&differential.tangent_x, &common_factor),
                    divide_univariate_polynomial_exact(&differential.tangent_y, &common_factor),
                ) else {
                    return Err(CurveError::Topology(
                        "source hodograph gcd did not divide both coordinates".into(),
                    ));
                };
                Some(BezierParallelPrimitiveTangent2 {
                    factor: common_factor,
                    field: Arc::new(BezierAnalyticParallelTangentField2 { x, y }),
                    reversed_field: OnceLock::new(),
                })
            };
            // Hypersolve's GCD and division certify all coefficients under
            // STRICT. Retain only that successful factorization evidence.
            let _ = self.data.source.primitive_tangent.set(primitive);
        }
        let Some(primitive) = self
            .data
            .source
            .primitive_tangent
            .get()
            .expect("successful source hodograph factorization was retained")
        else {
            return Ok(Classification::Decided(None));
        };
        let strict = policy.strict_counterpart();
        let field = match real_sign(&Real::eval_poly(&primitive.factor, interior), &strict) {
            Some(RealSign::Positive) => &primitive.field,
            Some(RealSign::Negative) => primitive.reversed_field.get_or_init(|| {
                let negative_one = Real::from(-1_i8);
                Arc::new(BezierAnalyticParallelTangentField2 {
                    x: polynomial_scale(&primitive.field.x, &negative_one),
                    y: polynomial_scale(&primitive.field.y, &negative_one),
                })
            }),
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "regular source branch interior remained singular".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        Ok(Classification::Decided(Some(Arc::clone(field))))
    }

    /// Recovers a constant oriented tangent when monic hodograph GCD
    /// normalization alone is unsupported.
    ///
    /// A coefficient may have unresolved zero status even though the two
    /// tangent polynomials have exact rank one. One nonzero interior tangent
    /// supplies a scale-free direction; every polynomial coefficient must
    /// then replay with zero cross product under STRICT. This certifies a line
    /// image without assuming that the unresolved coefficient is nonzero and
    /// without using approximate construction evidence.
    pub(super) fn source_constant_tangent_field_at_interior(
        &self,
        differential: &BezierParallelDifferential2,
        interior: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Arc<BezierAnalyticParallelTangentField2>>>> {
        let strict = policy.strict_counterpart();
        let tangent_x = Real::eval_poly(&differential.tangent_x, interior);
        let tangent_y = Real::eval_poly(&differential.tangent_y, interior);
        let tangent_x_sign = real_sign(&tangent_x, &strict);
        let tangent_y_sign = real_sign(&tangent_y, &strict);
        let (direction_x, direction_y) = match (tangent_x_sign, tangent_y_sign) {
            (Some(sign @ (RealSign::Positive | RealSign::Negative)), _) => {
                let orientation = if sign == RealSign::Positive {
                    Real::one()
                } else {
                    Real::from(-1_i8)
                };
                let ratio = (tangent_y / &tangent_x)?;
                (orientation.clone(), orientation * ratio)
            }
            (_, Some(sign @ (RealSign::Positive | RealSign::Negative))) => {
                let orientation = if sign == RealSign::Positive {
                    Real::one()
                } else {
                    Real::from(-1_i8)
                };
                let ratio = (tangent_x / &tangent_y)?;
                (orientation.clone() * ratio, orientation)
            }
            (Some(RealSign::Zero), Some(RealSign::Zero)) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let cross = polynomial_subtract(
            &polynomial_scale(&differential.tangent_x, &direction_y),
            &polynomial_scale(&differential.tangent_y, &direction_x),
        );
        for coefficient in cross {
            match real_sign(&coefficient, &strict) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(None));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "analytic-parallel-regularized-tangent",
            "constant-direction-rank",
        );
        Ok(Classification::Decided(Some(Arc::new(
            BezierAnalyticParallelTangentField2 {
                x: vec![direction_x],
                y: vec![direction_y],
            },
        ))))
    }

    /// Certifies whether two retained ranges of this parameterized source use
    /// the same one-sided unit-normal sheet.
    ///
    /// Across a source cusp, equal source parameters do not identify equal
    /// parallel points: cancelling the common hodograph factor yields
    /// opposite oriented frames on branches where that factor changes sign.
    /// Arrangement contact deduplication uses this compact certificate before
    /// treating shared carrier/parameter provenance as shared point evidence.
    pub(crate) fn regular_source_ranges_share_normal_sheet(
        &self,
        first_range: &BezierParameterRange2,
        second_range: &BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let first = match self.source_oriented_regularized_tangent_field(
            &CurveParameterRange2::from_bezier_range(first_range.clone()),
            policy,
        )? {
            Classification::Decided(field) => field,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match self.source_oriented_regularized_tangent_field(
            &CurveParameterRange2::from_bezier_range(second_range.clone()),
            policy,
        )? {
            Classification::Decided(field) => field,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(match (first, second) {
            (None, None) => Classification::Decided(true),
            (Some(first), Some(second)) => Classification::Decided(first == second),
            (None, Some(_)) | (Some(_), None) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
        })
    }

    /// Constructs a point and an oriented unit-tangent support on one regular
    /// source branch, including its finite one-sided stationary endpoint.
    ///
    /// Cancelling the exact common hodograph factor and retaining its branch
    /// sign preserves the selected source normal. The parameter keeps its native,
    /// selected-fiber or recursive authority; no global root projection is needed.
    /// The source weight and primitive speed must be nonzero at the contact.
    ///
    /// The point uses this parallel's distance, while the tangent is anchored
    /// on `tangent_anchor` in the same source chart. Its direction is relative
    /// to the increasing source tangent, before any parallel derivative scale.
    pub(crate) fn regular_source_point_and_tangent_support(
        &self,
        tangent_anchor: &Self,
        parameter: &CurveParameter2,
        range: &CurveParameterRange2,
        tangent_direction: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(CurvePoint2, BezierAlgebraicChord2)>> {
        debug_assert!(Arc::ptr_eq(&self.data.source, &tangent_anchor.data.source));
        if tangent_direction == RealSign::Zero {
            return Err(CurveError::Topology(
                "regular source tangent direction was zero".into(),
            ));
        }
        let strict = policy.strict_counterpart();
        if let Some(weight) = self.source_power_basis()?.weight {
            match parameter.polynomial_sign(weight, &strict)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let frame = match self.source_oriented_regularized_tangent_field(range, &strict)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (x, y) = match &frame {
            Some(frame) => (&frame.x, &frame.y),
            None => {
                let differential = self.differential()?;
                (&differential.tangent_x, &differential.tangent_y)
            }
        };
        let speed_squared = polynomial_add(&polynomial_multiply(x, x), &polynomial_multiply(y, y));
        match parameter.polynomial_sign(&speed_squared, &strict)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "regular source frame had negative squared speed".into(),
                ));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let retain = |parallel: &Self, tangent_distance| -> CurveResult<_> {
            let Some(point) =
                BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                    parallel.clone(),
                    parameter,
                    frame.clone(),
                    tangent_distance,
                    policy,
                )
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            Ok(match point.represented_point(policy)? {
                Classification::Decided(Some(point)) => {
                    Classification::Decided(CurvePoint2::from(point))
                }
                Classification::Decided(None) => Classification::Decided(CurvePoint2::from(point)),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        };
        let point = match retain(self, Real::zero())? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let anchor = if self.distance() == tangent_anchor.distance() {
            point.clone()
        } else {
            match retain(tangent_anchor, Real::zero())? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        };
        let support = match retain(
            tangent_anchor,
            match tangent_direction {
                RealSign::Positive => Real::one(),
                RealSign::Negative => -Real::one(),
                RealSign::Zero => unreachable!("zero tangent direction returned above"),
            },
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let tangent = match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
            anchor, support, policy,
        )? {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided((point, tangent)))
    }

    /// Tests the original source tangent at a retained parameter without
    /// materializing its coordinates or replacing its scalar authority.
    pub(crate) fn source_tangent_nonzero_at(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let speed = parallel_speed_squared_polynomial(self.differential()?);
        Ok(
            match parameter.polynomial_sign(&speed, &policy.strict_counterpart())? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(true),
                Classification::Decided(RealSign::Zero) => Classification::Decided(false),
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "source tangent had negative squared speed".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Retains a point on one certified regular carrier range without
    /// projecting selected-fiber or recursive-projective endpoints into a
    /// global parameter polynomial.
    pub(crate) fn point_evidence_on_regular_range(
        &self,
        parameter: &CurveParameter2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurvePoint2>> {
        policy.strict_predicate_pass(|| {
            let strict = policy;
            let interior = match range.strict_interior_scalar(strict)? {
                Classification::Decided(interior) => interior,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let frame = match self
                .source_oriented_regularized_tangent_field_at_interior(&interior, strict)?
            {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let Some(point) =
                BezierAnalyticParallelPoint2::new_with_region_parameter_and_frame_tangent(
                    self.clone(),
                    parameter,
                    frame,
                    Real::zero(),
                    policy,
                )
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            Ok(Classification::Decided(CurvePoint2::from(point)))
        })
    }

    /// Signs one represented vector crossed with, and dotted with, this
    /// parallel's increasing-parameter tangent at a retained parameter.
    ///
    /// The source tangent numerator supplies both linear forms; the exact
    /// parallel/source derivative-scale certificate then applies the common
    /// orientation factor. Each polynomial reuses the parameter's native,
    /// selected-fiber, or recursive authority without requiring a projected
    /// Bezier parameter. No normalized algebraic tangent is constructed.
    pub(crate) fn vector_tangent_cross_and_dot_signs(
        &self,
        parameter: &CurveParameter2,
        vector_x: &Real,
        vector_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(RealSign, RealSign)>> {
        self.vector_tangent_cross_and_dot_signs_with_tangent_field(
            parameter, vector_x, vector_y, None, None, policy,
        )
    }

    /// Signs a represented vector against this parallel's tangent on one
    /// certified regular source range.
    ///
    /// A retained source fragment can end at a stationary parameter.  Its
    /// authored hodograph then vanishes even though cancelling the exact
    /// common factor leaves a finite one-sided parallel tangent.  Reuse that
    /// source-oriented regularized field and replay the parallel/source derivative
    /// scale at the contact or its owned one-sided limit, for endpoint and algebraic
    /// contact predicates on the complete retained range.
    pub(crate) fn vector_tangent_cross_and_dot_signs_on_regular_range(
        &self,
        parameter: &CurveParameter2,
        vector_x: &Real,
        vector_y: &Real,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(RealSign, RealSign)>> {
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_field =
            match self.source_oriented_regularized_tangent_field_at_interior(&interior, &strict)? {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let derivative_scale_sign = match self
            .parallel_derivative_scale_sign_on_regular_range(parameter, range, &strict)?
        {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.vector_tangent_cross_and_dot_signs_with_tangent_field(
            parameter,
            vector_x,
            vector_y,
            tangent_field.as_deref(),
            Some(derivative_scale_sign),
            policy,
        )
    }

    pub(super) fn vector_tangent_cross_and_dot_signs_with_tangent_field(
        &self,
        parameter: &CurveParameter2,
        vector_x: &Real,
        vector_y: &Real,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        derivative_scale_sign: Option<RealSign>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(RealSign, RealSign)>> {
        let differential = self.differential()?;
        let (tangent_x, tangent_y) = tangent_field
            .map(|field| (&field.x[..], &field.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));
        let cross = polynomial_subtract(
            &polynomial_scale(tangent_y, vector_x),
            &polynomial_scale(tangent_x, vector_y),
        );
        let dot = polynomial_add(
            &polynomial_scale(tangent_x, vector_x),
            &polynomial_scale(tangent_y, vector_y),
        );
        let source_cross = match parameter.polynomial_sign(&cross, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source_dot = match parameter.polynomial_sign(&dot, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let scale = match derivative_scale_sign {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            None => match self.parallel_derivative_scale_sign(parameter, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        Ok(Classification::Decided((
            product_sign(source_cross, scale),
            product_sign(source_dot, scale),
        )))
    }

    /// Signs `cross_scale * (vector x T_source) + dot_scale *
    /// (vector dot T_source)` at one retained source parameter.
    ///
    /// Both products are linear forms in the same homogeneous source
    /// tangent, so combining their coefficients before evaluation is the
    /// smallest exact predicate. Unlike [`Self::vector_tangent_cross_and_dot_signs`],
    /// this intentionally does not apply the parallel derivative scale.
    pub(crate) fn vector_source_tangent_cross_dot_linear_combination_sign(
        &self,
        parameter: &CurveParameter2,
        vector_x: &Real,
        vector_y: &Real,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let differential = self.differential()?;
        let coefficient_x = dot_scale * vector_x - cross_scale * vector_y;
        let coefficient_y = cross_scale * vector_x + dot_scale * vector_y;
        parameter.polynomial_sign(
            &polynomial_add(
                &polynomial_scale(&differential.tangent_x, &coefficient_x),
                &polynomial_scale(&differential.tangent_y, &coefficient_y),
            ),
            policy,
        )
    }

    /// Returns the source-speed sign when two parallel queries retain the
    /// identical source and parameter evidence. In that structural case their
    /// source-tangent cross is exactly zero and their dot product is this
    /// nonnegative squared speed; no bivariate parameter-pair predicate is
    /// needed.
    pub(super) fn shared_source_parameter_speed_sign(
        &self,
        parameter: &BezierParameter2,
        other: &Self,
        other_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "parallel-shared-source-parameter",
            match (
                self.source() == other.source(),
                self.source().is_reversal_of(other.source()),
                parameter == other_parameter,
            ) {
                (true, _, true) => "same-source-same-parameter",
                (true, _, false) => "same-source-distinct-representation",
                (false, true, _) => "reversed-source",
                (false, false, _) => "different-source",
            },
        );
        if self.source() != other.source() || parameter != other_parameter {
            return Ok(None);
        }
        let speed_squared = parallel_speed_squared_polynomial(self.differential()?);
        signed_coefficients_at_parameter(&speed_squared, parameter, policy).map(Some)
    }

    /// Signs the two retained source-tangent bilinear forms without
    /// constructing either algebraic vector.
    ///
    /// Offset traversal applies its derivative-scale and reversal factors at
    /// the caller. Keeping this primitive on source tangents lets two
    /// independently promoted selected-fiber endpoints reuse the general
    /// parameter-pair predicate authority.
    pub(crate) fn source_tangent_pair_cross_and_dot_signs(
        &self,
        parameter: &BezierParameter2,
        other: &Self,
        other_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(RealSign, RealSign)>> {
        if let Some(speed) =
            self.shared_source_parameter_speed_sign(parameter, other, other_parameter, policy)?
        {
            return Ok(match speed {
                Classification::Decided(RealSign::Positive) => {
                    Classification::Decided((RealSign::Zero, RealSign::Positive))
                }
                Classification::Decided(RealSign::Zero) => {
                    Classification::Decided((RealSign::Zero, RealSign::Zero))
                }
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "Bezier source tangent squared norm was certified negative".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        let first = self.differential()?;
        let second = other.differential()?;
        let cross = bivariate_subtract(
            &bivariate_outer_product(&first.tangent_x, &second.tangent_y),
            &bivariate_outer_product(&first.tangent_y, &second.tangent_x),
        );
        let dot = bivariate_add(
            &bivariate_outer_product(&first.tangent_x, &second.tangent_x),
            &bivariate_outer_product(&first.tangent_y, &second.tangent_y),
        );
        let cross =
            match signed_bivariate_at_parameter_pair(&cross, parameter, other_parameter, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let dot =
            match signed_bivariate_at_parameter_pair(&dot, parameter, other_parameter, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(Classification::Decided((cross, dot)))
    }

    /// Signs one linear combination of the cross and dot products of two
    /// retained source tangents. Both normalized tangent products share the
    /// same positive denominator, so the homogeneous source numerators are a
    /// complete exact authority even when each parameter is algebraic.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn source_tangent_pair_cross_dot_linear_combination_sign(
        &self,
        parameter: &BezierParameter2,
        other: &Self,
        other_parameter: &BezierParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(speed) =
            self.shared_source_parameter_speed_sign(parameter, other, other_parameter, policy)?
        {
            return Ok(match speed {
                Classification::Decided(RealSign::Positive) => real_sign(dot_scale, policy).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    Classification::Decided,
                ),
                Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "Bezier source tangent squared norm was certified negative".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        let first = self.differential()?;
        let second = other.differential()?;
        let cross = bivariate_subtract(
            &bivariate_outer_product(&first.tangent_x, &second.tangent_y),
            &bivariate_outer_product(&first.tangent_y, &second.tangent_x),
        );
        let dot = bivariate_add(
            &bivariate_outer_product(&first.tangent_x, &second.tangent_x),
            &bivariate_outer_product(&first.tangent_y, &second.tangent_y),
        );
        let expression = bivariate_add(
            &bivariate_scale(cross, cross_scale),
            &bivariate_scale(dot, dot_scale),
        );
        signed_bivariate_at_parameter_pair(&expression, parameter, other_parameter, policy)
    }

    /// Returns the same exact parallel image with traversal direction reversed.
    ///
    /// Reversal flips the source tangent and therefore its left normal. The
    /// signed distance is negated so this carrier still traces the original
    /// parallel image, now from end to start.
    pub fn reversed(&self) -> Self {
        Self::from_source(self.source().reversed(), -self.distance().clone())
    }

    /// Applies a certified planar similarity without materializing a finite
    /// approximation of this analytic parallel.
    ///
    /// Uniform scale multiplies the signed normal distance.  Reflection also
    /// reverses the transformed source's left normal, so it negates that
    /// distance.  The parameter and traversal direction remain unchanged.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        let mut distance = self.distance() * transform.scale();
        if transform.reverses_orientation() {
            distance = -distance;
        }
        Ok(Self::from_source(
            self.source().transform_similarity(transform)?,
            distance,
        ))
    }

    /// Splits this exact parallel at one represented interior parameter.
    ///
    /// The two returned carriers use local `[0, 1]` parameters and retain the
    /// same signed left distance. Endpoint splits are rejected as boundaries
    /// because a zero-width source span has no defined unit normal.
    pub fn split_at_exact(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, Self)>> {
        match strict_interior_unit_parameter(parameter, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(self
            .source()
            .split_at_exact(parameter, policy)?
            .map(|(left, right)| {
                (
                    Self::from_source(left, self.distance().clone()),
                    Self::from_source(right, self.distance().clone()),
                )
            }))
    }

    /// Restricts this exact parallel to an ordered, nonempty represented range.
    ///
    /// The returned carrier is reparameterized to `[0, 1]` and retains the
    /// source orientation and signed left distance.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if in_closed_unit_interval(start, policy) != Some(true)
            || in_closed_unit_interval(end, policy) != Some(true)
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        match compare_reals(start, end, policy) {
            Some(std::cmp::Ordering::Less) => {}
            Some(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(std::cmp::Ordering::Greater) | None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
        Ok(self
            .source()
            .subcurve_between_exact(start, end, policy)?
            .map(|source| Self::from_source(source, self.distance().clone())))
    }

    /// Returns a conservative exact box for every defined point of this parallel.
    ///
    /// A unit normal changes either source coordinate by at most `|distance|`,
    /// so expanding a certified source box by that amount is exact broad-phase
    /// evidence without sampling the parallel or materializing a finite curve.
    pub fn conservative_bounds(&self) -> CurveResult<Classification<Aabb2>> {
        let source = match self.source().certified_bounds() {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius = self.distance().abs();
        Ok(Classification::Decided(Aabb2::new_unchecked(
            Point2::new(source.min_x() - &radius, source.min_y() - &radius),
            Point2::new(source.max_x() + &radius, source.max_y() + radius),
        )))
    }

    /// Bounds this analytic parallel over one represented source-parameter
    /// interval for certified finite projection.
    ///
    /// Unlike the broad-phase whole-source envelope, interval evaluation of
    /// the normalized tangent converges as the parameter interval shrinks.
    /// This lets the explicit lossy adapter honor its chord-error budget
    /// without sampling topology or fitting a replacement curve.
    pub(crate) fn finite_projection_bounds_over_parameter_interval(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        if in_closed_unit_interval(start, policy) != Some(true)
            || in_closed_unit_interval(end, policy) != Some(true)
            || compare_reals(start, end, policy) == Some(std::cmp::Ordering::Greater)
        {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        analytic_parallel_point_bounds_over_interval(
            self,
            &RealInterval {
                lower: start.clone(),
                upper: end.clone(),
            },
            &Real::zero(),
            &Real::zero(),
            &Real::zero(),
        )
    }

    pub(crate) fn point_bounds_at_parameter(
        &self,
        parameter: &BezierParameter2,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        analytic_parallel_point_bounds_refined(
            self,
            parameter,
            &Real::zero(),
            &Real::zero(),
            &Real::zero(),
            refinement_steps,
            policy,
        )
    }

    /// Returns complete exact point incidence on the requested finite range.
    ///
    /// Roots remain in the original support chart, including exterior and
    /// selected endpoint ranges. Source poles are excluded on this range;
    /// zero displacement needs no source normal.
    ///
    /// For a rational source `P=(X/W,Y/W)` with homogeneous tangent numerator
    /// `H`, a point `C` lies on one of the two unsigned parallels exactly when
    /// `(CW-(X,Y)) dot H = 0` and
    /// `|CW-(X,Y)|^2-d^2 W^2 = 0`. Their polynomial GCD supplies every real
    /// candidate. The sign of
    /// `(CW-(X,Y)) dot rotate90(H) * W * d` then removes the opposite normal
    /// branch without evaluating an approximate square root. Polynomial curves
    /// are the `W=1` specialization.
    pub(crate) fn point_incidence(
        &self,
        point: &Point2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        self.point_incidence_in_domain(point, CurveParameterDomain2::new(range, None), policy)
    }

    pub(super) fn point_incidence_in_domain(
        &self,
        point: &Point2,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        self.point_incidence_with_tangent_field(point, domain, None, policy)
    }

    pub(super) fn point_incidence_with_tangent_field(
        &self,
        point: &Point2,
        domain: CurveParameterDomain2<'_>,
        frame: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
        };
        let source = self.source_power_basis()?;

        if let Some(weight) = source.weight {
            match polynomial_is_nonzero_on_parameter_range(weight, domain.finite, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        let weighted_target = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let delta_x = polynomial_subtract(&weighted_target(point.x()), source.x_numerator);
        let delta_y = polynomial_subtract(&weighted_target(point.y()), source.y_numerator);
        if distance_sign == RealSign::Zero {
            return common_polynomial_roots(delta_x, delta_y, domain, policy);
        }

        let (tangent_x, tangent_y) = match frame {
            Some(frame) => (frame.x.as_slice(), frame.y.as_slice()),
            None => {
                let differential = self.differential()?;
                (
                    differential.tangent_x.as_slice(),
                    differential.tangent_y.as_slice(),
                )
            }
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(tangent_x, tangent_x),
            &polynomial_multiply(tangent_y, tangent_y),
        );
        let orthogonality = polynomial_add(
            &polynomial_multiply(&delta_x, tangent_x),
            &polynomial_multiply(&delta_y, tangent_y),
        );
        let weighted_distance = match source.weight {
            Some(weight) => polynomial_scale(weight, self.distance()),
            None => vec![self.distance().clone()],
        };
        let distance_relation = polynomial_subtract(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_multiply(&weighted_distance, &weighted_distance),
        );
        let incidence =
            match common_polynomial_roots(orthogonality, distance_relation, domain, policy)? {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

        let orientation = polynomial_subtract(
            &polynomial_multiply(&delta_y, tangent_x),
            &polynomial_multiply(&delta_x, tangent_y),
        );
        let orientation = match source.weight {
            Some(weight) => polynomial_multiply(&orientation, weight),
            None => orientation,
        };
        let branch = match polynomial_from_coefficients(
            polynomial_scale(&orientation, self.distance()),
            policy,
        )? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        match incidence {
            BezierParallelIncidence2::EntireCurve => {
                // A complete matching domain needs a normal everywhere.
                // Isolated contacts below only need their own regular frame;
                // unrelated stationary source parameters cannot exclude them.
                match polynomial_is_nonzero_on_parameter_range(
                    &speed_squared,
                    domain.finite,
                    policy,
                )? {
                    Classification::Decided(true) => (),
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match polynomial_roots_in_parameter_domain(&branch, domain, policy)? {
                    Classification::Decided(roots) if roots.is_empty() => {}
                    Classification::Decided(_) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match domain
                    .finite
                    .start()
                    .polynomial_sign(branch.coefficients(), policy)?
                {
                    Classification::Decided(RealSign::Positive) => Ok(Classification::Decided(
                        BezierParallelIncidence2::EntireCurve,
                    )),
                    Classification::Decided(RealSign::Negative) => Ok(Classification::Decided(
                        BezierParallelIncidence2::Parameters(Vec::new()),
                    )),
                    Classification::Decided(RealSign::Zero) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            BezierParallelIncidence2::Parameters(candidates) => {
                let mut retained = Vec::with_capacity(candidates.len());
                for candidate in candidates {
                    match signed_coefficients_at_parameter(&speed_squared, &candidate, policy)? {
                        Classification::Decided(RealSign::Positive) => (),
                        Classification::Decided(RealSign::Zero) => {
                            // A one-sided normal can survive a stationary
                            // source parameter. Its absence needs more proof.
                            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                        }
                        Classification::Decided(RealSign::Negative) => {
                            return Err(CurveError::Topology(
                                "a parallel source had negative squared speed".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    match signed_coefficients_at_parameter(
                        branch.coefficients(),
                        &candidate,
                        policy,
                    )? {
                        Classification::Decided(RealSign::Positive) => retained.push(candidate),
                        Classification::Decided(RealSign::Negative) => {}
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "parallel branch vanished at a regular nonzero-distance incidence"
                                    .to_owned(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(Classification::Decided(
                    BezierParallelIncidence2::Parameters(retained),
                ))
            }
        }
    }

    /// Classifies whether `point` belongs to this parallel's finite range.
    pub fn contains_point(
        &self,
        point: &Point2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .point_incidence(point, range, policy)?
            .map(|incidence| match incidence {
                BezierParallelIncidence2::EntireCurve => true,
                BezierParallelIncidence2::Parameters(parameters) => !parameters.is_empty(),
            }))
    }

    /// Classifies a retained algebraic point on the consumed finite parallel
    /// and, when supplied, its certified regular incident continuation.
    /// The point's selected field becomes the query axis of the
    /// existing exact parallel fiber system; the parallel itself remains an
    /// unevaluated normal-offset carrier.
    pub(crate) fn contains_point_evidence(
        &self,
        point: &CurvePoint2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .visit_point_incidence_evidence(point, range, incident, false, policy, &mut |_| {
                ControlFlow::Break(())
            })?
            .map(|flow| flow.is_break()))
    }

    /// Visits borrowed exact point-incidence evidence without rebuilding or
    /// copying retained root handles. A collecting caller clones its witnesses.
    /// `Some(parameter)` is a certified isolated contact; `None` denotes the
    /// entire queried domain. `Break` stops at the first requested witness.
    ///
    /// A continuing visitor may see a parameter again when independent proof
    /// routes overlap. Only `Decided(Continue(()))` proves a complete visit;
    /// uncertainty leaves certified witnesses but does not prove exhaustiveness.
    /// `regular_domain` selects the normal of an owned regular source cell,
    /// including its one-sided endpoint limits. Incident extensions must still
    /// certify that they stay on this cell's source-normal sheet.
    pub(crate) fn visit_point_incidence_evidence(
        &self,
        point: &CurvePoint2,
        range: &CurveParameterRange2,
        incident: Option<&BezierParallelIncidentDomain2>,
        regular_domain: bool,
        policy: &CurveContext,
        visitor: &mut impl FnMut(Option<&CurveParameter2>) -> ControlFlow<()>,
    ) -> CurveResult<Classification<ControlFlow<()>>> {
        let expanded = match incident
            .map(|incident| incident.expanded_range(range, policy))
            .transpose()?
        {
            Some(Classification::Decided(range)) => Some(range),
            Some(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            None => None,
        };
        let domain = CurveParameterDomain2::new(
            expanded.as_ref().unwrap_or(range),
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let frame = || {
            if regular_domain {
                self.source_tangent_field_in_regular_domain(domain, policy)
            } else {
                Ok(Classification::Decided(None))
            }
        };
        let retained = |point: &CurvePoint2, visitor: &mut _| {
            let frame = match frame()? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            BezierParallelPointQuery2 {
                parallel: self,
                range,
                frame: frame.as_deref(),
            }
            .visit_point_parameters(point, incident, domain, policy, visitor)
        };
        let CurvePoint2(CurvePointData2::Algebraic(point)) = point else {
            return match point {
                CurvePoint2(CurvePointData2::Endpoint(endpoint)) => {
                    match endpoint.resolve(policy)? {
                        Classification::Decided(Some(point)) => self
                            .visit_point_incidence_evidence(
                                &point,
                                range,
                                incident,
                                regular_domain,
                                policy,
                                visitor,
                            ),
                        Classification::Decided(None) => {
                            Ok(Classification::Uncertain(UncertaintyReason::RealSign))
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    }
                }
                CurvePoint2(CurvePointData2::Exact(point)) => {
                    let frame = match frame()? {
                        Classification::Decided(frame) => frame,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    self.point_incidence_with_tangent_field(point, domain, frame.as_deref(), policy)
                        .map(|result| {
                            result.map(|incidence| match incidence {
                                BezierParallelIncidence2::EntireCurve => visitor(None),
                                BezierParallelIncidence2::Parameters(parameters) => {
                                    for parameter in parameters {
                                        let parameter = CurveParameter2::from(parameter);
                                        if let stop @ ControlFlow::Break(()) =
                                            visitor(Some(&parameter))
                                        {
                                            return stop;
                                        }
                                    }
                                    ControlFlow::Continue(())
                                }
                            })
                        })
                }
                CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
                    return match point.predicate_point_evidence(policy)? {
                        Classification::Decided(Some(point)) => self
                            .visit_point_incidence_evidence(
                                &point,
                                range,
                                incident,
                                regular_domain,
                                policy,
                                visitor,
                            ),
                        // The optional Cartesian view is not the point's
                        // domain. Its source parameter and positive speed
                        // already share a retained field with exact replay.
                        Classification::Decided(None) => {
                            retained(&CurvePoint2::from(point.clone()), visitor)
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    };
                }
                CurvePoint2(CurvePointData2::Similarity(point)) => {
                    return match point.predicate_point_evidence(policy)? {
                        Classification::Decided(Some(point)) => self
                            .visit_point_incidence_evidence(
                                &point,
                                range,
                                incident,
                                regular_domain,
                                policy,
                                visitor,
                            ),
                        Classification::Decided(None) => {
                            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                        }
                        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                    };
                }
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => point
                    .visit_incidence_on_parallel(
                        self,
                        range,
                        incident,
                        regular_domain,
                        policy,
                        visitor,
                    ),
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                    retained(point, visitor)
                }
                CurvePoint2(CurvePointData2::Algebraic(_)) => unreachable!(),
            };
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        retained(&CurvePoint2::from(point.point_image().clone()), visitor)
    }

    pub(super) fn source_circle_polynomial(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameterPolynomial>> {
        let source = self.source_power_basis()?;
        let weight = source
            .weight
            .map_or_else(|| vec![Real::one()], ToOwned::to_owned);
        let delta_x =
            polynomial_subtract(source.x_numerator, &polynomial_scale(&weight, center.x()));
        let delta_y =
            polynomial_subtract(source.y_numerator, &polynomial_scale(&weight, center.y()));
        let incidence = polynomial_subtract(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_scale(&polynomial_multiply(&weight, &weight), radius_squared),
        );
        match polynomial_from_coefficients(incidence, policy)? {
            Classification::Decided(Some(polynomial)) => Ok(Classification::Decided(polynomial)),
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Boundary))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Solves source-curve circle incidence on the complete finite source
    /// domain without imposing analytic-parallel tangent regularity.
    pub(crate) fn source_circle_incidence(
        &self,
        center: &Point2,
        radius_squared: &Real,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        let source = self.source_power_basis()?;
        if let Some(weight) = source.weight {
            match polynomial_is_nonzero_on_parameter_range(weight, range, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let incidence = match self.source_circle_polynomial(center, radius_squared, policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        CurveParameterDomain2::new(range, None).finite_roots(&incidence, policy)
    }

    /// Solves source-curve circle incidence on the finite component of one
    /// incident exterior parameter ray.
    ///
    /// The first rational weight zero is a projective pole barrier: roots at
    /// or beyond it do not belong to the endpoint's connected affine support.
    /// Polynomial sources have no barrier. This is the shared exterior-domain
    /// authority used by exact chamfer setback construction.
    pub(crate) fn source_circle_incidence_on_incident_ray(
        &self,
        center: &Point2,
        radius_squared: &Real,
        anchor: &Real,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        let incidence = match self.source_circle_polynomial(center, radius_squared, policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidates = match incidence.isolate_incident_ray_roots(anchor, direction, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let source = self.source_power_basis()?;
        let weight = if let Some(weight) = source.weight {
            match polynomial_from_coefficients(weight.to_vec(), policy)? {
                Classification::Decided(Some(weight)) => Some(weight),
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            None
        };
        let barrier = if let Some(weight) = weight.as_ref() {
            match weight.isolate_incident_ray_roots(anchor, direction, policy)? {
                Classification::Decided(roots) => roots.into_iter().next(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            None
        };

        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            if let Some(barrier) = barrier.as_ref() {
                let ordering = match candidate.cmp_by_refinement(barrier, policy)? {
                    Classification::Decided(ordering) => ordering,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let before_barrier = match direction {
                    BezierParameterRayDirection2::Decreasing => {
                        ordering == std::cmp::Ordering::Greater
                    }
                    BezierParameterRayDirection2::Increasing => {
                        ordering == std::cmp::Ordering::Less
                    }
                };
                if !before_barrier {
                    continue;
                }
            }
            if let Some(weight) = weight.as_ref() {
                match signed_coefficients_at_parameter(weight.coefficients(), &candidate, policy)? {
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                    Classification::Decided(RealSign::Zero) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            retained.push(candidate);
        }
        Ok(Classification::Decided(retained))
    }

    /// Returns the first source pole or source-speed zero on one endpoint ray.
    /// A stationary anchor is already a speed barrier and owns an empty
    /// extension. Otherwise the open ray stops before its first barrier.
    pub(crate) fn incident_ray_regular_barrier(
        &self,
        anchor: &Real,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let speed_squared = parallel_speed_squared_polynomial(differential);
        regular_incident_ray_barrier_from_polynomials(
            source.weight,
            &speed_squared,
            anchor,
            direction,
            policy,
        )
    }

    /// Builds one regular extension domain from a finite scalar endpoint.
    /// Ordinary algebraic roots and retained selected/recursive scalars keep
    /// their native regularity proofs. Any endpoint-to-anchor bridge remains
    /// part of the domain; no approximate value selects the incident chart.
    pub(crate) fn incident_domain_from_parameter(
        &self,
        endpoint: &CurveParameter2,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidentDomain2>> {
        let anchor = if let Some(BezierParameter2::Exact(value)) = endpoint.as_bezier_parameter() {
            BezierIncidentRayAnchor2 {
                adjacent_interval: None,
                represented_anchor: value.clone(),
            }
        } else {
            if endpoint.as_bezier_parameter().is_none() && !endpoint.is_retained_scalar() {
                return Err(CurveError::Topology(
                    "a parallel incident domain requires a finite scalar endpoint".into(),
                ));
            }
            let source = self.source_power_basis()?;
            let differential = self.differential()?;
            let speed_squared = parallel_speed_squared_polynomial(differential);
            let anchor = if let Some(BezierParameter2::Algebraic(endpoint)) =
                endpoint.as_bezier_parameter()
            {
                algebraic_incident_ray_regular_anchor_from_polynomials(
                    source.weight,
                    &speed_squared,
                    endpoint,
                    direction,
                )
            } else {
                retained_incident_ray_regular_anchor_from_polynomials(
                    source.weight,
                    &speed_squared,
                    endpoint,
                    direction,
                    policy,
                )
            };
            match anchor? {
                Classification::Decided(anchor) => anchor,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        };
        let barrier = match self.incident_ray_regular_barrier(
            &anchor.represented_anchor,
            direction,
            policy,
        )? {
            Classification::Decided(barrier) => barrier,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(BezierParallelIncidentDomain2 {
            endpoint: endpoint.clone(),
            bridge: anchor.adjacent_interval,
            anchor: anchor.represented_anchor,
            direction,
            barrier,
        }))
    }

    /// A parallel of an affine parameterization differs from its source only
    /// by a constant normal translation, so its parameter speed is constant.
    /// Return the exact `setback / speed` parameter displacement; general
    /// nonlinear carriers deliberately decline this fast path.
    pub(super) fn affine_fixed_distance_parameter_delta(
        &self,
        setback: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let source = self.source_power_basis()?;
        let structurally_affine = |coefficients: &[Real]| {
            coefficients
                .iter()
                .skip(2)
                .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        };
        if !structurally_affine(source.x_numerator) || !structurally_affine(source.y_numerator) {
            return Ok(Classification::Decided(None));
        }
        let weight = match source.weight {
            None => Real::one(),
            Some(weight)
                if weight
                    .iter()
                    .skip(1)
                    .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero) =>
            {
                let Some(weight) = weight.first() else {
                    return Err(CurveError::Topology(
                        "an affine rational source had no weight coefficient".into(),
                    ));
                };
                match policy.strict_predicate_pass(|| real_sign(weight, policy)) {
                    Some(RealSign::Positive | RealSign::Negative) => weight.clone(),
                    Some(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
            }
            Some(_) => return Ok(Classification::Decided(None)),
        };
        let derivative = |coefficients: &[Real]| -> CurveResult<Real> {
            (coefficients.get(1).cloned().unwrap_or_else(Real::zero) / &weight).map_err(Into::into)
        };
        let derivative_x = derivative(source.x_numerator)?;
        let derivative_y = derivative(source.y_numerator)?;
        let speed_squared = &derivative_x * &derivative_x + &derivative_y * &derivative_y;
        match policy.strict_predicate_pass(|| real_sign(&speed_squared, policy)) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an affine parallel source had negative squared speed".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.sqrt()?;
        Ok(Classification::Decided(Some((setback / speed)?)))
    }

    /// Returns both fixed-distance parameters from one compact selected
    /// parameter when this analytic parallel has an affine source chart.
    #[cfg(test)]
    pub(crate) fn affine_fixed_distance_parameters_from_selected_parameter(
        &self,
        center: &BezierAlgebraicSelectedFiberParameter2,
        setback: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[BezierAlgebraicSelectedFiberParameter2; 2]>>> {
        center.validate_policy(policy)?;
        Ok(
            match self.affine_fixed_distance_parameter_delta(setback, policy)? {
                Classification::Decided(Some(delta)) => Classification::Decided(Some([
                    center.translated(&(-delta.clone())),
                    center.translated(&delta),
                ])),
                Classification::Decided(None) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Isolates one retained `H(alpha,v)=0` image on the finite source chart
    /// and, when requested, its complete regular incident cell.
    ///
    /// `None` means the selected relation is identically zero on at least one
    /// requested cell. Keeping that outcome distinct lets the caller descend
    /// to an unsquared lower equation instead of promoting the selected center.
    pub(super) fn selected_fiber_image_parameters(
        &self,
        center: &BezierAlgebraicSelectedFiberParameter2,
        incidence: &BivariatePolynomial,
        global_schedule: Option<Vec<Real>>,
        range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
        policy.strict_predicate_pass(|| {
            let retained = &center.data.authority.data.retained_parameter;
            let candidates = if let Some(coefficients) = global_schedule {
                let (coefficients, square_free) = match hypersolve::square_free_part(
                    coefficients.clone(),
                    hypersolve::PredicatePolicy::STRICT,
                ) {
                    Some(square_free) => (square_free, true),
                    None => (coefficients, false),
                };
                let polynomial =
                    match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
                        Classification::Decided(polynomial) => polynomial,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let schedule = if square_free && range == &CurveParameterRange2::unit() {
                    polynomial.isolate_square_free_unit_interval_roots(policy)?
                } else {
                    CurveParameterDomain2::new(range, None).finite_roots(&polynomial, policy)?
                };
                let schedule = match schedule {
                    Classification::Decided(schedule) => schedule,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let exact_authority = BezierAlgebraicSelectedFiberAuthority2::new(
                    incidence.clone(),
                    retained.clone(),
                    policy,
                );
                let retained_parameter = BezierParameter2::Algebraic(retained.clone());
                let mut selected = Vec::with_capacity(schedule.len());
                for candidate in schedule {
                    match candidate {
                        BezierParameter2::Exact(candidate) => {
                            match signed_coefficients_at_parameter(
                                &bivariate_specialize_second(incidence, &candidate),
                                &retained_parameter,
                                policy,
                            )? {
                                Classification::Decided(RealSign::Zero) => {
                                    selected.push(exact_authority.parameter(
                                        IsolatedRootInterval {
                                            lower: candidate.clone(),
                                            upper: candidate.clone(),
                                            exact_root: Some(candidate),
                                            distinct_root_count: 1,
                                        },
                                    ));
                                }
                                Classification::Decided(
                                    RealSign::Negative | RealSign::Positive,
                                ) => {}
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        BezierParameter2::Algebraic(candidate) => {
                            match selected_fiber_parameters_in_interval(
                                incidence,
                                retained,
                                candidate.interval().start(),
                                candidate.interval().end(),
                                policy,
                            )? {
                                Classification::Decided(Some(mut local)) => {
                                    selected.append(&mut local)
                                }
                                Classification::Decided(None) => {
                                    return Ok(Classification::Decided(None));
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                    }
                }
                selected
            } else {
                match selected_fiber_parameters_in_range(incidence, retained, range, policy)? {
                    Classification::Decided(Some(candidates)) => candidates,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let mut finite = Vec::with_capacity(candidates.len());
            for candidate in candidates {
                match CurveParameterDomain2::new(range, None).contains_finite_parameter(
                    &CurveParameter2::from_selected_fiber(candidate.clone()),
                    policy,
                )? {
                    Classification::Decided(true) => finite.push(candidate),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let mut candidates = finite;
            if let Some(direction) = direction {
                let endpoint = CurveParameter2::from_selected_fiber(center.clone());
                let incident =
                    match self.incident_domain_from_parameter(&endpoint, direction, policy)? {
                        Classification::Decided(incident) => incident,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                if let Some(bridge) = incident.bridge() {
                    let adjacent = match selected_fiber_parameters_in_interval(
                        incidence,
                        retained,
                        bridge.start(),
                        bridge.end(),
                        policy,
                    )? {
                        Classification::Decided(Some(parameters)) => parameters,
                        Classification::Decided(None) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let mut retained_adjacent = Vec::with_capacity(adjacent.len());
                    for parameter in adjacent {
                        let ordering = match parameter.cmp_by_refinement(center, policy)? {
                            Classification::Decided(ordering) => ordering,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let beyond_endpoint = match direction {
                            BezierParameterRayDirection2::Decreasing => {
                                ordering == std::cmp::Ordering::Less
                            }
                            BezierParameterRayDirection2::Increasing => {
                                ordering == std::cmp::Ordering::Greater
                            }
                        };
                        if beyond_endpoint
                            && match CurveParameterDomain2::new(range, None)
                                .contains_finite_parameter(
                                    &CurveParameter2::from_selected_fiber(parameter.clone()),
                                    policy,
                                )? {
                                Classification::Decided(inside) => !inside,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        {
                            retained_adjacent.push(parameter);
                        }
                    }
                    if direction == BezierParameterRayDirection2::Decreasing {
                        retained_adjacent.reverse();
                    }
                    candidates.extend(retained_adjacent);
                }
                let exterior = match selected_fiber_parameters_on_incident_ray(
                    incidence,
                    retained,
                    incident.anchor(),
                    incident.direction(),
                    incident.barrier(),
                    policy,
                )? {
                    Classification::Decided(Some(parameters)) => parameters,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                for parameter in exterior {
                    match CurveParameterDomain2::new(range, None).contains_finite_parameter(
                        &CurveParameter2::from_selected_fiber(parameter.clone()),
                        policy,
                    )? {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => candidates.push(parameter),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            Ok(Classification::Decided(Some(candidates)))
        })
    }

    /// Projects one lower fixed-distance equation through the compact center
    /// authority. A selected zero-image source factor is reported as an
    /// identically-zero equation; a foreign factor is saturated and ignored.
    pub(super) fn selected_fiber_fixed_distance_equation_projection(
        &self,
        center: &BezierAlgebraicSelectedFiberParameter2,
        equation: &BivariatePolynomial,
        range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<SelectedParallelEquationProjection2>> {
        let image = match center.retained_polynomial_image_relation(equation, policy)? {
            Classification::Decided(Some(image)) => image,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let had_zero_source_factor = image.identically_zero_source_factor.is_some();
        if let Some(factor) = image.identically_zero_source_factor {
            match center.predicate_sign(&factor, policy)? {
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(
                        SelectedParallelEquationProjection2::IdenticallyZero,
                    ));
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if image.identically_zero_image_relation {
            return if had_zero_source_factor {
                Ok(Classification::Uncertain(UncertaintyReason::Predicate))
            } else {
                Ok(Classification::Decided(
                    SelectedParallelEquationProjection2::IdenticallyZero,
                ))
            };
        }
        let Some(incidence) = image.relation else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        Ok(
            match self.selected_fiber_image_parameters(
                center,
                &incidence,
                image.global_schedule,
                range,
                direction,
                policy,
            )? {
                Classification::Decided(Some(parameters)) => Classification::Decided(
                    SelectedParallelEquationProjection2::Candidates(parameters),
                ),
                Classification::Decided(None) => {
                    Classification::Decided(SelectedParallelEquationProjection2::IdenticallyZero)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(super) fn selected_fiber_parallel_normal_positive_dimensional_projection(
        &self,
        center: &BezierAlgebraicSelectedFiberParameter2,
        system: &BezierParallelFixedDistanceSystem2,
        range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierSelectedParallelNormalPositiveProjection2>> {
        let center_speed_squared =
            bivariate_specialize_second(&system.center_speed_squared, &Real::zero());
        let candidate_speed_squared =
            bivariate_specialize_first(&system.candidate_speed_squared, &Real::zero());
        if system.center_speed_squared
            == bivariate_outer_product(&center_speed_squared, &[Real::one()])
            && system.candidate_speed_squared
                == bivariate_outer_product(&[Real::one()], &candidate_speed_squared)
            && let (
                Classification::Decided(Some(center_speed)),
                Classification::Decided(Some(candidate_speed)),
            ) = (
                positive_polynomial_speed_at(
                    &center_speed_squared,
                    &CurveParameter2::from_selected_fiber(center.clone()),
                    policy,
                )?,
                positive_polynomial_speed_at(&candidate_speed_squared, range.start(), policy)?,
            )
        {
            let incidence = collapse_two_normal_polynomial_speeds(
                &system.circle,
                &center_speed,
                &candidate_speed,
            );
            match self.selected_fiber_fixed_distance_equation_projection(
                center, &incidence, range, direction, policy,
            )? {
                Classification::Decided(SelectedParallelEquationProjection2::Candidates(
                    parameters,
                )) => {
                    return Ok(Classification::Decided(
                        BezierSelectedParallelNormalPositiveProjection2::AuthoredPolynomialCandidates {
                            parameters,
                            incidence,
                        },
                    ));
                }
                Classification::Decided(SelectedParallelEquationProjection2::IdenticallyZero) => {
                    return Ok(Classification::Decided(
                        BezierSelectedParallelNormalPositiveProjection2::CoincidentCircleComponent,
                    ));
                }
                Classification::Uncertain(_) => {}
            }
        }
        let project = |equation: &BivariatePolynomial| {
            self.selected_fiber_fixed_distance_equation_projection(
                center, equation, range, direction, policy,
            )
        };
        let selected_expression_is_zero = |expression: &BezierAlgebraicCuspTwoTermExpression2| {
            selected_fiber_square_root_polynomial_is_identically_zero(
                expression,
                &system.center_speed_squared,
                center,
                policy,
            )
        };
        parallel_normal_positive_dimensional_projection(
            &system.circle,
            &system.squared_branch,
            &system.center_speed_squared,
            &system.candidate_speed_squared,
            range,
            &project,
            &selected_expression_is_zero,
            policy,
        )
    }

    /// Enumerates exact fixed-distance contacts on this support from a point
    /// in another support's parameter chart. Finite chart enumeration is
    /// independent of the chart that owns the corner. An incident extension
    /// ray is meaningful only when the center uses this same support chart.
    pub(crate) fn fixed_distance_incidence(
        &self,
        center_parallel: &Self,
        center: &CurveParameter2,
        setback: &Real,
        isolation_range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        if direction.is_some() && self != center_parallel {
            return Err(CurveError::Topology(
                "a fixed-distance extension needs an anchor in its support chart".into(),
            ));
        }
        if let Some(parameter) = center.as_bezier_parameter() {
            self.fixed_distance_incidence_from_parameter_with_domain(
                center_parallel,
                parameter,
                &(setback * setback),
                isolation_range,
                direction,
                policy,
            )
        } else if let Some(parameter) = center.as_selected_fiber() {
            self.fixed_distance_incidence_from_selected_parameter(
                center_parallel,
                parameter,
                setback,
                isolation_range,
                direction,
                policy,
            )
        } else if center.as_recursive_projective().is_some() {
            self.fixed_distance_incidence_from_recursive_parameter(
                center_parallel,
                center,
                setback,
                isolation_range,
                direction,
                policy,
            )
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
    }

    /// Solves fixed-distance cuts from one compact selected-fiber center.
    ///
    /// Affine charts translate the local root directly. General charts retain
    /// the projected cut equation as `H(alpha, v)=0`, so every cut shares the
    /// center's already-selected base root and no global center scalar is
    /// constructed. A twice-squared two-normal equation is enumeration only:
    /// its candidates are correlated with the authored center and replayed on
    /// the unsquared radical sheet. When both positive speeds are polynomial,
    /// the authored equation is projected directly instead.
    pub(super) fn fixed_distance_incidence_from_selected_parameter(
        &self,
        center_parallel: &Self,
        center: &BezierAlgebraicSelectedFiberParameter2,
        setback: &Real,
        isolation_range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        center.validate_policy(policy)?;
        match if self == center_parallel {
            self.affine_fixed_distance_parameter_delta(setback, policy)?
        } else {
            Classification::Decided(None)
        } {
            Classification::Decided(Some(delta)) => {
                let endpoint = CurveParameter2::from_selected_fiber(center.clone());
                let incident = if let Some(direction) = direction {
                    match self.incident_domain_from_parameter(&endpoint, direction, policy)? {
                        Classification::Decided(incident) => Some(incident),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    None
                };
                let mut retained = Vec::with_capacity(2);
                for candidate in [
                    center.translated(&(-delta.clone())),
                    center.translated(&delta),
                ] {
                    let candidate = CurveParameter2::from_selected_fiber(candidate);
                    match CurveParameterDomain2::new(isolation_range, None)
                        .contains_finite_parameter(&candidate, policy)?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            let Some(incident) = incident.as_ref() else {
                                continue;
                            };
                            match incident.contains_extension_parameter(&candidate, policy)? {
                                Classification::Decided(true) => {}
                                Classification::Decided(false) => continue,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    retained.push(candidate);
                }
                return Ok(Classification::Decided(retained));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let radius_squared = setback * setback;
        let retained_parameter =
            BezierParameter2::Algebraic(center.data.authority.data.retained_parameter.clone());
        let retains_base_root = matches!(
            policy.strict_predicate_pass(|| {
                center.cmp_bezier_parameter(&retained_parameter, policy)
            })?,
            Classification::Decided(std::cmp::Ordering::Equal)
        );
        if !retains_base_root
            && let Classification::Decided(parameter) =
                policy.strict_predicate_pass(|| center.promoted_bezier_parameter(policy))?
        {
            // Prefer the bounded ordinary projection when this local scalar
            // has a compact global parameter. Fixed-distance incidence can
            // then reduce the two-normal equation in that low-degree field
            // instead of taking the norm of the much larger distance image.
            // The identity fiber `u = alpha` remains local because its
            // positive-dimensional and radical-sheet correlations are more
            // informative than an independently projected scalar. High-degree
            // fibers likewise decline this bounded promotion.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-fiber-fixed-distance",
                "bounded-ordinary-center-promotion",
            );
            return self.fixed_distance_incidence_from_parameter_with_domain(
                center_parallel,
                &parameter,
                &radius_squared,
                isolation_range,
                direction,
                policy,
            );
        }
        let (projected_incidence, radical_system) =
            if real_sign(self.distance(), &CurveContext::STRICT) == Some(RealSign::Zero)
                && real_sign(center_parallel.distance(), &CurveContext::STRICT)
                    == Some(RealSign::Zero)
            {
                let incidence = match parallel_source_fixed_distance_incidence(
                    center_parallel,
                    self,
                    &radius_squared,
                    &CurveParameter2::from_selected_fiber(center.clone()),
                    isolation_range,
                    policy,
                )? {
                    Classification::Decided(incidence) => incidence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (incidence, None)
            } else {
                let system = match parallel_fixed_distance_system(
                    center_parallel,
                    self,
                    &radius_squared,
                    isolation_range,
                    &CurveParameter2::from_selected_fiber(center.clone()),
                    policy,
                )? {
                    Classification::Decided(system) => system,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (system.incidence.clone(), Some(system))
            };
        let selected_image =
            match center.retained_polynomial_image_relation(&projected_incidence, policy)? {
                Classification::Decided(Some(image)) => image,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let had_zero_source_factor = selected_image.identically_zero_source_factor.is_some();
        let mut positive_dimensional =
            selected_image.identically_zero_image_relation && !had_zero_source_factor;
        if let Some(factor) = selected_image.identically_zero_source_factor {
            match center.predicate_sign(&factor, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-fiber-fixed-distance",
                        "saturated-foreign-zero-image-component",
                    );
                }
                Classification::Decided(RealSign::Zero) => {
                    positive_dimensional = true;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if selected_image.identically_zero_image_relation && !positive_dimensional {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        let mut candidates = None;
        if !positive_dimensional {
            let Some(selected_incidence) = selected_image.relation else {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            };
            match self.selected_fiber_image_parameters(
                center,
                &selected_incidence,
                selected_image.global_schedule,
                isolation_range,
                direction,
                policy,
            )? {
                Classification::Decided(Some(parameters)) => {
                    candidates = Some(parameters);
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-fiber-fixed-distance",
                        "retained-local-image",
                    );
                }
                Classification::Decided(None) => {
                    positive_dimensional = true;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let projected_root_required = !positive_dimensional;
        let mut authored_positive_incidence = None;
        let candidates =
            if let Some(candidates) = candidates {
                candidates
            } else if let Some(system) = &radical_system {
                match self.selected_fiber_parallel_normal_positive_dimensional_projection(
                center, system, isolation_range, direction, policy,
            )? {
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::Candidates(parameters),
                ) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-fiber-fixed-distance",
                        "retained-positive-dimensional-lower-equation",
                    );
                    parameters
                }
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::AuthoredPolynomialCandidates {
                        parameters,
                        incidence,
                    },
                ) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-fiber-fixed-distance",
                        "retained-positive-polynomial-speed-equation",
                    );
                    authored_positive_incidence = Some(incidence);
                    parameters
                }
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::CoincidentCircleComponent,
                ) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "selected-fiber-fixed-distance",
                        "coincident-fixed-distance-component",
                    );
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Decided(
                    BezierSelectedParallelNormalPositiveProjection2::Degenerate,
                ) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            } else {
                // The unsquared source-circle equation itself vanishes for every
                // candidate parameter on this component. The fixed-distance cut
                // is a continuum rather than a finite chamfer solution set.
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            };
        let authored_source_is_unique = center
            .data
            .authority
            .data
            .incidence
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default()
            <= 2;
        let mut retained_candidates = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            if projected_root_required {
                let projected_root = match algebraic_selected_fiber_pair_projected_root(
                    center,
                    &candidate,
                    &projected_incidence,
                    policy,
                )? {
                    Classification::Decided(projected_root) => projected_root,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if !projected_root {
                    continue;
                }
            }
            if !authored_source_is_unique && let Some(incidence) = &authored_positive_incidence {
                let authored_root = match algebraic_selected_fiber_pair_projected_root(
                    center, &candidate, incidence, policy,
                )? {
                    Classification::Decided(root) => root,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if !authored_root {
                    continue;
                }
            }
            if authored_positive_incidence.is_none()
                && let Some(system) = &radical_system
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "selected-fiber-fixed-distance",
                    "unsquared-two-normal-replay",
                );
                match algebraic_selected_fiber_pair_two_normal_sum_sign(
                    center,
                    &candidate,
                    &system.circle,
                    &system.center_speed_squared,
                    &system.candidate_speed_squared,
                    true,
                    policy,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "selected-fiber-fixed-distance",
                            "unsquared-two-normal-replay-undecided",
                        );
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            retained_candidates.push(CurveParameter2::from_selected_fiber(candidate));
        }
        Ok(Classification::Decided(retained_candidates))
    }

    /// Builds the fixed-distance equation from one recursively retained
    /// center parameter without projecting that center to an ordinary scalar.
    /// The center-speed root is selected once under STRICT; candidate-speed
    /// squaring remains enumeration-only and is replayed for every root.
    pub(super) fn recursive_fixed_distance_system(
        &self,
        center_parallel: &Self,
        center: &BezierRecursiveProjectiveParameter2,
        radius_squared: &Real,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRecursiveFixedDistanceSystem2>> {
        center.validate_policy(policy)?;
        // A zero-distance parallel is its rational source curve. Preserve the
        // unsquared Cartesian circle equation directly: neither endpoint
        // speed participates, stationary candidate parameters remain valid,
        // and the projected degree is far smaller than the generic two-normal
        // formulation.
        if real_sign(self.distance(), &CurveContext::STRICT) == Some(RealSign::Zero)
            && real_sign(center_parallel.distance(), &CurveContext::STRICT) == Some(RealSign::Zero)
        {
            let incidence = match parallel_source_fixed_distance_incidence(
                center_parallel,
                self,
                radius_squared,
                &CurveParameter2::from_recursive_projective(center.clone()),
                range,
                policy,
            )? {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let first_degree = bivariate_first_active_degree(&incidence);
            let Some(rational) = recursive_projective_bivariate_first_parameter_polynomial(
                &incidence,
                center,
                first_degree,
            ) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(scalar) = center.projective_scalar() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let field = scalar.numerator.field();
            let Some(zero) = field.constant(Real::zero()) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let radical = vec![zero];
            let Some((base, projection)) =
                recursive_quadratic_polynomial_projection(rational.clone())
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let field_base = field.base_and_extension_path().0;
            if !recursive_quadratic_bases_equivalent(&base, &field_base) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let source = self.source_power_basis()?;
            let unit = [Real::one()];
            let source_weight = source.weight.unwrap_or(&unit);
            let Some(source_weight) = recursive_quadratic_real_polynomial(&field, source_weight)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(speed_squared) = recursive_quadratic_real_polynomial(&field, &unit) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            return Ok(Classification::Decided(
                BezierRecursiveFixedDistanceSystem2 {
                    field,
                    base,
                    projection,
                    incidence: BezierRecursiveQuadraticParallelExpression2::new(
                        rational,
                        radical,
                        speed_squared.into(),
                    ),
                    source_weight,
                    unit_target_speed: true,
                },
            ));
        }
        let BezierParallelFixedDistanceSystem2 { circle, .. } =
            match parallel_fixed_distance_system(
                center_parallel,
                self,
                radius_squared,
                range,
                &CurveParameter2::from_recursive_projective(center.clone()),
                policy,
            )? {
                Classification::Decided(system) => system,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let differential = center_parallel.differential()?;
        let tangent_degree = differential
            .tangent_x
            .len()
            .max(differential.tangent_y.len())
            .saturating_sub(1);
        let Some(speed_degree) = tangent_degree.checked_mul(2) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let speed_squared = parallel_speed_squared_polynomial(differential);
        let Some(speed_squared) = center.homogeneous_polynomial_value(&speed_squared, speed_degree)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match policy.strict_predicate_pass(|| speed_squared.sign(policy))? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a recursive fixed-distance center had negative squared speed".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let parent = speed_squared.field();
        let Some(field) = parent.extension(speed_squared) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(center_speed) = field.element(
            parent.constant(Real::zero()).ok_or_else(|| {
                CurveError::Topology("a recursive fixed-distance center lost its zero".into())
            })?,
            parent.constant(Real::one()).ok_or_else(|| {
                CurveError::Topology("a recursive fixed-distance center lost its unit".into())
            })?,
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };

        // `product` and `center` multiply sqrt(S_center); `candidate` and
        // `rational` do not. Raise every first-axis evaluation to one common
        // projective degree before combining the authored equation.
        let speed_terms_degree = bivariate_first_active_degree(&circle.product)
            .max(bivariate_first_active_degree(&circle.center));
        let Some(speed_terms_degree) = speed_terms_degree.checked_add(tangent_degree) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let common_degree = speed_terms_degree
            .max(bivariate_first_active_degree(&circle.candidate))
            .max(bivariate_first_active_degree(&circle.rational));
        let Some(with_center_speed_degree) = common_degree.checked_sub(tangent_degree) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let substitute = |polynomial: &BivariatePolynomial, degree| {
            recursive_projective_bivariate_first_parameter_polynomial(polynomial, center, degree)?
                .into_iter()
                .map(|coefficient| field.lift(&coefficient))
                .collect::<Option<Vec<_>>>()
        };
        let Some((product, center_term, candidate, rational)) = (|| {
            Some((
                substitute(&circle.product, with_center_speed_degree)?,
                substitute(&circle.center, with_center_speed_degree)?,
                substitute(&circle.candidate, common_degree)?,
                substitute(&circle.rational, common_degree)?,
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let source = self.source_power_basis()?;
        let unit = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit);
        let candidate_speed_squared = parallel_speed_squared_polynomial(self.differential()?);
        let Some((source_weight, candidate_speed_squared)) = (|| {
            Some((
                recursive_quadratic_real_polynomial(&field, source_weight)?,
                recursive_quadratic_real_polynomial(&field, &candidate_speed_squared)?,
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(incidence) = (|| {
            let rational = recursive_quadratic_polynomial_combine(
                &recursive_quadratic_polynomial_scale(&center_term, &center_speed)?,
                &rational,
                false,
            )?;
            let radical = recursive_quadratic_polynomial_combine(
                &recursive_quadratic_polynomial_scale(&product, &center_speed)?,
                &candidate,
                false,
            )?;
            Some(BezierRecursiveQuadraticParallelExpression2::new(
                rational,
                radical,
                candidate_speed_squared.into(),
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(projected_coefficients) = incidence.squared_magnitude_difference() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some((base, projection)) =
            recursive_quadratic_polynomial_projection(projected_coefficients.to_vec())
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let field_base = field.base_and_extension_path().0;
        if !recursive_quadratic_bases_equivalent(&base, &field_base) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Ok(Classification::Decided(
            BezierRecursiveFixedDistanceSystem2 {
                field,
                base,
                projection,
                incidence,
                source_weight,
                unit_target_speed: false,
            },
        ))
    }

    /// Solves fixed-distance candidates from one recursive center over the
    /// finite source domain and, when requested, its single regular incident
    /// extension cell. The affine-line projection is filtered back through
    /// that exact domain so no root beyond a pole or speed barrier is admitted.
    pub(super) fn fixed_distance_incidence_from_recursive_parameter(
        &self,
        center_parallel: &Self,
        center: &CurveParameter2,
        setback: &Real,
        range: &CurveParameterRange2,
        direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        let Some(parameter) = center.as_recursive_projective() else {
            return Err(CurveError::Topology(
                "recursive fixed-distance incidence requires a recursive center".into(),
            ));
        };
        match if self == center_parallel {
            self.affine_fixed_distance_parameter_delta(setback, policy)?
        } else {
            Classification::Decided(None)
        } {
            Classification::Decided(Some(delta)) => {
                let first = match parameter.translated(&(-delta.clone()), policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let second = match parameter.translated(&delta, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let incident = if let Some(direction) = direction {
                    match self.incident_domain_from_parameter(center, direction, policy)? {
                        Classification::Decided(incident) => Some(incident),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    None
                };
                let mut retained = Vec::with_capacity(2);
                for candidate in [first, second] {
                    let in_range = match CurveParameterDomain2::new(range, None)
                        .contains_finite_parameter(
                            &CurveParameter2::from_recursive_projective(candidate.clone()),
                            policy,
                        )? {
                        Classification::Decided(inside) => inside,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    if !in_range {
                        let Some(incident) = incident.as_ref() else {
                            continue;
                        };
                        match incident.contains_extension_parameter(
                            &CurveParameter2::from_recursive_projective(candidate.clone()),
                            policy,
                        )? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    retained.push(CurveParameter2::from_recursive_projective(candidate));
                }
                return Ok(Classification::Decided(retained));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let radius_squared = setback * setback;
        let system = match self.recursive_fixed_distance_system(
            center_parallel,
            parameter,
            &radius_squared,
            range,
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let incident = if let Some(direction) = direction {
            match self.incident_domain_from_parameter(center, direction, policy)? {
                Classification::Decided(incident) => Some(incident),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            None
        };
        let expanded_range = if let Some(incident) = incident.as_ref() {
            match incident.expanded_range(range, policy)? {
                Classification::Decided(expanded) => Some(expanded),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            None
        };
        let domain = CurveParameterDomain2::new(
            expanded_range.as_ref().unwrap_or(range),
            incident
                .as_ref()
                .map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let projected = match selected_axis_parameters_in_domain(domain, policy, |axis| {
            system.parameters(&system.projection, axis, policy)
        })? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let (candidates, transverse_certificate_applies) = match projected {
            BezierAlgebraicFiberProjection2::Parameters(candidates) => (candidates, true),
            BezierAlgebraicFiberProjection2::IdenticallyZero => {
                let strict = policy.strict_counterpart();
                let rational_zero = match recursive_quadratic_polynomial_is_identically_zero(
                    &system.incidence.rational,
                    &strict,
                )? {
                    Classification::Decided(zero) => zero,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radical_zero = match recursive_quadratic_polynomial_is_identically_zero(
                    &system.incidence.radical,
                    &strict,
                )? {
                    Classification::Decided(zero) => zero,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if rational_zero && radical_zero {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                let mut residual = None;
                for coefficients in [&system.incidence.rational, &system.incidence.radical] {
                    let Some(projection) = system.projected_polynomial(coefficients) else {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    };
                    match selected_axis_parameters_in_domain(domain, policy, |axis| {
                        system.parameters(&projection, axis, policy)
                    })? {
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
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let Some(candidates) = residual else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                (candidates, false)
            }
            BezierAlgebraicFiberProjection2::Degenerate => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let in_range = match CurveParameterDomain2::new(range, None)
                .contains_finite_parameter(&candidate.clone().into(), policy)?
            {
                Classification::Decided(inside) => inside,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if !in_range {
                let Some(incident) = incident.as_ref() else {
                    continue;
                };
                match incident.contains_extension_parameter(
                    &CurveParameter2::from(candidate.clone()),
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if transverse_certificate_applies
                && let Some(is_root) = system.expression_root_by_interval(&candidate)
            {
                if is_root {
                    retained.push(CurveParameter2::from(candidate));
                }
                continue;
            }
            let evaluation = match system.candidate_evaluation(&candidate, policy)? {
                Classification::Decided(Some(evaluation)) => evaluation,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match policy.strict_predicate_pass(|| system.expression_sign(&evaluation, policy))? {
                Classification::Decided(RealSign::Zero) => {
                    retained.push(CurveParameter2::from(candidate))
                }
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(retained))
    }

    /// Solves analytic-parallel circle incidence on the regular affine cell
    /// incident to one authored endpoint.
    ///
    /// Candidate roots use the same squared two-normal relation and exact
    /// branch replay as [`Self::circle_incidence`]. The first source-weight or
    /// source-speed zero terminates the cell, so no returned root crosses a
    /// projective pole or a normal-field singularity. Roots remain ordered
    /// away from `anchor`.
    pub(crate) fn circle_incidence_on_incident_ray(
        &self,
        center: &Point2,
        radius_squared: &Real,
        incident: &BezierParallelIncidentDomain2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<(BezierParameter2, Option<RealSign>)>>> {
        let anchor = incident.anchor();
        let direction = incident.direction();
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let weight_coefficients = source
            .weight
            .map_or_else(|| vec![Real::one()], ToOwned::to_owned);
        let speed_squared = parallel_speed_squared_polynomial(differential);

        let delta_x = polynomial_subtract(
            source.x_numerator,
            &polynomial_scale(&weight_coefficients, center.x()),
        );
        let delta_y = polynomial_subtract(
            source.y_numerator,
            &polynomial_scale(&weight_coefficients, center.y()),
        );
        let distance_square_delta = self.distance() * self.distance() - radius_squared;
        let radial = polynomial_add(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_scale(
                &polynomial_multiply(&weight_coefficients, &weight_coefficients),
                &distance_square_delta,
            ),
        );
        let normal_projection = polynomial_subtract(
            &polynomial_multiply(&delta_y, &differential.tangent_x),
            &polynomial_multiply(&delta_x, &differential.tangent_y),
        );
        let normal = polynomial_scale(
            &polynomial_multiply(&weight_coefficients, &normal_projection),
            &(Real::from(2_u8) * self.distance()),
        );
        let squared = polynomial_subtract(
            &polynomial_multiply(&polynomial_multiply(&radial, &radial), &speed_squared),
            &polynomial_multiply(&normal, &normal),
        );
        let candidate_polynomial = match polynomial_from_coefficients(squared, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                // A coincident squared carrier has no isolated fillet center;
                // deciding its selected component requires overlap topology.
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidates =
            match candidate_polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
                Classification::Decided(candidates) => candidates,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let candidates = match retain_parameters_before_incident_barrier(
            candidates,
            incident.barrier(),
            direction,
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let radial_sign = match signed_coefficients_at_parameter(&radial, &candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let normal_sign = match signed_coefficients_at_parameter(&normal, &candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match (radial_sign, normal_sign) {
                (RealSign::Zero, RealSign::Zero)
                | (RealSign::Positive, RealSign::Negative)
                | (RealSign::Negative, RealSign::Positive) => {
                    retained.push((candidate, None));
                }
                (RealSign::Positive, RealSign::Positive)
                | (RealSign::Negative, RealSign::Negative) => {}
                (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "squared parallel-circle candidate lost its exact mate".into(),
                    ));
                }
            }
        }
        Ok(Classification::Decided(retained))
    }

    /// Solves circle incidence while retaining the oriented radial crossing
    /// sign and reusing represented tangent contacts certified by exact
    /// offset-join construction.
    ///
    /// Tangency makes each supplied parameter a root of the squared eliminant
    /// with multiplicity at least two. Exact synthetic division removes those
    /// known factors before Sturm isolation, avoiding a request that the scalar
    /// layer rediscover nested-radical cancellation. Every remaining root is
    /// still isolated and branch-filtered by the complete generic engine.
    pub(crate) fn circle_incidence(
        &self,
        center: &Point2,
        radius_squared: &Real,
        range: &CurveParameterRange2,
        certified_tangent_parameters: &[(Real, u8)],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<(BezierParameter2, Option<RealSign>)>>> {
        self.circle_incidence_with_tangent_field(
            center,
            radius_squared,
            range,
            certified_tangent_parameters,
            None,
            policy,
        )
    }

    pub(super) fn circle_incidence_with_tangent_field(
        &self,
        center: &Point2,
        radius_squared: &Real,
        range: &CurveParameterRange2,
        certified_tangent_parameters: &[(Real, u8)],
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<(BezierParameter2, Option<RealSign>)>>> {
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
        let speed_squared = parallel_speed_squared_polynomial(differential);
        for coefficients in source
            .weight
            .into_iter()
            .chain(Some(speed_squared.as_slice()))
        {
            match polynomial_is_nonzero_on_parameter_range(coefficients, range, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let domain = CurveParameterDomain2::new(range, None);
        let weight = source
            .weight
            .map_or_else(|| vec![Real::one()], ToOwned::to_owned);
        let delta_x =
            polynomial_subtract(source.x_numerator, &polynomial_scale(&weight, center.x()));
        let delta_y =
            polynomial_subtract(source.y_numerator, &polynomial_scale(&weight, center.y()));
        let distance_square_delta = self.distance() * self.distance() - radius_squared;
        let radial = polynomial_add(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_scale(
                &polynomial_multiply(&weight, &weight),
                &distance_square_delta,
            ),
        );
        let normal_projection = polynomial_subtract(
            &polynomial_multiply(&delta_y, &differential.tangent_x),
            &polynomial_multiply(&delta_x, &differential.tangent_y),
        );
        let normal = polynomial_scale(
            &polynomial_multiply(&weight, &normal_projection),
            &(Real::from(2_u8) * self.distance()),
        );
        // A zero offset is the source itself. Its unsquared radial equation
        // already owns every circle contact and its oriented derivative;
        // squaring would double roots and discard transverse sign evidence.
        let zero_distance = self.distance().zero_status() == ZeroKnowledge::Zero;
        let radial_derivative = zero_distance.then(|| polynomial_derivative(&radial));
        let mut eliminant = if zero_distance {
            radial.clone()
        } else {
            polynomial_subtract(
                &polynomial_multiply(&polynomial_multiply(&radial, &radial), &speed_squared),
                &polynomial_multiply(&normal, &normal),
            )
        };
        let mut certified_parameters: Vec<(&Real, u8)> =
            Vec::with_capacity(certified_tangent_parameters.len());
        for (parameter, multiplicity) in certified_tangent_parameters {
            if *multiplicity == 0 {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            // Speed is nonzero on this domain, so a certified multiplicity
            // m in radial^2 * speed^2 gives ceil(m/2) in the radial equation.
            let multiplicity = if zero_distance {
                multiplicity.div_ceil(2)
            } else {
                *multiplicity
            };
            if let Some(index) = certified_parameters
                .iter()
                .position(|(retained, _)| *retained == parameter)
            {
                let previous_multiplicity = certified_parameters[index].1;
                if previous_multiplicity >= multiplicity {
                    continue;
                }
                for _ in previous_multiplicity..multiplicity {
                    if eliminant.len() < 2 {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    eliminant = divide_by_linear_root(&eliminant, parameter);
                }
                certified_parameters[index].1 = multiplicity;
                continue;
            }
            for _ in 0..multiplicity {
                if eliminant.len() < 2 {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                eliminant = divide_by_linear_root(&eliminant, parameter);
            }
            certified_parameters.push((parameter, multiplicity));
        }
        let candidate_polynomial = match polynomial_from_coefficients(eliminant, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidates = match domain.finite_roots(&candidate_polynomial, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let simple_roots = candidate_polynomial.simple_root_classifications(&candidates, policy)?;
        let candidate_derivative = polynomial_derivative(candidate_polynomial.coefficients());
        let mut retained = Vec::with_capacity(candidates.len() + certified_parameters.len());
        for (candidate, simple_root) in candidates.into_iter().zip(simple_roots) {
            let radial_sign = match signed_coefficients_at_parameter(&radial, &candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let normal_sign = match signed_coefficients_at_parameter(&normal, &candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match (radial_sign, normal_sign) {
                (RealSign::Zero, RealSign::Zero)
                | (RealSign::Positive, RealSign::Negative)
                | (RealSign::Negative, RealSign::Positive) => {
                    let radial_crossing_sign = if let Some(derivative) = &radial_derivative {
                        // At incidence, d(|P-C|^2-R^2)/dt = radial'/weight^2.
                        // The denominator is positive even for a negative gauge.
                        match signed_coefficients_at_parameter(derivative, &candidate, policy)? {
                            Classification::Decided(sign) => Some(sign),
                            Classification::Uncertain(_) => None,
                        }
                    } else if matches!(simple_root, Classification::Decided(true))
                        && radial_sign != RealSign::Zero
                    {
                        let derivative_sign = match signed_coefficients_at_parameter(
                            &candidate_derivative,
                            &candidate,
                            policy,
                        )? {
                            Classification::Decided(
                                sign @ (RealSign::Positive | RealSign::Negative),
                            ) => sign,
                            Classification::Decided(RealSign::Zero) => {
                                return Err(CurveError::Topology(
                                    "a certified simple circle-incidence root had zero derivative"
                                        .into(),
                                ));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let mut squared_derivative_sign = derivative_sign;
                        for (deflated_parameter, multiplicity) in &certified_parameters {
                            if multiplicity % 2 == 0 {
                                continue;
                            }
                            match candidate.cmp_by_refinement(
                                &BezierParameter2::Exact((*deflated_parameter).clone()),
                                policy,
                            )? {
                                Classification::Decided(std::cmp::Ordering::Less) => {
                                    squared_derivative_sign = match squared_derivative_sign {
                                        RealSign::Positive => RealSign::Negative,
                                        RealSign::Negative => RealSign::Positive,
                                        RealSign::Zero => unreachable!(
                                            "a simple quotient root has nonzero derivative"
                                        ),
                                    };
                                }
                                Classification::Decided(std::cmp::Ordering::Greater) => {}
                                Classification::Decided(std::cmp::Ordering::Equal) => {
                                    return Err(CurveError::Topology(
                                        "a residual circle-incidence root duplicated a deflated tangent"
                                            .into(),
                                    ));
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        Some(if squared_derivative_sign == radial_sign {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    } else if matches!(simple_root, Classification::Decided(false))
                        && radial_sign != RealSign::Zero
                        && matches!(
                            policy.strict_predicate_pass(|| self
                                .parallel_derivative_scale_sign(
                                    &candidate.clone().into(),
                                    policy
                                ))?,
                            Classification::Decided(RealSign::Positive | RealSign::Negative)
                        )
                    {
                        // The other speed sheet is nonzero when radial != 0.
                        // A repeated norm root therefore proves zero radial
                        // derivative on the selected sheet. Retain this as a
                        // tangent certificate only on a regular target branch;
                        // a zero offset derivative needs its one-sided frame.
                        Some(RealSign::Zero)
                    } else {
                        None
                    };
                    retained.push((candidate, radial_crossing_sign));
                }
                (RealSign::Positive, RealSign::Positive)
                | (RealSign::Negative, RealSign::Negative) => {}
                (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "squared parallel-circle candidate lost its exact mate".into(),
                    ));
                }
            }
        }
        for (parameter, _) in certified_parameters {
            match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            let candidate = BezierParameter2::Exact(parameter.clone());
            let mut insert_at = retained.len();
            for (index, existing) in retained.iter_mut().enumerate() {
                let ordering = candidate.cmp_by_refinement(&existing.0, policy)?;
                match ordering {
                    Classification::Decided(std::cmp::Ordering::Less) => {
                        insert_at = index;
                        break;
                    }
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        // A construction certificate is a lower bound on the
                        // contact order. If a residual factor remains at the
                        // same parameter, it is still the certified tangent,
                        // never a transverse root of the quotient.
                        *existing = (candidate.clone(), Some(RealSign::Zero));
                        insert_at = usize::MAX;
                        break;
                    }
                    Classification::Decided(std::cmp::Ordering::Greater) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if insert_at != usize::MAX {
                retained.insert(insert_at, (candidate, Some(RealSign::Zero)));
            }
        }
        Ok(Classification::Decided(retained))
    }

    /// Replays an ordinary center parameter over a finite candidate range
    /// and, when requested, the regular ray incident to that same chart.
    pub(super) fn fixed_distance_incidence_from_parameter_with_domain(
        &self,
        center_parallel: &Self,
        center_parameter: &BezierParameter2,
        radius_squared: &Real,
        isolation_range: &CurveParameterRange2,
        incident_direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        let BezierParameter2::Algebraic(center_parameter) = center_parameter else {
            let center_parameter = center_parameter
                .scalar()
                .expect("represented fixed-distance center has an exact parameter");
            let center = match center_parallel.point_at(center_parameter, policy)? {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut parameters = match if real_sign(self.distance(), &CurveContext::STRICT)
                == Some(RealSign::Zero)
            {
                self.source_circle_incidence(&center, radius_squared, isolation_range, policy)
                    .map(|result| {
                        result.map(|parameters| {
                            parameters
                                .into_iter()
                                .map(|parameter| (parameter, None))
                                .collect()
                        })
                    })
            } else {
                self.circle_incidence(&center, radius_squared, isolation_range, &[], policy)
            }? {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Some(direction) = incident_direction {
                let endpoint = CurveParameter2::from(center_parameter.clone());
                let incident =
                    match self.incident_domain_from_parameter(&endpoint, direction, policy)? {
                        Classification::Decided(incident) => incident,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let exterior = match self.circle_incidence_on_incident_ray(
                    &center,
                    radius_squared,
                    &incident,
                    policy,
                )? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                parameters.extend(exterior);
            }
            return Ok(Classification::Decided(
                parameters
                    .into_iter()
                    .map(|(parameter, _)| CurveParameter2::from(parameter))
                    .collect(),
            ));
        };

        if real_sign(self.distance(), &CurveContext::STRICT) == Some(RealSign::Zero)
            && real_sign(center_parallel.distance(), &CurveContext::STRICT) == Some(RealSign::Zero)
        {
            let incidence = match parallel_source_fixed_distance_incidence(
                center_parallel,
                self,
                radius_squared,
                &CurveParameter2::from(BezierParameter2::Algebraic(center_parameter.clone())),
                isolation_range,
                policy,
            )? {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let incidence =
                match reduce_algebraic_cusp_bivariate(incidence, center_parameter, policy)? {
                    Classification::Decided(incidence) => incidence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            return self.fixed_distance_candidate_parameters(
                &incidence,
                center_parameter,
                isolation_range,
                incident_direction,
                policy,
            );
        }

        let BezierParallelFixedDistanceSystem2 {
            incidence,
            center_speed_squared,
            candidate_speed_squared,
            squared_branch,
            circle,
        } = match parallel_fixed_distance_system(
            center_parallel,
            self,
            radius_squared,
            isolation_range,
            &CurveParameter2::from(BezierParameter2::Algebraic(center_parameter.clone())),
            policy,
        )? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        // Polynomial source speeds make the authored distance relation
        // polynomial on each regular cell. Project that relation directly;
        // replaying a twice-squared equation would introduce avoidable
        // conjugates and repeated refinement of equivalent square roots.
        let center_scalar =
            CurveParameter2::from(BezierParameter2::Algebraic(center_parameter.clone()));
        let connected = incident_direction.is_none()
            || matches!(
                CurveParameterDomain2::new(isolation_range, None)
                    .contains_finite_parameter(&center_scalar, policy)?,
                Classification::Decided(true)
            );
        if connected
            && let (
                Classification::Decided(Some(center_speed)),
                Classification::Decided(Some(candidate_speed)),
            ) = (
                positive_polynomial_speed_at(
                    &bivariate_specialize_second(&center_speed_squared, &Real::zero()),
                    &center_scalar,
                    policy,
                )?,
                positive_polynomial_speed_at(
                    &bivariate_specialize_first(&candidate_speed_squared, &Real::zero()),
                    isolation_range.start(),
                    policy,
                )?,
            )
        {
            let incidence =
                collapse_two_normal_polynomial_speeds(&circle, &center_speed, &candidate_speed);
            let incidence =
                match reduce_algebraic_cusp_bivariate(incidence, center_parameter, policy)? {
                    Classification::Decided(incidence) => incidence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            return self.fixed_distance_candidate_parameters(
                &incidence,
                center_parameter,
                isolation_range,
                incident_direction,
                policy,
            );
        }
        let incidence = match reduce_algebraic_cusp_bivariate(incidence, center_parameter, policy)?
        {
            Classification::Decided(incidence) => incidence,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_speed_squared = match reduce_algebraic_cusp_bivariate(
            center_speed_squared,
            center_parameter,
            policy,
        )? {
            Classification::Decided(speed_squared) => speed_squared,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let reduce_expression = |expression| {
            reduce_algebraic_cusp_radical_expression(expression, center_parameter, policy)
        };
        let squared_branch = match reduce_expression(squared_branch)? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_term = match reduce_expression(BezierAlgebraicCuspTwoTermExpression2 {
            rational: circle.rational,
            radical: circle.center,
        })? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidate_term = match reduce_expression(BezierAlgebraicCuspTwoTermExpression2 {
            rational: circle.candidate,
            radical: circle.product,
        })? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let circle = BezierParallelTwoNormalExpression2 {
            product: candidate_term.radical.clone(),
            center: center_term.radical.clone(),
            candidate: candidate_term.rational.clone(),
            rational: center_term.rational.clone(),
        };
        let candidates = match self.fixed_distance_candidate_parameters(
            &incidence,
            center_parameter,
            isolation_range,
            incident_direction,
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let center_parameter = BezierParameter2::Algebraic(center_parameter.clone());
        let radical_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2,
                            candidate: &CurveParameter2| {
            if let Some(candidate) = candidate.as_bezier_parameter() {
                algebraic_cusp_selected_square_root_sum_sign(
                    &incidence,
                    expression,
                    &center_speed_squared,
                    &center_parameter,
                    candidate,
                    policy,
                )
            } else if let Some(candidate) = candidate.as_selected_fiber() {
                candidate.square_root_sum_sign(expression, &center_speed_squared, policy)
            } else {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            if let Some(parameter) = candidate.as_selected_fiber() {
                let sign = parameter.two_normal_sum_sign(
                    &circle,
                    &center_speed_squared,
                    &candidate_speed_squared,
                    policy,
                )?;
                match sign {
                    Classification::Decided(RealSign::Zero) => retained.push(candidate),
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                continue;
            }
            match radical_sign(&squared_branch, &candidate)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let center_sign = match radical_sign(&center_term, &candidate)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidate_sign = match radical_sign(&candidate_term, &candidate)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match (center_sign, candidate_sign) {
                (RealSign::Zero, RealSign::Zero)
                | (RealSign::Positive, RealSign::Negative)
                | (RealSign::Negative, RealSign::Positive) => retained.push(candidate),
                (RealSign::Positive, RealSign::Positive)
                | (RealSign::Negative, RealSign::Negative) => {}
                (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "fixed-distance radical candidate lost its exact mate".into(),
                    ));
                }
            }
        }
        Ok(Classification::Decided(retained))
    }

    pub(super) fn fixed_distance_candidate_parameters(
        &self,
        incidence: &BivariatePolynomial,
        center_parameter: &BezierAlgebraicParameter2,
        isolation_range: &CurveParameterRange2,
        incident_direction: Option<BezierParameterRayDirection2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        let finite_range = isolation_range;
        let candidates: Vec<CurveParameter2> =
            if center_parameter.polynomial().degree() > MAX_FIXED_DISTANCE_QUOTIENT_DEGREE {
                // A global norm multiplies the candidate degree by the selected
                // center field degree. Retain high-degree contacts directly in
                // Q(alpha), sharing the reduced incidence across every root.
                let (_, [isolation_lower, isolation_upper]) =
                    match CurveParameterDomain2::new(finite_range, None).finite_envelope(policy)? {
                        Classification::Decided(range) => range,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                let roots = match selected_fiber_parameters_in_interval(
                    incidence,
                    center_parameter,
                    isolation_lower,
                    isolation_upper,
                    policy,
                )? {
                    Classification::Decided(Some(roots)) => roots,
                    Classification::Decided(None) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                roots
                    .into_iter()
                    .map(CurveParameter2::from_selected_fiber)
                    .collect()
            } else {
                match algebraic_selected_reduced_fiber_parameters_with_resultant_limit(
                    incidence,
                    center_parameter,
                    MAX_FIXED_DISTANCE_RESULTANT_DEGREE,
                    MAX_FIXED_DISTANCE_QUOTIENT_DEGREE,
                    finite_range,
                    policy,
                )? {
                    Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                        candidates,
                    )) => candidates.into_iter().map(CurveParameter2::from).collect(),
                    Classification::Decided(
                        BezierAlgebraicFiberProjection2::IdenticallyZero
                        | BezierAlgebraicFiberProjection2::Degenerate,
                    ) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
        let mut finite = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let inside = CurveParameterDomain2::new(isolation_range, None)
                .contains_finite_parameter(&candidate, policy)?;
            match inside {
                Classification::Decided(true) => finite.push(candidate),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let mut candidates = finite;
        let Some(direction) = incident_direction else {
            return Ok(Classification::Decided(candidates));
        };
        let center = BezierParameter2::Algebraic(center_parameter.clone());
        let incident =
            match self.incident_domain_from_parameter(&center.clone().into(), direction, policy)? {
                Classification::Decided(incident) => incident,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if let Some(interval) = incident.bridge() {
            let adjacent = match selected_fiber_parameters_in_interval(
                incidence,
                center_parameter,
                interval.start(),
                interval.end(),
                policy,
            )? {
                Classification::Decided(Some(parameters)) => parameters,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut retained_adjacent = Vec::with_capacity(adjacent.len());
            for parameter in adjacent {
                let ordering = match parameter.cmp_bezier_parameter(&center, policy)? {
                    Classification::Decided(ordering) => ordering,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let beyond_endpoint = match direction {
                    BezierParameterRayDirection2::Decreasing => {
                        ordering == std::cmp::Ordering::Less
                    }
                    BezierParameterRayDirection2::Increasing => {
                        ordering == std::cmp::Ordering::Greater
                    }
                };
                if beyond_endpoint {
                    retained_adjacent.push(parameter);
                }
            }
            if direction == BezierParameterRayDirection2::Decreasing {
                retained_adjacent.reverse();
            }
            candidates.extend(
                retained_adjacent
                    .into_iter()
                    .map(CurveParameter2::from_selected_fiber),
            );
        }
        let exterior = match selected_fiber_parameters_on_incident_ray(
            incidence,
            center_parameter,
            incident.anchor(),
            incident.direction(),
            incident.barrier(),
            policy,
        )? {
            Classification::Decided(Some(parameters)) => parameters,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        candidates.extend(
            exterior
                .into_iter()
                .map(CurveParameter2::from_selected_fiber),
        );
        Ok(Classification::Decided(candidates))
    }

    /// Returns two inverse rational-quadratic parameter charts on a supporting
    /// circle as rational functions of this parallel's source parameter.
    ///
    /// A line through the conic start and a circle point gives the ordinary
    /// rational inverse `u=2A/(2A-B)`. Parallel coordinates contain
    /// `1/sqrt(S)`, but on circle incidence the selected equation
    /// `radial + normal/sqrt(S)=0` eliminates that radical. The resulting two
    /// power-basis polynomials let `rational_parameter_image` carry an isolated
    /// parallel root directly into the conic parameter tower. Start- and
    /// end-anchored charts cover one another's projective denominator pole.
    /// Radical elimination requires a nonzero divisor at every queried root.
    /// Otherwise a certified rational parallel can supply the coordinates
    /// directly, without cancelling an exceptional incidence fiber.
    pub(super) fn circle_rational_quadratic_parameter_maps<'a>(
        &self,
        center: &Point2,
        radius_squared: &Real,
        conic: &RationalQuadraticBezier2,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        parameters: impl IntoIterator<Item = &'a BezierParameter2>,
        policy: &CurveContext,
    ) -> CurveResult<Option<[(Vec<Real>, Vec<Real>); 2]>> {
        let mut parameters = parameters.into_iter().peekable();
        if parameters.peek().is_none() {
            return Ok(None);
        }
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let (tangent_x, tangent_y) = tangent_field
            .map(|field| (&field.x[..], &field.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));
        let unit_weight = [Real::one()];
        let weight = source.weight.unwrap_or(&unit_weight);
        let weighted = |coordinate: &Real| polynomial_scale(weight, coordinate);
        let delta_x = polynomial_subtract(source.x_numerator, &weighted(center.x()));
        let delta_y = polynomial_subtract(source.y_numerator, &weighted(center.y()));
        let distance_square_delta = self.distance() * self.distance() - radius_squared;
        let radial = polynomial_add(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_scale(&polynomial_multiply(weight, weight), &distance_square_delta),
        );
        let normal_projection = polynomial_subtract(
            &polynomial_multiply(&delta_y, tangent_x),
            &polynomial_multiply(&delta_x, tangent_y),
        );
        let normal = polynomial_scale(
            &polynomial_multiply(weight, &normal_projection),
            &(Real::from(2_u8) * self.distance()),
        );
        let strict = policy.strict_counterpart();
        let mut rational_parallel = None;
        for parameter in parameters {
            if !matches!(
                signed_coefficients_at_parameter(&normal, parameter, &strict)?,
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            ) {
                // At radial = normal = 0 the incidence equation cannot
                // eliminate the speed radical. A certified rational parallel
                // supplies the original coordinates instead; otherwise keep
                // the exceptional fiber for the general contact authority.
                if tangent_field.is_none()
                    && let Classification::Decided(Some(curve)) =
                        self.exact_rational_parallel_component(&CurveContext::STRICT)?
                {
                    rational_parallel = Some(curve);
                    break;
                }
                return Ok(None);
            }
        }
        let rational_parallel = rational_parallel
            .as_ref()
            .map(RationalBezier2::homogeneous_power_basis)
            .transpose()?;
        let lifted_line = |anchor: &Point2, point: &Point2| {
            let (line_x, line_y) = point.delta_from(anchor);
            if let Some(source) = rational_parallel {
                return polynomial_subtract(
                    &polynomial_scale(
                        &polynomial_subtract(
                            &source.x_numerator,
                            &polynomial_scale(&source.weight, anchor.x()),
                        ),
                        &line_y,
                    ),
                    &polynomial_scale(
                        &polynomial_subtract(
                            &source.y_numerator,
                            &polynomial_scale(&source.weight, anchor.y()),
                        ),
                        &line_x,
                    ),
                );
            }
            let source_from_start_x =
                polynomial_subtract(source.x_numerator, &weighted(anchor.x()));
            let source_from_start_y =
                polynomial_subtract(source.y_numerator, &weighted(anchor.y()));
            let source_line = polynomial_subtract(
                &polynomial_scale(&source_from_start_x, &line_y),
                &polynomial_scale(&source_from_start_y, &line_x),
            );
            let radical_line = polynomial_scale(
                &polynomial_add(
                    &polynomial_scale(tangent_y, &line_y),
                    &polynomial_scale(tangent_x, &line_x),
                ),
                &(-self.distance().clone()),
            );
            polynomial_subtract(
                &polynomial_multiply(&source_line, &normal),
                &polynomial_multiply(weight, &polynomial_multiply(&radical_line, &radial)),
            )
        };
        let chart =
            |anchor: &Point2, opposite: &Point2, opposite_weight: &Real, complement: bool| {
                let control_line = polynomial_scale(
                    &lifted_line(anchor, conic.control()),
                    conic.control_weight(),
                );
                let opposite_line =
                    polynomial_scale(&lifted_line(anchor, opposite), opposite_weight);
                let direct_numerator = polynomial_scale(&control_line, &Real::from(2_u8));
                let denominator = polynomial_subtract(&direct_numerator, &opposite_line);
                if complement {
                    (
                        polynomial_subtract(&denominator, &direct_numerator),
                        denominator,
                    )
                } else {
                    (direct_numerator, denominator)
                }
            };
        Ok(Some([
            chart(conic.start(), conic.end(), conic.end_weight(), false),
            chart(conic.end(), conic.start(), conic.start_weight(), true),
        ]))
    }

    /// Uses a caller-certified nonzero direction and retained tangencies for
    /// the same supporting line.
    ///
    /// Scaling a line direction does not change incidence. Retained line and
    /// chord carriers already own an exact unit tangent, and reusing it avoids
    /// carrying a large endpoint-difference scale through the Sturm sequence.
    /// The finite range always owns root enumeration and regularity checks.
    /// Regularizing its source requires a certified regular interior; that
    /// orientation owns the normal sheet at stationary boundary contacts.
    pub(crate) fn supporting_line_incidence_with_direction(
        &self,
        line: &LineSeg2,
        direction: (&Real, &Real),
        certified_tangencies: &[Real],
        range: &CurveParameterRange2,
        regularize_source: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let tangent_field = if regularize_source {
            match self.source_oriented_regularized_tangent_field(range, policy)? {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            None
        };
        self.supporting_line_incidence_with_certified_contacts(
            line,
            Some(direction),
            &[],
            certified_tangencies,
            false,
            None,
            tangent_field.as_deref(),
            Some(range),
            policy,
        )
    }

    /// Returns every selected-branch supporting-line contact in the regular
    /// affine cell incident to one authored endpoint.
    ///
    /// The same squared line/normal equation as the finite-domain authority is
    /// isolated on the open incident ray. The first source-weight zero or
    /// tangent-speed zero is an exact projective/regularity barrier; contacts
    /// at or beyond it belong to another analytic component and are excluded.
    /// Roots remain in order away from `anchor`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn supporting_line_incidence_on_incident_ray_with_direction(
        &self,
        line: &LineSeg2,
        direction_x: &Real,
        direction_y: &Real,
        incident: &BezierParallelIncidentDomain2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let anchor = incident.anchor();
        let direction = incident.direction();
        if incident.barrier() == Some(&BezierParameter2::Exact(anchor.clone())) {
            return Ok(Classification::Decided(
                BezierParallelIncidence2::Parameters(Vec::new()),
            ));
        }
        match real_sign(self.distance(), policy) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let speed_squared = parallel_speed_squared_polynomial(differential);

        let weighted_start = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let source_from_start_x =
            polynomial_subtract(source.x_numerator, &weighted_start(line.start().x()));
        let source_from_start_y =
            polynomial_subtract(source.y_numerator, &weighted_start(line.start().y()));
        let line_numerator = polynomial_subtract(
            &polynomial_scale(&source_from_start_y, direction_x),
            &polynomial_scale(&source_from_start_x, direction_y),
        );
        let normal_projection = polynomial_add(
            &polynomial_scale(&differential.tangent_x, direction_x),
            &polynomial_scale(&differential.tangent_y, direction_y),
        );
        let signed_normal_term = polynomial_scale(&normal_projection, self.distance());
        let signed_normal_term = match source.weight {
            Some(weight) => polynomial_multiply(&signed_normal_term, weight),
            None => signed_normal_term,
        };
        let line_polynomial = match polynomial_from_coefficients(line_numerator.clone(), policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let normal_polynomial =
            match polynomial_from_coefficients(signed_normal_term.clone(), policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let retain_before_barrier =
            |parameters: Vec<BezierParameter2>| -> CurveResult<Classification<Vec<_>>> {
                retain_parameters_before_incident_barrier(
                    parameters,
                    incident.barrier(),
                    direction,
                    policy,
                )
            };
        let (line_polynomial, normal_polynomial) = match (line_polynomial, normal_polynomial) {
            (None, None) => {
                return Ok(Classification::Decided(
                    BezierParallelIncidence2::EntireCurve,
                ));
            }
            (Some(polynomial), None) | (None, Some(polynomial)) => {
                let parameters =
                    match polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
                        Classification::Decided(parameters) => parameters,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                return Ok(
                    retain_before_barrier(parameters)?.map(BezierParallelIncidence2::Parameters)
                );
            }
            (Some(line), Some(normal)) => (line, normal),
        };

        let squared_relation = polynomial_subtract(
            &polynomial_multiply(
                &polynomial_multiply(&line_numerator, &line_numerator),
                &speed_squared,
            ),
            &polynomial_multiply(&signed_normal_term, &signed_normal_term),
        );
        let squared_polynomial = match polynomial_from_coefficients(squared_relation, policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(squared_polynomial) = squared_polynomial else {
            let line_sign = match polynomial_incident_anchor_sign(
                &line_polynomial,
                anchor,
                direction,
                policy,
            ) {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let normal_sign = match polynomial_incident_anchor_sign(
                &normal_polynomial,
                anchor,
                direction,
                policy,
            ) {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if real_signs_are_opposite(line_sign, normal_sign) {
                return Ok(Classification::Decided(
                    BezierParallelIncidence2::EntireCurve,
                ));
            }
            let parameters =
                match line_polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            return Ok(retain_before_barrier(parameters)?.map(BezierParallelIncidence2::Parameters));
        };

        let parameters =
            match squared_polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let parameters = match retain_before_barrier(parameters)? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branch_product = polynomial_multiply(&line_numerator, &signed_normal_term);
        let mut retained = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            let branch_sign =
                match signed_coefficients_at_parameter(&branch_product, &parameter, policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(_) => {
                        let line_sign = signed_coefficients_at_parameter(
                            line_polynomial.coefficients(),
                            &parameter,
                            policy,
                        )?;
                        let normal_sign = signed_coefficients_at_parameter(
                            normal_polynomial.coefficients(),
                            &parameter,
                            policy,
                        )?;
                        match (line_sign, normal_sign) {
                            (Classification::Decided(line), Classification::Decided(normal)) => {
                                product_sign(line, normal)
                            }
                            (Classification::Uncertain(reason), _)
                            | (_, Classification::Uncertain(reason)) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                };
            match branch_sign {
                RealSign::Negative => retained.push(parameter),
                RealSign::Positive => {}
                RealSign::Zero => {
                    let line_sign = match signed_coefficients_at_parameter(
                        line_polynomial.coefficients(),
                        &parameter,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let normal_sign = match signed_coefficients_at_parameter(
                        normal_polynomial.coefficients(),
                        &parameter,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    match (line_sign, normal_sign) {
                        (RealSign::Zero, RealSign::Zero)
                        | (RealSign::Positive, RealSign::Negative)
                        | (RealSign::Negative, RealSign::Positive) => retained.push(parameter),
                        (RealSign::Positive, RealSign::Positive)
                        | (RealSign::Negative, RealSign::Negative) => {}
                        (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                        | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "squared parallel-line candidate lost its exact mate".into(),
                            ));
                        }
                    }
                }
            }
        }
        Ok(Classification::Decided(
            BezierParallelIncidence2::Parameters(retained),
        ))
    }

    /// Returns the unsigned squared-incidence candidates for one supporting
    /// line. This is a finite-carrier rejection aid: callers may prove that
    /// every point from either normal branch lies outside their retained
    /// segment before paying for the selected-branch sign at an algebraic
    /// root. Candidates that survive finite clipping must still pass the
    /// ordinary selected-branch relation.
    pub(crate) fn supporting_line_squared_incidence(
        &self,
        line: &LineSeg2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let source = self.source_power_basis()?;
        let weighted_start = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let source_from_start_x =
            polynomial_subtract(source.x_numerator, &weighted_start(line.start().x()));
        let source_from_start_y =
            polynomial_subtract(source.y_numerator, &weighted_start(line.start().y()));
        let (line_x, line_y) = line.delta();
        let line_numerator = polynomial_subtract(
            &polynomial_scale(&source_from_start_y, &line_x),
            &polynomial_scale(&source_from_start_x, &line_y),
        );
        // This polynomial only enumerates a superset. Poles and undefined
        // normals must not narrow its domain before exact branch replay.
        let coefficients = if distance_sign == RealSign::Zero {
            line_numerator
        } else {
            let differential = self.differential()?;
            let normal_projection = polynomial_add(
                &polynomial_scale(&differential.tangent_x, &line_x),
                &polynomial_scale(&differential.tangent_y, &line_y),
            );
            let signed_normal_term = polynomial_scale(&normal_projection, self.distance());
            let signed_normal_term = match source.weight {
                Some(weight) => polynomial_multiply(&signed_normal_term, weight),
                None => signed_normal_term,
            };
            polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_multiply(&line_numerator, &line_numerator),
                    &parallel_speed_squared_polynomial(differential),
                ),
                &polynomial_multiply(&signed_normal_term, &signed_normal_term),
            )
        };
        match polynomial_from_coefficients(coefficients, policy)? {
            Classification::Decided(Some(polynomial)) => {
                Ok(CurveParameterDomain2::new(range, None)
                    .finite_roots(&polynomial, policy)?
                    .map(BezierParallelIncidence2::Parameters))
            }
            Classification::Decided(None) => Ok(Classification::Decided(
                BezierParallelIncidence2::EntireCurve,
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Recovers exact point and segment-parameter evidence for one certified
    /// supporting-line incidence parameter.
    ///
    /// For homogeneous source point `(X/W,Y/W)`, tangent numerator `H`, line
    /// direction `V`, and source-line numerator `L`, incidence gives
    ///
    /// `L/W + d(V dot H)/sqrt(H dot H) = 0`.
    ///
    /// When `D=V dot H` is nonzero this eliminates the radical from the
    /// parallel point:
    ///
    /// `Q=((XD+H_yL)/(WD), (YD-H_xL)/(WD))`.
    ///
    /// When `D=0`, certified incidence also has `L=0`; regularity then makes
    /// the left unit normal equal to the represented unit line direction up
    /// to the exact sign of `cross(H,V)`.  Thus both mathematical branches
    /// remain rational images of the selected source parameter.  The returned
    /// line parameter is absent exactly when the contact lies outside the
    /// finite segment domain `[0,1]`.
    pub(crate) fn supporting_line_contact_evidence(
        &self,
        line: &LineSeg2,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(CurvePoint2, Option<BezierParameter2>)>> {
        self.supporting_line_contact_evidence_impl(line, parameter, false, policy)
    }

    /// Recovers supporting-line contact evidence with an unclipped finite
    /// affine parameter on `line`. This is valid only after an incident-cell
    /// authority has excluded projective and regularity barriers.
    pub(crate) fn supporting_line_contact_evidence_affine(
        &self,
        line: &LineSeg2,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(CurvePoint2, Option<BezierParameter2>)>> {
        self.supporting_line_contact_evidence_impl(line, parameter, true, policy)
    }

    pub(super) fn supporting_line_contact_evidence_impl(
        &self,
        line: &LineSeg2,
        parameter: &BezierParameter2,
        affine_line_parameter: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(CurvePoint2, Option<BezierParameter2>)>> {
        if let Some(parameter) = parameter.scalar() {
            let point = match self.point_at(parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (line_x, line_y) = line.delta();
            let (from_start_x, from_start_y) = point.delta_from(line.start());
            let denominator = &line_x * &line_x + &line_y * &line_y;
            let line_parameter = ((from_start_x * &line_x + from_start_y * &line_y) / denominator)?;
            let line_parameter = if affine_line_parameter {
                Some(BezierParameter2::Exact(line_parameter))
            } else {
                match in_closed_unit_interval(&line_parameter, policy) {
                    Some(true) => Some(BezierParameter2::Exact(line_parameter)),
                    Some(false) => None,
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            };
            return Ok(Classification::Decided((point.into(), line_parameter)));
        }

        let BezierParameter2::Algebraic(algebraic_parameter) = parameter else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let source = self.source_power_basis()?;
        if let Classification::Uncertain(reason) =
            Self::certify_finite_source(&source, &CurveContext::STRICT)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        let differential = self.differential()?;
        if let Classification::Uncertain(reason) =
            Self::certify_regular_differential(differential, &CurveContext::STRICT)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        let weight = source
            .weight
            .map_or_else(|| vec![Real::one()], <[Real]>::to_vec);
        let weighted_start = |coordinate: &Real| polynomial_scale(&weight, coordinate);
        let source_from_start_x =
            polynomial_subtract(source.x_numerator, &weighted_start(line.start().x()));
        let source_from_start_y =
            polynomial_subtract(source.y_numerator, &weighted_start(line.start().y()));
        let (line_x, line_y) = line.delta();
        let line_numerator = polynomial_subtract(
            &polynomial_scale(&source_from_start_y, &line_x),
            &polynomial_scale(&source_from_start_x, &line_y),
        );
        let tangent_projection = polynomial_add(
            &polynomial_scale(&differential.tangent_x, &line_x),
            &polynomial_scale(&differential.tangent_y, &line_y),
        );
        let strict = &CurveContext::STRICT;
        let tangent_projection_sign =
            match signed_coefficients_at_parameter(&tangent_projection, parameter, strict)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

        let (x_numerator, y_numerator, denominator) = if tangent_projection_sign != RealSign::Zero {
            (
                polynomial_add(
                    &polynomial_multiply(source.x_numerator, &tangent_projection),
                    &polynomial_multiply(&differential.tangent_y, &line_numerator),
                ),
                polynomial_subtract(
                    &polynomial_multiply(source.y_numerator, &tangent_projection),
                    &polynomial_multiply(&differential.tangent_x, &line_numerator),
                ),
                polynomial_multiply(&weight, &tangent_projection),
            )
        } else {
            match signed_coefficients_at_parameter(&line_numerator, parameter, strict)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "parallel supporting-line incidence lost its exact zero-projection mate"
                            .to_owned(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let tangent_cross = polynomial_subtract(
                &polynomial_scale(&differential.tangent_x, &line_y),
                &polynomial_scale(&differential.tangent_y, &line_x),
            );
            let orientation =
                match signed_coefficients_at_parameter(&tangent_cross, parameter, strict)? {
                    Classification::Decided(RealSign::Positive) => Real::one(),
                    Classification::Decided(RealSign::Negative) => -Real::one(),
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "regular parallel tangent vanished at a supporting-line contact"
                                .to_owned(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let line_length = (&line_x * &line_x + &line_y * &line_y).sqrt()?;
            let displacement = (self.distance() * orientation / line_length)?;
            (
                polynomial_add(
                    source.x_numerator,
                    &polynomial_scale(&weight, &(&line_x * &displacement)),
                ),
                polynomial_add(
                    source.y_numerator,
                    &polynomial_scale(&weight, &(&line_y * displacement)),
                ),
                weight,
            )
        };

        let from_start_x = polynomial_subtract(
            &x_numerator,
            &polynomial_scale(&denominator, line.start().x()),
        );
        let from_start_y = polynomial_subtract(
            &y_numerator,
            &polynomial_scale(&denominator, line.start().y()),
        );
        let line_parameter_numerator = polynomial_add(
            &polynomial_scale(&from_start_x, &line_x),
            &polynomial_scale(&from_start_y, &line_y),
        );
        let line_length_squared = &line_x * &line_x + &line_y * &line_y;
        let line_parameter_denominator = polynomial_scale(&denominator, &line_length_squared);
        let mut parameter_map = RationalParameterImageMap2::new(
            line_parameter_numerator,
            line_parameter_denominator,
            policy,
        );
        let line_parameter = match if affine_line_parameter {
            parameter_map.image_unbounded(parameter)
        } else {
            parameter_map.image(parameter)
        }? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let image = rational_point_image_from_power_basis(
            algebraic_parameter,
            x_numerator.clone(),
            y_numerator.clone(),
            denominator.clone(),
            policy,
        )?;
        let image = match image {
            Classification::Decided(image) => image,
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Err(CurveError::Topology(
                    "a certified supporting-line contact acquired a zero denominator".into(),
                ));
            }
            Classification::Uncertain(_) => {
                // The selected line parameter already certifies this affine denominator.
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    algebraic_parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(
                        algebraic_parameter,
                        &policy.strict_counterpart(),
                    ),
                    x_numerator,
                    y_numerator,
                    denominator,
                    "retained an exact analytic-parallel supporting-line contact",
                )
            }
        };
        Ok(Classification::Decided((
            CurvePoint2::from(image),
            line_parameter,
        )))
    }

    pub(super) fn supporting_line_incidence_with_certified_contacts(
        &self,
        line: &LineSeg2,
        certified_direction: Option<(&Real, &Real)>,
        certified_crossings: &[Real],
        certified_tangencies: &[Real],
        deep_branch_refinement: bool,
        inferred_contact_kinds: Option<&mut Vec<Option<BezierLineContactKind>>>,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        retained_range: Option<&CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let source = self.source_power_basis()?;
        let unit = CurveParameterRange2::unit();
        let range = retained_range.unwrap_or(&unit);
        let domain = CurveParameterDomain2::new(range, None);
        if let Some(weight) = source.weight {
            match polynomial_is_nonzero_on_parameter_range(weight, range, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
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
        let weighted_start = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let source_from_start_x =
            polynomial_subtract(source.x_numerator, &weighted_start(line.start().x()));
        let source_from_start_y =
            polynomial_subtract(source.y_numerator, &weighted_start(line.start().y()));
        let (line_x, line_y) = certified_direction.map_or_else(
            || line.delta(),
            |(line_x, line_y)| (line_x.clone(), line_y.clone()),
        );
        let line_numerator = polynomial_subtract(
            &polynomial_scale(&source_from_start_y, &line_x),
            &polynomial_scale(&source_from_start_x, &line_y),
        );
        let line_polynomial = match polynomial_from_coefficients(line_numerator.clone(), policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if distance_sign == RealSign::Zero {
            return match line_polynomial {
                Some(polynomial) => Ok(domain
                    .finite_roots(&polynomial, policy)?
                    .map(BezierParallelIncidence2::Parameters)),
                None => Ok(Classification::Decided(
                    BezierParallelIncidence2::EntireCurve,
                )),
            };
        }
        let speed_squared = parallel_speed_squared_polynomial(differential);
        match polynomial_is_nonzero_on_parameter_range(&speed_squared, range, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let normal_projection = polynomial_add(
            &polynomial_scale(&differential.tangent_x, &line_x),
            &polynomial_scale(&differential.tangent_y, &line_y),
        );
        let signed_normal_term = polynomial_scale(&normal_projection, self.distance());
        let signed_normal_term = match source.weight {
            Some(weight) => polynomial_multiply(&signed_normal_term, weight),
            None => signed_normal_term,
        };
        let normal_polynomial =
            match polynomial_from_coefficients(signed_normal_term.clone(), policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (line_polynomial, normal_polynomial) = match (line_polynomial, normal_polynomial) {
            (None, None) => {
                return Ok(Classification::Decided(
                    BezierParallelIncidence2::EntireCurve,
                ));
            }
            (Some(polynomial), None) | (None, Some(polynomial)) => {
                return Ok(domain
                    .finite_roots(&polynomial, policy)?
                    .map(BezierParallelIncidence2::Parameters));
            }
            (Some(line_polynomial), Some(normal_polynomial)) => {
                (line_polynomial, normal_polynomial)
            }
        };
        let branch_product = polynomial_multiply(&line_numerator, &signed_normal_term);
        let mut squared_relation = polynomial_subtract(
            &polynomial_multiply(
                &polynomial_multiply(&line_numerator, &line_numerator),
                &speed_squared,
            ),
            &polynomial_multiply(&signed_normal_term, &signed_normal_term),
        );
        for parameter in certified_crossings {
            if squared_relation.len() < 2 {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            squared_relation = divide_by_linear_root(&squared_relation, parameter);
        }
        for parameter in certified_tangencies {
            for _ in 0..2 {
                if squared_relation.len() < 2 {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                squared_relation = divide_by_linear_root(&squared_relation, parameter);
            }
        }
        let squared_polynomial =
            match polynomial_from_coefficients(squared_relation.clone(), policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let retained_range_is_strictly_disjoint = match retained_range {
            Some(range) => {
                strict_polynomial_sign_on_curve_region_range(&squared_relation, range, policy)?
                    .is_some()
            }
            None => false,
        };
        let incidence = if retained_range_is_strictly_disjoint {
            BezierParallelIncidence2::Parameters(Vec::new())
        } else {
            match squared_polynomial.as_ref() {
                Some(polynomial) => match domain.finite_roots(polynomial, policy)? {
                    Classification::Decided(parameters) => {
                        BezierParallelIncidence2::Parameters(parameters)
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                None => BezierParallelIncidence2::EntireCurve,
            }
        };

        match incidence {
            BezierParallelIncidence2::EntireCurve => {
                let mut sample = match range.strict_interior_scalar(policy)? {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let line_sign = loop {
                    match real_sign(&line_polynomial.evaluate(&sample), policy) {
                        Some(sign @ (RealSign::Positive | RealSign::Negative)) => break sign,
                        Some(RealSign::Zero) => {
                            let [lower, _] = match range.ordered_endpoints(policy)? {
                                Classification::Decided(endpoints) => endpoints,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            sample = match lower
                                .strict_scalar_between_ordered(&sample.into(), policy)?
                            {
                                Classification::Decided(sample) => sample,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                        }
                        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                    }
                };
                let normal_sign = match real_sign(&normal_polynomial.evaluate(&sample), policy) {
                    Some(sign) => sign,
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
                if real_signs_are_opposite(line_sign, normal_sign) {
                    return Ok(Classification::Decided(
                        BezierParallelIncidence2::EntireCurve,
                    ));
                }
                Ok(domain
                    .finite_roots(&line_polynomial, policy)?
                    .map(BezierParallelIncidence2::Parameters))
            }
            BezierParallelIncidence2::Parameters(candidates) => {
                let mut retained = Vec::with_capacity(candidates.len());
                let mut retained_kinds = Vec::with_capacity(candidates.len());
                let inferred_kind = |candidate: &BezierParameter2| -> CurveResult<_> {
                    let Some(polynomial) = squared_polynomial.as_ref() else {
                        return Ok(None);
                    };
                    Ok(match polynomial.changes_sign_at_root(candidate, policy)? {
                        Classification::Decided(false) => Some(BezierLineContactKind::Tangent),
                        Classification::Decided(true) | Classification::Uncertain(_) => None,
                    })
                };
                for (candidate_index, candidate) in candidates.iter().cloned().enumerate() {
                    // The product polynomial is a compact fast path.  If its
                    // local-field GCD cannot certify the sign, consult the two
                    // retained branch factors below before permitting a
                    // terminal approximation.
                    let branch_sign = match policy.strict_predicate_pass(|| {
                        signed_coefficients_at_parameter(&branch_product, &candidate, policy)
                    })? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            let separated_branch_sign = match (
                                policy.strict_predicate_pass(|| {
                                    signed_coefficients_at_parameter(
                                        line_polynomial.coefficients(),
                                        &candidate,
                                        policy,
                                    )
                                })?,
                                policy.strict_predicate_pass(|| {
                                    signed_coefficients_at_parameter(
                                        normal_polynomial.coefficients(),
                                        &candidate,
                                        policy,
                                    )
                                })?,
                            ) {
                                (
                                    Classification::Decided(line_sign),
                                    Classification::Decided(normal_sign),
                                ) => Some(product_sign(line_sign, normal_sign)),
                                _ => None,
                            };
                            if let Some(sign) = separated_branch_sign {
                                sign
                            } else {
                                let odd_root = match squared_polynomial.as_ref() {
                                    Some(polynomial) => {
                                        match policy.strict_predicate_pass(|| {
                                            polynomial.changes_sign_at_root(&candidate, policy)
                                        })? {
                                            Classification::Decided(changes) => changes,
                                            Classification::Uncertain(_) => false,
                                        }
                                    }
                                    None => false,
                                };
                                if odd_root {
                                    let before = match parallel_line_neighbor_sign(
                                        self,
                                        line,
                                        certified_direction,
                                        &candidates,
                                        candidate_index,
                                        false,
                                        tangent_field,
                                        retained_range,
                                        policy,
                                    )? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    let after = match parallel_line_neighbor_sign(
                                        self,
                                        line,
                                        certified_direction,
                                        &candidates,
                                        candidate_index,
                                        true,
                                        tangent_field,
                                        retained_range,
                                        policy,
                                    )? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    };
                                    if before != after {
                                        RealSign::Negative
                                    } else {
                                        RealSign::Positive
                                    }
                                } else if deep_branch_refinement
                                    && let Some(sign) = deep_exact_coefficients_sign_at_parameter(
                                        &branch_product,
                                        &candidate,
                                        policy,
                                    )?
                                {
                                    sign
                                } else {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                    };
                    match branch_sign {
                        RealSign::Negative => {
                            retained_kinds.push(inferred_kind(&candidate)?);
                            retained.push(candidate);
                            continue;
                        }
                        RealSign::Positive => continue,
                        RealSign::Zero => {}
                    }
                    let line_sign = match signed_coefficients_at_parameter(
                        line_polynomial.coefficients(),
                        &candidate,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let normal_sign = match signed_coefficients_at_parameter(
                        normal_polynomial.coefficients(),
                        &candidate,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    match (line_sign, normal_sign) {
                        (RealSign::Zero, RealSign::Zero) => {
                            retained_kinds.push(None);
                            retained.push(candidate);
                        }
                        (RealSign::Positive, RealSign::Negative)
                        | (RealSign::Negative, RealSign::Positive) => {
                            retained_kinds.push(inferred_kind(&candidate)?);
                            retained.push(candidate);
                        }
                        (RealSign::Positive, RealSign::Positive)
                        | (RealSign::Negative, RealSign::Negative) => {}
                        (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                        | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "squared parallel-line candidate lost its exact mate".to_owned(),
                            ));
                        }
                    }
                }
                for (parameter, kind) in certified_crossings
                    .iter()
                    .map(|parameter| (parameter, BezierLineContactKind::Crossing))
                    .chain(
                        certified_tangencies
                            .iter()
                            .map(|parameter| (parameter, BezierLineContactKind::Tangent)),
                    )
                {
                    let candidate = BezierParameter2::Exact(parameter.clone());
                    if let Some(range) = retained_range {
                        match bezier_parameter_is_in_curve_region_range(
                            &candidate, range, true, policy,
                        )? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    let mut insert_at = retained.len();
                    for (index, existing) in retained.iter_mut().enumerate() {
                        let ordering = candidate.cmp_by_refinement(existing, policy)?;
                        match ordering {
                            Classification::Decided(std::cmp::Ordering::Less) => {
                                insert_at = index;
                                break;
                            }
                            Classification::Decided(std::cmp::Ordering::Equal) => {
                                // Preserve the exact construction witness and its
                                // crossing direction when a higher-order residual
                                // factor isolates the same parameter again.
                                *existing = candidate.clone();
                                retained_kinds[index] = Some(kind);
                                insert_at = usize::MAX;
                                break;
                            }
                            Classification::Decided(std::cmp::Ordering::Greater) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    if insert_at != usize::MAX {
                        retained.insert(insert_at, candidate);
                        retained_kinds.insert(insert_at, Some(kind));
                    }
                }
                if let Some(inferred_contact_kinds) = inferred_contact_kinds {
                    *inferred_contact_kinds = retained_kinds;
                }
                Ok(Classification::Decided(
                    BezierParallelIncidence2::Parameters(retained),
                ))
            }
        }
    }

    /// Uses a caller-certified direction plus one exact transverse contact
    /// and any exact tangencies on the same supporting line.  The certified
    /// crossing is divided from the squared incidence before isolation and
    /// restored with its known side transition, so an authored endpoint does
    /// not require a second radical branch or neighbor-sign proof.
    pub(crate) fn relation_to_supporting_line_with_direction_and_certified_contacts(
        &self,
        line: &LineSeg2,
        direction_x: &Real,
        direction_y: &Real,
        certified_crossing: Option<(&Real, BezierLineCrossingDirection)>,
        certified_tangencies: &[Real],
        deep_branch_refinement: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineContactRelation>> {
        self.relation_to_supporting_line_with_certified_contacts(
            line,
            Some((direction_x, direction_y)),
            certified_crossing,
            certified_tangencies,
            deep_branch_refinement,
            None,
            None,
            None,
            policy,
        )
    }

    pub(crate) fn relation_to_supporting_line_on_regular_range(
        &self,
        line: &LineSeg2,
        range: &CurveParameterRange2,
        certified_crossing: Option<(&Real, BezierLineCrossingDirection)>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineContactRelation>> {
        self.relation_to_supporting_line_on_regular_range_with_certified_contacts(
            line,
            range,
            certified_crossing,
            &[],
            false,
            policy,
        )
    }

    pub(crate) fn relation_to_supporting_line_on_regular_range_with_certified_contacts(
        &self,
        line: &LineSeg2,
        range: &CurveParameterRange2,
        certified_crossing: Option<(&Real, BezierLineCrossingDirection)>,
        certified_tangencies: &[Real],
        deep_branch_refinement: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineContactRelation>> {
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_field =
            match self.source_oriented_regularized_tangent_field_at_interior(&interior, &strict)? {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let derivative_scale_sign = if tangent_field.is_some() {
            Some(
                match self.parallel_derivative_scale_sign_at_exact(&interior, &strict)? {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            )
        } else {
            None
        };
        self.relation_to_supporting_line_with_certified_contacts(
            line,
            None,
            certified_crossing,
            certified_tangencies,
            deep_branch_refinement,
            tangent_field.as_deref(),
            derivative_scale_sign,
            Some(range),
            policy,
        )
    }

    pub(super) fn relation_to_supporting_line_with_certified_contacts(
        &self,
        line: &LineSeg2,
        certified_direction: Option<(&Real, &Real)>,
        certified_crossing: Option<(&Real, BezierLineCrossingDirection)>,
        certified_tangencies: &[Real],
        deep_branch_refinement: bool,
        tangent_field: Option<&BezierAnalyticParallelTangentField2>,
        derivative_scale_sign: Option<RealSign>,
        retained_range: Option<&CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineContactRelation>> {
        let certified_parameters = certified_crossing
            .iter()
            .map(|(parameter, _)| (*parameter).clone())
            .collect::<Vec<_>>();
        let mut inferred_contact_kinds = Vec::new();
        let parameters = match self.supporting_line_incidence_with_certified_contacts(
            line,
            certified_direction,
            &certified_parameters,
            certified_tangencies,
            deep_branch_refinement,
            Some(&mut inferred_contact_kinds),
            tangent_field,
            retained_range,
            policy,
        )? {
            Classification::Decided(BezierParallelIncidence2::EntireCurve) => {
                return Ok(Classification::Decided(
                    BezierLineContactRelation::OnSupportingLine,
                ));
            }
            Classification::Decided(BezierParallelIncidence2::Parameters(parameters)) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if parameters.is_empty() {
            return Ok(Classification::Decided(
                BezierLineContactRelation::NoContact,
            ));
        }

        let (tangent_direction_x, tangent_direction_y) = certified_direction.map_or_else(
            || line.delta(),
            |(direction_x, direction_y)| (direction_x.clone(), direction_y.clone()),
        );
        let mut contacts = Vec::with_capacity(parameters.len());
        for (index, parameter) in parameters.iter().enumerate() {
            if let Some((certified_parameter, direction)) = certified_crossing
                && parameter.scalar() == Some(certified_parameter)
            {
                contacts.push(BezierLineContact::with_crossing_direction(
                    parameter.clone(),
                    BezierLineContactKind::Crossing,
                    Some(direction),
                )?);
                continue;
            }
            let certified_tangent = parameter
                .scalar()
                .is_some_and(|parameter| certified_tangencies.contains(parameter));
            let inferred_tangent =
                inferred_contact_kinds.get(index) == Some(&Some(BezierLineContactKind::Tangent));
            // A regular first-order contact owns a smaller and stronger
            // classification than two neighboring radical evaluations.  In
            // particular, analytic continuation across a rational endpoint
            // can retain the correct squared-incidence root while selecting
            // the wrong radical-side germ outside the authored domain.  The
            // exact oriented-line derivative is `direction x tangent`, so a
            // nonzero sign proves both crossing kind and direction without
            // consulting that exterior germ.
            let tangent_is_parallel = match self
                .vector_tangent_cross_and_dot_signs_with_tangent_field(
                    &parameter.clone().into(),
                    &tangent_direction_x,
                    &tangent_direction_y,
                    tangent_field,
                    derivative_scale_sign,
                    policy,
                )? {
                Classification::Decided((cross, dot)) => {
                    if cross != RealSign::Zero {
                        if certified_tangent || inferred_tangent {
                            return Err(CurveError::Topology(
                                "certified supporting-line tangency has a transverse tangent"
                                    .to_owned(),
                            ));
                        }
                        contacts.push(BezierLineContact::with_crossing_direction(
                            parameter.clone(),
                            BezierLineContactKind::Crossing,
                            Some(match cross {
                                RealSign::Positive => {
                                    BezierLineCrossingDirection::NegativeToPositive
                                }
                                RealSign::Negative => {
                                    BezierLineCrossingDirection::PositiveToNegative
                                }
                                RealSign::Zero => unreachable!("the contact is transverse"),
                            }),
                        )?);
                        continue;
                    }
                    if dot == RealSign::Zero {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    true
                }
                Classification::Uncertain(_) => false,
            };
            let retained_endpoint_interior_after = if tangent_is_parallel
                && let Some((start, end)) =
                    retained_range.and_then(CurveParameterRange2::as_bezier_parameters)
            {
                let at_start = match parameter.cmp_by_refinement(start, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let at_end = match parameter.cmp_by_refinement(end, policy)? {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match (at_start, at_end) {
                    (std::cmp::Ordering::Equal, _) => Some(true),
                    (_, std::cmp::Ordering::Equal) => Some(false),
                    _ => None,
                }
            } else {
                None
            };
            if let Some(after) = retained_endpoint_interior_after {
                let side = match parallel_line_neighbor_sign(
                    self,
                    line,
                    certified_direction,
                    &parameters,
                    index,
                    after,
                    tangent_field,
                    retained_range,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => crate::classify::LineSide::Left,
                    Classification::Decided(RealSign::Negative) => crate::classify::LineSide::Right,
                    Classification::Decided(RealSign::Zero) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                contacts.push(BezierLineContact::with_tangent_side(
                    parameter.clone(),
                    side,
                )?);
                continue;
            }
            let before = match parallel_line_neighbor_sign(
                self,
                line,
                certified_direction,
                &parameters,
                index,
                false,
                tangent_field,
                retained_range,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let after = match parallel_line_neighbor_sign(
                self,
                line,
                certified_direction,
                &parameters,
                index,
                true,
                tangent_field,
                retained_range,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if before == after {
                let side = match before {
                    RealSign::Positive => crate::classify::LineSide::Left,
                    RealSign::Negative => crate::classify::LineSide::Right,
                    RealSign::Zero => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                };
                contacts.push(BezierLineContact::with_tangent_side(
                    parameter.clone(),
                    side,
                )?);
            } else {
                if certified_tangent || inferred_tangent {
                    return Err(CurveError::Topology(
                        "certified supporting-line tangency changed strict sides".to_owned(),
                    ));
                }
                let direction = if after == RealSign::Positive {
                    BezierLineCrossingDirection::NegativeToPositive
                } else {
                    BezierLineCrossingDirection::PositiveToNegative
                };
                contacts.push(BezierLineContact::with_crossing_direction(
                    parameter.clone(),
                    BezierLineContactKind::Crossing,
                    Some(direction),
                )?);
            }
        }
        Ok(Classification::Decided(
            BezierLineContactRelation::Contacts { contacts },
        ))
    }

    pub(crate) fn supporting_line_parameter_order(
        &self,
        parameter: &BezierParameter2,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        Ok(signed_parallel_linear_projection_at_parameter(
            self, parameter, line, None, false, None, policy,
        )?
        .map(|sign| match sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }

    pub(crate) fn supporting_line_parameter_order_on_regular_range(
        &self,
        parameter: &BezierParameter2,
        line: &LineSeg2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_field =
            match self.source_oriented_regularized_tangent_field_at_interior(&interior, &strict)? {
                Classification::Decided(field) => field,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let Some(tangent_field) = tangent_field else {
            return self.supporting_line_parameter_order(parameter, line, policy);
        };
        Ok(signed_parallel_linear_projection_at_parameter(
            self,
            parameter,
            line,
            None,
            false,
            Some(&tangent_field),
            policy,
        )?
        .map(|sign| match sign {
            RealSign::Negative => std::cmp::Ordering::Less,
            RealSign::Zero => std::cmp::Ordering::Equal,
            RealSign::Positive => std::cmp::Ordering::Greater,
        }))
    }

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
    pub(super) fn certifies_simple_closed_parallel(
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

    pub(super) fn parallel_intersections_without_regular_frame(
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
    pub(super) fn replay_regular_pair_endpoint_tangents(
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
    pub(super) fn parallel_intersections_from_system_in_domain(
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
    pub(super) fn zero_distance_pair_intersections_in_domain(
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
    pub(super) fn rational_parallel_pair_intersections(
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

    pub(super) fn rational_parallel_pair_intersections_in_regular_domains(
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
    pub(super) fn rational_unit_parallel_pair_intersections(
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
                        first.fit_exact_line_image(policy)?
                    && let Classification::Decided(BezierLineImageFitRelation::Fit(second)) =
                        other.fit_exact_line_image(policy)?
                    && matches!(
                        first.line().intersect_line(second.line(), policy)?,
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

    pub(super) fn replay_parallel_pair_projection(
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

    pub(super) fn replay_parallel_pair_projection_with_ranges(
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

    pub(super) fn certified_transverse_parallel_contact(
        &self,
        other: &Self,
        first_parameter: &BezierParameter2,
        second_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> bool {
        let (Some(first_parameter), Some(second_parameter)) =
            (first_parameter.scalar(), second_parameter.scalar())
        else {
            return false;
        };
        let Ok(Classification::Decided(first_derivative)) =
            self.derivative_at(first_parameter, policy)
        else {
            return false;
        };
        let Ok(Classification::Decided(second_derivative)) =
            other.derivative_at(second_parameter, policy)
        else {
            return false;
        };
        !matches!(
            real_sign(
                &(first_derivative.dx() * second_derivative.dy()
                    - first_derivative.dy() * second_derivative.dx()),
                policy,
            ),
            Some(RealSign::Zero) | None
        )
    }

    /// Returns the parallel/source derivative orientation at this exact
    /// contact, reusing its scalar, selected-fiber, or projective authority.
    /// A fillet support can cross cusps between contacts on one source range.
    pub(crate) fn parallel_derivative_scale_sign(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if self
            .data
            .source
            .source
            .has_exact_affine_line_parameterization()
        {
            return Ok(Classification::Decided(RealSign::Positive));
        }
        if let Some(parameter) = parameter
            .as_bezier_parameter()
            .and_then(BezierParameter2::scalar)
        {
            return self.parallel_derivative_scale_sign_at_exact(parameter, policy);
        }
        self.parallel_derivative_scale_sign_from_polynomials(parameter, policy)
    }

    /// Replays orientation at the contact, using the owned one-sided limit
    /// only when the derivative vanishes at a retained range endpoint.
    ///
    /// A regular source frame can span cusps of its parallel. Its interior
    /// sample therefore cannot certify the parallel's orientation at every
    /// contact. At a stationary endpoint the first nonzero Taylor coefficient
    /// of each curvature predicate supplies the exact local sign; no new
    /// scalar image or root isolation is needed.
    pub(crate) fn parallel_derivative_scale_sign_on_regular_range(
        &self,
        parameter: &CurveParameter2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let scale = self.parallel_derivative_scale_sign(parameter, policy)?;
        if scale != Classification::Decided(RealSign::Zero) {
            return Ok(scale);
        }
        let strict = policy.strict_counterpart();
        let [lower, upper] = match range.ordered_endpoints(&strict)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let increasing = match parameter.cmp_by_refinement(lower, &strict)? {
            Classification::Decided(std::cmp::Ordering::Equal) => true,
            Classification::Decided(_) => match parameter.cmp_by_refinement(upper, &strict)? {
                Classification::Decided(std::cmp::Ordering::Equal) => false,
                Classification::Decided(_) => return Ok(scale),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            },
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.parallel_derivative_scale_sign_from_side(
            parameter,
            if increasing {
                BezierParameterRayDirection2::Increasing
            } else {
                BezierParameterRayDirection2::Decreasing
            },
            &strict,
        )
    }

    /// Uses the surviving contact side when an original offset has a cusp.
    /// The source normal must be defined at the contact; a pole or stationary
    /// source cannot acquire a tangent from an unrelated incident range.
    pub(crate) fn parallel_derivative_scale_sign_at_side(
        &self,
        parameter: &CurveParameter2,
        side: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let scale = self.parallel_derivative_scale_sign(parameter, policy)?;
        if scale != Classification::Decided(RealSign::Zero) {
            return Ok(scale);
        }
        let strict = policy.strict_counterpart();
        match self.source_tangent_nonzero_at(parameter, &strict)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if let Some(weight) = self.source_power_basis()?.weight {
            match parameter.polynomial_sign(weight, &strict)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        self.parallel_derivative_scale_sign_from_side(parameter, side, &strict)
    }

    pub(super) fn parallel_derivative_scale_sign_from_side(
        &self,
        parameter: &CurveParameter2,
        side: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let side_sign = |coefficients: &[Real]| {
            parameter_polynomial_side_sign(coefficients, parameter, side, policy)
        };
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let curvature =
            parallel_signed_curvature_polynomial(differential, source.weight, self.distance());
        parallel_derivative_scale_from_curvature_sign(side_sign(&curvature)?, || {
            let speed = parallel_speed_squared_polynomial(differential);
            side_sign(&polynomial_subtract(
                &polynomial_multiply(&curvature, &curvature),
                &polynomial_power(&speed, 3),
            ))
        })
    }

    /// On a connected pole-free, regular, cusp-free range, the continuous
    /// scalar multiplying the source tangent is nonzero and has one sign.
    /// A sufficient proof on an outward envelope avoids adjoining contact
    /// parameters merely to rediscover this common derivative orientation.
    pub(super) fn certified_derivative_scale_sign_on_range(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        policy.bounded_exact_predicate_pass(|| {
            let domain = CurveParameterDomain2::new(range, None);
            let Ok(Classification::Decided((_, [lower, upper]))) = domain.finite_envelope(policy)
            else {
                return Ok(None);
            };
            let envelope =
                CurveParameterRange2::new_validated(lower.clone().into(), upper.clone().into());
            let Ok(Classification::Decided(analysis)) =
                self.singularity_analysis(&envelope, policy)
            else {
                return Ok(None);
            };
            if !analysis.source_is_regular() || !analysis.parallel_is_cusp_free() {
                return Ok(None);
            }
            let sample = ((lower + upper) / Real::from(2_i8))?;
            let sign = match self.parallel_derivative_scale_sign_at_exact(&sample, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => {
                    return Ok(None);
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "parallel-derivative-orientation",
                "certified-regular-range",
            );
            Ok(Some(sign))
        })
    }

    pub(crate) fn has_exact_affine_line_parameterization(&self) -> bool {
        self.data
            .source
            .source
            .has_exact_affine_line_parameterization()
    }

    /// Certifies nonnegative tangent turning over the consumed finite range.
    /// Reversing traversal reverses the curvature sign.
    ///
    /// For the homogeneous tangent numerator `v`, both rational division and
    /// a regular parallel multiply `cross(v, v')` by a positive factor. A
    /// Bernstein hull therefore gives a cheap sufficient certificate without
    /// constructing unit normals or isolating curvature roots. Source
    /// regularity is essential: a cusp can reverse the tangent even when this
    /// polynomial never changes sign. An inconclusive hull is not a rejection
    /// of the curve; the caller must use its general arrangement path.
    pub(crate) fn certifies_nonnegative_turn(
        &self,
        range: &CurveParameterRange2,
        reversed: bool,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        policy.bounded_exact_predicate_pass(|| {
            let unit = CurveParameterRange2::unit();
            let native = matches!(
                CurveParameterDomain2::new(&unit, None).contains_finite_range(range, policy),
                Ok(Classification::Decided(true))
            );
            // Preserve the cheap whole-unit proof when it covers this range.
            // Otherwise an outward finite envelope is only a sufficient sign
            // certificate; it never replaces the retained endpoint authority.
            let envelope = if native { &unit } else { range };
            let (_, [lower, upper]) =
                match CurveParameterDomain2::new(envelope, None).finite_envelope(policy) {
                    Ok(Classification::Decided(envelope)) => envelope,
                    Ok(Classification::Uncertain(_)) | Err(_) => return Ok(false),
                };
            let differential = self.differential()?;
            let curvature = polynomial_trim_structural_zeros(polynomial_subtract(
                &polynomial_multiply(&differential.tangent_x, &differential.tangent_derivative_y),
                &polynomial_multiply(&differential.tangent_y, &differential.tangent_derivative_x),
            ));
            let curvature = if native {
                curvature
            } else {
                restrict_power_basis_to_interval(&curvature, lower, upper)
            };
            let positive_turn = if reversed {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            for control in
                power_to_bernstein_coefficients(&curvature, curvature.len().saturating_sub(1))?
            {
                match real_sign(&control, policy) {
                    Some(RealSign::Zero) => {}
                    Some(sign) if sign == positive_turn => {}
                    _ => return Ok(false),
                }
            }
            for component in [&differential.tangent_x, &differential.tangent_y] {
                let restricted =
                    (!native).then(|| restrict_power_basis_to_interval(component, lower, upper));
                if univariate_unit_interval_strict_bernstein_sign(
                    restricted.as_deref().unwrap_or(component),
                    policy,
                )?
                .is_some()
                {
                    return Ok(true);
                }
            }
            let speed_squared = parallel_speed_squared_polynomial(differential);
            let speed_squared = if native {
                speed_squared
            } else {
                restrict_power_basis_to_interval(&speed_squared, lower, upper)
            };
            Ok(
                univariate_unit_interval_strict_bernstein_sign(&speed_squared, policy)?
                    == Some(RealSign::Positive),
            )
        })
    }

    /// Returns which strict side of its oriented tangent contains the local
    /// analytic-parallel branch.
    ///
    /// Away from a source singularity and a parallel cusp, the parallel
    /// derivative is a nonzero scalar multiple of the source tangent. Its
    /// signed curvature is therefore the source tangent cross its derivative,
    /// multiplied by a positive square. This predicate needs no unit normal or
    /// selected Cartesian point.
    pub(crate) fn tangent_side_at(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        match self.parallel_derivative_scale_sign(&parameter.clone().into(), policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let differential = self.differential()?;
        let signed_curvature = polynomial_subtract(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_derivative_y),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_derivative_x),
        );
        Ok(
            match signed_coefficients_at_parameter(&signed_curvature, parameter, policy)? {
                Classification::Decided(RealSign::Positive) => {
                    Classification::Decided(crate::classify::LineSide::Left)
                }
                Classification::Decided(RealSign::Negative) => {
                    Classification::Decided(crate::classify::LineSide::Right)
                }
                Classification::Decided(RealSign::Zero) => {
                    Classification::Decided(crate::classify::LineSide::On)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(super) fn parallel_derivative_scale_sign_from_polynomials(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(RealSign::Positive));
        }
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let speed_squared = parallel_speed_squared_polynomial(differential);
        let signed_curvature =
            parallel_signed_curvature_polynomial(differential, source.weight, self.distance());
        match parameter.polynomial_sign(&speed_squared, policy)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(RealSign::Zero));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "parallel source speed squared was certified negative".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        parallel_derivative_scale_from_curvature_sign(
            parameter.polynomial_sign(&signed_curvature, policy)?,
            || {
                let squared_difference = polynomial_subtract(
                    &polynomial_multiply(&signed_curvature, &signed_curvature),
                    &polynomial_power(&speed_squared, 3),
                );
                parameter.polynomial_sign(&squared_difference, policy)
            },
        )
    }

    pub(crate) fn derivative_scale_constraint(
        &self,
        axis: CurveResultantParameter,
        expected: RealSign,
        side: Option<BezierParameterRayDirection2>,
    ) -> BezierParallelDerivativeConstraint2 {
        debug_assert_ne!(expected, RealSign::Zero);
        BezierParallelDerivativeConstraint2 {
            parallel: self.clone(),
            axis,
            expected,
            side,
            polynomials: OnceLock::new(),
        }
    }

    pub(super) fn derivative_scale_polynomials(
        &self,
        axis: CurveResultantParameter,
        expected: RealSign,
        side: Option<BezierParameterRayDirection2>,
    ) -> CurveResult<BezierParallelDerivativePolynomials2> {
        debug_assert_ne!(expected, RealSign::Zero);
        // The identity offset has scale +1 even at a stationary source
        // parameter, matching the scalar derivative predicate.
        if self.distance().zero_status() == hyperreal::ZeroKnowledge::Zero {
            return Ok(BezierParallelDerivativePolynomials2 {
                offset_distance: self.distance().clone(),
                speed_squared: BivariatePolynomial::new(vec![vec![Real::one()]]),
                signed_curvature: BivariatePolynomial::new(vec![vec![Real::zero()]]),
                cusp_norm: BivariatePolynomial::new(vec![vec![-Real::one()]]),
                expected,
                axis,
                side,
            });
        }
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let speed = parallel_speed_squared_polynomial(differential);
        let curvature =
            parallel_signed_curvature_polynomial(differential, source.weight, self.distance());
        let norm = polynomial_subtract(
            &polynomial_multiply(&curvature, &curvature),
            &polynomial_power(&speed, 3),
        );
        let lift = |coefficients: Vec<Real>| match axis {
            CurveResultantParameter::First => BivariatePolynomial::new(
                coefficients
                    .into_iter()
                    .map(|coefficient| vec![coefficient])
                    .collect(),
            ),
            CurveResultantParameter::Second => BivariatePolynomial::new(vec![coefficients]),
        };
        Ok(BezierParallelDerivativePolynomials2 {
            offset_distance: self.distance().clone(),
            speed_squared: lift(speed),
            signed_curvature: lift(curvature),
            cusp_norm: lift(norm),
            axis,
            expected,
            side,
        })
    }

    #[cfg(test)]
    pub(super) fn algebraic_parallel_cusp_frame(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParallelAlgebraicCuspFrame2>>> {
        let retained_parameter = BezierParameter2::Algebraic(parameter.clone());
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let speed_squared = parallel_speed_squared_polynomial(differential);
        match signed_coefficients_at_parameter(&speed_squared, &retained_parameter, policy)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "parallel cusp speed squared was certified negative".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let signed_curvature =
            parallel_signed_curvature_polynomial(differential, source.weight, self.distance());
        match signed_coefficients_at_parameter(&signed_curvature, &retained_parameter, policy)? {
            Classification::Decided(RealSign::Negative) => {}
            Classification::Decided(RealSign::Positive | RealSign::Zero) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let cusp_residual = polynomial_subtract(
            &polynomial_multiply(&signed_curvature, &signed_curvature),
            &polynomial_power(&speed_squared, 3),
        );
        match signed_coefficients_at_parameter(&cusp_residual, &retained_parameter, policy)? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let mut normal_x_numerator = polynomial_multiply(&differential.tangent_y, &speed_squared);
        let mut normal_y_numerator = polynomial_scale(
            &polynomial_multiply(&differential.tangent_x, &speed_squared),
            &Real::from(-1_i8),
        );
        let denominator = if let Some(weight) = source.weight {
            normal_x_numerator = polynomial_multiply(&normal_x_numerator, weight);
            normal_y_numerator = polynomial_multiply(&normal_y_numerator, weight);
            polynomial_multiply(weight, &signed_curvature)
        } else {
            signed_curvature.clone()
        };
        Ok(Classification::Decided(Some(
            BezierParallelAlgebraicCuspFrame2 {
                data: Arc::new(BezierParallelAlgebraicCuspFrameData2 {
                    parallel: Some(self.clone()),
                    cardinal_normal: None,
                    represented_unit_normal: None,
                    direct_center: None,
                    parameter: parameter.clone(),
                    source_x_numerator: polynomial_multiply(source.x_numerator, &signed_curvature),
                    source_y_numerator: polynomial_multiply(source.y_numerator, &signed_curvature),
                    normal_x_numerator,
                    normal_y_numerator,
                    denominator,
                }),
            },
        )))
    }

    /// Constructs an exact point on any source parallel at one selected
    /// algebraic cusp of this parallel.
    ///
    /// With homogeneous tangent numerator `H`, `S=H dot H`, and selected
    /// signed curvature term `Q=d W^2(H'_x H_y-H'_y H_x)`, a regular cusp
    /// satisfies `Q<0` and `Q^2=S^3`. Therefore the source left normal is the
    /// rational expression `(H_y S/Q, -H_x S/Q)`: no nested square root or
    /// sampled cusp coordinate is needed. `image_distance` selects which
    /// parallel of the shared source is evaluated at that cusp parameter.
    ///
    /// `None` certifies that the supplied algebraic parameter is not a regular
    /// cusp on the selected signed-curvature branch. Predicate uncertainty is
    /// preserved separately in [`Classification`].
    #[cfg(test)]
    pub(crate) fn algebraic_parallel_cusp_point_image(
        &self,
        parameter: &BezierAlgebraicParameter2,
        image_distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezierAlgebraicPointImage2>>> {
        Ok(
            match self.algebraic_parallel_cusp_frame(parameter, policy)? {
                Classification::Decided(Some(frame)) => Classification::Decided(Some(
                    frame.point_image_at_parallel_distance(image_distance, policy)?,
                )),
                Classification::Decided(None) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    #[cfg(test)]
    pub(crate) fn algebraic_cusp_semicircle(
        &self,
        parameter: &BezierAlgebraicParameter2,
        radial_distance: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircle2>>> {
        match real_sign(&radial_distance, policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        Ok(
            match self.algebraic_parallel_cusp_frame(parameter, policy)? {
                Classification::Decided(Some(frame)) => {
                    Classification::Decided(Some(BezierAlgebraicCuspSemicircle2 {
                        data: Arc::new(BezierAlgebraicCuspSemicircleData2 {
                            parallel_system_cache: Mutex::default(),
                            frame: BezierSelectedCircleFrame2::Rational(frame),
                            radial_distance,
                            clockwise,
                        }),
                    }))
                }
                Classification::Decided(None) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Evaluates the parallel/source derivative scale without constructing the
    /// normalized derivative or its square root.
    ///
    /// For homogeneous tangent numerator `H`, the scale can change sign only
    /// on the selected negative-curvature branch where
    /// `(d W^2 (H' x_rev H))^2 - |H|^6` changes sign.  This is the same exact
    /// polynomial certificate used for algebraic parameters, evaluated
    /// directly at a represented scalar with bounded temporary storage.
    pub(super) fn parallel_derivative_scale_sign_at_exact(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(RealSign::Positive));
        }
        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let tangent_x = Real::eval_poly(&differential.tangent_x, parameter);
        let tangent_y = Real::eval_poly(&differential.tangent_y, parameter);
        let tangent_derivative_x = Real::eval_poly(&differential.tangent_derivative_x, parameter);
        let tangent_derivative_y = Real::eval_poly(&differential.tangent_derivative_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => return Ok(Classification::Decided(RealSign::Zero)),
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "parallel source speed squared was certified negative".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let curvature_cross =
            &tangent_derivative_x * &tangent_y - &tangent_derivative_y * &tangent_x;
        let mut signed_curvature = curvature_cross * self.distance();
        if let Some(weight) = source.weight {
            let weight = Real::eval_poly(weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            signed_curvature = signed_curvature * &weight * weight;
        }
        match real_sign(&signed_curvature, policy) {
            Some(RealSign::Positive | RealSign::Zero) => {
                Ok(Classification::Decided(RealSign::Positive))
            }
            Some(RealSign::Negative) => {
                let squared_difference = &signed_curvature * &signed_curvature
                    - &speed_squared * &speed_squared * &speed_squared;
                Ok(match real_sign(&squared_difference, policy) {
                    Some(RealSign::Positive) => Classification::Decided(RealSign::Negative),
                    Some(RealSign::Negative) => Classification::Decided(RealSign::Positive),
                    Some(RealSign::Zero) => Classification::Decided(RealSign::Zero),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                })
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    pub(super) fn apply_parallel_derivative_scale_to_tangent_sign(
        &self,
        source_sign: Classification<RealSign>,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        self.apply_parallel_derivative_scale_to_tangent_sign_with_override(
            source_sign,
            parameter,
            None,
            policy,
        )
    }

    pub(super) fn apply_parallel_derivative_scale_to_tangent_sign_with_override(
        &self,
        source_sign: Classification<RealSign>,
        parameter: &BezierParameter2,
        scale_sign: Option<RealSign>,
        policy: &CurveContext,
    ) -> CurveResult<Option<RealSign>> {
        let source = match source_sign {
            Classification::Decided(RealSign::Zero) => return Ok(Some(RealSign::Zero)),
            Classification::Decided(source @ (RealSign::Positive | RealSign::Negative)) => source,
            Classification::Uncertain(_) => return Ok(None),
        };
        if let Some(scale @ (RealSign::Positive | RealSign::Negative)) = scale_sign {
            return Ok(Some(product_sign(source, scale)));
        }
        if scale_sign == Some(RealSign::Zero) {
            return Ok(None);
        }
        Ok(
            match self.parallel_derivative_scale_sign(&parameter.clone().into(), policy)? {
                Classification::Decided(scale @ (RealSign::Positive | RealSign::Negative)) => {
                    Some(product_sign(source, scale))
                }
                Classification::Decided(RealSign::Zero) | Classification::Uncertain(_) => None,
            },
        )
    }

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
    pub(super) fn intersection_candidate_system(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
        self.intersection_candidate_system_with_tangent_field(other, None, policy)
    }

    pub(super) fn intersection_candidate_system_with_tangent_field(
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
    pub(super) fn rational_quadratic_circle_intersections_fast_path_with_tangent_field(
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
            support.rational_bezier_decomposition_with_policy(policy)
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

    pub(super) fn intersections_with_tangent_field(
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
    pub(super) fn replay_parallel_rational_candidate_system(
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

    pub(super) fn replay_degenerate_component(
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
    pub(super) fn replay_constant_parameter_components(
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
        let parallel_point = match self.point_at(&interior, policy)? {
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
    pub(super) fn rational_parallel_components_in_domains<const N: usize>(
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
            let offset = match self.exact_pythagorean_hodograph_offset(policy)? {
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
                                .exact_pythagorean_hodograph_offset(policy)?
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
                    match self.exact_pythagorean_hodograph_offset(policy)? {
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

    pub(super) fn certified_transverse_contact_sign(
        &self,
        other: &RationalBezier2,
        parallel_parameter: &BezierParameter2,
        other_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> Option<RealSign> {
        let (Some(parallel_parameter), Some(other_parameter)) =
            (parallel_parameter.scalar(), other_parameter.scalar())
        else {
            return None;
        };
        let Ok(Classification::Decided(parallel_derivative)) =
            self.derivative_at(parallel_parameter, policy)
        else {
            return None;
        };
        let Classification::Decided(other_derivative) =
            other.derivative_at_affine_classified(other_parameter, policy)
        else {
            return None;
        };
        match real_sign(
            &(parallel_derivative.dx() * other_derivative.dy()
                - parallel_derivative.dy() * other_derivative.dx()),
            policy,
        ) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => Some(sign),
            Some(RealSign::Zero) | None => None,
        }
    }

    pub(super) fn polynomial_power_basis(&self) -> CurveResult<&(Vec<Real>, Vec<Real>)> {
        if let Some(power_basis) = self.data.source.polynomial_power_basis.get() {
            return Ok(power_basis);
        }
        let power_basis = match self.source() {
            BezierParallelSource2::Quadratic(source) => {
                // Preserve the certified affine degree. Re-expanding shifted
                // Bernstein controls can hide zero coefficients and inflate
                // the field used to replay a selected source parameter.
                if let Some(line) = source.retained_exact_line_image() {
                    (
                        vec![line.start().x().clone(), line.end().x() - line.start().x()],
                        vec![line.start().y().clone(), line.end().y() - line.start().y()],
                    )
                } else {
                    polynomial_control_power_basis(&source.control_points())?
                }
            }
            BezierParallelSource2::Cubic(source) => {
                polynomial_control_power_basis(&source.control_points())?
            }
            BezierParallelSource2::Rational(_) => {
                return Err(CurveError::Topology(
                    "rational parallel requested a polynomial source basis".to_owned(),
                ));
            }
        };
        let _ = self.data.source.polynomial_power_basis.set(power_basis);
        Ok(self
            .data
            .source
            .polynomial_power_basis
            .get()
            .expect("parallel polynomial basis was initialized"))
    }

    pub(super) fn source_power_basis(&self) -> CurveResult<BezierParallelPowerBasisRef<'_>> {
        match self.source() {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                let (x_numerator, y_numerator) = self.polynomial_power_basis()?;
                Ok(BezierParallelPowerBasisRef {
                    x_numerator,
                    y_numerator,
                    weight: None,
                })
            }
            BezierParallelSource2::Rational(source) => {
                let source = source.homogeneous_power_basis()?;
                Ok(BezierParallelPowerBasisRef {
                    x_numerator: &source.x_numerator,
                    y_numerator: &source.y_numerator,
                    weight: Some(&source.weight),
                })
            }
        }
    }

    /// Certifies the orthonormal source frame at one selected point. A circle
    /// retains that local frame even when the source has singularities elsewhere.
    pub(super) fn certify_source_frame_at(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        if let Some(weight) = self.source_power_basis()?.weight {
            match policy.strict_predicate_pass(|| parameter.polynomial_sign(weight, policy))? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let speed_squared = parallel_speed_squared_polynomial(self.differential()?);
        Ok(
            match policy
                .strict_predicate_pass(|| parameter.polynomial_sign(&speed_squared, policy))?
            {
                Classification::Decided(RealSign::Positive) => Classification::Decided(()),
                Classification::Decided(RealSign::Zero) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "source speed squared was certified negative".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Certifies only the source and normal frame consumed by this query.
    /// Incident rays are open at their pole or speed barrier. A zero offset
    /// needs source finiteness but carries no unit-normal premise.
    pub(super) fn certify_source_frame_in_domain(
        &self,
        domain: SelectedThirdAxisDomain2<'_>,
        frame: Option<&BezierAnalyticParallelTangentField2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        policy.strict_predicate_pass(|| {
            let source = self.source_power_basis()?;
            if let Some(weight) = source.weight {
                match domain.polynomial_is_nonzero(weight, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if self.distance().zero_status() != ZeroKnowledge::Zero {
                let differential = self.differential()?;
                let (x, y) = frame
                    .map(|frame| (&frame.x[..], &frame.y[..]))
                    .unwrap_or((&differential.tangent_x, &differential.tangent_y));
                let speed_squared =
                    polynomial_add(&polynomial_multiply(x, x), &polynomial_multiply(y, y));
                match domain.polynomial_is_nonzero(&speed_squared, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(()))
        })
    }

    pub(super) fn certify_finite_source(
        source: &BezierParallelPowerBasisRef<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        Self::certify_finite_weight(source.weight, policy)
    }

    pub(super) fn certify_finite_weight(
        weight: Option<&[Real]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let Some(weight) = weight else {
            return Ok(Classification::Decided(()));
        };
        let weight_polynomial = match polynomial_from_coefficients(weight.to_vec(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(
            match weight_polynomial.isolate_unit_interval_roots(policy)? {
                Classification::Decided(roots) if roots.is_empty() => Classification::Decided(()),
                Classification::Decided(_) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(super) fn certify_regular_differential(
        differential: &BezierParallelDifferential2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        Self::certify_regular_tangent_field(
            &differential.tangent_x,
            &differential.tangent_y,
            policy,
        )
    }

    pub(super) fn certify_regular_tangent_field(
        tangent_x: &[Real],
        tangent_y: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let speed_squared = polynomial_add(
            &polynomial_multiply(tangent_x, tangent_x),
            &polynomial_multiply(tangent_y, tangent_y),
        );
        let speed_polynomial = match polynomial_from_coefficients(speed_squared, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match speed_polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) if roots.is_empty() => {}
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match real_sign(&speed_polynomial.evaluate(&Real::zero()), policy) {
            Some(RealSign::Positive) => Ok(Classification::Decided(())),
            Some(RealSign::Zero) => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Some(RealSign::Negative) => Err(CurveError::Topology(
                "Bezier tangent squared norm was certified negative".to_owned(),
            )),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    pub(super) fn differential(&self) -> CurveResult<&BezierParallelDifferential2> {
        if let Some(differential) = self.data.source.differential.get() {
            return Ok(differential);
        }
        let (tangent_x, tangent_y) = match self.source() {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                let (source_x, source_y) = self.polynomial_power_basis()?;
                (
                    polynomial_derivative(source_x),
                    polynomial_derivative(source_y),
                )
            }
            BezierParallelSource2::Rational(source) => {
                let source = source.homogeneous_power_basis()?;
                let x_numerator = polynomial_trim_structural_zeros(source.x_numerator.clone());
                let y_numerator = polynomial_trim_structural_zeros(source.y_numerator.clone());
                let weight = polynomial_trim_structural_zeros(source.weight.clone());
                let x_derivative = polynomial_derivative(&x_numerator);
                let y_derivative = polynomial_derivative(&y_numerator);
                let weight_derivative = polynomial_derivative(&weight);
                (
                    polynomial_subtract(
                        &polynomial_multiply(&x_derivative, &weight),
                        &polynomial_multiply(&x_numerator, &weight_derivative),
                    ),
                    polynomial_subtract(
                        &polynomial_multiply(&y_derivative, &weight),
                        &polynomial_multiply(&y_numerator, &weight_derivative),
                    ),
                )
            }
        };
        let differential = BezierParallelDifferential2 {
            tangent_derivative_x: polynomial_derivative(&tangent_x),
            tangent_derivative_y: polynomial_derivative(&tangent_y),
            tangent_x,
            tangent_y,
        };
        let _ = self.data.source.differential.set(differential);
        Ok(self
            .data
            .source
            .differential
            .get()
            .expect("parallel differential was initialized"))
    }

    /// Returns the signed distance measured along the source's left normal.
    pub fn distance(&self) -> &Real {
        &self.data.distance
    }

    /// Evaluates the retained source at any finite affine parameter.
    ///
    /// Corner construction uses this after an offset-incidence solve has
    /// certified the same parameter on an analytic parallel. Keeping the
    /// source evaluation on the shared carrier avoids allocating a temporary
    /// zero-distance parallel merely to recover the tangency contact.
    pub(crate) fn source_point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> Classification<Point2> {
        match self.source() {
            BezierParallelSource2::Quadratic(source) => {
                Classification::Decided(source.point_at(parameter.clone()))
            }
            BezierParallelSource2::Cubic(source) => {
                Classification::Decided(source.point_at(parameter.clone()))
            }
            BezierParallelSource2::Rational(source) => {
                source.point_at_affine_classified(parameter, policy)
            }
        }
    }

    pub(super) fn rational_source(&self) -> Option<&RationalBezier2> {
        match self.source() {
            BezierParallelSource2::Rational(source) => Some(source),
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => None,
        }
    }

    /// Evaluates this analytic parallel at any finite affine parameter.
    ///
    /// Fragment and operation ranges own parameter admission. Evaluation
    /// certifies a finite source point and, for nonzero displacement, its
    /// defined normal at the requested parameter.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(self.source_point_at(parameter, policy));
        }
        let differential = self.differential()?;
        let source_point = if let Some(source) = self.rational_source() {
            let source = source.homogeneous_power_basis()?;
            let weight = Real::eval_poly(&source.weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            Point2::new(
                (Real::eval_poly(&source.x_numerator, parameter) / &weight)?,
                (Real::eval_poly(&source.y_numerator, parameter) / weight)?,
            )
        } else {
            let (source_x, source_y) = self.polynomial_power_basis()?;
            Point2::new(
                Real::eval_poly(source_x, parameter),
                Real::eval_poly(source_y, parameter),
            )
        };
        let tangent_x = Real::eval_poly(&differential.tangent_x, parameter);
        let tangent_y = Real::eval_poly(&differential.tangent_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "Bezier derivative squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.sqrt()?;
        let normal_x = ((Real::zero() - &tangent_y) / &speed)?;
        let normal_y = (tangent_x / &speed)?;
        Ok(Classification::Decided(source_point.translated(
            self.distance() * normal_x,
            self.distance() * normal_y,
        )))
    }

    /// Evaluates the exact first derivative at any finite affine parameter.
    ///
    /// Fragment and operation ranges own admission. Source poles and undefined
    /// normals remain excluded; regular-source cusps have an exact zero derivative.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveDerivative2>> {
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return match self.source() {
                BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                    let differential = self.differential()?;
                    Ok(Classification::Decided(CurveDerivative2::new(
                        Real::eval_poly(&differential.tangent_x, parameter),
                        Real::eval_poly(&differential.tangent_y, parameter),
                    )))
                }
                BezierParallelSource2::Rational(source) => {
                    Ok(source.derivative_at_affine_classified(parameter, policy))
                }
            };
        }
        let differential = self.differential()?;
        let weight = if let Some(source) = self.rational_source() {
            let source = source.homogeneous_power_basis()?;
            let weight = Real::eval_poly(&source.weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            Some(weight)
        } else {
            None
        };
        let tangent_x = Real::eval_poly(&differential.tangent_x, parameter);
        let tangent_y = Real::eval_poly(&differential.tangent_y, parameter);
        let tangent_derivative_x = Real::eval_poly(&differential.tangent_derivative_x, parameter);
        let tangent_derivative_y = Real::eval_poly(&differential.tangent_derivative_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "Bezier derivative squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.clone().sqrt()?;
        let speed_cubed = &speed_squared * &speed;
        // For H=(X'W-XW', Y'W-YW'), S=H·H and C=H'_x H_y-H'_y H_x,
        // JH' S-JH(H·H')=H C. Both coordinates therefore share one scalar:
        // Q'=H*(1/W²+d C/S^(3/2)), with W=1 for a polynomial source.
        let curvature = &tangent_derivative_x * &tangent_y - &tangent_derivative_y * &tangent_x;
        let weight_squared = weight.map_or_else(Real::one, |weight| &weight * &weight);
        let source_scale = (Real::one() / &weight_squared)?;
        let normal_curvature = self.distance() * curvature;
        let mut scale = &source_scale + (&normal_curvature / speed_cubed)?;
        if scale.zero_status() != ZeroKnowledge::Zero
            && scale.exact_rational_ref().is_none()
            && let Ok(Some(certified_scale)) =
                policy.bounded_exact_predicate_pass(|| -> CurveResult<Option<Real>> {
                    let Some(sign @ (RealSign::Positive | RealSign::Negative)) =
                        real_sign(&normal_curvature, policy)
                    else {
                        return Ok(None);
                    };
                    // K²=S³ determines the magnitude without a nested square
                    // root. Its sign distinguishes a cusp from the opposite
                    // normal sheet, where the derivative is twice P'.
                    let signed_curvature = &normal_curvature * &weight_squared;
                    let squared_difference = &signed_curvature * &signed_curvature
                        - &speed_squared * &speed_squared * &speed_squared;
                    let selected_scale = if sign == RealSign::Negative {
                        Real::zero()
                    } else {
                        Real::from(2_i8) * &source_scale
                    };
                    match crate::classify::is_zero(&squared_difference, policy) {
                        Some(true) => return Ok(Some(selected_scale)),
                        Some(false) => return Ok(None),
                        None => {}
                    }
                    if matches!(self.source(), BezierParallelSource2::Quadratic(_)) {
                        let speed_squared = parallel_speed_squared_polynomial(differential);
                        let mut curvature = parallel_signed_curvature_polynomial(
                            differential,
                            None,
                            self.distance(),
                        );
                        if sign == RealSign::Positive {
                            curvature = polynomial_scale(&curvature, &-Real::one());
                        }
                        if exact_quadratic_parallel_cusp_candidates(
                            &speed_squared,
                            &curvature,
                            policy,
                        )?
                        .is_some_and(|candidates| candidates.iter().any(|cusp| cusp == parameter))
                        {
                            return Ok(Some(selected_scale));
                        }
                    }
                    Ok(None)
                })
        {
            // Reuse pointwise or represented selected-branch evidence without
            // discovering a global root set or consuming approximation.
            scale = certified_scale;
        }
        Ok(Classification::Decided(CurveDerivative2::new(
            tangent_x * &scale,
            tangent_y * scale,
        )))
    }

    /// Isolates source singularities and parallel cusps on a closed exact range.
    ///
    /// Poles outside the range do not restrict this query. Root evidence stays
    /// in the source chart, and selected endpoint authorities decide membership.
    ///
    /// On regular source spans a parallel cusp satisfies
    /// `d * (P'' x P') + |P'|^3 = 0`. The root scheduler isolates the polynomial
    /// obtained by squaring that equation, then rejects source singularities and
    /// the opposite-sign roots introduced by squaring.
    pub fn singularity_analysis(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelSingularityAnalysis2>> {
        let domain = CurveParameterDomain2::new(range, None);
        let source = match self.rational_source() {
            Some(source) => Some(source.homogeneous_power_basis()?),
            None => None,
        };
        let differential = self.differential()?;
        let weight = if let Some(source) = source {
            let weight = polynomial_trim_structural_zeros(source.weight.clone());
            let weight_polynomial = match polynomial_from_coefficients(weight.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match domain.finite_roots(&weight_polynomial, policy)? {
                Classification::Decided(roots) if roots.is_empty() => {}
                Classification::Decided(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            Some(weight)
        } else {
            None
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let speed_polynomial = match polynomial_from_coefficients(speed_squared.clone(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let source_singularities = match domain.finite_roots(&speed_polynomial, policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(
                BezierParallelSingularityAnalysis2 {
                    range: range.clone(),
                    source_singularities,
                    parallel_cusps: Vec::new(),
                    source_speed_squared_degree: speed_polynomial.degree(),
                    parallel_cusp_polynomial_degree: None,
                },
            ));
        }

        let signed_curvature_term =
            parallel_signed_curvature_polynomial(differential, weight.as_deref(), self.distance());
        let squared_cusp_polynomial = polynomial_subtract(
            &polynomial_multiply(&signed_curvature_term, &signed_curvature_term),
            &polynomial_power(&speed_squared, 3),
        );
        let cusp_polynomial = match polynomial_from_coefficients(squared_cusp_polynomial, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                // K²=S³ identically does not choose a normal sheet. When S
                // is nonzero throughout this connected range, K cannot
                // vanish or change sign. K>0 at one retained endpoint then
                // excludes the unsquared cusp equation K+S^(3/2)=0 everywhere.
                if source_singularities.is_empty()
                    && range
                        .start()
                        .polynomial_sign(&signed_curvature_term, policy)?
                        == Classification::Decided(RealSign::Positive)
                {
                    return Ok(Classification::Decided(
                        BezierParallelSingularityAnalysis2 {
                            range: range.clone(),
                            source_singularities,
                            parallel_cusps: Vec::new(),
                            source_speed_squared_degree: speed_polynomial.degree(),
                            parallel_cusp_polynomial_degree: None,
                        },
                    ));
                }
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        // Cusps stay selected roots of the cusp polynomial. Closed-form
        // radicals would defeat later exact sign and equality decisions at
        // cusp-split domain boundaries.
        let candidates = match domain.finite_roots(&cusp_polynomial, policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut parallel_cusps = Vec::new();
        for candidate in candidates {
            // On the squared cusp equation, a negative curvature term proves
            // both the intended unsquared sign and nonzero source speed. At
            // a source singularity the hodograph and curvature term vanish.
            // One sign certificate therefore excludes those roots too; no
            // comparison with a separately isolated source root is needed.
            let sign =
                match signed_coefficients_at_parameter(&signed_curvature_term, &candidate, policy)?
                {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            if sign == RealSign::Negative {
                parallel_cusps.push(candidate);
            }
        }
        Ok(Classification::Decided(
            BezierParallelSingularityAnalysis2 {
                range: range.clone(),
                source_singularities,
                parallel_cusps,
                source_speed_squared_degree: speed_polynomial.degree(),
                parallel_cusp_polynomial_degree: Some(cusp_polynomial.degree()),
            },
        ))
    }

    /// A constant parallel must satisfy the squared cusp equation identically.
    /// One nonzero coefficient therefore certifies a nonconstant image on
    /// every nonempty regular interval, even when its midpoint is a cusp or
    /// its two endpoints coincide. No root isolation or scalar sampling is needed.
    pub(crate) fn nonconstant_image_certificate(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let differential = self.differential()?;
        let speed_squared = parallel_speed_squared_polynomial(differential);
        let source = self.source_power_basis()?;
        let curvature =
            parallel_signed_curvature_polynomial(differential, source.weight, self.distance());
        let cusp = polynomial_subtract(
            &polynomial_multiply(&curvature, &curvature),
            &polynomial_power(&speed_squared, 3),
        );
        Ok(
            match polynomial_coefficients_are_identically_zero(&cusp, policy) {
                Classification::Decided(false) => Classification::Decided(()),
                // The squared identity alone cannot select the normal's sheet.
                Classification::Decided(true) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Materializes an exactly recognized rational-circle parallel without
    /// reconstructing its homogeneous speed polynomial.
    ///
    /// The shared circular-conic recognizer supplies the exact supporting-circle
    /// and traversal certificate used by rational intersection and decomposition.
    /// Scaling the original homogeneous controls about that center preserves
    /// its parameter exactly, including negative-radius continuation.
    pub(crate) fn exact_circular_parallel_component(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        let Some(source) = self.rational_source() else {
            return Ok(Classification::Decided(None));
        };
        let arc = match crate::arc_bezier::rational_bezier_circular_arc(source, policy)? {
            Classification::Decided(Some(arc)) => arc,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_scale = arc.left_offset_radius_scale(self.distance())?;
        if radius_scale == Real::one() {
            return Ok(Classification::Decided(Some(source.clone())));
        }
        let translation_x = arc.center().x() * (Real::one() - &radius_scale);
        let translation_y = arc.center().y() * (Real::one() - &radius_scale);
        let curve = source.transformed_affine([
            &radius_scale,
            &Real::zero(),
            &Real::zero(),
            &radius_scale,
            &translation_x,
            &translation_y,
        ]);
        let curve = if matches!(
            real_sign(&radius_scale, &CurveContext::STRICT),
            Some(RealSign::Positive | RealSign::Negative)
        ) {
            let radius_squared = arc.radius_squared_ref() * &radius_scale * &radius_scale;
            let two = Real::from(2_i8);
            curve.with_implicit_quadratic_conic(
                Arc::new([
                    Real::one(),
                    Real::zero(),
                    Real::one(),
                    -(&two * arc.center().x()),
                    -(&two * arc.center().y()),
                    arc.center().x() * arc.center().x() + arc.center().y() * arc.center().y()
                        - &radius_squared,
                ]),
                Some(Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
                    center: arc.center().clone(),
                    radius_squared,
                    tangent_contacts: None,
                })),
            )
        } else {
            curve
        };
        Ok(Classification::Decided(Some(curve)))
    }

    /// Materializes this parallel exactly when the homogeneous hodograph is Pythagorean.
    ///
    /// For a rational source `(X/W, Y/W)`, let
    /// `H = (X'W-XW', Y'W-YW')`. If `H dot H = sigma^2` for a polynomial
    /// `sigma` with certified nonzero sign over `[0, 1]`, the unit normal is
    /// rational and the complete parallel is converted to an arbitrary-degree
    /// [`RationalBezier2`]. Polynomial PH curves are the `W=1` specialization.
    /// `None` means this authored-unit PH construction is unavailable, for
    /// example because the polynomial-square identity fails or the source has
    /// a stationary parameter or pole. Unresolved scalar signs remain explicit
    /// [`Classification::Uncertain`].
    pub fn exact_pythagorean_hodograph_offset(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        if let Some(cached) = self.data.certified_ph_offset.get() {
            return Ok(Classification::Decided(cached.as_deref().cloned()));
        }
        // Curve construction is structural. APPROXIMATE_512 may terminate an
        // equality predicate, but it must never select a speed sheet or mint a
        // rational component. Reuse only the caller's STRICT counterpart here.
        match self.compute_pythagorean_hodograph_offset(&policy.strict_counterpart())? {
            Classification::Decided(offset) => Ok(Classification::Decided(
                self.retain_certified_ph_offset(offset),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(super) fn retain_certified_ph_offset(
        &self,
        offset: Option<CertifiedPythagoreanHodographOffset2>,
    ) -> Option<CertifiedPythagoreanHodographOffset2> {
        let _ = self.data.certified_ph_offset.set(offset.map(Arc::new));
        self.data
            .certified_ph_offset
            .get()
            .expect("a certified PH result was retained")
            .as_deref()
            .cloned()
    }

    /// Retains the complete source hodograph's unit speed proof independently
    /// of offset distance. Primitive fields selected on a regular range obtain
    /// their own certificate and never populate this source-wide cache.
    pub(super) fn source_unit_ph_speed(
        &self,
        tangent_x: &[Real],
        tangent_y: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Arc<BezierParameterPolynomial>>>> {
        if let Some(speed) = self.data.source.unit_ph_speed.get() {
            return Ok(Classification::Decided(speed.clone()));
        }
        match certify_ph_speed_on_range(
            tangent_x,
            tangent_y,
            &Real::zero(),
            &CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(speed) => {
                let _ = self.data.source.unit_ph_speed.set(speed);
                Ok(Classification::Decided(
                    self.data
                        .source
                        .unit_ph_speed
                        .get()
                        .expect("the source speed decision was retained")
                        .clone(),
                ))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    #[cold]
    pub(super) fn compute_pythagorean_hodograph_offset(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        // A retained quadratic line is an affine degree elevation by
        // construction.  Reuse that certificate before rebuilding the zero
        // quadratic coefficient from independently embedded endpoint fields:
        // the latter equality can be harder to prove than the authored line
        // fact even though its hodograph is constant.
        if let BezierParallelSource2::Quadratic(source) = self.source()
            && let Some(line) = source.retained_exact_line_image()
        {
            let offset_line = line.offset_left(self.distance().clone())?;
            let translation = offset_line.start().delta_from(line.start());
            let curve = RationalBezier2::try_new_with_exact_line_image(
                source
                    .control_points()
                    .into_iter()
                    .map(|point| point.translated(translation.0.clone(), translation.1.clone()))
                    .collect(),
                vec![Real::one(); 3],
                offset_line,
            )?;
            let (dx, dy) = line.delta();
            let speed_polynomial = match self.source_unit_ph_speed(&[dx], &[dy], policy)? {
                Classification::Decided(Some(speed)) => speed,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(Some(
                CertifiedPythagoreanHodographOffset2 {
                    curve,
                    speed_polynomial,
                    source_degree: 2,
                    distance: self.distance().clone(),
                },
            )));
        }
        let differential = self.differential()?;
        let speed = match self.source_unit_ph_speed(
            &differential.tangent_x,
            &differential.tangent_y,
            policy,
        )? {
            Classification::Decided(Some(speed)) => speed,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(self
            .compute_pythagorean_hodograph_offset_from_tangent_field(
                &differential.tangent_x,
                &differential.tangent_y,
                &speed,
                &CurveParameterRange2::unit(),
                true,
                policy,
            )?
            .map(|curve| {
                curve.map(|curve| CertifiedPythagoreanHodographOffset2 {
                    curve,
                    speed_polynomial: speed,
                    source_degree: self.source_degree(),
                    distance: self.distance().clone(),
                })
            }))
    }

    /// Materializes one exact rational parallel from an oriented PH tangent
    /// field. The field can be either the source's complete homogeneous
    /// hodograph or the primitive GCD quotient selected on one regular range.
    /// The retained speed certificate selects the polynomial sheet and proves
    /// root exclusion on `range`. Callers prove that the field orientation agrees
    /// with the source before entering this constructor.
    #[cold]
    pub(super) fn compute_pythagorean_hodograph_offset_from_tangent_field(
        &self,
        tangent_x: &[Real],
        tangent_y: &[Real],
        speed_polynomial: &BezierParameterPolynomial,
        range: &CurveParameterRange2,
        allow_circular_specialization: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        let speed = speed_polynomial.coefficients();
        // Preserve a constant tangent field as a direct affine translation.
        // The general homogeneous formula is algebraically equivalent, but it
        // introduces canceling speed radicals into every control coordinate.
        // Keeping the source coefficients intact is both smaller and essential
        // when an opaque authored coefficient proves line rank by correlation.
        if let ([tangent_x], [tangent_y]) = (tangent_x, tangent_y) {
            let translation_x = ((-self.distance() * tangent_y) / &speed[0])?;
            let translation_y = ((self.distance() * tangent_x) / &speed[0])?;
            let source = self.source().to_rational_bezier()?;
            let curve = source.transformed_affine([
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &Real::one(),
                &translation_x,
                &translation_y,
            ]);
            return Ok(Classification::Decided(Some(curve)));
        }
        let rational_source = self.rational_source();
        let rational_power_basis = match rational_source {
            Some(source) => Some(source.homogeneous_power_basis()?),
            None => None,
        };
        let polynomial_power_basis = if rational_source.is_none() {
            Some(self.polynomial_power_basis()?)
        } else {
            None
        };
        let (source_x, source_y) = match (rational_power_basis, polynomial_power_basis) {
            (Some(source), None) => (&source.x_numerator, &source.y_numerator),
            (None, Some((source_x, source_y))) => (source_x, source_y),
            _ => unreachable!("parallel source has exactly one power basis"),
        };
        let weight = if let Some(source) = rational_power_basis {
            let weight = polynomial_trim_structural_zeros(source.weight.clone());
            let weight_polynomial = match polynomial_from_coefficients(weight.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match CurveParameterDomain2::new(range, None)
                .finite_roots(&weight_polynomial, policy)?
            {
                Classification::Decided(roots) if roots.is_empty() => {}
                Classification::Decided(_) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            Some(weight)
        } else {
            None
        };
        let circular_component = if allow_circular_specialization {
            match self.exact_circular_parallel_component(policy)? {
                Classification::Decided(component) => component,
                // Circle recognition is a specialization. The already-certified
                // PH speed below remains the complete construction authority.
                Classification::Uncertain(_) => None,
            }
        } else {
            None
        };
        if let Some(curve) = circular_component {
            return Ok(Classification::Decided(Some(curve)));
        }

        let (normal_x_term, normal_y_term, denominator) = if let Some(weight) = &weight {
            let weighted_distance = polynomial_scale(weight, self.distance());
            (
                polynomial_multiply(&weighted_distance, tangent_y),
                polynomial_multiply(&weighted_distance, tangent_x),
                polynomial_multiply(weight, speed),
            )
        } else {
            (
                polynomial_scale(tangent_y, self.distance()),
                polynomial_scale(tangent_x, self.distance()),
                speed.to_vec(),
            )
        };
        let numerator_x =
            polynomial_subtract(&polynomial_multiply(source_x, speed), &normal_x_term);
        let numerator_y = polynomial_add(&polynomial_multiply(source_y, speed), &normal_y_term);
        let base_degree = numerator_x
            .len()
            .max(numerator_y.len())
            .max(denominator.len())
            .saturating_sub(1);
        let weights = power_to_bernstein_coefficients(&denominator, base_degree)?;
        let x = power_to_bernstein_coefficients(&numerator_x, base_degree)?;
        let y = power_to_bernstein_coefficients(&numerator_y, base_degree)?;
        let controls = x
            .into_iter()
            .zip(y)
            .zip(weights)
            .map(|((x, y), weight)| crate::HomogeneousControl2::new(x, y, weight))
            .collect();
        RationalBezier2::from_homogeneous_controls(controls, policy).map(|curve| curve.map(Some))
    }

    /// Builds a Levien-style endpoint-tangent cubic for later verification.
    ///
    /// Independent endpoint tangents provide two scalar arm lengths, solved so
    /// the cubic also passes through the exact analytic parallel at `t=1/2`.
    /// Parallel, negative-arm, or undecidable cases use exact Hermite endpoint
    /// derivatives instead. Neither lane is accepted without
    /// [`Self::verify_polynomial_candidate`].
    pub fn levien_cubic_candidate(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LevienCubicOffsetCandidate2>> {
        let zero = Real::zero();
        let one = Real::one();
        let half = (Real::one() / Real::from(2_i8))?;
        let start = match self.point_at(&zero, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match self.point_at(&one, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let midpoint = match self.point_at(&half, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let start_derivative = match self.derivative_at(&zero, policy)? {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end_derivative = match self.derivative_at(&one, policy)? {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if start_derivative.zero_status() != ZeroKnowledge::NonZero
            || end_derivative.zero_status() != ZeroKnowledge::NonZero
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }

        let tangent_cross = start_derivative.dx() * end_derivative.dy()
            - start_derivative.dy() * end_derivative.dx();
        let midpoint_base = start.lerp(&end, half);
        let midpoint_delta = midpoint.delta_from(&midpoint_base);
        let rhs_scale = (Real::from(8_i8) / Real::from(3_i8))?;
        let rhs_x = midpoint_delta.0 * &rhs_scale;
        let rhs_y = midpoint_delta.1 * rhs_scale;
        let solved_arms = match real_sign(&tangent_cross, policy) {
            Some(RealSign::Positive | RealSign::Negative) => {
                let start_arm = ((&rhs_x * end_derivative.dy() - &rhs_y * end_derivative.dx())
                    / &tangent_cross)?;
                let end_arm = ((start_derivative.dy() * &rhs_x - start_derivative.dx() * &rhs_y)
                    / &tangent_cross)?;
                match (real_sign(&start_arm, policy), real_sign(&end_arm, policy)) {
                    (Some(RealSign::Positive), Some(RealSign::Positive)) => {
                        Some((start_arm, end_arm))
                    }
                    _ => None,
                }
            }
            Some(RealSign::Zero) | None => None,
        };
        let (control1, control2, matched_midpoint) = if let Some((start_arm, end_arm)) = solved_arms
        {
            (
                start.translated(
                    start_derivative.dx() * &start_arm,
                    start_derivative.dy() * start_arm,
                ),
                end.translated(
                    Real::zero() - end_derivative.dx() * &end_arm,
                    Real::zero() - end_derivative.dy() * end_arm,
                ),
                true,
            )
        } else {
            let one_third = (Real::one() / Real::from(3_i8))?;
            (
                start.translated(
                    start_derivative.dx() * &one_third,
                    start_derivative.dy() * &one_third,
                ),
                end.translated(
                    Real::zero() - end_derivative.dx() * &one_third,
                    Real::zero() - end_derivative.dy() * one_third,
                ),
                false,
            )
        };
        Ok(Classification::Decided(LevienCubicOffsetCandidate2 {
            curve: CubicBezier2::new(start, control1, control2, end),
            matched_midpoint,
            distance: self.distance().clone(),
        }))
    }

    /// Conservatively verifies a polynomial Bezier candidate against this exact parallel.
    ///
    /// Each dyadic leaf bounds the midpoint error plus a Lipschitz remainder.
    /// Source and candidate derivative control hulls bound speed, while
    /// `|n'| <= |P''| / |P'|` bounds variation of the exact unit normal. No
    /// sampled normal ray or floating conversion participates in acceptance.
    pub fn verify_polynomial_candidate(
        &self,
        candidate: BezierParallelApproximationCurve2,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelApproximation2>> {
        let analysis = match self.singularity_analysis(&CurveParameterRange2::unit(), policy)? {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let source = match self.source() {
            BezierParallelSource2::Quadratic(source) => {
                PolynomialBezierNode2::Quadratic(source.clone())
            }
            BezierParallelSource2::Cubic(source) => PolynomialBezierNode2::Cubic(source.clone()),
            BezierParallelSource2::Rational(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let candidate_node = PolynomialBezierNode2::from_candidate(&candidate);
        let mut trace = ParallelVerificationTrace::default();
        match verify_parallel_node(
            source,
            candidate_node,
            self.distance(),
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => Ok(Classification::Decided(
                CertifiedBezierParallelApproximation2 {
                    curve: candidate,
                    error_bound: options.max_error.clone(),
                    leaf_count: trace.leaf_count,
                    maximum_depth: trace.maximum_depth,
                    distance: self.distance().clone(),
                },
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}
