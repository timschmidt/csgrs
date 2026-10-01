//! Selected circle/rational-Bezier systems, intersections and component
//! replay.

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    /// Intersects this selected algebraic half circle with a finite rational
    /// Bezier without constructing algebraic control points for the circle.
    ///
    /// The circle equation is specialized in the exact local field of the cusp
    /// root. A resultant supplies ordinary parameters on `other`; local-field
    /// Sturm counting removes candidates contributed by other roots of the
    /// cusp polynomial. The half-plane equation then selects exactly one of the
    /// two circle halves, and the signed tangent equation classifies contact.
    pub(crate) fn rational_intersections(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        Ok(self
            .rational_intersections_internal(other, range, false, policy)?
            .map(|(intersections, _)| intersections))
    }

    /// Builds the simple radial relation between this selected center and a
    /// rational point on a concentric circle.
    ///
    /// If `p` is the rational radial and `q` the selected center radial, the
    /// contact is one root of `cross(p, q) = 0`.  Substituting the analytic
    /// parallel expression for `q` leaves one two-term source-speed radical.
    /// Its squared incidence has simple directional roots instead of the
    /// repeated root of the tangent circle equation.  The retained unsquared
    /// relation and dot sign subsequently select the geometric normal branch
    /// and reject the antipodal point.
    pub(in crate::bezier_offset) fn selected_parallel_normal_concentric_arc_tangent_candidate(
        &self,
        other: &RationalBezier2,
        support: &crate::CircularArc2,
        expected_radial_dot_sign: RealSign,
    ) -> CurveResult<BezierSelectedParallelNormalRationalTangentCandidate2> {
        let frame = self.data.frame.parallel_normal().ok_or_else(|| {
            CurveError::Topology("a concentric arc map lost its parallel-normal frame".into())
        })?;
        let source = frame.center_support.source_power_basis()?;
        let differential = frame.center_support.differential()?;
        let other = other.homogeneous_power_basis()?;
        let unit = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit);
        let delta_x = polynomial_subtract(
            source.x_numerator,
            &polynomial_scale(source_weight, support.center().x()),
        );
        let delta_y = polynomial_subtract(
            source.y_numerator,
            &polynomial_scale(source_weight, support.center().y()),
        );
        let other_delta_x = polynomial_subtract(
            &other.x_numerator,
            &polynomial_scale(&other.weight, support.center().x()),
        );
        let other_delta_y = polynomial_subtract(
            &other.y_numerator,
            &polynomial_scale(&other.weight, support.center().y()),
        );
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let radial_alignment = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_subtract(
                &bivariate_outer_product(&delta_y, &other_delta_x),
                &bivariate_outer_product(&delta_x, &other_delta_y),
            ),
            radical: bivariate_scale(
                bivariate_add(
                    &bivariate_outer_product(
                        &polynomial_multiply(source_weight, &differential.tangent_x),
                        &other_delta_x,
                    ),
                    &bivariate_outer_product(
                        &polynomial_multiply(source_weight, &differential.tangent_y),
                        &other_delta_y,
                    ),
                ),
                frame.center_support.distance(),
            ),
        };
        let radial_dot = BezierAlgebraicCuspTwoTermExpression2 {
            rational: bivariate_add(
                &bivariate_outer_product(&delta_x, &other_delta_x),
                &bivariate_outer_product(&delta_y, &other_delta_y),
            ),
            radical: bivariate_scale(
                bivariate_subtract(
                    &bivariate_outer_product(
                        &polynomial_multiply(source_weight, &differential.tangent_x),
                        &other_delta_y,
                    ),
                    &bivariate_outer_product(
                        &polynomial_multiply(source_weight, &differential.tangent_y),
                        &other_delta_x,
                    ),
                ),
                frame.center_support.distance(),
            ),
        };
        let speed_squared = bivariate_outer_product(&speed_squared, &unit);
        let incidence = bivariate_subtract(
            &bivariate_multiply(
                &bivariate_multiply(&radial_alignment.rational, &radial_alignment.rational),
                &speed_squared,
            ),
            &bivariate_multiply(&radial_alignment.radical, &radial_alignment.radical),
        );
        Ok(BezierSelectedParallelNormalRationalTangentCandidate2 {
            incidence,
            radial_alignment,
            radial_dot,
            speed_squared,
            expected_radial_dot_sign,
        })
    }

    /// Replays a circle contact from an exact tangent-support certificate.
    ///
    /// A direct arc/Bezier fillet center already lies on the exact concentric
    /// arc offset, and its three radii prove that the fillet and authored arc
    /// circles are tangent. Isolating the ordinary circle equation at that
    /// point asks a fiber solver to rediscover a double root. Instead, isolate
    /// the simple radial directions on the rational arc. The unsquared radial
    /// relation, signed radial dot, concentric-circle equations, and signed
    /// radius identity together prove the original unsquared circle contact
    /// and zero tangent cross. The selected-half predicates are then replayed
    /// normally. No squared candidate is admitted on its own.
    pub(crate) fn certified_tangent_rational_intersections(
        &self,
        other: &RationalBezier2,
        support: &crate::CircularArc2,
        source_radius: &Real,
        signed_center_radius: &Real,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        if !self.uses_selected_parallel_normal_frame() {
            return self.rational_intersections_with_parameter_map(
                other,
                &crate::CurveParameterRange2::unit(),
                policy,
            );
        }
        // The retained fillet carrier owns `support` as the STRICT promotion
        // certificate for `other`; manually authored rational conics need not
        // duplicate that proof as lineage metadata. When provenance is
        // available, still reject a mismatched internal caller immediately.
        if let Some(circle) = other.retained_circular_conic() {
            let center_residual = circle.center.distance_squared(support.center());
            match real_sign(&center_residual, &CurveContext::STRICT) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a certified arc tangent replay used a different rational circle".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let radius_residual = &circle.radius_squared - support.radius_squared_ref();
            match real_sign(&radius_residual, &CurveContext::STRICT) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a certified arc tangent replay used a different rational circle".into(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let source_radius_mismatch = source_radius * source_radius - support.radius_squared_ref();
        match real_sign(&source_radius_mismatch, &CurveContext::STRICT) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a certified arc tangent replay retained an inconsistent source radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let expected_radial_dot_sign = match real_sign(signed_center_radius, &CurveContext::STRICT)
        {
            Some(RealSign::Positive) => RealSign::Positive,
            Some(RealSign::Negative) => RealSign::Negative,
            Some(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a certified arc tangent retained a zero signed center radius".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let signed_radius_delta = signed_center_radius - source_radius;
        let tangent_radius_residual = &signed_radius_delta * &signed_radius_delta
            - self.radial_distance() * self.radial_distance();
        match real_sign(&tangent_radius_residual, &CurveContext::STRICT) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a certified arc tangent replay lost its signed tangent-radius identity".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let frame = self.data.frame.parallel_normal().ok_or_else(|| {
            CurveError::Topology("a certified arc tangent lost its parallel-normal frame".into())
        })?;
        let center = BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
            frame.center_support.clone(),
            &frame.center_parameter,
            Real::zero(),
            &CurveContext::STRICT,
        )
        .expect("a parallel-normal frame owns a scalar parameter");
        match center.circle_residual_sign_to_exact(
            support.center(),
            &(signed_center_radius * signed_center_radius),
            &CurveContext::STRICT,
        )? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a certified arc tangent center left its concentric offset".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let candidate = self.selected_parallel_normal_concentric_arc_tangent_candidate(
            other,
            support,
            expected_radial_dot_sign,
        )?;
        let result = self.selected_parallel_normal_rational_intersections_internal(
            other,
            &crate::CurveParameterRange2::unit(),
            true,
            Some(candidate),
            policy,
        )?;
        Ok(result)
    }

    /// Discovers contacts and components in a pole-free enclosure of `range`.
    /// The consuming curve pair clips with its original endpoint authorities,
    /// after the circle's incidence evidence is available for endpoint identity.
    pub(crate) fn rational_intersections_with_parameter_map(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        self.rational_intersections_internal(other, range, true, policy)
    }

    /// Returns the lower-degree predicate whose sign at every finite incidence
    /// contact is the oriented tangent cross sign.
    pub(in crate::bezier_offset) fn rational_incidence_tangent_cross_sign_polynomial(
        &self,
        incidence: &BivariatePolynomial,
    ) -> BivariatePolynomial {
        // On `incidence = 0`, differentiating in the rational parameter gives
        // `2 * dot(Q-C,Q') * D / W`. The stored tangent cross is that dot
        // product times `-turn * D * W`, so the two expressions differ only
        // by the strictly positive factor `W^2 / 2` at a finite contact. The
        // derivative is both smaller and directly correlated with the
        // isolated incidence root.
        bivariate_scale(
            bivariate_parameter_derivative(incidence, CurveResultantParameter::Second),
            &(-self.turn_sign()),
        )
    }

    /// Retains a rational-frame circle contact in the selected center fiber
    /// when global resultant projection cannot sign its coefficients.
    ///
    /// Every expression here is the zero-radical specialization of the
    /// general selected-parallel-normal system. Reusing that local authority
    /// avoids a duplicate solver and keeps the contact as one shared root
    /// allocation rather than a degree-multiplied global scalar.
    pub(in crate::bezier_offset) fn rational_frame_selected_fiber_intersections(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: &BezierAlgebraicCuspCircleRationalSystem2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let zero = BivariatePolynomial::new(vec![vec![Real::zero()]]);
        let pure = |rational: &BivariatePolynomial| BezierAlgebraicCuspTwoTermExpression2 {
            rational: rational.clone(),
            radical: zero.clone(),
        };
        self.selected_parallel_normal_rational_selected_fiber_intersections(
            other,
            range,
            BezierSelectedParallelNormalCircleRationalSystem2 {
                incidence: system.incidence.clone(),
                circle: pure(&system.incidence),
                selected_half_plane: system.selected_half_plane.clone(),
                diameter: pure(&system.diameter_side),
                radius_squared_denominator: system.radius_squared_denominator.clone(),
                speed_squared: BivariatePolynomial::new(vec![vec![Real::one()]]),
                tangent_cross: pure(&system.tangent_cross),
                angular_tangent: pure(&system.angular_tangent),
            },
            self.cusp_parameter().clone(),
            None,
            Some(self.rational_incidence_tangent_cross_sign_polynomial(&system.incidence)),
            policy,
        )
    }

    pub(in crate::bezier_offset) fn selected_parallel_normal_rational_selected_fiber_intersections(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: BezierSelectedParallelNormalCircleRationalSystem2,
        center_parameter: BezierAlgebraicParameter2,
        candidate: Option<BezierSelectedParallelNormalRationalTangentCandidate2>,
        tangent_cross_sign_predicate: Option<BivariatePolynomial>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let tangent_candidates = candidate.is_some();
        let (candidate_incidence, tangent_certificate) = candidate.map_or_else(
            || (system.incidence.clone(), None),
            |candidate| {
                (
                    candidate.incidence,
                    Some((
                        candidate.radial_alignment,
                        candidate.radial_dot,
                        candidate.speed_squared,
                        candidate.expected_radial_dot_sign,
                    )),
                )
            },
        );
        // Seed under STRICT; publish the policy consumed by later predicates.
        // Refinement may represent the center exactly; the shared finite-fiber
        // kernel handles either form.
        let roots = match policy.strict_predicate_pass(|| {
            selected_fiber_parameters_in_range(
                &candidate_incidence,
                &center_parameter,
                range,
                policy,
            )
        })? {
            Classification::Decided(Some(roots)) if roots.is_empty() => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                        contacts: Vec::new(),
                        overlaps: Vec::new(),
                    },
                ));
            }
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => {
                if tangent_candidates {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                // The incidence is the norm of A*sqrt(speed_squared)+B.
                // A zero norm may belong only to the conjugate speed sheet.
                match policy.strict_predicate_pass(|| {
                    algebraic_selected_square_root_polynomial_is_identically_zero(
                        &system.circle.radical,
                        &system.circle.rational,
                        &system.speed_squared,
                        &center_parameter,
                        policy,
                    )
                })? {
                    Classification::Decided(true) => {
                        return self.selected_parallel_normal_replay_rational_circle_component(
                            other,
                            range,
                            system,
                            center_parameter,
                            policy,
                        );
                    }
                    Classification::Decided(false) => {
                        // The nonzero authored factor has exactly A's zeros:
                        // its conjugate vanishes identically, so B=A*sqrt(S).
                        // A is therefore nonzero, and this descent terminates
                        // in the same finite-contact authority used above.
                        let mut system = system;
                        system.incidence = system.circle.rational.clone();
                        return self
                            .selected_parallel_normal_rational_selected_fiber_intersections(
                                other,
                                range,
                                system,
                                center_parameter,
                                None,
                                tangent_cross_sign_predicate,
                                policy,
                            );
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
        let parameters = roots;
        let isolated_incidence = if tangent_candidates {
            None
        } else {
            parameters
                .first()
                .map(|parameter| parameter.data.authority.clone())
        };

        let mut retained = Vec::with_capacity(parameters.len());
        for other_parameter in parameters {
            if let Some((radial_alignment, radial_dot, speed_squared, expected_dot_sign)) =
                &tangent_certificate
            {
                match other_parameter.radical_sum_sign(radial_alignment, speed_squared, policy)? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match other_parameter.radical_sum_sign(radial_dot, speed_squared, policy)? {
                    Classification::Decided(sign) if sign == *expected_dot_sign => {}
                    Classification::Decided(_) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                match other_parameter.radical_sum_sign(
                    &system.circle,
                    &system.speed_squared,
                    policy,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let selected =
                match other_parameter.predicate_sign(&system.selected_half_plane, policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match other_parameter.radical_sum_sign(
                    &system.diameter,
                    &system.speed_squared,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero general selected circle had an indeterminate local rational endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let tangent_cross_sign = if tangent_candidates {
                RealSign::Zero
            } else if let Some(predicate) = &tangent_cross_sign_predicate {
                match other_parameter.predicate_sign(predicate, policy)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                match other_parameter.radical_sum_sign(
                    &system.tangent_cross,
                    &system.speed_squared,
                    policy,
                )? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            retained.push((other_parameter, location, tangent_cross_sign));
        }

        let map = BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2 {
            data: Arc::new(
                BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    isolated_incidence,
                    diameter: system.diameter,
                    radius_squared_denominator: system.radius_squared_denominator,
                    speed_squared: system.speed_squared,
                    tangent_cross: system.tangent_cross,
                    angular_tangent: system.angular_tangent,
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                },
            ),
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                contacts: retained
                    .into_iter()
                    .map(|(other_parameter, location, tangent_cross_sign)| {
                        map.contact(other_parameter, location, tangent_cross_sign)
                    })
                    .collect(),
                overlaps: Vec::new(),
            },
        ))
    }

    /// Replays a rational carrier whose complete image lies on this selected
    /// supporting circle. Diameter and angular-stationary roots partition the
    /// source into regular monotone cells, all retained in one selected field.
    pub(in crate::bezier_offset) fn selected_parallel_normal_replay_rational_circle_component(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: BezierSelectedParallelNormalCircleRationalSystem2,
        center_parameter: BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        // The source speed is certified positive at the retained center.
        // A missing radical needs no conjugate equation or repeated roots.
        let angular_incidence = if system
            .angular_tangent
            .radical
            .coefficients
            .iter()
            .flatten()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            system.angular_tangent.rational.clone()
        } else if system
            .angular_tangent
            .rational
            .coefficients
            .iter()
            .flatten()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            system.angular_tangent.radical.clone()
        } else {
            bivariate_subtract(
                &bivariate_multiply(
                    &bivariate_multiply(
                        &system.angular_tangent.rational,
                        &system.angular_tangent.rational,
                    ),
                    &system.speed_squared,
                ),
                &bivariate_multiply(
                    &system.angular_tangent.radical,
                    &system.angular_tangent.radical,
                ),
            )
        };
        let boundary_incidence =
            bivariate_multiply(&system.selected_half_plane, &angular_incidence);
        let roots = match policy.strict_predicate_pass(|| {
            selected_fiber_root_intervals_in_interval(
                &boundary_incidence,
                BezierParameter2::Algebraic(center_parameter.clone()),
                lower,
                upper,
                policy,
            )
        })? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        #[derive(Clone)]
        struct Boundary {
            parameter: BezierAlgebraicSelectedFiberParameter2,
            represented_endpoint: bool,
        }

        let authority = BezierAlgebraicSelectedFiberAuthority2::new(
            boundary_incidence,
            center_parameter.clone(),
            policy,
        );
        let exact_boundary = |value: Real| Boundary {
            parameter: BezierAlgebraicSelectedFiberAuthority2::exact_parameter(
                center_parameter.clone(),
                value,
                policy,
            ),
            represented_endpoint: true,
        };
        let mut boundaries = Vec::with_capacity(roots.len() + 2);
        boundaries.push(exact_boundary(lower.clone()));
        boundaries.extend(roots.into_iter().map(|root| Boundary {
            parameter: authority.parameter(root),
            represented_endpoint: false,
        }));
        boundaries.push(exact_boundary(upper.clone()));
        for index in 1..boundaries.len() {
            let mut cursor = index;
            while cursor > 0 {
                let order = match boundaries[cursor]
                    .parameter
                    .cmp_by_refinement(&boundaries[cursor - 1].parameter, policy)?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if order != std::cmp::Ordering::Less {
                    break;
                }
                boundaries.swap(cursor, cursor - 1);
                cursor -= 1;
            }
        }
        let mut distinct: Vec<Boundary> = Vec::with_capacity(boundaries.len());
        for boundary in boundaries {
            if let Some(previous) = distinct.last_mut() {
                match previous
                    .parameter
                    .cmp_by_refinement(&boundary.parameter, policy)?
                {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        if boundary.represented_endpoint {
                            *previous = boundary;
                        }
                        continue;
                    }
                    Classification::Decided(std::cmp::Ordering::Less) => {}
                    Classification::Decided(std::cmp::Ordering::Greater) => {
                        return Err(CurveError::Topology(
                            "selected rational-circle component boundaries were not ordered".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            distinct.push(boundary);
        }
        let boundaries = distinct;
        let center = BezierParameter2::Algebraic(center_parameter);
        let cusp_location = |boundary: &Boundary| {
            let selected = match boundary
                .parameter
                .predicate_sign(&system.selected_half_plane, policy)?
            {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location = match selected {
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match boundary.parameter.radical_sum_sign(
                    &system.diameter,
                    &system.speed_squared,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "positive selected circle had an indeterminate component endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                RealSign::Negative => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            };
            Ok(Classification::Decided(location))
        };

        let expected_same_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let mut overlaps = Vec::new();
        let mut covered_boundaries = vec![false; boundaries.len()];
        for (index, pair) in boundaries.windows(2).enumerate() {
            let sample = match pair[0]
                .parameter
                .strict_scalar_between_ordered(&pair[1].parameter, policy)?
            {
                Classification::Decided(sample) => BezierParameter2::Exact(sample),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let selected = match signed_bivariate_at_parameter_pair(
                &system.selected_half_plane,
                &center,
                &sample,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected == RealSign::Negative {
                continue;
            }
            if selected == RealSign::Zero {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            let angular = match algebraic_cusp_independent_radical_sum_sign(
                &system.angular_tangent,
                &system.speed_squared,
                &center,
                &sample,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if angular == RealSign::Zero {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            let first = match cusp_location(&pair[0])? {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match cusp_location(&pair[1])? {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let orientation = if angular == expected_same_sign {
                CurveOverlapOrientation2::Same
            } else {
                CurveOverlapOrientation2::Reversed
            };
            // Complete boundary isolation makes the selected-half and angular
            // signs constant on this regular cell. Its nonzero angular sign
            // therefore proves strict cusp order without comparing mapped roots.
            // Defer the shared map until every publication predicate has run.
            covered_boundaries[index] = true;
            covered_boundaries[index + 1] = true;
            overlaps.push((pair, first, second, orientation));
        }

        let mut contacts = Vec::new();
        for (index, boundary) in boundaries.iter().enumerate() {
            if covered_boundaries[index] {
                continue;
            }
            let location = match cusp_location(boundary)? {
                Classification::Decided(location) => location,
                Classification::Uncertain(UncertaintyReason::Boundary) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if location != BezierAlgebraicCuspSemicircleContactLocation2::Interior {
                contacts.push((boundary.parameter.clone(), location));
            }
        }
        let map = BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2 {
            data: Arc::new(
                BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    isolated_incidence: None,
                    diameter: system.diameter,
                    radius_squared_denominator: system.radius_squared_denominator,
                    speed_squared: system.speed_squared,
                    tangent_cross: system.tangent_cross,
                    angular_tangent: system.angular_tangent,
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                },
            ),
        };
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                contacts: contacts
                    .into_iter()
                    .map(|(parameter, location)| map.contact(parameter, location, RealSign::Zero))
                    .collect(),
                overlaps: overlaps
                    .into_iter()
                    .map(|(pair, first, second, orientation)| {
                        let first =
                            map.mapped_parameter(pair[0].parameter.clone(), first, RealSign::Zero);
                        let second =
                            map.mapped_parameter(pair[1].parameter.clone(), second, RealSign::Zero);
                        let (cusp_start, cusp_end) =
                            if orientation == CurveOverlapOrientation2::Same {
                                (first, second)
                            } else {
                                (second, first)
                            };
                        BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2 {
                            other_start: pair[0].parameter.clone(),
                            other_end: pair[1].parameter.clone(),
                            cusp_start,
                            cusp_end,
                            orientation,
                            map: map.clone(),
                        }
                    })
                    .collect(),
            },
        ))
    }

    /// Replays a rational carrier whose complete image lies on a pair-radial
    /// supporting circle. The two authored source roots remain independent;
    /// selected-diameter and angular-stationary roots are projected only on
    /// the rational carrier axis and replayed against the retained pair
    /// radical before becoming component boundaries.
    pub(in crate::bezier_offset) fn selected_radial_replay_rational_circle_component(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: BezierSelectedRadialCircleRationalSystem2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        let BezierSelectedRadialCircleRationalSystem2 {
            pair_map,
            branch,
            discriminant,
            selected_half_plane,
            diameter,
            radius_squared_denominator,
            tangent_cross,
            angular_tangent,
            ..
        } = system;
        let Some([first_parameter, second_parameter]) = pair_map.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let (first_parameter, second_parameter) = (&first_parameter, &second_parameter);
        let selected_roots = match selected_pair_square_root_expression_third_axis_parameters(
            &selected_half_plane,
            &discriminant,
            first_parameter,
            second_parameter,
            branch,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_roots = match selected_pair_square_root_expression_third_axis_parameters(
            &angular_tangent,
            &discriminant,
            first_parameter,
            second_parameter,
            branch,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut boundaries = Vec::with_capacity(selected_roots.len() + angular_roots.len() + 2);
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(lower.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        boundaries.extend(selected_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: true,
            }
        }));
        boundaries.extend(angular_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: false,
            }
        }));
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(upper.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });

        let parameter_map = BezierAlgebraicCuspSemicircleRationalParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                semicircle: self.clone(),
                curve: other.clone(),
                system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial {
                    pair_map: pair_map.clone(),
                    branch,
                    discriminant: discriminant.clone(),
                    diameter: diameter.clone(),
                    radius_squared_denominator,
                    tangent_cross: tangent_cross.clone(),
                    angular_tangent: angular_tangent.clone(),
                },
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        };
        let expression_sign = |expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                               parameter: &BezierParameter2| {
            algebraic_cusp_trivariate_square_root_sum_sign(
                expression,
                &discriminant,
                first_parameter,
                second_parameter,
                parameter,
                branch,
                policy,
            )
        };
        self.publish_partitioned_rational_circle_component(
            other,
            parameter_map,
            boundaries,
            policy,
            |boundary| {
                if boundary.selected_relation {
                    Ok(Classification::Decided(RealSign::Zero))
                } else {
                    expression_sign(&selected_half_plane, &boundary.parameter)
                }
            },
            |boundary| expression_sign(&diameter, &boundary.parameter),
            |parameter| expression_sign(&angular_tangent, parameter),
        )
    }

    /// Publishes a rational carrier whose complete image lies on an
    /// arbitrary-depth selected-radial supporting circle. The target
    /// parameter is the only projected axis; every candidate is replayed in
    /// the authored recursive quadratic tower before it becomes a component
    /// boundary.
    pub(in crate::bezier_offset) fn recursive_selected_radial_replay_rational_circle_component(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: Arc<BezierRecursiveCircleTargetSystem2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        if !system.unit_target_speed {
            return Err(CurveError::Topology(
                "a recursive rational-circle component retained analytic target speed".into(),
            ));
        }
        if let Some(frame) = self.exact_point_component_frame(policy)? {
            #[cfg(feature = "dispatch-trace")]
            {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-rational-kernel",
                    "recursive-quadratic-component",
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-rational-kernel",
                    "recursive-rational-frame-component",
                );
            }
            return self.replay_rational_circle_component_with_exact_frame(
                other,
                range,
                &frame,
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented {
                    frame: frame.clone(),
                },
                policy,
            );
        }
        let selected_roots = match system.retained_expression_parameters(
            &system.selected_half_plane,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_roots = match system.retained_expression_parameters(
            &system.tangent_dot_source,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut boundaries = Vec::with_capacity(selected_roots.len() + angular_roots.len() + 2);
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(lower.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        boundaries.extend(selected_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: true,
            }
        }));
        boundaries.extend(angular_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: false,
            }
        }));
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(upper.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        let parameter_map = BezierAlgebraicCuspSemicircleRationalParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                semicircle: self.clone(),
                curve: other.clone(),
                system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive {
                    system: Arc::clone(&system),
                },
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-rational-kernel",
            "recursive-quadratic-component",
        );
        self.publish_partitioned_rational_circle_component(
            other,
            parameter_map,
            boundaries,
            policy,
            |boundary| {
                if boundary.selected_relation {
                    Ok(Classification::Decided(RealSign::Zero))
                } else {
                    system.expression_sign_at_parameter(
                        &system.selected_half_plane,
                        &boundary.parameter,
                        policy,
                    )
                }
            },
            |boundary| {
                system.expression_sign_at_parameter(&system.diameter, &boundary.parameter, policy)
            },
            |parameter| {
                // The source dot already contains the circle's traversal
                // sign. Component orientation needs cross(Q-C,Q') itself;
                // the publisher applies clockwise/counterclockwise later.
                Ok(system
                    .expression_sign_at_parameter(&system.tangent_dot_source, parameter, policy)?
                    .map(|sign| {
                        if self.is_clockwise() {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }
                    }))
            },
        )
    }

    /// On a certified target circle, subtracting its circle equation from
    /// this circle's equation leaves their radical axis. Dividing out the
    /// already-certified positive absolute source weight halves the parameter
    /// degree and preserves the sign of the original circle residual.
    /// In particular, an irrational-weight conic retains a quadratic tangent
    /// equation instead of a quartic whose repeated root needs another
    /// coefficient-field projection.
    pub(in crate::bezier_offset) fn recursive_rational_circle_incidence_polynomial(
        &self,
        other: &RationalBezier2,
        frame: &BezierRecursiveCircleFrame2,
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<BezierRecursiveQuadraticValue2>>> {
        let support = match policy.strict_predicate_pass(|| {
            crate::arc_bezier::rational_bezier_circular_arc(other, policy)
        })? {
            Classification::Decided(Some(support)) => support,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let field = &frame.field;
        let source = other.homogeneous_power_basis()?;
        let weight_sign = match policy
            .strict_predicate_pass(|| crate::classify::real_sign(&source.weight[0], policy))
        {
            Some(RealSign::Positive) => Real::one(),
            Some(RealSign::Negative) => Real::from(-1_i8),
            Some(RealSign::Zero) | None => return Ok(None),
        };
        Ok((|| {
            let two = field.constant(Real::from(2))?;
            let denominator = &frame.center.denominator;
            let denominator_squared = denominator.multiply(denominator)?;
            let x_factor = denominator
                .multiply(&field.constant(support.center().x().clone())?)?
                .subtract(&frame.center.x)?
                .multiply(denominator)?
                .multiply(&two)?;
            let y_factor = denominator
                .multiply(&field.constant(support.center().y().clone())?)?
                .subtract(&frame.center.y)?
                .multiply(denominator)?
                .multiply(&two)?;
            let constant = frame
                .center
                .x
                .multiply(&frame.center.x)?
                .add(&frame.center.y.multiply(&frame.center.y)?)?
                .add(&denominator_squared.multiply(&field.constant(
                    support.radius_squared()
                        - support.center().x() * support.center().x()
                        - support.center().y() * support.center().y()
                        - self.radial_distance() * self.radial_distance(),
                )?)?)?;
            let x = recursive_quadratic_real_polynomial(field, &source.x_numerator)?;
            let y = recursive_quadratic_real_polynomial(field, &source.y_numerator)?;
            let weight = recursive_quadratic_real_polynomial(field, &source.weight)?;
            let incidence = recursive_quadratic_polynomial_combine(
                &recursive_quadratic_polynomial_combine(
                    &recursive_quadratic_polynomial_scale(&x, &x_factor)?,
                    &recursive_quadratic_polynomial_scale(&y, &y_factor)?,
                    false,
                )?,
                &recursive_quadratic_polynomial_scale(&weight, &constant)?,
                false,
            )?;
            recursive_quadratic_polynomial_scale_real(&incidence, &weight_sign)
        })())
    }

    /// Proves that the supporting circles have only one common point.
    /// A retained tangency at their shared boundary endpoint suffices when
    /// unequal radii prove distinct supports. Otherwise a concentric parent
    /// may certify the center distance and the circle discriminant directly.
    /// Neither proof needs to rediscover a double root in expanded coordinates.
    pub(crate) fn certifies_unique_rational_circle_contact(
        &self,
        other: &RationalBezier2,
        certified_endpoint_tangency: bool,
        policy: &CurveContext,
    ) -> CurveResult<bool> {
        // A proper quadratic circle chart visits each finite point at most
        // once. Higher-degree parameterizations can revisit a tangent point
        // away from their shared boundary endpoint and need full pair replay.
        if !matches!(
            other.quadratic_homogeneous_controls(&policy.strict_counterpart())?,
            Classification::Decided(Some(_))
        ) {
            return Ok(false);
        }
        let Classification::Decided(Some(target)) = policy.strict_predicate_pass(|| {
            crate::arc_bezier::rational_bezier_circular_arc(other, policy)
        })?
        else {
            return Ok(false);
        };
        if certified_endpoint_tangency {
            let radius_difference =
                self.radial_distance() * self.radial_distance() - target.radius_squared();
            if matches!(
                policy.strict_predicate_pass(|| real_sign(&radius_difference, policy)),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                return Ok(true);
            }
        }
        let Some(frame) = self.data.frame.selected_radial() else {
            return Ok(false);
        };
        let parent = frame.center_parameter.semicircle_carrier();
        let distance_squared = parent.radial_distance() * parent.radial_distance();
        if policy.strict_predicate_pass(|| real_sign(&distance_squared, policy))
            != Some(RealSign::Positive)
        {
            return Ok(false);
        }
        let radical_axis = &distance_squared + target.radius_squared()
            - self.radial_distance() * self.radial_distance();
        let discriminant = Real::from(4) * &distance_squared * target.radius_squared()
            - &radical_axis * &radical_axis;
        if policy.strict_predicate_pass(|| real_sign(&discriminant, policy)) != Some(RealSign::Zero)
        {
            return Ok(false);
        }
        let Classification::Decided(center) = parent.center_point_evidence(policy)? else {
            return Ok(false);
        };
        Ok(policy.strict_predicate_pass(|| {
            center.same_point(&CurvePoint2::from(target.center().clone()), policy)
        }) == Classification::Decided(true))
    }

    pub(in crate::bezier_offset) fn recursive_selected_radial_rational_intersections_internal(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        system: Arc<BezierRecursiveCircleTargetSystem2>,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        if !system.unit_target_speed {
            return Err(CurveError::Topology(
                "a recursive circle/rational intersection retained analytic target speed".into(),
            ));
        }
        let circle_polynomial = system
            .expression_polynomial(&system.circle)
            .ok_or_else(|| {
                CurveError::Topology(
                    "a recursive circle/rational solve exceeded its coefficient field".into(),
                )
            })?;
        match recursive_quadratic_polynomial_is_identically_zero(&circle_polynomial, policy)? {
            Classification::Decided(true) => {
                return Ok(self
                    .recursive_selected_radial_replay_rational_circle_component(
                        other, range, system, policy,
                    )?
                    .map(|intersections| (intersections, None)));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let candidates = match recursive_projective_polynomial_parameters(
            &system.field,
            circle_polynomial,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let location = match policy.strict_predicate_pass(|| {
                system.contact_location_at_region_parameter(&candidate, policy)
            })? {
                Classification::Decided(Some(location)) => location,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_cross_sign = match system.polynomial_sign_at_region_parameter(
                &system.tangent_cross_source,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let tangent_dot_sign = if other.degree() == 1 {
                match policy.strict_predicate_pass(|| {
                    system.expression_sign_at_region_parameter(
                        &system.tangent_dot_source,
                        &candidate,
                        policy,
                    )
                })? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => None,
                }
            } else {
                None
            };
            let point =
                match rational_point_evidence_at_region_parameter(other, &candidate, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: candidate,
                point,
                tangent_cross_sign,
                tangent_dot_sign,
                location,
            });
        }
        let parameter_map = if retain_parameter_map
            && contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive {
                        system,
                    },
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        } else {
            None
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-rational-kernel",
            "recursive-quadratic",
        );
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        )))
    }

    pub(in crate::bezier_offset) fn selected_parallel_normal_rational_intersections_internal(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        candidate: Option<BezierSelectedParallelNormalRationalTangentCandidate2>,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        let system = match self.selected_parallel_normal_rational_system(other, range, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let center_parameter = self.selected_frame_parameter().ok_or_else(|| {
            CurveError::Topology(
                "the selected parallel-normal rational kernel lost its center parameter".into(),
            )
        })?;
        let center_parameter =
            match promote_curve_region_bezier_parameter(&center_parameter, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };

        let tangent_candidates = candidate.is_some();
        let tangent_certificate = match candidate {
            Some(candidate) => {
                let incidence = match reduce_bivariate_in_selected_parameter(
                    candidate.incidence,
                    &center_parameter,
                    policy,
                )? {
                    Classification::Decided(incidence) => incidence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radial_alignment = match reduce_radical_expression_in_selected_parameter(
                    candidate.radial_alignment,
                    &center_parameter,
                    policy,
                )? {
                    Classification::Decided(expression) => expression,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radial_dot = match reduce_radical_expression_in_selected_parameter(
                    candidate.radial_dot,
                    &center_parameter,
                    policy,
                )? {
                    Classification::Decided(expression) => expression,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let speed_squared = match reduce_bivariate_in_selected_parameter(
                    candidate.speed_squared,
                    &center_parameter,
                    policy,
                )? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Some(BezierSelectedParallelNormalRationalTangentCandidate2 {
                    incidence,
                    radial_alignment,
                    radial_dot,
                    speed_squared,
                    expected_radial_dot_sign: candidate.expected_radial_dot_sign,
                })
            }
            None => None,
        };
        // Circle construction can contribute a repeated source diagonal.
        // Keep one copy, preserving every contact, and isolate the same zero
        // set without forcing local-field square-free reconstruction. Contact
        // tangency still comes from the original geometric tangent predicate.
        let incidence = deflate_bivariate_parameter_diagonal_exact(&system.incidence)
            .map(|residual| {
                bivariate_multiply(
                    &residual,
                    &BivariatePolynomial::new(vec![
                        vec![Real::zero(), Real::from(-1_i8)],
                        vec![Real::one()],
                    ]),
                )
            })
            .unwrap_or(system.incidence);
        let incidence =
            match reduce_bivariate_in_selected_parameter(incidence, &center_parameter, policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let circle = match reduce_radical_expression_in_selected_parameter(
            system.circle,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let selected_half_plane = match reduce_bivariate_in_selected_parameter(
            system.selected_half_plane,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let diameter = match reduce_radical_expression_in_selected_parameter(
            system.diameter,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius_squared_denominator = match reduce_bivariate_in_selected_parameter(
            system.radius_squared_denominator,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let speed_squared = match reduce_bivariate_in_selected_parameter(
            system.speed_squared,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross = match reduce_radical_expression_in_selected_parameter(
            system.tangent_cross,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_tangent = match reduce_radical_expression_in_selected_parameter(
            system.angular_tangent,
            &center_parameter,
            policy,
        )? {
            Classification::Decided(expression) => expression,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let BezierParameter2::Algebraic(selected_center) = center_parameter.clone() {
            return Ok(self
                .selected_parallel_normal_rational_selected_fiber_intersections(
                    other,
                    range,
                    BezierSelectedParallelNormalCircleRationalSystem2 {
                        incidence,
                        circle,
                        selected_half_plane,
                        diameter,
                        radius_squared_denominator,
                        speed_squared,
                        tangent_cross,
                        angular_tangent,
                    },
                    selected_center,
                    tangent_certificate,
                    None,
                    policy,
                )?
                .map(|intersections| (intersections, None)));
        }
        let root_incidence = tangent_certificate
            .as_ref()
            .map_or(&incidence, |candidate| &candidate.incidence);
        let candidates = match selected_parameter_fiber_parameters(
            root_incidence,
            &center_parameter,
            MAX_FIXED_DISTANCE_RESULTANT_DEGREE,
            MAX_FIXED_DISTANCE_QUOTIENT_DEGREE,
            range,
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero)
                if !tangent_candidates =>
            {
                // A represented center parameter can make the entire target
                // circle incident, just as a selected algebraic parameter can.
                // Its exact frame removes the squared source-speed equation,
                // including a possibly vanishing conjugate factor, and reuses
                // the common finite-contact/overlap replay authority.
                let Some(frame) = self.exact_point_component_frame(policy)? else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                return self.represented_rational_intersections_internal(
                    other,
                    range,
                    retain_parameter_map,
                    Some(frame),
                    "exact-parallel-normal-component",
                    policy,
                );
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided((
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                    None,
                )));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let radical_sign = |expression: &BezierAlgebraicCuspTwoTermExpression2,
                            expression_speed_squared: &BivariatePolynomial,
                            candidate: &BezierParameter2| {
            algebraic_cusp_correlated_radical_sum_sign(
                root_incidence,
                expression,
                expression_speed_squared,
                &center_parameter,
                candidate,
                policy,
            )
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            if let Some(tangent) = &tangent_certificate {
                match radical_sign(
                    &tangent.radial_alignment,
                    &tangent.speed_squared,
                    &candidate,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match radical_sign(&tangent.radial_dot, &tangent.speed_squared, &candidate)? {
                    Classification::Decided(sign) if sign == tangent.expected_radial_dot_sign => {}
                    Classification::Decided(_) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                match radical_sign(&circle, &speed_squared, &candidate)? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let selected_sign = match algebraic_selected_correlated_predicate_sign(
                root_incidence,
                &selected_half_plane,
                &center_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected_sign == RealSign::Negative {
                continue;
            }
            let location = if selected_sign == RealSign::Zero {
                match radical_sign(&diameter, &speed_squared, &candidate)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero general selected circle had an indeterminate endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = if tangent_candidates {
                RealSign::Zero
            } else {
                match radical_sign(&tangent_cross, &speed_squared, &candidate)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let point = match rational_point_evidence_at_parameter(other, &candidate, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(candidate),
                point,
                tangent_cross_sign,
                tangent_dot_sign: None,
                location,
            });
        }
        let parameter_map = if retain_parameter_map
            && contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                        cusp_parameter: center_parameter,
                        incidence: root_incidence.clone(),
                        diameter: BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
                            coordinate: diameter,
                            speed_squared,
                        },
                        radius_squared_denominator,
                    },
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        } else {
            None
        };
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        )))
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_frame_source(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierChordNormalProjectiveFrameSource2>>> {
        let Some(frame) = self.data.frame.chord_normal() else {
            return Ok(Classification::Decided(None));
        };
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::Topology(
                "a chord-normal projective system crossed predicate policies".into(),
            ));
        }
        let CurvePoint2(CurvePointData2::AlgebraicChordPair(center)) = &frame.center else {
            return Ok(Classification::Decided(None));
        };
        if !center.accepts_policy(policy) {
            return Err(CurveError::Topology(
                "a chord-pair center crossed predicate policies".into(),
            ));
        }
        let Some(first_support) =
            chord_parallel_support_source(center.data.first.retained_support(), policy)?
        else {
            return Ok(Classification::Decided(None));
        };
        let Some(second_support) =
            chord_parallel_support_source(center.data.second.retained_support(), policy)?
        else {
            return Ok(Classification::Decided(None));
        };
        let anchor_speed = if frame.anchor.shares_retained_support(&first_support.source) {
            0_usize
        } else if frame.anchor.shares_retained_support(&second_support.source) {
            1_usize
        } else {
            // A third independent normalization radical is mathematically
            // valid but belongs to the rank-independent fallback.  Ordinary
            // chord/chord fillets share their anchor with one center support.
            return Ok(Classification::Decided(None));
        };
        let mut coordinates = Vec::with_capacity(12);
        for chord in [&first_support.source, &second_support.source, &frame.anchor] {
            for endpoint in [chord.start(), chord.end()] {
                match represented_point_evidence_coordinates(endpoint, policy)? {
                    Classification::Decided(point) => coordinates.extend(point),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let coordinates: [AlgebraicRootRepresentation; 12] = coordinates
            .try_into()
            .expect("a chord-normal frame retains twelve endpoint coordinates");
        Ok(Classification::Decided(Some(
            BezierChordNormalProjectiveFrameSource2 {
                first_support,
                second_support,
                anchor_speed,
                coordinates,
            },
        )))
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_rational_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierChordNormalDenseIntersectionSystem2>>> {
        let frame_source = match self.chord_normal_projective_frame_source(policy)? {
            Classification::Decided(Some(frame_source)) => frame_source,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target_weight_sign = match other.denominator_sign(range) {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite chord-normal rational candidate had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target = other.homogeneous_power_basis()?;
        let target_scale = if target_weight_sign == RealSign::Negative {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let target_x = polynomial_scale(&target.x_numerator, &target_scale);
        let target_y = polynomial_scale(&target.y_numerator, &target_scale);
        let target_weight = polynomial_scale(&target.weight, &target_scale);
        let target_tangent_x = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_x), &target_weight),
            &polynomial_multiply(&target_x, &polynomial_derivative(&target_weight)),
        );
        let target_tangent_y = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_y), &target_weight),
            &polynomial_multiply(&target_y, &polynomial_derivative(&target_weight)),
        );
        self.chord_normal_dense_target_system(frame_source, Vec::new(), true, |_, rank| {
            let target_axis = rank.checked_sub(1)?;
            let axis = |coefficients: &[Real]| {
                DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
            };
            Some(BezierChordNormalDenseTarget2 {
                x: axis(&target_x)?,
                y: axis(&target_y)?,
                weight: axis(&target_weight)?,
                tangent_x: axis(&target_tangent_x)?,
                tangent_y: axis(&target_tangent_y)?,
            })
        })
    }

    pub(in crate::bezier_offset) fn chord_normal_dense_target_system<F>(
        &self,
        frame_source: BezierChordNormalProjectiveFrameSource2,
        target_coordinates: Vec<AlgebraicRootRepresentation>,
        rational_target: bool,
        target: F,
    ) -> CurveResult<Classification<Option<BezierChordNormalDenseIntersectionSystem2>>>
    where
        F: FnOnce(Vec<DenseTensorPolynomial>, usize) -> Option<BezierChordNormalDenseTarget2>,
    {
        let BezierChordNormalProjectiveFrameSource2 {
            first_support,
            second_support,
            anchor_speed,
            coordinates,
        } = frame_source;
        let mut represented_coordinates = Vec::from(coordinates);
        represented_coordinates.extend(target_coordinates);
        let Some((sources, mut coordinates)) =
            represented_affine_tensor_basis(&represented_coordinates)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let target_coordinates = coordinates.split_off(12);
        let [
            first_start_x,
            first_start_y,
            first_end_x,
            first_end_y,
            second_start_x,
            second_start_y,
            second_end_x,
            second_end_y,
            anchor_start_x,
            anchor_start_y,
            anchor_end_x,
            anchor_end_y,
        ]: [DenseTensorPolynomial; 12] = coordinates
            .try_into()
            .expect("a dense chord-normal frame retains twelve coordinates");
        let Some(BezierChordNormalDenseTarget2 {
            x,
            y,
            weight,
            tangent_x,
            tangent_y,
        }) = target(target_coordinates, sources.len() + 1)
        else {
            return Ok(Classification::Decided(None));
        };
        let rank = sources.len() + 1;
        let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);

        let system = (|| {
            let first_dx = reduce(first_end_x.subtract(&first_start_x)?)?;
            let first_dy = reduce(first_end_y.subtract(&first_start_y)?)?;
            let second_dx = reduce(second_end_x.subtract(&second_start_x)?)?;
            let second_dy = reduce(second_end_y.subtract(&second_start_y)?)?;
            let anchor_dx = reduce(anchor_end_x.subtract(&anchor_start_x)?)?;
            let anchor_dy = reduce(anchor_end_y.subtract(&anchor_start_y)?)?;
            let first_speed_squared = reduce(
                first_dx
                    .multiply(&first_dx)?
                    .add(&first_dy.multiply(&first_dy)?)?,
            )?;
            let second_speed_squared = reduce(
                second_dx
                    .multiply(&second_dx)?
                    .add(&second_dy.multiply(&second_dy)?)?,
            )?;

            let parallel_c = |x: &DenseTensorPolynomial,
                              y: &DenseTensorPolynomial,
                              dx: &DenseTensorPolynomial,
                              dy: &DenseTensorPolynomial,
                              support: &BezierChordParallelSupportSource2,
                              speed_index: usize| {
                let rational = reduce(
                    dy.multiply(x)?
                        .subtract(&dx.multiply(y)?)?
                        .add(&dy.scale(&support.translation_x)?)?
                        .subtract(&dx.scale(&support.translation_y)?)?,
                )?;
                let mut expression = BezierDenseTwoSquareRootExpression2::from_rational(rational)?;
                if support.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal {
                    let coefficient = DenseTensorPolynomial::from_axis_polynomial(
                        rank,
                        0,
                        &[-support.distance.clone()],
                    )?;
                    let radical = if speed_index == 0 {
                        BezierDenseTwoSquareRootExpression2::from_first_radical(coefficient)?
                    } else {
                        BezierDenseTwoSquareRootExpression2::from_second_radical(coefficient)?
                    };
                    expression = expression.add(&radical)?;
                }
                expression.reduced(&sources)
            };
            let first_c = parallel_c(
                &first_start_x,
                &first_start_y,
                &first_dx,
                &first_dy,
                &first_support,
                0,
            )?;
            let second_c = parallel_c(
                &second_start_x,
                &second_start_y,
                &second_dx,
                &second_dy,
                &second_support,
                1,
            )?;
            let denominator = reduce(
                first_dy
                    .scale(&Real::from(-1_i8))?
                    .multiply(&second_dx)?
                    .subtract(&second_dy.scale(&Real::from(-1_i8))?.multiply(&first_dx)?)?,
            )?;
            let center_x = first_c
                .multiply_rational(&second_dx)?
                .scale(&Real::from(-1_i8))?
                .add(&second_c.multiply_rational(&first_dx)?)?
                .reduced(&sources)?;
            let center_y = first_c
                .multiply_rational(&second_dy.scale(&Real::from(-1_i8))?)?
                .subtract(&second_c.multiply_rational(&first_dy.scale(&Real::from(-1_i8))?)?)?
                .reduced(&sources)?;

            let common_denominator = reduce(weight.multiply(&denominator)?)?;
            let point_x = BezierDenseTwoSquareRootExpression2::from_rational(reduce(
                x.multiply(&denominator)?,
            )?)?
            .reduced(&sources)?;
            let point_y = BezierDenseTwoSquareRootExpression2::from_rational(reduce(
                y.multiply(&denominator)?,
            )?)?
            .reduced(&sources)?;
            let center_x = center_x.multiply_rational(&weight)?.reduced(&sources)?;
            let center_y = center_y.multiply_rational(&weight)?.reduced(&sources)?;
            let radial_x = point_x.subtract(&center_x)?.reduced(&sources)?;
            let radial_y = point_y.subtract(&center_y)?.reduced(&sources)?;
            let radius = self.radial_distance().clone();
            let radius_squared = &radius * &radius;
            let incidence = radial_x
                .square(&first_speed_squared, &second_speed_squared)?
                .add(&radial_y.square(&first_speed_squared, &second_speed_squared)?)?
                .subtract(&BezierDenseTwoSquareRootExpression2::from_rational(
                    reduce(
                        common_denominator
                            .multiply(&common_denominator)?
                            .scale(&radius_squared)?,
                    )?,
                )?)?
                .reduced(&sources)?;

            let dot_anchor = radial_x
                .multiply_rational(&anchor_dx)?
                .add(&radial_y.multiply_rational(&anchor_dy)?)?;
            let cross_anchor = radial_y
                .multiply_rational(&anchor_dx)?
                .subtract(&radial_x.multiply_rational(&anchor_dy)?)?;
            let selected_half_plane = dot_anchor
                .multiply_rational(&denominator)?
                .scale(&(-(&radius * self.turn_sign())))?
                .reduced(&sources)?;
            let diameter = cross_anchor
                .multiply_rational(&denominator)?
                .scale(&radius)?
                .reduced(&sources)?;
            let radius_scale = reduce(
                weight
                    .multiply(&denominator.multiply(&denominator)?)?
                    .scale(&radius_squared)?,
            )?;
            let radius_squared_denominator = if anchor_speed == 0 {
                BezierDenseTwoSquareRootExpression2::from_first_radical(radius_scale)?
            } else {
                BezierDenseTwoSquareRootExpression2::from_second_radical(radius_scale)?
            }
            .reduced(&sources)?;
            let tangent_dot = radial_x
                .multiply_rational(&tangent_x)?
                .add(&radial_y.multiply_rational(&tangent_y)?)?;
            let tangent_cross = tangent_dot
                .multiply_rational(&denominator)?
                .scale(&(-self.turn_sign()))?
                .reduced(&sources)?;
            let angular_tangent = Some(
                radial_x
                    .multiply_rational(&tangent_y)?
                    .subtract(&radial_y.multiply_rational(&tangent_x)?)?
                    .multiply_rational(&denominator)?
                    .reduced(&sources)?,
            );
            let source_representations = sources.clone();
            let map = Arc::new(BezierChordNormalDenseMapSystem2 {
                source_representations,
                first_speed_squared: reduce(first_speed_squared)?,
                second_speed_squared: reduce(second_speed_squared)?,
                diameter,
                radius_squared_denominator,
            });
            Some(BezierChordNormalDenseIntersectionSystem2 {
                map,
                incidence,
                selected_half_plane,
                tangent_cross,
                angular_tangent,
                geometry: (!rational_target).then_some(BezierChordNormalDenseTargetGeometry2 {
                    point_x,
                    point_y,
                    center_x,
                    center_y,
                    common_denominator,
                }),
            })
        })();
        Ok(Classification::Decided(system))
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_chord_system(
        &self,
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierChordNormalDenseIntersectionSystem2>>> {
        let frame_source = match self.chord_normal_projective_frame_source(policy)? {
            Classification::Decided(Some(frame_source)) => frame_source,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut target_coordinates = Vec::with_capacity(4);
        for endpoint in [chord.start(), chord.end()] {
            match represented_point_evidence_coordinates(endpoint, policy)? {
                Classification::Decided(point) => target_coordinates.extend(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        self.chord_normal_dense_target_system(
            frame_source,
            target_coordinates,
            false,
            |coordinates, rank| {
                let [start_x, start_y, end_x, end_y]: [DenseTensorPolynomial; 4] =
                    coordinates.try_into().ok()?;
                let target_axis = rank.checked_sub(1)?;
                let one_minus = DenseTensorPolynomial::from_axis_polynomial(
                    rank,
                    target_axis,
                    &[Real::one(), Real::from(-1_i8)],
                )?;
                let parameter = DenseTensorPolynomial::from_axis_polynomial(
                    rank,
                    target_axis,
                    &[Real::zero(), Real::one()],
                )?;
                let weight =
                    DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, &[Real::one()])?;
                let x = start_x
                    .multiply(&one_minus)?
                    .add(&end_x.multiply(&parameter)?)?;
                let y = start_y
                    .multiply(&one_minus)?
                    .add(&end_y.multiply(&parameter)?)?;
                let tangent_x = end_x.subtract(&start_x)?;
                let tangent_y = end_y.subtract(&start_y)?;
                Some(BezierChordNormalDenseTarget2 {
                    x,
                    y,
                    weight,
                    tangent_x,
                    tangent_y,
                })
            },
        )
    }

    pub(in crate::bezier_offset) fn represented_rational_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retained_frame: Option<BezierRepresentedSelectedRadialCircleFrame2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedCircleRationalSystem2>> {
        let frame = match retained_frame {
            Some(frame) => frame,
            None => match self.represented_circle_frame(policy)? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        let Some((sources, center)) = represented_affine_tensor_basis(&frame.center) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [center_x, center_y]: [DenseTensorPolynomial; 2] = center
            .try_into()
            .expect("a represented circle frame retains both center coordinates");
        let target_weight_sign = match other.denominator_sign(range) {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite represented-circle candidate had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target = other.homogeneous_power_basis()?;
        let target_scale = if target_weight_sign == RealSign::Negative {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let target_x = polynomial_scale(&target.x_numerator, &target_scale);
        let target_y = polynomial_scale(&target.y_numerator, &target_scale);
        let target_weight = polynomial_scale(&target.weight, &target_scale);
        let target_tangent_x = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_x), &target_weight),
            &polynomial_multiply(&target_x, &polynomial_derivative(&target_weight)),
        );
        let target_tangent_y = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_y), &target_weight),
            &polynomial_multiply(&target_y, &polynomial_derivative(&target_weight)),
        );
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let Some((incidence, tangent_cross)) = (|| {
            let x = axis(&target_x)?;
            let y = axis(&target_y)?;
            let weight = axis(&target_weight)?;
            let tangent_x = axis(&target_tangent_x)?;
            let tangent_y = axis(&target_tangent_y)?;
            let dx = x.subtract(&center_x.multiply(&weight)?)?;
            let dy = y.subtract(&center_y.multiply(&weight)?)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let incidence = dx
                .multiply(&dx)?
                .add(&dy.multiply(&dy)?)?
                .subtract(&weight.multiply(&weight)?.scale(&radius_squared)?)?;
            // cross(turn*J(Q-C), Q') = -turn*dot(Q-C, Q').
            let tangent_cross = dx
                .multiply(&tangent_x)?
                .add(&dy.multiply(&tangent_y)?)?
                .scale(&(-self.turn_sign()))?;
            Some((incidence, tangent_cross))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(
            BezierRepresentedCircleRationalSystem2 {
                frame,
                sources,
                incidence,
                tangent_cross,
            },
        ))
    }

    pub(in crate::bezier_offset) fn represented_rational_component_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        frame: &BezierRepresentedSelectedRadialCircleFrame2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRepresentedCircleRationalComponentSystem2>> {
        let represented = [
            frame.center[0].clone(),
            frame.center[1].clone(),
            frame.unit_radial[0].clone(),
            frame.unit_radial[1].clone(),
        ];
        let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [center_x, center_y, unit_x, unit_y]: [DenseTensorPolynomial; 4] = coordinates
            .try_into()
            .expect("a represented rational-circle component retains its full frame");
        let target_weight_sign = match other.denominator_sign(range) {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a finite represented circle component had a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let target = other.homogeneous_power_basis()?;
        let target_scale = if target_weight_sign == RealSign::Negative {
            Real::from(-1_i8)
        } else {
            Real::one()
        };
        let target_x = polynomial_scale(&target.x_numerator, &target_scale);
        let target_y = polynomial_scale(&target.y_numerator, &target_scale);
        let target_weight = polynomial_scale(&target.weight, &target_scale);
        let target_tangent_x = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_x), &target_weight),
            &polynomial_multiply(&target_x, &polynomial_derivative(&target_weight)),
        );
        let target_tangent_y = polynomial_subtract(
            &polynomial_multiply(&polynomial_derivative(&target_y), &target_weight),
            &polynomial_multiply(&target_y, &polynomial_derivative(&target_weight)),
        );
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let Some((selected_half_plane, diameter, radius_squared_denominator, angular_tangent)) =
            (|| {
                let x = axis(&target_x)?;
                let y = axis(&target_y)?;
                let weight = axis(&target_weight)?;
                let tangent_x = axis(&target_tangent_x)?;
                let tangent_y = axis(&target_tangent_y)?;
                let radial_x = x.subtract(&center_x.multiply(&weight)?)?;
                let radial_y = y.subtract(&center_y.multiply(&weight)?)?;
                let selected_scale = &frame.signed_radius * self.turn_sign();
                let selected_half_plane = unit_x
                    .multiply(&radial_y)?
                    .subtract(&unit_y.multiply(&radial_x)?)?
                    .scale(&selected_scale)?;
                let diameter = unit_x
                    .multiply(&radial_x)?
                    .add(&unit_y.multiply(&radial_y)?)?
                    .scale(&frame.signed_radius)?;
                let angular_tangent = radial_x
                    .multiply(&tangent_y)?
                    .subtract(&radial_y.multiply(&tangent_x)?)?;
                Some((
                    selected_half_plane,
                    diameter,
                    weight.scale(&(&frame.signed_radius * &frame.signed_radius))?,
                    angular_tangent,
                ))
            })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let quadratic_parameterization = match other.quadratic_homogeneous_controls(policy)? {
            Classification::Decided(quadratic) => quadratic,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let quadratic_conic_inverse = quadratic_parameterization
            .as_ref()
            .filter(|controls| {
                let sign = real_sign(&controls[0][2], &policy.strict_counterpart());
                matches!(sign, Some(RealSign::Positive | RealSign::Negative))
                    && controls
                        .iter()
                        .all(|control| real_sign(&control[2], &policy.strict_counterpart()) == sign)
            })
            .map(|quadratic| represented_quadratic_conic_inverse(frame, quadratic));
        let mut system = BezierRepresentedCircleRationalComponentSystem2 {
            sources,
            selected_half_plane,
            diameter,
            radius_squared_denominator,
            angular_tangent,
            quadratic_selected_parameters: None,
        };
        if let Some(quadratic_conic_inverse) = quadratic_conic_inverse.as_ref()
            && range.scalar_endpoints().is_some()
            && matches!(
                CurveParameterDomain2::new(&CurveParameterRange2::unit(), None)
                    .contains_finite_range(range, policy),
                Ok(Classification::Decided(true))
            )
        {
            // A selected boundary parameter is construction evidence.  Keep
            // its endpoint signs and projective inverse exact even when the
            // caller permits terminal APPROXIMATE_512 equality decisions.
            system.quadratic_selected_parameters = Some(
                match policy.strict_predicate_pass(|| {
                    system.quadratic_selected_parameters(quadratic_conic_inverse, range, policy)
                })? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            );
        }
        Ok(Classification::Decided(system))
    }

    /// Caches formal target equations, never a target-domain regularity proof.
    pub(in crate::bezier_offset) fn recursive_circle_parallel_system(
        &self,
        other: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<BezierRecursiveCircleTargetSystem2>>> {
        let cached = self
            .data
            .parallel_system_cache
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .as_ref()
            .filter(|cached| {
                cached.policy == *policy
                    && cached.permits_approximate_512 == policy.permits_approximate_512()
                    && cached
                        .target
                        .upgrade()
                        .is_some_and(|target| Arc::ptr_eq(&target, &other.data))
            })
            .map(|cached| Arc::clone(&cached.system));
        if let Some(cached) = cached {
            return Ok(Classification::Decided(cached));
        }
        let built = self.recursive_circle_target_system(other, false, true, policy)?;
        if let Classification::Decided(system) = &built {
            *self
                .data
                .parallel_system_cache
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner) =
                Some(BezierCircleParallelSystemCacheEntry2 {
                    target: Arc::downgrade(&other.data),
                    policy: *policy,
                    permits_approximate_512: policy.permits_approximate_512(),
                    system: Arc::clone(system),
                });
        }
        Ok(built)
    }

    pub(in crate::bezier_offset) fn recursive_selected_radial_rational_system(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<BezierRecursiveCircleTargetSystem2>>> {
        match other.denominator_sign(range) {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let target = BezierParallel2::from_source(
            BezierParallelSource2::Rational(other.clone()),
            Real::zero(),
        );
        self.recursive_circle_target_system(&target, true, false, policy)
    }

    pub(in crate::bezier_offset) fn recursive_circle_target_system(
        &self,
        other: &BezierParallel2,
        unit_target_speed: bool,
        project_incidence: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<BezierRecursiveCircleTargetSystem2>>> {
        let frame = match self.recursive_circle_frame_authority(policy)? {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source = other.source_power_basis()?;
        let differential = other.differential()?;
        let unit_weight = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit_weight);
        let field = frame.field.clone();
        let circular_incidence =
            if unit_target_speed && let BezierParallelSource2::Rational(curve) = other.source() {
                self.recursive_rational_circle_incidence_polynomial(curve, &frame, policy)?
            } else {
                None
            };
        #[cfg(feature = "dispatch-trace")]
        if circular_incidence.is_some() {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-rational-kernel",
                "circular-radical-axis",
            );
        }
        let Some((
            base,
            projection,
            circle,
            selected_half_plane,
            diameter,
            radius_squared_denominator,
            tangent_cross_source,
            tangent_dot_source,
            weight,
        )) = (|| {
            let real =
                |coefficients: &[Real]| recursive_quadratic_real_polynomial(&field, coefficients);
            let add = |first: &[BezierRecursiveQuadraticValue2],
                       second: &[BezierRecursiveQuadraticValue2]| {
                recursive_quadratic_polynomial_combine(first, second, false)
            };
            let subtract = |first: &[BezierRecursiveQuadraticValue2],
                            second: &[BezierRecursiveQuadraticValue2]| {
                recursive_quadratic_polynomial_combine(first, second, true)
            };
            let multiply = |first: &[BezierRecursiveQuadraticValue2],
                            second: &[BezierRecursiveQuadraticValue2]| {
                recursive_quadratic_polynomial_multiply(first, second)
            };
            let scale = |polynomial: &[BezierRecursiveQuadraticValue2], scale: &Real| {
                recursive_quadratic_polynomial_scale_real(polynomial, scale)
            };
            let scale_value =
                |polynomial: &[BezierRecursiveQuadraticValue2],
                 scale: &BezierRecursiveQuadraticValue2| {
                    recursive_quadratic_polynomial_scale(polynomial, scale)
                };

            let x = real(source.x_numerator)?;
            let y = real(source.y_numerator)?;
            let weight = real(source_weight)?;
            let tangent_x = real(&differential.tangent_x)?;
            let tangent_y = real(&differential.tangent_y)?;
            let speed_squared: Arc<[_]> = if unit_target_speed {
                real(&[Real::one()])?
            } else {
                add(
                    &multiply(&tangent_x, &tangent_x)?,
                    &multiply(&tangent_y, &tangent_y)?,
                )?
            }
            .into();
            let normal_x = scale(&tangent_y, &(-other.distance()))?;
            let normal_y = scale(&tangent_x, other.distance())?;
            // With C=(Cx/D,Cy/D), R=D(X,Y)-W(Cx,Cy), and
            // N=d*J(H), the exact circle equation is
            //
            //   sqrt(S) (|R|^2 + W^2 D^2(d^2-r^2))
            //     + 2 W D R.N = 0.
            //
            // Analytic parallels square this one procedural target-speed root
            // to enumerate candidates. A rational target is the exact
            // zero-distance specialization with procedural speed fixed to
            // one, so its unsquared circle polynomial is projected directly;
            // derivative zeros then remain geometric candidates only when
            // they actually satisfy the circle equation.
            let radial_x = subtract(
                &scale_value(&x, &frame.center.denominator)?,
                &scale_value(&weight, &frame.center.x)?,
            )?;
            let radial_y = subtract(
                &scale_value(&y, &frame.center.denominator)?,
                &scale_value(&weight, &frame.center.y)?,
            )?;
            let weight_denominator = scale_value(&weight, &frame.center.denominator)?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let distance_radius = other.distance() * other.distance() - &radius_squared;
            let circle_radical = match circular_incidence {
                Some(incidence) => incidence,
                None => add(
                    &add(
                        &multiply(&radial_x, &radial_x)?,
                        &multiply(&radial_y, &radial_y)?,
                    )?,
                    &scale(
                        &multiply(&weight_denominator, &weight_denominator)?,
                        &distance_radius,
                    )?,
                )?,
            };
            let radial_dot_normal = add(
                &multiply(&radial_x, &normal_x)?,
                &multiply(&radial_y, &normal_y)?,
            )?;
            let circle_rational = scale(
                &multiply(&weight_denominator, &radial_dot_normal)?,
                &Real::from(2_i8),
            )?;
            let circle = BezierRecursiveQuadraticParallelExpression2::new(
                circle_rational,
                circle_radical,
                speed_squared.clone(),
            );
            let (base, projection) = if project_incidence {
                let projected_coefficients = if unit_target_speed {
                    add(&circle.rational, &circle.radical)?
                } else {
                    circle.squared_magnitude_difference()?.to_vec()
                };
                let (base, projection) =
                    recursive_quadratic_polynomial_projection(projected_coefficients)?;
                (base, Some(projection))
            } else {
                (field.base_and_extension_path().0, None)
            };
            let (anchor_x, anchor_y, anchor_denominator) =
                frame.center.difference_numerators(&frame.support_center)?;
            let weight_squared = multiply(&weight, &weight)?;
            let weight_squared_denominator =
                scale_value(&weight_squared, &frame.center.denominator)?;
            let cross_anchor_radial = subtract(
                &scale_value(&radial_y, &anchor_x)?,
                &scale_value(&radial_x, &anchor_y)?,
            )?;
            let cross_anchor_normal = subtract(
                &scale_value(&normal_y, &anchor_x)?,
                &scale_value(&normal_x, &anchor_y)?,
            )?;
            let dot_anchor_radial = add(
                &scale_value(&radial_x, &anchor_x)?,
                &scale_value(&radial_y, &anchor_y)?,
            )?;
            let dot_anchor_normal = add(
                &scale_value(&normal_x, &anchor_x)?,
                &scale_value(&normal_y, &anchor_y)?,
            )?;
            let selected_scale =
                self.radial_distance() * &frame.normal_denominator * self.turn_sign();
            let diameter_scale = self.radial_distance() * &frame.normal_denominator;
            let selected_half_plane = BezierRecursiveQuadraticParallelExpression2::new(
                scale(
                    &multiply(&weight_squared_denominator, &cross_anchor_normal)?,
                    &selected_scale,
                )?,
                scale(&multiply(&weight, &cross_anchor_radial)?, &selected_scale)?,
                speed_squared.clone(),
            );
            let diameter = BezierRecursiveQuadraticParallelExpression2::new(
                scale(
                    &multiply(&weight_squared_denominator, &dot_anchor_normal)?,
                    &diameter_scale,
                )?,
                scale(&multiply(&weight, &dot_anchor_radial)?, &diameter_scale)?,
                speed_squared.clone(),
            );
            let radius_squared_scale =
                &radius_squared * &frame.normal_denominator * &frame.normal_denominator;
            let radius_squared_denominator = BezierRecursiveQuadraticParallelExpression2::new(
                real(&[Real::zero()])?,
                scale(
                    &scale_value(&weight_squared_denominator, &anchor_denominator)?,
                    &radius_squared_scale,
                )?,
                speed_squared.clone(),
            );

            let radial_dot_tangent = add(
                &multiply(&radial_x, &tangent_x)?,
                &multiply(&radial_y, &tangent_y)?,
            )?;
            let tangent_cross_source = scale(
                &multiply(&weight, &radial_dot_tangent)?,
                &(-self.turn_sign()),
            )?;
            let cross_radial_tangent = subtract(
                &multiply(&radial_x, &tangent_y)?,
                &multiply(&radial_y, &tangent_x)?,
            )?;
            let cross_normal_tangent = subtract(
                &multiply(&normal_x, &tangent_y)?,
                &multiply(&normal_y, &tangent_x)?,
            )?;
            let tangent_dot_source = BezierRecursiveQuadraticParallelExpression2::new(
                scale(
                    &multiply(&weight_squared_denominator, &cross_normal_tangent)?,
                    &self.turn_sign(),
                )?,
                scale(
                    &multiply(&weight, &cross_radial_tangent)?,
                    &self.turn_sign(),
                )?,
                speed_squared.clone(),
            );
            Some((
                base,
                projection,
                circle,
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross_source,
                tangent_dot_source,
                weight,
            ))
        })()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let direct_pair_fast_path = if unit_target_speed || !self.uses_selected_radial_frame() {
            None
        } else {
            match self.direct_pair_radial_parallel_fast_path(other, policy)? {
                Classification::Decided(fast_path) => Some(fast_path),
                Classification::Uncertain(_) => None,
            }
        };
        Ok(Classification::Decided(Arc::new(
            BezierRecursiveCircleTargetSystem2 {
                field,
                base,
                direct_pair_fast_path,
                unit_target_speed,
                projection,
                incidence_univariate: OnceLock::new(),
                represented_center_schedule: OnceLock::new(),
                circle,
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross_source,
                tangent_dot_source,
                weight,
            },
        )))
    }

    pub(in crate::bezier_offset) fn represented_center_parallel_system(
        &self,
        other: &BezierParallel2,
        represented: &[AlgebraicRootRepresentation],
    ) -> CurveResult<Classification<BezierRepresentedCenterParallelSystem2>> {
        if represented.len() < 2 {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let Some((sources, coordinates)) = represented_affine_tensor_basis(represented) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let center_x = coordinates[0].clone();
        let center_y = coordinates[1].clone();
        let source = other.source_power_basis()?;
        let differential = other.differential()?;
        let unit_weight = [Real::one()];
        let source_weight = source.weight.unwrap_or(&unit_weight);
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let Some(system) = (|| {
            let x = axis(source.x_numerator)?;
            let y = axis(source.y_numerator)?;
            let weight = axis(source_weight)?;
            let tangent_x = axis(&differential.tangent_x)?;
            let tangent_y = axis(&differential.tangent_y)?;
            let speed_squared = tangent_x
                .multiply(&tangent_x)?
                .add(&tangent_y.multiply(&tangent_y)?)?;
            let weight_squared = weight.multiply(&weight)?;
            let radial_x = x.subtract(&center_x.multiply(&weight)?)?;
            let radial_y = y.subtract(&center_y.multiply(&weight)?)?;
            let normal_x = tangent_y.scale(&(-other.distance()))?;
            let normal_y = tangent_x.scale(other.distance())?;
            let radius_squared = self.radial_distance() * self.radial_distance();
            let distance_radius = other.distance() * other.distance() - &radius_squared;
            let circle_radical = radial_x
                .multiply(&radial_x)?
                .add(&radial_y.multiply(&radial_y)?)?
                .add(&weight_squared.scale(&distance_radius)?)?;
            let circle_rational = radial_x
                .multiply(&normal_x)?
                .add(&radial_y.multiply(&normal_y)?)?
                .multiply(&weight)?
                .scale(&Real::from(2_i8))?;
            let projection = circle_rational.multiply(&circle_rational)?.subtract(
                &circle_radical
                    .multiply(&circle_radical)?
                    .multiply(&speed_squared)?,
            )?;
            Some(BezierRepresentedCenterParallelSystem2 {
                sources,
                coordinates,
                projection,
                circle: BezierRepresentedCircleParallelExpression2 {
                    rational: circle_rational,
                    radical: circle_radical,
                },
                radial_x,
                radial_y,
                normal_x,
                normal_y,
                tangent_x,
                tangent_y,
                speed_squared,
                weight,
                weight_squared,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(system))
    }

    pub(in crate::bezier_offset) fn represented_parallel_system(
        &self,
        other: &BezierParallel2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<BezierRepresentedCircleParallelSystem2>>> {
        let frame = match self.represented_circle_frame(policy)? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let represented = [
            frame.center[0].clone(),
            frame.center[1].clone(),
            frame.unit_radial[0].clone(),
            frame.unit_radial[1].clone(),
        ];
        let common = match self.represented_center_parallel_system(other, &represented)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierRepresentedCenterParallelSystem2 {
            sources,
            coordinates,
            projection,
            circle,
            radial_x,
            radial_y,
            normal_x,
            normal_y,
            tangent_x,
            tangent_y,
            speed_squared,
            weight,
            weight_squared,
        } = common;
        let [_, _, unit_x, unit_y]: [DenseTensorPolynomial; 4] = coordinates
            .try_into()
            .expect("a represented circle/parallel frame retains four coordinates");
        let rank = sources.len() + 1;
        let target_axis = sources.len();
        let axis = |coefficients: &[Real]| {
            DenseTensorPolynomial::from_axis_polynomial(rank, target_axis, coefficients)
        };
        let Some(system) = (|| {
            let radius_squared = self.radial_distance() * self.radial_distance();
            // These angular expressions are all multiplied by the same
            // positive W^2*sqrt(S) scale. They therefore remain directly
            // comparable in the parameter map below.
            let cross_unit_radial = unit_x
                .multiply(&radial_y)?
                .subtract(&unit_y.multiply(&radial_x)?)?;
            let cross_unit_normal = unit_x
                .multiply(&normal_y)?
                .subtract(&unit_y.multiply(&normal_x)?)?;
            let dot_unit_radial = unit_x
                .multiply(&radial_x)?
                .add(&unit_y.multiply(&radial_y)?)?;
            let dot_unit_normal = unit_x
                .multiply(&normal_x)?
                .add(&unit_y.multiply(&normal_y)?)?;
            let selected_scale = self.radial_distance() * self.turn_sign();
            let selected_half_plane = BezierRepresentedCircleParallelExpression2 {
                rational: weight_squared
                    .multiply(&cross_unit_normal)?
                    .scale(&selected_scale)?,
                radical: weight
                    .multiply(&cross_unit_radial)?
                    .scale(&selected_scale)?,
            };
            let diameter = BezierRepresentedCircleParallelExpression2 {
                rational: weight_squared
                    .multiply(&dot_unit_normal)?
                    .scale(self.radial_distance())?,
                radical: weight
                    .multiply(&dot_unit_radial)?
                    .scale(self.radial_distance())?,
            };
            let radius_squared_denominator = BezierRepresentedCircleParallelExpression2 {
                rational: axis(&[Real::zero()])?,
                radical: weight_squared.scale(&radius_squared)?,
            };

            // The analytic parallel derivative is a scalar multiple of H on
            // every regular cell. Retain the circle/source-tangent relation
            // here and apply that scalar's exact sign only after projection.
            let tangent_cross_source = radial_x
                .multiply(&tangent_x)?
                .add(&radial_y.multiply(&tangent_y)?)?
                .multiply(&weight)?
                .scale(&(-self.turn_sign()))?;
            let cross_radial_tangent = radial_x
                .multiply(&tangent_y)?
                .subtract(&radial_y.multiply(&tangent_x)?)?;
            let cross_normal_tangent = normal_x
                .multiply(&tangent_y)?
                .subtract(&normal_y.multiply(&tangent_x)?)?;
            let tangent_dot_source = BezierRepresentedCircleParallelExpression2 {
                rational: weight_squared
                    .multiply(&cross_normal_tangent)?
                    .scale(&self.turn_sign())?,
                radical: weight
                    .multiply(&cross_radial_tangent)?
                    .scale(&self.turn_sign())?,
            };
            Some(BezierRepresentedCircleParallelSystem2 {
                sources,
                projection,
                circle,
                selected_half_plane,
                diameter,
                radius_squared_denominator,
                tangent_cross_source,
                tangent_dot_source,
                speed_squared,
                weight,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(Arc::new(system)))
    }

    pub(in crate::bezier_offset) fn represented_rational_contact_location(
        &self,
        frame: &BezierRepresentedSelectedRadialCircleFrame2,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let point = match represented_point_evidence_coordinates(point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let difference = |point: &AlgebraicRootRepresentation,
                          center: &AlgebraicRootRepresentation| {
            Classification::from(represented_affine_coordinate(
                &[(point, &Real::one()), (center, &Real::from(-1_i8))],
                &Real::zero(),
            ))
        };
        let (Classification::Decided(dx), Classification::Decided(dy)) = (
            difference(&point[0], &frame.center[0]),
            difference(&point[1], &frame.center[1]),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let [dot, cross] =
            match Classification::from(represented_vector_dot_cross(&frame.unit_radial, &[dx, dy]))
            {
                Classification::Decided(products) => products,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let dot = match Classification::from(represented_affine_coordinate(
            &[(&dot, &frame.signed_radius)],
            &Real::zero(),
        )) {
            Classification::Decided(dot) => dot,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let oriented_cross_scale = &frame.signed_radius * self.turn_sign();
        let oriented_cross = match Classification::from(represented_affine_coordinate(
            &[(&cross, &oriented_cross_scale)],
            &Real::zero(),
        )) {
            Classification::Decided(cross) => cross,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cross_sign = match represented_policy_sign(&oriented_cross, policy) {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if cross_sign == RealSign::Negative {
            return Ok(Classification::Decided(None));
        }
        if cross_sign == RealSign::Positive {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )));
        }
        Ok(match represented_policy_sign(&dot, policy) {
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(Some(BezierAlgebraicCuspSemicircleContactLocation2::Start))
            }
            Classification::Decided(RealSign::Negative) => {
                Classification::Decided(Some(BezierAlgebraicCuspSemicircleContactLocation2::End))
            }
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a represented nonzero circle contact had zero diameter coordinates".into(),
                ));
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_replay_rational_circle_component(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        system: &BezierChordNormalDenseIntersectionSystem2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        let selected_roots = match system.selected_half_plane_parameters(range, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_roots = match system.angular_tangent_parameters(range, policy)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut boundaries = Vec::with_capacity(selected_roots.len() + angular_roots.len() + 2);
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(lower.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        boundaries.extend(selected_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: true,
            }
        }));
        boundaries.extend(angular_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: false,
            }
        }));
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(upper.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        let parameter_map = BezierAlgebraicCuspSemicircleRationalParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                semicircle: self.clone(),
                curve: other.clone(),
                system:
                    BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                        system: system.rational_parameter_map_system(),
                    },
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        };
        self.publish_partitioned_rational_circle_component(
            other,
            parameter_map,
            boundaries,
            policy,
            |boundary| {
                if boundary.selected_relation {
                    Ok(Classification::Decided(RealSign::Zero))
                } else {
                    system.selected_half_plane_sign(&boundary.parameter, policy)
                }
            },
            |boundary| system.diameter_sign(&boundary.parameter, policy),
            |parameter| system.angular_tangent_sign(parameter, policy),
        )
    }

    pub(in crate::bezier_offset) fn chord_normal_projective_rational_intersections_internal(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        system: BezierChordNormalDenseIntersectionSystem2,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        let candidates =
            match system.contact_parameters(SelectedThirdAxisDomain2::Finite(range), policy)? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                    return Ok(self
                        .chord_normal_projective_replay_rational_circle_component(
                            other, range, &system, policy,
                        )?
                        .map(|intersections| (intersections, None)));
                }
                Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                    return Ok(Classification::Decided((
                        BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                        None,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let selected = match system.selected_half_plane_sign(&candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location = match selected {
                RealSign::Negative => continue,
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match system.diameter_sign(&candidate, policy)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a nonzero chord-normal circle contact had zero local diameter".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let tangent_cross_sign = match system.tangent_cross_sign(&candidate, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let point = match rational_point_evidence_at_parameter(other, &candidate, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(candidate),
                point,
                tangent_cross_sign,
                tangent_dot_sign: None,
                location,
            });
        }
        let parameter_map = if retain_parameter_map
            && contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system:
                        BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
                            system: system.rational_parameter_map_system(),
                        },
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        } else {
            None
        };
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        )))
    }

    pub(in crate::bezier_offset) fn replay_rational_circle_component_with_exact_frame(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        frame: &BezierRepresentedSelectedRadialCircleFrame2,
        parameter_system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        let system =
            match self.represented_rational_component_system(other, range, frame, policy)? {
                Classification::Decided(system) => system,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (selected_roots, angular_roots) =
            if let Some(selected_roots) = &system.quadratic_selected_parameters {
                (selected_roots.clone(), Vec::new())
            } else {
                let selected_roots = match policy.strict_predicate_pass(|| {
                    system.parameters(&system.selected_half_plane, range, policy)
                })? {
                    Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => {
                        roots
                    }
                    Classification::Decided(
                        BezierAlgebraicFiberProjection2::IdenticallyZero
                        | BezierAlgebraicFiberProjection2::Degenerate,
                    ) => {
                        return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                    ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let angular_roots = match policy.strict_predicate_pass(|| {
                    system.parameters(&system.angular_tangent, range, policy)
                })? {
                    Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => {
                        roots
                    }
                    Classification::Decided(
                        BezierAlgebraicFiberProjection2::IdenticallyZero
                        | BezierAlgebraicFiberProjection2::Degenerate,
                    ) => {
                        return Ok(Classification::Decided(
                        BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                    ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (selected_roots, angular_roots)
            };
        let mut boundaries = Vec::with_capacity(selected_roots.len() + angular_roots.len() + 2);
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(lower.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        boundaries.extend(selected_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: true,
            }
        }));
        boundaries.extend(angular_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: false,
            }
        }));
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(upper.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        let parameter_map = BezierAlgebraicCuspSemicircleRationalParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                semicircle: self.clone(),
                curve: other.clone(),
                system: parameter_system,
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        };
        #[cfg(feature = "dispatch-trace")]
        {
            let path = match &parameter_map.data.system {
                BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { .. } => {
                    "recursive-rational-frame-component"
                }
                _ => "represented-circle-component",
            };
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-rational-kernel",
                path,
            );
        }
        self.publish_partitioned_rational_circle_component(
            other,
            parameter_map,
            boundaries,
            policy,
            |boundary| {
                if boundary.selected_relation {
                    Ok(Classification::Decided(RealSign::Zero))
                } else {
                    system.sign(&system.selected_half_plane, &boundary.parameter, policy)
                }
            },
            |boundary| system.sign(&system.diameter, &boundary.parameter, policy),
            |parameter| system.sign(&system.angular_tangent, parameter, policy),
        )
    }

    pub(in crate::bezier_offset) fn represented_rational_intersections_internal(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        retained_frame: Option<BezierRepresentedSelectedRadialCircleFrame2>,
        _dispatch_path: &'static str,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-rational-kernel",
            _dispatch_path,
        );
        let system = match self.represented_rational_system(other, range, retained_frame, policy)? {
            Classification::Decided(system) => system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidates = match selected_dense_last_axis_parameters(
            &system.incidence,
            &system.sources,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                return Ok(self
                    .replay_rational_circle_component_with_exact_frame(
                        other,
                        range,
                        &system.frame,
                        BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented {
                            frame: system.frame.clone(),
                        },
                        policy,
                    )?
                    .map(|intersections| (intersections, None)));
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Decided((
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                    None,
                )));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut contacts = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let mut selected_tuple = system.sources.clone();
            selected_tuple.push(match &candidate {
                BezierParameter2::Exact(parameter) => {
                    AlgebraicRootRepresentation::from_exact_value(parameter)
                }
                BezierParameter2::Algebraic(parameter) => {
                    parameter_representation(parameter, policy)
                }
            });
            match dense_polynomial_tuple_sign(&system.incidence, &selected_tuple, policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let point = match rational_point_evidence_at_parameter(other, &candidate, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location =
                match self.represented_rational_contact_location(&system.frame, &point, policy)? {
                    Classification::Decided(Some(location)) => location,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let tangent_cross_sign = match dense_polynomial_tuple_sign(
                &system.tangent_cross,
                &selected_tuple,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(candidate),
                point,
                tangent_cross_sign,
                tangent_dot_sign: None,
                location,
            });
        }
        let parameter_map = if retain_parameter_map
            && contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented {
                        frame: system.frame,
                    },
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        } else {
            None
        };
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        )))
    }

    /// Adapts an exact rational line image to the authoritative selected-
    /// circle/chord kernel.
    ///
    /// A retained pair-radial center already lives in a recursive quadratic
    /// tower. Intersecting it with a line adds at most one positive quadratic
    /// generator, whereas projecting the equivalent circle/rational system
    /// takes a dense norm over every selected source axis. Keep the compact
    /// chord solve as the contact authority. An affine parameterization reuses
    /// the chord scalar directly; a nonlinear line image inverts only the
    /// certified contact coordinate on the target's native parameter axis.
    pub(in crate::bezier_offset) fn exact_line_image_selected_radial_rational_intersections(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<
            Option<(
                BezierAlgebraicCuspSemicircleRationalIntersections2,
                Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
            )>,
        >,
    > {
        if !matches!(
            CurveParameterDomain2::new(&CurveParameterRange2::unit(), None)
                .contains_finite_range(range, policy),
            Ok(Classification::Decided(true))
        ) {
            return Ok(Classification::Decided(None));
        }
        let (line, affine_parameterization) =
            if let Some(line) = other.exact_linear_parameterization_line() {
                (line, true)
            } else {
                let fit = match other.fit_exact_line_image(&policy.strict_counterpart())? {
                    Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit,
                    Classification::Decided(BezierLineImageFitRelation::NotLine)
                    | Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                };
                if retain_parameter_map {
                    return Ok(Classification::Decided(None));
                }
                (fit.line().clone(), false)
            };
        let chord = match BezierAlgebraicChord2::try_new(
            CurvePoint2::from(line.start().clone()),
            CurvePoint2::from(line.end().clone()),
            policy,
        )? {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
        };
        let similarity_source = self.selected_radial_similarity_source(policy)?;
        let (solve_circle, solve_chord, solve_other, similarity) =
            if let Some((source_circle, transform)) = similarity_source {
                let (a, b, d, e, xoff, yoff) = transform.affine_components();
                let inverse_scale_squared =
                    (Real::one() / (transform.scale() * transform.scale()))?;
                let diagonal = real_sign(b, &CurveContext::STRICT) == Some(RealSign::Zero)
                    && real_sign(d, &CurveContext::STRICT) == Some(RealSign::Zero);
                let anti_diagonal = real_sign(a, &CurveContext::STRICT) == Some(RealSign::Zero)
                    && real_sign(e, &CurveContext::STRICT) == Some(RealSign::Zero);
                let inverse_point = |point: &Point2| -> CurveResult<Point2> {
                    let x = point.x() - xoff;
                    let y = point.y() - yoff;
                    if diagonal {
                        return Ok(Point2::new((x / a)?, (y / e)?));
                    }
                    if anti_diagonal {
                        return Ok(Point2::new((y / d)?, (x / b)?));
                    }
                    Ok(Point2::new(
                        Real::dot2_refs([a, d], [&x, &y]) * &inverse_scale_squared,
                        Real::dot2_refs([b, e], [&x, &y]) * &inverse_scale_squared,
                    ))
                };
                let source_line =
                    LineSeg2::try_new(inverse_point(line.start())?, inverse_point(line.end())?)?;
                let source_chord = match BezierAlgebraicChord2::try_new(
                    CurvePoint2::from(source_line.start().clone()),
                    CurvePoint2::from(source_line.end().clone()),
                    policy,
                )? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                };
                let Some(controls) = other.affine_control_points() else {
                    return Ok(Classification::Decided(None));
                };
                let source_controls = controls
                    .iter()
                    .map(inverse_point)
                    .collect::<CurveResult<Vec<_>>>()?;
                let source_other =
                    RationalBezier2::try_new(source_controls, other.weights().to_vec())?;
                (source_circle, source_chord, source_other, Some(transform))
            } else {
                (self.clone(), chord.clone(), other.clone(), None)
            };
        let intersections =
            match solve_circle.chord_intersections_prefer_exact_line(&solve_chord, true, policy)? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
            };
        let contacts = intersections;
        if contacts.is_empty() {
            return Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                },
                None,
            ))));
        }

        let zero = Real::zero();
        let one = Real::one();
        let nonlinear_tangent = if affine_parameterization {
            None
        } else {
            let source = other.homogeneous_power_basis()?;
            let axis_numerator = match chord.data.parameter_axis.axis {
                Axis2::X => &source.x_numerator,
                Axis2::Y => &source.y_numerator,
            };
            Some(polynomial_subtract(
                &polynomial_multiply(&polynomial_derivative(axis_numerator), &source.weight),
                &polynomial_multiply(axis_numerator, &polynomial_derivative(&source.weight)),
            ))
        };
        let line_axis_sign = if nonlinear_tangent.is_some() {
            let axis_delta = match chord.data.parameter_axis.axis {
                Axis2::X => line.end().x() - line.start().x(),
                Axis2::Y => line.end().y() - line.start().y(),
            };
            match real_sign(&axis_delta, &CurveContext::STRICT) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => Some(sign),
                Some(RealSign::Zero) | None => return Ok(Classification::Decided(None)),
            }
        } else {
            None
        };
        let mut retained_parameters = Vec::with_capacity(contacts.len());
        let mut rational_contacts = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let solve_point = contact.point.clone();
            let tangent_dot_sign = if affine_parameterization && other.degree() == 1 {
                match contact.tangent_dot_sign(&solve_circle, &solve_chord, policy)? {
                    Classification::Decided(sign) => Some(sign),
                    Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                }
            } else {
                None
            };
            let (cusp_parameter, point, tangent_cross_sign) = if let Some(transform) =
                similarity.as_ref()
            {
                let point = CurvePoint2::from(BezierSimilarityPoint2::new(
                    contact.point.clone(),
                    transform.clone(),
                    policy,
                ));
                let cusp_parameter = match &contact.cusp_parameter {
                        BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) => {
                            BezierAlgebraicCuspSemicircleParameter2::Exact(parameter.clone())
                        }
                        BezierAlgebraicCuspSemicircleParameter2::Mapped(_) => {
                            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                                BezierAlgebraicCuspSemicircleMappedParameterData2::SimilarityTransport {
                                    semicircle: self.clone(),
                                    source: contact.cusp_parameter.clone(),
                                    point: point.clone(),
                                    policy: *policy,
                                },
                            ))
                        }
                    };
                let tangent_cross_sign = if transform.reverses_orientation() {
                    product_sign(contact.tangent_cross_sign, RealSign::Negative)
                } else {
                    contact.tangent_cross_sign
                };
                (cusp_parameter, point, tangent_cross_sign)
            } else {
                (
                    contact.cusp_parameter.clone(),
                    contact.point.clone(),
                    contact.tangent_cross_sign,
                )
            };
            let location = match &cusp_parameter {
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) if parameter == &zero => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Start
                }
                BezierAlgebraicCuspSemicircleParameter2::Exact(parameter) if parameter == &one => {
                    BezierAlgebraicCuspSemicircleContactLocation2::End
                }
                BezierAlgebraicCuspSemicircleParameter2::Exact(_)
                | BezierAlgebraicCuspSemicircleParameter2::Mapped(_) => {
                    BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }
            };
            if affine_parameterization {
                let retained_parameter =
                    match contact.chord_parameter.exact_line_curve_parameter(policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                    };
                let other_parameter = if let Some(transform) = similarity.as_ref() {
                    retained_parameter.transported_recursive_line_identity(line.clone(), transform)
                } else {
                    retained_parameter
                };
                retained_parameters.push((other_parameter.clone(), cusp_parameter.clone()));
                rational_contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                    other_parameter,
                    point,
                    tangent_cross_sign,
                    tangent_dot_sign,
                    location,
                });
                continue;
            }

            let exact_parameters =
                match represented_point_evidence_coordinates(&solve_point, policy)? {
                    Classification::Decided([x, y]) => {
                        match (x.exact_point_witness(), y.exact_point_witness()) {
                            (Some(x), Some(y)) => {
                                match solve_other.point_incidence_on_range(
                                    &Point2::new(x.clone(), y.clone()),
                                    range,
                                    policy,
                                )? {
                                    Classification::Decided(
                                        crate::RationalBezierPointIncidence2::Parameters(
                                            parameters,
                                        ),
                                    ) => Some(
                                        parameters.into_iter().map(CurveParameter2::from).collect(),
                                    ),
                                    Classification::Decided(
                                        crate::RationalBezierPointIncidence2::EntireCurve,
                                    )
                                    | Classification::Uncertain(_) => {
                                        return Ok(Classification::Decided(None));
                                    }
                                }
                            }
                            _ => None,
                        }
                    }
                    Classification::Uncertain(_) => None,
                };
            let source_parameters = if let Some(parameters) = exact_parameters {
                parameters
            } else {
                match solve_chord.collinear_source_parameters_at_chord_endpoint(
                    &solve_other,
                    &solve_point,
                    &CurveParameterRange2::unit(),
                    policy,
                )? {
                    Classification::Decided(parameters) => parameters,
                    Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                }
            };
            let tangent = nonlinear_tangent
                .as_ref()
                .expect("a nonlinear line image retains its axis tangent");
            let line_axis_sign =
                line_axis_sign.expect("a nonlinear line image has a nonzero chord axis");
            for other_parameter in source_parameters {
                let source_tangent_sign =
                    if let Some(parameter) = other_parameter.as_bezier_parameter() {
                        signed_coefficients_at_parameter(tangent, parameter, policy)?
                    } else if let Some(parameter) = other_parameter.as_recursive_projective() {
                        parameter.polynomial_sign(tangent, policy)?
                    } else {
                        return Ok(Classification::Decided(None));
                    };
                let source_tangent_sign = match source_tangent_sign {
                    Classification::Decided(sign) => product_sign(sign, line_axis_sign),
                    Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                };
                let target_point = match rational_point_evidence_at_region_parameter(
                    other,
                    &other_parameter,
                    policy,
                )? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(_) => {
                        return Ok(Classification::Decided(None));
                    }
                };
                rational_contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                    other_parameter,
                    point: target_point,
                    tangent_cross_sign: product_sign(tangent_cross_sign, source_tangent_sign),
                    tangent_dot_sign: None,
                    location,
                });
            }
        }

        if !affine_parameterization {
            for index in 1..rational_contacts.len() {
                let mut cursor = index;
                while cursor > 0 {
                    match rational_contacts[cursor]
                        .other_parameter
                        .cmp_by_refinement(&rational_contacts[cursor - 1].other_parameter, policy)?
                    {
                        Classification::Decided(std::cmp::Ordering::Less) => {
                            rational_contacts.swap(cursor, cursor - 1);
                            cursor -= 1;
                        }
                        Classification::Decided(_) => break,
                        Classification::Uncertain(_) => {
                            return Ok(Classification::Decided(None));
                        }
                    }
                }
            }
        }

        let parameter_map = if retain_parameter_map
            && rational_contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            let system =
                match self.recursive_selected_radial_rational_system(other, range, policy)? {
                    Classification::Decided(system) => system,
                    Classification::Uncertain(_) => return Ok(Classification::Decided(None)),
                };
            let parameter_cache = BezierAlgebraicCuspSemicircleParameterCache2::default();
            for (other_parameter, cusp_parameter) in retained_parameters {
                parameter_cache.retain_cusp_parameter(other_parameter, &cusp_parameter, policy);
            }
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive {
                        system,
                    },
                    policy: policy.retained_object_policy(),
                    parameter_cache,
                }),
            })
        } else {
            None
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-rational-kernel",
            if affine_parameterization {
                "exact-linear-chord-authority"
            } else {
                "exact-line-image-chord-authority"
            },
        );
        Ok(Classification::Decided(Some((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts: rational_contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        ))))
    }

    pub(in crate::bezier_offset) fn rational_intersections_internal(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        retain_parameter_map: bool,
        policy: &CurveContext,
    ) -> CurveResult<
        Classification<(
            BezierAlgebraicCuspSemicircleRationalIntersections2,
            Option<BezierAlgebraicCuspSemicircleRationalParameterMap2>,
        )>,
    > {
        // Selected bounds schedule discovery through a pole-free enclosure.
        // Exact clipping belongs to the consuming pair, where circle contact
        // evidence can identify an independently retained endpoint. Comparing
        // freshly isolated scalars before that replay reconstructs fields and
        // can lose a tangency proof already owned by the geometry.
        let discovery_range = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &discovery_range;
        let unit = CurveParameterRange2::unit();
        if (self.uses_selected_chord_normal_frame() || self.uses_selected_radial_frame())
            && matches!(
                CurveParameterDomain2::new(&unit, None)
                    .contains_finite_range(range, &policy.strict_counterpart()),
                Ok(Classification::Decided(true))
            )
            && let Classification::Decided(other_bounds) = other.certified_bounds_classified()
        {
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                if let Classification::Decided(circle_bounds) =
                    self.conservative_bounds_refined(refinement_steps, policy)?
                    && circle_bounds.overlaps(&other_bounds, &CurveContext::STRICT)
                        == Classification::Decided(false)
                {
                    return Ok(Classification::Decided((
                        BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                        },
                        None,
                    )));
                }
            }
        }
        if self.uses_selected_chord_normal_frame() {
            if let Classification::Decided(Some(system)) =
                self.chord_normal_projective_rational_system(other, range, policy)?
            {
                return self.chord_normal_projective_rational_intersections_internal(
                    other,
                    range,
                    retain_parameter_map,
                    system,
                    policy,
                );
            }
            return self.represented_rational_intersections_internal(
                other,
                range,
                retain_parameter_map,
                None,
                "represented",
                policy,
            );
        }
        if self.uses_selected_radial_frame() {
            if let Classification::Decided(Some(intersections)) = self
                .exact_line_image_selected_radial_rational_intersections(
                    other,
                    range,
                    retain_parameter_map,
                    policy,
                )?
            {
                return Ok(Classification::Decided(intersections));
            }
            let system = match self.selected_radial_rational_system(other, range, policy)? {
                Classification::Decided(system) => system,
                Classification::Uncertain(UncertaintyReason::Unsupported) => {
                    if let Some(circle) = other.retained_circular_conic()
                        && self.recursive_selected_radial_has_same_supporting_circle(
                            &circle.center,
                            &circle.radius_squared,
                            policy,
                        )? == Classification::Decided(true)
                    {
                        let component = match self
                            .recursive_selected_radial_rational_system(other, range, policy)?
                        {
                            Classification::Decided(system) => system,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-rational-kernel",
                            "recursive-structural-circle",
                        );
                        return Ok(self
                            .recursive_selected_radial_replay_rational_circle_component(
                                other, range, component, policy,
                            )?
                            .map(|intersections| (intersections, None)));
                    }
                    let recursive = match self
                        .recursive_selected_radial_rational_system(other, range, policy)?
                    {
                        Classification::Decided(system) => system,
                        Classification::Uncertain(UncertaintyReason::Unsupported) => {
                            if let Some(frame) = self.exact_point_component_frame(policy)? {
                                return self.represented_rational_intersections_internal(
                                    other,
                                    range,
                                    retain_parameter_map,
                                    Some(frame),
                                    "exact-rational-frame",
                                    policy,
                                );
                            }
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    return self.recursive_selected_radial_rational_intersections_internal(
                        other,
                        range,
                        retain_parameter_map,
                        recursive,
                        policy,
                    );
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let Some([first_parameter, second_parameter]) =
                system.pair_map.compact_source_parameters()
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let (first_parameter, second_parameter) = (&first_parameter, &second_parameter);
            let candidates = match selected_trivariate_third_axis_parameters(
                &system.incidence_projection,
                first_parameter,
                second_parameter,
                SelectedThirdAxisDomain2::Finite(range),
                policy,
            )? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                    return Ok(self
                        .selected_radial_replay_rational_circle_component(
                            other, range, system, policy,
                        )?
                        .map(|intersections| (intersections, None)));
                }
                Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                    return Ok(Classification::Decided((
                        BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                        None,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut tangent_projection = None;
            let mut tangent_projection_attempted = false;
            let mut tangent_constraint = None;
            let radical_sign = |expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
                                candidate: &BezierParameter2| {
                algebraic_cusp_trivariate_square_root_sum_sign(
                    expression,
                    &system.discriminant,
                    first_parameter,
                    second_parameter,
                    candidate,
                    system.branch,
                    policy,
                )
            };
            let mut contacts = Vec::with_capacity(candidates.len());
            for mut candidate in candidates {
                let has_box_root = projected_selected_trivariate_candidate_has_box_root(
                    &system.incidence_projection,
                    first_parameter,
                    second_parameter,
                    &candidate,
                    8,
                )?;
                if !has_box_root {
                    if trivariate_parameter_triple_bounded_box_sign(
                        &system.incidence_projection,
                        first_parameter,
                        second_parameter,
                        &candidate,
                        64,
                    )?
                    .is_some()
                    {
                        continue;
                    }
                    if !tangent_projection_attempted {
                        tangent_projection = system
                            .tangent_cross
                            .radical
                            .multiply(&system.tangent_cross.radical)
                            .and_then(|radical_squared| {
                                TrivariatePolynomial2::sum_products(&[
                                    (
                                        &system.tangent_cross.rational,
                                        &system.tangent_cross.rational,
                                        false,
                                    ),
                                    (&radical_squared, &system.discriminant, true),
                                ])
                            });
                        tangent_projection_attempted = true;
                    }
                    if tangent_constraint.is_none() {
                        tangent_constraint = Some(match &tangent_projection {
                            Some(projection) => selected_trivariate_third_axis_constraint(
                                projection,
                                first_parameter,
                                second_parameter,
                            )?,
                            None => None,
                        });
                    }
                    if let Some(constraint) = tangent_constraint
                        .as_ref()
                        .and_then(|constraint| constraint.as_deref())
                    {
                        candidate =
                            selected_parameter_reduced_by_constraint(&candidate, constraint)?;
                    }
                }
                match algebraic_cusp_projected_trivariate_square_root_sum_sign(
                    &system.incidence,
                    &system.discriminant,
                    &system.incidence_projection,
                    first_parameter,
                    second_parameter,
                    &candidate,
                    system.branch,
                    has_box_root,
                    policy,
                )? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let selected = match radical_sign(&system.selected_half_plane, &candidate)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let location = match selected {
                    RealSign::Negative => continue,
                    RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                    RealSign::Zero => match radical_sign(&system.diameter, &candidate)? {
                        Classification::Decided(RealSign::Positive) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::Start
                        }
                        Classification::Decided(RealSign::Negative) => {
                            BezierAlgebraicCuspSemicircleContactLocation2::End
                        }
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a nonzero pair-radial circle had an indeterminate endpoint".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                };
                let tangent_cross_sign = match if let Some(tangent_projection) = &tangent_projection
                {
                    algebraic_cusp_projected_trivariate_square_root_sum_sign(
                        &system.tangent_cross,
                        &system.discriminant,
                        tangent_projection,
                        first_parameter,
                        second_parameter,
                        &candidate,
                        system.branch,
                        false,
                        policy,
                    )?
                } else {
                    radical_sign(&system.tangent_cross, &candidate)?
                } {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                // A line/circle corner publication needs the companion dot
                // sign as well as the already-retained cross sign. The
                // selected-radial system owns this angular predicate now;
                // retaining its one-word result avoids replaying a general
                // circle-pair solve after the center has been selected.
                let tangent_dot_sign = if other.degree() == 1 {
                    match algebraic_cusp_trivariate_square_root_sum_sign(
                        &system.angular_tangent,
                        &system.discriminant,
                        first_parameter,
                        second_parameter,
                        &candidate,
                        system.branch,
                        &CurveContext::STRICT,
                    )? {
                        Classification::Decided(sign) => Some(if self.is_clockwise() {
                            product_sign(sign, RealSign::Negative)
                        } else {
                            sign
                        }),
                        // This sign may become persistent angular authority.
                        // An APPROXIMATE_512 terminal is predicate evidence,
                        // not a reusable exact tangent relation; decline the
                        // fast path and retain the general publication kernel.
                        Classification::Uncertain(_) => None,
                    }
                } else {
                    None
                };
                let point = match rational_point_evidence_at_parameter(other, &candidate, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                    other_parameter: CurveParameter2::from(candidate),
                    point,
                    tangent_cross_sign,
                    tangent_dot_sign,
                    location,
                });
            }
            let parameter_map = if retain_parameter_map
                && contacts.iter().any(|contact| {
                    contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
                }) {
                Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                    data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                        semicircle: self.clone(),
                        curve: other.clone(),
                        system:
                            BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial {
                                pair_map: system.pair_map,
                                branch: system.branch,
                                discriminant: system.discriminant,
                                diameter: system.diameter,
                                radius_squared_denominator: system.radius_squared_denominator,
                                tangent_cross: system.tangent_cross,
                                angular_tangent: system.angular_tangent,
                            },
                        policy: policy.retained_object_policy(),
                        parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                    }),
                })
            } else {
                None
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-rational-kernel",
                "direct-pair-radial",
            );
            return Ok(Classification::Decided((
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                    contacts,
                    overlaps: Vec::new(),
                },
                parameter_map,
            )));
        }
        if self.uses_selected_parallel_normal_frame() {
            return self.selected_parallel_normal_rational_intersections_internal(
                other,
                range,
                retain_parameter_map,
                None,
                policy,
            );
        }
        if let Some(circle) = other.retained_circular_conic()
            && self.is_disjoint_from_supporting_circle(
                &circle.center,
                &circle.radius_squared,
                policy,
            )? == Classification::Decided(true)
        {
            return Ok(Classification::Decided((
                BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                },
                None,
            )));
        }
        let system = self.rational_system(other)?;
        let mut diagonal_location = self
            .data
            .frame
            .rational()
            .and_then(|frame| frame.data.parallel.as_ref())
            .and_then(|parallel| parallel.source().to_rational_bezier().ok())
            .filter(|source| source == other)
            .and_then(|_| {
                let center_distance = self.center_parallel_distance();
                let start_distance = &center_distance + self.radial_distance();
                let end_distance = center_distance - self.radial_distance();
                if real_sign(&start_distance, &CurveContext::STRICT) == Some(RealSign::Zero) {
                    Some(BezierAlgebraicCuspSemicircleContactLocation2::Start)
                } else if real_sign(&end_distance, &CurveContext::STRICT) == Some(RealSign::Zero) {
                    Some(BezierAlgebraicCuspSemicircleContactLocation2::End)
                } else {
                    None
                }
            });
        let incidence_source = if diagonal_location.is_some() {
            if let Some(residual) = deflate_bivariate_parameter_diagonal_exact(&system.incidence) {
                residual
            } else {
                let report = deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter(
                    &system.incidence,
                    CurveResultantParameter::First,
                    &parameter_representation(self.cusp_parameter(), policy),
                    policy.predicate_policy(),
                );
                if report.certainty == PredicateCertainty::Approximate {
                    policy.observe_approximate_512();
                }
                match report.status {
                    AlgebraicFiberDiagonalDeflationStatus::Deflated => {
                        report.reduced_polynomial.ok_or_else(|| {
                            CurveError::Topology(
                                "deflated source-related circle incidence omitted its residual"
                                    .into(),
                            )
                        })?
                    }
                    AlgebraicFiberDiagonalDeflationStatus::IdenticallyZeroFiber => {
                        diagonal_location = None;
                        system.incidence.clone()
                    }
                    AlgebraicFiberDiagonalDeflationStatus::NotARoot => {
                        return Err(CurveError::Topology(
                            "a source-related selected-circle endpoint was absent from its incidence fiber"
                                .into(),
                        ));
                    }
                    AlgebraicFiberDiagonalDeflationStatus::InvalidEvidence => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    AlgebraicFiberDiagonalDeflationStatus::UnsupportedCoefficient => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                    }
                    AlgebraicFiberDiagonalDeflationStatus::Undecided => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                    }
                }
            }
        } else {
            system.incidence.clone()
        };
        let reduce = |polynomial: &BivariatePolynomial| {
            bivariate_reduce_axis(
                polynomial,
                self.cusp_parameter().polynomial(),
                CurveResultantParameter::First,
                policy,
            )
        };
        if let Some(circle) = other.retained_circular_conic()
            && self.has_same_supporting_circle(&circle.center, &circle.radius_squared, policy)?
                == Classification::Decided(true)
        {
            let selected_half_plane = match reduce(&system.selected_half_plane)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let diameter_side = match reduce(&system.diameter_side)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let radius_squared_denominator = match reduce(&system.radius_squared_denominator)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let angular_tangent = match reduce(&system.angular_tangent)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(self
                .replay_rational_circle_component(
                    other,
                    range,
                    selected_half_plane,
                    diameter_side,
                    radius_squared_denominator,
                    angular_tangent,
                    policy,
                )?
                .map(|intersections| (intersections, None)));
        }
        let incidence = match reduce(&incidence_source)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let selected_half_plane = match reduce(&system.selected_half_plane)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let tangent_cross = self.rational_incidence_tangent_cross_sign_polynomial(&incidence);
        let mut diameter_side = None;
        // Root enumeration is construction evidence. Exhaust it with the
        // requested policy's terminal approximation suppressed; only later
        // contact/equality predicates may consume APPROXIMATE_512. If the
        // degree-multiplied projection cannot sign its `Real` coefficients,
        // the compact selected-fiber authority below remains exact.
        let projection = policy.strict_predicate_pass(|| {
            selected_fiber_parameters(
                &incidence,
                &BezierParameter2::Algebraic(self.cusp_parameter().clone()),
                range,
                policy,
            )
        })?;
        let candidates = match projection {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
                let diameter_side = match reduce(&system.diameter_side)? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let radius_squared_denominator = match reduce(&system.radius_squared_denominator)? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let angular_tangent = match reduce(&system.angular_tangent)? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(self
                    .replay_rational_circle_component(
                        other,
                        range,
                        selected_half_plane,
                        diameter_side,
                        radius_squared_denominator,
                        angular_tangent,
                        policy,
                    )?
                    .map(|intersections| (intersections, None)));
            }
            Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
                return Ok(Classification::Decided((
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                    None,
                )));
            }
            Classification::Uncertain(UncertaintyReason::RealSign) => {
                // The degree-multiplied projection can leave every
                // resultant coefficient as an unresolved `Real` even
                // though the original bivariate fiber is regular. Keep
                // the target root in the selected cusp field and let the
                // local Hypersolve isolator operate on the unreduced
                // incidence instead of flattening that field.
                return Ok(policy
                    .strict_predicate_pass(|| {
                        self.rational_frame_selected_fiber_intersections(
                            other, range, &system, policy,
                        )
                    })?
                    .map(|intersections| (intersections, None)));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let mut contacts =
            Vec::with_capacity(candidates.len() + usize::from(diagonal_location.is_some()));
        if let Some(location) = diagonal_location
            && match CurveParameterDomain2::new(range, None)
                .contains_finite_parameter(&CurveParameter2::from(cusp_parameter.clone()), policy)?
            {
                Classification::Decided(inside) => inside,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        {
            let point = match rational_point_evidence_at_parameter(other, &cusp_parameter, policy)?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(cusp_parameter.clone()),
                point,
                tangent_cross_sign: RealSign::Zero,
                tangent_dot_sign: None,
                location,
            });
        }
        for candidate in candidates {
            let selected_sign = match algebraic_selected_correlated_predicate_sign(
                &incidence,
                &selected_half_plane,
                &cusp_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected_sign == RealSign::Negative {
                continue;
            }
            let location = if selected_sign == RealSign::Zero {
                if diameter_side.is_none() {
                    diameter_side = Some(match reduce(&system.diameter_side)? {
                        Classification::Decided(polynomial) => polynomial,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    });
                }
                match algebraic_selected_correlated_predicate_sign(
                    &incidence,
                    diameter_side
                        .as_ref()
                        .expect("diameter-side reduction was initialized"),
                    &cusp_parameter,
                    &candidate,
                    policy,
                )? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "nonzero algebraic semicircle radius had an indeterminate diameter endpoint"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let tangent_cross_sign = match algebraic_selected_correlated_predicate_sign(
                &incidence,
                &tangent_cross,
                &cusp_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let point = match rational_point_evidence_at_parameter(other, &candidate, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(candidate),
                point,
                tangent_cross_sign,
                tangent_dot_sign: None,
                location,
            });
        }
        let parameter_map = if retain_parameter_map
            && contacts.iter().any(|contact| {
                contact.location == BezierAlgebraicCuspSemicircleContactLocation2::Interior
            }) {
            let diameter_side = match diameter_side {
                Some(diameter_side) => diameter_side,
                None => match reduce(&system.diameter_side)? {
                    Classification::Decided(polynomial) => polynomial,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            let radius_squared_denominator = match reduce(&system.radius_squared_denominator)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Some(BezierAlgebraicCuspSemicircleRationalParameterMap2 {
                data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                    semicircle: self.clone(),
                    curve: other.clone(),
                    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                        cusp_parameter,
                        incidence,
                        diameter: BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(
                            diameter_side,
                        ),
                        radius_squared_denominator,
                    },
                    policy: policy.retained_object_policy(),
                    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
                }),
            })
        } else {
            None
        };
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
                contacts,
                overlaps: Vec::new(),
            },
            parameter_map,
        )))
    }

    /// Publishes the regular monotone cells of one rational carrier already
    /// proven to lie on this supporting circle.
    ///
    /// Field-specific code supplies only exact selected-half, diameter, and
    /// angular signs. Ordering, duplicate-boundary collapse, orientation,
    /// mapped parameters, and endpoint-only contacts remain one authority for
    /// one-field and pair-radial circles.
    pub(in crate::bezier_offset) fn publish_partitioned_rational_circle_component(
        &self,
        other: &RationalBezier2,
        parameter_map: BezierAlgebraicCuspSemicircleRationalParameterMap2,
        mut boundaries: Vec<BezierAlgebraicCuspSemicircleRationalComponentBoundary2>,
        policy: &CurveContext,
        selected_sign: impl Fn(
            &BezierAlgebraicCuspSemicircleRationalComponentBoundary2,
        ) -> CurveResult<Classification<RealSign>>,
        diameter_sign: impl Fn(
            &BezierAlgebraicCuspSemicircleRationalComponentBoundary2,
        ) -> CurveResult<Classification<RealSign>>,
        angular_sign: impl Fn(&BezierParameter2) -> CurveResult<Classification<RealSign>>,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        for index in 1..boundaries.len() {
            let mut cursor = index;
            while cursor > 0 {
                let order = match boundaries[cursor]
                    .parameter
                    .cmp_by_refinement(&boundaries[cursor - 1].parameter, policy)?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if order != std::cmp::Ordering::Less {
                    break;
                }
                boundaries.swap(cursor, cursor - 1);
                cursor -= 1;
            }
        }
        let mut distinct: Vec<BezierAlgebraicCuspSemicircleRationalComponentBoundary2> =
            Vec::with_capacity(boundaries.len());
        for boundary in boundaries {
            if let Some(previous) = distinct.last_mut() {
                let order = match previous
                    .parameter
                    .cmp_by_refinement(&boundary.parameter, policy)?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if order == std::cmp::Ordering::Equal {
                    previous.selected_relation |= boundary.selected_relation;
                    if previous.parameter.scalar().is_some() {
                        previous.correlation =
                            BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent;
                    } else if previous.selected_relation {
                        previous.correlation =
                            BezierAlgebraicCuspSemicircleRationalCorrelation2::Map;
                    }
                    continue;
                }
                if order == std::cmp::Ordering::Greater {
                    return Err(CurveError::Topology(
                        "rational circle component boundaries were not ordered".into(),
                    ));
                }
            }
            distinct.push(boundary);
        }
        let boundaries = distinct;
        let endpoint = |boundary: &BezierAlgebraicCuspSemicircleRationalComponentBoundary2| {
            let selected = match selected_sign(boundary)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location = match selected {
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                RealSign::Zero => match diameter_sign(boundary)? {
                    Classification::Decided(RealSign::Positive) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::Start
                    }
                    Classification::Decided(RealSign::Negative) => {
                        BezierAlgebraicCuspSemicircleContactLocation2::End
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "positive-radius rational overlap endpoint had zero diameter side"
                                .into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
                RealSign::Negative => return Ok(Classification::Decided(None)),
            };
            let contact = BezierAlgebraicCuspSemicircleRationalMapContact2 {
                other_parameter: CurveParameter2::from(boundary.parameter.clone()),
                location,
                correlation: boundary.correlation.clone(),
            };
            Ok(Classification::Decided(Some((
                parameter_map.mapped_parameter(contact),
                location,
            ))))
        };

        let expected_same_sign = if self.is_clockwise() {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        let mut overlaps = Vec::new();
        let mut covered_boundaries = vec![false; boundaries.len()];
        for (index, pair) in boundaries.windows(2).enumerate() {
            let sample = match pair[0]
                .parameter
                .strict_scalar_between_ordered(&pair[1].parameter, policy)?
            {
                Classification::Decided(sample) => BezierParameter2::Exact(sample),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let sample_boundary = BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter: sample.clone(),
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
                selected_relation: false,
            };
            let selected = match selected_sign(&sample_boundary)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected == RealSign::Negative {
                continue;
            }
            if selected == RealSign::Zero {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            let angular = match angular_sign(&sample)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if angular == RealSign::Zero {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            let orientation = if angular == expected_same_sign {
                CurveOverlapOrientation2::Same
            } else {
                CurveOverlapOrientation2::Reversed
            };
            let cell_endpoint =
                |boundary: &BezierAlgebraicCuspSemicircleRationalComponentBoundary2,
                 at_right: bool| {
                    if boundary.selected_relation {
                        // A selected-half root is one of the two semicircle
                        // endpoints.  The regular cell's exact angular
                        // orientation and the side on which the root closes
                        // determine which endpoint without re-solving the
                        // retained three-field diameter predicate.
                        let at_end = match orientation {
                            CurveOverlapOrientation2::Same => at_right,
                            CurveOverlapOrientation2::Reversed => !at_right,
                        };
                        let parameter = BezierAlgebraicCuspSemicircleParameter2::Exact(if at_end {
                            Real::one()
                        } else {
                            Real::zero()
                        });
                        return Ok(Classification::Decided(Some((
                            parameter,
                            if at_end {
                                BezierAlgebraicCuspSemicircleContactLocation2::End
                            } else {
                                BezierAlgebraicCuspSemicircleContactLocation2::Start
                            },
                        ))));
                    }
                    endpoint(boundary)
                };
            let first = match cell_endpoint(&pair[0], false)? {
                Classification::Decided(Some((parameter, _))) => parameter,
                Classification::Decided(None) => {
                    return Err(CurveError::Topology(
                        "selected rational overlap acquired an excluded start".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match cell_endpoint(&pair[1], true)? {
                Classification::Decided(Some((parameter, _))) => parameter,
                Classification::Decided(None) => {
                    return Err(CurveError::Topology(
                        "selected rational overlap acquired an excluded end".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (cusp_start, cusp_end) = if orientation == CurveOverlapOrientation2::Same {
                (first, second)
            } else {
                (second, first)
            };
            match cusp_start.cmp_by_refinement(&cusp_end, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {}
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::Topology(
                        "regular rational circle cell mapped to a zero cusp range".into(),
                    ));
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    return Err(CurveError::Topology(
                        "rational circle angular orientation disagreed with cusp ordering".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            covered_boundaries[index] = true;
            covered_boundaries[index + 1] = true;
            overlaps.push(BezierAlgebraicCuspSemicircleMappedOverlap2 {
                other_range: CurveParameterRange2::new_validated(
                    pair[0].parameter.clone().into(),
                    pair[1].parameter.clone().into(),
                ),
                cusp_start,
                cusp_end,
                orientation,
                parameter_map: BezierAlgebraicCuspSemicircleMappedOverlapMap2::Rational(
                    parameter_map.clone(),
                ),
                map_reversed: false,
            });
        }
        let mut contacts = Vec::new();
        // Closed overlap cells already own their boundary visits. A separate
        // visit at the same geometric point still has its own source parameter.
        for (index, boundary) in boundaries.iter().enumerate() {
            if covered_boundaries[index] {
                continue;
            }
            let Some((_, location)) = (match endpoint(boundary)? {
                Classification::Decided(endpoint) => endpoint,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }) else {
                continue;
            };
            if location == BezierAlgebraicCuspSemicircleContactLocation2::Interior {
                continue;
            }
            let point =
                match rational_point_evidence_at_parameter(other, &boundary.parameter, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            contacts.push(BezierAlgebraicCuspSemicircleRationalContact2 {
                other_parameter: CurveParameter2::from(boundary.parameter.clone()),
                point,
                tangent_cross_sign: RealSign::Zero,
                tangent_dot_sign: None,
                location,
            });
        }
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps },
        ))
    }

    /// Replays a rational curve whose complete selected-root image lies on the
    /// cusp supporting circle. Selected-half and angular-stationary roots form
    /// the complete cell boundary set; every retained open cell is therefore
    /// regular, monotone, and has one exact parameter orientation.
    pub(in crate::bezier_offset) fn replay_rational_circle_component(
        &self,
        other: &RationalBezier2,
        range: &CurveParameterRange2,
        selected_half_plane: BivariatePolynomial,
        diameter_side: BivariatePolynomial,
        radius_squared_denominator: BivariatePolynomial,
        angular_tangent: BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleRationalIntersections2>> {
        let envelope = match other.finite_discovery_envelope(range, policy)? {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = &envelope;
        let (lower, upper) = range
            .scalar_endpoints()
            .expect("the component envelope is represented");
        match other.denominator_sign(range) {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Err(CurveError::Topology(
                    "a coincident rational circle acquired a zero denominator".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let projected = |polynomial: &BivariatePolynomial| {
            algebraic_selected_reduced_fiber_parameters(
                polynomial,
                self.cusp_parameter(),
                range,
                policy,
            )
        };
        let selected_roots = match projected(&selected_half_plane)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let angular_roots = match projected(&angular_tangent)? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(roots)) => roots,
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection,
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };

        let angular_relation = Arc::new(angular_tangent);
        let mut boundaries = Vec::with_capacity(selected_roots.len() + angular_roots.len() + 2);
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(lower.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        boundaries.extend(angular_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Relation(
                    Arc::clone(&angular_relation),
                ),
                selected_relation: false,
            }
        }));
        boundaries.extend(selected_roots.into_iter().map(|parameter| {
            BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
                parameter,
                correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Map,
                selected_relation: true,
            }
        }));
        boundaries.push(BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
            parameter: BezierParameter2::Exact(upper.clone()),
            correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent,
            selected_relation: false,
        });
        let cusp_parameter = BezierParameter2::Algebraic(self.cusp_parameter().clone());
        let parameter_map = BezierAlgebraicCuspSemicircleRationalParameterMap2 {
            data: Arc::new(BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
                semicircle: self.clone(),
                curve: other.clone(),
                system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
                    cusp_parameter: cusp_parameter.clone(),
                    incidence: selected_half_plane.clone(),
                    diameter: BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(
                        diameter_side.clone(),
                    ),
                    radius_squared_denominator,
                },
                policy: policy.retained_object_policy(),
                parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2::default(),
            }),
        };

        let predicate_sign =
            |predicate: &BivariatePolynomial,
             boundary: &BezierAlgebraicCuspSemicircleRationalComponentBoundary2|
             -> CurveResult<Classification<RealSign>> {
                match &boundary.correlation {
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Map => {
                        algebraic_selected_correlated_predicate_sign(
                            &selected_half_plane,
                            predicate,
                            &cusp_parameter,
                            &boundary.parameter,
                            policy,
                        )
                    }
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::MapWithChordTangent {
                        ..
                    } => algebraic_selected_correlated_predicate_sign(
                        &selected_half_plane,
                        predicate,
                        &cusp_parameter,
                        &boundary.parameter,
                        policy,
                    ),
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Independent => {
                        signed_bivariate_at_parameter_pair(
                            predicate,
                            &cusp_parameter,
                            &boundary.parameter,
                            policy,
                        )
                    }
                    BezierAlgebraicCuspSemicircleRationalCorrelation2::Relation(incidence) => {
                        algebraic_selected_correlated_predicate_sign(
                            incidence,
                            predicate,
                            &cusp_parameter,
                            &boundary.parameter,
                            policy,
                        )
                    }
                }
            };
        self.publish_partitioned_rational_circle_component(
            other,
            parameter_map,
            boundaries,
            policy,
            |boundary| {
                if boundary.selected_relation {
                    Ok(Classification::Decided(RealSign::Zero))
                } else {
                    predicate_sign(&selected_half_plane, boundary)
                }
            },
            |boundary| predicate_sign(&diameter_side, boundary),
            |parameter| {
                signed_bivariate_at_parameter_pair(
                    angular_relation.as_ref(),
                    &cusp_parameter,
                    parameter,
                    policy,
                )
            },
        )
    }
}
