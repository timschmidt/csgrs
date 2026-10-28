//! Angular queries retain a certified point in its original joint field.

use std::cmp::Ordering;

use super::*;

impl BezierAlgebraicCuspSemicircle2 {
    /// A known endpoint factors the line/circle quadratic. Two incident
    /// endpoints already exhaust it; otherwise construct the other contact in
    /// the same selected field. Recheck the original circle and selected half,
    /// and retain finite-line clipping and transverse evidence separately.
    pub(super) fn shared_point_incident_chord_contacts(
        &self,
        chord: &BezierAlgebraicChord2,
        clip_to_finite_chord: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Vec<BezierAlgebraicCuspSemicircleRetainedChordContact2>>>>
    {
        if self.data.frame.rational().is_some() {
            return Ok(Classification::Decided(None));
        }
        let Some((normal_x, normal_y)) = self.represented_frame_unit_normal()? else {
            return Ok(Classification::Decided(None));
        };
        let center = match self.center_point_evidence(policy)? {
            Classification::Decided(center) => center,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if ![&center, chord.start(), chord.end()].iter().any(|point| {
            matches!(
                point.0,
                CurvePointData2::Polynomial(_) | CurvePointData2::RationalImage(_)
            )
        }) {
            return Ok(Classification::Decided(None));
        }
        let radius_squared = self.radial_distance() * self.radial_distance();
        let mut incidence = [false; 2];
        let mut undecided = None;
        for (index, point) in [chord.start(), chord.end()].into_iter().enumerate() {
            match common_point_circle_residual_sign(point, &center, &radius_squared, policy)? {
                Classification::Decided(sign) => incidence[index] = sign == RealSign::Zero,
                Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                    return Ok(Classification::Uncertain(
                        UncertaintyReason::BudgetExhausted,
                    ));
                }
                Classification::Uncertain(reason) => undecided = Some(reason),
            }
        }
        if !incidence.contains(&true) {
            return Ok(match undecided {
                Some(reason) => Classification::Uncertain(reason),
                None => Classification::Decided(None),
            });
        }
        let reverse_sign = |sign| match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Positive => RealSign::Negative,
            RealSign::Zero => RealSign::Zero,
        };
        let mut candidates = Vec::with_capacity(2);
        if incidence == [true, true] {
            // (A-C).(B-A) = -|B-A|^2/2, with the opposite sign at B.
            let start_cross = if self.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            candidates.push((
                chord.start().clone(),
                start_cross,
                Some(chord.start_parameter()),
            ));
            candidates.push((
                chord.end().clone(),
                reverse_sign(start_cross),
                Some(chord.end_parameter()),
            ));
        } else {
            let (incident, parameter) = if incidence[0] {
                (chord.start(), chord.start_parameter())
            } else {
                (chord.end(), chord.end_parameter())
            };
            let (second, dot_sign) = match common_point_circle_second_line_contact(
                incident,
                &center,
                chord.start(),
                chord.end(),
                policy,
            )? {
                Classification::Decided(contact) => contact,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let first_cross = if self.is_clockwise() {
                dot_sign
            } else {
                reverse_sign(dot_sign)
            };
            candidates.push((incident.clone(), first_cross, Some(parameter)));
            if dot_sign != RealSign::Zero {
                match common_point_circle_residual_sign(&second, &center, &radius_squared, policy)?
                {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(_) => {
                        return Err(CurveError::Topology(
                            "a derived line contact does not satisfy its original circle".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                candidates.push((second, reverse_sign(first_cross), None));
            }
        }
        let mut contacts = Vec::with_capacity(candidates.len());
        for (point, tangent_cross_sign, endpoint_parameter) in candidates {
            let (cusp_parameter, complementary) = match self.selected_incident_point_parameter(
                &point,
                &center,
                (&normal_x, &normal_y),
                policy,
            )? {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if complementary {
                continue;
            }
            let chord_parameter = match endpoint_parameter {
                Some(parameter) => parameter,
                None if clip_to_finite_chord => {
                    match chord.parameter_at_certified_point(point.clone(), policy)? {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                None => chord.parameter_at_certified_support_point(point.clone(), policy)?,
            };
            contacts.push(BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                cusp_parameter,
                chord_parameter,
                point,
                tangent_cross_sign,
            });
        }
        Ok(Classification::Decided(Some(contacts)))
    }

    pub(super) fn represented_frame_unit_normal(&self) -> CurveResult<Option<(Real, Real)>> {
        Ok(match self.data.frame.chord_normal() {
            Some(frame) => frame
                .anchor
                .certified_unit_tangent()
                .or_else(|| {
                    frame
                        .anchor
                        .certified_axis_direction()
                        .map(BezierAlgebraicChordAxisDirection2::unit_tangent)
                })
                .map(|(x, y)| (-y, x)),
            None => self.data.frame.represented_unit_normal()?,
        })
    }

    /// The caller has already proved circle incidence. The signed tangent
    /// projection selects the closed half chart; a zero projection is a
    /// diameter endpoint and retains the original chart's canonical ownership.
    pub(super) fn selected_incident_point_parameter(
        &self,
        point: &CurvePoint2,
        center: &CurvePoint2,
        (normal_x, normal_y): (&Real, &Real),
        policy: &CurveContext,
    ) -> CurveResult<Classification<(BezierAlgebraicCuspSemicircleParameter2, bool)>> {
        // A single projection can cancel unrelated fields. Retain a shared
        // authority for both coordinates before promising every angular query.
        // Strictly replay point dependencies here; a qualified source cannot
        // become certified merely because one local sign happens to be exact.
        let strict = policy.strict_counterpart();
        let radial = match common_point_displacement(point, center, &strict)? {
            Classification::Decided(radial) => radial,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for form in &radial {
            if let Classification::Uncertain(reason) =
                point_rational_denominator_sign(&form.source, &form.denominator, &strict)?
            {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let radius_sign = match real_sign(self.radial_distance(), policy) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let tangent = match radial_order(
            &radial,
            &(-self.turn_sign() * normal_y),
            &(self.turn_sign() * normal_x),
            policy,
        )? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if tangent == Ordering::Equal {
            let normal = match radial_order(&radial, normal_x, normal_y, policy)? {
                Classification::Decided(Ordering::Equal) => {
                    return Err(CurveError::InvalidCurveParameter);
                }
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            return Ok(Classification::Decided((
                BezierAlgebraicCuspSemicircleParameter2::Exact(if normal == radius_sign {
                    Real::zero()
                } else {
                    Real::one()
                }),
                false,
            )));
        }

        let complementary = tangent != radius_sign;
        let semicircle = if complementary {
            self.complementary_half()
        } else {
            self.clone()
        };
        let policy = policy
            .retained_object_policy_with_dependencies(semicircle.data.frame.evidence_policy());
        Ok(Classification::Decided((
            BezierAlgebraicCuspSemicircleParameter2::Mapped(Arc::new(
                BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint {
                    semicircle,
                    point: point.clone(),
                    radial: Arc::new(radial),
                    policy,
                },
            )),
            complementary,
        )))
    }
}

fn radial_order(
    radial: &[PointFormSource2; 2],
    a: &Real,
    b: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Ordering>> {
    let form = point_form_linear_combination(radial, a, b);
    Ok(
        point_rational_form_sign(&form.source, &form.numerator, &form.denominator, policy)?.map(
            |sign| match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            },
        ),
    )
}

impl BezierAlgebraicCuspSemicircleMappedParameterData2 {
    pub(super) fn incident_point_order_to_real(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        let Self::IncidentPoint {
            semicircle, radial, ..
        } = self
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // This carrier contains only interior parameters. The ordinary
        // rational chart's formula is used only within its closed domain.
        for (boundary, outside, order) in [
            (Real::zero(), Ordering::Less, Ordering::Greater),
            (Real::one(), Ordering::Greater, Ordering::Less),
        ] {
            match compare_reals(parameter, &boundary, policy) {
                Some(value) if value == outside || value == Ordering::Equal => {
                    return Ok(Classification::Decided(order));
                }
                Some(_) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
        }
        let Some((normal_x, normal_y)) = semicircle.represented_frame_unit_normal()? else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        // For n=(P-C).N and b=turn*cross(N,P-C), the sign of
        // r*((1-2v)*b-2v(1-v)*n) orders the selected angle against v.
        // Both angles are interior to one half, so no antipodal zero is lost.
        let tangent = (Real::one() - Real::from(2_i8) * parameter) * semicircle.turn_sign();
        let normal = Real::from(2_i8) * parameter * (Real::one() - parameter);
        let order = radial_order(
            radial,
            &(-&tangent * &normal_y - &normal * &normal_x),
            &(&tangent * &normal_x - &normal * &normal_y),
            policy,
        )?;
        Ok(match real_sign(semicircle.radial_distance(), policy) {
            Some(RealSign::Positive) => order,
            Some(RealSign::Negative) => order.map(Ordering::reverse),
            _ => Classification::Uncertain(UncertaintyReason::RealSign),
        })
    }
}

impl BezierAlgebraicCuspSemicircleParameter2 {
    pub(super) fn incident_points_order(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<Ordering>>> {
        let (Self::Mapped(first), Self::Mapped(second)) = (self, other) else {
            return Ok(None);
        };
        if !matches!(
            first.as_ref(),
            BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint { .. }
        ) && !matches!(
            second.as_ref(),
            BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint { .. }
        ) {
            return Ok(None);
        }
        let circle = first.semicircle_carrier();
        if circle != second.semicircle_carrier() {
            return Ok(None);
        }
        let (Some(first), Some(second), Some((x, y))) = (
            first.retained_point_evidence(),
            second.retained_point_evidence(),
            circle.represented_frame_unit_normal()?,
        ) else {
            return Ok(None);
        };
        // cos(theta) strictly decreases on a selected closed half. The center
        // cancels from this comparison, including independently replayed
        // definitions of the same point and arbitrarily close distinct cuts.
        let order = common_point_linear_order(first, second, &x, &y, policy)?;
        Ok(Some(match real_sign(circle.radial_distance(), policy) {
            Some(RealSign::Positive) => order.map(Ordering::reverse),
            Some(RealSign::Negative) => order,
            _ => Classification::Uncertain(UncertaintyReason::RealSign),
        }))
    }
}

#[cfg(all(test, feature = "svg"))]
mod tests {
    use super::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn selected_center_and_point() -> (CurvePoint2, CurvePoint2) {
        let mut coefficients = vec![Real::zero(); 8];
        coefficients[0] = (-1).into();
        coefficients[7] = 2.into();
        let center = crate::CurvePointDefinition2::PolynomialFiber {
            equation: BivariatePolynomial::new(vec![
                vec![
                    (-2).into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                ],
                vec![1.into()],
            ]),
            parameter: BezierParameter2::Algebraic(
                BezierAlgebraicParameter2::try_isolate(
                    crate::BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
                    BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
                )
                .unwrap(),
            ),
            root: hypersolve::IsolatedRootInterval {
                lower: 1.into(),
                upper: 2.into(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap();
        let point = center
            .rational_image(&crate::RationalMap2::new(
                [
                    BivariatePolynomial::new(vec![
                        vec![q(-4, 5)],
                        vec![q(19, 5)],
                        vec![q(-16, 5)],
                        vec![2.into()],
                    ]),
                    BivariatePolynomial::new(vec![
                        vec![q(3, 5), 1.into()],
                        vec![q(2, 5), (-2).into()],
                        vec![q(-8, 5), 2.into()],
                    ]),
                ],
                BivariatePolynomial::new(vec![vec![1.into()], vec![(-2).into()], vec![2.into()]]),
            ))
            .unwrap();
        (center, point)
    }

    fn circle(center: CurvePoint2, policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
        let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(
            Point2::from_values(0, 0).into(),
            Point2::from_values(3, 4).into(),
            policy,
        )
        .unwrap() else {
            panic!("nonzero exact anchor")
        };
        let Classification::Decided(Some(circle)) =
            BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
                center,
                chord,
                Real::one(),
                true,
                policy,
            )
            .unwrap()
        else {
            panic!("nonzero circle")
        };
        circle
    }

    #[test]
    fn selected_point_angular_authority_keeps_inherited_frame_policy() {
        let (center, point) = selected_center_and_point();
        let qualified =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                policy.observe_approximate_512();
                Ok::<_, CurveError>(circle(center.clone(), policy))
            })
            .unwrap()
            .into_value();
        let parameter =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                qualified.parameter_at_certified_incident_point(&point, policy)
            })
            .unwrap()
            .into_value();
        let Classification::Decided(parameter) = parameter else {
            panic!("retained exact point")
        };
        let parameter = parameter.as_algebraic_cusp().unwrap();
        assert!(parameter.validate_policy(&CurveContext::STRICT).is_err());
        assert!(matches!(
            qualified.native_parameter_definition(parameter),
            Err(CurveError::IncompatiblePredicatePolicy)
        ));
        let strict = circle(center.clone(), &CurveContext::STRICT);
        let Classification::Decided(inherited) = strict
            .parameter_at_certified_incident_point(&point, &CurveContext::APPROXIMATE_512)
            .unwrap()
        else {
            panic!("retained current operation policy")
        };
        assert!(
            inherited
                .as_algebraic_cusp()
                .unwrap()
                .validate_policy(&CurveContext::STRICT)
                .is_err()
        );
        let fresh =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                strict.parameter_at_certified_incident_point(&point, policy)
            })
            .unwrap();
        assert_eq!(fresh.certainty, crate::CurveCertainty::Certified);
        let Classification::Decided(parameter) = fresh.into_value() else {
            panic!("strict dependencies")
        };
        parameter
            .as_algebraic_cusp()
            .unwrap()
            .validate_policy(&CurveContext::STRICT)
            .unwrap();
    }

    #[test]
    fn selected_point_angular_replay_checks_incidence_half_and_point_policy() {
        let (center, point) = selected_center_and_point();
        let strict = circle(center.clone(), &CurveContext::STRICT);
        let Classification::Decided(parameter) = strict
            .replay_parameter_definition(
                CircleParameterDefinition2::IncidentPoint(point.clone()),
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("checked angular point")
        };
        assert!(
            matches!(parameter, BezierAlgebraicCuspSemicircleParameter2::Mapped(ref data) if matches!(data.as_ref(), BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint { .. }))
        );
        assert!(matches!(
            strict.replay_parameter_definition(
                CircleParameterDefinition2::IncidentPoint(center.clone()),
                &CurveContext::STRICT
            ),
            Err(CurveError::InvalidCurveParameter)
        ));
        assert!(matches!(
            strict.complementary_half().replay_parameter_definition(
                CircleParameterDefinition2::IncidentPoint(point.clone()),
                &CurveContext::STRICT
            ),
            Err(CurveError::InvalidBezierRange)
        ));
        let qualified =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                policy.observe_approximate_512();
                Ok::<_, CurveError>(CurvePoint2::from(BezierSimilarityPoint2::new(
                    point.clone(),
                    Similarity2::try_from_real_affine(
                        1.into(),
                        0.into(),
                        0.into(),
                        1.into(),
                        0.into(),
                        0.into(),
                    )
                    .unwrap(),
                    policy,
                )))
            })
            .unwrap()
            .into_value();
        assert!(matches!(
            strict
                .selected_incident_point_parameter(
                    &qualified,
                    &center,
                    (&q(-4, 5), &q(3, 5)),
                    &CurveContext::APPROXIMATE_512
                )
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        ));
    }

    #[test]
    fn circle_derivatives_keep_frame_qualification_after_center_cancellation() {
        let (center, _) = selected_center_and_point();
        let qualified =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                policy.observe_approximate_512();
                Ok::<_, CurveError>(BezierAlgebraicCuspSemicircleFragment2::full(
                    circle(center.clone(), policy),
                    policy,
                ))
            })
            .unwrap()
            .into_value();
        let curve = crate::Curve2::from_retained_fragment(
            crate::BezierSplitFragment2::AlgebraicCuspSemicircle(qualified),
        );
        let at = CurveParameter2::from(Real::zero());
        assert!(
            curve
                .derivative_at_with_policy(&at, &CurveContext::STRICT)
                .is_err()
        );
        let retained = curve
            .derivative_at_with_policy(&at, &CurveContext::APPROXIMATE_512)
            .unwrap()
            .into_value()
            .coordinate_image()
            .unwrap();
        // The center cancels mathematically, but its frame was selected using
        // qualified evidence. A vector cannot become strict replay authority.
        assert!(retained.definition().is_err());
        let strict = crate::Curve2::from_retained_fragment(
            crate::BezierSplitFragment2::AlgebraicCuspSemicircle(
                BezierAlgebraicCuspSemicircleFragment2::full(
                    circle(center, &CurveContext::STRICT),
                    &CurveContext::STRICT,
                ),
            ),
        );
        let fresh = strict
            .derivative_at_with_policy(&at, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(fresh.certainty, crate::CurveCertainty::Certified);
        assert!(
            fresh
                .into_value()
                .coordinate_image()
                .unwrap()
                .definition()
                .is_ok()
        );
    }
}
