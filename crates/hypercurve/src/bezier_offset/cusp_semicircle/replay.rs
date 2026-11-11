//! Checked native definitions for a selected circle and its angular contacts.

use super::*;

pub(crate) struct CircleChordNormalDefinition2 {
    pub(crate) center: CurvePoint2,
    pub(crate) anchor: BezierAlgebraicChord2,
    pub(crate) radial_distance: Real,
    pub(crate) clockwise: bool,
}

pub(crate) struct CircleParallelNormalDefinition2 {
    pub(crate) center_support: BezierParallel2,
    pub(crate) center_parameter: CurveParameter2,
    pub(crate) radial_distance: Real,
    pub(crate) clockwise: bool,
}

pub(crate) enum CircleParameterDefinition2 {
    Exact(Real),
    IncidentPoint(CurvePoint2),
    ChordNormal {
        chord: BezierAlgebraicChord2,
        point: CurvePoint2,
        radial_distance: Real,
    },
    ParallelNormal {
        parallel: BezierParallel2,
        parameter: CurveParameter2,
        radial_distance: Real,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
    },
}

impl BezierAlgebraicCuspSemicircle2 {
    pub(crate) fn replay_storage_key(&self) -> *const () {
        Arc::as_ptr(&self.data).cast()
    }

    pub(crate) fn native_chord_normal_definition(&self) -> Option<CircleChordNormalDefinition2> {
        let frame = self.data.frame.chord_normal()?;
        CurveContext::STRICT
            .accepts_retained_policy(frame.policy)
            .then(|| CircleChordNormalDefinition2 {
                center: frame.center.clone(),
                anchor: frame.anchor.clone(),
                radial_distance: self.data.radial_distance.clone(),
                clockwise: self.data.clockwise,
            })
    }

    pub(crate) fn native_parallel_normal_definition(
        &self,
    ) -> Option<CircleParallelNormalDefinition2> {
        let frame = self.data.frame.parallel_normal()?;
        CurveContext::STRICT
            .accepts_retained_policy(frame.policy)
            .then(|| CircleParallelNormalDefinition2 {
                center_support: frame.center_support.clone(),
                center_parameter: frame.center_parameter.clone(),
                radial_distance: self.data.radial_distance.clone(),
                clockwise: self.data.clockwise,
            })
    }

    pub(crate) fn replay_parallel_normal_definition(
        definition: CircleParallelNormalDefinition2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        // Serialized selectors are inputs to the regular source-frame owner,
        // never evidence that its denominator and tangent are nonzero.
        match Self::from_selected_parallel_normal(
            definition.center_support,
            definition.center_parameter,
            definition.radial_distance,
            definition.clockwise,
            policy,
        )? {
            Classification::Decided(Some(circle)) => Ok(Classification::Decided(circle)),
            Classification::Decided(None) => Err(CurveError::InvalidCurveParameter),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn replay_chord_normal_definition(
        definition: CircleChordNormalDefinition2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        // The graph reader has already replayed the point and chord through
        // their strict owners. This constructor checks the frame/radius and
        // retains those immutable dependencies.
        match Self::from_retained_center_and_chord_normal(
            definition.center,
            definition.anchor,
            definition.radial_distance,
            definition.clockwise,
            policy,
        )? {
            Classification::Decided(Some(circle)) => Ok(Classification::Decided(circle)),
            Classification::Decided(None) => Err(CurveError::InvalidCurveParameter),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Replays an overlap boundary through its original endpoint authority.
    /// Preserve classified failures from chart inversion instead of treating
    /// an exhausted proof as an unsupported serialization variant.
    pub(crate) fn native_parameter_definition(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
    ) -> CurveResult<Classification<Option<CircleParameterDefinition2>>> {
        let policy = CurveContext::STRICT;
        parameter.validate_policy(&policy)?;
        if let Some(definition) = self.retained_parameter_definition(parameter) {
            return Ok(Classification::Decided(Some(definition)));
        }
        let BezierAlgebraicCuspSemicircleParameter2::Mapped(data) = parameter else {
            return Ok(Classification::Decided(None));
        };
        if data.semicircle_carrier() != self || data.coincident_pair_endpoint_source().is_none() {
            return Ok(Classification::Decided(None));
        }
        let point = match parameter.coincident_point_evidence(self, &policy)? {
            Classification::Decided(Some(point)) => point,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let local = decided!(
            self.parameter_at_certified_incident_point(&point, &policy)?,
            Ok
        );
        if local.is_algebraic_cusp_complement() {
            return Err(CurveError::InvalidBezierRange);
        }
        // Use the same canonical chart as the reader, so the first export
        // and a fresh replay have identical definitions. Otherwise retain P
        // and let the existing incident-point reader re-prove its equations.
        Ok(Classification::Decided(Some(
            match local.as_algebraic_cusp() {
                Some(BezierAlgebraicCuspSemicircleParameter2::Exact(value)) => {
                    CircleParameterDefinition2::Exact(value.clone())
                }
                Some(BezierAlgebraicCuspSemicircleParameter2::Mapped(_)) => {
                    CircleParameterDefinition2::IncidentPoint(point)
                }
                None => return Err(CurveError::InvalidCurveParameter),
            },
        )))
    }

    fn retained_parameter_definition(
        &self,
        parameter: &BezierAlgebraicCuspSemicircleParameter2,
    ) -> Option<CircleParameterDefinition2> {
        match parameter {
            BezierAlgebraicCuspSemicircleParameter2::Exact(value) => {
                Some(CircleParameterDefinition2::Exact(value.clone()))
            }
            BezierAlgebraicCuspSemicircleParameter2::Mapped(parameter) => {
                if let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedParallelContact {
                    semicircle,
                    parallel,
                    parameter,
                    location,
                    radial_product_sign,
                    policy,
                    ..
                } = parameter.as_ref()
                    && semicircle == self
                    && self.data.frame.parallel_normal().is_some()
                    && CurveContext::STRICT.accepts_retained_policy(*policy)
                {
                    let radial_distance = match radial_product_sign {
                        RealSign::Positive => self.radial_distance().clone(),
                        RealSign::Negative => -self.radial_distance().clone(),
                        RealSign::Zero => return None,
                    };
                    return Some(CircleParameterDefinition2::ParallelNormal {
                        parallel: parallel.clone(),
                        parameter: parameter.clone(),
                        radial_distance,
                        location: *location,
                    });
                }
                if let BezierAlgebraicCuspSemicircleMappedParameterData2::IncidentPoint {
                    semicircle,
                    point,
                    policy,
                    ..
                } = parameter.as_ref()
                    && semicircle == self
                    && CurveContext::STRICT.accepts_retained_policy(*policy)
                {
                    return Some(CircleParameterDefinition2::IncidentPoint(point.clone()));
                }
                if let BezierAlgebraicCuspSemicircleMappedParameterData2::Chord { map, contact } =
                    parameter.as_ref()
                    && map.data.semicircle == *self
                    && map.data.finite_chord_domain
                    && contact.chord_location == BezierAlgebraicCuspSemicircleContactLocation2::End
                    && CurveContext::STRICT.accepts_retained_policy(map.data.policy)
                    && let Some(frame) = self.data.frame.chord_normal()
                    && (map.data.chord.start().shares_storage(&frame.center)
                        || map
                            .data
                            .chord
                            .start()
                            .same_point(&frame.center, &CurveContext::STRICT)
                            == Classification::Decided(true))
                {
                    // Point inversion introduced precisely C -> P. Preserve
                    // P and its independent field, then re-prove incidence on
                    // replay instead of persisting the derived root cache.
                    return Some(CircleParameterDefinition2::IncidentPoint(
                        map.data.chord.end().clone(),
                    ));
                }
                let BezierAlgebraicCuspSemicircleMappedParameterData2::SelectedChordNormalContact {
                    semicircle,
                    anchor_tangent: BezierSelectedChordNormalAnchor2::RetainedChord(anchor),
                    chord,
                    radial_product_sign,
                    point,
                    policy,
                } = parameter.as_ref()
                else {
                    return None;
                };
                let frame = self.data.frame.chord_normal()?;
                if semicircle != self
                    || anchor != &frame.anchor
                    || !CurveContext::STRICT.accepts_retained_policy(*policy)
                {
                    return None;
                }
                let radial_distance = match radial_product_sign {
                    RealSign::Positive => self.radial_distance().clone(),
                    RealSign::Negative => -self.radial_distance().clone(),
                    RealSign::Zero => return None,
                };
                Some(CircleParameterDefinition2::ChordNormal {
                    chord: chord.clone(),
                    point: point.clone(),
                    radial_distance,
                })
            }
        }
    }

    pub(crate) fn replay_parameter_definition(
        &self,
        definition: CircleParameterDefinition2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        match definition {
            CircleParameterDefinition2::ParallelNormal {
                parallel,
                parameter,
                radial_distance,
                location,
            } => self.replay_parallel_normal_contact(
                parallel,
                parameter,
                radial_distance,
                location,
                policy,
            ),
            CircleParameterDefinition2::IncidentPoint(point) => {
                match self.retained_point_incidence_sign(&point, policy)? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let parameter = decided!(
                    self.parameter_at_certified_incident_point(&point, policy)?,
                    Ok
                );
                if parameter.is_algebraic_cusp_complement() {
                    return Err(CurveError::InvalidBezierRange);
                }
                parameter
                    .as_algebraic_cusp()
                    .cloned()
                    .map(Classification::Decided)
                    .ok_or(CurveError::InvalidCurveParameter)
            }
            CircleParameterDefinition2::Exact(value) => {
                let parameter = BezierAlgebraicCuspSemicircleParameter2::Exact(value);
                for (boundary, invalid) in [
                    (Real::zero(), std::cmp::Ordering::Less),
                    (Real::one(), std::cmp::Ordering::Greater),
                ] {
                    match parameter.order_to_real(&boundary, policy)? {
                        Classification::Decided(order) if order == invalid => {
                            return Err(CurveError::InvalidBezierRange);
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                        Classification::Decided(_) => {}
                    }
                }
                Ok(Classification::Decided(parameter))
            }
            CircleParameterDefinition2::ChordNormal {
                chord,
                point,
                radial_distance,
            } => {
                let Some(frame) = self.data.frame.chord_normal() else {
                    return Err(CurveError::InvalidCurveParameter);
                };
                // A native record states this signed relation. The modeling
                // solver's two-sided fallback search is not its replay rule.
                let expected = chord.normal_displaced_point_evidence(
                    frame.center.clone(),
                    radial_distance.clone(),
                    policy,
                )?;
                match expected.same_point(&point, policy) {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(CurveError::InvalidCurveParameter);
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                // Reuse only the incidence just proved above. The existing
                // circle owner still checks the radius and selected half.
                self.certified_chord_normal_contact_parameter(
                    BezierSelectedChordNormalAnchor2::RetainedChord(frame.anchor.clone()),
                    chord,
                    point,
                    radial_distance,
                    true,
                    policy,
                )
            }
        }
    }

    fn replay_parallel_normal_contact(
        &self,
        other: BezierParallel2,
        other_parameter: CurveParameter2,
        radial_distance: Real,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameter2>> {
        let frame = self
            .data
            .frame
            .parallel_normal()
            .ok_or(CurveError::InvalidCurveParameter)?;
        if !policy.accepts_retained_policy(frame.policy) {
            return Err(CurveError::IncompatiblePredicatePolicy);
        }
        let strict = policy.strict_counterpart();
        match real_sign(
            &(&radial_distance * &radial_distance
                - self.radial_distance() * self.radial_distance()),
            &strict,
        ) {
            Some(RealSign::Zero) => {}
            Some(_) => return Err(CurveError::InvalidCurveParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_sign = match real_sign(&radial_distance, &strict) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => return Err(CurveError::InvalidCurveParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        // P=Q(b)+d*N_Q(b), C=P-s*N_Q(b). This is a definition on the
        // original source normal, not an offset-composition commutation rule.
        let other_center = other.with_distance(other.distance() - &radial_distance);
        let first_range = decided!(
            frame
                .center_support
                .regular_source_envelope_at(&frame.center_parameter, &strict)?,
            Ok
        );
        let second_range = decided!(
            other_center.regular_source_envelope_at(&other_parameter, &strict)?,
            Ok
        );
        let result = decided!(
            frame.center_support.parallel_intersections_in_domain(
                &other_center,
                [&first_range, &second_range].map(|range| CurveParameterDomain2::new(range, None)),
                ParameterComponentQuery2::AllComponents(None),
                &strict,
            )?,
            Ok
        );
        let (intersections, components) = result.into_parts();
        let mut uncertain = None;
        let mut matched_contact = None;
        for contact in intersections.contacts() {
            match parallel_pair_contact_parameters_are_retained(
                std::slice::from_ref(contact),
                &frame.center_parameter,
                &other_parameter,
                &strict,
            )? {
                Classification::Decided(true) => {
                    matched_contact = Some(contact);
                    break;
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => uncertain = Some(reason),
            }
        }
        let mut incident = matched_contact.is_some();
        if !incident {
            for component in &components {
                match component.contains_pair(&frame.center_parameter, &other_parameter, &strict)? {
                    Classification::Decided(true) => {
                        incident = true;
                        break;
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => uncertain = Some(reason),
                }
            }
        }
        if !incident {
            if let Some(reason) = uncertain {
                return Ok(Classification::Uncertain(reason));
            }
            if !intersections.is_complete() {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            return Err(CurveError::InvalidCurveParameter);
        }

        // A replayed contact already owns exact parallel-tangent signs.
        // Transport them back through nonzero derivative scales when possible.
        // A cusp of either center locus instead uses the source-tangent owner.
        let mut tangents = None;
        let mut transport_uncertainty = None;
        if let Some(contact) = matched_contact
            && let (Some(cross), Some(dot)) =
                (contact.tangent_cross_sign(), contact.tangent_dot_sign())
        {
            let first = frame
                .center_support
                .parallel_derivative_scale_sign(&frame.center_parameter, &strict)?;
            if let Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)) =
                first
            {
                match other_center.parallel_derivative_scale_sign(&other_parameter, &strict)? {
                    Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)) => {
                        let scale = product_sign(first, second);
                        tangents = Some((product_sign(cross, scale), product_sign(dot, scale)));
                    }
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Uncertain(reason) => transport_uncertainty = Some(reason),
                }
            } else if let Classification::Uncertain(reason) = first {
                transport_uncertainty = Some(reason);
            }
        }
        let (cross, dot) = if let Some(tangents) = tangents {
            tangents
        } else {
            let replay = (|| {
                let first = decided!(
                    promote_curve_region_bezier_parameter(&frame.center_parameter, &strict)?,
                    Ok
                );
                let second = decided!(
                    promote_curve_region_bezier_parameter(&other_parameter, &strict)?,
                    Ok
                );
                frame
                    .center_support
                    .source_tangent_pair_cross_and_dot_signs(&first, &other, &second, &strict)
            })()?;
            match replay {
                Classification::Decided(tangents) => tangents,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(
                        transport_uncertainty.unwrap_or(reason),
                    ));
                }
            }
        };
        let circle_radial_sign = match real_sign(self.radial_distance(), &strict) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => return Err(CurveError::InvalidCurveParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let radial_product_sign = product_sign(circle_radial_sign, radial_sign);
        let half = product_sign(
            product_sign(radial_product_sign, cross),
            if self.is_clockwise() {
                RealSign::Negative
            } else {
                RealSign::Positive
            },
        );
        // This decision consumes replayed source-tangent signs. The existing
        // chord-frame location method consumes a different predicate authority.
        // In particular, the serialized location is never its own proof.
        let actual_location = match half {
            RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            RealSign::Negative => return Err(CurveError::InvalidBezierRange),
            RealSign::Zero => match product_sign(radial_product_sign, dot) {
                RealSign::Positive => BezierAlgebraicCuspSemicircleContactLocation2::Start,
                RealSign::Negative => BezierAlgebraicCuspSemicircleContactLocation2::End,
                RealSign::Zero => {
                    return Err(CurveError::Topology(
                        "a regular native normal contact lost its radial direction".into(),
                    ));
                }
            },
        };
        if location != actual_location {
            return Err(CurveError::InvalidCurveParameter);
        }
        // Preserve the original source and parameter definitions, not the
        // independently discovered roots used to establish their equal values.
        self.certified_selected_parallel_contact_parameter(
            other,
            other_parameter,
            actual_location,
            radial_sign,
            cross,
            dot,
            policy,
        )
    }
}

impl BezierAlgebraicCuspSemicircleParameter2 {
    pub(crate) fn replay_storage_key(&self) -> Option<*const ()> {
        match self {
            Self::Exact(_) => None,
            Self::Mapped(parameter) => Some(Arc::as_ptr(parameter).cast()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::HomogeneousControl2;
    use crate::test_support::decided_ok;

    fn parallel_frame(
        radius: Real,
        clockwise: bool,
        policy: &CurveContext,
    ) -> BezierAlgebraicCuspSemicircle2 {
        let source = QuadraticBezier2::new(pt!(-1, 0), pt!(0, 0), pt!(1, 0));
        decided_ok(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                source.parallel_left(r!(0)).unwrap(),
                r!(1 / 2).into(),
                radius,
                clockwise,
                policy,
            ),
        )
        .unwrap()
    }

    fn vertical_parallel(x: Real) -> BezierParallel2 {
        QuadraticBezier2::new(
            Point2::new(x.clone(), r!(-1)),
            Point2::new(x.clone(), r!(0)),
            Point2::new(x, r!(1)),
        )
        .parallel_left(r!(0))
        .unwrap()
    }

    #[test]
    fn replay_selected_normal_contact_proves_both_sides_and_native_continuation() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for radius in [r!(1), r!(-1)] {
                for clockwise in [false, true] {
                    let circle = parallel_frame(radius.clone(), clockwise, &policy);
                    let signed_radius = if clockwise {
                        -radius.clone()
                    } else {
                        radius.clone()
                    };
                    let expected: CurvePoint2 = Point2::new(-signed_radius.clone(), r!(0)).into();
                    let parameter = decided_ok(circle.replay_parameter_definition(
                        CircleParameterDefinition2::ParallelNormal {
                            parallel: vertical_parallel(-signed_radius.clone()),
                            parameter: r!(1 / 2).into(),
                            radial_distance: signed_radius,
                            location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                        },
                        &policy,
                    ));
                    assert_eq!(
                        decided_ok(parameter.order_to_real(&r!(1 / 2), &policy)),
                        std::cmp::Ordering::Equal
                    );
                    let point =
                        decided_ok(parameter.coincident_point_evidence(&circle, &policy)).unwrap();
                    assert_eq!(
                        point.same_point(&expected, &policy),
                        Classification::Decided(true)
                    );
                    assert!(matches!(
                        decided_ok(circle.native_parameter_definition(&parameter)),
                        Some(CircleParameterDefinition2::ParallelNormal { .. })
                    ));
                    let fragment = decided_ok(BezierAlgebraicCuspSemicircleFragment2::try_new(
                        circle,
                        BezierAlgebraicCuspSemicircleParameter2::Exact(r!(0)),
                        parameter,
                        false,
                        &policy,
                    ));
                    let geometry = crate::SvgGeometry2::new(
                        crate::CurveRegion2::empty(),
                        Vec::new(),
                        vec![crate::CurvePath2::try_new(vec![fragment.into()]).unwrap()],
                    );
                    let text = crate::export_svg_document(&geometry).unwrap();
                    let restored = crate::import_svg_document(&text).unwrap();
                    let curve = &restored.paths()[0].curves()[0];
                    assert_eq!(
                        curve.end().same_point(&expected, &policy),
                        Classification::Decided(true)
                    );
                    // The rational half-circle chart at u=1/4 has coefficients
                    // 4/5 in the anchor normal and 3/5 in its turn direction.
                    let turn = if clockwise { r!(-1) } else { r!(1) };
                    let interior: CurvePoint2 =
                        Point2::new(-turn * &radius * r!(3 / 5), &radius * r!(4 / 5)).into();
                    assert_eq!(
                        curve
                            .point_at(&r!(1 / 4).into())
                            .unwrap()
                            .same_point(&interior, &policy),
                        Classification::Decided(true)
                    );
                    let crate::CurvePointLocations2::Locations(locations) =
                        curve.point_locations(&interior).unwrap()
                    else {
                        panic!("one exact interior point survives native replay");
                    };
                    assert_eq!(locations.len(), 1);
                    let (first, second) = curve
                        .split_at(locations[0].parameter().unwrap().clone())
                        .unwrap();
                    assert_eq!(
                        first.end().same_point(&interior, &policy),
                        Classification::Decided(true)
                    );
                    assert_eq!(
                        second.start().same_point(&interior, &policy),
                        Classification::Decided(true)
                    );
                    assert_eq!(crate::export_svg_document(&restored).unwrap(), text);
                }
            }
        }
    }

    #[test]
    fn replay_selected_normal_contact_rejects_forged_location_radius_side_and_parameter() {
        let circle = parallel_frame(r!(1), false, &CurveContext::STRICT);
        for (location, radial_distance, parameter) in [
            (
                BezierAlgebraicCuspSemicircleContactLocation2::Start,
                r!(1),
                r!(1 / 2),
            ),
            (
                BezierAlgebraicCuspSemicircleContactLocation2::End,
                r!(1),
                r!(1 / 2),
            ),
            (
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                r!(-1),
                r!(1 / 2),
            ),
            (
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                r!(2),
                r!(1 / 2),
            ),
            (
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                r!(1),
                r!(1 / 4),
            ),
        ] {
            assert!(matches!(
                circle.replay_parameter_definition(
                    CircleParameterDefinition2::ParallelNormal {
                        parallel: vertical_parallel(r!(-1)),
                        parameter: parameter.into(),
                        radial_distance,
                        location,
                    },
                    &CurveContext::STRICT,
                ),
                Err(CurveError::InvalidCurveParameter)
            ));
        }
        let outside = circle.replay_parameter_definition(
            CircleParameterDefinition2::ParallelNormal {
                parallel: vertical_parallel(r!(1)),
                parameter: r!(1 / 2).into(),
                radial_distance: r!(-1),
                location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            },
            &CurveContext::STRICT,
        );
        assert!(matches!(outside, Err(CurveError::InvalidBezierRange)));
    }

    #[test]
    fn replay_selected_normal_contact_retains_continuous_center_correspondence() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = parallel_frame(r!(1), false, &policy);
            // The second center locus is the same infinite line with a
            // different parameter scale. The selected pair, not an arbitrary
            // root in a box, determines this replayed endpoint.
            let source = QuadraticBezier2::new(pt!(-2, 0), pt!(0, 0), pt!(2, 0));
            for (distance, location, endpoint) in [
                (
                    r!(1),
                    BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    r!(0),
                ),
                (
                    r!(-1),
                    BezierAlgebraicCuspSemicircleContactLocation2::End,
                    r!(1),
                ),
            ] {
                let parameter = decided_ok(circle.replay_parameter_definition(
                    CircleParameterDefinition2::ParallelNormal {
                        parallel: source.parallel_left(distance.clone()).unwrap(),
                        parameter: r!(1 / 2).into(),
                        radial_distance: distance.clone(),
                        location,
                    },
                    &policy,
                ));
                assert_eq!(
                    decided_ok(parameter.order_to_real(&endpoint, &policy)),
                    std::cmp::Ordering::Equal
                );
                let point =
                    decided_ok(parameter.coincident_point_evidence(&circle, &policy)).unwrap();
                assert_eq!(
                    point.same_point(&Point2::new(r!(0), distance).into(), &policy),
                    Classification::Decided(true)
                );
            }
            assert!(matches!(
                circle.replay_parameter_definition(
                    CircleParameterDefinition2::ParallelNormal {
                        parallel: source.parallel_left(r!(1)).unwrap(),
                        parameter: r!(3 / 4).into(),
                        radial_distance: r!(1),
                        location: BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    },
                    &policy,
                ),
                Err(CurveError::InvalidCurveParameter)
            ));
        }
    }

    #[test]
    fn replay_selected_normal_contact_consumes_collapsed_center_locus() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // (1-t², 2t)/(1+t²) is the unit circle. Its left unit
            // parallel is wholly collapsed, while its source frame is regular.
            let source = RationalBezier2::from_homogeneous_controls(vec![
                HomogeneousControl2::new(r!(1), r!(0), r!(1)),
                HomogeneousControl2::new(r!(1), r!(1), r!(1)),
                HomogeneousControl2::new(r!(0), r!(2), r!(2)),
            ])
            .unwrap();
            let circle = decided_ok(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    source.parallel_left(r!(1)).unwrap(),
                    r!(1 / 2).into(),
                    r!(1),
                    true,
                    &policy,
                ),
            )
            .unwrap();
            let parameter = decided_ok(circle.replay_parameter_definition(
                CircleParameterDefinition2::ParallelNormal {
                    parallel: vertical_parallel(r!(-1)),
                    parameter: r!(1 / 2).into(),
                    radial_distance: r!(1),
                    location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                },
                &policy,
            ));
            assert_eq!(
                decided_ok(parameter.order_to_real(&r!(1 / 3), &policy)),
                std::cmp::Ordering::Equal
            );
            let fragment = decided_ok(BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle,
                BezierAlgebraicCuspSemicircleParameter2::Exact(r!(0)),
                parameter,
                false,
                &policy,
            ));
            let geometry = crate::SvgGeometry2::new(
                crate::CurveRegion2::empty(),
                Vec::new(),
                vec![crate::CurvePath2::try_new(vec![fragment.into()]).unwrap()],
            );
            let text = crate::export_svg_document(&geometry).unwrap();
            let replayed = crate::import_svg_document(&text).unwrap();
            let curve = &replayed.paths()[0].curves()[0];
            for (actual, expected) in [
                (curve.start(), pt!(r!(-3 / 5), r!(-4 / 5))),
                (curve.end(), pt!(-1, 0)),
                (
                    curve.point_at(&r!(1 / 4).into()).unwrap(),
                    pt!(r!(-24 / 25), r!(-7 / 25)),
                ),
            ] {
                assert_eq!(
                    actual.same_point(&expected.into(), &policy),
                    Classification::Decided(true)
                );
            }
            assert_eq!(crate::export_svg_document(&replayed).unwrap(), text);
        }
    }

    #[test]
    fn replay_selected_normal_circle_retains_filled_set_through_composition() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = parallel_frame(r!(1), false, &policy);
            let contact = decided_ok(circle.replay_parameter_definition(
                CircleParameterDefinition2::ParallelNormal {
                    parallel: vertical_parallel(r!(-1)),
                    parameter: r!(1 / 2).into(),
                    radial_distance: r!(1),
                    location: BezierAlgebraicCuspSemicircleContactLocation2::Interior,
                },
                &policy,
            ));
            let arc = decided_ok(BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle,
                BezierAlgebraicCuspSemicircleParameter2::Exact(r!(0)),
                contact,
                false,
                &policy,
            ));
            let origin: CurvePoint2 = pt!(0, 0).into();
            let boundary = crate::CurvePath2::try_new(vec![
                arc.into(),
                decided_ok(BezierAlgebraicChord2::try_new(
                    pt!(-1, 0).into(),
                    origin.clone(),
                    &policy,
                ))
                .into(),
                decided_ok(BezierAlgebraicChord2::try_new(
                    origin,
                    pt!(0, 1).into(),
                    &policy,
                ))
                .into(),
            ])
            .unwrap();
            let reflection = crate::Similarity2::try_from_real_affine(
                r!(-2),
                r!(0),
                r!(0),
                r!(2),
                r!(3),
                r!(-4),
            )
            .unwrap();
            for reflected in [false, true] {
                for reversed in [false, true] {
                    let boundary = if reflected {
                        boundary.transform_similarity(&reflection).unwrap()
                    } else {
                        boundary.clone()
                    };
                    let boundary = if reversed {
                        boundary.reversed().unwrap()
                    } else {
                        boundary
                    };
                    let outcome = crate::CurveRegion2::try_from_boundary_paths_with_policy(
                        &[boundary],
                        crate::FillRule::NonZero,
                        &policy,
                    )
                    .unwrap();
                    assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                    let region = outcome.into_value();
                    let geometry = crate::SvgGeometry2::new(region.clone(), Vec::new(), Vec::new());
                    let text = crate::export_svg_document(&geometry).unwrap();
                    let replayed = crate::import_svg_document(&text).unwrap();
                    // Independent quarter-disk inequalities, transported by
                    // the declared x'=3-2x, y'=2y-4 reflection when requested.
                    for (point, expected) in [
                        (
                            pt!(r!(-1 / 4), r!(1 / 4)),
                            crate::RegionPointLocation::Inside,
                        ),
                        (pt!(-1, 1), crate::RegionPointLocation::Outside),
                        (
                            pt!(r!(1 / 4), r!(1 / 4)),
                            crate::RegionPointLocation::Outside,
                        ),
                    ] {
                        let point = if reflected {
                            Point2::new(r!(3) - r!(2) * point.x(), r!(2) * point.y() - r!(4))
                        } else {
                            point
                        };
                        for candidate in [&region, replayed.region()] {
                            let location = candidate
                                .classify_point_with_policy(&point.clone().into(), &policy)
                                .unwrap();
                            assert_eq!(location.certainty, crate::CurveCertainty::Certified);
                            assert_eq!(location.into_value(), Classification::Decided(expected));
                        }
                    }
                    let rebuilt = crate::CurveRegion2::try_from_boundary_paths_with_policy(
                        &replayed.region().boundary_paths().unwrap(),
                        crate::FillRule::NonZero,
                        &policy,
                    )
                    .unwrap();
                    assert_eq!(rebuilt.certainty, crate::CurveCertainty::Certified);
                    for candidate in [replayed.region(), &rebuilt.value] {
                        let difference = region
                            .boolean_region_with_policy(candidate, crate::BooleanOp::Xor, &policy)
                            .unwrap();
                        assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                        assert!(difference.value.is_empty());
                        let union = difference
                            .value
                            .boolean_region_with_policy(candidate, crate::BooleanOp::Union, &policy)
                            .unwrap();
                        assert_eq!(union.certainty, crate::CurveCertainty::Certified);
                        assert!(!union.value.is_empty());
                    }
                    assert_eq!(crate::export_svg_document(&replayed).unwrap(), text);
                }
            }
        }
    }

    #[test]
    fn circle_replay_and_point_queries_preserve_consumed_policy() {
        let build = |policy: &CurveContext| {
            let Classification::Decided(anchor) = BezierAlgebraicChord2::try_new(
                Point2::from_values(0, 0).into(),
                Point2::from_values(1, 0).into(),
                policy,
            )
            .unwrap() else {
                panic!("a nonzero exact frame chord")
            };
            let Classification::Decided(Some(circle)) =
                BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
                    Point2::from_values(0, 0).into(),
                    anchor,
                    Real::one(),
                    true,
                    policy,
                )
                .unwrap()
            else {
                panic!("a positive-radius frame")
            };
            circle
        };
        let outcome =
            crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
                policy.observe_approximate_512();
                let circle = build(policy);
                let fragment = BezierAlgebraicCuspSemicircleFragment2::full(circle.clone(), policy);
                Ok::<_, CurveError>((circle, fragment))
            })
            .unwrap();
        assert_eq!(
            outcome.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        let (qualified, fragment) = outcome.into_value();
        assert!(qualified.native_chord_normal_definition().is_none());
        assert!(
            fragment
                .point_parameter(&Point2::from_values(1, 0).into(), &CurveContext::STRICT)
                .is_err()
        );
        let strict = build(&CurveContext::STRICT);
        let definition = strict.native_chord_normal_definition().unwrap();
        assert!(matches!(
            BezierAlgebraicCuspSemicircle2::replay_chord_normal_definition(
                definition,
                &CurveContext::STRICT
            )
            .unwrap(),
            Classification::Decided(_)
        ));
    }
}
