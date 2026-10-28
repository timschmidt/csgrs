//! Checked native definitions for a selected circle and its angular contacts.

use super::*;

pub(crate) struct CircleChordNormalDefinition2 {
    pub(crate) center: CurvePoint2,
    pub(crate) anchor: BezierAlgebraicChord2,
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
        let local = match self.parameter_at_certified_incident_point(&point, &policy)? {
            Classification::Decided(local) => local,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
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
            CircleParameterDefinition2::IncidentPoint(point) => {
                match self.retained_point_incidence_sign(&point, policy)? {
                    Classification::Decided(RealSign::Zero) => {}
                    Classification::Decided(_) => return Err(CurveError::InvalidCurveParameter),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let parameter = match self.parameter_at_certified_incident_point(&point, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
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
