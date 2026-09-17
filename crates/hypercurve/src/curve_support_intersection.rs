//! Support intersections restricted by retained parameter domains.
//!
//! A range is independent of the support's equations and parameter chart.
//! In particular, selected cuts do not require a new rational control net.

use super::*;
use crate::bezier_offset::{
    BezierAlgebraicChordPairIntersections2, BezierAlgebraicChordRationalIntersections2,
};
use crate::curve_support::CurveSupport2;
use crate::{BezierSplitFragment2, BezierSubcurve2, CurveFamily2};

#[derive(Default)]
struct Evidence {
    contacts: Vec<CurveIntersectionContact2>,
    overlaps: Vec<CurveIntersectionOverlap2>,
    blockers: Vec<CurveIntersectionPairBlocker2>,
    parameter_components: Vec<CurveIntersectionParameterComponent2>,
}

pub(super) struct Span {
    support: CurveSupport2,
    range: CurveParameterRange2,
    pub(super) chart: CurveSpanRange2,
    reversed: bool,
}

fn decided<T>(value: CurveResult<Classification<T>>, family: CurveFamily2) -> ExactCurveResult<T> {
    match value
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Intersection, family, cause))?
    {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            family,
            reason,
        )),
    }
}

pub(super) fn spans(curve: &Curve2, policy: &CurveContext) -> ExactCurveResult<Vec<Span>> {
    Ok(curve
        .source_spans(policy, CurveOperation2::Intersection)?
        .iter()
        .map(|span| Span {
            support: CurveSupport2::from_fragment(&span.fragment),
            range: if matches!(span.fragment, BezierSplitFragment2::Materialized { .. }) {
                CurveParameterRange2::unit()
            } else {
                span.fragment.curve_region_parameter_range()
            },
            chart: span.chart(),
            reversed: span.fragment.source_is_reversed(),
        })
        .collect())
}

fn contains(
    range: &CurveParameterRange2,
    parameter: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    decided(
        crate::bezier_split::CurveParameterDomain2::new(range, None)
            .contains_finite_parameter(parameter, policy),
        family,
    )
}

struct Pair<'a> {
    first: &'a Span,
    second: &'a Span,
    indices: [usize; 2],
    policy: &'a CurveContext,
}

fn reverse_sign(sign: hyperreal::RealSign) -> hyperreal::RealSign {
    match sign {
        hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
        hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
        hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
    }
}

impl Pair<'_> {
    fn overlap(
        &self,
        [mut first_range, mut second_range]: [CurveParameterRange2; 2],
        mut orientation: RationalBezierOverlapOrientation2,
        mut inclusion: [bool; 2],
        correspondence: CurveOverlapCorrespondence2,
    ) -> ExactCurveResult<CurveIntersectionOverlap2> {
        let descending = decided(
            first_range
                .start()
                .cmp_by_refinement(first_range.end(), self.policy),
            self.first.support.family(),
        )?
        .is_gt();
        if descending != self.first.reversed {
            first_range = CurveParameterRange2::new_validated(
                first_range.end().clone(),
                first_range.start().clone(),
            );
            second_range = CurveParameterRange2::new_validated(
                second_range.end().clone(),
                second_range.start().clone(),
            );
            inclusion.reverse();
        }
        if self.first.reversed != self.second.reversed {
            orientation = match orientation {
                RationalBezierOverlapOrientation2::Same => {
                    RationalBezierOverlapOrientation2::Reversed
                }
                RationalBezierOverlapOrientation2::Reversed => {
                    RationalBezierOverlapOrientation2::Same
                }
            };
        }
        Ok(CurveIntersectionOverlap2 {
            first_span_index: self.indices[0],
            second_span_index: self.indices[1],
            first_range,
            second_range,
            orientation,
            endpoint_inclusion: inclusion,
            parameter_correspondence: correspondence,
        })
    }

    fn chords(
        &self,
        first: &crate::BezierAlgebraicChord2,
        second: &crate::BezierAlgebraicChord2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        // Chord restrictions own new finite chord supports, so their local
        // domains already match the finite-domain authority below.
        match decided(
            first.chord_intersections(second, self.policy),
            CurveFamily2::Line,
        )? {
            BezierAlgebraicChordPairIntersections2::Contacts(contacts) => {
                for contact in contacts {
                    self.append_contact(
                        result,
                        self.contact(
                            CurveParameter2::from_algebraic_chord(
                                contact.first_parameter().clone(),
                            ),
                            CurveParameter2::from_algebraic_chord(
                                contact.second_parameter().clone(),
                            ),
                            contact.point().clone(),
                            contact.tangent_cross_sign() != hyperreal::RealSign::Zero,
                            Some(contact.tangent_cross_sign()),
                        ),
                    )?;
                }
            }
            BezierAlgebraicChordPairIntersections2::Overlaps(overlaps) => {
                for overlap in overlaps {
                    let convert = |[start, end]: [&crate::bezier_offset::BezierAlgebraicChordParameter2;
                                       2]| {
                        CurveParameterRange2::new_validated(
                            CurveParameter2::from_algebraic_chord(start.clone()),
                            CurveParameter2::from_algebraic_chord(end.clone()),
                        )
                    };
                    let first_range = convert(overlap.first_range());
                    let second_range = convert(overlap.second_range());
                    let correspondence = CurveOverlapCorrespondence2::Chords {
                        first: first.clone(),
                        second: second.clone(),
                        first_range: first_range.clone(),
                        second_range: second_range.clone(),
                    };
                    result.overlaps.push(self.overlap(
                        [first_range, second_range],
                        overlap.orientation(),
                        [true, true],
                        correspondence,
                    )?);
                }
            }
        }
        Ok(())
    }

    fn chord_rational(
        &self,
        chord: &crate::BezierAlgebraicChord2,
        source: &RationalBezier2,
        chord_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let span = if chord_first { self.second } else { self.first };
        let family = span.support.family();
        let (contacts, overlaps) = match decided(
            chord.rational_intersections(source, &span.range, None, self.policy),
            family,
        )? {
            BezierAlgebraicChordRationalIntersections2::Contacts(contacts) => {
                (contacts, Vec::new())
            }
            BezierAlgebraicChordRationalIntersections2::Overlaps(overlaps) => {
                (Vec::new(), overlaps)
            }
            BezierAlgebraicChordRationalIntersections2::ContactsAndOverlaps {
                contacts,
                overlaps,
            } => (contacts, overlaps),
            BezierAlgebraicChordRationalIntersections2::DegenerateProjection => {
                if let Classification::Decided(Some(point)) =
                    BezierSubcurve2::Rational(source.clone())
                        .point_image(&self.policy.strict_counterpart())
                {
                    let point = CurvePoint2::from(point);
                    if decided(chord.contains_point_evidence(&point, self.policy), family)? {
                        let parameter = chord
                            .parameter_at_certified_support_point(point.clone(), self.policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(
                                    CurveOperation2::Intersection,
                                    family,
                                    cause,
                                )
                            })?;
                        let parameter = Some(CurveParameter2::from_algebraic_chord(parameter));
                        self.point_image_component(
                            if chord_first {
                                [parameter, None]
                            } else {
                                [None, parameter]
                            },
                            &point,
                            result,
                        )?;
                    }
                    return Ok(());
                }
                self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent);
                return Ok(());
            }
            BezierAlgebraicChordRationalIntersections2::NotSourceRelated => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    family,
                    UncertaintyReason::Unsupported,
                ));
            }
        };
        let contact = |chord: CurveParameter2, source: CurveParameter2, point, cross| {
            let (first, second, cross) = if chord_first {
                (chord, source, cross)
            } else {
                (source, chord, reverse_sign(cross))
            };
            self.contact(
                first,
                second,
                point,
                cross != hyperreal::RealSign::Zero,
                Some(cross),
            )
        };
        for evidence in contacts {
            if contains(&span.range, evidence.other_parameter(), family, self.policy)? {
                self.append_contact(
                    result,
                    contact(
                        CurveParameter2::from_algebraic_chord(evidence.chord_parameter().clone()),
                        evidence.other_parameter().clone(),
                        evidence.point().clone(),
                        evidence.tangent_cross_sign(),
                    ),
                )?;
            }
        }
        let chord_span = if chord_first { self.first } else { self.second };
        for overlap in overlaps {
            if let Some((chord_range, source_range)) = decided(
                overlap.clipped_ranges(&chord_span.range, &span.range, self.policy),
                family,
            )? {
                let ranges = if chord_first {
                    [chord_range, source_range]
                } else {
                    [source_range, chord_range]
                };
                result.overlaps.push(self.overlap(
                    ranges,
                    overlap.orientation(),
                    [true, true],
                    CurveOverlapCorrespondence2::ChordRational {
                        source: Arc::new(overlap),
                        chord_first,
                    },
                )?);
            } else {
                // Positive-length clipping intentionally omits a singleton;
                // open curves retain it as an exact endpoint contact.
                for parameter in [span.range.start(), span.range.end()] {
                    if !contains(overlap.source_range(), parameter, family, self.policy)? {
                        continue;
                    }
                    let mapped = decided(
                        overlap.chord_parameter_at_source_parameter(parameter, self.policy),
                        family,
                    )?
                    .ok_or_else(|| {
                        ExactCurveError::blocked(
                            CurveOperation2::Intersection,
                            family,
                            UncertaintyReason::Boundary,
                        )
                    })?;
                    let point = mapped.point().clone();
                    self.append_contact(
                        result,
                        contact(
                            CurveParameter2::from_algebraic_chord(mapped),
                            parameter.clone(),
                            point,
                            hyperreal::RealSign::Zero,
                        ),
                    )?;
                }
            }
        }
        Ok(())
    }

    fn contact(
        &self,
        first: CurveParameter2,
        second: CurveParameter2,
        point: CurvePoint2,
        transverse: bool,
        cross: Option<hyperreal::RealSign>,
    ) -> CurveIntersectionContact2 {
        let cross = cross.map(|sign| {
            if self.first.reversed != self.second.reversed {
                reverse_sign(sign)
            } else {
                sign
            }
        });
        CurveIntersectionContact2 {
            first: CurveLocation2 {
                span_index: self.indices[0],
                span_range: self.first.chart.clone(),
                local_parameter: first,
            },
            second: CurveLocation2 {
                span_index: self.indices[1],
                span_range: self.second.chart.clone(),
                local_parameter: second,
            },
            point,
            certified_transverse: transverse,
            tangent_cross_sign: cross,
        }
    }

    fn append_contact(
        &self,
        result: &mut Evidence,
        contact: CurveIntersectionContact2,
    ) -> ExactCurveResult<()> {
        let index = match matching_contact_index(&result.contacts, &contact, self.policy) {
            Classification::Decided(index) => index,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    self.first.support.family(),
                    reason,
                ));
            }
        };
        if let Some(index) = index {
            result.contacts[index].certified_transverse |= contact.certified_transverse;
            if result.contacts[index].tangent_cross_sign.is_none() {
                result.contacts[index].tangent_cross_sign = contact.tangent_cross_sign;
            }
        } else {
            result.contacts.push(contact);
        }
        Ok(())
    }

    fn admit_contact(
        &self,
        result: &mut Evidence,
        contact: CurveIntersectionContact2,
    ) -> ExactCurveResult<()> {
        if contains(
            &self.first.range,
            contact.first().local_parameter(),
            self.first.support.family(),
            self.policy,
        )? && contains(
            &self.second.range,
            contact.second().local_parameter(),
            self.second.support.family(),
            self.policy,
        )? {
            self.append_contact(result, contact)?;
        }
        Ok(())
    }

    fn circle_contact(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        circle_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        mut other_parameter: CurveParameter2,
        point: Option<CurvePoint2>,
        cross: Option<hyperreal::RealSign>,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let mut circle_parameter = CurveParameter2::from_algebraic_cusp(circle_parameter);
        // Identify either endpoint before ordering against the complete
        // range. The retained endpoint can be much cheaper to order against
        // its opposite boundary than an independently isolated equal contact.
        // Keep this optional canonicalization bounded; domain admission below
        // still owns every required exact comparison.
        let mut point = point;
        let mut reused_endpoint = false;
        for at_start in [true, false] {
            let endpoint =
                CurveParameter2::from_algebraic_cusp(circle.endpoint_parameter(at_start).clone());
            if self
                .policy
                .bounded_exact_predicate_pass(|| {
                    circle_parameter.cmp_by_refinement(&endpoint, self.policy)
                })
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        crate::CurveFamily2::CircularArc,
                        cause,
                    )
                })?
                != Classification::Decided(std::cmp::Ordering::Equal)
            {
                continue;
            }
            if let Classification::Decided(Some(retained)) = circle
                .endpoint_point_evidence(at_start, self.policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        crate::CurveFamily2::CircularArc,
                        cause,
                    )
                })?
            {
                circle_parameter = endpoint;
                point = Some(retained);
                reused_endpoint = true;
            }
            break;
        }
        let other = if circle_first {
            self.second
        } else {
            self.first
        };
        let chord_parameter = other_parameter.as_algebraic_chord();
        if (reused_endpoint || chord_parameter.is_some())
            && let CurveSupport2::Bezier(source) = &other.support
        {
            let source = RationalBezier2::try_from_subcurve(source).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    other.support.family(),
                    cause,
                )
            })?;
            if let Some(line) = source.exact_linear_parameterization_line() {
                let witness = point
                    .as_ref()
                    .or_else(|| chord_parameter.map(|parameter| parameter.point()))
                    .expect("a reused endpoint or chord contact retains its point");
                let parameter = self
                    .policy
                    .strict_predicate_pass(|| {
                        crate::bezier_offset::affine_line_parameter_at_incident_point(
                            &line,
                            witness,
                            self.policy,
                        )
                    })
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Intersection,
                            other.support.family(),
                            cause,
                        )
                    })?;
                match parameter {
                    Classification::Decided(parameter) => other_parameter = parameter,
                    Classification::Uncertain(reason) if chord_parameter.is_some() => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Intersection,
                            other.support.family(),
                            reason,
                        ));
                    }
                    Classification::Uncertain(_) => {}
                }
            }
        }
        let (first, second) = if circle_first {
            (circle_parameter.clone(), other_parameter)
        } else {
            (other_parameter, circle_parameter.clone())
        };
        if !contains(
            &self.first.range,
            &first,
            self.first.support.family(),
            self.policy,
        )? || !contains(
            &self.second.range,
            &second,
            self.second.support.family(),
            self.policy,
        )? {
            return Ok(());
        }
        let point = match point {
            Some(point) => point,
            None => Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicCuspSemicircle(
                circle.clone(),
            ))
            .point_at(&circle_parameter, self.policy)?
            .into_value(),
        };
        let cross = cross.map(|sign| {
            if circle_first {
                sign
            } else {
                reverse_sign(sign)
            }
        });
        self.append_contact(
            result,
            self.contact(
                first,
                second,
                point,
                matches!(
                    cross,
                    Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Negative)
                ),
                cross,
            ),
        )
    }

    fn circle_overlap(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        source: CurveCircleOverlap2,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let orientation = source.orientation();
        let correspondence = CurveOverlapCorrespondence2::Circle {
            source: source.clone(),
            swapped: !circle_first,
        };
        if let Some((first, second)) = decided(
            correspondence.clipped_ranges(&self.first.range, &self.second.range, self.policy),
            self.first.support.family(),
        )? {
            result.overlaps.push(self.overlap(
                [first, second],
                orientation,
                [true, true],
                correspondence,
            )?);
            return Ok(());
        }
        let (circle_span, other_span) = if circle_first {
            (self.first, self.second)
        } else {
            (self.second, self.first)
        };
        if let Some([parameter, mapped]) = decided(
            source.singleton_contact(&circle_span.range, &other_span.range, self.policy),
            circle_span.support.family(),
        )? {
            self.circle_contact(
                circle,
                parameter
                    .as_algebraic_cusp()
                    .ok_or_else(|| {
                        ExactCurveError::invalid(
                            CurveOperation2::Intersection,
                            crate::CurveFamily2::CircularArc,
                            CurveError::InvalidCurveParameter,
                        )
                    })?
                    .clone(),
                mapped,
                None,
                Some(hyperreal::RealSign::Zero),
                circle_first,
                result,
            )?;
        }
        Ok(())
    }

    fn circles(
        &self,
        first: &crate::BezierAlgebraicCuspSemicircleFragment2,
        second: &crate::BezierAlgebraicCuspSemicircleFragment2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        use crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2 as Intersections;
        match decided(
            first
                .semicircle()
                .pair_intersections(second.semicircle(), self.policy),
            self.first.support.family(),
        )? {
            Intersections::NoContacts => {}
            Intersections::Contacts {
                contacts,
                parameter_map,
            } => {
                for contact in contacts {
                    self.circle_contact(
                        first,
                        parameter_map.first_contact_parameter(&contact),
                        CurveParameter2::from_algebraic_cusp(
                            parameter_map.second_contact_parameter(&contact),
                        ),
                        None,
                        Some(contact.tangent_cross_sign),
                        true,
                        result,
                    )?;
                }
            }
            Intersections::EndpointContacts(contacts) => {
                for contact in contacts {
                    self.circle_contact(
                        first,
                        contact
                            .first_location
                            .endpoint_parameter()
                            .expect("endpoint contact"),
                        CurveParameter2::from_algebraic_cusp(
                            contact
                                .second_location
                                .endpoint_parameter()
                                .expect("endpoint contact"),
                        ),
                        None,
                        Some(contact.tangent_cross_sign),
                        true,
                        result,
                    )?;
                }
            }
            Intersections::Overlap(source) => {
                self.circle_overlap(first, CurveCircleOverlap2::Pair(source), true, result)?
            }
        }
        Ok(())
    }

    fn circle_chord(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        chord: &crate::BezierAlgebraicChord2,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        use crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2 as Intersections;
        let intersections = match circle
            .certified_chord_endpoint_contact(chord, self.policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    crate::CurveFamily2::CircularArc,
                    cause,
                )
            })? {
            Classification::Decided(Some(contact)) => Intersections::Contacts(vec![contact]),
            Classification::Decided(None) | Classification::Uncertain(_) => decided(
                circle.semicircle().chord_intersections(chord, self.policy),
                crate::CurveFamily2::CircularArc,
            )?,
        };
        if let Intersections::Contacts(contacts) = intersections {
            for contact in contacts {
                self.circle_contact(
                    circle,
                    contact.cusp_parameter,
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter),
                    Some(contact.point),
                    Some(contact.tangent_cross_sign),
                    circle_first,
                    result,
                )?;
            }
        }
        Ok(())
    }

    fn circle_selected_contacts(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        contacts: Vec<crate::bezier_offset::BezierAlgebraicCuspSemicircleSelectedFiberContact2>,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        for contact in contacts {
            self.circle_contact(
                circle,
                contact.cusp_parameter(),
                CurveParameter2::from_selected_fiber(contact.other_parameter().clone()),
                Some(contact.point_evidence()),
                Some(contact.tangent_cross_sign()),
                circle_first,
                result,
            )?;
        }
        Ok(())
    }

    fn require_unit_domain(&self, span: &Span) -> ExactCurveResult<()> {
        let unit = CurveParameterRange2::unit();
        for endpoint in [span.range.start(), span.range.end()] {
            if !contains(&unit, endpoint, span.support.family(), self.policy)? {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    span.support.family(),
                    UncertaintyReason::Unsupported,
                ));
            }
        }
        Ok(())
    }

    fn circle_rational(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        rational: &RationalBezier2,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        use crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2 as Intersections;
        let span = if circle_first {
            self.second
        } else {
            self.first
        };
        if let Some(line) = rational.exact_linear_parameterization_line() {
            let chord = decided(
                crate::BezierAlgebraicChord2::from_affine_line_range(
                    &line,
                    &span.range,
                    self.policy,
                ),
                span.support.family(),
            )?;
            return self.circle_chord(circle, &chord, circle_first, result);
        }
        let (intersections, map) = decided(
            circle
                .semicircle()
                .rational_intersections_with_parameter_map(rational, &span.range, self.policy),
            crate::CurveFamily2::CircularArc,
        )?;
        match intersections {
            Intersections::Mapped { contacts, overlaps } => {
                for contact in contacts {
                    let parameter = contact.location.endpoint_parameter().unwrap_or_else(|| {
                        map.as_ref()
                            .expect("interior contact retains its map")
                            .contact_parameter(&contact)
                    });
                    self.circle_contact(
                        circle,
                        parameter,
                        contact.other_parameter,
                        Some(contact.point),
                        Some(contact.tangent_cross_sign),
                        circle_first,
                        result,
                    )?;
                }
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Mapped(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::SelectedFiber { contacts, overlaps } => {
                self.circle_selected_contacts(circle, contacts, circle_first, result)?;
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Selected(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::DegenerateProjection => self.blocker(
                result,
                CurveIntersectionPairBlockerKind2::Uncertain(UncertaintyReason::Unsupported),
            ),
        }
        Ok(())
    }

    fn circle_parallel(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        parallel: &crate::BezierParallel2,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        use crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2 as Intersections;
        let span = if circle_first {
            self.second
        } else {
            self.first
        };
        let strict = self.policy.strict_counterpart();
        // The rational component is optional. Keep its attempt separate so a
        // failed projection cannot erase the native selected-circle replay.
        if let Classification::Decided(Some(component)) = parallel
            .exact_rational_parallel_component_on_regular_range(&span.range, &strict)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    span.support.family(),
                    cause,
                )
            })?
        {
            // This image has the same parameters on the certified regular
            // range. Publish against that support without changing the chart,
            // so affine chord contacts use the common source-parameter map.
            let image = Span {
                support: CurveSupport2::Bezier(BezierSubcurve2::Rational(
                    component.curve().clone(),
                )),
                range: span.range.clone(),
                chart: span.chart.clone(),
                reversed: span.reversed,
            };
            let mut candidate = Evidence::default();
            match (Pair {
                first: if circle_first { self.first } else { &image },
                second: if circle_first { &image } else { self.second },
                indices: self.indices,
                policy: &strict,
            })
            .circle_rational(circle, component.curve(), circle_first, &mut candidate)
            {
                Ok(()) if candidate.blockers.is_empty() => {
                    for contact in candidate.contacts {
                        self.append_contact(result, contact)?;
                    }
                    result.overlaps.extend(candidate.overlaps);
                    return Ok(());
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
                _ => {}
            }
        }
        self.require_unit_domain(span)?;
        let intersections = match span.range.as_bezier_parameters() {
            Some((start, end)) => circle.semicircle().parallel_intersections_in_range(
                parallel,
                &BezierParameterRange2::new_validated(start.clone(), end.clone()),
                self.policy,
            ),
            None => circle
                .semicircle()
                .parallel_intersections(parallel, self.policy),
        };
        match decided(intersections, crate::CurveFamily2::CircularArc)? {
            Intersections::Mapped { contacts, overlaps } => {
                let map = if contacts
                    .iter()
                    .any(|c| c.retained_cusp_parameter().is_none())
                {
                    Some(decided(
                        circle
                            .semicircle()
                            .parallel_parameter_map(parallel, self.policy),
                        crate::CurveFamily2::CircularArc,
                    )?)
                } else {
                    None
                };
                for contact in contacts {
                    let parameter = contact.retained_cusp_parameter().unwrap_or_else(|| {
                        map.as_ref()
                            .expect("interior contact retains its map")
                            .contact_parameter(&contact)
                    });
                    self.circle_contact(
                        circle,
                        parameter,
                        CurveParameter2::from(contact.parallel_parameter),
                        None,
                        contact.tangent_cross_sign,
                        circle_first,
                        result,
                    )?;
                }
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Mapped(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::SelectedFiber { contacts, overlaps } => {
                self.circle_selected_contacts(circle, contacts, circle_first, result)?;
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Selected(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::RetainedContacts(contacts) => {
                for contact in contacts {
                    self.circle_contact(
                        circle,
                        contact.cusp_parameter(),
                        contact.other_parameter().clone(),
                        Some(contact.point_evidence()),
                        Some(contact.tangent_cross_sign()),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::CoincidentCircleComponent => {
                self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent)
            }
            Intersections::DegenerateProjection => self.blocker(
                result,
                CurveIntersectionPairBlockerKind2::Uncertain(UncertaintyReason::Unsupported),
            ),
        }
        Ok(())
    }

    fn rational(
        &self,
        first: &RationalBezier2,
        second: &RationalBezier2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let unit = CurveParameterRange2::unit();
        let unit_domain = crate::bezier_split::CurveParameterDomain2::new(&unit, None);
        let unit_covers_pair = [self.first, self.second].into_iter().all(|span| {
            matches!(
                unit_domain.contains_finite_range(&span.range, &self.policy.strict_counterpart()),
                Ok(Classification::Decided(true))
            )
        }) && [first, second].into_iter().all(|source| {
            matches!(
                source.denominator_sign(&crate::CurveParameterRange2::unit()),
                Classification::Decided(
                    hyperreal::RealSign::Positive | hyperreal::RealSign::Negative
                )
            )
        });
        if !unit_covers_pair {
            return self.finite_rational(first, second, result);
        }
        if self.rational_unit_circles(first, second, result)? {
            return Ok(());
        }
        // A collapsed rational map has an entire parameter fiber, rather
        // than one isolated root. Reuse the zero-distance kernel's complete
        // point-component replay and the common finite-domain publication.
        let strict = self.policy.strict_counterpart();
        if [first, second].into_iter().any(|curve| {
            matches!(
                BezierSubcurve2::Rational(curve.clone()).point_image(&strict),
                Classification::Decided(Some(_))
            )
        }) {
            let parallel = first.parallel_left(Real::zero()).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    self.first.support.family(),
                    cause,
                )
            })?;
            return self.parallel_rational(&parallel, second, true, result);
        }
        let context = RationalBezierIntersectionContext::try_new(first, second, self.policy)?;
        self.rational_context(&context, result)
    }

    fn rational_unit_circles(
        &self,
        first: &RationalBezier2,
        second: &RationalBezier2,
        result: &mut Evidence,
    ) -> ExactCurveResult<bool> {
        // A circle equation alone does not prove that an arbitrary rational
        // parameterization is injective. Exact quadratic charts (including
        // degree elevations) can reuse native angular incidence. The caller
        // has already proved that the unit charts cover the active ranges.
        if ![first, second].into_iter().all(|curve| {
            curve.retained_circular_conic().is_some()
                && matches!(
                    curve.quadratic_homogeneous_controls(self.policy),
                    Ok(Classification::Decided(Some(_)))
                )
        }) {
            return Ok(false);
        }
        let first_curve = Curve2::from(first.clone());
        let second_curve = Curve2::from(second.clone());
        let Some(native) = native_arc_intersection(&first_curve, &second_curve, self.policy, None)?
        else {
            return Ok(false);
        };
        let evidence = match native {
            NativeArcIntersectionDispatch::Points {
                first_arc,
                second_arc,
                points,
            } => build_native_arc_evidence(
                &first_curve,
                &second_curve,
                &first_arc,
                &second_arc,
                &points,
                self.policy,
                1,
            ),
            NativeArcIntersectionDispatch::Coincident {
                first_arc,
                second_arc,
            } => build_native_coincident_arc_evidence(
                &first_curve,
                &second_curve,
                &first_arc,
                &second_arc,
                self.policy,
                1,
            ),
        };
        let evidence = match evidence {
            Ok(evidence) => evidence,
            Err(ExactCurveError::Blocked(_)) => return Ok(false),
            Err(error) => return Err(error),
        };
        for contact in evidence.contacts() {
            self.admit_contact(
                result,
                self.contact(
                    contact.first().local_parameter().clone(),
                    contact.second().local_parameter().clone(),
                    contact.point().clone(),
                    contact.is_certified_transverse(),
                    contact.tangent_cross_sign(),
                ),
            )?;
        }
        for overlap in evidence.overlaps() {
            self.rational_overlap(first, overlap, result)?;
        }
        Ok(true)
    }

    fn rational_context(
        &self,
        context: &RationalBezierIntersectionContext,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let [first, second] = context.curves();
        let evidence = context.try_contacts()?;
        // The scalar context is a cheap complete authority for isolated
        // contacts and injective correspondences. Shared non-injective images
        // need every component and off-diagonal pair on the active domains.
        if matches!(
            evidence,
            RationalBezierIntersectionContacts2::Incomplete { .. }
                | RationalBezierIntersectionContacts2::DegenerateResultant
        ) || (evidence.overlap().is_some()
            && !(first.has_certified_injective_axis(self.policy)
                && second.has_certified_injective_axis(self.policy)))
        {
            return self.finite_rational(first, second, result);
        }

        for contact in evidence.isolated_contacts() {
            self.admit_contact(
                result,
                self.contact(
                    contact.first_parameter().clone().into(),
                    contact.second_parameter().clone().into(),
                    contact.point().clone(),
                    contact.is_certified_transverse(),
                    contact.tangent_cross_sign(),
                ),
            )?;
        }
        if let Some(overlap) = evidence.overlap() {
            self.rational_overlap(
                first,
                &CurveIntersectionOverlap2 {
                    first_span_index: self.indices[0],
                    second_span_index: self.indices[1],
                    first_range: CurveParameterRange2::from_bezier_range(
                        overlap.first_range().clone(),
                    ),
                    second_range: CurveParameterRange2::from_bezier_range(
                        overlap.second_range().clone(),
                    ),
                    orientation: overlap.orientation(),
                    endpoint_inclusion: [overlap.includes_start(), overlap.includes_end()],
                    parameter_correspondence: CurveOverlapCorrespondence2::Rational {
                        source: RationalCurveOverlap2::new(
                            context.overlap_parameter_correspondence(overlap),
                            overlap,
                        ),
                        swapped: false,
                    },
                },
                result,
            )?;
        }
        Ok(())
    }

    fn rational_overlap(
        &self,
        first: &RationalBezier2,
        overlap: &CurveIntersectionOverlap2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        if let Some(clipped) = decided(
            overlap.restrict_raw(&self.first.range, &self.second.range, self.policy),
            self.first.support.family(),
        )? {
            result.overlaps.push(self.overlap(
                [clipped.first_range, clipped.second_range],
                clipped.orientation,
                clipped.endpoint_inclusion,
                clipped.parameter_correspondence,
            )?);
        } else {
            // A regularized filled region can discard a singleton overlap.
            // An open curve must retain the shared endpoint contact.
            self.overlap_endpoint_contacts(first, overlap, result)?;
        }
        Ok(())
    }

    fn overlap_includes(
        &self,
        range: &CurveParameterRange2,
        inclusion: [bool; 2],
        parameter: &CurveParameter2,
    ) -> ExactCurveResult<bool> {
        for (boundary, included) in [(range.start(), inclusion[0]), (range.end(), inclusion[1])] {
            if !included
                && decided(
                    parameter.same_value(boundary, self.policy),
                    self.first.support.family(),
                )?
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn overlap_endpoint_contacts(
        &self,
        first: &RationalBezier2,
        overlap: &CurveIntersectionOverlap2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let CurveOverlapCorrespondence2::Rational {
            source: correspondence,
            swapped: false,
        } = &overlap.parameter_correspondence
        else {
            unreachable!("rational and native circle components retain their rational transport");
        };
        let first_overlap = overlap.first_range();
        let second_overlap = overlap.second_range();
        for forward in [true, false] {
            let (span, range, other) = if forward {
                (self.first, first_overlap, self.second)
            } else {
                (self.second, second_overlap, self.first)
            };
            for parameter in [span.range.start(), span.range.end()] {
                if !contains(range, parameter, span.support.family(), self.policy)? {
                    continue;
                }
                let mapped = if forward {
                    correspondence.source.map_first_to_second_region_parameter(
                        parameter,
                        &correspondence.first_range,
                        &correspondence.second_range,
                        self.policy,
                    )
                } else {
                    correspondence.source.map_second_to_first_region_parameter(
                        parameter,
                        &correspondence.first_range,
                        &correspondence.second_range,
                        self.policy,
                    )
                };
                let Some(mapped) = decided(mapped, span.support.family())? else {
                    continue;
                };
                if !contains(&other.range, &mapped, other.support.family(), self.policy)? {
                    continue;
                }
                let (first_parameter, second_parameter) = if forward {
                    (parameter.clone(), mapped)
                } else {
                    (mapped, parameter.clone())
                };
                if !self.overlap_includes(
                    overlap.first_range(),
                    overlap.endpoint_inclusion,
                    &first_parameter,
                )? {
                    continue;
                }
                let point = Curve2::from(first.clone())
                    .point_at(&first_parameter, self.policy)?
                    .into_value();
                self.append_contact(
                    result,
                    self.contact(
                        first_parameter,
                        second_parameter,
                        point,
                        false,
                        Some(hyperreal::RealSign::Zero),
                    ),
                )?;
            }
        }
        Ok(())
    }

    fn point_image_component(
        &self,
        parameters: [Option<CurveParameter2>; 2],
        point: &CurvePoint2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let domain = |span: &Span,
                      parameter: Option<&CurveParameter2>|
         -> ExactCurveResult<Option<CurveParameterSet2>> {
            Ok(Some(if let Some(parameter) = parameter {
                let parameter = parameter.clone();
                if !contains(&span.range, &parameter, span.support.family(), self.policy)? {
                    return Ok(None);
                }
                CurveParameterSet2::Single(parameter)
            } else {
                let source = match &span.support {
                    CurveSupport2::Bezier(BezierSubcurve2::Rational(source)) => {
                        Some(source.clone())
                    }
                    CurveSupport2::Bezier(BezierSubcurve2::RationalQuadratic(source)) => {
                        Some(source.clone().into())
                    }
                    CurveSupport2::Parallel(parallel) => match parallel.source() {
                        crate::BezierParallelSource2::Rational(source) => Some(source.clone()),
                        _ => None,
                    },
                    _ => None,
                };
                if let Some(source) = source {
                    let weight = &source
                        .homogeneous_power_basis()
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Intersection,
                                span.support.family(),
                                cause,
                            )
                        })?
                        .weight;
                    if !decided(
                        crate::bezier_offset::polynomial_is_nonzero_on_parameter_range(
                            weight,
                            &span.range,
                            &self.policy.strict_counterpart(),
                        ),
                        span.support.family(),
                    )? {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Intersection,
                            span.support.family(),
                            UncertaintyReason::Boundary,
                        ));
                    }
                }
                let range = if span.reversed {
                    CurveParameterRange2::new_validated(
                        span.range.end().clone(),
                        span.range.start().clone(),
                    )
                } else {
                    span.range.clone()
                };
                CurveParameterSet2::Range(range)
            }))
        };
        let Some(first_parameters) = domain(self.first, parameters[0].as_ref())? else {
            return Ok(());
        };
        let Some(second_parameters) = domain(self.second, parameters[1].as_ref())? else {
            return Ok(());
        };
        result
            .parameter_components
            .push(CurveIntersectionParameterComponent2 {
                first_span_index: self.indices[0],
                second_span_index: self.indices[1],
                first_parameters,
                second_parameters,
                point: point.clone(),
            });
        Ok(())
    }

    fn analytic_contact_point(&self, parameter: &CurveParameter2) -> ExactCurveResult<CurvePoint2> {
        match &self.first.support {
            CurveSupport2::Parallel(parallel) => decided(
                parallel.point_evidence_on_regular_range(parameter, &self.first.range, self.policy),
                self.first.support.family(),
            ),
            CurveSupport2::Bezier(source) => {
                Curve2::from(RationalBezier2::try_from_subcurve(source).map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        self.first.support.family(),
                        cause,
                    )
                })?)
                .point_at(parameter, self.policy)
                .map(CurveOutcome::into_value)
            }
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                self.first.support.family(),
                UncertaintyReason::Unsupported,
            )),
        }
    }

    fn analytic_overlap(
        &self,
        overlap: &crate::RationalBezierIntersectionOverlap2,
        correspondence: CurveOverlapCorrespondence2,
        swapped: bool,
        mut map: impl FnMut(
            &CurveParameter2,
            bool,
        ) -> CurveResult<Classification<Option<CurveParameter2>>>,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let family = self.first.support.family();
        if let Some((first_range, second_range)) = decided(
            correspondence.clipped_ranges(&self.first.range, &self.second.range, self.policy),
            family,
        )? {
            let lower_first = if swapped { &second_range } else { &first_range };
            let inclusion = [
                self.overlap_includes(
                    &CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
                    [overlap.includes_start(), overlap.includes_end()],
                    lower_first.start(),
                )?,
                self.overlap_includes(
                    &CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
                    [overlap.includes_start(), overlap.includes_end()],
                    lower_first.end(),
                )?,
            ];
            result.overlaps.push(self.overlap(
                [first_range, second_range],
                overlap.orientation(),
                inclusion,
                correspondence,
            )?);
            return Ok(());
        }
        // Filled regions discard a singleton. Open curves retain its exact
        // paired parameters, including when the active intervals only touch.
        let lower_ranges = [
            CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
            CurveParameterRange2::from_bezier_range(overlap.second_range().clone()),
        ];
        for forward in [true, false] {
            let (span, other) = if forward {
                (self.first, self.second)
            } else {
                (self.second, self.first)
            };
            let lower_forward = forward != swapped;
            let original = &lower_ranges[usize::from(!lower_forward)];
            for parameter in [span.range.start(), span.range.end()] {
                if !contains(original, parameter, span.support.family(), self.policy)? {
                    continue;
                }
                let Some(mapped) = decided(map(parameter, lower_forward), family)? else {
                    continue;
                };
                if !contains(&other.range, &mapped, other.support.family(), self.policy)? {
                    continue;
                }
                let (first, second) = if forward {
                    (parameter.clone(), mapped)
                } else {
                    (mapped, parameter.clone())
                };
                if !self.overlap_includes(
                    &CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
                    [overlap.includes_start(), overlap.includes_end()],
                    if swapped { &second } else { &first },
                )? {
                    continue;
                }
                let point = self.analytic_contact_point(&first)?;
                self.append_contact(
                    result,
                    self.contact(first, second, point, false, Some(hyperreal::RealSign::Zero)),
                )?;
            }
        }
        Ok(())
    }

    fn analytic_component_overlaps(
        &self,
        overlap: &crate::RationalBezierIntersectionOverlap2,
        components: &[crate::bezier_offset::BezierParameterComponentOverlap2],
        swapped: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<bool> {
        let mut retained = false;
        for source in components
            .iter()
            .filter(|source| source.overlap() == overlap)
        {
            retained = true;
            self.analytic_overlap(
                overlap,
                CurveOverlapCorrespondence2::ParameterComponent {
                    source: source.clone(),
                    swapped,
                },
                swapped,
                |parameter, forward| {
                    source.map_curve_parameter(
                        if forward {
                            hypersolve::CurveResultantParameter::First
                        } else {
                            hypersolve::CurveResultantParameter::Second
                        },
                        parameter,
                        self.policy,
                    )
                },
                result,
            )?;
        }
        Ok(retained)
    }

    fn analytic_rational_overlap(
        &self,
        overlap: &crate::RationalBezierIntersectionOverlap2,
        first: &RationalBezier2,
        second: &RationalBezier2,
        swapped: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let source = RationalCurveOverlap2::new(
            RationalBezierOverlapParameterCorrespondence2::for_overlap(
                first,
                second,
                overlap,
                self.policy,
            ),
            overlap,
        );
        self.analytic_overlap(
            overlap,
            CurveOverlapCorrespondence2::Rational {
                source: source.clone(),
                swapped,
            },
            swapped,
            |parameter, forward| {
                if forward {
                    source.source.map_first_to_second_region_parameter(
                        parameter,
                        &source.first_range,
                        &source.second_range,
                        self.policy,
                    )
                } else {
                    source.source.map_second_to_first_region_parameter(
                        parameter,
                        &source.first_range,
                        &source.second_range,
                        self.policy,
                    )
                }
            },
            result,
        )
    }

    fn parallel_rational(
        &self,
        parallel: &crate::BezierParallel2,
        rational: &RationalBezier2,
        parallel_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        for span in [self.first, self.second] {
            self.require_unit_domain(span)?;
        }
        let span = if parallel_first {
            self.first
        } else {
            self.second
        };
        let family = span.support.family();
        let evidence = decided(
            parallel.intersections_on_regular_range(rational, &span.range, self.policy),
            family,
        )?;
        for contact in evidence.contacts() {
            let parameters = [
                contact.parallel_parameter().clone().into(),
                contact.other_parameter().clone().into(),
            ];
            let [first, second] = if parallel_first {
                parameters
            } else {
                let [a, b] = parameters;
                [b, a]
            };
            if contains(
                &self.first.range,
                &first,
                self.first.support.family(),
                self.policy,
            )? && contains(
                &self.second.range,
                &second,
                self.second.support.family(),
                self.policy,
            )? {
                let cross = contact.tangent_cross_sign().map(|sign| {
                    if parallel_first {
                        sign
                    } else {
                        reverse_sign(sign)
                    }
                });
                self.append_contact(
                    result,
                    self.contact(
                        first,
                        second,
                        contact.point().clone(),
                        contact.is_certified_transverse(),
                        cross,
                    ),
                )?;
            }
        }
        for component in evidence.parameter_components() {
            let parameters = [component.parallel_parameter(), component.other_parameter()]
                .map(|parameter| parameter.cloned().map(Into::into));
            self.point_image_component(
                if parallel_first {
                    parameters
                } else {
                    {
                        let [first, second] = parameters;
                        [second, first]
                    }
                },
                component.point(),
                result,
            )?;
        }
        let mut rational_image = None;
        for overlap in evidence.overlaps() {
            if self.analytic_component_overlaps(
                overlap,
                evidence.component_overlaps(),
                !parallel_first,
                result,
            )? {
                continue;
            }
            let image = match &rational_image {
                Some(image) => image,
                None => {
                    let Some(image) = decided(
                        parallel.exact_rational_parallel_component_on_regular_range(
                            &span.range,
                            &self.policy.strict_counterpart(),
                        ),
                        family,
                    )?
                    else {
                        self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent);
                        continue;
                    };
                    rational_image.insert(image)
                }
            };
            self.analytic_rational_overlap(
                overlap,
                image.curve(),
                rational,
                !parallel_first,
                result,
            )?;
        }
        if let Some(candidates) = evidence.incomplete_candidates() {
            self.blocker(
                result,
                CurveIntersectionPairBlockerKind2::IncompleteReplay {
                    candidates: if parallel_first {
                        candidates.clone()
                    } else {
                        candidates.clone().swapped()
                    },
                },
            );
        }
        Ok(())
    }

    fn pair_contacts(
        &self,
        evidence: &crate::BezierParallelPairIntersectionSet2,
        mut point: impl FnMut(&CurveParameter2) -> ExactCurveResult<CurvePoint2>,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let family = self.first.support.family();
        for contact in evidence.contacts() {
            let first = contact.first_parameter();
            let second = contact.second_parameter();
            if contains(&self.first.range, first, family, self.policy)?
                && contains(
                    &self.second.range,
                    second,
                    self.second.support.family(),
                    self.policy,
                )?
            {
                let point = point(first)?;
                self.append_contact(
                    result,
                    self.contact(
                        first.clone(),
                        second.clone(),
                        point,
                        contact.is_certified_transverse(),
                        contact.tangent_cross_sign(),
                    ),
                )?;
            }
        }
        for component in evidence.parameter_components() {
            self.point_image_component(
                [component.first_parameter(), component.second_parameter()]
                    .map(|parameter| parameter.cloned().map(Into::into)),
                component.point(),
                result,
            )?;
        }
        if let Some(candidates) = evidence.incomplete_candidates() {
            self.blocker(
                result,
                CurveIntersectionPairBlockerKind2::IncompleteReplay {
                    candidates: candidates.clone(),
                },
            );
        }
        Ok(())
    }

    fn finite_rational(
        &self,
        first: &RationalBezier2,
        second: &RationalBezier2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let family = self.first.support.family();
        let evidence = decided(
            crate::bezier_offset::rational_pair_intersections_on_ranges(
                first,
                second,
                [&self.first.range, &self.second.range],
                self.policy,
            ),
            family,
        )?;
        let mut point_support = None;
        self.pair_contacts(&evidence, |parameter| {
            let support = match &point_support {
                Some(support) => support,
                None => point_support.insert(first.parallel_left(Real::zero()).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Intersection, family, cause)
                })?),
            };
            crate::bezier_offset::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                support.clone(), parameter, Real::zero(), self.policy,
            ).map(CurvePoint2::from).ok_or_else(|| ExactCurveError::blocked(
                CurveOperation2::Intersection, family, UncertaintyReason::Unsupported,
            ))
        }, result)?;
        for overlap in evidence.overlaps() {
            if !self.analytic_component_overlaps(
                overlap,
                evidence.component_overlaps(),
                false,
                result,
            )? {
                self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent);
            }
        }
        Ok(())
    }

    fn parallels(
        &self,
        first: &crate::BezierParallel2,
        second: &crate::BezierParallel2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        for span in [self.first, self.second] {
            self.require_unit_domain(span)?;
        }
        let family = self.first.support.family();
        let evidence = decided(
            first.parallel_intersections_on_regular_ranges(
                second,
                &self.first.range,
                &self.second.range,
                self.policy,
            ),
            family,
        )?;
        self.pair_contacts(
            &evidence,
            |parameter| self.analytic_contact_point(parameter),
            result,
        )?;
        let mut rational_images = None;
        for overlap in evidence.overlaps() {
            if self.analytic_component_overlaps(
                overlap,
                evidence.component_overlaps(),
                false,
                result,
            )? {
                continue;
            }
            let (a, b) = match &rational_images {
                Some(images) => images,
                None => {
                    let strict = self.policy.strict_counterpart();
                    let a = decided(
                        first.exact_rational_parallel_component_on_regular_range(
                            &self.first.range,
                            &strict,
                        ),
                        family,
                    )?;
                    let b = decided(
                        second.exact_rational_parallel_component_on_regular_range(
                            &self.second.range,
                            &strict,
                        ),
                        family,
                    )?;
                    match (a, b) {
                        (Some(a), Some(b)) => {
                            rational_images.insert((a.curve().clone(), b.curve().clone()))
                        }
                        _ => {
                            // Non-rational selected components carry their own maps.
                            // The remaining raw overlaps are certified source-image
                            // correspondences with the unchanged source parameters.
                            rational_images.insert((
                                first.source().to_rational_bezier().map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Intersection,
                                        family,
                                        cause,
                                    )
                                })?,
                                second.source().to_rational_bezier().map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Intersection,
                                        family,
                                        cause,
                                    )
                                })?,
                            ))
                        }
                    }
                }
            };
            self.analytic_rational_overlap(overlap, a, b, false, result)?;
        }
        Ok(())
    }

    fn chord_parallel(
        &self,
        chord: &crate::BezierAlgebraicChord2,
        parallel: &crate::BezierParallel2,
        chord_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        use crate::bezier_offset::BezierAlgebraicChordParallelIntersections2 as Intersections;
        let span = if chord_first { self.second } else { self.first };
        let strict = self.policy.strict_counterpart();
        // A branch-local rational image uses exactly the original parameter
        // chart and gives finite overlaps the existing chord correspondence.
        if let Classification::Decided(Some(image)) = parallel
            .exact_rational_parallel_component_on_regular_range(&span.range, &strict)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    span.support.family(),
                    cause,
                )
            })?
        {
            return self.chord_rational(chord, image.curve(), chord_first, result);
        }
        match decided(
            chord.parallel_intersections_on_regular_range(parallel, &span.range, self.policy),
            span.support.family(),
        )? {
            Intersections::Contacts(contacts) => {
                for contact in contacts {
                    let source = CurveParameter2::from(contact.parallel_parameter().clone());
                    if !contains(&span.range, &source, span.support.family(), self.policy)? {
                        continue;
                    }
                    let chord =
                        CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                    let cross = contact.tangent_cross_sign();
                    let (first, second, cross) = if chord_first {
                        (chord, source, cross)
                    } else {
                        (source, chord, reverse_sign(cross))
                    };
                    self.append_contact(
                        result,
                        self.contact(
                            first,
                            second,
                            contact.point().clone(),
                            cross != hyperreal::RealSign::Zero,
                            Some(cross),
                        ),
                    )?;
                }
            }
            Intersections::CoincidentSupportComponent => {
                self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent)
            }
            Intersections::DegenerateProjection => self.blocker(
                result,
                CurveIntersectionPairBlockerKind2::Uncertain(UncertaintyReason::Unsupported),
            ),
        }
        Ok(())
    }

    fn blocker(&self, result: &mut Evidence, kind: CurveIntersectionPairBlockerKind2) {
        result.blockers.push(CurveIntersectionPairBlocker2 {
            first_span_index: self.indices[0],
            second_span_index: self.indices[1],
            kind,
        });
    }
}

pub(super) fn intersect(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
    prepared: Option<&[PreparedRationalPair]>,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let first_spans = spans(first, policy)?;
    let second_spans = spans(second, policy)?;
    debug_assert!(
        prepared.is_none_or(|pairs| pairs.len() == first_spans.len() * second_spans.len())
    );
    let mut result = Evidence::default();
    for (first_index, first) in first_spans.iter().enumerate() {
        for (second_index, second) in second_spans.iter().enumerate() {
            let pair = Pair {
                first,
                second,
                indices: [first_index, second_index],
                policy,
            };
            let rational = |curve: &BezierSubcurve2| {
                RationalBezier2::try_from_subcurve(curve).map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        CurveFamily2::RationalBezier,
                        cause,
                    )
                })
            };
            // Native scheduling may retain a shared circle certificate or an
            // injective lineage map. Publication still uses the same authority
            // as restricted and generated supports, including every parameter
            // component and residual contact of non-injective rational images.
            let outcome = match prepared
                .map(|pairs| &pairs[first_index * second_spans.len() + second_index])
            {
                Some(PreparedRationalPair::Rational(context)) => {
                    pair.rational_context(context, &mut result)
                }
                Some(PreparedRationalPair::RetainedLineageOverlap {
                    first_range,
                    second_range,
                    orientation,
                }) => {
                    result.overlaps.push(pair.overlap(
                        [
                            CurveParameterRange2::new_validated(
                                first_range.start().clone().into(),
                                first_range.end().clone().into(),
                            ),
                            CurveParameterRange2::new_validated(
                                second_range.start().clone().into(),
                                second_range.end().clone().into(),
                            ),
                        ],
                        *orientation,
                        [true, true],
                        CurveOverlapCorrespondence2::affine(first_range, second_range),
                    )?);
                    Ok(())
                }
                Some(PreparedRationalPair::Blocked(reason)) => {
                    pair.blocker(
                        &mut result,
                        CurveIntersectionPairBlockerKind2::Uncertain(*reason),
                    );
                    Ok(())
                }
                None => match (&first.support, &second.support) {
                    (CurveSupport2::Bezier(first), CurveSupport2::Bezier(second)) => {
                        pair.rational(&rational(first)?, &rational(second)?, &mut result)
                    }
                    (CurveSupport2::Line(first), CurveSupport2::Line(second)) => {
                        pair.chords(first, second, &mut result)
                    }
                    (CurveSupport2::Line(chord), CurveSupport2::Bezier(source)) => {
                        pair.chord_rational(chord, &rational(source)?, true, &mut result)
                    }
                    (CurveSupport2::Bezier(source), CurveSupport2::Line(chord)) => {
                        pair.chord_rational(chord, &rational(source)?, false, &mut result)
                    }
                    (CurveSupport2::Circle(first), CurveSupport2::Circle(second)) => {
                        pair.circles(first, second, &mut result)
                    }
                    (CurveSupport2::Circle(circle), CurveSupport2::Line(chord)) => {
                        pair.circle_chord(circle, chord, true, &mut result)
                    }
                    (CurveSupport2::Line(chord), CurveSupport2::Circle(circle)) => {
                        pair.circle_chord(circle, chord, false, &mut result)
                    }
                    (CurveSupport2::Circle(circle), CurveSupport2::Bezier(source)) => {
                        pair.circle_rational(circle, &rational(source)?, true, &mut result)
                    }
                    (CurveSupport2::Bezier(source), CurveSupport2::Circle(circle)) => {
                        pair.circle_rational(circle, &rational(source)?, false, &mut result)
                    }
                    (CurveSupport2::Circle(circle), CurveSupport2::Parallel(parallel)) => {
                        pair.circle_parallel(circle, parallel, true, &mut result)
                    }
                    (CurveSupport2::Parallel(parallel), CurveSupport2::Circle(circle)) => {
                        pair.circle_parallel(circle, parallel, false, &mut result)
                    }
                    (CurveSupport2::Parallel(parallel), CurveSupport2::Bezier(source)) => {
                        pair.parallel_rational(parallel, &rational(source)?, true, &mut result)
                    }
                    (CurveSupport2::Bezier(source), CurveSupport2::Parallel(parallel)) => {
                        pair.parallel_rational(parallel, &rational(source)?, false, &mut result)
                    }
                    (CurveSupport2::Parallel(first), CurveSupport2::Parallel(second)) => {
                        pair.parallels(first, second, &mut result)
                    }
                    (CurveSupport2::Line(chord), CurveSupport2::Parallel(parallel)) => {
                        pair.chord_parallel(chord, parallel, true, &mut result)
                    }
                    (CurveSupport2::Parallel(parallel), CurveSupport2::Line(chord)) => {
                        pair.chord_parallel(chord, parallel, false, &mut result)
                    }
                },
            };
            match outcome {
                Ok(()) => {}
                Err(ExactCurveError::Blocked(blocker)) => pair.blocker(
                    &mut result,
                    CurveIntersectionPairBlockerKind2::Uncertain(blocker.reason()),
                ),
                Err(error) => return Err(error),
            }
        }
    }
    Ok(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: first_spans.len() * second_spans.len(),
            contacts: result.contacts.into(),
            overlaps: result.overlaps.into(),
            blockers: result.blockers.into(),
            parameter_components: (!result.parameter_components.is_empty())
                .then(|| result.parameter_components.into()),
        }),
    })
}

#[cfg(test)]
mod circle_dispatch_tests {
    use super::*;
    use crate::bezier_offset::{
        BezierAlgebraicCuspSemicircle2, BezierAlgebraicCuspSemicircleFragment2,
    };
    use crate::{CurveCertainty, LineSeg2, QuadraticBezier2};

    fn p(x: i32, y: i32) -> Point2 {
        Point2::from_values(x, y)
    }
    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    pub(super) fn exact<T: std::fmt::Debug>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }
    fn circle(y: i32, root_degree: i32, policy: &CurveContext) -> Curve2 {
        let polynomial = exact(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-1), Real::zero(), Real::from(root_degree)],
                policy,
            )
            .unwrap(),
        );
        let interval =
            exact(crate::BezierParameterInterval::try_new(q(1, 2), q(3, 4), policy).unwrap());
        let parameter = exact(
            crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        );
        let support = QuadraticBezier2::new(p(-1, y), p(-1, y), p(root_degree - 1, y))
            .parallel_left(Real::zero())
            .unwrap();
        let circle = exact(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                support,
                BezierParameter2::algebraic(parameter),
                Real::one(),
                false,
                policy,
            )
            .unwrap(),
        )
        .unwrap();
        Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicCuspSemicircle(
            BezierAlgebraicCuspSemicircleFragment2::full(circle, policy),
        ))
    }
    pub(super) fn oriented(curve: &Curve2, reversed: bool, policy: &CurveContext) -> Curve2 {
        if reversed {
            let out = curve.reversed(policy).unwrap();
            assert_eq!(out.certainty, CurveCertainty::Certified);
            out.value
        } else {
            curve.clone()
        }
    }
    pub(super) fn same(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
        let equal = first.coincides_with(second, policy);
        assert_eq!(equal.certainty, CurveCertainty::Certified);
        assert_eq!(equal.value, Classification::Decided(true));
    }
    fn replay(
        first: &Curve2,
        second: &Curve2,
        result: &CurveIntersectionResult2,
        policy: &CurveContext,
    ) {
        for contact in result.contacts() {
            for (curve, location) in [(first, contact.first()), (second, contact.second())] {
                let parameter = exact(location.parameter(policy).unwrap());
                let point = curve.point_at(&parameter, policy).unwrap();
                assert_eq!(point.certainty, CurveCertainty::Certified);
                same(&point.value, contact.point(), policy);
            }
        }
        let first_spans = spans(first, policy).unwrap();
        let second_spans = spans(second, policy).unwrap();
        for overlap in result.overlaps() {
            for (first_parameter, second_parameter) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                let first_parameter = exact(
                    CurveLocation2::new(
                        overlap.first_span_index(),
                        first_spans[overlap.first_span_index()].chart.clone(),
                        first_parameter.clone(),
                    )
                    .parameter(policy)
                    .unwrap(),
                );
                let second_parameter = exact(
                    CurveLocation2::new(
                        overlap.second_span_index(),
                        second_spans[overlap.second_span_index()].chart.clone(),
                        second_parameter.clone(),
                    )
                    .parameter(policy)
                    .unwrap(),
                );
                let a = first.point_at(&first_parameter, policy).unwrap();
                let b = second.point_at(&second_parameter, policy).unwrap();
                assert_eq!(a.certainty, CurveCertainty::Certified);
                assert_eq!(b.certainty, CurveCertainty::Certified);
                same(&a.value, &b.value, policy);
            }
        }
    }
    #[track_caller]
    pub(super) fn query(
        first: &Curve2,
        second: &Curve2,
        policy: &CurveContext,
    ) -> CurveIntersectionResult2 {
        let result = first.intersect_curve(second, policy).unwrap();
        assert_eq!(result.certainty, CurveCertainty::Certified);
        assert!(result.value.is_complete(), "{:?}", result.value.blockers());
        replay(first, second, &result.value, policy);
        result.value
    }

    #[test]
    fn selected_circle_normal_requires_a_regular_selected_source_point() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = QuadraticBezier2::new(p(-1, 0), p(-1, 0), p(1, 0))
                .parallel_left(Real::zero())
                .unwrap();
            let singular = BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                source.clone(),
                BezierParameter2::Exact(Real::zero()),
                Real::one(),
                false,
                &policy,
            )
            .unwrap();
            assert!(matches!(
                singular,
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
            let regular = BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                source,
                BezierParameter2::Exact(q(1, 2)),
                Real::one(),
                false,
                &policy,
            )
            .unwrap();
            assert!(matches!(regular, Classification::Decided(Some(_))));
        }
    }

    #[test]
    fn selected_circle_pairs_replay_crossings_tangencies_and_disjointness() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let original = circle(0, 2, &policy);
            for (y, expected) in [
                (1, Some(Point2::new(-q(3, 4).sqrt().unwrap(), q(1, 2)))),
                (2, Some(p(0, 1))),
                (3, None),
            ] {
                let other = circle(y, 3, &policy);
                for a in [false, true] {
                    for b in [false, true] {
                        for swapped in [false, true] {
                            let first = oriented(&original, a, &policy);
                            let second = oriented(&other, b, &policy);
                            let (first, second) = if swapped {
                                (&second, &first)
                            } else {
                                (&first, &second)
                            };
                            let result = query(first, second, &policy);
                            assert!(result.overlaps().is_empty());
                            assert_eq!(result.contacts().len(), usize::from(expected.is_some()));
                            if let Some(point) = &expected {
                                let contact = &result.contacts()[0];
                                same(contact.point(), &point.clone().into(), &policy);
                                let sign = if y == 2 {
                                    hyperreal::RealSign::Zero
                                } else if a ^ b ^ swapped {
                                    hyperreal::RealSign::Negative
                                } else {
                                    hyperreal::RealSign::Positive
                                };
                                assert_eq!(contact.tangent_cross_sign(), Some(sign));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn complementary_circle_halves_keep_both_endpoint_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = circle(0, 2, &policy);
            let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) =
                first.retained_fragment().unwrap()
            else {
                unreachable!()
            };
            let second =
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(
                        fragment.semicircle().complementary_half(),
                        &policy,
                    ),
                ));
            for a in [false, true] {
                for b in [false, true] {
                    for swapped in [false, true] {
                        let first = oriented(&first, a, &policy);
                        let second = oriented(&second, b, &policy);
                        let (first, second) = if swapped {
                            (&second, &first)
                        } else {
                            (&first, &second)
                        };
                        let result = query(first, second, &policy);
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), 2);
                        for point in [p(0, 1), p(0, -1)] {
                            assert!(result.contacts().iter().any(|contact| {
                                contact
                                    .point()
                                    .coincides_with(&point.clone().into(), &policy)
                                    .value
                                    == Classification::Decided(true)
                            }));
                        }
                        assert!(
                            result
                                .contacts()
                                .iter()
                                .all(|contact| contact.tangent_cross_sign()
                                    == Some(hyperreal::RealSign::Zero))
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_affine_domains_keep_exterior_and_algebraic_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = circle(0, 2, &policy);
            let selected = |shift: i32| {
                let polynomial = exact(
                    crate::BezierParameterPolynomial::try_new_power_basis(
                        vec![
                            Real::from(2 * shift * shift - 1),
                            Real::from(-4 * shift),
                            Real::from(2),
                        ],
                        &policy,
                    )
                    .unwrap(),
                );
                let interval = exact(
                    crate::BezierParameterInterval::try_new_ordered(
                        Real::from(shift) + q(1, 2),
                        Real::from(shift) + q(3, 4),
                        &policy,
                    )
                    .unwrap(),
                );
                BezierParameter2::algebraic(exact(
                    crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
                        .unwrap(),
                ))
            };
            for y in [Real::zero(), q(1, 2), Real::from(2)] {
                for (origin, delta, start, end, elevated) in [
                    (
                        -3,
                        2,
                        BezierParameter2::Exact(Real::one()),
                        BezierParameter2::Exact(Real::from(2)),
                        false,
                    ),
                    (
                        3,
                        -2,
                        BezierParameter2::Exact(Real::one()),
                        BezierParameter2::Exact(Real::from(2)),
                        false,
                    ),
                    (
                        3,
                        2,
                        BezierParameter2::Exact(Real::from(-2)),
                        BezierParameter2::Exact(Real::from(-1)),
                        false,
                    ),
                    (-3, 2, selected(0), selected(2), false),
                    (
                        -3,
                        2,
                        BezierParameter2::Exact(Real::one()),
                        BezierParameter2::Exact(Real::from(2)),
                        true,
                    ),
                ] {
                    let point = |x| Point2::new(Real::from(x), y.clone());
                    let source = if elevated {
                        BezierSubcurve2::Rational(
                            RationalBezier2::try_new_with_exact_line_image(
                                vec![point(origin), point(origin + delta)],
                                vec![Real::one(); 2],
                                LineSeg2::try_new(point(origin), point(origin + delta)).unwrap(),
                            )
                            .unwrap()
                            .elevated_to_degree(5)
                            .unwrap(),
                        )
                    } else {
                        BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                            point(origin),
                            point(origin + delta / 2),
                            point(origin + delta),
                        ))
                    };
                    let line =
                        Curve2::from_retained_fragment(BezierSplitFragment2::RetainedBezier {
                            reversed: false,
                            source_curve: source,
                            start,
                            end,
                            start_image: None,
                            end_image: None,
                        });
                    for reverse_circle in [false, true] {
                        for reverse_line in [false, true] {
                            let circle = oriented(&circle, reverse_circle, &policy);
                            let line = oriented(&line, reverse_line, &policy);
                            for (swapped, (first, second)) in
                                [(&circle, &line), (&line, &circle)].into_iter().enumerate()
                            {
                                let outcome = first.intersect_curve(second, &policy).unwrap();
                                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                                assert!(
                                    outcome.value.is_complete(),
                                    "{:?}",
                                    outcome.value.blockers()
                                );
                                assert_eq!(
                                    outcome.value.contacts().len(),
                                    usize::from(y != Real::from(2))
                                );
                                assert!(outcome.value.overlaps().is_empty());
                                for contact in outcome.value.contacts() {
                                    let negative = reverse_circle
                                        ^ reverse_line
                                        ^ (delta < 0)
                                        ^ (swapped == 1);
                                    assert_eq!(
                                        contact.tangent_cross_sign(),
                                        Some(if negative {
                                            hyperreal::RealSign::Negative
                                        } else {
                                            hyperreal::RealSign::Positive
                                        })
                                    );
                                    assert!(contact.is_certified_transverse());
                                }
                                replay(first, second, &outcome.value, &policy);
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_affine_clipping_replays_exact_boundary_identity() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = circle(0, 2, &policy);
            // P(t)=(-3+2t,1/2) meets the left half of the unit circle at
            // t=3/2-sqrt(3)/4. Isolate the same scalar independently of its
            // represented radical so endpoint equality must replay evidence.
            let polynomial = exact(
                crate::BezierParameterPolynomial::try_new_power_basis(
                    vec![Real::from(33), Real::from(-48), Real::from(16)],
                    &policy,
                )
                .unwrap(),
            );
            let interval = exact(
                crate::BezierParameterInterval::try_new_ordered(Real::one(), q(9, 8), &policy)
                    .unwrap(),
            );
            let selected = BezierParameter2::algebraic(exact(
                crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
                    .unwrap(),
            ));
            for boundary in [
                BezierParameter2::Exact(q(3, 2) - q(3, 16).sqrt().unwrap()),
                selected,
            ] {
                for (start, end) in [
                    (boundary.clone(), BezierParameter2::Exact(Real::from(2))),
                    (BezierParameter2::Exact(Real::one()), boundary.clone()),
                ] {
                    let point = |x| Point2::new(Real::from(x), q(1, 2));
                    let line =
                        Curve2::from_retained_fragment(BezierSplitFragment2::RetainedBezier {
                            reversed: false,
                            source_curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                                point(-3),
                                point(-2),
                                point(-1),
                            )),
                            start,
                            end,
                            start_image: None,
                            end_image: None,
                        });
                    for reverse_line in [false, true] {
                        let line = oriented(&line, reverse_line, &policy);
                        for (first, second) in [(&circle, &line), (&line, &circle)] {
                            let result = query(first, second, &policy);
                            let [contact] = result.contacts() else {
                                panic!("one transverse boundary contact")
                            };
                            assert!(contact.is_certified_transverse());
                            let point = line
                                .point_at(&CurveParameter2::from(boundary.clone()), &policy)
                                .unwrap();
                            assert_eq!(point.certainty, CurveCertainty::Certified);
                            same(&point.value, contact.point(), &policy);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_affine_parallel_domains_preserve_source_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = circle(0, 2, &policy);
            for y in [Real::zero(), q(1, 2), Real::from(2)] {
                for (origin, delta, start, end) in
                    [(-1, 2, 0, 1), (-3, 2, 1, 2), (3, -2, 1, 2), (3, 2, -2, -1)]
                {
                    let point = |x| Point2::new(Real::from(x), y.clone());
                    let image = LineSeg2::try_new(point(origin), point(origin + delta)).unwrap();
                    for distance in [Real::one(), Real::from(-1)] {
                        let source = image.offset_left(-distance.clone()).unwrap();
                        let parallel = QuadraticBezier2::from_line_segment(source)
                            .parallel_left(distance)
                            .unwrap();
                        // A constant nonzero tangent certifies this entire
                        // affine source domain, including its exterior part.
                        let line = Curve2::from_retained_fragment(
                            CurveSupport2::Parallel(parallel)
                                .restrict_certified(
                                    CurveParameterRange2::new_validated(
                                        Real::from(start).into(),
                                        Real::from(end).into(),
                                    ),
                                    None,
                                    false,
                                    &policy,
                                )
                                .unwrap(),
                        );
                        for reverse_line in [false, true] {
                            let line = oriented(&line, reverse_line, &policy);
                            for (swapped, (first, second)) in
                                [(&circle, &line), (&line, &circle)].into_iter().enumerate()
                            {
                                let result = query(first, second, &policy);
                                assert_eq!(
                                    result.contacts().len(),
                                    usize::from(y != Real::from(2))
                                );
                                assert!(result.overlaps().is_empty());
                                for contact in result.contacts() {
                                    assert!(contact.is_certified_transverse());
                                    assert_eq!(
                                        contact.tangent_cross_sign(),
                                        Some(if reverse_line ^ (delta < 0) ^ (swapped == 1) {
                                            hyperreal::RealSign::Negative
                                        } else {
                                            hyperreal::RealSign::Positive
                                        })
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_fillet_tangency_replays_an_exterior_affine_source_chart() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for [a, b, c, d] in [[1, 0, 0, 1], [1, 1, -1, 1]] {
                let p = |x: i32, y: i32| Point2::from_values(a * x + b * y + 2, c * x + d * y - 3);
                let scale = Real::from(a * a + c * c).sqrt().unwrap();
                let path = crate::CurvePath2::try_new(vec![
                    LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
                    QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
                ])
                .unwrap();
                let crate::CurveCornerSolutions2::Unique(path) = path
                    .fillet_vertex_by_radius(
                        1,
                        scale * q(1, 4),
                        crate::CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap()
                    .value
                else {
                    panic!("unique fillet")
                };
                let circle = &path.curves()[1];
                let line = Curve2::from_retained_fragment(BezierSplitFragment2::RetainedBezier {
                    reversed: false,
                    source_curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        p(-3, 0),
                        p(-2, 0),
                        p(-1, 0),
                    )),
                    start: BezierParameter2::Exact(Real::one()),
                    end: BezierParameter2::Exact(Real::from(2)),
                    start_image: None,
                    end_image: None,
                });
                let parallel = QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(p(-3, 0), p(-1, 0))
                        .unwrap()
                        .offset_left(-q(1, 3))
                        .unwrap(),
                )
                .parallel_left(q(1, 3))
                .unwrap();
                let analytic = Curve2::from_retained_fragment(
                    CurveSupport2::Parallel(parallel)
                        .restrict_certified(
                            CurveParameterRange2::new_validated(
                                Real::one().into(),
                                Real::from(2).into(),
                            ),
                            None,
                            false,
                            &policy,
                        )
                        .unwrap(),
                );
                for line in [line, analytic] {
                    for reverse_line in [false, true] {
                        let line = oriented(&line, reverse_line, &policy);
                        for (first, second) in [(circle, &line), (&line, circle)] {
                            let outcome = first.intersect_curve(second, &policy).unwrap();
                            assert_eq!(outcome.certainty, CurveCertainty::Certified);
                            assert!(
                                outcome.value.is_complete(),
                                "{:?}",
                                outcome.value.blockers()
                            );
                            let [contact] = outcome.value.contacts() else {
                                panic!("one tangency")
                            };
                            assert_eq!(
                                contact.tangent_cross_sign(),
                                Some(hyperreal::RealSign::Zero)
                            );
                            same(contact.point(), &circle.start(), &policy);
                            replay(first, second, &outcome.value, &policy);
                            let topology = first.intersection_topology(second, &policy).unwrap();
                            assert_eq!(topology.certainty, CurveCertainty::Certified);
                            assert!(topology.value.result().is_complete());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_intersects_rational_curves_and_retained_chords() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = circle(0, 2, &policy);
            let chord = exact(
                crate::BezierAlgebraicChord2::try_new(p(-2, 0).into(), p(2, 0).into(), &policy)
                    .unwrap(),
            );
            let root = (Real::from(5).sqrt().unwrap() - Real::one()) * q(1, 2);
            let nonlinear_point = Point2::new(-root.clone().sqrt().unwrap(), root);
            for (other, point, tangent) in [
                (
                    Curve2::from(LineSeg2::try_new(p(-2, 0), p(2, 0)).unwrap()),
                    p(-1, 0),
                    false,
                ),
                (
                    Curve2::try_polynomial_bspline(
                        1,
                        vec![p(-2, 0), p(0, 0), p(2, 0)],
                        vec![
                            Real::from(4),
                            Real::from(4),
                            Real::from(6),
                            Real::from(10),
                            Real::from(10),
                        ],
                        &policy,
                    )
                    .unwrap()
                    .value,
                    p(-1, 0),
                    false,
                ),
                (
                    Curve2::try_nurbs(
                        1,
                        vec![p(-2, 0), p(0, 0), p(2, 0)],
                        vec![Real::one(), Real::from(2), Real::one()],
                        vec![
                            Real::from(4),
                            Real::from(4),
                            Real::from(6),
                            Real::from(10),
                            Real::from(10),
                        ],
                        &policy,
                    )
                    .unwrap()
                    .value,
                    p(-1, 0),
                    false,
                ),
                (
                    Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(chord)),
                    p(-1, 0),
                    false,
                ),
                (
                    Curve2::from(QuadraticBezier2::new(p(-2, 0), p(0, 2), p(2, 0))),
                    p(0, 1),
                    true,
                ),
                (
                    Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4))),
                    nonlinear_point,
                    false,
                ),
            ] {
                for a in [false, true] {
                    for b in [false, true] {
                        for swapped in [false, true] {
                            let first = oriented(&source, a, &policy);
                            let second = oriented(&other, b, &policy);
                            let (first, second) = if swapped {
                                (&second, &first)
                            } else {
                                (&first, &second)
                            };
                            let result = query(first, second, &policy);
                            assert!(result.overlaps().is_empty());
                            assert_eq!(result.contacts().len(), 1);
                            same(result.contacts()[0].point(), &point.clone().into(), &policy);
                            assert_eq!(result.contacts()[0].is_certified_transverse(), !tangent);
                        }
                    }
                }
            }
        }
    }

    fn rational_semicircle(policy: &CurveContext) -> Curve2 {
        use crate::HomogeneousControl2;
        exact(
            RationalBezier2::from_homogeneous_controls(
                vec![
                    HomogeneousControl2::new(Real::zero(), Real::one(), Real::one()),
                    HomogeneousControl2::new(Real::from(-1), Real::zero(), Real::zero()),
                    HomogeneousControl2::new(Real::zero(), Real::from(-1), Real::one()),
                ],
                policy,
            )
            .unwrap(),
        )
        .into()
    }

    #[test]
    fn retained_quadratic_circle_pairs_clip_components_and_endpoint_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for major in [false, true] {
                let arc =
                    crate::CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), major).unwrap();
                let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
                let weight = q(1, 2).sqrt().unwrap();
                let source = RationalBezier2::try_new(
                    vec![p(1, 0), p(1, 1), p(0, 1)],
                    vec![
                        Real::one(),
                        if major { -weight } else { weight },
                        Real::one(),
                    ],
                )
                .unwrap()
                .with_implicit_quadratic_conic(implicit, Some(circle));
                for degree in [2, 5] {
                    let source = source.elevated_to_degree(degree).unwrap();
                    let retain = |a, b| {
                        Curve2::from_retained_fragment(BezierSplitFragment2::RetainedBezier {
                            source_curve: BezierSubcurve2::Rational(source.clone()),
                            start: BezierParameter2::Exact(q(a, 8)),
                            end: BezierParameter2::Exact(q(b, 8)),
                            reversed: false,
                            start_image: None,
                            end_image: None,
                        })
                    };
                    // Each discovery starts with the same complete circle
                    // chart. Its retained intervals own a component, one
                    // isolated endpoint, or an empty intersection.
                    for (a, b, contacts, overlaps) in [
                        ((2, 6), (4, 8), 0, 1),
                        ((0, 4), (4, 8), 1, 0),
                        ((1, 3), (5, 7), 0, 0),
                    ] {
                        let first = retain(a.0, a.1);
                        let second = retain(b.0, b.1);
                        for first_reversed in [false, true] {
                            for second_reversed in [false, true] {
                                let first = oriented(&first, first_reversed, &policy);
                                let second = oriented(&second, second_reversed, &policy);
                                for (first, second) in [(&first, &second), (&second, &first)] {
                                    let result = query(first, second, &policy);
                                    assert_eq!(result.contacts().len(), contacts);
                                    assert_eq!(result.overlaps().len(), overlaps);
                                    assert!(result.parameter_components().is_empty());
                                    if let Some(overlap) = result.overlaps().first() {
                                        let mut endpoints = [
                                            overlap.first_range().start().scalar().unwrap().clone(),
                                            overlap.first_range().end().scalar().unwrap().clone(),
                                        ];
                                        if endpoints[0] == q(6, 8) {
                                            endpoints.reverse();
                                        }
                                        assert_eq!(endpoints, [q(4, 8), q(6, 8)]);
                                        assert!(overlap.includes_start() && overlap.includes_end());
                                        assert_eq!(
                                            overlap.orientation(),
                                            if first_reversed == second_reversed {
                                                RationalBezierOverlapOrientation2::Same
                                            } else {
                                                RationalBezierOverlapOrientation2::Reversed
                                            }
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_overlaps_preserve_oriented_ranges_and_singleton_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = circle(0, 2, &policy);
            for other in [circle(0, 3, &policy), rational_semicircle(&policy)] {
                for (first_range, second_range, contacts, overlaps) in [
                    ((q(0, 1), q(1, 1)), (q(0, 1), q(1, 1)), 0, 1),
                    ((q(1, 4), q(3, 4)), (q(1, 2), q(1, 1)), 0, 1),
                    ((q(0, 1), q(1, 2)), (q(1, 2), q(1, 1)), 1, 0),
                    ((q(0, 1), q(1, 4)), (q(3, 4), q(1, 1)), 0, 0),
                ] {
                    let first = source
                        .subcurve(first_range.0.into(), first_range.1.into(), &policy)
                        .unwrap()
                        .value;
                    let second = other
                        .subcurve(second_range.0.into(), second_range.1.into(), &policy)
                        .unwrap()
                        .value;
                    for a in [false, true] {
                        for b in [false, true] {
                            for swapped in [false, true] {
                                let first = oriented(&first, a, &policy);
                                let second = oriented(&second, b, &policy);
                                let (first, second) = if swapped {
                                    (&second, &first)
                                } else {
                                    (&first, &second)
                                };
                                let result = query(first, second, &policy);
                                assert_eq!(result.contacts().len(), contacts);
                                assert_eq!(result.overlaps().len(), overlaps);
                                if contacts == 1 {
                                    same(result.contacts()[0].point(), &p(-1, 0).into(), &policy);
                                }
                                if let Some(overlap) = result.overlaps().first() {
                                    assert_eq!(
                                        overlap.orientation(),
                                        if a ^ b {
                                            RationalBezierOverlapOrientation2::Reversed
                                        } else {
                                            RationalBezierOverlapOrientation2::Same
                                        }
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_and_analytic_parallel_keep_contact_authority() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = circle(0, 2, &policy);
            let parallel = QuadraticBezier2::new(p(-2, 0), p(0, 0), p(2, 0))
                .parallel_left(q(1, 2))
                .unwrap();
            let fragment = exact(
                crate::BezierParallelFragment2::try_new(
                    parallel,
                    BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    &policy,
                )
                .unwrap(),
            );
            let other =
                Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(fragment));
            for a in [false, true] {
                for b in [false, true] {
                    for swapped in [false, true] {
                        let first = oriented(&source, a, &policy);
                        let second = oriented(&other, b, &policy);
                        let (first, second) = if swapped {
                            (&second, &first)
                        } else {
                            (&first, &second)
                        };
                        let result = query(first, second, &policy);
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), 1);
                        same(
                            result.contacts()[0].point(),
                            &Point2::new(-q(3, 4).sqrt().unwrap(), q(1, 2)).into(),
                            &policy,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_and_non_ph_parallel_keep_both_crossings() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = circle(0, 2, &policy);
            let parallel = QuadraticBezier2::new(p(-2, 0), p(-1, 0), p(0, 1))
                .parallel_left(q(1, 8))
                .unwrap();
            let fragment = exact(
                crate::BezierParallelFragment2::try_new(
                    parallel,
                    BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    &policy,
                )
                .unwrap(),
            );
            let other =
                Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(fragment));
            for swapped in [false, true] {
                let (first, second) = if swapped {
                    (&other, &source)
                } else {
                    (&source, &other)
                };
                let result = query(first, second, &policy);
                assert!(result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), 2);
                assert!(
                    result
                        .contacts()
                        .iter()
                        .all(|contact| contact.is_certified_transverse())
                );
            }
        }
    }

    #[test]
    fn selected_circle_retains_only_its_finite_source_frame() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // P(t) = ((2t^2 - 1)/(2t - 1), 0) has a pole at 1/2.
            // At alpha = sqrt(1/2), its point is the origin and its
            // tangent points right: the derivative numerator is
            // 4(t - 1/2)^2 + 1. The retained circle is therefore the
            // ordinary radius-one left semicircle, despite the remote pole.
            let source = exact(
                RationalBezier2::from_homogeneous_controls(
                    [(-1, -1), (-1, 0), (1, 1)]
                        .into_iter()
                        .map(|(x, w)| {
                            crate::HomogeneousControl2::new(
                                Real::from(x),
                                Real::zero(),
                                Real::from(w),
                            )
                        })
                        .collect(),
                    &policy,
                )
                .unwrap(),
            )
            .parallel_left(Real::zero())
            .unwrap();
            assert!(matches!(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    source.clone(),
                    BezierParameter2::Exact(q(1, 2)),
                    Real::one(),
                    false,
                    &policy,
                )
                .unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary),
            ));
            let polynomial = exact(
                crate::BezierParameterPolynomial::try_new_power_basis(
                    vec![Real::from(-1), Real::zero(), Real::from(2)],
                    &policy,
                )
                .unwrap(),
            );
            let interval =
                exact(crate::BezierParameterInterval::try_new(q(1, 2), q(3, 4), &policy).unwrap());
            let parameter = exact(
                crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
                    .unwrap(),
            );
            let circle = exact(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    source,
                    BezierParameter2::algebraic(parameter),
                    Real::one(),
                    false,
                    &policy,
                )
                .unwrap(),
            )
            .unwrap();
            let circle =
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(circle, &policy),
                ));
            let overlap = query(&circle, &rational_semicircle(&policy), &policy);
            assert_eq!(overlap.overlaps().len(), 1);
            assert!(overlap.contacts().is_empty());
            let line = Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap());
            let contact = query(&circle, &line, &policy);
            assert_eq!(contact.contacts().len(), 1);
            same(contact.contacts()[0].point(), &p(-1, 0).into(), &policy);

            let parallel = QuadraticBezier2::new(p(-2, 0), p(-1, 0), p(0, 1))
                .parallel_left(q(1, 8))
                .unwrap();
            let parallel =
                Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(exact(
                    crate::BezierParallelFragment2::try_new(
                        parallel,
                        BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                        &policy,
                    )
                    .unwrap(),
                )));
            for first_reversed in [false, true] {
                for second_reversed in [false, true] {
                    let first = oriented(&circle, first_reversed, &policy);
                    let second = oriented(&parallel, second_reversed, &policy);
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&second, &first)
                        } else {
                            (&first, &second)
                        };
                        let result = query(first, second, &policy);
                        assert_eq!(result.contacts().len(), 2);
                        assert!(result.overlaps().is_empty());
                        assert!(
                            result
                                .contacts()
                                .iter()
                                .all(|contact| contact.is_certified_transverse())
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_split_results_reenter_intersections() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = circle(0, 2, &policy);
            let line = Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap());
            let topology = source.intersection_topology(&line, &policy).unwrap();
            assert_eq!(topology.certainty, CurveCertainty::Certified);
            assert_eq!(topology.value.first().len(), 2);
            assert_eq!(topology.value.second().len(), 2);
            for piece in topology.value.first() {
                let result = query(piece, piece, &policy);
                assert_eq!(result.overlaps().len(), 1);
                assert!(result.contacts().is_empty());
            }
            let result = query(
                &topology.value.first()[0],
                &topology.value.first()[1],
                &policy,
            );
            assert_eq!(result.contacts().len(), 1);
            assert!(result.overlaps().is_empty());
            same(result.contacts()[0].point(), &p(-1, 0).into(), &policy);
        }
    }
}

#[cfg(test)]
mod analytic_dispatch_tests {
    use super::circle_dispatch_tests::{exact, oriented, query, same};
    use super::*;
    use crate::{
        BezierParallel2, BezierParallelFragment2, CurveCertainty, CurvePath2, LineSeg2,
        QuadraticBezier2,
    };

    fn p(x: i32, y: i32) -> Point2 {
        Point2::from_values(x, y)
    }
    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn certified<T>(outcome: CurveOutcome<T>) -> T {
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        outcome.value
    }
    fn curve(parallel: BezierParallel2, policy: &CurveContext) -> Curve2 {
        let fragment = exact(
            BezierParallelFragment2::try_new(
                parallel,
                BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                policy,
            )
            .unwrap(),
        );
        Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(fragment))
    }
    fn parabola(policy: &CurveContext) -> Curve2 {
        curve(
            QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1))
                .parallel_left(Real::one())
                .unwrap(),
            policy,
        )
    }
    fn trim(curve: &Curve2, start: Real, end: Real, policy: &CurveContext) -> Curve2 {
        certified(curve.subcurve(start.into(), end.into(), policy).unwrap())
    }

    #[test]
    fn common_analytic_pairs_replay_contacts_in_both_orders_and_traversals() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = parabola(&policy);
            let second = curve(
                QuadraticBezier2::new(p(1, 1), p(1, 2), p(2, 3))
                    .parallel_left(Real::one())
                    .unwrap(),
                &policy,
            );
            let line = Curve2::from(
                LineSeg2::try_new(
                    Point2::new((-1).into(), q(3, 2)),
                    Point2::new(3.into(), q(3, 2)),
                )
                .unwrap(),
            );
            let chord =
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(exact(
                    crate::BezierAlgebraicChord2::try_new(first.end(), first.start(), &policy)
                        .unwrap(),
                )));
            for (other, count) in [(&line, 1), (&chord, 2), (&second, 1)] {
                for reversed in [false, true] {
                    let a = oriented(&first, reversed, &policy);
                    for swapped in [false, true] {
                        let (a, b) = if swapped { (other, &a) } else { (&a, other) };
                        let result = query(a, b, &policy);
                        assert_eq!(result.contacts().len(), count);
                        assert!(result.overlaps().is_empty());
                        assert!(result.parameter_components().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn common_analytic_overlaps_preserve_clips_and_singleton_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = parabola(&policy);
            let middle = trim(&first, q(1, 4), q(3, 4), &policy);
            let left = trim(&first, Real::zero(), q(1, 2), &policy);
            let right = trim(&first, q(1, 2), Real::one(), &policy);
            for reversed in [false, true] {
                let middle = oriented(&middle, reversed, &policy);
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&middle, &first)
                    } else {
                        (&first, &middle)
                    };
                    let result = query(a, b, &policy);
                    assert_eq!(result.overlaps().len(), 1);
                    assert!(result.contacts().is_empty());
                    let overlap = &result.overlaps()[0];
                    assert!(overlap.includes_start() && overlap.includes_end());
                    assert_eq!(
                        overlap.orientation(),
                        if reversed {
                            RationalBezierOverlapOrientation2::Reversed
                        } else {
                            RationalBezierOverlapOrientation2::Same
                        }
                    );
                    let smaller =
                        CurveParameterRange2::new_validated(q(3, 8).into(), q(5, 8).into());
                    let ranges = exact(
                        overlap
                            .parameter_correspondence
                            .clipped_ranges(&smaller, &smaller, &policy)
                            .unwrap(),
                    )
                    .unwrap();
                    assert_eq!(ranges.0.scalar_endpoints(), Some((&q(3, 8), &q(5, 8))));
                    assert_eq!(ranges.1.scalar_endpoints(), Some((&q(3, 8), &q(5, 8))));
                }
            }
            for (a, b) in [(&left, &right), (&right, &left)] {
                let result = query(a, b, &policy);
                assert_eq!(result.contacts().len(), 1);
                assert!(result.overlaps().is_empty());
                assert_eq!(
                    result.contacts()[0].first().local_parameter().scalar(),
                    Some(&q(1, 2))
                );
                assert_eq!(
                    result.contacts()[0].second().local_parameter().scalar(),
                    Some(&q(1, 2))
                );
            }
        }
    }

    #[test]
    fn common_analytic_overlaps_transport_distinct_source_charts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 1));
            let subcurve = source
                .subcurve_between_exact(&q(1, 4), &q(3, 4), &policy)
                .unwrap();
            let first = curve(source.parallel_left(Real::one()).unwrap(), &policy);
            let second = subcurve.parallel_left(Real::one()).unwrap();
            for reverse_source in [false, true] {
                let second = curve(
                    if reverse_source {
                        second.reversed()
                    } else {
                        second.clone()
                    },
                    &policy,
                );
                let a = trim(&first, q(3, 8), q(5, 8), &policy);
                let b = trim(&second, q(1, 4), q(3, 4), &policy);
                for swapped in [false, true] {
                    let (a, b) = if swapped { (&b, &a) } else { (&a, &b) };
                    let result = query(a, b, &policy);
                    assert_eq!(result.overlaps().len(), 1);
                    assert!(result.contacts().is_empty());
                    assert_eq!(
                        result.overlaps()[0].orientation(),
                        if reverse_source {
                            RationalBezierOverlapOrientation2::Reversed
                        } else {
                            RationalBezierOverlapOrientation2::Same
                        }
                    );
                }
                let disjoint = trim(&first, Real::zero(), q(1, 8), &policy);
                assert!(query(&disjoint, &second, &policy).is_disjoint());
            }
            let parallel = curve(
                QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
                    .parallel_left(Real::one())
                    .unwrap(),
                &policy,
            );
            let rational = Curve2::from(LineSeg2::try_new(p(0, 1), p(2, 1)).unwrap());
            let chord =
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(exact(
                    crate::BezierAlgebraicChord2::try_new(p(2, 1).into(), p(0, 1).into(), &policy)
                        .unwrap(),
                )));
            let parallel = trim(&parallel, q(1, 4), q(3, 4), &policy);
            for other in [&rational, &chord] {
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (other, &parallel)
                    } else {
                        (&parallel, other)
                    };
                    let result = query(a, b, &policy);
                    assert_eq!(result.overlaps().len(), 1);
                    assert!(result.contacts().is_empty());
                }
            }
        }
    }

    #[test]
    fn common_parameter_components_survive_paths_cuts_and_cloning() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let line = curve(
                QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
                    .parallel_left(Real::one())
                    .unwrap(),
                &policy,
            );
            let constant = Curve2::from(QuadraticBezier2::new(p(1, 1), p(1, 1), p(1, 1)));
            let constant = trim(&constant, q(1, 4), q(3, 4), &policy);
            for swapped in [false, true] {
                let (a, b) = if swapped {
                    (&constant, &line)
                } else {
                    (&line, &constant)
                };
                let result = query(a, b, &policy);
                assert!(!result.is_disjoint());
                assert!(result.contacts().is_empty() && result.overlaps().is_empty());
                let [component] = result.parameter_components() else {
                    panic!("one complete point-image component")
                };
                let (fixed, free) = if swapped {
                    (component.second_parameters(), component.first_parameters())
                } else {
                    (component.first_parameters(), component.second_parameters())
                };
                assert!(
                    matches!(fixed,CurveParameterSet2::Single(parameter) if parameter.scalar()==Some(&q(1,2)))
                );
                assert!(matches!(free, CurveParameterSet2::Range(_)));
                same(component.point(), &p(1, 1).into(), &policy);
                let cloned = result.clone();
                assert!(std::ptr::eq(
                    result.parameter_components().as_ptr(),
                    cloned.parameter_components().as_ptr()
                ));
                let first_path = CurvePath2::try_new(vec![a.clone()]).unwrap();
                let second_path = CurvePath2::try_new(vec![b.clone()]).unwrap();
                let path_result =
                    certified(first_path.intersect_path(&second_path, &policy).unwrap());
                assert!(path_result.is_complete() && !path_result.is_disjoint());
                assert_eq!(path_result.parameter_components().len(), 1);
                assert_eq!(path_result.parameter_components()[0].component(), component);
                let topology = certified(
                    first_path
                        .intersection_topology(&second_path, &policy)
                        .unwrap(),
                );
                assert_eq!(topology.result().parameter_components().len(), 1);
                let pieces = if swapped {
                    topology.second()
                } else {
                    topology.first()
                };
                assert_eq!(pieces[0].curves().len(), 2);
                let curve_topology = certified(a.intersection_topology(b, &policy).unwrap());
                assert_eq!(curve_topology.result().parameter_components().len(), 1);
            }
            let excluded = trim(&line, Real::zero(), q(1, 4), &policy);
            assert!(query(&excluded, &constant, &policy).is_disjoint());
        }
    }

    #[test]
    fn coincident_constant_parallels_retain_the_parameter_rectangle() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = curve(
                QuadraticBezier2::new(p(3, 4), p(3, 4), p(3, 4))
                    .parallel_left(Real::zero())
                    .unwrap(),
                &policy,
            );
            let a = trim(&first, q(1, 8), q(3, 8), &policy);
            let b = trim(&first, q(5, 8), q(7, 8), &policy);
            let result = query(&a, &b, &policy);
            assert!(!result.is_disjoint());
            let [component] = result.parameter_components() else {
                panic!("one parameter rectangle")
            };
            for (domain, start, end) in [
                (component.first_parameters(), q(1, 8), q(3, 8)),
                (component.second_parameters(), q(5, 8), q(7, 8)),
            ] {
                let CurveParameterSet2::Range(range) = domain else {
                    panic!("whole retained domain")
                };
                assert_eq!(range.scalar_endpoints(), Some((&start, &end)));
            }
        }
    }
    fn selected(value: Real, policy: &CurveContext) -> CurveParameter2 {
        let polynomial = exact(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![(-1).into(), Real::zero(), 2.into()],
                policy,
            )
            .unwrap(),
        );
        let interval =
            exact(crate::BezierParameterInterval::try_new(q(1, 2), q(3, 4), policy).unwrap());
        let root = exact(
            crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        );
        CurveParameter2::from_selected_fiber(
            crate::bezier_offset::exact_selected_fiber_parameter_for_test(root, value, policy),
        )
    }

    fn finite_rational(
        source: RationalBezier2,
        bounds: [Real; 2],
        selected_bounds: bool,
        policy: &CurveContext,
    ) -> Curve2 {
        let images = selected_bounds.then(|| {
            bounds.each_ref().map(|parameter| {
                CurvePoint2::from(exact(source.point_at_affine_classified(parameter, policy)))
            })
        });
        let [start, end] = bounds.map(|value| {
            if selected_bounds {
                selected(value, policy)
            } else {
                value.into()
            }
        });
        Curve2::from_retained_fragment(
            CurveSupport2::Bezier(BezierSubcurve2::Rational(source))
                .restrict_certified(
                    CurveParameterRange2::new_validated(start, end),
                    images,
                    false,
                    policy,
                )
                .unwrap(),
        )
    }

    fn retained_chord(start: Point2, end: Point2, policy: &CurveContext) -> Curve2 {
        Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(exact(
            crate::BezierAlgebraicChord2::try_new(start.into(), end.into(), policy).unwrap(),
        )))
    }

    #[test]
    fn finite_rational_pairs_replay_all_roots_and_stationary_points() {
        finite_rational_contact_cases(false);
    }

    #[test]
    fn finite_chord_rational_contacts_replay_all_roots_and_stationary_points() {
        finite_rational_contact_cases(true);
    }

    fn finite_rational_contact_cases(retained_line: bool) {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let cutter = |a: Point2, b: Point2| {
                if retained_line {
                    retained_chord(a, b, &policy)
                } else {
                    Curve2::from(LineSeg2::try_new(a, b).unwrap())
                }
            };
            let parabola = RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1)),
            ))
            .unwrap();
            // y=(t-2)(t-3)(t-4): three exterior roots in the original chart.
            let cubic = RationalBezier2::try_new(
                vec![
                    p(0, -24),
                    Point2::new(q(1, 3), -q(46, 3)),
                    Point2::new(q(2, 3), -q(29, 3)),
                    p(1, -6),
                ],
                vec![Real::one(); 4],
            )
            .unwrap();
            // (x,y)=((t-2)^2,(t-2)^3): the contact at t=2 has zero speed.
            let cusp = RationalBezier2::try_new(
                vec![
                    p(4, -8),
                    Point2::new(q(8, 3), (-4).into()),
                    Point2::new(q(5, 3), (-2).into()),
                    p(1, -1),
                ],
                vec![Real::one(); 4],
            )
            .unwrap();
            for (source, bounds, chord, count, stationary) in [
                (
                    parabola.clone(),
                    [1.into(), 2.into()],
                    cutter(p(0, 2), p(2, 2)),
                    1,
                    false,
                ),
                (
                    parabola,
                    [1.into(), 2.into()],
                    cutter(p(1, 1), p(2, 4)),
                    2,
                    false,
                ),
                (
                    cubic,
                    [1.into(), 5.into()],
                    cutter(p(0, 0), p(6, 0)),
                    3,
                    false,
                ),
                (
                    cusp,
                    [1.into(), 3.into()],
                    cutter(p(0, -1), p(0, 1)),
                    1,
                    true,
                ),
            ] {
                for selected_bounds in [false, true] {
                    let source =
                        finite_rational(source.clone(), bounds.clone(), selected_bounds, &policy);
                    for reversed in [false, true] {
                        let source = oriented(&source, reversed, &policy);
                        for (a, b) in [(&source, &chord), (&chord, &source)] {
                            let result = query(a, b, &policy);
                            assert_eq!(result.contacts().len(), count);
                            assert!(result.overlaps().is_empty());
                            assert!(result.parameter_components().is_empty());
                            if stationary {
                                assert_eq!(
                                    result.contacts()[0].tangent_cross_sign(),
                                    Some(hyperreal::RealSign::Zero)
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn finite_chord_rational_retracing_keeps_all_overlap_branches() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0)),
            ))
            .unwrap();
            for selected_bounds in [false, true] {
                let source = finite_rational(
                    source.clone(),
                    [(-2).into(), 2.into()],
                    selected_bounds,
                    &policy,
                );
                for (start, end, contacts, overlaps) in [(1, 4, 0, 2), (0, 1, 0, 2), (-1, 0, 1, 0)]
                {
                    let chord = retained_chord(p(start, 0), p(end, 0), &policy);
                    for reversed in [false, true] {
                        let source = oriented(&source, reversed, &policy);
                        for (a, b) in [(&source, &chord), (&chord, &source)] {
                            let result = query(a, b, &policy);
                            assert_eq!(result.contacts().len(), contacts);
                            assert_eq!(result.overlaps().len(), overlaps);
                            for overlap in result.overlaps() {
                                let CurveOverlapCorrespondence2::ChordRational { source, .. } =
                                    &overlap.parameter_correspondence
                                else {
                                    panic!("the overlap must retain its chord/source transport");
                                };
                                let sample = CurveParameter2::from(exact(
                                    source
                                        .source_range()
                                        .strict_interior_scalar(&policy)
                                        .unwrap(),
                                ));
                                let mapped = exact(
                                    source
                                        .chord_parameter_at_source_parameter(&sample, &policy)
                                        .unwrap(),
                                )
                                .unwrap();
                                let chord_range = CurveParameterRange2::new_validated(
                                    CurveParameter2::from_algebraic_chord(mapped),
                                    CurveParameter2::from_algebraic_chord(
                                        source.chord_range()[1].clone(),
                                    ),
                                );
                                let (_, clipped) = exact(
                                    source
                                        .clipped_ranges(
                                            &chord_range,
                                            source.source_range(),
                                            &policy,
                                        )
                                        .unwrap(),
                                )
                                .unwrap();
                                assert!([clipped.start(), clipped.end()].into_iter().any(
                                    |parameter| {
                                        parameter.same_value(&sample, &policy).unwrap()
                                            == Classification::Decided(true)
                                    }
                                ));
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn finite_chord_rational_point_fibers_and_poles_use_the_active_domain() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let constant =
                RationalBezier2::try_new(vec![p(2, 0); 3], vec![Real::one(); 3]).unwrap();
            let point = finite_rational(constant, [1.into(), 3.into()], true, &policy);
            for (y, expected) in [(0, 1), (1, 0)] {
                let chord = retained_chord(p(0, y), p(4, y), &policy);
                for (a, b) in [(&point, &chord), (&chord, &point)] {
                    let result = query(a, b, &policy);
                    assert_eq!(result.parameter_components().len(), expected);
                    assert!(result.contacts().is_empty());
                    assert!(result.overlaps().is_empty());
                }
            }
            // A pole elsewhere in the unit chart must not constrain a retained
            // finite fragment. The point fiber still owns its entire active range.
            let rootful_point = exact(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        crate::HomogeneousControl2::new(1.into(), 0.into(), 1.into()),
                        crate::HomogeneousControl2::new((-1).into(), 0.into(), (-1).into()),
                    ],
                    &policy,
                )
                .unwrap(),
            );
            let chord = retained_chord(p(0, 0), p(2, 0), &policy);
            for bounds in [[Real::zero(), q(1, 4)], [q(3, 4), Real::one()]] {
                let point = finite_rational(rootful_point.clone(), bounds, true, &policy);
                for (a, b) in [(&point, &chord), (&chord, &point)] {
                    assert_eq!(query(a, b, &policy).parameter_components().len(), 1);
                }
            }
            let undefined = Curve2::from(rootful_point);
            assert!(!certified(chord.intersect_curve(&undefined, &policy).unwrap()).is_complete());
            // x=t/(2-t) is finite on [3,4], while [1,3] contains its pole.
            let source = exact(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        crate::HomogeneousControl2::new(0.into(), 0.into(), 2.into()),
                        crate::HomogeneousControl2::new(1.into(), 0.into(), 1.into()),
                    ],
                    &policy,
                )
                .unwrap(),
            );
            let chord = retained_chord(p(-4, 0), p(-1, 0), &policy);
            let finite = finite_rational(source.clone(), [3.into(), 4.into()], false, &policy);
            assert_eq!(query(&chord, &finite, &policy).overlaps().len(), 1);
            let pole = finite_rational(source, [1.into(), 3.into()], false, &policy);
            let result = certified(chord.intersect_curve(&pole, &policy).unwrap());
            assert!(!result.is_complete());
            assert!(result.contacts().is_empty());
            assert!(result.overlaps().is_empty());
        }
    }

    #[test]
    fn finite_rational_pairs_keep_nonlinear_correspondences_and_residual_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parabola = RationalBezier2::try_new(
                vec![p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1)],
                vec![Real::one(); 3],
            )
            .unwrap();
            // (u^2,u^4) traverses the same parabola with a nonlinear chart.
            let squared = RationalBezier2::try_new(
                vec![
                    p(0, 0),
                    p(0, 0),
                    Point2::new(q(1, 6), Real::zero()),
                    Point2::new(q(1, 2), Real::zero()),
                    p(1, 1),
                ],
                vec![Real::one(); 5],
            )
            .unwrap();
            for selected_bounds in [false, true] {
                let first = finite_rational(
                    parabola.clone(),
                    [1.into(), 4.into()],
                    selected_bounds,
                    &policy,
                );
                let second = finite_rational(
                    squared.clone(),
                    [1.into(), 2.into()],
                    selected_bounds,
                    &policy,
                );
                for reversed in [false, true] {
                    let first = oriented(&first, reversed, &policy);
                    for swapped in [false, true] {
                        let (a, b, source, image) = if swapped {
                            (&second, &first, q(3, 2), q(9, 4))
                        } else {
                            (&first, &second, q(9, 4), q(3, 2))
                        };
                        let result = query(a, b, &policy);
                        let [overlap] = result.overlaps() else {
                            panic!("one nonlinear finite correspondence: {result:?}")
                        };
                        assert!(result.contacts().is_empty());
                        let parameter = selected(source, &policy);
                        let CurveOverlapCorrespondence2::ParameterComponent {
                            source,
                            swapped: false,
                        } = &overlap.parameter_correspondence
                        else {
                            panic!("the finite overlap retains its component proof")
                        };
                        let mapped = exact(
                            source
                                .map_curve_parameter(
                                    hypersolve::CurveResultantParameter::First,
                                    &parameter,
                                    &policy,
                                )
                                .unwrap(),
                        )
                        .unwrap();
                        assert_eq!(
                            mapped.same_value(&image.into(), &policy).unwrap(),
                            Classification::Decided(true)
                        );
                        let restored = exact(
                            source
                                .map_curve_parameter(
                                    hypersolve::CurveResultantParameter::Second,
                                    &mapped,
                                    &policy,
                                )
                                .unwrap(),
                        )
                        .unwrap();
                        assert_eq!(
                            restored.same_value(&parameter, &policy).unwrap(),
                            Classification::Decided(true)
                        );
                        same(
                            &certified(a.point_at(&parameter, &policy).unwrap()),
                            &certified(b.point_at(&mapped, &policy).unwrap()),
                            &policy,
                        );
                    }
                }
            }
            // Identical nodal cubics share the diagonal but also meet at
            // ordered off-diagonal pairs (-2,2) and (2,-2).
            let nodal = RationalBezier2::try_new(
                vec![
                    p(0, 0),
                    Point2::new(Real::zero(), -q(4, 3)),
                    Point2::new(q(1, 3), -q(8, 3)),
                    p(1, -3),
                ],
                vec![Real::one(); 4],
            )
            .unwrap();
            let nodal = finite_rational(nodal, [(-3).into(), 3.into()], true, &policy);
            let result = query(&nodal, &nodal, &policy);
            assert_eq!(result.overlaps().len(), 1);
            assert_eq!(result.contacts().len(), 2);
            for contact in result.contacts() {
                assert_eq!(
                    contact
                        .first()
                        .local_parameter()
                        .same_value(contact.second().local_parameter(), &policy)
                        .unwrap(),
                    Classification::Decided(false)
                );
                same(contact.point(), &p(4, 0).into(), &policy);
                assert!(contact.is_certified_transverse());
            }
        }
    }

    #[test]
    fn finite_rational_pairs_keep_retracing_and_point_fibers() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let squared =
                RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 0)], vec![Real::one(); 3])
                    .unwrap();
            let squared = finite_rational(squared, [(-2).into(), 2.into()], true, &policy);
            for (start, end, contacts, overlaps) in [(1, 4, 0, 2), (0, 1, 0, 2), (-1, 0, 1, 0)] {
                let line = Curve2::from(LineSeg2::new_unchecked(p(start, 0), p(end, 0)));
                for (a, b) in [(&squared, &line), (&line, &squared)] {
                    let result = query(a, b, &policy);
                    assert_eq!(result.contacts().len(), contacts);
                    assert_eq!(result.overlaps().len(), overlaps);
                }
            }
            let constant =
                RationalBezier2::try_new(vec![p(2, 0); 2], vec![Real::one(); 2]).unwrap();
            let first = finite_rational(constant.clone(), [2.into(), 3.into()], true, &policy);
            let second = finite_rational(constant, [(-3).into(), (-2).into()], false, &policy);
            assert_eq!(
                query(&first, &second, &policy).parameter_components().len(),
                1
            );
            for (a, b) in [(&squared, &first), (&first, &squared)] {
                let result = query(a, b, &policy);
                assert_eq!(result.parameter_components().len(), 2);
                assert!(result.contacts().is_empty() && result.overlaps().is_empty());
                for component in result.parameter_components() {
                    same(component.point(), &p(2, 0).into(), &policy);
                }
            }
        }
    }

    #[test]
    fn finite_rational_pairs_certify_poles_on_the_active_domain() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = exact(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        crate::HomogeneousControl2::new(0.into(), 0.into(), 2.into()),
                        crate::HomogeneousControl2::new(1.into(), 0.into(), 1.into()),
                    ],
                    &policy,
                )
                .unwrap(),
            );
            let line = Curve2::from(LineSeg2::new_unchecked(p(-4, 0), p(-1, 0)));
            let finite = finite_rational(source.clone(), [3.into(), 4.into()], true, &policy);
            assert_eq!(query(&line, &finite, &policy).overlaps().len(), 1);
            let pole = finite_rational(source, [1.into(), 3.into()], false, &policy);
            assert!(!certified(line.intersect_curve(&pole, &policy).unwrap()).is_complete());
            let rootful_point = exact(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        crate::HomogeneousControl2::new(1.into(), 0.into(), 1.into()),
                        crate::HomogeneousControl2::new((-1).into(), 0.into(), (-1).into()),
                    ],
                    &policy,
                )
                .unwrap(),
            );
            let line = Curve2::from(LineSeg2::new_unchecked(p(0, 0), p(2, 0)));
            for bounds in [
                [Real::zero(), q(1, 4)],
                [q(3, 4), Real::one()],
                [2.into(), 3.into()],
            ] {
                let point = finite_rational(rootful_point.clone(), bounds, true, &policy);
                for (a, b) in [(&point, &line), (&line, &point)] {
                    assert_eq!(query(a, b, &policy).parameter_components().len(), 1);
                }
            }
        }
    }

    #[test]
    fn finite_chord_analytic_parallel_replays_exterior_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel =
                QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1))
                    .parallel_left(Real::one())
                    .unwrap();
            let source = Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(
                BezierParallelFragment2::from_certified_range(
                    parallel,
                    BezierParameterRange2::from_exact(1.into(), 2.into()),
                    false,
                ),
            ));
            let chord = retained_chord(p(1, 0), p(1, 7), &policy);
            for reversed in [false, true] {
                let source = oriented(&source, reversed, &policy);
                for (a, b) in [(&source, &chord), (&chord, &source)] {
                    let result = query(a, b, &policy);
                    assert_eq!(result.contacts().len(), 1);
                    assert!(result.contacts()[0].is_certified_transverse());
                    assert!(result.overlaps().is_empty());
                }
            }
        }
    }

    #[test]
    fn common_nonlinear_overlap_keeps_selected_parameter_charts() {
        // Q(u) is P(t)'s unit parallel through t=(4u-u^2)/(6-3u).
        // Neither its map nor a selected endpoint may be replaced with an
        // affine interpolation or a reconstructed global parameter root.
        let rational = RationalBezier2::try_new(
            vec![
                p(0, 1),
                Point2::new(q(-1, 18), Real::one()),
                Point2::new(q(-15, 134), q(133, 134)),
                Point2::new(q(-43, 264), q(43, 44)),
                Point2::new(q(-117, 580), q(1111, 1160)),
                Point2::new(q(-25, 112), q(211, 224)),
                Point2::new(q(-9, 40), q(301, 320)),
            ],
            vec![
                Real::one(),
                q(3, 4),
                q(67, 120),
                q(33, 80),
                q(29, 96),
                q(7, 32),
                q(5, 32),
            ],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel = curve(
                QuadraticBezier2::new(
                    p(0, 0),
                    Point2::new(q(3, 16), Real::zero()),
                    Point2::new(q(3, 8), q(9, 64)),
                )
                .parallel_left(Real::one())
                .unwrap(),
                &policy,
            );
            let start = selected(q(5, 28), &policy);
            let end = selected(q(13, 20), &policy);
            assert!(start.as_bezier_parameter().is_none());
            let parallel = certified(parallel.subcurve(start, end, &policy).unwrap());
            for other in [
                Curve2::from(rational.clone()),
                curve(rational.parallel_left(Real::zero()).unwrap(), &policy),
            ] {
                let other = certified(
                    other
                        .subcurve(
                            selected(q(1, 4), &policy),
                            selected(q(3, 4), &policy),
                            &policy,
                        )
                        .unwrap(),
                );
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&other, &parallel)
                    } else {
                        (&parallel, &other)
                    };
                    let result = query(a, b, &policy);
                    assert!(result.contacts().is_empty());
                    assert_eq!(result.overlaps().len(), 1);
                    let overlap = &result.overlaps()[0];
                    assert!(overlap.includes_start() && overlap.includes_end());
                    let (first, second) = if swapped {
                        (overlap.second_range(), overlap.first_range())
                    } else {
                        (overlap.first_range(), overlap.second_range())
                    };
                    assert!(first.start().as_bezier_parameter().is_none());
                    assert!(second.start().as_bezier_parameter().is_none());
                    let first_clip = CurveParameterRange2::new_validated(
                        selected(q(5, 28), &policy),
                        selected(q(1, 2), &policy),
                    );
                    let second_clip = CurveParameterRange2::new_validated(
                        selected(q(1, 4), &policy),
                        selected(q(3, 4), &policy),
                    );
                    let (a_clip, b_clip) = if swapped {
                        (&second_clip, &first_clip)
                    } else {
                        (&first_clip, &second_clip)
                    };
                    let restricted = certified(
                        overlap
                            .restrict(
                                [a_clip.start().clone(), a_clip.end().clone()],
                                [b_clip.start().clone(), b_clip.end().clone()],
                                &policy,
                            )
                            .unwrap(),
                    );
                    let restricted = exact(restricted).unwrap();
                    let repeated = exact(certified(
                        restricted
                            .restrict(
                                [
                                    overlap.first_range().start().clone(),
                                    overlap.first_range().end().clone(),
                                ],
                                [
                                    overlap.second_range().start().clone(),
                                    overlap.second_range().end().clone(),
                                ],
                                &policy,
                            )
                            .unwrap(),
                    ))
                    .unwrap();
                    assert_eq!(
                        repeated, restricted,
                        "restriction must never widen to the original map domain"
                    );
                    let clipped = (restricted.first_range(), restricted.second_range());
                    for (x, y) in [
                        (clipped.0.start(), clipped.1.start()),
                        (clipped.0.end(), clipped.1.end()),
                    ] {
                        same(
                            &certified(a.point_at(x, &policy).unwrap()),
                            &certified(b.point_at(y, &policy).unwrap()),
                            &policy,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn stationary_source_touch_replays_the_selected_normal_limits() {
        // P'=(2t-1)^2(1,t) has equal normal limits at t=1/2.
        // P'=(2t-1)(1,t) has opposite limits. Both sources are non-PH,
        // so the complete component projection must retain this distinction.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (source, touches) in [
                (
                    RationalBezier2::try_new(
                        vec![
                            p(0, 0),
                            Point2::new(q(1, 4), Real::zero()),
                            Point2::new(q(1, 6), q(1, 12)),
                            Point2::new(q(1, 12), q(-1, 12)),
                            Point2::new(q(1, 3), q(1, 6)),
                        ],
                        vec![Real::one(); 5],
                    )
                    .unwrap(),
                    true,
                ),
                (
                    RationalBezier2::try_new(
                        vec![
                            p(0, 0),
                            Point2::new(q(-1, 3), Real::zero()),
                            Point2::new(q(-1, 3), q(-1, 6)),
                            Point2::new(Real::zero(), q(1, 6)),
                        ],
                        vec![Real::one(); 4],
                    )
                    .unwrap(),
                    false,
                ),
            ] {
                let parallel = source.parallel_left((-1).into()).unwrap();
                let retained = |start, end| {
                    Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(
                        BezierParallelFragment2::from_certified_range(
                            parallel.clone(),
                            BezierParameterRange2::from_exact(start, end),
                            false,
                        ),
                    ))
                };
                let a = retained(Real::zero(), q(1, 2));
                let b = retained(q(1, 2), Real::one());
                let reversed_source =
                    Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(
                        BezierParallelFragment2::from_certified_range(
                            parallel.reversed(),
                            BezierParameterRange2::from_exact(Real::zero(), q(1, 2)),
                            false,
                        ),
                    ));
                for (first, second) in [
                    (&a, &b),
                    (&b, &a),
                    (&a, &reversed_source),
                    (&reversed_source, &a),
                ] {
                    let result = query(first, second, &policy);
                    assert!(result.overlaps().is_empty());
                    let boundary_contacts = result
                        .contacts()
                        .iter()
                        .filter(|contact| {
                            contact.first().local_parameter().scalar() == Some(&q(1, 2))
                                && contact.second().local_parameter().scalar() == Some(&q(1, 2))
                        })
                        .count();
                    assert_eq!(boundary_contacts, usize::from(touches));
                }
            }
        }
    }

    #[test]
    fn point_image_components_retain_chord_cuts_and_authored_poles() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let chord =
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(exact(
                    crate::BezierAlgebraicChord2::try_new(p(0, 1).into(), p(2, 1).into(), &policy)
                        .unwrap(),
                )));
            let constant = QuadraticBezier2::new(p(1, 1), p(1, 1), p(1, 1));
            for point in [
                Curve2::from(constant.clone()),
                curve(constant.parallel_left(Real::zero()).unwrap(), &policy),
            ] {
                for (a, b) in [(&chord, &point), (&point, &chord)] {
                    let result = query(a, b, &policy);
                    assert_eq!(result.parameter_components().len(), 1);
                    let topology = certified(a.intersection_topology(b, &policy).unwrap());
                    assert_eq!(topology.result().parameter_components().len(), 1);
                }
            }
            let rootful = Curve2::from(exact(
                RationalBezier2::from_homogeneous_controls(
                    [-1, 0, 1]
                        .into_iter()
                        .map(|weight| {
                            crate::HomogeneousControl2::new(
                                weight.into(),
                                weight.into(),
                                weight.into(),
                            )
                        })
                        .collect(),
                    &policy,
                )
                .unwrap(),
            ));
            let line = curve(
                QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))
                    .parallel_left(Real::one())
                    .unwrap(),
                &policy,
            );
            let quadratic = Curve2::from(
                crate::RationalQuadraticBezier2::try_new(
                    p(1, 1),
                    p(1, 1),
                    p(1, 1),
                    (-1).into(),
                    (-1).into(),
                    Real::one(),
                )
                .unwrap(),
            );
            for rootful in [rootful, quadratic] {
                let result = certified(line.intersect_curve(&rootful, &policy).unwrap());
                assert!(!result.is_complete());
                assert!(result.parameter_components().is_empty());
                let finite = trim(&rootful, Real::zero(), q(1, 4), &policy);
                assert_eq!(
                    query(&line, &finite, &policy).parameter_components().len(),
                    1
                );
            }
        }
    }
}
