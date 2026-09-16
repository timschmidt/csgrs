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
}

pub(super) struct Span {
    support: CurveSupport2,
    range: CurveParameterRange2,
    pub(super) chart: CurveSpanRange2,
    reversed: bool,
    self_contacts: OnceLock<ExactCurveResult<RationalBezierIntersectionContacts2>>,
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
            self_contacts: OnceLock::new(),
        })
        .collect())
}

fn contains(
    range: &CurveParameterRange2,
    parameter: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let [lower, upper] = decided(range.ordered_endpoints(policy), family)?;
    if decided(parameter.cmp_by_refinement(lower, policy), family)?.is_lt() {
        return Ok(false);
    }
    Ok(!decided(parameter.cmp_by_refinement(upper, policy), family)?.is_gt())
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
            parameter_correspondence: Some(correspondence),
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
        self.require_unit_domain(span)?;
        let (contacts, overlaps) = match decided(
            chord.rational_intersections(source, None, self.policy),
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
        for overlap in overlaps {
            if let Some(clipped) = decided(
                overlap.clipped_to_source_range(&span.range, self.policy),
                family,
            )? {
                let [start, end] = clipped.chord_range();
                let chord_range = CurveParameterRange2::new_validated(
                    CurveParameter2::from_algebraic_chord(start.clone()),
                    CurveParameter2::from_algebraic_chord(end.clone()),
                );
                let source_range = clipped.source_range().clone();
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

    fn circle_contact(
        &self,
        circle: &crate::BezierAlgebraicCuspSemicircleFragment2,
        circle_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        other_parameter: CurveParameter2,
        point: Option<CurvePoint2>,
        cross: Option<hyperreal::RealSign>,
        circle_first: bool,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let mut circle_parameter = CurveParameter2::from_algebraic_cusp(circle_parameter);
        let (mut first, mut second) = if circle_first {
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
        // Endpoint equality is already a parameter-space theorem. Keep the
        // endpoint's original point and parameter authority when publishing a
        // new contact, so evaluation and later cuts replay the same witness.
        let mut point = point;
        for at_start in [true, false] {
            let endpoint =
                CurveParameter2::from_algebraic_cusp(circle.endpoint_parameter(at_start).clone());
            if self
                .policy
                .strict_predicate_pass(|| {
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
                if circle_first {
                    first = circle_parameter.clone();
                } else {
                    second = circle_parameter.clone();
                }
                point = Some(retained);
            }
            break;
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
        // Regularized regions omit a singleton overlap. Open curves keep its
        // endpoint contact, transported through the same original authority.
        let (circle_range, other_range) = source.parameter_ranges();
        let (circle_span, other_span) = if circle_first {
            (self.first, self.second)
        } else {
            (self.second, self.first)
        };
        // Both active domains are intervals in this one-to-one component.
        // With no positive overlap, any shared point is an endpoint of the
        // circle's clipped interval. Forward transport suffices, including
        // component endpoints that lie inside the active source range.
        for parameter in [
            circle_span.range.start(),
            circle_span.range.end(),
            circle_range.start(),
            circle_range.end(),
        ] {
            if !contains(
                &circle_range,
                parameter,
                circle_span.support.family(),
                self.policy,
            )? || !contains(
                &circle_span.range,
                parameter,
                circle_span.support.family(),
                self.policy,
            )? {
                continue;
            }
            let Some(mapped) = decided(
                source.map_parameter(parameter, true, self.policy),
                circle_span.support.family(),
            )?
            else {
                continue;
            };
            if !contains(
                &other_range,
                &mapped,
                other_span.support.family(),
                self.policy,
            )? || !contains(
                &other_span.range,
                &mapped,
                other_span.support.family(),
                self.policy,
            )? {
                continue;
            }
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
            return Ok(());
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
        self.require_unit_domain(if circle_first {
            self.second
        } else {
            self.first
        })?;
        let (intersections, map) = decided(
            circle
                .semicircle()
                .rational_intersections_with_parameter_map(rational, self.policy),
            crate::CurveFamily2::CircularArc,
        )?;
        match intersections {
            Intersections::Contacts(contacts) => {
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
            }
            Intersections::SelectedFiberContacts(contacts) => {
                self.circle_selected_contacts(circle, contacts, circle_first, result)?
            }
            Intersections::Overlaps(overlaps) => {
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Mapped(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::SelectedFiberOverlaps(overlaps) => {
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
        self.require_unit_domain(span)?;
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
            if component
                .curve()
                .exact_linear_parameterization_line()
                .is_none()
            {
                let mut candidate = Evidence::default();
                match (Pair {
                    first: self.first,
                    second: self.second,
                    indices: self.indices,
                    policy: &strict,
                })
                .circle_rational(
                    circle,
                    component.curve(),
                    circle_first,
                    &mut candidate,
                ) {
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
        }
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
            Intersections::Contacts(contacts) => {
                let map = if contacts
                    .iter()
                    .any(|c| c.location.endpoint_parameter().is_none())
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
                    let parameter = contact.location.endpoint_parameter().unwrap_or_else(|| {
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
            }
            Intersections::SelectedFiberContacts(contacts) => {
                self.circle_selected_contacts(circle, contacts, circle_first, result)?
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
            Intersections::Overlaps(overlaps) => {
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Mapped(source),
                        circle_first,
                        result,
                    )?;
                }
            }
            Intersections::SelectedFiberOverlaps(overlaps) => {
                for source in overlaps {
                    self.circle_overlap(
                        circle,
                        CurveCircleOverlap2::Selected(source),
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
        // The retained resultant context isolates in the unit source chart.
        // An exterior restriction must use a finite-domain replay before this
        // kernel can certify completeness; never mistake an omitted chart for
        // an empty geometric intersection.
        for span in [self.first, self.second] {
            self.require_unit_domain(span)?;
        }
        let context = RationalBezierIntersectionContext::try_new(first, second, self.policy)?;
        let evidence = context.try_contacts()?;
        let family = self.first.support.family();
        for contact in evidence.isolated_contacts() {
            let first_parameter = CurveParameter2::from(contact.first_parameter().clone());
            let second_parameter = CurveParameter2::from(contact.second_parameter().clone());
            if contains(&self.first.range, &first_parameter, family, self.policy)?
                && contains(
                    &self.second.range,
                    &second_parameter,
                    self.second.support.family(),
                    self.policy,
                )?
            {
                self.append_contact(
                    result,
                    self.contact(
                        first_parameter,
                        second_parameter,
                        contact.point().clone(),
                        contact.is_certified_transverse(),
                        contact.tangent_cross_sign(),
                    ),
                )?;
            }
        }
        if let Some(overlap) = evidence.overlap() {
            let correspondence = RationalCurveOverlap2::new(
                context.overlap_parameter_correspondence(overlap),
                overlap,
            );
            self.overlap_self_contacts(first, second, overlap, &correspondence, result)?;
            if let Some((first_range, second_range)) = decided(
                correspondence.clipped_ranges(&self.first.range, &self.second.range, self.policy),
                family,
            )? {
                let mut inclusion = [true, true];
                for (index, parameter) in [first_range.start(), first_range.end()]
                    .into_iter()
                    .enumerate()
                {
                    inclusion[index] = self.overlap_includes(overlap, parameter)?;
                }
                result.overlaps.push(self.overlap(
                    [first_range, second_range],
                    overlap.orientation(),
                    inclusion,
                    CurveOverlapCorrespondence2::Rational(correspondence),
                )?);
            } else {
                // A regularized filled region can discard a singleton overlap.
                // An open curve must retain the shared endpoint contact.
                self.overlap_endpoint_contacts(first, overlap, &correspondence, result)?;
            }
        }
        self.retain_blockers(&evidence, result);
        Ok(())
    }

    fn retain_blockers(
        &self,
        evidence: &RationalBezierIntersectionContacts2,
        result: &mut Evidence,
    ) {
        let kind = match evidence {
            RationalBezierIntersectionContacts2::Incomplete { candidates, .. } => {
                Some(CurveIntersectionPairBlockerKind2::IncompleteReplay {
                    candidates: candidates.clone(),
                })
            }
            RationalBezierIntersectionContacts2::DegenerateResultant => {
                Some(CurveIntersectionPairBlockerKind2::SharedComponent)
            }
            _ => None,
        };
        if let Some(kind) = kind {
            self.blocker(result, kind);
        }
    }

    fn overlap_self_contacts(
        &self,
        first: &RationalBezier2,
        second: &RationalBezier2,
        overlap: &crate::RationalBezierIntersectionOverlap2,
        correspondence: &RationalCurveOverlap2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        if first.has_certified_injective_axis(self.policy)
            && second.has_certified_injective_axis(self.policy)
        {
            return Ok(());
        }
        // A geometric overlap need not describe every pair of parameters at
        // a self-crossing. A full-domain projective correspondence lets the
        // existing diagonal-deflated self-contact authority recover all of
        // those fibers. A partial or non-bijective correspondence needs its
        // own complete component replay before domain clipping is complete.
        if !matches!(
            correspondence.source,
            RationalBezierOverlapParameterCorrespondence2::Identity
                | RationalBezierOverlapParameterCorrespondence2::UnitComplement
                | RationalBezierOverlapParameterCorrespondence2::EndpointProjective { .. }
        ) {
            self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent);
            return Ok(());
        }
        let evidence = self
            .first
            .self_contacts
            .get_or_init(|| first.self_intersection_contacts(self.policy))
            .as_ref()
            .map_err(Clone::clone)?;
        let family = self.first.support.family();
        for contact in evidence.isolated_contacts() {
            // Self contacts are unordered. Both ordered fibers can survive
            // different active domains on the two operands.
            for swapped in [false, true] {
                let (first_parameter, source_parameter) = if swapped {
                    (contact.second_parameter(), contact.first_parameter())
                } else {
                    (contact.first_parameter(), contact.second_parameter())
                };
                let first_parameter = CurveParameter2::from(first_parameter.clone());
                if !contains(&self.first.range, &first_parameter, family, self.policy)? {
                    continue;
                }
                let second_parameter = decided(
                    correspondence.source.map_first_to_second_region_parameter(
                        &CurveParameter2::from(source_parameter.clone()),
                        &correspondence.first_range,
                        &correspondence.second_range,
                        self.policy,
                    ),
                    family,
                )?
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        family,
                        UncertaintyReason::Boundary,
                    )
                })?;
                if !contains(
                    &self.second.range,
                    &second_parameter,
                    self.second.support.family(),
                    self.policy,
                )? {
                    continue;
                }
                let cross = contact.tangent_cross_sign().map(|sign| {
                    if swapped
                        != (overlap.orientation() == RationalBezierOverlapOrientation2::Reversed)
                    {
                        reverse_sign(sign)
                    } else {
                        sign
                    }
                });
                self.append_contact(
                    result,
                    self.contact(
                        first_parameter,
                        second_parameter,
                        contact.point().clone(),
                        contact.is_certified_transverse(),
                        cross,
                    ),
                )?;
            }
        }
        self.retain_blockers(evidence, result);
        if evidence.overlap().is_some() {
            self.blocker(result, CurveIntersectionPairBlockerKind2::SharedComponent);
        }
        Ok(())
    }

    fn overlap_includes(
        &self,
        overlap: &crate::RationalBezierIntersectionOverlap2,
        parameter: &CurveParameter2,
    ) -> ExactCurveResult<bool> {
        for (boundary, included) in [
            (overlap.first_range().start(), overlap.includes_start()),
            (overlap.first_range().end(), overlap.includes_end()),
        ] {
            if !included
                && decided(
                    parameter.same_value(&boundary.clone().into(), self.policy),
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
        overlap: &crate::RationalBezierIntersectionOverlap2,
        correspondence: &RationalCurveOverlap2,
        result: &mut Evidence,
    ) -> ExactCurveResult<()> {
        let first_overlap = CurveParameterRange2::from_bezier_range(overlap.first_range().clone());
        let second_overlap =
            CurveParameterRange2::from_bezier_range(overlap.second_range().clone());
        for forward in [true, false] {
            let (span, range, other) = if forward {
                (self.first, &first_overlap, self.second)
            } else {
                (self.second, &second_overlap, self.first)
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
                if !self.overlap_includes(overlap, &first_parameter)? {
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
) -> ExactCurveResult<CurveIntersectionResult2> {
    let first_spans = spans(first, policy)?;
    let second_spans = spans(second, policy)?;
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
            let outcome = match (&first.support, &second.support) {
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
                _ => Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    first.support.family(),
                    UncertaintyReason::Unsupported,
                )),
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
    fn exact<T: std::fmt::Debug>(value: Classification<T>) -> T {
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
    fn oriented(curve: &Curve2, reversed: bool, policy: &CurveContext) -> Curve2 {
        if reversed {
            let out = curve.reversed(policy).unwrap();
            assert_eq!(out.certainty, CurveCertainty::Certified);
            out.value
        } else {
            curve.clone()
        }
    }
    fn same(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
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
    fn query(first: &Curve2, second: &Curve2, policy: &CurveContext) -> CurveIntersectionResult2 {
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
