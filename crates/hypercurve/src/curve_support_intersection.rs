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
    let retained = |fragment: &BezierSplitFragment2, chart| Span {
        support: CurveSupport2::from_fragment(fragment),
        range: if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
            CurveParameterRange2::unit()
        } else {
            fragment.curve_region_parameter_range()
        },
        chart,
        reversed: fragment.source_is_reversed(),
        self_contacts: OnceLock::new(),
    };
    if let Some(fragment) = curve.retained_fragment() {
        return Ok(vec![retained(
            fragment,
            CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
        )]);
    }
    if let Some(spans) = curve.restricted_source_spans(policy, CurveOperation2::Intersection)? {
        return Ok(spans
            .iter()
            .map(|span| {
                retained(
                    &span.fragment,
                    CurveSpanRange2::from_affine_chart(&span.source_scale, &span.source_offset),
                )
            })
            .collect());
    }
    let native =
        curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let evaluators =
        curve.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    Ok(native
        .iter()
        .zip(evaluators)
        .map(|(fragment, evaluator)| Span {
            support: CurveSupport2::Bezier(BezierSubcurve2::Rational(evaluator.clone())),
            range: CurveParameterRange2::unit(),
            chart: fragment.span_range().clone(),
            reversed: false,
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
        let unit = CurveParameterRange2::unit();
        for parameter in [span.range.start(), span.range.end()] {
            if !contains(&unit, parameter, family, self.policy)? {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    family,
                    UncertaintyReason::Unsupported,
                ));
            }
        }
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
        let unit = CurveParameterRange2::unit();
        for span in [self.first, self.second] {
            for endpoint in [span.range.start(), span.range.end()] {
                if !contains(&unit, endpoint, span.support.family(), self.policy)? {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        span.support.family(),
                        UncertaintyReason::Unsupported,
                    ));
                }
            }
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
