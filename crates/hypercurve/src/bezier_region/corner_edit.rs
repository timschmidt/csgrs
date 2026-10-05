//! Retained corner cuts, cusp runs and corner fragment edits.

use super::*;

/// Proves that two cuts on one closed retained carrier bound a nonempty source
/// interval complementary to the edited seam neighborhood. The comparison is
/// made in the carrier's native parameter field and then reversed only when
/// traversal opposes that parameterization.
pub(super) fn retained_single_fragment_corner_cuts_are_separated(
    fragment: &BezierSplitFragment2,
    previous_cut: &CornerTrimCut2,
    next_cut: &CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let ordering = match next_cut
        .parameter
        .cmp_by_refinement(&previous_cut.parameter, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(ordering) => ordering,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            ));
        }
    };
    Ok(if fragment.source_is_reversed() {
        ordering == std::cmp::Ordering::Greater
    } else {
        ordering == std::cmp::Ordering::Less
    })
}

pub(super) enum RetainedCuspHalfRelation2 {
    Shared { complementary: bool },
    Overlap(crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2),
    EndpointContacts(Vec<crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2>),
}

pub(super) enum RetainedCuspRunCandidateCut2 {
    Outside,
    Owned {
        parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        placement: CornerPlacement2,
    },
    Uncertain(UncertaintyReason),
}

/// Proves that two selected half-circle charts lie on one supporting circle.
/// Shared storage is the allocation-free path. Independently reconstructed
/// frames reuse the authoritative circle-pair overlap publisher. Each probe is
/// STRICT-first; only an unresolved relation is retried as the operation's
/// terminal APPROXIMATE_512 decision.
pub(super) fn retained_cusp_half_relation(
    source: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    target: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<RetainedCuspHalfRelation2>> {
    let chart_relation =
        policy.strict_predicate_pass(|| source.shared_frame_chart_relation(target, policy));
    let chart_relation = if matches!(chart_relation, Classification::Uncertain(_))
        && policy.permits_approximate_512()
    {
        source.shared_frame_chart_relation(target, policy)
    } else {
        chart_relation
    };
    match chart_relation {
        Classification::Decided(Some(complementary)) => {
            return Ok(Some(RetainedCuspHalfRelation2::Shared { complementary }));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::CircularArc,
                reason,
            ));
        }
    }
    let relation = policy.strict_predicate_pass(|| source.pair_intersections(target, policy));
    let relation = match relation {
        Ok(Classification::Uncertain(_)) if policy.permits_approximate_512() => {
            source.pair_intersections(target, policy)
        }
        relation => relation,
    }
    .map_err(|cause| curve_region_edit_error(operation, cause))?;
    Ok(match relation {
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap),
        ) => Some(RetainedCuspHalfRelation2::Overlap(overlap)),
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(
                contacts,
            ),
        ) => Some(RetainedCuspHalfRelation2::EndpointContacts(contacts)),
        Classification::Decided(
            crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts
            | crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                ..
            },
        ) => None,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::CircularArc,
                reason,
            ));
        }
    })
}

pub(super) fn retained_cusp_run_candidate_cut(
    source_semicircle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    point: &crate::CurvePoint2,
    candidate: &crate::BezierAlgebraicCuspSemicircleFragment2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<RetainedCuspRunCandidateCut2> {
    let Some(relation) =
        retained_cusp_half_relation(source_semicircle, candidate.semicircle(), operation, policy)?
    else {
        return Ok(RetainedCuspRunCandidateCut2::Outside);
    };
    let candidate_parameter = match relation {
        RetainedCuspHalfRelation2::Shared {
            complementary: false,
        } => parameter.clone(),
        RetainedCuspHalfRelation2::Shared {
            complementary: true,
        } => {
            return Ok(RetainedCuspRunCandidateCut2::Outside);
        }
        RetainedCuspHalfRelation2::Overlap(overlap) => overlap.map_parameter(parameter, true),
        RetainedCuspHalfRelation2::EndpointContacts(contacts) => {
            use crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::{
                End, Interior, Start,
            };
            use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior;
            let full_source = crate::BezierAlgebraicCuspSemicircleFragment2::full(
                source_semicircle.clone(),
                policy,
            );
            // Endpoint-only coincident halves cannot own a point certified in
            // the strict interior of the source half. This avoids asking a
            // general point locator to rediscover a known non-endpoint.
            match full_source
                .contains_parameter(parameter, false, false, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(true) => {
                    return Ok(RetainedCuspRunCandidateCut2::Outside);
                }
                Classification::Decided(false) | Classification::Uncertain(_) => {}
            }
            let source_location = match full_source
                .certified_incident_point_evidence_location(parameter, point, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Start,
                ) => Start,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::End,
                ) => End,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Interior
                    | Exterior,
                ) => return Ok(RetainedCuspRunCandidateCut2::Outside),
                Classification::Uncertain(reason) => {
                    return Ok(RetainedCuspRunCandidateCut2::Uncertain(reason));
                }
            };
            let Some(target_location) = contacts.iter().find_map(|contact| {
                (contact.first_location == source_location).then_some(contact.second_location)
            }) else {
                return Err(ExactCurveError::invalid(
                    operation,
                    CurveFamily2::CircularArc,
                    CurveError::Topology(
                        "a coincident selected-circle endpoint lost its paired endpoint".into(),
                    ),
                ));
            };
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                match target_location {
                    Start => Real::zero(),
                    End => Real::one(),
                    Interior => {
                        return Err(ExactCurveError::invalid(
                            operation,
                            CurveFamily2::CircularArc,
                            CurveError::Topology(
                                "a coincident endpoint contact mapped to an interior parameter"
                                    .into(),
                            ),
                        ));
                    }
                },
            )
        }
    };
    use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
        End, Exterior, Interior, Start,
    };
    // The retained half relation certifies that the mapped parameter and
    // point lie on this candidate's supporting circle. Classify its authored
    // domain with the endpoint chord before constructing an independent dense
    // angular comparison; the latter is needed only if the structural point
    // evidence cannot decide.
    let incident_location = candidate
        .certified_incident_point_evidence_location(&candidate_parameter, point, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    let placement = match incident_location {
        Classification::Decided(Interior) => Some(CornerPlacement2::Trim),
        Classification::Decided(End) if previous => Some(CornerPlacement2::Corner),
        Classification::Decided(Start) if !previous => Some(CornerPlacement2::Corner),
        Classification::Decided(Start | End | Exterior) => None,
        Classification::Uncertain(location_reason) => match candidate
            .contains_parameter(&candidate_parameter, false, false, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(true) => Some(CornerPlacement2::Trim),
            Classification::Decided(false) => {
                let include_start = previous == candidate.is_reversed();
                match candidate
                    .contains_parameter(&candidate_parameter, include_start, !include_start, policy)
                    .map_err(|cause| curve_region_edit_error(operation, cause))?
                {
                    Classification::Decided(contains) => {
                        contains.then_some(CornerPlacement2::Corner)
                    }
                    Classification::Uncertain(_) => {
                        return Ok(RetainedCuspRunCandidateCut2::Uncertain(location_reason));
                    }
                }
            }
            Classification::Uncertain(_) => {
                return Ok(RetainedCuspRunCandidateCut2::Uncertain(location_reason));
            }
        },
    };
    Ok(
        placement.map_or(RetainedCuspRunCandidateCut2::Outside, |placement| {
            RetainedCuspRunCandidateCut2::Owned {
                parameter: candidate_parameter,
                placement,
            }
        }),
    )
}

pub(super) fn retained_cusp_smooth_run_neighbor(
    boundary: &CurveCornerChain2<'_>,
    index: usize,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    let fragments = boundary.fragments();
    if fragments.len() <= 1 {
        return Ok(None);
    }
    let Some(next_index) = boundary.neighbor(index, previous) else {
        return Ok(None);
    };
    let (
        BezierSplitFragment2::AlgebraicCuspSemicircle(source),
        BezierSplitFragment2::AlgebraicCuspSemicircle(next),
    ) = (&fragments[index], &fragments[next_index])
    else {
        return Ok(None);
    };
    source
        .validate_policy(policy)
        .and_then(|()| next.validate_policy(policy))
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    if (source.semicircle().is_clockwise() ^ source.is_reversed())
        != (next.semicircle().is_clockwise() ^ next.is_reversed())
    {
        return Ok(None);
    }
    let (source_start, next_start) = if previous {
        (true, false)
    } else {
        (false, true)
    };
    // A selected-radial fillet diameter and the concentric source parameter
    // that authored its frame are tangent but cannot belong to one supporting
    // circle: the former is centered at a nonzero-radius point of the latter.
    // Reject that structurally distinct adjacency before asking the general
    // circle-pair kernel to rediscover a high-multiplicity tangent contact.
    match source
        .endpoint_selected_radial_source_tangent_relation(source_start, next, next_start, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(_)) => return Ok(None),
        Classification::Decided(None) | Classification::Uncertain(_) => {}
    }
    let tangent_relation = policy.strict_predicate_pass(|| {
        exact_selected_circle_pair_tangent_cross_and_dot(
            source,
            source,
            source_start,
            next,
            next,
            next_start,
            policy,
        )
    });
    match tangent_relation {
        Classification::Decided((RealSign::Positive | RealSign::Negative, _))
        | Classification::Decided((RealSign::Zero, Some(RealSign::Negative))) => return Ok(None),
        Classification::Decided((RealSign::Zero, Some(RealSign::Zero))) => {
            return Err(ExactCurveError::invalid(
                operation,
                CurveFamily2::CircularArc,
                CurveError::Topology(
                    "nonzero selected-circle endpoint tangents had zero cross and dot".into(),
                ),
            ));
        }
        Classification::Decided((RealSign::Zero, Some(RealSign::Positive) | None))
        | Classification::Uncertain(_) => {}
    }
    retained_cusp_half_relation(source.semicircle(), next.semicircle(), operation, policy)
        .map(|relation| relation.map(|_| next_index))
}

/// Chooses the ancestral parameter frame for one incident smooth run. When a
/// seam endpoint was authored by a coincident-circle overlap map, solving in
/// its source chart preserves that exact evidence for later endpoint
/// ownership. The geometric carrier solve still runs only once.
pub(super) fn retained_cusp_smooth_run_authority(
    boundary: &CurveCornerChain2<'_>,
    index: usize,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    let Some(mut neighbor_index) =
        retained_cusp_smooth_run_neighbor(boundary, index, previous, operation, policy)?
    else {
        return Ok(None);
    };
    let mut authority_index = index;
    for _ in 1..boundary.fragments().len() {
        let (
            BezierSplitFragment2::AlgebraicCuspSemicircle(authority),
            BezierSplitFragment2::AlgebraicCuspSemicircle(neighbor),
        ) = (
            &boundary.fragments()[authority_index],
            &boundary.fragments()[neighbor_index],
        )
        else {
            unreachable!("a selected-circle smooth-run neighbor was already certified")
        };
        let source_is_authority = if previous {
            authority.endpoint_pair_overlap_source(true, neighbor, false)
        } else {
            authority.endpoint_pair_overlap_source(false, neighbor, true)
        };
        if source_is_authority != Some(false) {
            break;
        }
        authority_index = neighbor_index;
        let Some(next_index) = retained_cusp_smooth_run_neighbor(
            boundary,
            authority_index,
            previous,
            operation,
            policy,
        )?
        else {
            break;
        };
        if next_index == index {
            break;
        }
        neighbor_index = next_index;
    }
    Ok(Some(authority_index))
}

/// Rebinds one full-support selected-circle contact to the exact fragment in
/// the contiguous smooth run that owns it. This turns a representation-chart
/// extension into an authored trim without admitting a true curve extension.
pub(super) fn rebind_retained_cusp_run_cut(
    boundary: &CurveCornerChain2<'_>,
    immediate_index: usize,
    source_index: usize,
    previous: bool,
    search_trim: bool,
    cut: &mut CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<usize>> {
    if cut.placement != CornerPlacement2::Extension && !search_trim {
        return Ok(Some(immediate_index));
    }
    let BezierSplitFragment2::AlgebraicCuspSemicircle(source) = &boundary.fragments()[source_index]
    else {
        return Ok(None);
    };
    let Some(parameter) = cut.parameter.as_algebraic_cusp().cloned() else {
        return Ok(None);
    };
    let complementary = cut.parameter.is_algebraic_cusp_complement();
    let complementary_semicircle;
    let source_semicircle = if complementary {
        complementary_semicircle = source.semicircle().complementary_half();
        &complementary_semicircle
    } else {
        source.semicircle()
    };
    let mut index = immediate_index;
    if source_index == immediate_index && cut.placement == CornerPlacement2::Extension {
        let Some(next_index) = retained_cusp_smooth_run_neighbor(
            boundary,
            immediate_index,
            previous,
            operation,
            policy,
        )?
        else {
            return Ok(None);
        };
        if next_index == immediate_index {
            return Ok(None);
        }
        index = next_index;
    }
    let mut uncertainty = None;
    for step in 0..boundary.fragments().len() {
        let BezierSplitFragment2::AlgebraicCuspSemicircle(candidate) = &boundary.fragments()[index]
        else {
            unreachable!("the smooth selected-circle neighbor was checked")
        };
        match retained_cusp_run_candidate_cut(
            source_semicircle,
            &parameter,
            &cut.point,
            candidate,
            previous,
            operation,
            policy,
        )? {
            RetainedCuspRunCandidateCut2::Outside => {}
            RetainedCuspRunCandidateCut2::Owned {
                parameter,
                placement,
            } => {
                cut.parameter = CurveParameter2::from_algebraic_cusp(parameter);
                cut.placement = placement;
                return Ok(Some(index));
            }
            RetainedCuspRunCandidateCut2::Uncertain(reason) => {
                uncertainty.get_or_insert(reason);
            }
        }
        if step + 1 == boundary.fragments().len() {
            break;
        }
        let Some(next_index) =
            retained_cusp_smooth_run_neighbor(boundary, index, previous, operation, policy)?
        else {
            break;
        };
        if next_index == immediate_index {
            break;
        }
        index = next_index;
    }
    match uncertainty {
        Some(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::CircularArc,
            reason,
        )),
        None => Ok(None),
    }
}

pub(super) fn retained_corner_decision<T>(
    decision: Classification<T>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match decision {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

pub(super) fn retained_chord_on_certified_line(
    line: &LineSeg2,
    start: CurvePoint2,
    end: CurvePoint2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    let support = retained_algebraic_line_support(line, operation, policy)?;
    match support
        .chord_between_certified_support_points(start, end, policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(Some(chord)) => Ok(BezierSplitFragment2::AlgebraicChord(chord)),
        Classification::Decided(None) => Err(curve_region_edit_error(
            operation,
            CurveError::Topology("a retained line trim collapsed to one point".into()),
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Line,
            reason,
        )),
    }
}

pub(super) fn retained_algebraic_line_support(
    line: &LineSeg2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::BezierAlgebraicChord2> {
    match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        CurvePoint2::from(line.start().clone()),
        CurvePoint2::from(line.end().clone()),
        policy,
    )
    .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(support) => Ok(support),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Line,
            reason,
        )),
    }
}

pub(super) fn retained_selected_corner_parameter_is_in_native_chart(
    parameter: &CurveParameter2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if parameter.as_bezier_parameter().is_none() && !parameter.is_retained_scalar() {
        return Ok(false);
    }
    let compare = |boundary: CurveParameter2| {
        retained_corner_decision(
            policy
                .strict_predicate_pass(|| parameter.cmp_by_refinement(&boundary, policy))
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )
    };
    Ok(
        !compare(CurveParameter2::from(BezierParameter2::Exact(Real::zero())))?.is_lt()
            && !compare(CurveParameter2::from(BezierParameter2::Exact(Real::one())))?.is_gt(),
    )
}

/// Rational cuts can use native control-net subdivision. Other scalar cuts
/// retain their source relation instead of embedding selected fields in a new
/// control net, whether the operation keeps one endpoint or cuts both ends.
pub(super) fn corner_parameter_needs_retained_source(parameter: &CurveParameter2) -> bool {
    parameter.is_retained_scalar()
        || parameter.as_bezier_parameter().is_some_and(|parameter| {
            parameter
                .scalar()
                .is_none_or(|value| value.exact_rational_ref().is_none())
        })
}

pub(super) fn retained_corner_fragment_extension(
    fragment: &BezierSplitFragment2,
    parameter: CurveParameter2,
    cut_point: &CurvePoint2,
    replacement: Option<&BezierSplitFragment2>,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. })
        && let Some(chord_parameter) = parameter.as_algebraic_chord()
    {
        let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
        return retained_corner_fragment_extension(
            &promoted,
            parameter,
            cut_point,
            replacement,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if let Some(replacement) = replacement {
        let BezierSplitFragment2::Materialized { curve, .. } = replacement else {
            return Ok(vec![replacement.clone()]);
        };
        let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let replacement = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: curve.clone(),
        };
        let replacement_cut = if keep_before_cut {
            Real::one()
        } else {
            Real::zero()
        };
        if parameter.scalar() == Some(&replacement_cut) {
            return Ok(vec![replacement]);
        }
        return retained_corner_fragment_trim(
            &replacement,
            CurveParameter2::from(parameter),
            cut_point,
            None,
            keep_before_cut,
            operation,
            policy,
        )
        .map(|fragment| fragment.into_iter().collect());
    }
    if let BezierSplitFragment2::SelectedFiber(fragment) = fragment {
        if !retained_selected_corner_parameter_is_in_native_chart(&parameter, operation, policy)? {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        }
        let compare = |first: &CurveParameter2, second: &CurveParameter2| {
            retained_corner_decision(
                policy
                    .strict_predicate_pass(|| first.cmp_by_refinement(second, policy))
                    .map_err(|cause| curve_region_edit_error(operation, cause))?,
                operation,
            )
        };
        let keep_lower_parameter_range = keep_before_cut != fragment.is_reversed();
        let range = fragment.range();
        if (keep_lower_parameter_range && !compare(&parameter, range.end())?.is_gt())
            || (!keep_lower_parameter_range && !compare(&parameter, range.start())?.is_lt())
        {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "a selected-fiber extension cut did not extend its authored range".into(),
                ),
            ));
        }
        let (source_start_point, source_end_point) = if fragment.is_reversed() {
            (fragment.end_point(), fragment.start_point())
        } else {
            (fragment.start_point(), fragment.end_point())
        };
        let (range, start_point, end_point) = if keep_lower_parameter_range {
            (
                CurveParameterRange2::new_validated(range.start().clone(), parameter),
                source_start_point.clone(),
                cut_point.clone(),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, range.end().clone()),
                cut_point.clone(),
                source_end_point.clone(),
            )
        };
        let extended = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                fragment.source().clone(),
                range,
                start_point,
                end_point,
            ),
        );
        return if fragment.is_reversed() {
            extended
                .reversed()
                .map(|fragment| vec![fragment])
                .map_err(|cause| curve_region_edit_error(operation, cause))
        } else {
            Ok(vec![extended])
        };
    }
    if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
        return retained_cusp_fragment_extension(
            fragment,
            parameter,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
        let parameter = parameter.as_algebraic_chord().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            )
        })?;
        let start = chord.start_parameter();
        let end = chord.end_parameter();
        let (start, end) = if keep_before_cut {
            (&start, parameter)
        } else {
            (parameter, &end)
        };
        let retained = if parameter.chord() == chord {
            crate::BezierAlgebraicChord2::from_certified_ordered_parameter_range(
                chord, start, end, policy,
            )
        } else {
            crate::BezierAlgebraicChord2::from_ordered_parameter_range(chord, start, end, policy)
        };
        return retained
            .map(|chord| vec![BezierSplitFragment2::AlgebraicChord(chord)])
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }

    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(source),
        ..
    } = fragment
    else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    if source.retained_exact_line_image().is_none() {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::QuadraticBezier,
            UncertaintyReason::Unsupported,
        ));
    }
    let start = if keep_before_cut {
        CurvePoint2::from(source.start().clone())
    } else {
        cut_point.clone()
    };
    let end = if keep_before_cut {
        cut_point.clone()
    } else {
        CurvePoint2::from(source.end().clone())
    };
    if let (CurvePoint2(CurvePointData2::Exact(start)), CurvePoint2(CurvePointData2::Exact(end))) =
        (&start, &end)
    {
        let line = LineSeg2::try_new(start.clone(), end.clone())
            .map_err(|cause| curve_region_edit_error(operation, cause))?;
        return Ok(vec![BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line)),
        }]);
    }
    retained_chord_on_certified_line(
        source
            .retained_exact_line_image()
            .expect("the retained extension source is an exact line"),
        start,
        end,
        operation,
        policy,
    )
    .map(|fragment| vec![fragment])
}

pub(super) fn retained_cusp_fragment_extension(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    parameter: CurveParameter2,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
    fragment
        .validate_policy(policy)
        .map_err(|cause| curve_region_edit_error(operation, cause))?;
    let complementary_cut = parameter.is_algebraic_cusp_complement();
    let cut = parameter.as_algebraic_cusp().cloned().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )
    })?;
    let compare =
        |first: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
         second: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2| {
            first
                .cmp_by_refinement(second, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        operation,
                        CurveFamily2::RationalBezier,
                        reason,
                    )),
                })
        };
    let zero = crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero());
    let one = crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one());
    let start = fragment.start_parameter().clone();
    let end = fragment.end_parameter().clone();
    let cut_side = if complementary_cut {
        None
    } else {
        let start_order = compare(&cut, &start)?;
        let end_order = compare(&cut, &end)?;
        match (start_order, end_order) {
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less) => Some(false),
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater) => Some(true),
            _ => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-circle extension cut did not lie outside its authored span"
                            .into(),
                    ),
                ));
            }
        }
    };
    let base = fragment.semicircle().clone();
    let complement = base.complementary_half();
    let mut result = Vec::with_capacity(3);
    let mut push = |semicircle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
                    range_start: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                    range_end: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                    reversed: bool|
     -> ExactCurveResult<()> {
        match compare(&range_start, &range_end)? {
            std::cmp::Ordering::Less => result.push(BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::from_certified_range(
                    semicircle.clone(),
                    range_start,
                    range_end,
                    reversed,
                    policy,
                ),
            )),
            std::cmp::Ordering::Equal => {}
            std::cmp::Ordering::Greater => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-circle extension produced a descending stored range".into(),
                    ),
                ));
            }
        }
        Ok(())
    };

    match (fragment.is_reversed(), keep_before_cut, cut_side) {
        (false, true, Some(true)) => push(&base, start, cut, false)?,
        (false, true, Some(false)) => {
            push(&base, start, one.clone(), false)?;
            push(&complement, zero.clone(), one.clone(), false)?;
            push(&base, zero.clone(), cut, false)?;
        }
        (false, true, None) => {
            push(&base, start, one.clone(), false)?;
            push(&complement, zero.clone(), cut, false)?;
        }
        (false, false, Some(false)) => push(&base, cut, end, false)?,
        (false, false, Some(true)) => {
            push(&base, cut, one.clone(), false)?;
            push(&complement, zero.clone(), one.clone(), false)?;
            push(&base, zero.clone(), end, false)?;
        }
        (false, false, None) => {
            push(&complement, cut, one.clone(), false)?;
            push(&base, zero.clone(), end, false)?;
        }
        (true, true, Some(false)) => push(&base, cut, end, true)?,
        (true, true, Some(true)) => {
            push(&base, zero.clone(), end, true)?;
            push(&complement, zero.clone(), one.clone(), true)?;
            push(&base, cut, one.clone(), true)?;
        }
        (true, true, None) => {
            push(&base, zero.clone(), end, true)?;
            push(&complement, cut, one.clone(), true)?;
        }
        (true, false, Some(true)) => push(&base, start, cut, true)?,
        (true, false, Some(false)) => {
            push(&base, zero.clone(), cut, true)?;
            push(&complement, zero.clone(), one.clone(), true)?;
            push(&base, start, one.clone(), true)?;
        }
        (true, false, None) => {
            push(&complement, zero, cut, true)?;
            push(&base, start, one, true)?;
        }
    }
    if result.is_empty() {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology("a selected-circle extension collapsed to one point".into()),
        ));
    }
    Ok(result)
}

pub(super) fn retained_corner_fragment_trim(
    fragment: &BezierSplitFragment2,
    parameter: CurveParameter2,
    cut_point: &CurvePoint2,
    replacement_curve: Option<&BezierSubcurve2>,
    keep_before_cut: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<BezierSplitFragment2>> {
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. }) {
        if let Some(chord_parameter) = parameter.as_algebraic_chord() {
            let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
            return retained_corner_fragment_trim(
                &promoted,
                parameter,
                cut_point,
                replacement_curve,
                keep_before_cut,
                operation,
                policy,
            );
        }
        let promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
        return retained_corner_fragment_trim(
            &BezierSplitFragment2::AnalyticParallel(promoted),
            parameter,
            cut_point,
            replacement_curve,
            keep_before_cut,
            operation,
            policy,
        );
    }
    if !matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
        if matches!(
            fragment,
            BezierSplitFragment2::SelectedFiber(_) | BezierSplitFragment2::AnalyticParallel(_)
        ) && parameter.as_bezier_parameter().is_none()
            && !parameter.is_retained_scalar()
        {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        }
        let reversed = fragment.source_is_reversed();
        let keep_lower = keep_before_cut != reversed;
        let domain = fragment.curve_region_parameter_range();
        let retained_boundary = if keep_lower {
            domain.start()
        } else {
            domain.end()
        };
        // A cut at the outer endpoint consumes this whole incident span.
        // Compare source parameters: coincident geometric endpoints can also
        // enclose a nonempty trace on a nonlinear support.
        if retained_corner_decision(
            policy
                .strict_predicate_pass(|| parameter.same_value(retained_boundary, policy))
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )? {
            return Ok(None);
        }
        let source_points = match fragment {
            BezierSplitFragment2::SelectedFiber(selected) => Some(if reversed {
                [selected.end_point().clone(), selected.start_point().clone()]
            } else {
                [selected.start_point().clone(), selected.end_point().clone()]
            }),
            BezierSplitFragment2::AnalyticParallel(_) if parameter.is_retained_scalar() => {
                // The first common-scalar cut keeps the untouched endpoint as
                // a lazy source image. Later cuts reuse it from SelectedFiber,
                // so repeated restrictions do not build an endpoint chain.
                let source = Arc::new(fragment.clone());
                Some([
                    CurvePoint2::from_endpoint(Arc::clone(&source), !reversed),
                    CurvePoint2::from_endpoint(source, reversed),
                ])
            }
            _ => None,
        };
        let (range, points) = if keep_lower {
            (
                CurveParameterRange2::new_validated(domain.start().clone(), parameter),
                source_points.map(|[start, _]| [start, cut_point.clone()]),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, domain.end().clone()),
                source_points.map(|[_, end]| [cut_point.clone(), end]),
            )
        };
        // The corner solver proved order and finite-domain placement; the
        // check above excludes a fully consumed span before publication.
        return CurveSupport2::from_fragment(fragment)
            .restrict_certified(range, points, reversed, policy)
            .map(Some)
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }
    let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
        return Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        ));
    };
    let curve = replacement_curve.unwrap_or(curve);
    if let BezierSubcurve2::Quadratic(line) = curve
        && let Some(support) = line.retained_exact_line_image()
        && (parameter.as_algebraic_chord().is_some()
            || (matches!(
                parameter.as_bezier_parameter(),
                Some(BezierParameter2::Algebraic(_))
            ) && line.retained_parallel_line_tangent_contacts().is_empty()))
    {
        let (start, end) = if keep_before_cut {
            (CurvePoint2::from(line.start().clone()), cut_point.clone())
        } else {
            (cut_point.clone(), CurvePoint2::from(line.end().clone()))
        };
        // A promoted chord parameter need not use the materialized unit
        // chart. The certified line is injective, so its point identity
        // also certifies a consumed range.
        if retained_corner_decision(
            policy.strict_predicate_pass(|| start.same_point(&end, policy)),
            operation,
        )? {
            return Ok(None);
        }
        return retained_chord_on_certified_line(support, start, end, operation, policy).map(Some);
    }
    let retained_boundary = CurveParameter2::from(if keep_before_cut {
        Real::zero()
    } else {
        Real::one()
    });
    if retained_corner_decision(
        policy
            .strict_predicate_pass(|| parameter.same_value(&retained_boundary, policy))
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )? {
        return Ok(None);
    }
    // Keep nonrational cuts in their source chart. Rebuilding their control
    // points hides the selected parameter relation inside new coefficients,
    // forcing subsequent intersections to reconstruct the same extension.
    if corner_parameter_needs_retained_source(&parameter) {
        let zero = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
        let one = CurveParameter2::from(BezierParameter2::Exact(Real::one()));
        for (boundary, expected) in [
            (&zero, std::cmp::Ordering::Greater),
            (&one, std::cmp::Ordering::Less),
        ] {
            let ordering = match parameter
                .cmp_by_refinement(boundary, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?
            {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        operation,
                        CurveFamily2::RationalBezier,
                        reason,
                    ));
                }
            };
            if ordering != expected {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "a selected-fiber corner cut lay outside its open materialized range"
                            .into(),
                    ),
                ));
            }
        }
        let rational = RationalBezier2::try_from_subcurve(curve)
            .map_err(|cause| curve_region_edit_error(operation, cause))?;
        let (source_start, source_end) = curve.endpoint_refs();
        let (range, start_point, end_point) = if keep_before_cut {
            (
                CurveParameterRange2::new_validated(zero, parameter),
                CurvePoint2::from(source_start.clone()),
                cut_point.clone(),
            )
        } else {
            (
                CurveParameterRange2::new_validated(parameter, one),
                cut_point.clone(),
                CurvePoint2::from(source_end.clone()),
            )
        };
        return CurveSupport2::Bezier(BezierSubcurve2::Rational(rational))
            .restrict_certified(range, Some([start_point, end_point]), false, policy)
            .map(Some)
            .map_err(|cause| curve_region_edit_error(operation, cause));
    }
    let parameter = parameter.as_bezier_parameter().cloned().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            CurveFamily2::RationalBezier,
            UncertaintyReason::Unsupported,
        )
    })?;
    let split = match curve
        .split_at_parameters_refined(
            &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            &[parameter],
            policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))?
    {
        Classification::Decided(split) => split,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                operation,
                CurveFamily2::RationalBezier,
                reason,
            ));
        }
    };
    if split.fragments().len() != 2 {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "an interior retained corner cut did not produce two source fragments".into(),
            ),
        ));
    }
    let selected = if keep_before_cut {
        split.fragments()[0].clone()
    } else {
        split.fragments()[1].clone()
    };
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Rational(curve),
    } = selected
    else {
        return Ok(Some(selected));
    };
    Ok(Some(canonicalize_retained_corner_materialization(
        BezierSplitFragment2::Materialized {
            start,
            end,
            curve: BezierSubcurve2::Rational(curve),
        },
    )))
}

/// Publishes one cut in an existing circular cell cover. The parameter,
/// point, and optional cell endpoint are already certified by the caller.
/// Complementary cells extend the retained source; authored cells stand alone.
pub(super) fn retained_circular_cut_fragments(
    extension_source: Option<&[BezierSplitFragment2]>,
    spans: &[RationalBezier2],
    span_index: usize,
    parameter: &CurveParameter2,
    point: &CurvePoint2,
    endpoint: Option<BezierEndpoint>,
    keep_before_cut: bool,
) -> Vec<BezierSplitFragment2> {
    let span = &spans[span_index];
    let materialized_span = |index: usize| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Rational(spans[index].clone()),
    };
    let partial = match (keep_before_cut, endpoint) {
        (true, Some(BezierEndpoint::End)) | (false, Some(BezierEndpoint::Start)) => {
            Some(materialized_span(span_index))
        }
        (_, Some(_)) => None,
        (_, None) => {
            let (start, end, start_point, end_point) = if keep_before_cut {
                (
                    CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                    parameter.clone(),
                    CurvePoint2::from(span.start().clone()),
                    point.clone(),
                )
            } else {
                (
                    parameter.clone(),
                    CurveParameter2::from(BezierParameter2::Exact(Real::one())),
                    point.clone(),
                    CurvePoint2::from(span.end().clone()),
                )
            };
            Some(BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::Rational(span.clone()),
                    CurveParameterRange2::new_validated(start, end),
                    start_point,
                    end_point,
                ),
            ))
        }
    };
    let mut fragments = Vec::with_capacity(spans.len() + extension_source.map_or(0, <[_]>::len));
    if keep_before_cut {
        fragments.extend(extension_source.into_iter().flatten().cloned());
        fragments.extend((0..span_index).map(materialized_span));
        fragments.extend(partial);
    } else {
        fragments.extend(partial);
        fragments.extend((span_index + 1..spans.len()).map(materialized_span));
        fragments.extend(extension_source.into_iter().flatten().cloned());
    }
    fragments
}

pub(super) fn canonicalize_retained_corner_materialization(
    fragment: BezierSplitFragment2,
) -> BezierSplitFragment2 {
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Rational(curve),
    } = fragment
    else {
        return fragment;
    };
    BezierSplitFragment2::Materialized {
        start,
        end,
        curve: canonicalize_exact_rational_subcurve(
            BezierSubcurve2::Rational(curve),
            &CurveContext::STRICT,
        ),
    }
}

/// Retains the single source interval between two cuts on one closed carrier.
/// The next cut is the interval's traversal start and the previous cut is its
/// traversal end. Interior analytic cuts preserve their global parameter
/// authority. Exterior cuts retain one certified pole-free source interval,
/// so both point witnesses and every later query keep the same parameter chart.
pub(super) fn retained_corner_fragment_between_cuts(
    fragment: &BezierSplitFragment2,
    previous_cut: &CornerTrimCut2,
    next_cut: &CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierSplitFragment2> {
    match (
        previous_cut.replacement.as_deref(),
        next_cut.replacement.as_deref(),
    ) {
        (Some(previous), Some(next)) if previous != next => {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "one retained corner interval had incompatible replacement carriers".into(),
                ),
            ));
        }
        (Some(replacement), _) | (_, Some(replacement))
            if !matches!(replacement, BezierSplitFragment2::Materialized { .. }) =>
        {
            return Ok(replacement.clone());
        }
        _ => {}
    }
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. }) {
        if let Some(chord_parameter) = previous_cut
            .parameter
            .as_algebraic_chord()
            .or_else(|| next_cut.parameter.as_algebraic_chord())
        {
            let promoted = BezierSplitFragment2::AlgebraicChord(chord_parameter.chord().clone());
            return retained_corner_fragment_between_cuts(
                &promoted,
                previous_cut,
                next_cut,
                operation,
                policy,
            );
        }
        let promoted = promoted_endpoint_image_corner_fragment(fragment, operation)?;
        return retained_corner_fragment_between_cuts(
            &BezierSplitFragment2::AnalyticParallel(promoted),
            previous_cut,
            next_cut,
            operation,
            policy,
        );
    }
    if let BezierSplitFragment2::SelectedFiber(selected) = fragment
        && (previous_cut.placement == CornerPlacement2::Extension
            || next_cut.placement == CornerPlacement2::Extension)
    {
        if previous_cut.replacement.is_none()
            && next_cut.replacement.is_none()
            && retained_selected_corner_parameter_is_in_native_chart(
                &previous_cut.parameter,
                operation,
                policy,
            )?
            && retained_selected_corner_parameter_is_in_native_chart(
                &next_cut.parameter,
                operation,
                policy,
            )?
        {
            let (lower_parameter, upper_parameter, lower_point, upper_point) =
                if selected.is_reversed() {
                    (
                        previous_cut.parameter.clone(),
                        next_cut.parameter.clone(),
                        previous_cut.point.clone(),
                        next_cut.point.clone(),
                    )
                } else {
                    (
                        next_cut.parameter.clone(),
                        previous_cut.parameter.clone(),
                        next_cut.point.clone(),
                        previous_cut.point.clone(),
                    )
                };
            let order = retained_corner_decision(
                policy
                    .strict_predicate_pass(|| {
                        lower_parameter.cmp_by_refinement(&upper_parameter, policy)
                    })
                    .map_err(|cause| curve_region_edit_error(operation, cause))?,
                operation,
            )?;
            if order != std::cmp::Ordering::Less {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "selected extension cuts did not bound one traversal interval".into(),
                    ),
                ));
            }
            return CurveSupport2::from_fragment(fragment)
                .restrict_certified(
                    CurveParameterRange2::new_validated(lower_parameter, upper_parameter),
                    Some([lower_point, upper_point]),
                    selected.is_reversed(),
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause));
        }
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "a selected exterior interval reached reconstruction without its certified source range"
                    .into(),
            ),
        ));
    }
    if let BezierSplitFragment2::AnalyticParallel(fragment) = fragment
        && (previous_cut.placement == CornerPlacement2::Extension
            || next_cut.placement == CornerPlacement2::Extension)
    {
        let previous = previous_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let next = next_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        // In traversal terms the retained complement runs from the next cut
        // to the previous cut. The same oriented pair is ascending for a
        // forward carrier and descending for a reversed carrier; `try_new`
        // normalizes storage while retaining that traversal bit.
        let range = BezierParameterRange2::new_validated(next, previous);
        let rebuilt = retained_corner_decision(
            crate::BezierParallelFragment2::try_new(fragment.parallel().clone(), range, policy)
                .map_err(|cause| curve_region_edit_error(operation, cause))?,
            operation,
        )?;
        return Ok(BezierSplitFragment2::AnalyticParallel(rebuilt));
    }
    if let BezierSplitFragment2::Materialized { curve, .. } = fragment {
        let replacement = match (
            previous_cut.replacement_curve(),
            next_cut.replacement_curve(),
        ) {
            (Some(previous), Some(next)) if previous != next => {
                return Err(curve_region_edit_error(
                    operation,
                    CurveError::Topology(
                        "one retained corner interval had incompatible canonical sources".into(),
                    ),
                ));
            }
            (Some(curve), _) | (_, Some(curve)) => Some(curve.clone()),
            (None, None) => None,
        };
        let curve = replacement.as_ref().unwrap_or(curve);
        if [&next_cut.parameter, &previous_cut.parameter]
            .into_iter()
            .any(corner_parameter_needs_retained_source)
        {
            for parameter in [&next_cut.parameter, &previous_cut.parameter] {
                if !retained_selected_corner_parameter_is_in_native_chart(
                    parameter, operation, policy,
                )? {
                    return Err(curve_region_edit_error(
                        operation,
                        CurveError::Topology(
                            "a selected corner interval left its materialized source chart".into(),
                        ),
                    ));
                }
            }
            // The corner chain already certified separation in this chart.
            // Keep both selected point witnesses on the original support,
            // just as the one-sided cut publisher does.
            return CurveSupport2::Bezier(curve.clone())
                .restrict_certified(
                    CurveParameterRange2::new_validated(
                        next_cut.parameter.clone(),
                        previous_cut.parameter.clone(),
                    ),
                    Some([next_cut.point.clone(), previous_cut.point.clone()]),
                    false,
                    policy,
                )
                .map_err(|cause| curve_region_edit_error(operation, cause));
        }
        let previous_parameter = previous_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let next_parameter = next_cut
            .parameter
            .as_bezier_parameter()
            .cloned()
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    UncertaintyReason::Unsupported,
                )
            })?;
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        let next_order = match next_parameter
            .cmp_by_refinement_with_policy(&zero, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        let previous_order = match previous_parameter
            .cmp_by_refinement_with_policy(&one, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        if next_order == std::cmp::Ordering::Less || previous_order == std::cmp::Ordering::Greater {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "a canonical one-fragment corner interval left its replacement envelope".into(),
                ),
            ));
        }
        let mut parameters = Vec::with_capacity(2);
        let selected_index = if next_order == std::cmp::Ordering::Greater {
            parameters.push(next_parameter);
            1
        } else {
            0
        };
        if previous_order == std::cmp::Ordering::Less {
            parameters.push(previous_parameter);
        }
        let split = match curve
            .split_at_parameters_refined(
                &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                &parameters,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(operation, cause))?
        {
            Classification::Decided(split) => split,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::RationalBezier,
                    reason,
                ));
            }
        };
        if split.fragments().len() != parameters.len() + 1
            || selected_index >= split.fragments().len()
        {
            return Err(curve_region_edit_error(
                operation,
                CurveError::Topology(
                    "two separated retained corner cuts did not isolate one source interval".into(),
                ),
            ));
        }
        return Ok(canonicalize_retained_corner_materialization(
            split.fragments()[selected_index].clone(),
        ));
    }

    let reversed = fragment.source_is_reversed();
    let (lower, upper) = if reversed {
        (previous_cut, next_cut)
    } else {
        (next_cut, previous_cut)
    };
    let points = match fragment {
        BezierSplitFragment2::SelectedFiber(_) => Some([lower.point.clone(), upper.point.clone()]),
        BezierSplitFragment2::AnalyticParallel(_)
            if lower.parameter.is_retained_scalar() || upper.parameter.is_retained_scalar() =>
        {
            Some([lower.point.clone(), upper.point.clone()])
        }
        _ => None,
    };
    CurveSupport2::from_fragment(fragment)
        .restrict_certified(
            CurveParameterRange2::new_validated(lower.parameter.clone(), upper.parameter.clone()),
            points,
            reversed,
            policy,
        )
        .map_err(|cause| curve_region_edit_error(operation, cause))
}

/// Keeps both cuts, their point witnesses and their original support chart.
/// The finite envelope certifies source finiteness; it never becomes a new
/// geometric carrier or a restriction on the exact cut representation.
pub(super) fn retain_corner_extension_interval(
    fragment: &BezierSplitFragment2,
    previous_cut: &mut CornerTrimCut2,
    next_cut: &mut CornerTrimCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let reversed = fragment.source_is_reversed();
    let (lower, upper) = if reversed {
        (&*previous_cut, &*next_cut)
    } else {
        (&*next_cut, &*previous_cut)
    };
    let order = retained_corner_decision(
        policy
            .strict_predicate_pass(|| lower.parameter.cmp_by_refinement(&upper.parameter, policy))
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    if order != std::cmp::Ordering::Less {
        return Err(curve_region_edit_error(
            operation,
            CurveError::Topology(
                "retained extension cuts did not bound one traversal interval".into(),
            ),
        ));
    }
    let range =
        CurveParameterRange2::new_validated(lower.parameter.clone(), upper.parameter.clone());
    let support = CurveSupport2::from_fragment(fragment);
    let source = match &support {
        CurveSupport2::Bezier(curve) => RationalBezier2::try_from_subcurve(curve),
        CurveSupport2::Parallel(parallel) => parallel.source().to_rational_bezier(),
        CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
            return Err(ExactCurveError::blocked(
                operation,
                support.family(),
                UncertaintyReason::Unsupported,
            ));
        }
    }
    .map_err(|cause| curve_region_edit_error(operation, cause))?;
    retained_corner_decision(
        source
            .finite_discovery_envelope(&range, policy)
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
        operation,
    )?;
    let replacement = Arc::new(
        support
            .restrict_certified(
                range,
                Some([lower.point.clone(), upper.point.clone()]),
                reversed,
                policy,
            )
            .map_err(|cause| curve_region_edit_error(operation, cause))?,
    );
    previous_cut.replacement = Some(replacement.clone());
    next_cut.replacement = Some(replacement);
    Ok(())
}

pub(super) fn curve_region_boundary_loop_from_native_material_contour(
    contour: Contour2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegionBoundaryLoop2> {
    let mut boundaries =
        CurveRegion2::try_from_native_contours_raw(vec![contour], Vec::new(), policy)?
            .into_boundary_loops();
    if boundaries.len() != 1 {
        return Err(curve_region_edit_error(
            CurveOperation2::Offset,
            CurveError::Topology("a native offset cap did not produce one boundary loop".into()),
        ));
    }
    // Promotion-local arrangement indices start at zero for every cap. They
    // are not source-curve identities and cannot be combined across caps;
    // the unified band arrangement publishes its own global provenance.
    Ok(boundaries
        .pop()
        .expect("the native offset cap has exactly one boundary loop")
        .without_arrangement_sources())
}

pub(super) struct ExactOffsetSpan2 {
    pub(super) fragments: Vec<BezierSplitFragment2>,
    pub(super) source_end: CurvePoint2,
    pub(super) offset_start: CurvePoint2,
    pub(super) offset_end: CurvePoint2,
    pub(super) start_tangent: Option<CurveTangent2>,
    pub(super) end_tangent: Option<CurveTangent2>,
}
